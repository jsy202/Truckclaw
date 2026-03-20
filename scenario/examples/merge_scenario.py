"""
3-vehicle platooning merge scenario  –  CARLA Town06
=====================================================

  V1  Lead       : TM autopilot  (constant speed, no lane change)
  V2  Follower 1 : FollowerController + CACC
  V3  Merger     : TM autopilot throughout (SOLO→ALIGNING→LANE_CHANGE→JOINED)
                   speed controlled via tm.set_desired_speed()
                   lane change via tm.force_lane_change()
                   → FollowerController + CACC after JOINED

Run:
  PYTHONPATH=/home/user/carla_source/PythonAPI/carla \
      python3 examples/merge_scenario.py
"""

import sys, os, tty, termios, select
import carla
import numpy as np
from enum import Enum, auto

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from PlatooningSimulator import Core, PlatooningControllers

# ── constants ────────────────────────────────────────────────────────────────
DT             = 0.01      # CARLA physics timestep (100 Hz)
SAMPLING_RATE  = 10        # CACC update interval  (10 Hz)

LEAD_SPEED_KMH  = 60.0     # V1 cruise speed
SOLO_SPEED_KMH  = 58.0     # V3 solo speed – slightly slower than platoon so platoon overtakes

SOLO_SECS       = 25.0     # solo time before auto-aligning kicks in
SOLO_STEPS      = int(SOLO_SECS / (DT * SAMPLING_RATE))
ALIGN_TIMEOUT   = 600      # sampling ticks max in ALIGNING before forcing LC

TARGET_GAP_M    = 10.0     # longitudinal gap V3 must be behind V2 before lane change
STRAIGHT_M      = 200.0    # metres of straight road required ahead before merging
CONFIRM_TICKS   = 15       # sampling ticks confirmed in platoon lane → FOLLOWING
FOLLOW_DIST_M   = 13.0     # TM min-distance after merge ≈ CACC desired gap at 60 km/h
CATCHUP_KMH     = 20.0     # extra speed added when closing gap after merge

TOTAL_SECS     = 150.0
TOTAL_STEPS    = int(TOTAL_SECS / DT)

CAM_HEIGHT     = 80.0      # spectator height above V3 (m)
CAM_ALPHA      = 0.03      # position lerp per physics tick at 100 Hz → τ ≈ 0.33 s
CAM_EVERY      = 1         # update camera every physics tick (100 Hz, tiny nudge each time)

BLUEPRINT_PREFS = [
    'vehicle.carlamotors.european_hgv',
    'vehicle.mitsubishi.fusorosa',
    'vehicle.carlamotors.firetruck',
    'vehicle.mercedes.sprinter',
    'vehicle.bmw.grandtourer',
]


# ============================================================================
# Helpers
# ============================================================================

def pick_blueprint(bps):
    for name in BLUEPRINT_PREFS:
        found = bps.filter(name)
        if found:
            print(f"[bp] {found[0].id}")
            return found[0]
    raise RuntimeError("No vehicle blueprint found.")


def find_good_spawn(carla_map, spawn_points, clear_m=400.0):
    """Spawn on a highway section: no junction for clear_m ahead, adjacent lane exists."""
    for idx, sp in enumerate(spawn_points):
        wpt = carla_map.get_waypoint(sp.location)
        if wpt.lane_type != carla.LaneType.Driving or wpt.is_junction:
            continue
        # check clear road ahead
        cur, ok = wpt, True
        for _ in range(int(clear_m / 10)):
            nxt = cur.next(10.0)
            if not nxt or nxt[0].is_junction:
                ok = False; break
            cur = nxt[0]
        if not ok:
            continue
        # adjacent driving lane exists and has history
        adj = wpt.get_right_lane() or wpt.get_left_lane()
        if adj and adj.lane_type == carla.LaneType.Driving and adj.previous(50.0):
            print(f"[spawn] V1/V2  idx={idx}  road={wpt.road_id}  lane={wpt.lane_id}")
            return sp, wpt.lane_id
    raise RuntimeError("No suitable highway spawn found in Town06.")


def find_v3_spawn(carla_map, v1_spawn, ahead_m=8.0):
    """
    Spawn V3 slightly AHEAD of V1 in the adjacent lane.
    Ahead = guaranteed clear road, no wall risk.
    With SOLO_SPEED < LEAD_SPEED the platoon will naturally overtake V3.
    """
    wpt = carla_map.get_waypoint(v1_spawn.location)
    for adj in (wpt.get_right_lane(), wpt.get_left_lane()):
        if adj is None or adj.lane_type != carla.LaneType.Driving:
            continue
        nxt = adj.next(ahead_m)
        t   = nxt[0].transform if nxt else adj.transform
        sp  = carla.Transform(
            carla.Location(x=t.location.x, y=t.location.y, z=t.location.z + 0.3),
            t.rotation,
        )
        print(f"[spawn] V3     lane={adj.lane_id}  ({sp.location.x:.1f}, {sp.location.y:.1f})")
        return sp, adj.lane_id
    raise RuntimeError("No adjacent lane found for V3.")


# ============================================================================
# CACC
# ============================================================================
def v_ref_cacc(predecessor, ego):
    tau = 0.66; h = 0.5; c = 2.0; L = 5.0
    d  = ego.distance_to(predecessor)
    vp = predecessor.speed; ve = ego.speed
    return (tau/h * (vp - ve + c*(d - L - h*ve)) + ve) * 3.6


# ============================================================================
# Non-blocking keyboard input (Linux)
# ============================================================================
class KeyInput:
    """
    Non-blocking single-key reader for Linux.
    Usage:
        kb = KeyInput()
        ...
        k = kb.read()   # returns '' if no key pressed
        ...
        kb.restore()    # call on exit
    """

    def __init__(self):
        self._active = False
        try:
            self._fd  = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)   # non-blocking chars but keeps Ctrl+C as SIGINT
            self._active = True
            print("[keys] Press SPACE to trigger merge.  Ctrl-C to quit.")
        except termios.error:
            print("[keys] Not a TTY – keyboard disabled. Auto-merge via timeout.")

    def read(self):
        """Return pressed key as string, or '' if nothing / not active."""
        if not self._active:
            return ''
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == '\x03':   # Ctrl-C
                self.restore()
                raise KeyboardInterrupt
            return ch
        return ''

    def restore(self):
        if self._active:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


# ============================================================================
# MergeManager  –  TM-based state machine for V3
# ============================================================================
class MergeState(Enum):
    SOLO        = auto()   # driving solo, waiting for SPACE
    SLOWING     = auto()   # slowing down to open gap behind V2
    LANE_CHANGE = auto()   # executing lane change into platoon lane
    FOLLOWING   = auto()   # TM ACC following V2 in platoon lane


class MergeManager:
    """
    Controls V3's merge journey using the Traffic Manager API only.

    SOLO        : V3 drives at SOLO_SPEED_KMH; waiting for SPACE key.
    SLOWING     : V3 slows down until it is TARGET_GAP_M behind V2.
    LANE_CHANGE : force_lane_change into the exact platoon lane every ~1 s.
    FOLLOWING   : TM ACC follows V2 at FOLLOW_DIST_M gap (TM built-in).
    """

    def __init__(self, v3, platoon, tm, tm_port, plat_lane_id, carla_map):
        self.v3           = v3
        self.platoon      = platoon
        self.tm           = tm
        self.tm_port      = tm_port
        self.plat_lane_id = plat_lane_id
        self._map         = carla_map

        self.state          = MergeState.SOLO
        self._sticks        = 0
        self._lc_confirm    = 0
        self._lc_dir        = None
        self.merge_complete = False

    # ── helpers ───────────────────────────────────────────────────────────────

    def _tail(self):
        return self.platoon.follower_vehicles[-1]

    def _v3_speed(self):
        v = self.v3._carla_vehicle.get_velocity()
        return float(np.sqrt(v.x**2 + v.y**2 + v.z**2))

    def _behind_offset(self):
        """
        Signed longitudinal distance V3 is BEHIND V2 (positive = V3 is behind).
        Uses V2's forward vector as projection axis.
        """
        tail = self._tail()
        tl   = tail._carla_vehicle.get_location()
        el   = self.v3._carla_vehicle.get_location()
        yaw  = np.deg2rad(tail._carla_vehicle.get_transform().rotation.yaw)
        fwd  = np.array([np.cos(yaw), np.sin(yaw)])
        return float(np.dot(np.array([tl.x - el.x, tl.y - el.y]), fwd))

    def _is_straight(self, ahead_m=STRAIGHT_M):
        """True if V3's road ahead has no junction for ahead_m metres."""
        wpt = self._map.get_waypoint(self.v3._carla_vehicle.get_location())
        for _ in range(int(ahead_m / 10)):
            nxt = wpt.next(10.0)
            if not nxt:
                return False
            if nxt[0].is_junction:
                return False
            wpt = nxt[0]
        return True

    def _set_speed(self, kmh):
        self.tm.set_desired_speed(self.v3._carla_vehicle, float(max(kmh, 5.0)))

    def _resolve_lc_dir(self):
        """
        Determine which direction (True=left, False=right) leads toward V2's lane.
        Uses lateral offset of V2 relative to V3 — not lane_id comparison,
        which breaks across road segments with different road_id.
        """
        v2_loc = self._tail()._carla_vehicle.get_location()
        v3_loc = self.v3._carla_vehicle.get_location()
        v3_yaw = np.deg2rad(
            self._map.get_waypoint(v3_loc).transform.rotation.yaw)

        # V3's rightward unit vector (90° clockwise from forward)
        right_vec = np.array([np.sin(v3_yaw), -np.cos(v3_yaw)])

        # lateral component of (V2 − V3): positive = V2 is to V3's right
        diff    = np.array([v2_loc.x - v3_loc.x, v2_loc.y - v3_loc.y])
        lateral = float(np.dot(diff, right_vec))

        direction = lateral < 0   # True = left, False = right
        side = "LEFT" if direction else "RIGHT"
        print(f"    [LC dir] lateral={lateral:.1f}m → {side}")

        # sanity-check: confirm an adjacent driving lane actually exists that way
        wpt   = self._map.get_waypoint(v3_loc)
        check = wpt.get_left_lane() if direction else wpt.get_right_lane()
        if check is None or check.lane_type != carla.LaneType.Driving:
            # no driving lane that way — try opposite
            direction = not direction
            side = "LEFT" if direction else "RIGHT"
            print(f"    [LC dir] no driving lane, flipping to {side}")

        return direction

    # ── main update (called every sampling tick) ──────────────────────────────

    def update(self):
        self._sticks += 1
        t = self._sticks * DT * SAMPLING_RATE

        if self.state == MergeState.SOLO:
            # auto-trigger after SOLO_STEPS if no keyboard available
            if self._sticks >= SOLO_STEPS:
                print(f"  [V3 {t:.0f}s] SOLO timeout → auto try_merge")
                self.try_merge()

        elif self.state == MergeState.SLOWING:
            offset = self._behind_offset()
            v2_kmh = self._tail().speed * 3.6
            if offset < TARGET_GAP_M:
                # not far enough behind V2 yet → slow down
                self._set_speed(max(v2_kmh - 12.0, 5.0))
                if self._sticks % 20 == 0:
                    print(f"    [SLOW] gap={offset:.1f}m  need={TARGET_GAP_M}m  "
                          f"v3={self._v3_speed()*3.6:.1f} km/h")
            else:
                # gap achieved → begin lane change
                print(f"  [V3 {t:.0f}s] SLOWING → LANE_CHANGE  gap={offset:.1f}m")
                self._set_speed(v2_kmh)          # match V2 speed for smooth LC
                self._lc_dir = self._resolve_lc_dir()
                self.state   = MergeState.LANE_CHANGE

        elif self.state == MergeState.LANE_CHANGE:
            v2 = self._tail()
            # +8 km/h over V2 to compensate TM's speed reduction during lane change
            self._set_speed(v2.speed * 3.6 + 8.0)

            # re-trigger force_lane_change every ~1 s
            if self._lc_dir is not None and self._sticks % 10 == 0:
                self.tm.force_lane_change(self.v3._carla_vehicle, self._lc_dir)

            # confirm by comparing to V2's CURRENT road_id + lane_id (not stale plat_lane_id)
            v2_wpt = self._map.get_waypoint(v2._carla_vehicle.get_location())
            v3_wpt = self._map.get_waypoint(self.v3._carla_vehicle.get_location())
            in_platoon_lane = (v3_wpt.road_id == v2_wpt.road_id and
                               v3_wpt.lane_id  == v2_wpt.lane_id)

            if in_platoon_lane:
                self._lc_confirm += 1
                if self._lc_confirm >= CONFIRM_TICKS:
                    gap = float(self.v3._carla_vehicle.get_location().distance(
                                v2._carla_vehicle.get_location()))
                    print(f"  [V3 {t:.0f}s] LANE_CHANGE → FOLLOWING  gap={gap:.1f}m")
                    self.tm.distance_to_leading_vehicle(
                        self.v3._carla_vehicle, FOLLOW_DIST_M)
                    self._set_speed(v2.speed * 3.6 + CATCHUP_KMH)
                    self.state          = MergeState.FOLLOWING
                    self.merge_complete = True
            else:
                self._lc_confirm = 0

            if self._sticks % 20 == 0:
                print(f"    [LC] v3_lane={v3_wpt.lane_id} v2_lane={v2_wpt.lane_id} "
                      f"road_match={v3_wpt.road_id==v2_wpt.road_id} "
                      f"confirm={self._lc_confirm}/{CONFIRM_TICKS} "
                      f"dir={'L' if self._lc_dir else 'R'}")

        elif self.state == MergeState.FOLLOWING:
            v2     = self._tail()
            # use 3D distance (reliable in same lane, no projection error)
            gap    = float(self.v3._carla_vehicle.get_location().distance(
                           v2._carla_vehicle.get_location()))
            v2_kmh = v2.speed * 3.6

            if gap > FOLLOW_DIST_M + 2.0:
                # too far back → catch up
                self._set_speed(v2_kmh + CATCHUP_KMH)
            else:
                # close enough → match V2, TM min-distance prevents tailgating
                self._set_speed(v2_kmh)

            if self._sticks % 10 == 0:
                mode = 'CATCHUP' if gap > FOLLOW_DIST_M + 2 else 'hold'
                print(f"    [FOLLOW] gap={gap:.1f}m  v3={self._v3_speed()*3.6:.1f}  "
                      f"v2={v2_kmh:.1f} km/h  {mode}")

    # ── keyboard trigger ──────────────────────────────────────────────────────

    def try_merge(self):
        """
        Called when SPACE is pressed.
        1. Check straight road ahead.
        2. If gap already ≥ TARGET_GAP_M → go straight to LANE_CHANGE.
        3. Otherwise → SLOWING until gap is enough.
        """
        if self.state not in (MergeState.SOLO, MergeState.SLOWING):
            return

        t = self._sticks * DT * SAMPLING_RATE

        if not self._is_straight():
            print(f"  [V3 {t:.0f}s] SPACE: road not straight – wait for straight section")
            return

        offset = self._behind_offset()
        print(f"  [V3 {t:.0f}s] SPACE: straight OK  gap={offset:.1f}m")

        if offset >= TARGET_GAP_M:
            print(f"  [V3] gap sufficient → LANE_CHANGE")
            self._lc_dir = self._resolve_lc_dir()
            self._set_speed(self._tail().speed * 3.6)
            self.state = MergeState.LANE_CHANGE
        else:
            print(f"  [V3] gap too small ({offset:.1f}m < {TARGET_GAP_M}m) → SLOWING")
            self.state = MergeState.SLOWING


# ============================================================================
# Smooth camera
# ============================================================================
class SmoothCamera:
    """
    Smooth top-down camera following target_actor.

    Design choices that eliminate shaking:
    - x/y: exponential moving average at 10 Hz (sub-sampling kills physics jitter)
    - z  : fixed at first-seen ground height + CAM_HEIGHT (no vertical bounce)
    - yaw: fixed at 0  (vehicle yaw oscillates → tracking it causes rotation shake)
    """

    def __init__(self, spectator, height=CAM_HEIGHT, alpha=CAM_ALPHA):
        self._spec  = spectator
        self._h     = height
        self._alpha = alpha
        self._x = self._y = self._z = None

    def update(self, target_actor):
        try:
            loc = target_actor.get_location()
            if self._x is None:
                self._x = loc.x
                self._y = loc.y
                self._z = loc.z + self._h   # fixed forever

            self._x += self._alpha * (loc.x - self._x)
            self._y += self._alpha * (loc.y - self._y)

            self._spec.set_transform(carla.Transform(
                carla.Location(x=self._x, y=self._y, z=self._z),
                carla.Rotation(pitch=-90, yaw=0, roll=0),  # fixed: no rotation shake
            ))
        except Exception:
            pass


# ============================================================================
# Main
# ============================================================================
def main():
    print("\n=== Platooning Merge Scenario (Town06) ===\n")

    sim  = Core.Simulation(world="Town06", dt=DT,
                           large_map=False, render=True, synchronous=True)
    cmap = sim.map
    bps  = sim.get_vehicle_blueprints()
    bp   = pick_blueprint(bps)
    sps  = cmap.get_spawn_points()
    print(f"{len(sps)} spawn points\n")

    # ── spawn selection ──────────────────────────────────────────────────────
    v1_sp, plat_lane = find_good_spawn(cmap, sps)
    v3_sp, v3_lane   = find_v3_spawn(cmap, v1_sp)

    # ── Traffic Manager setup ────────────────────────────────────────────────
    tm      = sim.get_trafficmanager()
    tm.set_synchronous_mode(True)
    tm_port = tm.get_port()

    # ── V1: lead (TM autopilot) ──────────────────────────────────────────────
    platoon = Core.Platoon(sim)
    v1      = platoon.add_lead_vehicle(bp, v1_sp)
    sim.tick()

    tm.auto_lane_change(v1._carla_vehicle, False)
    tm.ignore_lights_percentage(v1._carla_vehicle, 100)
    tm.ignore_signs_percentage(v1._carla_vehicle, 100)
    tm.set_desired_speed(v1._carla_vehicle, LEAD_SPEED_KMH)
    v1.set_autopilot(True, tm_port)
    print(f"V1 TM autopilot ON  @ {LEAD_SPEED_KMH} km/h")

    # ── V2: follower (CACC) ──────────────────────────────────────────────────
    v2_sp  = v1.transform_ahead(-16.0, force_straight=True)
    v2     = platoon.add_follower_vehicle(bp, v2_sp)
    v2_ctrl = PlatooningControllers.FollowerController(
                  v2, v_ref_cacc, platoon, dependencies=[-1, 0])
    v2.attach_controller(v2_ctrl)
    sim.tick()

    # ── V3: solo merger (TM autopilot until JOINED) ──────────────────────────
    v3  = Core.Vehicle(bp, v3_sp, sim.world, index=0)
    sim.tick()

    tm.auto_lane_change(v3._carla_vehicle, False)
    tm.ignore_lights_percentage(v3._carla_vehicle, 100)
    tm.ignore_signs_percentage(v3._carla_vehicle, 100)
    tm.set_desired_speed(v3._carla_vehicle, SOLO_SPEED_KMH)
    v3._carla_vehicle.set_autopilot(True, tm_port)
    print(f"V3 TM autopilot ON  @ {SOLO_SPEED_KMH} km/h")

    mgr = MergeManager(v3, platoon, tm, tm_port, plat_lane, cmap)
    cam = SmoothCamera(sim.spectator)
    kb  = KeyInput()

    print(f"\n  V1  lane={plat_lane}  @ ({v1_sp.location.x:.1f}, {v1_sp.location.y:.1f})")
    print(f"  V2  lane={plat_lane}  @ ({v2_sp.location.x:.1f}, {v2_sp.location.y:.1f})")
    print(f"  V3  lane={v3_lane}   @ ({v3_sp.location.x:.1f}, {v3_sp.location.y:.1f})")
    print(f"\n  Solo: {SOLO_SECS:.0f}s  Target gap: {TARGET_GAP_M}m\n")

    # ── logging ───────────────────────────────────────────────────────────────
    log_spd    = np.zeros((3, TOTAL_STEPS))
    log_gap_12 = np.zeros(TOTAL_STEPS)
    log_gap_23 = np.zeros(TOTAL_STEPS)
    log_state  = np.zeros(TOTAL_STEPS, dtype=int)
    state_int  = {MergeState.SOLO:0, MergeState.SLOWING:1,
                  MergeState.LANE_CHANGE:2, MergeState.FOLLOWING:3}
    v3_joined  = False

    # ── simulation loop ───────────────────────────────────────────────────────
    try:
        for step in range(TOTAL_STEPS):

            is_sample = (step % SAMPLING_RATE == 0)

            # keyboard: SPACE → check straight, then SLOWING or LANE_CHANGE
            if is_sample and not v3_joined:
                key = kb.read()
                if key == ' ':
                    mgr.try_merge()

            # V3 merge state machine (sampling cadence)
            if is_sample and not v3_joined:
                mgr.update()

            # On merge complete: disable TM, switch V3 to CACC
            if not v3_joined and mgr.merge_complete:
                v3._carla_vehicle.set_autopilot(False, tm_port)
                platoon.follower_vehicles.append(v3)
                platoon.reindex()
                v3_cacc = PlatooningControllers.FollowerController(
                    v3, v_ref_cacc, platoon, dependencies=[-1, 0])
                v3_cacc.target_speed = v3.speed * 3.6
                v3.attach_controller(v3_cacc)
                v3_joined = True
                print(f"\n>>> V3 JOINED (CACC)  t={step*DT:.1f}s"
                      f"  V2–V3 gap={v2.distance_to(v3):.1f}m\n")

            # Platoon step (V1 auto-skipped, handles V2 + V3 after join)
            sim.run_step(mode="sample" if is_sample else "control")

            # Smooth camera (every CAM_EVERY ticks)
            if step % CAM_EVERY == 0:
                cam.update(v3)

            # Logging
            log_spd[0, step]  = v1.speed
            log_spd[1, step]  = v2.speed
            log_spd[2, step]  = v3.speed
            log_gap_12[step]  = v1.distance_to(v2)
            log_gap_23[step]  = v2.distance_to(v3)
            log_state[step]   = state_int.get(mgr.state, 3)

            sim.tick()

            if step % 500 == 0:
                print(f"  t={step*DT:6.1f}s"
                      f"  V1={v1.speed*3.6:5.1f}"
                      f"  V2={v2.speed*3.6:5.1f}"
                      f"  V3={v3.speed*3.6:5.1f} km/h"
                      f"  gap12={log_gap_12[step]:5.1f}m"
                      f"  gap23={log_gap_23[step]:5.1f}m"
                      f"  {mgr.state.name}")

    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        kb.restore()

    sim.release_synchronous()
    print("\n=== Done ===")

    # ── plots ─────────────────────────────────────────────────────────────────
    try:
        from matplotlib import pyplot as plt
        t   = np.arange(TOTAL_STEPS) * DT
        col = ['tab:blue', 'tab:orange', 'tab:green']
        lbl = ['V1 lead', 'V2 follower', 'V3 merger']

        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
        fig.suptitle("Platooning Merge  –  Town06", fontsize=13)

        for i in range(3):
            axes[0].plot(t, log_spd[i]*3.6, label=lbl[i], color=col[i])
        axes[0].set_ylabel("Speed (km/h)"); axes[0].legend(); axes[0].grid(alpha=.3)

        axes[1].plot(t, log_gap_12, label="V1–V2", color=col[1])
        axes[1].plot(t, log_gap_23, label="V2–V3", color=col[2])
        axes[1].axhline(TARGET_GAP_M, ls='--', color='gray', alpha=.6,
                        label=f"target {TARGET_GAP_M}m")
        axes[1].set_ylabel("Gap (m)"); axes[1].legend(); axes[1].grid(alpha=.3)

        axes[2].step(t, log_state, color='tab:red', where='post', lw=1.5)
        axes[2].set_yticks([0,1,2,3])
        axes[2].set_yticklabels(['SOLO','SLOWING','LC','FOLLOWING'])
        axes[2].set_ylabel("V3 state"); axes[2].set_xlabel("Time (s)")
        axes[2].grid(alpha=.3)

        plt.tight_layout()
        out = os.path.join(os.path.dirname(__file__), 'merge_result.png')
        plt.savefig(out, dpi=150)
        print(f"Plot → {out}")
        plt.show()
    except ImportError:
        pass


if __name__ == '__main__':
    main()
