#!/usr/bin/env python3
"""
Platoon Bridge Server  –  port 18801

REST API used by platoon_bridge_ctl.py (inside Docker agents) to negotiate
vehicle transfers between platoon A and platoon B.

State is kept in memory; restart resets to the initial scenario.

Endpoints
---------
GET  /health
GET  /snapshot
GET  /platoons/{platoon_id}
GET  /platoons/{platoon_id}/transfer-candidates
GET  /transfers/{request_id}
POST /transfers
POST /transfers/{request_id}/accept
POST /transfers/{request_id}/reject
POST /transfers/{request_id}/commit
"""

import json
import re
import threading
import urllib.request
import uuid
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, HTTPServer

CARLA_TRIGGER_URL = "http://127.0.0.1:18802/start_merge"


def _notify_carla_trigger():
    """Fire-and-forget POST to CARLA scenario trigger server."""
    def _do():
        try:
            req = urllib.request.Request(
                CARLA_TRIGGER_URL, data=b"{}", method="POST",
                headers={"Content-Type": "application/json"},
            )
            urllib.request.urlopen(req, timeout=2)
            print("[bridge] CARLA trigger sent -> /start_merge")
        except Exception as e:
            print(f"[bridge] CARLA trigger failed (scenario running?): {e}")
    threading.Thread(target=_do, daemon=True).start()

# ── initial scenario state ────────────────────────────────────────────────────

INITIAL_STATE = {
    "platoon_a": {
        "platoon_id": "platoon_a",
        "destination_id": "dest_a",   # Highway Main Lane (x=-140, y=210)
        "status": "cruising",
        "members": [
            {"vehicle_id": "platoon_a_truck0", "role": "leader",   "destination_id": "dest_a"},
            {"vehicle_id": "platoon_a_truck1", "role": "follower", "destination_id": "dest_a"},
            {"vehicle_id": "platoon_a_truck2", "role": "follower", "destination_id": "dest_b"},
        ],
    },
    "platoon_b": {
        "platoon_id": "platoon_b",
        "destination_id": "dest_b",   # Exit Ramp (x=-60, y=140)
        "status": "cruising",
        "members": [
            {"vehicle_id": "platoon_b_truck0", "role": "leader",   "destination_id": "dest_b"},
            {"vehicle_id": "platoon_b_truck1", "role": "follower", "destination_id": "dest_b"},
            {"vehicle_id": "platoon_b_truck2", "role": "follower", "destination_id": "dest_b"},
        ],
    },
}

# ── in-memory state ───────────────────────────────────────────────────────────

import copy

_lock     = threading.Lock()
_platoons = copy.deepcopy(INITIAL_STATE)
_transfers: dict = {}


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _find_vehicle(vehicle_id: str):
    """Return (platoon_dict, member_dict) or (None, None)."""
    for p in _platoons.values():
        for m in p["members"]:
            if m["vehicle_id"] == vehicle_id:
                return p, m
    return None, None


def _active_transfer_for(platoon_id: str):
    """Return the single active (pending/accepted) transfer id for a platoon, or None."""
    for tid, t in _transfers.items():
        if t["status"] in ("pending", "accepted"):
            if t["from_platoon_id"] == platoon_id or t["to_platoon_id"] == platoon_id:
                return tid
    return None


def _transfer_candidates(platoon_id: str) -> list:
    p = _platoons.get(platoon_id)
    if p is None:
        return []
    platoon_dest = p["destination_id"]
    candidates = []
    for m in p["members"]:
        if m["role"] == "leader":
            continue
        if m["destination_id"] == platoon_dest:
            continue
        # find a peer platoon whose destination matches this vehicle's destination
        for peer_id, peer in _platoons.items():
            if peer_id == platoon_id:
                continue
            if peer["destination_id"] == m["destination_id"]:
                candidates.append({
                    "vehicle_id": m["vehicle_id"],
                    "reason": "destination_match",
                    "target_platoon_id": peer_id,
                    "target_position": "tail",
                })
    return candidates


# ── HTTP handler ──────────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):  # noqa: N802 – overriding stdlib
        print(f"[bridge] {self.address_string()} {fmt % args}")

    # ── helpers ───────────────────────────────────────────────────────────────

    def _send(self, code: int, body: dict):
        data = json.dumps(body, ensure_ascii=False, indent=2).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _ok(self, body: dict):
        self._send(200, body)

    def _err(self, code: int, msg: str):
        self._send(code, {"error": msg})

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode())

    # ── routing ───────────────────────────────────────────────────────────────

    def do_GET(self):  # noqa: N802
        p = self.path.split("?")[0].rstrip("/")

        if p == "/health":
            self._ok({"ok": True, "status": "live"})

        elif p == "/snapshot":
            with _lock:
                self._ok({
                    "platoons": _platoons,
                    "transfers": _transfers,
                    "timestamp": _now(),
                })

        elif re.fullmatch(r"/platoons/[^/]+", p):
            platoon_id = p.split("/")[2]
            with _lock:
                pl = _platoons.get(platoon_id)
                if pl is None:
                    return self._err(404, f"platoon '{platoon_id}' not found")
                active = _active_transfer_for(platoon_id)
                self._ok({**pl, "active_transfer": active})

        elif re.fullmatch(r"/platoons/[^/]+/transfer-candidates", p):
            platoon_id = p.split("/")[2]
            with _lock:
                if platoon_id not in _platoons:
                    return self._err(404, f"platoon '{platoon_id}' not found")
                self._ok({
                    "platoon_id": platoon_id,
                    "candidates": _transfer_candidates(platoon_id),
                })

        elif re.fullmatch(r"/transfers/[^/]+", p):
            request_id = p.split("/")[2]
            with _lock:
                t = _transfers.get(request_id)
                if t is None:
                    return self._err(404, f"transfer '{request_id}' not found")
                self._ok(t)

        else:
            self._err(404, "not found")

    def do_POST(self):  # noqa: N802
        p = self.path.split("?")[0].rstrip("/")

        # POST /transfers  – create request
        if p == "/transfers":
            body = self._read_body()
            required = ("vehicle_id", "from_platoon_id", "to_platoon_id")
            missing = [k for k in required if not body.get(k)]
            if missing:
                return self._err(400, f"missing fields: {missing}")

            vehicle_id    = body["vehicle_id"]
            from_platoon  = body["from_platoon_id"]
            to_platoon    = body["to_platoon_id"]
            reason        = body.get("reason", "destination_match")
            sender_agent  = body.get("sender_agent", from_platoon)
            receiver_agent = body.get("receiver_agent", to_platoon)

            with _lock:
                if from_platoon not in _platoons:
                    return self._err(404, f"platoon '{from_platoon}' not found")
                if to_platoon not in _platoons:
                    return self._err(404, f"platoon '{to_platoon}' not found")

                src_platoon, member = _find_vehicle(vehicle_id)
                if member is None:
                    return self._err(404, f"vehicle '{vehicle_id}' not found")
                if src_platoon["platoon_id"] != from_platoon:
                    return self._err(400, f"vehicle '{vehicle_id}' is not in '{from_platoon}'")
                if member["role"] == "leader":
                    return self._err(400, "cannot transfer the leader vehicle")

                active = _active_transfer_for(from_platoon) or _active_transfer_for(to_platoon)
                if active:
                    return self._err(409, f"transfer '{active}' already active")

                request_id = "tr_" + uuid.uuid4().hex[:8]
                t = {
                    "request_id":    request_id,
                    "vehicle_id":    vehicle_id,
                    "from_platoon_id": from_platoon,
                    "to_platoon_id": to_platoon,
                    "status":        "pending",
                    "reason":        reason,
                    "sender_agent":  sender_agent,
                    "receiver_agent": receiver_agent,
                    "created_at":    _now(),
                    "updated_at":    _now(),
                }
                _transfers[request_id] = t
                print(f"[bridge] TRANSFER REQUEST  {request_id}  {vehicle_id}  {from_platoon} → {to_platoon}")
                self._ok(t)

        # POST /transfers/{id}/accept
        elif re.fullmatch(r"/transfers/[^/]+/accept", p):
            request_id = p.split("/")[2]
            body = self._read_body()
            with _lock:
                t = _transfers.get(request_id)
                if t is None:
                    return self._err(404, f"transfer '{request_id}' not found")
                if t["status"] != "pending":
                    return self._err(409, f"transfer is '{t['status']}', expected 'pending'")
                t["status"] = "accepted"
                t["accept_reason"] = body.get("reason", "accepted")
                t["updated_at"] = _now()
                print(f"[bridge] ACCEPTED  {request_id}")
                self._ok(t)

        # POST /transfers/{id}/reject
        elif re.fullmatch(r"/transfers/[^/]+/reject", p):
            request_id = p.split("/")[2]
            body = self._read_body()
            with _lock:
                t = _transfers.get(request_id)
                if t is None:
                    return self._err(404, f"transfer '{request_id}' not found")
                if t["status"] != "pending":
                    return self._err(409, f"transfer is '{t['status']}', expected 'pending'")
                t["status"] = "rejected"
                t["reject_reason"] = body.get("reason", "rejected")
                t["updated_at"] = _now()
                print(f"[bridge] REJECTED  {request_id}")
                self._ok(t)

        # POST /transfers/{id}/commit
        elif re.fullmatch(r"/transfers/[^/]+/commit", p):
            request_id = p.split("/")[2]
            with _lock:
                t = _transfers.get(request_id)
                if t is None:
                    return self._err(404, f"transfer '{request_id}' not found")
                if t["status"] != "accepted":
                    return self._err(409, f"transfer is '{t['status']}', expected 'accepted'")

                vehicle_id   = t["vehicle_id"]
                from_platoon = _platoons[t["from_platoon_id"]]
                to_platoon   = _platoons[t["to_platoon_id"]]

                # find and remove from source
                member = next((m for m in from_platoon["members"] if m["vehicle_id"] == vehicle_id), None)
                if member is None:
                    return self._err(500, f"vehicle '{vehicle_id}' disappeared from source platoon")

                from_platoon["members"].remove(member)
                # append at tail of destination
                member["role"] = "follower"
                to_platoon["members"].append(member)

                t["status"] = "committed"
                t["committed_at"] = _now()
                t["updated_at"] = _now()

                _notify_carla_trigger()

                print(
                    f"[bridge] COMMITTED  {request_id}  {vehicle_id} "
                    f"moved to {to_platoon['platoon_id']}  "
                    f"(now {len(to_platoon['members'])} members)"
                )
                self._ok({
                    "transfer": t,
                    "from_platoon": from_platoon,
                    "to_platoon": to_platoon,
                })

        # POST /transfers/{id}/merging  – CARLA notifies physical maneuver has started
        elif re.fullmatch(r"/transfers/[^/]+/merging", p):
            request_id = p.split("/")[2]
            with _lock:
                t = _transfers.get(request_id)
                if t is None:
                    return self._err(404, f"transfer '{request_id}' not found")
                if t["status"] != "committed":
                    return self._err(409, f"transfer is '{t['status']}', expected 'committed'")
                t["status"] = "merging"
                t["merging_at"] = _now()
                t["updated_at"] = _now()
                print(f"[bridge] MERGING  {request_id}  physical maneuver started in CARLA")
                self._ok(t)

        # POST /transfers/{id}/carla_complete  – CARLA notifies physical maneuver done
        elif re.fullmatch(r"/transfers/[^/]+/carla_complete", p):
            request_id = p.split("/")[2]
            with _lock:
                t = _transfers.get(request_id)
                if t is None:
                    return self._err(404, f"transfer '{request_id}' not found")
                if t["status"] not in ("committed", "merging"):
                    return self._err(409, f"transfer is '{t['status']}', expected 'committed' or 'merging'")
                t["status"] = "carla_complete"
                t["carla_complete_at"] = _now()
                t["updated_at"] = _now()
                print(f"[bridge] CARLA_COMPLETE  {request_id}  physical maneuver confirmed")
                self._ok(t)

        else:
            self._err(404, "not found")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    host, port = "0.0.0.0", 18801
    server = HTTPServer((host, port), Handler)
    server.socket.setsockopt(__import__("socket").SOL_SOCKET, __import__("socket").SO_REUSEADDR, 1)
    print(f"[bridge] Platoon Bridge Server listening on {host}:{port}")
    print("[bridge] Initial state:")
    for pid, pl in _platoons.items():
        members = ", ".join(f"{m['vehicle_id']}({m['role']})" for m in pl["members"])
        print(f"  {pid} → {pl['destination_id']}  [{members}]")
    print()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[bridge] Shutting down.")


if __name__ == "__main__":
    main()
