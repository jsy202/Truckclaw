---
name: platoon-negotiator
description: Negotiate vehicle transfers between truck platoons by exchanging destination info. Use when asked to negotiate, offer, accept, or commit a platoon transfer.
---

# Platoon Negotiator — Platoon A (INITIATOR)

You are the INITIATOR. You always go first.

---

## Step 1 — Read own_vehicles from context

Read `/data/openclaw/.openclaw/workspace/data/platoon_decision_context.json`.
Note each entry in `own_vehicles[]`: vehicle_id and destination_id.

---

## Step 2 — Post destination list in Discord RIGHT NOW

Do this immediately. No bridge check. No analysis. No conclusions yet.
Send exactly this format, tagged to Platoon B:

```
<@1479297098938585170> Platoon A 트럭 목적지 공개:
- platoon_a_truck0: [destination_id]
- platoon_a_truck1: [destination_id]
- platoon_a_truck2: [destination_id]
너희 목적지도 알려줘.
```

**STOP HERE. Wait for TRUCKCLAW1 to reply with their list.**

---

## Step 3 — Receive Platoon B's list

Wait until TRUCKCLAW1 posts their truck destination list in Discord.
Read ONLY what they wrote. Do not proceed until their reply arrives.

---

## Step 4 — Check bridge state

```bash
python3 /project/scripts/platoon_bridge_ctl.py snapshot
```

Check if there is already an active or committed transfer. If committed and vehicle is already in Platoon B, stop — nothing to do.

---

## Step 5 — Compare and post result

Compare lists. A vehicle needs transfer if:
- Its destination_id matches Platoon B's platoon destination (from their message)
- It is not the leader (truck0)

Post comparison result in Discord:
```
<@1479297098938585170> 비교 결과: [vehicle_id] 이송 필요 (목적지 일치)
transfer 요청 생성한다.
```

---

## Step 6 — Create transfer request

```bash
python3 /project/scripts/platoon_bridge_ctl.py request <vehicle_id> platoon_a platoon_b
```

Post the request_id to Discord mentioning TRUCKCLAW1.

---

## Step 7 — Wait for commit confirmation from Platoon B

After Platoon B commits, verify with snapshot:
```bash
python3 /project/scripts/platoon_bridge_ctl.py snapshot
```

Post: "브리지 commit 확인됨. CARLA 물리 합류는 시뮬레이터에서 진행 중."
**Do NOT say "합류 완료".**

---

## Transfer Status Meanings

- `pending` → 요청 생성됨, 상대 응답 대기
- `accepted` → 상대가 수락함
- `committed` → 협상 완료, CARLA 물리 합류 대기 중
- `merging` → **CARLA에서 실제 차량이 합류 이동 중** (차선 변경 진행)
- `carla_complete` → 물리 합류 완료

`committed`는 협상만 끝난 것. `merging`이 돼야 실제로 차가 움직이는 것. `carla_complete`이 돼야 완전히 끝난 것.

## Bridge Helper

```bash
python3 /project/scripts/platoon_bridge_ctl.py snapshot
python3 /project/scripts/platoon_bridge_ctl.py request <vehicle> <from> <to>
```

## Rules
- Step 2 is always first — no exceptions.
- Never transfer truck0 (leader).
- Never say "합류 완료" — only "브리지 commit 완료".
- Every message to peer must include `<@1479297098938585170>`.
- Ignore Discord history for state — always use bridge snapshot.
