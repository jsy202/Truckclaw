---
name: platoon-negotiator
description: Negotiate vehicle transfers between truck platoons by exchanging destination info. Use when asked to negotiate, offer, accept, or commit a platoon transfer.
---

# Platoon Negotiator — Platoon B (RESPONDER)

You are the RESPONDER. You wait for Platoon A to go first.

---

## Step 1 — Wait for Platoon A's destination list

Do NOT check the bridge. Do NOT analyze. Do NOT propose anything.
Wait until TRUCKCLAW2 posts their truck destination list in this Discord channel.

---

## Step 2 — Post your own destination list

Read `/data/openclaw/.openclaw/workspace/data/platoon_decision_context.json`.
Note each entry in `own_vehicles[]`: vehicle_id and destination_id.

Reply to TRUCKCLAW2 with exactly this format:

```
<@1479297673432399923> Platoon B 트럭 목적지 공개:
- platoon_b_truck0: [destination_id]
- platoon_b_truck1: [destination_id]
- platoon_b_truck2: [destination_id]
```

---

## Step 3 — Check bridge state

```bash
python3 /project/scripts/platoon_bridge_ctl.py snapshot
```

If there is already a committed or merging transfer for this vehicle, skip to Step 6.

---

## Step 4 — Compare and accept

Wait for TRUCKCLAW2 to post a transfer request_id.
Then accept it:

```bash
python3 /project/scripts/platoon_bridge_ctl.py accept <request_id>
```

---

## Step 5 — Commit the transfer

```bash
python3 /project/scripts/platoon_bridge_ctl.py commit <request_id>
```

Post to Discord:
```
<@1479297673432399923> request_id [request_id] commit 완료.
```

---

## Step 6 — Trigger CARLA physical merge

```bash
curl -s -X POST http://172.18.0.1:18802/start_merge
```

Post to Discord:
```
<@1479297673432399923> CARLA 합류 트리거 전송 완료. 물리 합류 진행 중.
```

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
python3 /project/scripts/platoon_bridge_ctl.py accept <request_id>
python3 /project/scripts/platoon_bridge_ctl.py commit <request_id>
```

## Rules
- Step 1 is always first — wait for Platoon A, no exceptions.
- Never transfer truck0 (leader).
- Never say "합류 완료" — only "CARLA 합류 트리거 전송 완료".
- Every message to peer must include `<@1479297673432399923>`.
- Ignore Discord history for state — always use bridge snapshot.
