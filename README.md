# Truckclaw 🚛

**AI 에이전트가 협상하는 트럭 군집 이송 시뮬레이션**

CARLA 자율주행 시뮬레이터(Town06 고속도로)에서 두 트럭 군집이 실시간으로 주행하는 동안, Discord AI 봇(OpenClaw)이 서로 목적지 정보를 교환하고 협상하여 차량 이송을 결정합니다. 협상이 완료되면 CARLA 시뮬레이터에서 실제 차선 변경과 군집 합류가 자동으로 진행됩니다.

---

## 데모

```
[Discord 협상]
TRUCKCLAW2 (Platoon A): Platoon A 트럭 목적지 공개:
                         - platoon_a_truck0: dest_a
                         - platoon_a_truck1: dest_a
                         - platoon_a_truck2: dest_b   ← B 방향!
                         너희 목적지도 알려줘.

TRUCKCLAW1 (Platoon B): Platoon B 트럭 목적지 공개:
                         - platoon_b_truck0: dest_b
                         - platoon_b_truck1: dest_b
                         - platoon_b_truck2: dest_b

TRUCKCLAW2:              비교 결과: platoon_a_truck2 이송 필요 (목적지 일치)
                         transfer 요청 생성함. request_id: tr_xxxxxxxx

TRUCKCLAW1:              request_id tr_xxxxxxxx commit 완료.

[자동 트리거 → CARLA 물리 합류 시작]
```

---

## 개요

고속도로 주행 중 두 군집(각 3대)이 분기점 이전에 차량 이송을 협상합니다.

```
          fork(x=658)
             │
  ──────────────────────────→  dest_a (직진)
  P1: [truck0][truck1][truck2]
                              ↘
  P2: [truck0][truck1][truck2]  dest_b (우측 분기)
```

`platoon_a_truck2`의 목적지가 `dest_b`이므로, Platoon B로 이송하는 것이 효율적입니다. 두 AI 봇이 이를 판단하고 협상 → 물리 합류까지 자동으로 수행합니다.

---

## 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│                      Discord 채널                        │
│   TRUCKCLAW2 (Platoon A)  ↔  TRUCKCLAW1 (Platoon B)     │
└────────────┬────────────────────────┬────────────────────┘
             │                        │
             ▼                        ▼
┌────────────────────────────────────────────────────────┐
│              Bridge Server (port 18801)                 │
│  pending → accepted → committed → merging → complete   │
│                    │                                    │
│             commit 감지 시                              │
│         POST :18802/start_merge  ──────────────────────┼──→ CARLA 시나리오
└────────────────────────────────────────────────────────┘
                                                           │
                                              ┌────────────▼──────────────┐
                                              │    CARLA Town06 (포트 2000)│
                                              │  P1 [lane=-3]  P2 [lane=-5]│
                                              │  차선 변경 → 합류 완료     │
                                              └───────────────────────────┘
```

### 컴포넌트

| 컴포넌트 | 역할 |
|----------|------|
| **CARLA 시뮬레이터** | Town06 고속도로에서 트럭 6대 물리 시뮬레이션 |
| **Bridge Server** | 협상 상태 관리 REST API. 봇과 CARLA 사이 중재자 |
| **TRUCKCLAW2** (Platoon A 봇) | INITIATOR — 먼저 목적지 공개 후 transfer 요청 생성 |
| **TRUCKCLAW1** (Platoon B 봇) | RESPONDER — 수락 및 commit, CARLA 트리거 전송 |

---

## 전체 흐름

### 1단계 — 협상 (Discord)

1. 사용자가 봇에게 `군집 합류 가능한지 확인후 필요시 행동부탁` 입력
2. TRUCKCLAW2가 자신의 트럭 목적지 목록을 Discord에 공개
3. TRUCKCLAW1이 자신의 목적지 목록으로 응답
4. TRUCKCLAW2가 목적지 비교 → 이송 필요 차량 판단 → transfer 요청 생성
5. TRUCKCLAW1이 수락(accept) → commit

### 2단계 — 물리 합류 (CARLA)

1. Bridge Server가 commit 감지 → `POST :18802/start_merge` 자동 전송
2. CARLA 시나리오가 트리거 수신 → 어프로치 시작 (속도 65 km/h)
3. 두 군집이 나란히 정렬 → P2 tail 분리
4. 갭 확보 (TARGET_GAP = 6m)
5. 차선 변경 (lane=-5 → lane=-4 → lane=-3, 2단계)
6. P1 tail 뒤에 합류 (CACC 추종)
7. Bridge Server에 `carla_complete` 보고

### 이송 상태 머신

```
pending → accepted → committed → merging → carla_complete
  ↑요청생성   ↑B수락     ↑B커밋      ↑CARLA시작   ↑물리합류완료
```

---

## 프로젝트 구조

```
Truckclaw/
├── scenario/                          # CARLA 시뮬레이션
│   ├── examples/
│   │   └── two_platoon_truck_scenario.py   # 메인 시나리오 스크립트
│   └── src/PlatooningSimulator/       # 군집주행 컨트롤러 라이브러리
│       ├── Core.py                    # 차량/군집 기본 클래스
│       ├── PlatooningControllers.py   # CACC, LeadNavigator 등
│       └── ScenarioAgents.py          # 시나리오 에이전트
│
├── bridge/                            # 협상 브리지 서버
│   ├── platoon_bridge_server.py       # REST API 서버 (포트 18801)
│   └── platoon_bridge_ctl.py          # CLI 클라이언트 (봇이 사용)
│
├── agents/                            # OpenClaw 에이전트 설정
│   ├── platoon-a/                     # Platoon A (INITIATOR)
│   │   ├── SOUL.md                    # 에이전트 역할/행동 지침
│   │   ├── data/platoon_decision_context.json  # 차량/목적지 정보
│   │   └── skills/platoon-negotiator/SKILL.md  # 협상 스킬 단계별 지침
│   └── platoon-b/                     # Platoon B (RESPONDER)
│       ├── SOUL.md
│       ├── data/platoon_decision_context.json
│       └── skills/platoon-negotiator/SKILL.md
│
├── docker-compose.yml                 # 봇 컨테이너 실행
├── .env.example                       # 환경변수 예시
└── README.md
```

---

## 실행 방법

### 사전 요구사항

- [CARLA](https://carla.org/) 0.9.x (UE4Editor 또는 패키지 버전)
- Python 3.10+, `carla` Python 패키지
- [OpenClaw](https://openclaw.ai) 이미지 (Docker)
- Discord 봇 토큰 2개 (Platoon A용, Platoon B용)

### 1. 환경변수 설정

```bash
cp .env.example .env
# .env 편집: Discord 봇 토큰, OpenClaw 게이트웨이 토큰 입력
```

### 2. CARLA 시뮬레이터 실행

```bash
# 패키지 버전
./CarlaUE4.sh -RenderOffScreen

# 또는 UE4 에디터 (Play 버튼 클릭)
```

### 3. 브리지 서버 시작

```bash
python3 bridge/platoon_bridge_server.py
# → http://127.0.0.1:18801 에서 대기
```

### 4. OpenClaw 봇 실행

```bash
docker compose up -d
```

### 5. CARLA 시나리오 실행

```bash
PYTHONPATH=/path/to/carla/PythonAPI/carla \
  python3 scenario/examples/two_platoon_truck_scenario.py
# → 포트 18802 트리거 서버 자동 시작
# → 양 군집 20 km/h로 주행 시작
```

### 6. Discord에서 협상 시작

Discord 채널에서 아무 봇에게 입력:
```
군집 합류 가능한지 확인후 필요시 행동부탁
```

이후 모든 과정은 자동입니다.

---

## 브리지 API

| 엔드포인트 | 설명 |
|------------|------|
| `GET /snapshot` | 전체 상태 조회 |
| `POST /transfers` | transfer 요청 생성 |
| `POST /transfers/{id}/accept` | 수락 |
| `POST /transfers/{id}/commit` | 커밋 (→ CARLA 트리거 자동 발송) |
| `POST /transfers/{id}/merging` | CARLA 합류 시작 보고 |
| `POST /transfers/{id}/carla_complete` | 물리 합류 완료 보고 |

### 브리지 초기화

```bash
kill $(lsof -ti:18801) 2>/dev/null; sleep 1 && python3 bridge/platoon_bridge_server.py &
```

---

## 포트 정리

| 포트  | 용도 |
|-------|------|
| 2000  | CARLA 시뮬레이터 |
| 18801 | 브리지 서버 REST API |
| 18802 | CARLA 트리거 수신 엔드포인트 |
| 18789 | OpenClaw Platoon A 게이트웨이 |
| 18790 | OpenClaw Platoon B 게이트웨이 |

---

## 주요 파라미터 (시나리오)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `SYNC_SPEED_KMH` | 20 | 군집 기본 주행 속도 |
| `APPROACH_FAST_KMH` | 65 | 어프로치 속도 |
| `TARGET_GAP_M` | 6 | 차선 변경 전 확보할 갭 |
| `CONFIRM_TICKS` | 8 | 차선 변경 완료 확인 틱 수 |
| `MERGE_DISTANCE_LIMIT_M` | 55 | 합류 트리거 최대 거리 |

---

## 라이선스

[LICENSE](scenario/LICENSE) 참고
