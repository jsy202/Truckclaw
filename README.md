# Truck Platoon OpenClaw

CARLA Town06 고속도로에서 두 트럭 군집이 **AI 에이전트(OpenClaw + Discord)로 협상**하여 차량을 이송하는 시뮬레이션.

## 개요

```
Platoon A (3대)  ──→  fork 이전에 truck2를 Platoon B로 이송
Platoon B (3대)  ──→  truck2를 tail에 합류
```

협상은 Discord 채널에서 두 AI 봇이 직접 목적지를 교환하고 transfer를 commit.
commit 즉시 브리지 서버가 CARLA 시나리오에 트리거를 쏘아 물리 합류 시작.

## 구조

```
truck-platoon-openclaw/
├── scenario/          # CARLA 시뮬레이션 (Town06, 3+3대 군집)
│   ├── examples/two_platoon_truck_scenario.py
│   └── src/PlatooningSimulator/
├── bridge/            # 브리지 서버 (포트 18801)
│   ├── platoon_bridge_server.py
│   └── platoon_bridge_ctl.py
├── agents/            # OpenClaw 에이전트 워크스페이스
│   ├── platoon-a/     # INITIATOR (TRUCKCLAW2)
│   └── platoon-b/     # RESPONDER (TRUCKCLAW1)
├── docker-compose.yml
└── .env.example
```

## 실행 방법

### 1. 환경변수 설정

```bash
cp .env.example .env
# .env에서 Discord 봇 토큰, OpenClaw 게이트웨이 토큰 입력
```

### 2. 브리지 서버 시작

```bash
python3 bridge/platoon_bridge_server.py
```

### 3. OpenClaw 봇 실행

```bash
docker compose up -d
```

### 4. CARLA 시뮬레이션 실행

```bash
PYTHONPATH=/path/to/carla/PythonAPI/carla \
  python3 scenario/examples/two_platoon_truck_scenario.py
```

### 5. Discord에서 봇에게 명령

```
군집 합류 가능한지 확인후 필요시 행동부탁
```

두 봇이 목적지를 교환하고 transfer를 협상 → commit 후 자동으로 CARLA 물리 합류 시작.

## 흐름

```
[봇 협상]
TRUCKCLAW2: Platoon A 목적지 공개 → truck2=dest_b
TRUCKCLAW1: Platoon B 목적지 공개 → 전부 dest_b
TRUCKCLAW2: transfer 요청 생성 (tr_xxxxxxxx)
TRUCKCLAW1: accept → commit

[자동 트리거]
브리지 서버 → POST :18802/start_merge → CARLA 시나리오

[물리 합류]
P2 truck2 분리 → 갭 확보 → 차선 변경 (2 lanes) → P1 tail 합류
```

## 포트

| 포트  | 용도 |
|-------|------|
| 18801 | 브리지 서버 REST API |
| 18802 | CARLA 트리거 수신 (시나리오 내장) |
| 18789 | OpenClaw Platoon A 게이트웨이 |
| 18790 | OpenClaw Platoon B 게이트웨이 |

## 브리지 초기화

```bash
kill $(lsof -ti:18801) 2>/dev/null; sleep 1 && python3 bridge/platoon_bridge_server.py &
```

## 의존성

- [CARLA](https://carla.org/) 0.9.16
- Python 3.10+
- [OpenClaw](https://openclaw.ai)
- Docker / Docker Compose
