<div align="center">

# TurtleBot4-Wargame

### Camouflaged Target Detection & Human-in-the-Loop Engagement with TurtleBot4
### TurtleBot4 기반 위장 표적 탐지 · 피아식별 · 발사 승인 시스템

위장 도색된 적 로봇을 탐지하고, **IFF(피아식별)** 로 아군을 걸러낸 뒤  
**사람의 발사 승인(Human-in-the-Loop)** 을 거쳐 사격하는 ROS2 기반 AMR 워게임 프로젝트

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Robot](https://img.shields.io/badge/Robot-TurtleBot4-0B7A75)](https://turtlebot.github.io/turtlebot4-user-manual/)
[![Vision](https://img.shields.io/badge/Vision-YOLO%20%2B%20ByteTrack-111F68)](https://docs.ultralytics.com/)
[![Nav](https://img.shields.io/badge/Nav-SLAM%20%2B%20Nav2-5A5A5A)](https://docs.nav2.org/)
[![UI](https://img.shields.io/badge/UI-React%20%2B%20FastAPI-61DAFB?logo=react&logoColor=black)](https://react.dev/)

</div>

---

## Overview

일반적인 객체 탐지와 달리, 본 프로젝트의 표적은 **위장 도색된 TurtleBot**이라 기본 모델로는 안정적인 인식이 어려움.  
이를 해결하기 위해 위장 로봇 데이터로 YOLO를 직접 학습하고, 탐지된 로봇이 아군인지 **UDP Challenge-Response 기반 IFF**로 한 번 더 확인함.

로봇은 SLAM + Nav2로 맵을 만들며 순찰하다가 적을 발견하면 조준까지 자동으로 수행함.  
발사는 자동으로 하지 않으며, 팀 PC UI에서 **사람이 승인해야만 아두이노 슈터가 동작**함.

`mission_manager`가 순찰 · 조준 · 추적 · 복귀 상태를 관리하고, 심판 PC는 별도 카메라로 피격을 판정해 HP와 경기 상태를 양 팀에 전달함.

> **담당 파트** — YOLO 기반 위장 표적 탐지 · IFF(피아식별) (`patrol_ws/src/yolo_detector/`)

---

## Demo

<div align="center">
  <img src="assets/demo.gif" width="82%" alt="TurtleBot4 wargame demo">
  <br>
  <sub>순찰 중 위장 표적 탐지 → IFF 판정 → 조준 → 팀 PC 발사 승인 → 핑퐁볼 발사</sub>
</div>

---

## Key Contributions

1. **Camouflaged Target Detection**  
   위장 도색 TurtleBot 데이터로 YOLO(`y11s_my_best.pt`)를 학습하고, ByteTrack으로 프레임 간 Track ID 유지.

2. **IFF (Identification Friend or Foe)**  
   새로 추적되는 로봇마다 6자리 Challenge를 UDP로 브로드캐스트하고, 1초 안에 올바른 SHA-256 응답이 오면 아군으로 분류.

3. **Depth-based Decoy Filtering**  
   OAK-D Depth로 박스 중심과 주변 배경의 거리 차를 비교해, 10 cm 미만이면 **바닥·벽의 평면 위장 무늬(Flat Shadow)** 로 보고 표적에서 제외.

4. **Mission State Machine**  
   `PATROL → WAIT_COMMAND → COOLDOWN → TRACK → RETURN` 상태 전이로 순찰 · 조준 · 추적 · 복귀 관리. 장탄 5발 · 재장전 10초 규칙을 로봇 쪽에서도 한 번 더 검증.

5. **Human-in-the-Loop Fire Approval**  
   조준 완료 후 rosbridge로 연결된 React UI에서 사람이 승인(`shoot_approval`)해야 발사 단계로 진행.

6. **Judge System**  
   심판 PC 카메라로 AMR과 핑퐁볼을 추적해 피격 순간을 판정하고, HP · 경기 상태를 양 팀 UI에 전달.

---

## System

```text
┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐
│  SLAM + Nav2     │     │  yolo_detector   │     │ mission_manager  │     │  Team PC UI      │
│  + explore_lite  │ ──▶ │ (YOLO + IFF)     │ ──▶ │ (State Machine)  │ ──▶ │ (발사 승인)       │
│  (자율 순찰)      │     │                  │     │                  │     │                  │
└──────────────────┘     └──────────────────┘     └──────────────────┘     └──────────────────┘
                                  │                        │                        │
                                  ▼                        ▼                        ▼
                         aim/set_ori · enemy_tf     azimuth_tracker /       shoot_approval
                         enemy/detections           target_nav → cmd_vel    → fire_ready → Arduino
                                                                                     │
                                                                                     ▼
                                            Judge PC Camera ──▶ /judge/hit_event ──▶ HP 감소
```

<div align="center">

| Phase | Description |
|---|---|
| **1. Undock & Patrol** | 시작 시 자동 언도킹 후 SLAM + Nav2 + explore_lite로 맵을 만들며 순찰 (속도 70% 제한) |
| **2. Detection** | 폰 카메라 / OAK-D 영상에서 YOLO + ByteTrack으로 로봇 탐지 |
| **3. IFF** | UDP Challenge-Response로 아군 / 적군 분류, 평면 위장 무늬는 Depth로 제외 |
| **4. Aim** | `aim/set_ori` 오차를 이용해 `azimuth_tracker`가 제자리 회전 조준 |
| **5. Approval** | 정렬 완료(`aim/complete`) 시 팀 PC UI에서 사람이 발사 승인 |
| **6. Fire** | 아두이노 슈터가 핑퐁볼 발사, 심판 PC가 피격 판정 |
| **7. Track / Return** | 적 HP에 따라 추적 또는 순찰 복귀, 내 HP 0 또는 적 HP 0이면 시작 위치로 복귀 |

</div>

---

## Detection & IFF

<div align="center">

| Node | Input | Implementation |
|:---:|:---:|:---|
| **phone_cam_detector**<br>폰 카메라 | Iriun Webcam<br>(v4l2 → ffmpeg) | YOLO 탐지, bbox 높이 기반 거리 추정, 조준 오차 · 탐지 목록 발행 |
| **bbox4**<br>OAK-D | RGB + Depth | YOLO + ByteTrack + **IFF**, Depth 평면 필터, TF로 적 위치(`enemy_tf`) 발행 |

</div>

### YOLO Detection
- 위장 도색 TurtleBot 데이터로 학습한 커스텀 가중치 `y11s_my_best.pt` 사용 (`enemy`, `sensor` 클래스)
- 영상 수신과 추론 스레드를 분리하고 **최신 프레임만 추론**해 지연 누적 방지
- 조준 오차 `aim/set_ori`(−1 ~ 1 정규화), 탐지 목록 `enemy/detections`, 카메라 영상 `enemy_image/compressed` 발행

### Phone Camera (Depth 없음)
- OpenCV V4L2 백엔드가 Iriun 가상 장치를 열지 못해 **ffmpeg rawvideo 파이프**로 프레임 수신
- 폰 미연결 시 나오는 단색 대기화면을 감지해 추론을 멈추고, 연결되면 재시작 없이 자동 재개
- Depth 대신 핀홀 모델로 거리 추정: `distance = k / bbox_height` (93 cm · 343 px 실측 보정)

### Tracking
- ByteTrack으로 프레임 간 Track ID 유지 (`model.track(persist=True)`)
- IFF 판정 결과를 Track ID 단위로 캐싱해 같은 로봇에 반복 요청하지 않음
- 적의 map/odom 좌표 변화량으로 이동 속도를 계산해 움직이는 적 로그 출력

### IFF
- 새 Track ID가 등장하면 6자리 난수 Challenge를 UDP(:12345) 브로드캐스트
- 아군 로봇은 수신 스레드에서 `sha256(challenge)`로 자동 응답
- 1초 안에 일치하는 응답이 오면 friendly, 없으면 enemy로 분류

```text
 Detector (나)                              상대 로봇
     │  새 Track ID 등장                         │
     │── challenge (6자리 난수, UDP broadcast) ──>│
     │                                           │ sha256(challenge)
     │<──────────────── response ────────────────│
     │
     ├─ 1초 안에 일치 → friendly (파란 박스)
     └─ 응답 없음     → enemy    (빨간 박스, 조준 대상)
```

### Depth Decoy Filter

```text
Z_obj = median(depth[박스 중심 ROI])
Z_bg  = median(depth[박스를 좌우 20 px 확장한 영역의 세로 중앙 40% 구간])

|Z_bg − Z_obj| < 0.10 m  →  flat_ignored (회색 박스, 조준 제외)
그 외                    →  돌출된 실제 로봇으로 판단
```

---

## Control Architecture

```text
Phone Cam / OAK-D ──> yolo_detector ──(UDP)── IFF ── 상대 로봇
                           │
                           │ aim/set_ori · enemy_distance
                           ▼
                    mission_manager ──────────> SLAM / Nav2 / explore_lite
                           │  robot_mode (TRANSIENT_LOCAL)
                           ├──> azimuth_tracker ──┐
                           ├──> target_nav      ──┴──> cmd_vel ──> TurtleBot4
                           │
                           │ status · aim/complete
                           ▼
            rosbridge ⇄ Team PC UI ──(shoot_approval)──> mission_manager ──(fire_ready)──> arduino_shooter
```

- **yolo_detector** — 위장 표적 탐지, Track ID 유지, IFF 판정
- **mission_manager** — 상태 전이, HP · 장탄 관리, 언도킹, 시작 위치 복귀 주행
- **azimuth_tracker** — WAIT_COMMAND 상태에서만 제자리 회전 조준
- **target_nav** — TRACK 상태에서 회전 + 전진을 하나의 Twist로 합쳐 접근
- **arduino_shooter** — Serial로 발사 명령(`F`) 전송

### Mission State

```text
                 PATROL
                   │ 적 발견 (aim/set_ori 0.5초 이내 수신)
                   ▼
              WAIT_COMMAND ── 적 유실 5초 ──> PATROL
                   │ 정렬(|err| < 0.2) → 사람 승인 → 장탄·재장전 확인 → 발사
                   ▼
                COOLDOWN (15초, 적 무시하고 순찰 유지)
          ┌────────┴────────┐
   적 HP 2~3 (15초)      적 HP == 1 (즉시)
          ▼                 ▼
        PATROL            TRACK
                            │ track_complete (1 m 도달 / 신호 유실 3초)
                            ▼
                          PATROL

   내 HP 0 또는 적 HP 0 ──> RETURN (어느 상태에서든 즉시, Nav2로 시작 좌표 복귀)
```

### Aim / Track Control

```text
e      = EMA(aim/set_ori, α = 0.4)
ω_cmd  = clip(−Kp · e, ±0.3 rad/s)          # Kp = 0.8, |e| < 0.05 이면 정지
ω      = slew_limit(ω_cmd, 1.0 rad/s²)      # 부드러운 가감속
v      = 0.15 m/s  if distance > 1.0 m  else 0  (TRACK 전용)
```

<div align="center">

| Node | Active Mode | Output |
|---|:---:|---|
| `azimuth_tracker` | WAIT_COMMAND | 제자리 회전 (`linear.x = 0`) |
| `target_nav` | TRACK | 회전 + 전진 (한 메시지로 합쳐 cmd_vel 충돌 방지) |

</div>

---

## Judge System

<div align="center">

| Component | Implementation |
|---|---|
| **AMR 추적** | Grounding DINO 제로샷 탐지 1회 + CSRT 트래커로 두 AMR 추적 (화면 왼쪽 RED / 오른쪽 BLUE) |
| **공 추적** | 프레임 차분으로 핑퐁볼 궤적 추적 |
| **HIT 판정** | AMR 히트박스의 모션 에너지 피크 순간 + 공이 히트박스 근처에 있을 때만 HIT 인정 |
| **HP 반영** | `/judge/hit_event` 1건 = 해당 팀 HP −1, `/red\|blue/status`를 TRANSIENT_LOCAL로 발행 |
| **경기 제어** | 심판 UI 버튼 → FastAPI → `/judge/match_state` (`IDLE` / `RUNNING` / `END`) |

</div>

---

## Web UI

<div align="center">

| UI | Function |
|---|---|
| **Team PC** | 팀 로그인, 로봇 카메라 · 탐지 결과, HP · 배터리 · 장탄 · 재장전, **발사 승인** |
| **Judge PC** | 심판 카메라, 경기 시작 / 종료, 점수판, 피격 로그, 경기 기록 |

| Topic | Type | Description |
|---|---|---|
| `/{ns}/status` | `turtlebot4_wargame_msgs/TeamStatus` | HP · 배터리 · 장탄 · 재장전 · 미션 상태 |
| `/{ns}/aim/set_ori` | `std_msgs/Float32` | 화면 중심 대비 좌우 조준 오차 (−1 ~ 1) |
| `/{ns}/aim/complete` | `std_msgs/Bool` | 정렬 완료 여부 |
| `/{ns}/shoot_approval` | `std_msgs/Bool` | 사람의 발사 승인 |
| `/{ns}/enemy_image/compressed` | `sensor_msgs/CompressedImage` | 탐지 결과가 그려진 JPEG 영상 |
| `/{ns}/enemy/detections` | `turtlebot4_wargame_msgs/Detections` | bbox 목록 (0 ~ 1 정규화) |
| `/judge/match_state` | `std_msgs/String` | `IDLE` / `RUNNING` / `END` |
| `/judge/hit_event` | `turtlebot4_wargame_msgs/HitEvent` | 피격 팀 · 시각 |

</div>

---

## Engineering Challenges

<div align="center">

| Problem | Solution |
|---|---|
| 콜백에서 동기 추론 시 프레임레이트 급락 | 수신 / 추론 스레드 분리, 최신 프레임만 처리 |
| 같은 로봇에 IFF 요청이 매 프레임 반복 | ByteTrack ID 기준으로 판정 결과 캐싱 |
| 바닥·벽의 위장 무늬를 로봇으로 오탐 | Depth로 배경과의 거리 차를 비교해 평면 표적 제외 |
| OpenCV가 Iriun 가상 웹캠을 열지 못함 | ffmpeg rawvideo 파이프로 캡처, 멈춤 감지 시 자동 재시작 |
| 폰 카메라에는 Depth가 없어 거리 측정 불가 | bbox 높이 기반 핀홀 거리 추정 (실측 보정) |
| azimuth / target_nav가 cmd_vel을 서로 덮어씀 | `robot_mode`별로 한 노드만 발행, TRACK은 회전 + 전진을 한 메시지로 합침 |
| 로봇 여러 대 실행 시 토픽 충돌 | 모든 토픽을 상대경로로 작성, Namespace만 바꿔 실행 |
| 늦게 실행된 노드 · UI가 현재 상태를 받지 못함 | `robot_mode`, `status`를 TRANSIENT_LOCAL QoS로 발행 |
| SLAM + Nav2 + YOLO 동시 구동 시 Wi-Fi 대역폭 부족 | 사용하지 않는 주변장치(OAK-D 등) 분리 |

</div>

---

## Environment

<div align="center">

| Category | Specification |
|---|---|
| OS | Ubuntu 22.04 |
| Middleware | ROS2 Humble (Discovery Server) |
| Robot | TurtleBot4 (Create3 + Raspberry Pi) |
| Camera | OAK-D / Smartphone (Iriun Webcam) |
| Detection | Ultralytics YOLO + ByteTrack |
| Judge Vision | Grounding DINO + OpenCV CSRT |
| Navigation | SLAM Toolbox, Nav2, explore_lite |
| Shooter | Arduino (Serial, 9600 baud) |
| UI | React + Vite, FastAPI, rosbridge |
| Language | Python 3.10, TypeScript |

</div>

> **실제 로봇 구동 시 필요 항목**
> - TurtleBot4 표준 워크스페이스 (`turtlebot4_ws`)
> - `m-explore-ros2` (explore_lite) — `AMR_ws/src/`에 별도 clone
> - `ros-humble-rosbridge-server`, Node.js 18+, `ffmpeg`, `sshpass`
> - 로봇과 노트북의 `ROS_DOMAIN_ID`, `ROS_DISCOVERY_SERVER` 설정 일치

---

## Installation

```bash
git clone https://github.com/seb000423/turtlebot4_wargame.git
cd turtlebot4_wargame
source /opt/ros/humble/setup.bash

# 탐지 + 커스텀 메시지
cd patrol_ws
colcon build --symlink-install
source install/setup.bash

# 상태머신 · 조준 · 추적
cd ../AMR_ws
colcon build --symlink-install --packages-select tracking
source install/setup.bash
```

Python / UI dependencies:

```bash
pip install ultralytics opencv-python
pip install -r turtlebot4_wargame-ui/backend/requirements.txt

sudo apt install ros-humble-rosbridge-server ffmpeg sshpass

cd turtlebot4_wargame-ui/frontend-team-pc && npm install
```

---

## Usage

### 1. SLAM / Nav2

```bash
ros2 launch turtlebot4_navigation slam.launch.py namespace:=robot4
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=robot4
```

### 2. Detection

```bash
# 폰 카메라 (Iriun Webcam)
ros2 run yolo_detector phone_cam_detector --ros-args -r __ns:=/robot4

# OAK-D + IFF + Depth 필터
ros2 run yolo_detector bbox4 --ros-args -r __ns:=/robot4
```

### 3. Mission / Aim / Track

```bash
ros2 run tracking mission_manager --ros-args -r __ns:=/robot4
ros2 run tracking azimuth --ros-args -r __ns:=/robot4
ros2 run tracking target --ros-args -r __ns:=/robot4
```

### 4. rosbridge + Team PC UI

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
cd turtlebot4_wargame-ui/frontend-team-pc && npm run dev
```

Open:

```text
http://localhost:5173
```

### 5. All-in-One

```bash
export ROBOT_IP=<ROBOT_IP>
export ROBOT_PW=<ROBOT_PASSWORD>

./start_all.sh      # SLAM → Nav2 → Iriun → 탐지 → 상태머신 → 조준/추적 → rosbridge → UI → 슈터(SSH)
./stop_all.sh
```

- 로그는 `/tmp/turtlebot4_wargame_logs/`에 저장
- 슈터 노드는 로봇 Raspberry Pi에 SSH로 접속해 `/dev/ttyACM*` 포트를 자동 탐색 후 실행

### ROS2 Command Example

```bash
# 발사 승인
ros2 topic pub --once /robot4/shoot_approval std_msgs/msg/Bool "data: true"

# HP 수동 입력 (상태 전이 테스트)
ros2 topic pub --once /robot4/my_hp std_msgs/msg/Int32 "data: 2"
ros2 topic pub --once /robot4/enemy_hp std_msgs/msg/Int32 "data: 1"

# 상태 확인
ros2 topic echo /robot4/robot_mode
ros2 topic echo /robot4/status
```

> Robot IP, Namespace, YOLO 가중치 경로(`YOLO_MODEL_PATH`)는 실제 환경에 맞게 설정 필요.

---

## Repository Structure

```text
turtlebot4_wargame/
├── patrol_ws/src/
│   ├── yolo_detector/                 # YOLO 탐지 + IFF (담당 파트)
│   │   ├── yolo_detector/
│   │   │   ├── phone_cam_detector.py  #   폰 카메라용 (ffmpeg 캡처 · 거리 추정)
│   │   │   └── bbox4.py               #   OAK-D용 (Depth · TF · IFF · 평면 필터)
│   │   ├── custom_bytetrack.yaml      #   ByteTrack 튜닝 설정
│   │   └── y11s_my_best.pt            #   위장 TurtleBot 학습 가중치
│   └── turtlebot4_wargame_msgs/       # TeamStatus · Detection · Detections
│
├── AMR_ws/tracking/tracking/          # 상태머신 · 조준 · 추적
│   ├── mission_manager.py             #   상태 전이 · HP · 장탄 · 복귀
│   ├── azimuth_tracker.py             #   WAIT_COMMAND 제자리 조준
│   ├── target_nav.py                  #   TRACK 접근 주행
│   ├── random_patrol.py               #   explore 정체 시 웨이포인트 순찰
│   └── Tb4_follower.py                #   좌표 기반 추종 (실험)
│
├── turtlebot4_wargame-ui/
│   ├── frontend-team-pc/              # 팀 PC UI (발사 승인)
│   ├── frontend-judge-pc/             # 심판 PC UI
│   ├── backend/                       # FastAPI (경기 기록 · rosbridge mock)
│   └── judge-vision/                  # 심판 카메라 YOLO 탐지
│
├── judge-pc-essentials/               # 심판 PC 단독 구동 패키지
│   ├── backend/                       #   경기 제어 · HP 관리 ROS2 노드
│   ├── cod_portable/cod/              #   judge_hit_detect.py (피격 판정)
│   ├── frontend-judge-pc/
│   └── ros2_ws/src/                   #   HitEvent 포함 메시지 패키지
│
├── cod/                               # 제로샷 탐지 · 핑퐁볼 추적 실험
├── arduino_shooter/                   # 슈터 ROS2 노드 (Serial)
├── start_all.sh                       # 전체 프로세스 일괄 실행
├── stop_all.sh
└── README.md
```

---

<div align="center">

**ROS2 × TurtleBot4 × YOLO × React**

Camouflaged target detection with human-approved engagement.

</div>
