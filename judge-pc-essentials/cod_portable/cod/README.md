# cod — 위장(카모플라주) 상황 대비 제로샷 탐지/추적 실험

터틀봇에 디지털 카모 패턴을 입힌 뒤에도 물체를 찾아낼 수 있는 탐지 파이프라인을 준비하는
프로젝트. YOLO 같은 closed-set 탐지기는 학습 때 본 클래스만 찾기 때문에, 카모 패턴이나
"처음 보는 물체"에는 근본적으로 대응이 안 된다 — 그래서 라벨 학습 없이 동작하는 방식으로
접근한다.

## 배경 (이전 논의 요약)

- **closed-set 탐지기(YOLO 등)의 한계**: 학습 클래스에 없는 물체는 애초에 후보에도 안 오름.
  디지털 카모는 물체 경계의 그래디언트 신호 자체를 약화시켜서, 학습된 물체라도 카모를
  입히면 confidence가 떨어질 수 있음.
- **검토한 대안**:
  - Grounding DINO / SAM2 (open-vocabulary, 텍스트·프롬프트 기반, 라벨 학습 불필요)
  - SINet/ZoomNet/PFNet 등 전용 카모플라주 탐지(COD) 모델 (class-agnostic segmentation)
  - OAK-D depth 기반 전경/배경 분리 (RGB 텍스처를 안 쓰므로 카모에 안 속음) + SAM2 마스크
- **하드웨어 제약**: 개발 PC는 GPU 없이 CPU만 있음 (`torch.cuda.is_available() == False`).
  SAM2 비디오 마스크 전파는 CPU에서 프레임당 수백ms~초 단위로 느려질 수 있어 실시간 데모에
  부적합할 수 있음.

## 지금 단계에서 하려는 것

터틀봇이 아직 없어서, 아이폰 카메라(Iriun Webcam, `turtlebot4_wargame-ui/judge-vision`에서 이미
연결 확인함)로 임의의 물체를 비춰 다음을 검증:

1. 텍스트 프롬프트만으로 처음 보는 물체를 제로샷 탐지할 수 있는지
2. 그 물체가 움직여도 탐지(추적)가 유지되는지

## 선택한 구현 방식

매 프레임 무거운 제로샷 모델을 돌리는 대신:

- **최초 1회**: Grounding DINO(`IDEA-Research/grounding-dino-tiny`, HuggingFace
  transformers)로 텍스트 프롬프트 기반 제로샷 박스 탐지
- **이후 프레임**: OpenCV 내장 CSRT 트래커로 그 박스를 가볍게 추적 (CPU에서도 실시간)
- 트래킹이 끊기거나 사용자가 다시 요청하면 Grounding DINO 재탐지

SAM2 비디오 마스크 전파(픽셀 단위 정밀 분할)는 정확도는 더 높지만 CPU 실시간에는
부적합 판단 — 다음 단계(터틀봇 실물 테스트, 정밀 마스크 필요 시)에서 재검토.

## 실행

```bash
cd cod
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python zero_shot_track.py --camera 2 --prompt "a cup"
# 'd' 키: (재)탐지, 'q' 키: 종료
```

`--camera`는 로컬 장치 인덱스(숫자) 또는 스트림 URL을 받는다. 아이폰은 Iriun Webcam으로
연결하면 로컬 장치 인덱스로 잡힌다 (`v4l2-ctl --list-devices` 또는 judge-vision의
`--list-cameras`로 확인).

## 심판 PC 실시간 HIT 판정 (judge_hit_detect.py)

위 실험들을 합쳐서 실제 경기에 쓰는 스크립트. Grounding DINO(+CSRT)로 탁구공 슈터가 달린
AMR 두 대를(화면 기준 왼쪽=RED, 오른쪽=BLUE), 프레임차분(ball_track.py)으로 탁구공을 계속
추적하면서, AMR 히트박스 안의 모션 에너지가 국소 피크를 찍는 "순간 공이 그 히트박스 근처에
있었을 때만" HIT으로 인정한다 (AMR은 라켓과 달리 스스로 계속 주행하므로, 에너지 스파이크만
보면 그냥 움직이는 것도 HIT으로 오판할 수 있어서 공 근접 조건을 추가함). HIT이 확정되면
곧바로 roslibpy로 rosbridge에 접속해 frontend가 이미 구독 중인 스키마 그대로
`/judge/hit_event`(HitEvent: {victim, timestamp} — 팀 PC/프론트엔드가 이 이벤트 1건당
해당 팀 HP를 1 깎는 트리거)와 `/judge/detections`(Detections)를 publish한다.

```bash
python judge_hit_detect.py --camera 2 \
  --amr-prompt "a robot with a ping pong ball shooter on top." \
  --position-labels "Red Team AMR,Blue Team AMR" \
  --rosbridge-url ws://localhost:8000/ws
```

- `--dry-run`: rosbridge에 접속하지 않고 콘솔에만 출력 (접속 실패 시에도 자동으로 이 모드로
  전환됨).
- `--headless`: 화면 창 없이 실행 (모니터 없는 심판 PC나 자동화 테스트용).
- `--hit-height` / `--hit-cooldown` / `--ball-margin`: 각각 HIT으로 볼 최소 모션 에너지,
  같은 팀 재판정 최소 간격(프레임), 공이 히트박스에서 "근접"으로 인정되는 여유 픽셀.
- `--position-labels`는 기본값이 "Red Team AMR,Blue Team AMR"이고 박스를 왼쪽->오른쪽
  순서로 그대로 배정하므로 기본 설정 자체가 "왼쪽=RED, 오른쪽=BLUE"다. 경기장에서 팀이
  항상 같은 쪽에서 시작한다는 가정에 의존하므로, 카메라 좌우가 바뀌면 순서를 뒤집어야 한다.
- 테스트: `--camera test_videos/test4.webm --headless --dry-run`으로 카메라/rosbridge 없이
  파이프라인만 검증할 수 있다.
