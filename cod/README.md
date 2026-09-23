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
