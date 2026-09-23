"""
고정 좌표 ROI 대신, 그라운딩 디노(제로샷)로 타겟(사람/로봇)을 한 번 찾고 CSRT로 실시간
추적해서 그 박스를 동적 ROI로 쓴다. 그 ROI 안에서 프레임 차분 에너지의 국소 피크를
찾아 "HIT"을 실시간(미래 프레임 안 보고) 판정한다.

두 파이프라인의 결합:
  - zero_shot_track.py: 제로샷 탐지(최초 1회) + CSRT 추적 → 타겟 박스
  - live_hit_detect.py: ROI 안 모션 에너지 국소 피크 → 실시간 HIT 판정
"""

from __future__ import annotations

import argparse
import sys
from collections import deque

import cv2
import torch
from PIL import Image
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

MODEL_ID = "IDEA-Research/grounding-dino-tiny"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def load_model():
    processor = AutoProcessor.from_pretrained(MODEL_ID)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL_ID)
    model.eval()
    model.to(DEVICE)
    return processor, model


def detect(processor, model, frame_bgr, prompt: str, box_threshold: float, text_threshold: float):
    """입력: BGR 프레임, 텍스트 프롬프트. 출력: ((x,y,w,h), score, label) confidence 최고 1개, 없으면 None."""
    image = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    text = prompt.strip()
    if not text.endswith("."):
        text += "."
    inputs = processor(images=image, text=text, return_tensors="pt").to(DEVICE)
    with torch.no_grad():
        outputs = model(**inputs)
    results = processor.post_process_grounded_object_detection(
        outputs, inputs.input_ids, threshold=box_threshold, text_threshold=text_threshold,
        target_sizes=[image.size[::-1]],
    )[0]
    if len(results["boxes"]) == 0:
        return None
    best = int(results["scores"].argmax())
    x1, y1, x2, y2 = results["boxes"][best].tolist()
    return (x1, y1, x2 - x1, y2 - y1), float(results["scores"][best]), results["labels"][best]


def make_tracker():
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    return cv2.legacy.TrackerCSRT_create()


def motion_energy(diff_gray, roi):
    x1, y1, x2, y2 = [int(v) for v in roi]
    x1, y1 = max(0, x1), max(0, y1)
    region = diff_gray[y1:y2, x1:x2]
    if region.size == 0:
        return 0.0
    _, thresh = cv2.threshold(region, 15, 255, cv2.THRESH_BINARY)
    return float(thresh.sum()) / 255.0


def main() -> None:
    parser = argparse.ArgumentParser(description="제로샷 탐지+추적으로 동적 ROI를 잡고, 그 안에서 실시간 HIT 판정")
    parser.add_argument("--video", default="test_videos/test4.webm")
    parser.add_argument("--prompt", default="a person", help='타겟 프롬프트 (실제 게임에선 "a robot" 등)')
    parser.add_argument("--box-threshold", type=float, default=0.3)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--height", type=float, default=3000.0, help="이 에너지 이상이어야 HIT 후보")
    parser.add_argument("--cooldown", type=int, default=15)
    parser.add_argument("--redetect-every", type=int, default=90, help="트래킹 놓쳤을 때뿐 아니라 주기적으로도 재탐지(프레임 단위, 0=끄기)")
    args = parser.parse_args()

    print("loading Grounding DINO...", flush=True)
    processor, model = load_model()
    print("model loaded.", flush=True)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"could not open {args.video}", file=sys.stderr)
        sys.exit(1)

    ok, frame = cap.read()
    if not ok:
        print("empty video", file=sys.stderr)
        sys.exit(1)
    prev_gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)

    tracker = None
    roi_box = None  # (x,y,w,h)
    recent3: deque[tuple[int, float]] = deque(maxlen=3)
    hit_count = 0
    frames_since_hit = 999
    hit_flash = 0
    frame_idx = 0

    def try_detect(f):
        print(f"  [detecting '{args.prompt}' ...]", flush=True)
        det = detect(processor, model, f, args.prompt, args.box_threshold, args.text_threshold)
        if det is None:
            print("  not found", flush=True)
            return None, None
        (x, y, w, h), score, label = det
        print(f"  found '{label}' conf={score:.2f} box=({x:.0f},{y:.0f},{w:.0f},{h:.0f})", flush=True)
        t = make_tracker()
        t.init(f, (int(x), int(y), int(w), int(h)))
        return t, (int(x), int(y), int(w), int(h))

    tracker, roi_box = try_detect(frame)

    window = "auto-ROI real-time hit detection  [d]=redetect  [q]=quit"
    cv2.namedWindow(window)

    while True:
        ok, frame = cap.read()
        if not ok:
            cap.release()
            cap = cv2.VideoCapture(args.video)
            ok, frame = cap.read()
            prev_gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            recent3.clear()
            frame_idx = 0
            tracker, roi_box = try_detect(frame)
            continue

        frame_idx += 1
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        diff = cv2.absdiff(gray, prev_gray)
        prev_gray = gray

        if tracker is not None:
            ok_t, box = tracker.update(frame)
            if ok_t:
                roi_box = box
            else:
                print(f"frame {frame_idx}: tracking lost, redetecting", flush=True)
                tracker, roi_box = try_detect(frame)
        if args.redetect_every and frame_idx % args.redetect_every == 0:
            tracker, roi_box = try_detect(frame)

        display = frame.copy()
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if key == ord("d"):
            tracker, roi_box = try_detect(frame)

        frames_since_hit += 1
        if roi_box is not None:
            x, y, w, h = roi_box
            roi = (x, y, x + w, y + h)
            e = motion_energy(diff, roi)
            recent3.append((frame_idx, e))

            if len(recent3) == 3:
                (_, e_prev2), (mid_idx, e_mid), (_, e_now) = recent3
                is_local_peak = e_mid >= e_prev2 and e_mid >= e_now and e_mid >= args.height
                if is_local_peak and frames_since_hit >= args.cooldown:
                    hit_count += 1
                    frames_since_hit = 0
                    hit_flash = 8
                    print(f"frame {mid_idx}: HIT #{hit_count} (energy={e_mid:.0f})", flush=True)

            color = (0, 0, 255) if hit_flash > 0 else (0, 255, 0)
            cv2.rectangle(display, (int(x), int(y)), (int(x + w), int(y + h)), color, 3 if hit_flash > 0 else 2)
            cv2.putText(display, f"energy={e:.0f}", (int(x), max(0, int(y) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        else:
            cv2.putText(display, "no target - press 'd' to detect", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"frame {frame_idx}  hits={hit_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if hit_flash > 0:
            cv2.putText(display, "HIT!", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            hit_flash -= 1

        cv2.imshow(window, display)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
