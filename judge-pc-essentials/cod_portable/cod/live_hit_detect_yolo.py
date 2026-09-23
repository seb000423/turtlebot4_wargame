"""
그라운딩 디노(제로샷) 대신 일반 YOLOv8(COCO 사전학습, person 클래스 기본 포함)로 타겟을
매 프레임 직접 탐지한다. YOLO는 프레임당 수십ms라 재탐지 때문에 화면이 멈추는 문제가 아예
없다 — 그래서 CSRT 트래커도 필요 없이 매 프레임 새로 탐지해서 박스를 그대로 ROI로 쓴다.
그 ROI 안 모션 에너지 국소 피크로 실시간(미래 프레임 안 보고) HIT 판정.
"""

from __future__ import annotations

import argparse
import sys
from collections import deque

import cv2
from ultralytics import YOLO


def motion_energy(diff_gray, roi):
    x1, y1, x2, y2 = [int(v) for v in roi]
    x1, y1 = max(0, x1), max(0, y1)
    region = diff_gray[y1:y2, x1:x2]
    if region.size == 0:
        return 0.0
    _, thresh = cv2.threshold(region, 15, 255, cv2.THRESH_BINARY)
    return float(thresh.sum()) / 255.0


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLO 매 프레임 탐지로 동적 ROI + 실시간 HIT 판정")
    parser.add_argument("--video", default="test_videos/test4.webm")
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--cls", default="person", help="타겟 클래스 이름 (COCO 기준, 예: person)")
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument("--height", type=float, default=3000.0, help="이 에너지 이상이어야 HIT 후보")
    parser.add_argument("--cooldown", type=int, default=15)
    parser.add_argument("--upper-frac", type=float, default=0.55,
                         help="박스 위에서부터 이 비율만 ROI로 사용 (다리 발놀림 오탐 방지, 1.0=전신)")
    args = parser.parse_args()

    model = YOLO(args.model)
    name_to_id = {v: k for k, v in model.names.items()}
    if args.cls not in name_to_id:
        print(f"'{args.cls}' not in model classes: {list(model.names.values())}", file=sys.stderr)
        sys.exit(1)
    target_id = name_to_id[args.cls]
    print(f"loaded {args.model}, tracking class '{args.cls}' (id={target_id})", flush=True)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"could not open {args.video}", file=sys.stderr)
        sys.exit(1)

    ok, frame = cap.read()
    if not ok:
        print("empty video", file=sys.stderr)
        sys.exit(1)
    prev_gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)

    recent3: deque[tuple[int, float]] = deque(maxlen=3)
    hit_count = 0
    frames_since_hit = 999
    hit_flash = 0
    frame_idx = 0

    window = "YOLO auto-ROI real-time hit detection  [q]=quit"
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
            continue

        frame_idx += 1
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        diff = cv2.absdiff(gray, prev_gray)
        prev_gray = gray

        results = model.predict(frame, conf=args.conf, classes=[target_id], verbose=False)
        boxes = results[0].boxes
        roi_box = None
        if len(boxes) > 0:
            best = max(boxes, key=lambda b: b.conf[0].item())
            x1, y1, x2, y2 = best.xyxy[0].tolist()
            # 다리 쪽 발놀림이 오탐을 유발해서, 상체(팔/라켓 있는 위쪽)만 남기고 하반신은 제외
            y2 = y1 + (y2 - y1) * args.upper_frac
            roi_box = (x1, y1, x2 - x1, y2 - y1)

        display = frame.copy()
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
            cv2.putText(display, f"no '{args.cls}' detected", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"frame {frame_idx}  hits={hit_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if hit_flash > 0:
            cv2.putText(display, "HIT!", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            hit_flash -= 1

        cv2.imshow(window, display)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
