"""
best_turtlebot.pt(터틀봇 전용 학습 YOLO, 클래스: enemy/sensor)로 웹캠 실시간 탐지.

Grounding DINO(제로샷) + CSRT/SAM2 조합 대신, 터틀봇에 특화된 학습 모델이 있으니
매 프레임 이 모델을 직접 돌린다 — 별도 트래커가 필요 없어서(매 프레임 재탐지라
드리프트가 없음) 구조가 단순해지고, 작은 YOLO라 GPU에서 실시간이 나온다.
"""

from __future__ import annotations

import argparse
import sys

import cv2
from ultralytics import YOLO

CLASS_COLORS = {
    "enemy": (0, 0, 255),
    "sensor": (255, 200, 0),
}
DEFAULT_COLOR = (0, 255, 0)


def main() -> None:
    parser = argparse.ArgumentParser(description="best_turtlebot.pt live webcam viewer")
    parser.add_argument("--camera", default="0", help="local device index or stream URL")
    parser.add_argument("--model", default="best_turtlebot.pt")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default=None, help="cuda/cpu, 생략하면 자동 감지")
    args = parser.parse_args()

    model = YOLO(args.model)
    if args.device:
        model.to(args.device)
    else:
        import torch

        if torch.cuda.is_available():
            model.to("cuda")
    print(f"loaded {args.model}  classes={model.names}  device={model.device}", flush=True)

    source = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"camera source {args.camera!r} could not be opened", file=sys.stderr)
        sys.exit(1)

    paused = False
    frame = None
    window = "best_turtlebot.pt live  [space]=pause  [q]=quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1024, 576)

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                print("frame grab failed", file=sys.stderr)
                break
            results = model.predict(frame, conf=args.conf, verbose=False)
            boxes = results[0].boxes

        display = frame.copy()
        for b in boxes:
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
            cls_name = model.names[int(b.cls[0])]
            conf = float(b.conf[0])
            color = CLASS_COLORS.get(cls_name, DEFAULT_COLOR)
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                display, f"{cls_name} {conf:.2f}", (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
            )

        if paused:
            cv2.putText(display, "PAUSED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow(window, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
