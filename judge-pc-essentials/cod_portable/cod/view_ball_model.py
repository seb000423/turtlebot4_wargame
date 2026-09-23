"""
ball_best.pt를 실제 영상에 적용해서 라이브 창으로 보여준다. 학습 데이터에 없던 배경
(조명/표지판 등)을 공으로 오탐하는 문제가 있어서, "여러 프레임 동안 거의 같은 자리에
머무르는 박스는 배경 오탐으로 보고 제외"하는 필터를 추가했다 — 진짜 공은 랠리 중
계속 움직이므로 이 필터로 정적 오탐을 걸러낼 수 있다.
"""

from __future__ import annotations

import argparse
import sys
from collections import deque

import cv2
from ultralytics import YOLO


def box_center(box):
    x1, y1, x2, y2 = box.xyxy[0].tolist()
    return ((x1 + x2) / 2, (y1 + y2) / 2, x1, y1, x2, y2)


def main() -> None:
    parser = argparse.ArgumentParser(description="ball_best.pt live viewer with static false-positive filter")
    parser.add_argument("--video", default="test_videos/test2.webm")
    parser.add_argument("--model", default="ball_best.pt")
    parser.add_argument("--conf", type=float, default=0.1, help="이 정도로 낮게 잡고 정적 필터로 걸러냄")
    parser.add_argument("--static-history", type=int, default=10, help="정적 여부 판단에 볼 과거 프레임 수")
    parser.add_argument("--static-thresh", type=float, default=6.0, help="이 픽셀 이하로 움직이면 정적으로 간주")
    args = parser.parse_args()

    model = YOLO(args.model)
    print(f"loaded {args.model}  classes={model.names}")

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"could not open {args.video}", file=sys.stderr)
        sys.exit(1)

    # 위치별(그리드 근사) 최근 등장 이력을 모아서, 안 움직이면 정적 오탐으로 판단
    history: dict[tuple[int, int], deque] = {}

    def grid_key(cx, cy, cell=20):
        return (int(cx // cell), int(cy // cell))

    paused = False
    frame = None
    window = "ball_best.pt live  [space]=pause  [q]=quit"
    cv2.namedWindow(window)

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                cap.release()
                cap = cv2.VideoCapture(args.video)  # webm은 seek이 불안정해서 재오픈으로 루프
                history.clear()
                continue
            results = model.predict(frame, conf=args.conf, verbose=False)
            boxes = results[0].boxes

            live_keys = set()
            dynamic_boxes = []
            for b in boxes:
                cx, cy, x1, y1, x2, y2 = box_center(b)
                key = grid_key(cx, cy)
                live_keys.add(key)
                hist = history.setdefault(key, deque(maxlen=args.static_history))
                hist.append((cx, cy))

                if len(hist) >= args.static_history:
                    xs = [p[0] for p in hist]
                    ys = [p[1] for p in hist]
                    spread = max(xs) - min(xs) + max(ys) - min(ys)
                    is_static = spread < args.static_thresh
                else:
                    is_static = False  # 아직 이력 부족하면 일단 통과

                dynamic_boxes.append((x1, y1, x2, y2, b.conf[0].item(), is_static))

            # 안 나온 위치는 이력에서 서서히 정리 (메모리 방지, 너무 커지지 않게)
            if len(history) > 200:
                for k in list(history.keys()):
                    if k not in live_keys:
                        history.pop(k, None)

        display = frame.copy()
        for x1, y1, x2, y2, conf, is_static in dynamic_boxes:
            color = (0, 0, 255) if is_static else (0, 255, 0)
            label = f"{conf:.2f}{' (static, 무시)' if is_static else ''}"
            cv2.rectangle(display, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(display, label, (int(x1), max(0, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if paused:
            cv2.putText(display, "PAUSED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow(window, display)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
