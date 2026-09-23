"""
Colab에서 학습해 온 ball_best.pt를 실제로 검증한다: 로딩, 추론 속도, 그리고 가진
테스트 영상(test2.webm 탁구 방송, test3.webm 슈터)에서 실제로 공을 잡는지 확인.
"""

from __future__ import annotations

import sys
import time

import cv2
from ultralytics import YOLO


def main() -> None:
    model_path = sys.argv[1] if len(sys.argv) > 1 else "ball_best.pt"
    model = YOLO(model_path)
    print(f"loaded {model_path}  classes={model.names}")

    for video in ["test_videos/test2.webm", "test_videos/test3.webm"]:
        cap = cv2.VideoCapture(video)
        if not cap.isOpened():
            print(f"  {video}: could not open, skipping")
            continue

        ok, frame = cap.read()
        if not ok:
            print(f"  {video}: empty")
            continue

        # 워밍업
        model.predict(frame, conf=0.25, verbose=False)

        found, total, times = 0, 0, []
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            total += 1
            t0 = time.time()
            results = model.predict(frame, conf=0.25, verbose=False)
            times.append(time.time() - t0)
            if len(results[0].boxes) > 0:
                found += 1
        cap.release()

        avg_ms = (sum(times) / len(times)) * 1000 if times else 0
        pct = 100 * found / total if total else 0
        print(f"  {video}: {found}/{total} frames ({pct:.1f}%) ball found, avg {avg_ms:.1f}ms/frame")


if __name__ == "__main__":
    main()
