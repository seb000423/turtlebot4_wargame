"""
실시간 명중 판정: 영상을 (라이브 카메라처럼) 재생하면서, 미래 프레임을 미리 보지 않고
그 순간까지의 정보만으로 매 프레임 "맞았다/안 맞았다"를 즉석 판정한다.

방식: 타겟(라켓/enemy 로봇) ROI 안의 프레임 차분 에너지를 실시간으로 계산하고,
최근 N프레임의 롤링 중앙값을 기준선으로 삼아 그보다 확 튀는 국소 피크가 나오면
"HIT"으로 표시한다. 피크 확인에 1프레임 정도의 지연만 허용(직전 프레임이 그 앞뒤보다
높은지 확인) — 전체 영상을 미리 다 보고 계산하는 이전 방식과 달리 실시간 스트리밍에도
그대로 쓸 수 있는 구조.
"""

from __future__ import annotations

import argparse
from collections import deque

import cv2
import numpy as np

ROI = (940, 100, 1100, 230)  # 타겟(라켓) 근처 — 실제 게임에서는 enemy 로봇 박스로 교체


def motion_energy(diff_gray: np.ndarray, roi: tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = roi
    region = diff_gray[y1:y2, x1:x2]
    _, thresh = cv2.threshold(region, 15, 255, cv2.THRESH_BINARY)
    return float(thresh.sum()) / 255.0


def main() -> None:
    parser = argparse.ArgumentParser(description="실시간 명중(모션 피크) 판정 라이브 데모")
    parser.add_argument("--video", default="test_videos/test4.webm")
    parser.add_argument("--height", type=float, default=3000.0, help="이 에너지 이상이어야 후보")
    parser.add_argument("--cooldown", type=int, default=15, help="한 번 HIT 판정 후 최소 이 프레임 동안 재판정 금지")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.video)
    ok, prev = cap.read()
    prev_gray = cv2.GaussianBlur(cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY), (5, 5), 0)

    recent3: deque[tuple[int, float]] = deque(maxlen=3)  # (frame_idx, energy) 최근 3개 — 국소피크 판정용
    hit_count = 0
    frames_since_hit = 999
    hit_flash = 0  # 화면에 "HIT!" 표시를 몇 프레임 더 보여줄지
    frame_idx = 0

    window = "real-time hit detection  [q]=quit"
    cv2.namedWindow(window)

    while True:
        ok, frame = cap.read()
        if not ok:
            cap.release()
            cap = cv2.VideoCapture(args.video)  # 루프 재생
            ok, prev = cap.read()
            prev_gray = cv2.GaussianBlur(cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            recent3.clear()
            frame_idx = 0
            continue

        frame_idx += 1
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        diff = cv2.absdiff(gray, prev_gray)
        prev_gray = gray

        e = motion_energy(diff, ROI)
        recent3.append((frame_idx, e))
        frames_since_hit += 1

        # 국소 피크 판정: 가운데(직전 프레임)이 양옆보다 높고, 최소 높이 이상, 쿨다운 지났을 때
        # → 딱 1프레임 지연만으로 실시간 처리 가능 (미래를 길게 내다보지 않음)
        if len(recent3) == 3:
            (_, e_prev2), (mid_idx, e_mid), (_, e_now) = recent3
            is_local_peak = e_mid >= e_prev2 and e_mid >= e_now and e_mid >= args.height
            if is_local_peak and frames_since_hit >= args.cooldown:
                hit_count += 1
                frames_since_hit = 0
                hit_flash = 8
                print(f"frame {mid_idx}: HIT #{hit_count} (energy={e_mid:.0f})")

        display = frame.copy()
        cv2.rectangle(display, (ROI[0], ROI[1]), (ROI[2], ROI[3]),
                      (0, 0, 255) if hit_flash > 0 else (0, 255, 0), 3 if hit_flash > 0 else 1)
        cv2.putText(display, f"frame {frame_idx}  energy={e:.0f}  hits={hit_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if hit_flash > 0:
            cv2.putText(display, "HIT!", (ROI[0], ROI[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            hit_flash -= 1

        cv2.imshow(window, display)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
