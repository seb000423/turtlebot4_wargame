"""
탁구공 추적 (프레임 차분 기반). 그라운딩 디노는 프레임당 8~9초나 걸리고, 정지 프레임에서
광고판 텍스트를 공으로 오탐하는 경우도 있어서 이 용도엔 안 맞음. 공은 테이블/배경과 명암
대비가 항상 크진 않지만(위치에 따라 밝은 테이블 vs 어두운 광고판 배경), 화면에서 가장 빠르게
움직이는 작은 물체라는 점은 항상 성립 — 그래서 정적 색상 임계값 대신 연속 프레임 차분으로
움직이는 작고 둥근 blob을 찾는다.
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np


def find_ball_candidates(
    diff_gray: np.ndarray,
    min_area: float,
    max_area: float,
    min_circularity: float = 0.35,
    roi: tuple[int, int, int, int] | None = None,
) -> list[tuple[float, float, float, float]]:
    """입력: 두 프레임의 grayscale 차분, roi=(x1,y1,x2,y2) 있으면 그 밖은 무시.
    출력: (x,y,radius,area) 후보 목록 (필터 통과한 것만)."""
    if roi is not None:
        x1, y1, x2, y2 = roi
        masked = np.zeros_like(diff_gray)
        masked[y1:y2, x1:x2] = diff_gray[y1:y2, x1:x2]
        diff_gray = masked

    _, thresh = cv2.threshold(diff_gray, 18, 255, cv2.THRESH_BINARY)
    thresh = cv2.dilate(thresh, None, iterations=1)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (min_area <= area <= max_area):
            continue
        (x, y), radius = cv2.minEnclosingCircle(c)
        circularity = area / (np.pi * radius * radius + 1e-6)
        if circularity < min_circularity:
            continue
        candidates.append((x, y, radius, area))
    return candidates


EXPECTED_BALL_AREA = 90.0  # 관찰된 공 지름(~12px) 기준 대략적 면적 — 선수 몸통 등 큰 blob 배제용


def pick_best(candidates, last_pos, expected_area: float = EXPECTED_BALL_AREA):
    """입력: 후보 목록, 직전 프레임 공 위치(x,y) 또는 None. 출력: 최적 후보 1개 또는 None.
    공 크기/원형에 얼마나 가까운지를 기본 점수로 삼고, 직전 위치가 있으면 거리 페널티를 더한다
    — '가장 큰 blob'을 고르면 선수 몸통 움직임에 바로 붙어버리는 문제가 있어서 크기 기준으로 바꿈."""
    if not candidates:
        return None

    def size_penalty(area, radius):
        circularity = area / (np.pi * radius * radius + 1e-6)
        return abs(area - expected_area) / expected_area + (1.0 - circularity)

    if last_pos is None:
        return min(candidates, key=lambda c: size_penalty(c[3], c[2]))

    lx, ly = last_pos

    def score(c):
        x, y, r, area = c
        dist = ((x - lx) ** 2 + (y - ly) ** 2) ** 0.5
        return size_penalty(area, r) + dist / 40.0  # 거리 페널티 가중치는 경험적으로 조정

    return min(candidates, key=score)


def main() -> None:
    parser = argparse.ArgumentParser(description="Frame-difference based ping pong ball tracker")
    parser.add_argument("--video", required=True)
    parser.add_argument("--min-area", type=float, default=3.0)
    parser.add_argument("--max-area", type=float, default=400.0)
    parser.add_argument("--max-jump", type=float, default=120.0, help="프레임 간 허용 최대 이동거리(px)")
    parser.add_argument("--trail", type=int, default=40, help="화면에 남길 궤적 점 개수")
    parser.add_argument(
        "--roi", default=None, help="x1,y1,x2,y2 — 이 영역만 탐색(배경 관중 등 오탐 방지). 생략 시 전체 프레임"
    )
    args = parser.parse_args()
    roi = tuple(int(v) for v in args.roi.split(",")) if args.roi else None

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"could not open {args.video}", file=sys.stderr)
        sys.exit(1)

    ok, frame = cap.read()
    if not ok:
        print("empty video", file=sys.stderr)
        sys.exit(1)
    prev_gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)

    last_ball = None
    trail: list[tuple[int, int]] = []
    paused = False
    window = "ball tracking (frame-diff)  [space]=pause  [q]=quit"
    cv2.namedWindow(window)

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                print("video ended", flush=True)
                break
            gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            diff = cv2.absdiff(gray, prev_gray)
            prev_gray = gray

            candidates = find_ball_candidates(diff, args.min_area, args.max_area, roi=roi)
            best = pick_best(candidates, last_ball)
            if best is not None:
                x, y, r, area = best
                if last_ball is not None:
                    dist = ((x - last_ball[0]) ** 2 + (y - last_ball[1]) ** 2) ** 0.5
                    if dist > args.max_jump:
                        best = None
            if best is not None:
                x, y, r, area = best
                last_ball = (x, y)
                trail.append((int(x), int(y)))
                if len(trail) > args.trail:
                    trail.pop(0)
            else:
                last_ball = None

        display = frame.copy()
        if roi is not None:
            cv2.rectangle(display, (roi[0], roi[1]), (roi[2], roi[3]), (255, 0, 0), 1)
        for pt in trail:
            cv2.circle(display, pt, 2, (0, 255, 255), -1)
        if last_ball is not None:
            cv2.circle(display, (int(last_ball[0]), int(last_ball[1])), 10, (0, 0, 255), 2)
            cv2.putText(
                display, f"({int(last_ball[0])},{int(last_ball[1])})",
                (int(last_ball[0]) + 12, int(last_ball[1])),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
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
