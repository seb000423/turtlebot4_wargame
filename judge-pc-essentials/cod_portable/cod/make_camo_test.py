"""
터틀봇 사진의 흰 배경을 제거하고, 로봇 표면 + 배경 전체에 동일한 디지털 카모(픽셀 블록)
패턴을 합성한다. 배경까지 같은 패턴이어야 "카모가 배경과 텍스처를 섞어서 경계를 지운다"는
실제 위장 효과를 재현할 수 있다 (로봇만 카모를 입히고 배경이 흰색이면 대비가 커서
탐지가 오히려 쉬워짐).
"""

from __future__ import annotations

import cv2
import numpy as np

SRC = "test_images/turtlebot3_burger.jpg"
OUT = "test_images/turtlebot3_burger_camo.jpg"

PALETTE = [
    (58, 71, 58),
    (101, 101, 66),
    (46, 54, 34),
    (150, 140, 110),
    (20, 20, 20),
]


def generate_camo_pattern(h: int, w: int, block: int = 18, seed: int = 42) -> np.ndarray:
    """입력: 목표 높이/너비, 블록 크기(px). 출력: (h,w,3) BGR 디지털 카모 패턴."""
    rng = np.random.default_rng(seed)
    ph, pw = h // block + 2, w // block + 2
    idx = rng.integers(0, len(PALETTE), size=(ph, pw))
    pattern = np.zeros((ph * block, pw * block, 3), dtype=np.uint8)
    for i in range(ph):
        for j in range(pw):
            pattern[i * block : (i + 1) * block, j * block : (j + 1) * block] = PALETTE[idx[i, j]]
    return pattern[:h, :w]


def main() -> None:
    frame = cv2.imread(SRC)
    h, w = frame.shape[:2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    background_mask = gray > 240  # 원본은 거의 순백 배경
    robot_mask = ~background_mask

    camo = generate_camo_pattern(h, w)

    alpha = 0.75  # 로봇 표면에서 원본 대비 카모 색을 얼마나 강하게 덮을지
    blended_robot = (frame.astype(np.float32) * (1 - alpha) + camo.astype(np.float32) * alpha).astype(np.uint8)

    result = camo.copy()
    result[robot_mask] = blended_robot[robot_mask]

    cv2.imwrite(OUT, result)
    print(f"saved {OUT}  (robot px: {robot_mask.sum()}, total px: {h*w})")


if __name__ == "__main__":
    main()
