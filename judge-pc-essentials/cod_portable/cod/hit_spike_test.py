"""
"명중 판정 = 타겟 근처 모션 스파이크" 방법론 검증용.
test4.webm에서 오른쪽 선수(=실제 게임의 enemy 로봇 위치에 해당하는 타겟)쪽 ROI를 잡고,
프레임 차분 에너지(모션량)의 시계열에서 갑자기 튀는 지점(스파이크)을 찾는다.
그 스파이크가 실제로 라켓-공 접촉 순간과 일치하는지 프레임을 뽑아 눈으로 검증한다.
"""

from __future__ import annotations

import cv2
import numpy as np

VIDEO = "test_videos/test4.webm"
ROI = (940, 100, 1100, 230)  # 오른쪽 선수의 라켓/접촉 지점 근처로 좁힘 (몸통 전체 X)


def find_local_peaks(values: np.ndarray, height: float, distance: int) -> list[int]:
    """입력: 시계열, 최소 높이, 피크 간 최소 프레임 간격. 출력: 국소 최대값 인덱스 목록(높은 순 우선 채택)."""
    candidates = [i for i in range(1, len(values) - 1) if values[i] >= height and values[i] >= values[i - 1] and values[i] >= values[i + 1]]
    candidates.sort(key=lambda i: -values[i])
    chosen: list[int] = []
    for i in candidates:
        if all(abs(i - c) >= distance for c in chosen):
            chosen.append(i)
    return sorted(chosen)


def motion_energy(diff_gray: np.ndarray, roi: tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = roi
    region = diff_gray[y1:y2, x1:x2]
    _, thresh = cv2.threshold(region, 15, 255, cv2.THRESH_BINARY)
    return float(thresh.sum()) / 255.0  # 임계값 넘은 픽셀 수


def main() -> None:
    cap = cv2.VideoCapture(VIDEO)
    ok, prev = cap.read()
    prev_gray = cv2.GaussianBlur(cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY), (5, 5), 0)

    energies = []
    frames = [prev]
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        i += 1
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        diff = cv2.absdiff(gray, prev_gray)
        prev_gray = gray
        energies.append(motion_energy(diff, ROI))
        frames.append(frame)
    cap.release()

    energies = np.array(energies)
    baseline = np.median(energies)

    # 랠리 중엔 리턴할 때마다 비슷한 크기의 봉우리가 반복되므로, 전역 극단치가 아니라
    # 국소 피크(주변보다 높고, 최소 높이/간격 조건을 만족하는 지점)를 찾는다.
    peaks = find_local_peaks(energies, height=3000, distance=15)

    print(f"total frames analyzed: {len(energies)}")
    print(f"baseline motion energy: {baseline:.1f}")
    print(f"detected {len(peaks)} peak(s) (candidate hits):")
    events = [[p] for p in peaks]
    for ev in events:
        peak = ev[0]
        print(f"  frame {peak} (energy={energies[peak]:.0f})")
        frame_idx = peak + 1  # frames[0]=prev(첫 프레임), energies[k]는 frames[k+1]에 대응
        vis = frames[frame_idx].copy()
        cv2.rectangle(vis, (ROI[0], ROI[1]), (ROI[2], ROI[3]), (0, 0, 255), 2)
        cv2.putText(vis, f"PEAK frame {peak} energy={energies[peak]:.0f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imwrite(f"test_videos/frames/peak_{peak}.jpg", vis)

    # 참고용: 에너지 시계열 그래프 저장
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.figure(figsize=(14, 4))
    plt.plot(energies, label="motion energy in ROI")
    plt.axhline(baseline, color="gray", linestyle="--", label="baseline")
    for peak in peaks:
        plt.axvline(peak, color="red", alpha=0.5)
    plt.legend()
    plt.xlabel("frame")
    plt.ylabel("motion energy (thresholded diff pixel count)")
    plt.tight_layout()
    plt.savefig("test_videos/frames/motion_energy_plot.png")
    print("saved motion_energy_plot.png and peak_*.jpg")


if __name__ == "__main__":
    main()
