"""
심판 PC 실시간 HIT 판정: 아이폰(Iriun Webcam)으로 Red/Blue AMR과 탁구공을 찾아 추적하면서,
AMR 히트박스 안의 모션 에너지가 튀는 순간을 골라내 HIT을 판정하고 rosbridge로 각 팀 PC/심판
프론트엔드에 알린다.

구성 요소 (모두 이 폴더의 기존 실험 결과를 그대로 재사용):
- zero_shot_track.py: Grounding DINO 제로샷 탐지(최초 1회/재탐지 시) + CSRT 추적으로
  탁구공 슈터가 달린 AMR 두 대의 박스를 찾는다. 팀 색은 텍스트로 구별이 안 되므로(카메라엔
  로봇 색이 안 보일 수 있음) 화면상 왼쪽=RED, 오른쪽=BLUE로 위치 기반 position-labels를
  사용한다 (경기장에서 팀이 항상 같은 쪽에서 시작한다는 가정).
- ball_track.py: 탁구공은 Grounding DINO로 매 프레임 찾기엔 너무 느리고(프레임당 8~9초)
  오탐도 있어서, 프레임 차분 기반 blob 추적을 그대로 쓴다.
- live_hit_detect.py / hit_spike_test.py: ROI(여기서는 AMR 박스) 안의 모션 에너지가
  최근 N프레임 대비 국소 피크를 찍으면 "맞았다"로 본다. 다만 AMR은 라켓과 달리 스스로
  계속 움직이므로, 에너지 피크만으로 판정하면 그냥 주행하는 것도 HIT으로 오판할 수 있다.
  그래서 "피크가 뜬 그 순간 공이 실제로 그 박스 근처에 있었는가"를 추가 조건으로 요구한다.

판정 결과 전송: roslibpy로 rosbridge(기본 ws://localhost:8000/ws, frontend .env와 동일)에
접속해 프론트엔드가 이미 구독 중인 스키마 그대로 /judge/hit_event(HitEvent)와
/judge/detections(Detections)를 publish한다. rosbridge가 없거나 --dry-run이면 콘솔에만 찍는다.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque

import cv2
import numpy as np

from ball_track import find_ball_candidates, pick_best
from zero_shot_track import load_model, run_detect

TEAM_COLORS = ("red", "blue")


def motion_energy(diff_gray: np.ndarray, box: tuple[float, float, float, float]) -> float:
    """box=(x,y,w,h) 안의 임계값 넘은 픽셀 수 — live_hit_detect.py의 motion_energy와 동일하되
    ROI가 프레임 밖으로 나가거나(트래커 드리프트) 면적이 0이 되는 경우를 방어한다."""
    x, y, w, h = box
    x1, y1 = max(0, int(x)), max(0, int(y))
    x2, y2 = min(diff_gray.shape[1], int(x + w)), min(diff_gray.shape[0], int(y + h))
    if x2 <= x1 or y2 <= y1:
        return 0.0
    region = diff_gray[y1:y2, x1:x2]
    _, thresh = cv2.threshold(region, 15, 255, cv2.THRESH_BINARY)
    return float(thresh.sum()) / 255.0


def team_of(label: str) -> str | None:
    lower = label.lower()
    if "red" in lower:
        return "red"
    if "blue" in lower:
        return "blue"
    return None


def expand_box(box: tuple[float, float, float, float], margin: float) -> tuple[float, float, float, float]:
    x, y, w, h = box
    return (x - margin, y - margin, w + 2 * margin, h + 2 * margin)


def point_in_box(pt: tuple[float, float] | None, box: tuple[float, float, float, float]) -> bool:
    if pt is None:
        return False
    x, y, w, h = box
    px, py = pt
    return x <= px <= x + w and y <= py <= y + h


class JudgeBridge:
    """roslibpy로 rosbridge에 /judge/hit_event, /judge/detections publish.

    입력: url(예: ws://localhost:8000/ws), dry_run(True면 접속 자체를 안 함).
    연결에 실패해도 예외를 던지지 않고 dry-run 모드로 넘어간다 — 비전 파이프라인 검증은
    rosbridge 없이도 계속할 수 있어야 하기 때문.
    """

    def __init__(self, url: str, dry_run: bool = False, connect_timeout: float = 5.0):
        self.dry_run = dry_run
        self.client = None
        self.hit_topic = None
        self.det_topic = None
        self.image_topic = None
        if self.dry_run:
            print("[judge_hit_detect] dry-run 모드 — rosbridge에 접속하지 않음", flush=True)
            return

        import roslibpy

        try:
            self.client = roslibpy.Ros(host=url, port=None)
            self.client.run(timeout=connect_timeout)
        except Exception as e:  # noqa: BLE001 — 접속 실패 사유가 다양해서(타임아웃/거부/DNS) 폭넓게 받는다
            print(f"[judge_hit_detect] rosbridge 접속 실패({e}) — dry-run으로 전환", file=sys.stderr, flush=True)
            self.dry_run = True
            self.client = None
            return

        self.hit_topic = roslibpy.Topic(self.client, "/judge/hit_event", "turtlebot4_wargame_msgs/HitEvent")
        self.det_topic = roslibpy.Topic(self.client, "/judge/detections", "turtlebot4_wargame_msgs/Detections")
        self.image_topic = roslibpy.Topic(self.client, "/judge/image_raw", "sensor_msgs/CompressedImage")
        print(f"[judge_hit_detect] rosbridge 연결됨: {url}", flush=True)

    def publish_hit(self, victim: str, timestamp: float) -> None:
        if self.dry_run:
            print(f"[dry-run] /judge/hit_event victim={victim} ts={timestamp:.3f}", flush=True)
            return
        import roslibpy

        self.hit_topic.publish(roslibpy.Message({"victim": victim, "timestamp": timestamp}))

    def publish_detections(self, detections: list[dict]) -> None:
        if self.dry_run:
            return
        import roslibpy

        self.det_topic.publish(roslibpy.Message({"detections": detections}))

    def publish_image(self, jpeg_bytes: bytes) -> None:
        if self.dry_run:
            return
        import base64

        import roslibpy

        b64 = base64.b64encode(jpeg_bytes).decode("ascii")
        self.image_topic.publish(roslibpy.Message({"format": "jpeg", "data": b64}))

    def close(self) -> None:
        if self.client is not None:
            self.client.terminate()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Red/Blue AMR + 공 탐지 -> AMR 히트박스 모션에너지 HIT 판정 -> rosbridge publish")
    parser.add_argument("--camera", default="0", help="local device index, stream URL, or video file path")
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)

    parser.add_argument("--dino-model", default="IDEA-Research/grounding-dino-tiny")
    parser.add_argument(
        "--amr-prompt", default="a turtlebot robot.",
        help='Grounding DINO 텍스트 프롬프트 — 실측 결과 "a robot with a ping pong ball shooter on top." '
             '같은 장황한 프롬프트는 카모(위장) 넷 덮인 장애물을 로봇보다 더 높은 confidence로 오탐했다. '
             '실제 플랫폼명을 넣은 "a turtlebot robot."이 카모 장애물과 뚜렷하게 분리됨(0.48 vs 0.31)',
    )
    parser.add_argument(
        "--position-labels", default="Red Team AMR,Blue Team AMR",
        help="쉼표로 구분된 라벨을 박스의 왼쪽->오른쪽 순서로 부여 — 기본값은 화면 기준 왼쪽=RED, 오른쪽=BLUE. "
             "경기장에서 팀이 항상 같은 쪽에서 시작한다고 가정 (색 자체는 카메라로 구별 안 되므로 위치로 판단)",
    )
    parser.add_argument(
        "--box-threshold", type=float, default=0.2,
        help='실측 아레나 사진 기준 카메라에서 먼 쪽/카모 벽 근처 로봇은 confidence가 0.15~0.2대까지 '
             '떨어지는 경우가 있어서 zero_shot_track.py의 기본값(0.35)보다 낮춤. 오탐이 늘면 올릴 것',
    )
    parser.add_argument("--text-threshold", type=float, default=0.2)
    parser.add_argument("--shrink-ratio", type=float, default=0.5, help="박스 면적이 최초 탐지 대비 이 비율 밑으로 줄면 추적 포기")
    parser.add_argument("--redetect-cooldown", type=float, default=1.5, help="AMR을 모두 놓쳤을 때 자동 재탐지 최소 간격(초)")

    parser.add_argument("--ball-min-area", type=float, default=3.0)
    parser.add_argument("--ball-max-area", type=float, default=400.0)
    parser.add_argument("--ball-max-jump", type=float, default=120.0, help="프레임 간 공 위치 허용 최대 이동거리(px)")
    parser.add_argument(
        "--ball-search-pad", type=float, default=120.0,
        help="공을 화면 전체가 아니라 각 AMR 박스에서 이 픽셀만큼 확장한 범위 안에서만 찾는다. "
             "Iriun처럼 압축 스트리밍되는 카메라는 화면 전체에서 찾으면 압축 노이즈를 공으로 "
             "오인하기 쉬워서, 어차피 HIT 판정에 필요한 범위(로봇 근처)로 검색을 좁힌다.",
    )

    parser.add_argument(
        "--hit-height", type=float, default=1000.0,
        help="이 모션 에너지 이상이어야 HIT 후보. 3000 -> 500(공이 작아서) -> 1000(실측 후 재조정)",
    )
    parser.add_argument("--hit-cooldown", type=int, default=15, help="한 팀이 HIT 판정된 후 최소 이 프레임 동안 같은 팀 재판정 금지")
    parser.add_argument("--ball-margin", type=float, default=25.0, help="공이 AMR 박스에서 이 픽셀 이내에 있어야 '근접'으로 인정")
    parser.add_argument(
        "--shake-ratio", type=float, default=2.5,
        help="카메라 흔들림(화면 전체가 같이 움직임) 오탐 방지용. 로봇 박스 안 에너지 밀도가 "
             "배경(박스 밖) 에너지 밀도의 이 배수 이상이어야 '박스 안에서만 튄' 진짜 스파이크로 "
             "인정한다 — 흔들림은 배경도 똑같이 움직이므로 밀도 비율이 1에 가까워서 걸러짐",
    )

    parser.add_argument("--rosbridge-url", default="ws://localhost:8000/ws")
    parser.add_argument("--dry-run", action="store_true", help="rosbridge에 접속하지 않고 콘솔에만 출력")
    parser.add_argument("--headless", action="store_true", help="화면 창 없이 실행 (모니터 없는 심판 PC/자동화 테스트용)")
    parser.add_argument("--loop", action="store_true", help="비디오 파일 소스일 때 끝나면 처음부터 반복 재생 (테스트용)")
    parser.add_argument("--publish-every", type=int, default=1, help="N프레임마다 한 번씩만 /judge/detections publish (기본 매 프레임)")

    parser.add_argument(
        "--publish-image", dest="publish_image", action="store_true", default=True,
        help="/judge/image_raw로 카메라 화면을 publish해서 심판 PC UI(JudgeCameraFeed)에 띄운다 (기본 켜짐)",
    )
    parser.add_argument("--no-publish-image", dest="publish_image", action="store_false", help="이미지 publish 끄기")
    parser.add_argument("--image-max-width", type=int, default=960, help="publish 전 이 너비로 축소(4K 원본을 그대로 보내면 너무 무거움)")
    parser.add_argument("--image-quality", type=int, default=75, help="JPEG 압축 품질(0~100)")
    parser.add_argument("--image-fps", type=float, default=8.0, help="이미지 publish 주기(초당 프레임) — 매 프레임 보내면 대역폭이 너무 큼")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    position_labels = [s.strip() for s in args.position_labels.split(",")] if args.position_labels else None

    print(f"loading Grounding DINO ({args.dino_model})...", flush=True)
    processor, model = load_model(args.dino_model)
    print("model loaded.", flush=True)

    bridge = JudgeBridge(args.rosbridge_url, dry_run=args.dry_run)

    source = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(source)
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"camera source {args.camera!r} could not be opened", file=sys.stderr)
        sys.exit(1)

    ok, raw_frame = cap.read()
    if not ok:
        print("failed to read first frame", file=sys.stderr)
        sys.exit(1)
    frame_h, frame_w = raw_frame.shape[:2]
    prev_gray = cv2.GaussianBlur(cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)

    targets: list[dict] = []  # AMR: [{"tracker","box","label","init_area","score"}, ...]
    had_detected_before = False
    last_redetect_attempt = 0.0

    last_ball: tuple[float, float] | None = None
    last_ball_radius = 6.0
    recent_ball: deque[tuple[int, tuple[float, float] | None]] = deque(maxlen=3)  # (frame_idx, pos)

    recent_energy = {c: deque(maxlen=3) for c in TEAM_COLORS}  # color -> deque[(frame_idx, energy)]
    recent_bg_density: deque[tuple[int, float]] = deque(maxlen=3)  # (frame_idx, 배경 에너지 밀도) — 카메라 흔들림 판단용
    frames_since_hit = {c: 999 for c in TEAM_COLORS}
    hit_flash = {c: 0 for c in TEAM_COLORS}
    spike_flash = {c: 0 for c in TEAM_COLORS}  # HIT으로 확정 안 된(공 근접 실패/흔들림) 스파이크 표시용
    spike_reason = {c: "" for c in TEAM_COLORS}  # 화면 표시용: "shake" 또는 "no ball"
    latest_energy = {c: 0.0 for c in TEAM_COLORS}  # 화면에 실시간 에너지 수치 표시용

    frame_idx = 0
    last_image_publish = 0.0
    window = "judge hit detect  [d]=re-detect AMR  [q]=quit"
    if not args.headless:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 1024, 576)

    def ball_at(idx: int) -> tuple[float, float] | None:
        for i, pos in recent_ball:
            if i == idx:
                return pos
        return None

    def bg_density_at(idx: int) -> float:
        for i, density in recent_bg_density:
            if i == idx:
                return density
        return 0.0

    def try_detect_amr(frame) -> None:
        nonlocal targets, had_detected_before, last_redetect_attempt
        print(f"detecting '{args.amr_prompt}'...", flush=True)
        t0 = time.time()
        targets, _ = run_detect(
            processor, model, frame, args.amr_prompt, args.box_threshold, args.text_threshold,
            len(position_labels) if position_labels else 2, position_labels,
        )
        print(f"  took {time.time() - t0:.2f}s, found {len(targets)} target(s)", flush=True)
        had_detected_before = True
        last_redetect_attempt = time.time()

    try_detect_amr(raw_frame)

    try:
        while True:
            ok, raw_frame = cap.read()
            if not ok:
                if args.loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, raw_frame = cap.read()
                    if not ok:
                        break
                    prev_gray = cv2.GaussianBlur(cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
                    continue
                break

            frame_idx += 1
            gray = cv2.GaussianBlur(cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            diff = cv2.absdiff(gray, prev_gray)
            prev_gray = gray

            # 1) AMR 추적 갱신 (CSRT) + 놓치면 쿨다운 지나고 자동 재탐지
            still_tracking = []
            for tgt in targets:
                ok_t, box = tgt["tracker"].update(raw_frame)
                if ok_t:
                    x, y, w, h = box
                    if (w * h) / tgt["init_area"] < args.shrink_ratio:
                        ok_t = False  # CSRT가 가려진 채로 쪼그라들며 드리프트하는 경우 방어
                if ok_t:
                    tgt["box"] = tuple(int(v) for v in box)
                    still_tracking.append(tgt)
            targets = still_tracking

            # 2) 공 추적 — 화면 전체가 아니라 AMR 히트박스 주변에서만 찾는다.
            # Iriun은 와이파이 압축 스트리밍이라 화면에 움직임이 있으면 프레임 전체에 공 크기와
            # 비슷한 압축 노이즈 얼룩이 수백 개씩 생겨서, 전체 프레임에서 찾으면 노이즈를 진짜
            # 공으로 잘못 고르기 쉽다. 어차피 HIT 판정엔 "공이 히트박스 근처에 있었는가"만
            # 필요하므로 검색 범위를 그 근처로 좁혀서 노이즈 후보 자체를 줄인다.
            candidates = []
            for tgt in targets:
                rx, ry, rw, rh = expand_box(tgt["box"], args.ball_search_pad)
                x1, y1 = max(0, int(rx)), max(0, int(ry))
                x2, y2 = min(frame_w, int(rx + rw)), min(frame_h, int(ry + rh))
                if x2 <= x1 or y2 <= y1:
                    continue
                candidates += find_ball_candidates(
                    diff, args.ball_min_area, args.ball_max_area, roi=(x1, y1, x2, y2),
                )
            best = pick_best(candidates, last_ball)
            if best is not None:
                bx, by, br, _area = best
                if last_ball is not None:
                    dist = ((bx - last_ball[0]) ** 2 + (by - last_ball[1]) ** 2) ** 0.5
                    if dist > args.ball_max_jump:
                        best = None
            if best is not None:
                bx, by, br, _area = best
                last_ball = (bx, by)
                last_ball_radius = br
            else:
                last_ball = None
            recent_ball.append((frame_idx, last_ball))

            if not targets and had_detected_before and (time.time() - last_redetect_attempt) > args.redetect_cooldown:
                try_detect_amr(raw_frame)

            # 3) 팀별 히트박스 모션 에너지 -> 국소 피크 + 공 근접 + "박스 안에서만 튀었는가" 조건으로 HIT 판정
            box_energies = {}
            for tgt in targets:
                color = team_of(tgt["label"])
                if color is not None:
                    box_energies[color] = motion_energy(diff, tgt["box"])

            # 배경(로봇 박스 밖) 에너지 밀도 — 카메라가 흔들리면 배경도 로봇 박스만큼 같이
            # 움직여서 에너지가 튀므로, 이 밀도와 비교해 "박스 안에서만 튄" 진짜 스파이크인지 가른다.
            global_e = motion_energy(diff, (0, 0, frame_w, frame_h))
            box_area_sum = sum(tgt["box"][2] * tgt["box"][3] for tgt in targets if team_of(tgt["label"]))
            bg_area = max(1.0, frame_w * frame_h - box_area_sum)
            bg_density = max(1e-6, (global_e - sum(box_energies.values())) / bg_area)
            recent_bg_density.append((frame_idx, bg_density))

            detections = []
            for tgt in targets:
                color = team_of(tgt["label"])
                if color is None:
                    continue
                box = tgt["box"]
                e = box_energies[color]
                latest_energy[color] = e
                recent_energy[color].append((frame_idx, e))
                frames_since_hit[color] += 1

                hist = recent_energy[color]
                if len(hist) == 3:
                    (_, e_prev2), (mid_idx, e_mid), (_, e_now) = hist
                    is_local_peak = e_mid >= e_prev2 and e_mid >= e_now and e_mid >= args.hit_height
                    if is_local_peak and frames_since_hit[color] >= args.hit_cooldown:
                        box_density = e_mid / max(1.0, box[2] * box[3])
                        is_localized = box_density >= bg_density_at(mid_idx) * args.shake_ratio
                        near_box = expand_box(box, args.ball_margin)
                        ball_pos = ball_at(mid_idx)
                        if not is_localized:
                            # 화면엔 안 띄움(흔들림은 거의 매 프레임 걸릴 수 있어서 화면이 계속
                            # 깜빡이면 오히려 방해됨) — 콘솔 로그로만 남긴다. 쿨다운도 미소비.
                            print(
                                f"frame {mid_idx}: spike ignored for {color} — looks like camera shake "
                                f"(box density={box_density:.2f}, bg density={bg_density_at(mid_idx):.2f})",
                                flush=True,
                            )
                        elif point_in_box(ball_pos, near_box):
                            frames_since_hit[color] = 0
                            hit_flash[color] = 8
                            ts = time.time()
                            print(f"frame {mid_idx}: HIT {color.upper()} (energy={e_mid:.0f}, ball={ball_pos})", flush=True)
                            bridge.publish_hit(color, ts)
                        else:
                            spike_flash[color] = 8
                            spike_reason[color] = "no ball"
                            print(
                                f"frame {mid_idx}: motion spike ignored for {color} "
                                f"(energy={e_mid:.0f}, ball not nearby: {ball_pos})",
                                flush=True,
                            )
                            # 쿨다운은 소비하지 않음 — 오탐(자체 주행 등)이 다음 진짜 HIT을 막지 않도록

                x, y, w, h = box
                detections.append({
                    "x": max(0.0, x / frame_w), "y": max(0.0, y / frame_h),
                    "w": min(1.0, w / frame_w), "h": min(1.0, h / frame_h),
                    "conf": float(tgt.get("score", 0.8)),
                    "label": f"robot_{color}",
                })

            if last_ball is not None:
                bx, by = last_ball
                br = last_ball_radius
                detections.append({
                    "x": max(0.0, (bx - br) / frame_w), "y": max(0.0, (by - br) / frame_h),
                    "w": min(1.0, 2 * br / frame_w), "h": min(1.0, 2 * br / frame_h),
                    "conf": 1.0, "label": "ball",
                })

            if frame_idx % max(1, args.publish_every) == 0:
                bridge.publish_detections(detections)

            if args.publish_image and (time.time() - last_image_publish) >= (1.0 / max(0.1, args.image_fps)):
                last_image_publish = time.time()
                scale = min(1.0, args.image_max_width / frame_w)
                pub_frame = (
                    cv2.resize(raw_frame, (int(frame_w * scale), int(frame_h * scale)))
                    if scale < 1.0 else raw_frame
                )
                ok_enc, buf = cv2.imencode(".jpg", pub_frame, [cv2.IMWRITE_JPEG_QUALITY, args.image_quality])
                if ok_enc:
                    bridge.publish_image(buf.tobytes())

            if not args.headless:
                display = raw_frame.copy()
                for tgt in targets:
                    color = team_of(tgt["label"])
                    box_color = (0, 0, 255) if color == "red" else (255, 0, 0) if color == "blue" else (0, 255, 0)
                    x, y, w, h = tgt["box"]
                    is_spiking = color and spike_flash.get(color, 0) > 0
                    is_hit = color and hit_flash.get(color, 0) > 0
                    thick = 4 if is_hit else 3 if is_spiking else 2
                    outline_color = (0, 255, 255) if is_spiking and not is_hit else box_color
                    cv2.rectangle(display, (x, y), (x + w, y + h), outline_color, thick)
                    cv2.putText(display, tgt["label"], (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                    if color:
                        # 매 프레임 현재 에너지 수치를 박스 밑에 표시 — hit-height 임계값 넘겼는지 색으로 구분
                        e_now = latest_energy[color]
                        e_color = (0, 255, 255) if e_now >= args.hit_height else (200, 200, 200)
                        cv2.putText(
                            display, f"energy={e_now:.0f}/{args.hit_height:.0f}", (x, y + h + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, e_color, 1,
                        )
                    if is_hit:
                        cv2.putText(display, "HIT!", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, box_color, 3)
                    elif is_spiking:
                        cv2.putText(display, f"spike ({spike_reason.get(color, '')})", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                if last_ball is not None:
                    cv2.circle(display, (int(last_ball[0]), int(last_ball[1])), int(last_ball_radius) + 3, (0, 255, 255), 2)
                if not targets:
                    cv2.putText(display, "no AMR tracked - press 'd'", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(display, f"frame {frame_idx}", (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.imshow(window, display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("d"):
                    try_detect_amr(raw_frame)

            for c in TEAM_COLORS:
                if hit_flash[c] > 0:
                    hit_flash[c] -= 1
                if spike_flash[c] > 0:
                    spike_flash[c] -= 1
    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
        bridge.close()


if __name__ == "__main__":
    main()
