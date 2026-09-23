"""
웹캠(아이폰 Iriun 등) -> Grounding DINO 제로샷 탐지(최초 1회/재요청 시) -> CSRT 트래커로
움직임 추적. 학습 클래스 없이 텍스트 프롬프트만으로 처음 보는 물체를 찾고, 이후에는
가벼운 CSRT 트래커로 실시간 추적을 유지한다 (CPU 환경에서 매 프레임 제로샷 추론은
너무 느려서 채택하지 않음).
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import torch
from PIL import Image
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def load_model(model_id: str):
    processor = AutoProcessor.from_pretrained(model_id)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id)
    model.eval()
    model.to(DEVICE)
    return processor, model


def detect(processor, model, frame_bgr, prompt: str, box_threshold: float, text_threshold: float):
    """입력: BGR 프레임, 텍스트 프롬프트. 출력: 임계값 넘는 [((x,y,w,h), score, label), ...] 전부, confidence 내림차순."""
    image = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    text = prompt.strip()
    if not text.endswith("."):
        text += "."
    inputs = processor(images=image, text=text, return_tensors="pt").to(DEVICE)
    with torch.no_grad():
        outputs = model(**inputs)
    results = processor.post_process_grounded_object_detection(
        outputs,
        inputs.input_ids,
        threshold=box_threshold,
        text_threshold=text_threshold,
        target_sizes=[image.size[::-1]],
    )[0]
    detections = []
    for i in range(len(results["boxes"])):
        x1, y1, x2, y2 = results["boxes"][i].tolist()
        score = float(results["scores"][i])
        label = results["labels"][i]
        detections.append(((x1, y1, x2 - x1, y2 - y1), score, label))
    detections.sort(key=lambda d: -d[1])
    return detections


def make_tracker():
    # opencv-contrib-python 5.0.0부터 CSRT가 cv2/cv2.legacy 양쪽에서 완전히 빠짐
    # (TrackerDaSiamRPN/Nano/Vit는 별도 onnx 모델 다운로드가 필요해서, 추가 다운로드 없이
    # 바로 쓸 수 있는 MIL로 대체 — CSRT보다는 약하지만 shrink-ratio 놓침 판정으로 보완함).
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        return cv2.legacy.TrackerCSRT_create()
    return cv2.TrackerMIL_create()


def _overlap_ratio(box_a, box_b):
    """겹친 면적 / 두 박스 중 더 작은 쪽 면적. 표준 IoU는 한 박스가 다른 박스 안에 완전히
    포함되는 '같은 물체를 다른 스케일로 중복 탐지'한 경우를 못 잡는다 (작은 박스가 큰 박스에
    통째로 들어가 있어도 큰 박스 면적 때문에 IoU 자체는 낮게 나옴) — 그래서 작은 쪽 면적 기준
    포함 비율을 쓴다."""
    ax, ay, aw, ah = box_a
    bx, by, bw, bh = box_b
    inter_x1, inter_y1 = max(ax, bx), max(ay, by)
    inter_x2, inter_y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    inter = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)
    smaller_area = min(aw * ah, bw * bh)
    return inter / smaller_area if smaller_area > 0 else 0.0


def _suppress_overlapping(dets, overlap_threshold=0.6):
    """confidence 내림차순 dets에서 겹치는 박스는 가장 높은 confidence 것만 남긴다.

    Grounding DINO는 같은 물체를 여러 스케일의 겹치는 박스로 중복 탐지하는 경우가 흔해서,
    NMS 없이 그냥 top-N만 자르면 '서로 다른 물체 N개'가 아니라 '같은 물체의 중복 박스 N개'가
    뽑힐 수 있다 (실측: 카메라에 가까운 로봇 1대만 있어도 겹치는 박스 2개가 top-2를 차지해서,
    실제로 존재하는 먼 쪽 로봇이 threshold를 낮춰도 전혀 안 뽑히는 문제가 있었음).
    """
    kept = []
    for box, score, label in dets:
        if all(_overlap_ratio(box, k[0]) < overlap_threshold for k in kept):
            kept.append((box, score, label))
    return kept


def run_detect(processor, model, frame, prompt, box_threshold, text_threshold, max_targets, position_labels=None):
    """탐지 실행 후 새 CSRT 타겟 목록을 만들어 반환 (없으면 빈 리스트).

    position_labels가 주어지면(예: ["Red Team AMR", "Blue Team AMR"]), Grounding DINO의
    텍스트 라벨 대신 박스의 x좌표(왼쪽->오른쪽) 순서로 이 라벨들을 붙인다 — "레드팀/블루팀"은
    화면에 색으로 드러나지 않는 게임 로직상의 구분이라 텍스트 프롬프트로는 구별이 안 되고,
    이 경기장에서는 팀이 항상 같은 쪽에서 시작하므로 위치 기반이 실용적인 대안이다.
    """
    all_dets = detect(processor, model, frame, prompt, box_threshold, text_threshold)
    deduped = _suppress_overlapping(all_dets)  # 같은 물체의 중복 박스 제거 후 top-N
    dets = deduped[:max_targets]  # confidence 상위 N개까지만 (이미 내림차순 정렬됨)
    if not dets:
        print("  not found", flush=True)
        return [], all_dets
    print(f"  found {len(dets)} instance(s)", flush=True)

    if position_labels:
        dets = sorted(dets, key=lambda d: d[0][0])  # x좌표 오름차순 (왼쪽부터)

    targets = []
    for i, ((x, y, w, h), score, label) in enumerate(dets):
        if position_labels and i < len(position_labels):
            label = position_labels[i]
        print(f"    '{label}' conf={score:.2f} box=({x:.0f},{y:.0f},{w:.0f},{h:.0f})", flush=True)
        box = (int(x), int(y), int(w), int(h))
        t = make_tracker()
        t.init(frame, box)
        targets.append({"tracker": t, "box": box, "label": label, "init_area": max(1.0, w * h), "score": score})
    return targets, all_dets


def run_detect_split(processor, model, frame, prompt, box_threshold, text_threshold, position_labels, overlap_ratio=0.15):
    """전체 프레임에서 top-N을 한 번에 뽑는 run_detect 대신, 화면을 좌/우 절반으로 나눠
    각 절반에서 독립적으로 최고 confidence 후보 하나씩만 뽑는다.

    카메라에서 먼 쪽 로봇(화면에서 작게 잡히고 카모 배경과 겹침)은 confidence가 낮아서,
    화면 전체에서 top-N을 뽑으면 가까운 쪽의 카모 장애물 오탐(그것도 나름 그럴듯한
    confidence를 받음)에 밀려 아예 안 뽑히는 문제가 실측으로 확인됨. 게다가 절반으로 잘라
    각각 확대 효과를 주면(같은 물체가 상대적으로 더 크게 보임) Grounding DINO의 confidence
    자체도 올라간다 (실측: 크롭 전 0.15~0.2 -> 크롭 후 0.3대).

    "레드팀은 항상 왼쪽에서, 블루팀은 항상 오른쪽에서 시작한다"는 기존 position_labels 가정을
    검색 단계에도 그대로 적용 — position_labels[0]은 왼쪽 절반, position_labels[1]은 오른쪽
    절반에서 찾는다. 3팀 이상은 지원하지 않음(이 경기 규칙상 2팀 고정이라 충분).
    """
    h, w = frame.shape[:2]
    half = w // 2
    pad = int(half * overlap_ratio)
    x_ranges = [(0, min(w, half + pad)), (max(0, half - pad), w)]

    targets = []
    for (x1, x2), label in zip(x_ranges, position_labels):
        crop = frame[:, x1:x2]
        dets = _suppress_overlapping(detect(processor, model, crop, prompt, box_threshold, text_threshold))
        if not dets:
            continue
        (cx, cy, cw, ch), score, _ = dets[0]
        box = (int(cx + x1), int(cy), int(cw), int(ch))
        print(f"    '{label}' conf={score:.2f} box={box}", flush=True)
        t = make_tracker()
        t.init(frame, box)
        targets.append({"tracker": t, "box": box, "label": label, "init_area": max(1.0, cw * ch), "score": score})

    print(f"  found {len(targets)} instance(s)" if targets else "  not found", flush=True)
    return targets, []


def main() -> None:
    parser = argparse.ArgumentParser(description="Grounding DINO zero-shot detect once, then CSRT-track as it moves")
    parser.add_argument("--camera", default="0", help="local device index or stream URL")
    parser.add_argument("--prompt", required=True, help='text prompt, e.g. "a ping pong ball"')
    parser.add_argument(
        "--dino-model", default="IDEA-Research/grounding-dino-tiny",
        help="tiny(빠름, 부정확) 또는 IDEA-Research/grounding-dino-base(느리지만 더 정확)",
    )
    parser.add_argument("--box-threshold", type=float, default=0.35)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument(
        "--show-candidates", action="store_true",
        help="max-targets로 자르기 전, threshold를 넘은 모든 후보를 노란색으로 같이 그려서 프롬프트/threshold 튜닝에 사용",
    )
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--max-targets", type=int, default=2, help="confidence 상위 몇 개까지만 추적할지")
    parser.add_argument(
        "--position-labels", default=None,
        help='쉼표로 구분된 라벨을 박스의 왼쪽->오른쪽 순서로 붙임, 예: "Red Team AMR,Blue Team AMR"',
    )
    parser.add_argument(
        "--shrink-ratio", type=float, default=0.5,
        help="박스 면적이 최초 탐지 대비 이 비율 밑으로 줄면 '가려져서 드리프트'로 보고 추적 포기 (CSRT는 ok=True를 유지한 채로 드리프트하는 경우가 많아서 필요)",
    )
    parser.add_argument(
        "--redetect-cooldown", type=float, default=1.5,
        help="모든 타겟을 놓쳤을 때 자동 재탐지를 시도할 최소 간격(초) — 매 프레임 재탐지하면 무거워서 제한",
    )
    args = parser.parse_args()
    position_labels = args.position_labels.split(",") if args.position_labels else None

    print(f"loading Grounding DINO ({args.dino_model}, first run downloads weights)...", flush=True)
    processor, model = load_model(args.dino_model)
    print("model loaded.", flush=True)
    all_candidates: list = []  # [((x,y,w,h), score, label), ...] — --show-candidates 디버그용

    source = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(source)
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"camera source {args.camera!r} could not be opened", file=sys.stderr)
        sys.exit(1)

    targets: list[dict] = []  # [{"tracker":..., "box":(x,y,w,h), "label":str, "init_area":float}, ...]
    editing_prompt = False
    prompt_buffer = ""
    paused = False
    raw_frame = None
    had_detected_before = False
    last_redetect_attempt = 0.0
    window = "zero-shot detect + track  [d]=detect  [p]=prompt  [space]=pause  [q]=quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1024, 576)

    try:
        while True:
            if not paused or raw_frame is None:
                ok, raw_frame = cap.read()
                if not ok:
                    print("frame grab failed (video ended?)", file=sys.stderr)
                    break
                new_frame = True
            else:
                new_frame = False

            display = raw_frame.copy()
            key = cv2.waitKey(1) & 0xFF

            if key == ord(" ") and not editing_prompt:
                paused = not paused

            if editing_prompt:
                if key == 13 or key == 10:  # enter: confirm
                    if prompt_buffer.strip():
                        args.prompt = prompt_buffer.strip()
                        print(f"prompt set to '{args.prompt}'", flush=True)
                    editing_prompt = False
                elif key == 27:  # esc: cancel
                    editing_prompt = False
                elif key == 8 or key == 127:  # backspace
                    prompt_buffer = prompt_buffer[:-1]
                elif 32 <= key <= 126:  # printable ascii
                    prompt_buffer += chr(key)
                cv2.putText(
                    display, f"new prompt: {prompt_buffer}_", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
                )
                cv2.imshow(window, display)
                continue

            if key == ord("q"):
                break

            if key == ord("p"):
                editing_prompt = True
                prompt_buffer = ""
                continue

            if key == ord("d"):
                print(f"detecting '{args.prompt}'...", flush=True)
                t0 = time.time()
                targets, all_candidates = run_detect(
                    processor, model, raw_frame, args.prompt, args.box_threshold, args.text_threshold,
                    args.max_targets, position_labels,
                )
                print(f"  took {time.time() - t0:.2f}s", flush=True)
                had_detected_before = True
                last_redetect_attempt = time.time()

            if new_frame:
                still_tracking = []
                for tgt in targets:
                    ok_t, box = tgt["tracker"].update(raw_frame)
                    if ok_t:
                        x, y, w, h = box
                        area_ratio = (w * h) / tgt["init_area"]
                        if area_ratio < args.shrink_ratio:
                            # CSRT는 가려진 부분을 억지로 따라가며 박스가 쪼그라들어도
                            # ok=True를 유지하는 경우가 많아서, 면적 급감을 별도로 "놓침"으로 취급
                            ok_t = False
                    if ok_t:
                        tgt["box"] = tuple(int(v) for v in box)
                        still_tracking.append(tgt)
                targets = still_tracking

                if (
                    not targets
                    and had_detected_before
                    and (time.time() - last_redetect_attempt) > args.redetect_cooldown
                ):
                    last_redetect_attempt = time.time()
                    print(f"lost target, auto re-detecting '{args.prompt}'...", flush=True)
                    targets, all_candidates = run_detect(
                        processor, model, raw_frame, args.prompt, args.box_threshold, args.text_threshold,
                        args.max_targets, position_labels,
                    )

            if args.show_candidates:
                for (cx, cy, cw, ch), cscore, clabel in all_candidates:
                    cx, cy, cw, ch = int(cx), int(cy), int(cw), int(ch)
                    cv2.rectangle(display, (cx, cy), (cx + cw, cy + ch), (0, 255, 255), 1)
                    cv2.putText(
                        display, f"{clabel} {cscore:.2f}", (cx, min(display.shape[0] - 5, cy + ch + 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1,
                    )

            if not targets:
                cv2.putText(display, "press 'd' to detect", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                for tgt in targets:
                    x, y, w, h = tgt["box"]
                    label_lower = tgt["label"].lower()
                    if "red" in label_lower:
                        color = (0, 0, 255)
                    elif "blue" in label_lower:
                        color = (255, 0, 0)
                    else:
                        color = (0, 255, 0)
                    cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(display, tgt["label"], (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if paused:
                cv2.putText(display, "PAUSED", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            cv2.putText(
                display, f"prompt: {args.prompt}", (10, display.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1,
            )
            cv2.imshow(window, display)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
