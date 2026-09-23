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

MODEL_ID = "IDEA-Research/grounding-dino-tiny"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def load_model():
    processor = AutoProcessor.from_pretrained(MODEL_ID)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL_ID)
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
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    return cv2.legacy.TrackerCSRT_create()


def main() -> None:
    parser = argparse.ArgumentParser(description="Grounding DINO zero-shot detect once, then CSRT-track as it moves")
    parser.add_argument("--camera", default="0", help="local device index or stream URL")
    parser.add_argument("--prompt", required=True, help='text prompt, e.g. "a ping pong ball"')
    parser.add_argument("--box-threshold", type=float, default=0.35)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--max-targets", type=int, default=2, help="confidence 상위 몇 개까지만 추적할지")
    args = parser.parse_args()

    print("loading Grounding DINO (first run downloads weights)...", flush=True)
    processor, model = load_model()
    print("model loaded.", flush=True)

    source = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(source)
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"camera source {args.camera!r} could not be opened", file=sys.stderr)
        sys.exit(1)

    targets: list[dict] = []  # [{"tracker":..., "box":(x,y,w,h), "label":str}, ...]
    editing_prompt = False
    prompt_buffer = ""
    paused = False
    raw_frame = None
    window = "zero-shot detect + track  [d]=detect  [p]=prompt  [space]=pause  [q]=quit"
    cv2.namedWindow(window)

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
                dets = detect(processor, model, raw_frame, args.prompt, args.box_threshold, args.text_threshold)
                dets = dets[: args.max_targets]  # confidence 상위 N개까지만 (이미 내림차순 정렬됨)
                print(f"  took {time.time() - t0:.2f}s", flush=True)
                if not dets:
                    print("  not found", flush=True)
                    targets = []
                else:
                    print(f"  found {len(dets)} instance(s)", flush=True)
                    targets = []
                    for (x, y, w, h), score, label in dets:
                        print(f"    '{label}' conf={score:.2f} box=({x:.0f},{y:.0f},{w:.0f},{h:.0f})", flush=True)
                        box = (int(x), int(y), int(w), int(h))
                        t = make_tracker()
                        t.init(raw_frame, box)
                        targets.append({"tracker": t, "box": box, "label": label})

            if new_frame:
                still_tracking = []
                for tgt in targets:
                    ok_t, box = tgt["tracker"].update(raw_frame)
                    if ok_t:
                        tgt["box"] = tuple(int(v) for v in box)
                        still_tracking.append(tgt)
                targets = still_tracking

            if not targets:
                cv2.putText(display, "press 'd' to detect", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                for tgt in targets:
                    x, y, w, h = tgt["box"]
                    cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(display, tgt["label"], (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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
