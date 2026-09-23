"""
USB 웹캠 -> YOLO(enemy/sensor) 라이브 트래킹 -> 심판 백엔드(rosbridge mock) publish.

backend/app/config.py의 토픽 계약을 그대로 따른다:
  /judge/image_raw   {format, data(base64), mock}
  /judge/detections  {detections: [{x,y,w,h,conf,label}, ...]}  (x,y,w,h는 0~1 정규화)

팀(blue/red) 색상 매핑, 피격(hit_event) 판정은 이번 스크립트 범위 밖 — 다음 단계에서 추가.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import logging
import sys
import time
from pathlib import Path

import cv2
import websockets
from ultralytics import YOLO

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("judge_vision")

DEFAULT_MODEL = str(Path(__file__).resolve().parent / "weights" / "best.pt")
DEFAULT_WS_URL = "ws://localhost:8000/ws"
IMAGE_TOPIC = "/judge/image_raw"
DETECTIONS_TOPIC = "/judge/detections"


def list_cameras(max_index: int = 5) -> None:
    """입력: 탐색할 최대 인덱스. 출력 없음 — 열리는 카메라 인덱스와 해상도를 콘솔에 출력."""
    for idx in range(max_index + 1):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW if sys.platform == "win32" else 0)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            logger.info("camera index %d: OK (%dx%d)", idx, w, h)
        else:
            logger.info("camera index %d: not available", idx)
        cap.release()


def boxes_to_detections(result, frame_w: int, frame_h: int) -> list[dict]:
    """입력: ultralytics 추론 결과 1개(result), 프레임 크기. 출력: {x,y,w,h,conf,label,track_id} 리스트.
    x,y는 bbox 좌상단, w,h는 bbox 크기 — 모두 프레임 크기로 나눈 0~1 정규화 값."""
    detections: list[dict] = []
    if result.boxes is None:
        return detections

    names = result.names
    ids = result.boxes.id
    for i, box in enumerate(result.boxes):
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cls = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        det = {
            "x": round(max(0.0, x1) / frame_w, 4),
            "y": round(max(0.0, y1) / frame_h, 4),
            "w": round((x2 - x1) / frame_w, 4),
            "h": round((y2 - y1) / frame_h, 4),
            "conf": round(conf, 3),
            "label": names.get(cls, str(cls)),
        }
        if ids is not None:
            det["track_id"] = int(ids[i].item())
        detections.append(det)
    return detections


async def publish(ws, topic: str, msg: dict) -> None:
    await ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg}))


def open_camera(args: argparse.Namespace) -> cv2.VideoCapture:
    """입력: CLI args(camera index 또는 스트림 URL, width, height). 출력: 열린 cv2.VideoCapture (실패 시 isOpened()==False).
    args.camera가 숫자 문자열이면 로컬 장치 인덱스로 변환하고, 아니면(예: 아이폰 IP 카메라 앱의
    http://.../video, rtsp://... 등) 그대로 URL로 cv2에 전달한다."""
    source = int(args.camera) if args.camera.isdigit() else args.camera
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else 0
    cap = cv2.VideoCapture(source, backend)
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    return cap


MAX_CONSECUTIVE_FAILURES = 20  # USB 뽑혔다 다시 꽂히는 경우 등, 이 횟수만큼 실패하면 캡처를 재오픈
READ_TIMEOUT_SEC = 2.0  # cap.read()는 자체 타임아웃이 없어서, 헐거운 USB 연장선 등으로 드라이버가
# 응답 없이 멈춰버리면 이 타임아웃으로 감지해서 강제로 재오픈한다(그냥 기다리면 무한정 멈춤).


async def read_frame_with_timeout(cap: cv2.VideoCapture):
    """입력: 열린 VideoCapture. 출력: (ok, frame, timed_out).
    cap.read()를 별도 스레드에서 돌려서 이벤트 루프가 멈추지 않게 하고, READ_TIMEOUT_SEC 넘으면 포기."""
    loop = asyncio.get_running_loop()
    try:
        ok, frame = await asyncio.wait_for(loop.run_in_executor(None, cap.read), timeout=READ_TIMEOUT_SEC)
        return ok, frame, False
    except asyncio.TimeoutError:
        return False, None, True


INFER_TIMEOUT_SEC = 5.0  # model.track()도 블로킹 호출 — 손상된 프레임(끊긴 케이블 등)이 드물게
# 후처리에서 멈추는 경우를 대비해 타임아웃을 걸어서 그 프레임만 건너뛰고 계속 진행한다.


async def track_with_timeout(model: YOLO, frame, conf: float):
    """입력: 모델, 프레임, conf. 출력: (result 또는 None, timed_out)."""
    loop = asyncio.get_running_loop()
    try:
        results = await asyncio.wait_for(
            loop.run_in_executor(
                None, lambda: model.track(frame, persist=True, conf=conf, tracker="bytetrack.yaml", verbose=False)
            ),
            timeout=INFER_TIMEOUT_SEC,
        )
        return results[0], False
    except asyncio.TimeoutError:
        return None, True


async def stream(args: argparse.Namespace) -> None:
    model = YOLO(args.model)
    logger.info("model loaded: %s (classes=%s)", args.model, model.names)

    cap = open_camera(args)
    if not cap.isOpened():
        raise RuntimeError(f"camera source {args.camera!r} could not be opened")

    frame_interval = 1.0 / args.fps
    logger.info("camera %s opened, streaming at ~%s fps", args.camera, args.fps)
    fail_count = 0

    try:
        while True:
            try:
                async with websockets.connect(args.ws_url) as ws:
                    logger.info("connected to %s", args.ws_url)
                    await ws.send(json.dumps({"op": "advertise", "topic": IMAGE_TOPIC, "type": "sensor_msgs/CompressedImage"}))
                    await ws.send(json.dumps({"op": "advertise", "topic": DETECTIONS_TOPIC, "type": "turtlebot4_wargame_msgs/Detections"}))

                    while True:
                        loop_start = time.monotonic()
                        ok, frame, timed_out = await read_frame_with_timeout(cap)
                        if not ok:
                            fail_count = MAX_CONSECUTIVE_FAILURES if timed_out else fail_count + 1
                            if fail_count >= MAX_CONSECUTIVE_FAILURES:
                                if timed_out:
                                    logger.warning("camera read timed out after %.0fs (flaky USB connection?), reopening", READ_TIMEOUT_SEC)
                                else:
                                    logger.warning("camera unresponsive after %d failed reads, reopening", fail_count)
                                loop = asyncio.get_running_loop()
                                try:
                                    await asyncio.wait_for(loop.run_in_executor(None, cap.release), timeout=1.0)
                                except asyncio.TimeoutError:
                                    logger.warning("cap.release() also hung, abandoning old capture handle")
                                await asyncio.sleep(1.0)
                                cap = open_camera(args)
                                fail_count = 0
                            else:
                                logger.warning("frame grab failed, retrying")
                                await asyncio.sleep(0.1)
                            continue
                        fail_count = 0

                        result, infer_timed_out = await track_with_timeout(model, frame, args.conf)
                        if infer_timed_out:
                            logger.warning("YOLO inference timed out after %.0fs on this frame, skipping", INFER_TIMEOUT_SEC)
                            elapsed = time.monotonic() - loop_start
                            await asyncio.sleep(max(0.0, frame_interval - elapsed))
                            continue
                        h, w = frame.shape[:2]
                        detections = boxes_to_detections(result, w, h)

                        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                        if ok:
                            b64 = base64.b64encode(buf).decode("ascii")
                            await publish(ws, IMAGE_TOPIC, {"format": "jpeg", "data": b64, "mock": False})
                        await publish(ws, DETECTIONS_TOPIC, {"detections": detections})

                        elapsed = time.monotonic() - loop_start
                        await asyncio.sleep(max(0.0, frame_interval - elapsed))
            except (websockets.exceptions.ConnectionClosed, OSError) as exc:
                logger.warning("websocket disconnected (%s), reconnecting in 2s", exc)
                await asyncio.sleep(2.0)
    finally:
        cap.release()


def main() -> None:
    parser = argparse.ArgumentParser(description="Webcam -> YOLO -> judge backend publisher")
    parser.add_argument(
        "--camera",
        default="0",
        help="local device index (e.g. 0) or a network stream URL, e.g. an iPhone IP-camera app's "
        "http://<iphone-ip>:8080/video or rtsp://<iphone-ip>:8554/live",
    )
    parser.add_argument("--model", default=DEFAULT_MODEL, help="path to YOLO .pt weights")
    parser.add_argument("--ws-url", default=DEFAULT_WS_URL, help="backend rosbridge websocket URL")
    parser.add_argument("--conf", type=float, default=0.5, help="detection confidence threshold")
    parser.add_argument("--fps", type=float, default=10.0, help="target publish rate")
    parser.add_argument("--width", type=int, default=0, help="requested capture width (0=camera default)")
    parser.add_argument("--height", type=int, default=0, help="requested capture height (0=camera default)")
    parser.add_argument("--jpeg-quality", type=int, default=70, help="JPEG encode quality (1-100)")
    parser.add_argument("--list-cameras", action="store_true", help="probe camera indices 0-5 and exit")
    args = parser.parse_args()

    if args.list_cameras:
        list_cameras()
        return

    try:
        asyncio.run(stream(args))
    except KeyboardInterrupt:
        logger.info("stopped by user")


if __name__ == "__main__":
    main()
