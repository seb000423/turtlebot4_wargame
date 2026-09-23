#!/usr/bin/env python3
"""
phone_cam_detector.py
======================

AMR의 OAK-D 카메라 대신, Iriun Webcam 으로 연결한 스마트폰 카메라를
YOLO 입력으로 사용해 적(enemy)을 탐지하는 노드.

- 카메라 입력: ROS 이미지 토픽이 아니라 v4l2 장치(/dev/video2, Iriun 가상 웹캠)를 읽는다.
  이 환경에서는 OpenCV의 V4L2 백엔드가 v4l2loopback 장치를 열지 못하는 문제가 있어서
  (index/name 모두 "can't be used to capture" 로 실패), 대신 실제로 동작을 확인한
  ffmpeg 서브프로세스 파이프로 rawvideo(bgr24) 프레임을 읽는다.
- 캡처 쓰레드가 항상 "최신 프레임"만 유지하고(큐 누적 없음), 추론 쓰레드가
  그 최신 프레임을 가져다 쓰는 구조(bbox.py/yolo_seg_detector.py 와 동일한
  패턴)로, 추론이 느려도 이미지 수신이 밀리지 않게 한다.
- 폰이 아직 Iriun 앱에 연결되지 않은 동안은 장치가 단색 플레이스홀더 화면을 내보내는데,
  이 경우 YOLO 추론을 돌리지 않고 대기만 하다가, 실제 영상이 들어오는 순간(공간적
  분산이 생기는 순간) 자동으로 추론을 시작한다 -> 노드를 미리 켜놔도 되고, 폰이
  나중에 연결되어도 별도 재시작 없이 바로 동작한다.
- depth 카메라가 없으므로 거리(Z)/enemy_tf 는 발행하지 않는다. 좌우 오차(aim/set_ori)만
  기존과 동일한 포맷(-1~1 정규화 값)으로 발행해서 azimuth_tracker.py 를 그대로 쓴다.
- turtlebot4_wargame-ui 계약(topics.ts): 조준각은 aim/set_ori, 탐지 목록은 enemy/detections
  (0~1 정규화 좌표), 카메라 화면은 enemy_image/compressed(JPEG).
"""

import os
import select
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
from turtlebot4_wargame_msgs.msg import Detection, Detections

from ultralytics import YOLO

# =========================================
# 설정
# =========================================
# [주의] 로봇 온보드 PC 에 배포할 때는 실제 워크스페이스 경로에 맞게 바꿀 것
# (다른 yolo_detector 스크립트들도 각자 경로가 다르게 하드코딩되어 있음 - 통일 필요)
# 가중치 경로: 기본값은 이 패키지에 포함된 y11s_my_best.pt.
# 다른 위치의 가중치를 쓰려면 환경변수 YOLO_MODEL_PATH 로 덮어쓰세요.
MODEL_PATH = os.environ.get(
    "YOLO_MODEL_PATH",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "y11s_my_best.pt"),
)

TARGET_CLASS = "enemy"
CONF_TH = 0.5
INFERENCE_SIZE = 640

# USB 재연결/재부팅 등으로 장치 번호가 바뀔 수 있음 - v4l2-ctl --list-devices 로 확인.
# (한때는 /dev/video2 였는데 지금은 /dev/video0 로 바뀜)
CAMERA_DEVICE = "/dev/video0"   # Iriun Webcam (폰 카메라) v4l2 장치
CAPTURE_WIDTH = 1280            # 4K 원본을 그대로 쓰면 디코딩/추론 비용이 커서 다운스케일
CAPTURE_HEIGHT = 720

PLACEHOLDER_STD_THRESHOLD = 2.0  # 이 값 미만이면 "단색 화면(폰 미연결)"로 간주

# ---- depth 카메라 없이 bbox 높이로 거리 추정 (핀홀 모델: distance = k / pixel_height) ----
# k = 실측 거리(m) x 그 순간 박스 높이(px). 1280x720 캡처 기준으로 실측 캘리브레이션한 값.
# 실측: 상대 로봇을 93cm 에 세우고 측정 -> 박스 높이 343px -> k = 0.93 * 343
DISTANCE_CALIB_K = 0.93 * 343


def is_placeholder_frame(frame) -> bool:
    """폰 미연결 시 Iriun 이 내보내는 단색 대기화면인지 판별 (공간적 분산 거의 0)."""
    sample = frame[::8, ::8].reshape(-1, 3)
    return float(sample.std(axis=0).max()) < PLACEHOLDER_STD_THRESHOLD


# =========================================
# Phone Camera Receiver (ffmpeg 파이프, 항상 최신 프레임만 유지)
# =========================================
class PhoneCamReceiver:
    def __init__(self, device=CAMERA_DEVICE, width=CAPTURE_WIDTH, height=CAPTURE_HEIGHT, logger=None):
        self.device = device
        self.width = width
        self.height = height
        self.frame_size = width * height * 3
        self.logger = logger

        self.latest_frame = None
        self.lock = threading.Lock()
        self.frame_cond = threading.Condition()
        self.running = True

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _log(self, msg):
        if self.logger is not None:
            self.logger.info(msg)

    def _open_ffmpeg(self):
        cmd = [
            "ffmpeg", "-nostdin", "-loglevel", "error",
            # 입력 분석/내부 버퍼링을 최소화해서 ffmpeg 자체가 지연을 더하지 않게 함
            # (네트워크/Iriun 쪽 지연은 이 옵션으로 못 줄임 - 우리 쪽 파이프 구간만 최소화)
            "-fflags", "nobuffer", "-flags", "low_delay", "-probesize", "32",
            "-analyzeduration", "0",
            "-f", "v4l2", "-i", self.device,
            "-vf", f"scale={self.width}:{self.height}",
            "-pix_fmt", "bgr24", "-f", "rawvideo", "pipe:1",
        ]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    def _read_exact(self, stream, n, stall_timeout=3.0):
        """n바이트를 읽되, stall_timeout 동안 새 데이터가 전혀 안 오면(EOF 없이
        멈춘 상태) None 을 반환해서 호출부가 ffmpeg 를 재시작하게 한다.
        (네트워크 순단 시 ffmpeg 가 종료도 안 하고 그냥 멈춰버리는 경우가 있었음)
        """
        buf = bytearray()
        fd = stream.fileno()
        while len(buf) < n:
            ready, _, _ = select.select([fd], [], [], stall_timeout)
            if not ready:
                return None
            chunk = stream.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _capture_loop(self):
        proc = None
        while self.running:
            if proc is None:
                self._log(f"ffmpeg 캡처 시작 시도: {self.device}")
                proc = self._open_ffmpeg()

            buf = self._read_exact(proc.stdout, self.frame_size)
            if buf is None:
                # 장치 연결 끊김/오류 -> ffmpeg 재시작
                proc.kill()
                proc.wait()
                proc = None
                time.sleep(1.0)
                continue

            frame = np.frombuffer(buf, dtype=np.uint8).reshape((self.height, self.width, 3))

            with self.lock:
                self.latest_frame = frame

            with self.frame_cond:
                self.frame_cond.notify()

        if proc is not None:
            proc.kill()

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def stop(self):
        self.running = False


# =========================================
# Detector Node
# =========================================
class PhoneCamDetector(Node):
    def __init__(self):
        super().__init__("phone_cam_detector")

        self.model = YOLO(MODEL_PATH)
        try:
            self.model.to("cuda")
            self.get_logger().info(f"YOLO GPU 사용, 클래스: {self.model.names}")
        except Exception as e:
            self.get_logger().warn(f"CUDA 실패, CPU 사용: {e}")

        self.bridge = CvBridge()

        self.detect_pub = self.create_publisher(Bool, "is_enemy", 10)
        # turtlebot4_wargame-ui 계약(topics.ts): 조준각은 aim/set_ori, 탐지 목록은 enemy/detections,
        # 카메라 화면은 enemy_image/compressed(JPEG) - CameraFeed.tsx가 base64 JPEG를 기대함
        self.error_pub = self.create_publisher(Float32, "aim/set_ori", 10)
        self.distance_pub = self.create_publisher(Float32, "enemy_distance", 10)
        self.image_pub = self.create_publisher(Image, "enemy_image", 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, "enemy_image/compressed", 10)
        self.detections_pub = self.create_publisher(Detections, "enemy/detections", 10)

        self.last_image_pub_time = self.get_clock().now()
        self.image_pub_period = 0.0  # 스로틀 없이 처리된 프레임마다 그대로 발행 (실시간 시각 확인용)

        self.receiver = PhoneCamReceiver(logger=self.get_logger())
        self.phone_connected = False

        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()

        self.get_logger().info(
            f"Phone Camera Detector 시작 (device={CAMERA_DEVICE}) - 폰 연결 대기 중..."
        )

    def inference_loop(self):
        while rclpy.ok():
            with self.receiver.frame_cond:
                self.receiver.frame_cond.wait(timeout=1.0)

            frame = self.receiver.get_latest_frame()
            if frame is None:
                continue

            if is_placeholder_frame(frame):
                if self.phone_connected:
                    self.get_logger().warn("폰 연결 끊김 (대기화면 감지) - 추론 일시중지")
                self.phone_connected = False
                continue

            if not self.phone_connected:
                self.phone_connected = True
                self.get_logger().info("폰 영상 수신 시작 -> YOLO 추론 시작")

            results = self.model.predict(
                frame,
                imgsz=INFERENCE_SIZE,
                conf=CONF_TH,
                verbose=False,
            )

            detected = False
            img_h, img_w = frame.shape[:2]
            ui_detections = []  # turtlebot4_wargame-ui enemy/detections 용 (TARGET_CLASS 매칭된 것만)

            if results[0].boxes is not None and len(results[0].boxes) > 0:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                classes = results[0].boxes.cls.cpu().numpy()
                confs = results[0].boxes.conf.cpu().numpy()

                for box, cls_id, conf in zip(boxes, classes, confs):
                    cls_name = self.model.names[int(cls_id)]
                    if cls_name != TARGET_CLASS:
                        continue
                    if detected:
                        # 여러 개 검출되면 첫 번째 박스만 조준 대상으로 사용
                        continue

                    x1, y1, x2, y2 = map(int, box)
                    u = (x1 + x2) / 2.0
                    v = (y1 + y2) / 2.0
                    pixel_height = y2 - y1

                    # depth 카메라 없이 bbox 높이 기반 거리 추정 (핀홀 모델)
                    distance_m = DISTANCE_CALIB_K / pixel_height if pixel_height > 0 else None

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (int(u), int(v)), 5, (0, 0, 255), -1)
                    label = f"{cls_name} {conf:.2f}"
                    if distance_m is not None:
                        label += f" {distance_m:.2f}m"
                    cv2.putText(
                        frame, label,
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2,
                    )

                    detected = True

                    error_x = (u - img_w / 2.0) / (img_w / 2.0)
                    msg = Float32()
                    msg.data = float(error_x)
                    self.error_pub.publish(msg)

                    if distance_m is not None:
                        dist_msg = Float32()
                        dist_msg.data = float(distance_m)
                        self.distance_pub.publish(dist_msg)

                    # CameraFeed.tsx 가 x/y/w/h 를 0~1 비율로 그대로 CSS % 에 쓰므로
                    # (예: left: `${d.x * 100}%`), 픽셀이 아니라 이미지 크기로 나눈
                    # 정규화 값으로 발행해야 한다.
                    det = Detection()
                    det.x = float(x1) / img_w
                    det.y = float(y1) / img_h
                    det.w = float(x2 - x1) / img_w
                    det.h = float(y2 - y1) / img_h
                    det.conf = float(conf)
                    det.label = cls_name
                    ui_detections.append(det)

            state = Bool()
            state.data = detected
            self.detect_pub.publish(state)

            detections_msg = Detections()
            detections_msg.detections = ui_detections
            self.detections_pub.publish(detections_msg)

            now = self.get_clock().now()
            if (now - self.last_image_pub_time).nanoseconds / 1e9 >= self.image_pub_period:
                self.last_image_pub_time = now
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(img_msg)

                # turtlebot4_wargame-ui CameraFeed.tsx 용: 같은 프레임을 JPEG로 인코딩해서 발행.
                ok, jpeg_buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    compressed_msg = CompressedImage()
                    compressed_msg.header = img_msg.header
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = jpeg_buf.tobytes()
                    self.compressed_image_pub.publish(compressed_msg)

    def destroy_node(self):
        self.receiver.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = PhoneCamDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
