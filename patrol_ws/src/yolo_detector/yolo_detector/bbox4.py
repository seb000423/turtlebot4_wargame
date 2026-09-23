import os
import rclpy
import cv2
import numpy as np
import threading
import time
import socket
import random
import hashlib

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
from turtlebot4_wargame_msgs.msg import Detection, Detections

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from message_filters import Subscriber, ApproximateTimeSynchronizer

from ultralytics import YOLO

# 가중치 경로: 기본값은 이 패키지에 포함된 y11s_my_best.pt.
# 다른 위치의 가중치를 쓰려면 환경변수 YOLO_MODEL_PATH 로 덮어쓰세요.
MODEL_PATH = os.environ.get(
    "YOLO_MODEL_PATH",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "y11s_my_best.pt"),
)
TARGET_CLASS = "enemy"
CONF_TH = 0.5
INFERENCE_SIZE = 640

CAMERA_FRAME = "oakd_rgb_camera_optical_frame"
GLOBAL_FRAME = "odom"

class ImageReceiver:
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()

        self.latest_data = None
        self.data_lock = threading.Lock()
        self.frame_cond = threading.Condition()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = node.create_subscription(
            CameraInfo,
            "oakd/rgb/camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.rgb_sub = Subscriber(
            node,
            CompressedImage,
            "oakd/rgb/image_raw/compressed",
            qos_profile=qos_profile_sensor_data
        )

        self.depth_sub = Subscriber(
            node,
            Image,
            "oakd/stereo/image_raw",
            qos_profile=qos_profile_sensor_data
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=5,
            slop=0.1
        )

        self.sync.registerCallback(
            self.sync_callback
        )

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def sync_callback(self, rgb_msg, depth_msg):
        try:
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, 
                desired_encoding="passthrough"
            )

            with self.data_lock:
                self.latest_data = {
                    "frame": frame.copy(),
                    "depth": depth.copy(),
                    "stamp": rgb_msg.header.stamp,

                    "fx": self.fx,
                    "fy": self.fy,
                    "cx": self.cx,
                    "cy": self.cy
                }

            with self.frame_cond:
                self.frame_cond.notify()

        except Exception as e:

            self.node.get_logger().error(
                f"Sync Error : {e}"
            )

    def get_latest_data(self):
        with self.data_lock:
            if self.latest_data is None:
                return None

            return {
                "frame": self.latest_data["frame"].copy(),
                "depth": self.latest_data["depth"].copy(),
                "stamp": self.latest_data["stamp"],
                "fx": self.latest_data["fx"],
                "fy": self.latest_data["fy"],
                "cx": self.latest_data["cx"],
                "cy": self.latest_data["cy"]
            }

class CamouflageDetectorAsync(Node):
    def __init__(self):
        super().__init__("camouflage_detector_async")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.model = YOLO(MODEL_PATH)
        try:
            self.model.to("cuda")
            self.get_logger().info("YOLO GPU enabled")

        except Exception as e:
            self.get_logger().warn(
                f"CUDA failed : {e}"
            )

        self.target_track_id = None
        
        self.track_states = {}

        self.iff_port = 12345
        self.my_ip = self.get_local_ip()
        
        self.udp_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_send_sock.settimeout(0.0)

        self.udp_listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.udp_listen_sock.bind(('', self.iff_port))
        except Exception as e:
            self.get_logger().warn(f"Failed to bind UDP listener: {e}")

        self.iff_listen_thread = threading.Thread(
            target=self.iff_listen_loop,
            daemon=True
        )
        self.iff_listen_thread.start()

        self.detect_pub = self.create_publisher(
            Bool,
            "is_enemy",
            10
        )

        # turtlebot4_wargame-ui 계약(topics.ts): 조준각은 aim/set_ori, 탐지 목록은 enemy/detections,
        # 카메라 화면은 enemy_image/compressed(JPEG)
        self.error_pub = self.create_publisher(
            Float32,
            "aim/set_ori",
            10
        )

        self.enemy_pub = self.create_publisher(
            PointStamped,
            "enemy_tf",
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            "enemy_image",
            10
        )

        self.compressed_image_pub = self.create_publisher(
            CompressedImage,
            "enemy_image/compressed",
            10
        )

        self.detections_pub = self.create_publisher(
            Detections,
            "enemy/detections",
            10
        )

        self.last_image_pub_time = self.get_clock().now()
        self.image_pub_period = 0.05 # 원래 0.2(5프레임)이었던 것을 0.05(20프레임)으로 상향

        self.receiver = ImageReceiver(self)

        self.inference_thread = threading.Thread(
            target=self.inference_loop,
            daemon=True
        )

        self.inference_thread.start()
        self.get_logger().info("Camouflage Detector Started")

    def get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect(("8.8.8.8", 80))
            except:
                s.connect(("192.168.255.255", 1))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return '127.0.0.1'

    def iff_listen_loop(self):
        self.get_logger().info("IFF Listener Thread Started")
        while rclpy.ok():
            try:
                data, addr = self.udp_listen_sock.recvfrom(1024)
                
                if addr[0] == self.my_ip or addr[0] == '127.0.0.1':
                    continue

                message = data.decode().strip()
                
                if len(message) == 6 and message.isdigit():
                    self.get_logger().info(f"Received IFF Challenge from {addr}: {message}")
                    response = hashlib.sha256(message.encode()).hexdigest()
                    self.udp_listen_sock.sendto(response.encode(), addr)
            except Exception as e:
                pass

    def inference_loop(self):
        while rclpy.ok():
            data = self.receiver.get_latest_data()

            if data is None:
                time.sleep(0.01)
                continue

            frame = data["frame"]
            depth_image = data["depth"]
            stamp = data["stamp"]

            fx = data["fx"]
            fy = data["fy"]
            cx = data["cx"]
            cy = data["cy"]

            while True:
                try:
                    udp_data, addr = self.udp_send_sock.recvfrom(1024)
                    if addr[0] == self.my_ip or addr[0] == '127.0.0.1':
                        continue
                        
                    response = udp_data.decode().strip()
                    
                    for tid, state in self.track_states.items():
                        if state["status"] == "unknown" and state["expected_hash"] == response:
                            state["status"] = "friendly"
                            break
                except socket.error:
                    break

            start_time = time.time()
            results = self.model.track(
                frame,
                imgsz=640,
                conf=CONF_TH,
                persist=True,
                tracker="bytetrack.yaml",
                verbose=False
            )

            detected = False
            ui_detections = []  # turtlebot4_wargame-ui enemy/detections 용 (0~1 정규화 좌표)
            img_h, img_w = frame.shape[:2]

            if results[0].boxes is not None and len(results[0].boxes) > 0:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                classes = results[0].boxes.cls.cpu().numpy()
                confs = results[0].boxes.conf.cpu().numpy()
                
                track_ids = results[0].boxes.id
                if track_ids is not None:
                    track_ids = track_ids.cpu().numpy()
                else:
                    track_ids = [None] * len(boxes)

                for box, cls_id, conf, track_id in zip(boxes, classes, confs, track_ids):
                    track_id = int(track_id) if track_id is not None else None
                    cls_name = self.model.names[int(cls_id)]
                    if cls_name not in [TARGET_CLASS, "sensor"]:
                        continue

                    x1, y1, x2, y2 = map(int, box)
                    
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)
                    img_h, img_w = frame.shape[:2]

                    current_status = "enemy"
                    
                    if cls_name == TARGET_CLASS and track_id is not None:
                        if track_id not in self.track_states:
                            challenge = str(random.randint(100000, 999999))
                            expected = hashlib.sha256(challenge.encode()).hexdigest()
                            self.track_states[track_id] = {
                                "status": "unknown",
                                "challenge_time": time.time(),
                                "expected_hash": expected
                            }
                            try:
                                self.udp_send_sock.sendto(challenge.encode(), ('255.255.255.255', self.iff_port))
                            except Exception:
                                pass
                        
                        state = self.track_states[track_id]
                        if state["status"] == "unknown":
                            if time.time() - state["challenge_time"] > 1.0:
                                state["status"] = "enemy"
                        
                        current_status = state["status"]

                    X, Y, Z = None, None, None
                    is_protruding = True
                    
                    if depth_image is not None:
                        depth_h, depth_w = depth_image.shape[:2]
                        
                        box_w = x2 - x1
                        box_h = y2 - y1
                        
                        roi_size = min(box_w, box_h, 40) // 2
                        roi_x1 = max(0, u - roi_size)
                        roi_x2 = min(img_w, u + roi_size)
                        roi_y1 = max(0, v - roi_size)
                        roi_y2 = min(img_h, v + roi_size)

                        d_x1 = int(roi_x1 * depth_w / img_w)
                        d_x2 = int(roi_x2 * depth_w / img_w)
                        d_y1 = int(roi_y1 * depth_h / img_h)
                        d_y2 = int(roi_y2 * depth_h / img_h)

                        if d_x2 > d_x1 and d_y2 > d_y1:
                            roi_depth = depth_image[d_y1:d_y2, d_x1:d_x2]
                            values = roi_depth[roi_depth > 0]
                            
                            if len(values) > 0:
                                distance_mm = np.median(values)
                                Z = distance_mm / 1000.0
                                
                                bg_margin = 20
                                bg_d_x1 = max(0, int((x1 - bg_margin) * depth_w / img_w))
                                bg_d_x2 = min(depth_w, int((x2 + bg_margin) * depth_w / img_w))
                                bg_d_y1 = max(0, int((y1 + box_h*0.3) * depth_h / img_h))
                                bg_d_y2 = min(depth_h, int((y2 - box_h*0.3) * depth_h / img_h))
                                
                                if bg_d_y2 > bg_d_y1 and bg_d_x2 > bg_d_x1:
                                    bg_roi = depth_image[bg_d_y1:bg_d_y2, bg_d_x1:bg_d_x2]
                                    bg_values = bg_roi[bg_roi > 0]
                                    if len(bg_values) > 0:
                                        bg_distance_m = np.median(bg_values) / 1000.0
                                        if abs(bg_distance_m - Z) < 0.10:
                                            is_protruding = False

                                if fx is not None:
                                    X = ((u - cx) * Z / fx)
                                    Y = ((v - cy) * Z / fy)
                                    
                    if not is_protruding and cls_name == TARGET_CLASS:
                        current_status = "flat_ignored"

                    id_str = f" {track_id}" if track_id is not None else ""

                    if cls_name == "sensor":
                        color = (0, 255, 255)
                        display_name = f"sensor{id_str}"
                    elif current_status == "friendly":
                        color = (255, 0, 0)
                        display_name = f"Friendly{id_str}"
                    elif current_status == "unknown":
                        color = (0, 255, 0)
                        display_name = f"Detecting...{id_str}"
                    elif current_status == "flat_ignored":
                        color = (128, 128, 128)
                        display_name = f"Flat Shadow{id_str}"
                    else:
                        color = (0, 0, 255)
                        display_name = f"{TARGET_CLASS}{id_str}"

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
                    if X is not None and Y is not None:
                        self.get_logger().info(
                            f"[{display_name}] "
                            f"Camera 3D - X: {X:.2f}, Y: {Y:.2f}, Z: {Z:.2f}m"
                        )

                    cv2.circle(frame, (u, v), 5, color, -1)

                    text = f"{display_name} {conf:.2f}"
                    if Z is not None:
                        text += f" {Z:.2f}m"

                    cv2.putText(
                        frame,
                        text,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                        cv2.LINE_AA
                    )

                    if cls_name == TARGET_CLASS and current_status == "enemy":
                        if detected:
                            continue

                        detected = True

                        if X is not None and Y is not None and Z is not None:
                            global_pt = self.publish_enemy_tf(X, Y, Z, stamp)
                            
                            if global_pt is not None and track_id is not None:
                                curr_time = time.time()
                                curr_x = global_pt.point.x
                                curr_y = global_pt.point.y
                                
                                state = self.track_states.get(track_id)
                                if state is not None:
                                    if "last_global_pos" in state:
                                        last_x, last_y = state["last_global_pos"]
                                        dt = curr_time - state["last_pos_time"]
                                        if dt > 0.05:
                                            dist = np.sqrt((curr_x - last_x)**2 + (curr_y - last_y)**2)
                                            speed = dist / dt
                                            if speed > 0.05:
                                                self.get_logger().info(f"Moving Enemy {track_id} Detected! Speed: {speed:.2f} m/s")
                                    
                                    state["last_global_pos"] = (curr_x, curr_y)
                                    state["last_pos_time"] = curr_time

                        error_x = (u - img_w / 2) / (img_w / 2)

                        msg = Float32()
                        msg.data = float(error_x)
                        self.error_pub.publish(msg)

                        det = Detection()
                        det.x = float(x1) / img_w
                        det.y = float(y1) / img_h
                        det.w = float(x2 - x1) / img_w
                        det.h = float(y2 - y1) / img_h
                        det.conf = float(conf)
                        det.label = cls_name
                        ui_detections.append(det)

            if not detected:
                self.target_track_id = None

            state = Bool()
            state.data = detected
            self.detect_pub.publish(state)

            detections_msg = Detections()
            detections_msg.detections = ui_detections
            self.detections_pub.publish(detections_msg)

            now = self.get_clock().now()
            if ((now - self.last_image_pub_time).nanoseconds / 1e9 >= self.image_pub_period):
                self.last_image_pub_time = now

                img_msg = self.receiver.bridge.cv2_to_imgmsg(
                    frame,
                    encoding="bgr8"
                )
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = CAMERA_FRAME

                self.image_pub.publish(img_msg)

                ok, jpeg_buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    compressed_msg = CompressedImage()
                    compressed_msg.header = img_msg.header
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = jpeg_buf.tobytes()
                    self.compressed_image_pub.publish(compressed_msg)

    def publish_enemy_tf(self, X, Y, Z, stamp):
        camera_frame_id = CAMERA_FRAME
        global_frame_id = "odom"
        map_frame_id = "map"

        camera_point = PointStamped()
        camera_point.header.stamp = stamp
        camera_point.header.frame_id = camera_frame_id

        camera_point.point.x = X
        camera_point.point.y = Y
        camera_point.point.z = Z

        if self.tf_buffer.can_transform(
            map_frame_id,
            camera_frame_id,
            rclpy.time.Time.from_msg(stamp)
        ):
            target_frame = map_frame_id

        else:
            target_frame = global_frame_id

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                camera_frame_id,
                rclpy.time.Time.from_msg(stamp)
            )

            enemy_point = do_transform_point(
                camera_point,
                transform
            )

            enemy_point.header.frame_id = target_frame
            enemy_point.header.stamp = stamp

            self.enemy_pub.publish(
                enemy_point
            )

            self.get_logger().info(
                f"[{target_frame}] "
                f"Enemy : "
                f"({enemy_point.point.x:.2f}, "
                f"{enemy_point.point.y:.2f}, "
                f"{enemy_point.point.z:.2f})"
            )
            return enemy_point

        except Exception as e:
            self.get_logger().warn(
                f"TF transform failed : {e}"
            )
            return None

def main():
    rclpy.init()
    node = CamouflageDetectorAsync()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
