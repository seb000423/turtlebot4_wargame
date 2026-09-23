#!/usr/bin/env python3
"""
target_nav.py
==============

TRACK 상태(상대 HP == 1, 마무리 접근) 전담 노드.

이전 버전과 설계가 완전히 다르다:
  - 예전: /enemy_tf(map 좌표)로 Nav2 NavigateToPose goal 을 보내는 방식
  - 지금: azimuth_tracker 와 동일한 회전(P제어) 로직을 이 노드가 직접 계산하고,
    거기에 "거리가 1m 초과면 고정 속도로 전진"까지 더해서, 회전+전진을
    하나의 Twist 메시지로 합쳐서 cmd_vel 에 발행한다.

  [멀티로봇 대응] 모든 토픽을 상대경로로 작성했다. 실행 시 네임스페이스만
  지정하면(예: --ros-args -r __ns:=/robot5) 로봇이 몇 대여도 그대로 동작한다.
  또한 mission_manager 가 robot_mode 를 TRANSIENT_LOCAL 로 발행하므로,
  이 노드도 같은 QoS 로 구독해서 늦게 떠도 마지막 상태를 즉시 받는다.

왜 azimuth_tracker 를 그대로 같이 켜지 않는가:
  azimuth_tracker 와 이 노드가 각자 따로 cmd_vel (네임스페이스 적용됨) 에 Twist 를 발행하면,
  한쪽은 회전만(linear.x=0), 한쪽은 전진만(angular.z=0) 담은 메시지를 서로
  덮어써서 로봇이 뒤섞인 명령을 받는다 (Twist 는 필드별로 합쳐지지 않고
  메시지 전체가 통째로 마지막 발행자 것으로 대체되기 때문). 그래서 TRACK
  에서는 azimuth_tracker 를 켜지 않고(ACTIVE_MODES 에 "TRACK" 넣지 않음),
  이 노드 하나가 회전과 전진을 모두 계산해 한 번에 발행한다.

입력:
  - /set_ori         (Float32) : 좌우 오차(-1~1), azimuth 와 동일한 값
  - /enemy_distance  (Float32) : 상대까지의 거리(m). Detector 에 새로 추가 필요.
  - /robot_mode      (String)  : "TRACK" 일 때만 동작

출력:
  - cmd_vel (네임스페이스 적용됨)  (Twist)   : 회전 + 전진 명령
  - /track_complete  (Bool)    : 1m 이내 도달했거나, 신호가 유실됐을 때 발행

신호 유실 처리:
  /set_ori 또는 /enemy_distance 둘 중 하나라도 SIGNAL_TIMEOUT(3초) 이상
  안 들어오면, 그 자리에 정지하고 /track_complete 를 발행한 뒤 더 이상
  아무것도 하지 않는다 (mission_manager 가 이 신호를 받아 PATROL 로 전환).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Twist


class TargetNav(Node):

    def __init__(self):
        super().__init__("target_nav")

        # ---- 파라미터 (회전 게인은 azimuth_tracker 기본값과 동일하게 맞춤) ----
        self.declare_parameter('kp', 0.8)                  # P 제어 게인 (회전)
        self.declare_parameter('max_angular', 0.3)         # rad/s, 최대 회전 속도
        self.declare_parameter('deadband', 0.05)           # 회전 데드밴드
        self.declare_parameter('linear_speed', 0.15)       # m/s, 접근 시 고정 전진 속도
        self.declare_parameter('stop_distance', 1.0)       # m, 이 거리 이하면 정지+완료
        self.declare_parameter('signal_timeout', 3.0)      # s, 이 시간 이상 신호 없으면 유실 처리
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        self.kp = self.get_parameter('kp').value
        self.max_angular = self.get_parameter('max_angular').value
        self.deadband = self.get_parameter('deadband').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.signal_timeout = self.get_parameter('signal_timeout').value
        control_hz = self.get_parameter('control_hz').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.dt = 1.0 / control_hz

        # ---- 상태 변수 ----
        self.robot_mode = "PATROL"

        self.last_error_x = None
        self.last_ori_time = None

        self.last_distance = None
        self.last_distance_time = None

        # 이번 TRACK 세션에서 이미 /track_complete 를 보냈는지 (중복 발행 방지)
        self.done_published = False

        # ---- Publisher / Subscriber ----
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.done_pub = self.create_publisher(Bool, "track_complete", 10)

        # mission_manager 가 robot_mode 를 TRANSIENT_LOCAL 로 발행하므로,
        # 늦게 뜬 이 노드도 같은 QoS 로 구독해야 마지막 상태를 즉시 받는다.
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "robot_mode", self.on_robot_mode, mode_qos)
        self.create_subscription(Float32, "aim/set_ori", self.on_set_ori, 10)
        self.create_subscription(Float32, "enemy_distance", self.on_distance, 10)

        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"TargetNav Started (TRACK 전용, cmd_vel -> {cmd_vel_topic}, "
            f"stop_distance={self.stop_distance}m, linear_speed={self.linear_speed}m/s)"
        )

    # ==========================================
    # 콜백
    # ==========================================
    def on_robot_mode(self, msg: String):
        prev = self.robot_mode
        self.robot_mode = msg.data

        if self.robot_mode == "TRACK" and prev != "TRACK":
            # TRACK 진입: 세션 초기화. 시각 기준을 지금으로 맞춰서
            # 진입 직후 곧바로 "신호 유실"로 오판하지 않게 한다.
            self.done_published = False
            now = self.get_clock().now()
            self.last_ori_time = now
            self.last_distance_time = now
            self.get_logger().info("=== TRACK 시작 (azimuth 겸용 접근 제어) ===")

        if self.robot_mode != "TRACK" and prev == "TRACK":
            # TRACK 이탈: 즉시 정지
            self._publish_twist(0.0, 0.0)
            self.get_logger().info("TRACK 이탈 -> 정지")

    def on_set_ori(self, msg: Float32):
        self.last_error_x = msg.data
        self.last_ori_time = self.get_clock().now()

    def on_distance(self, msg: Float32):
        self.last_distance = msg.data
        self.last_distance_time = self.get_clock().now()

    # ==========================================
    # 제어 루프
    # ==========================================
    def control_loop(self):
        if self.robot_mode != "TRACK":
            return   # TRACK 이 아니면 아무것도 발행하지 않음 (cmd_vel 충돌 방지)

        if self.done_published:
            return   # 이미 완료 처리됨 (정지 상태 유지, 재발행 안 함)

        # ---- 신호 유실 판정 ----
        if self._signal_lost():
            self.get_logger().warn(
                f"TRACK 중 신호 유실 ({self.signal_timeout}초 이상 미수신) -> 종료"
            )
            self._publish_twist(0.0, 0.0)
            self._publish_done()
            return

        # ---- 거리 도달 판정 ----
        if self.last_distance is not None and self.last_distance <= self.stop_distance:
            self.get_logger().info(
                f"목표 거리 도달 ({self.last_distance:.2f}m <= {self.stop_distance}m) -> 종료"
            )
            self._publish_twist(0.0, 0.0)
            self._publish_done()
            return

        # ---- 회전(azimuth 와 동일한 P 제어) + 전진(고정 속도) 계산 ----
        angular_z = self._compute_angular()
        linear_x = self.linear_speed

        self._publish_twist(linear_x, angular_z)

    def _signal_lost(self) -> bool:
        now = self.get_clock().now()

        if self.last_ori_time is None or self.last_distance_time is None:
            return False  # 아직 한 번도 안 들어온 것 -> 유실이 아니라 "대기 중"

        ori_elapsed = (now - self.last_ori_time).nanoseconds / 1e9
        dist_elapsed = (now - self.last_distance_time).nanoseconds / 1e9

        return (ori_elapsed > self.signal_timeout) or (dist_elapsed > self.signal_timeout)

    def _compute_angular(self) -> float:
        if self.last_error_x is None:
            return 0.0

        e = self.last_error_x
        if abs(e) < self.deadband:
            return 0.0

        target = -self.kp * e
        return max(-self.max_angular, min(self.max_angular, target))

    def _publish_twist(self, linear_x: float, angular_z: float):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def _publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        self.done_published = True
        self.get_logger().info("track_complete 발행")


def main():
    rclpy.init()
    node = TargetNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_twist(0.0, 0.0)   # 종료 시 반드시 정지
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()