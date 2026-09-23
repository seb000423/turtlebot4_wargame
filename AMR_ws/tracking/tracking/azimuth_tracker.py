#!/usr/bin/env python3
"""
azimuth_tracker.py
===================

적 조준(제자리 회전) 전담 노드.

[병합 내역] mission_manager.py 가 /robot_mode 를 TRANSIENT_LOCAL QoS 로
발행하도록 바뀌어서, 이 노드도 같은 QoS(TRANSIENT_LOCAL)로 구독해야
"늦게 떠도 마지막 상태를 즉시 받는" 효과를 실제로 얻는다. 또한 토픽 이름을
전부 상대경로로 바꿔서, 실행 시 네임스페이스만 지정하면
(예: --ros-args -r __ns:=/robot5) 로봇이 몇 대여도 그대로 동작한다.

설계:
  - /robot_mode 를 구독해서, 상태가 "WAIT_COMMAND" 일 때만 회전한다.
    (이 상태에서는 explore_lite/Nav2 가 모두 정지해 있으므로 cmd_vel 을
     이 노드가 온전히 사용해도 충돌이 없다.)
  - 그 외 상태(PATROL/COOLDOWN/TRACK/RETURN)에서는 /set_ori 가 들어와도
    회전하지 않는다. (Nav2/target_nav 가 cmd_vel 을 써야 하므로, 이 노드가
    동시에 값을 쏘면 같은 토픽에서 충돌해 로봇이 제대로 움직이지 못한다.)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


# 이 상태일 때만 azimuth 가 회전 명령을 낸다.
# (최신 시나리오: ATTACK 상태가 사라지고 WAIT_COMMAND 하나로 통합됨.
#  조준이 필요한 건 WAIT_COMMAND(정렬 + 사격 준비 대기) 뿐이다.
#  PATROL/COOLDOWN/TRACK/RETURN 에서는 Nav2/explore_lite 가 cmd_vel 을
#  온전히 사용해야 하므로 azimuth 는 절대 발행하지 않는다.)
ACTIVE_MODES = ("WAIT_COMMAND",)


class AzimuthTracker(Node):
    def __init__(self):
        super().__init__('azimuth_tracker')

        # ---- 파라미터 ----
        self.declare_parameter('kp', 0.8)                 # P 제어 게인
        self.declare_parameter('max_angular', 0.3)        # rad/s, 최대 회전 속도
        self.declare_parameter('deadband', 0.05)          # 이 이하 오차는 무시(떨림 방지)
        self.declare_parameter('timeout_sec', 0.5)        # 이 시간 미수신 시 정지
        self.declare_parameter('control_hz', 20.0)        # 제어 루프 주파수
        self.declare_parameter('max_angular_accel', 1.0)  # rad/s^2, 부드러운 가감속
        self.declare_parameter('ema_alpha', 0.4)          # 저역통과 필터 (0~1)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        self.kp = self.get_parameter('kp').value
        self.max_angular = self.get_parameter('max_angular').value
        self.deadband = self.get_parameter('deadband').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.control_hz = self.get_parameter('control_hz').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.dt = 1.0 / self.control_hz
        self.max_delta_per_tick = self.max_angular_accel * self.dt

        # ---- 상태 변수 ----
        self.error_x = 0.0
        self.filtered_error = 0.0
        self.last_msg_time = None
        self.current_angular = 0.0   # 슬루레이트 제한용 (현재 발행 중인 각속도)

        # 현재 로봇 모드. ACTIVE_MODES(WAIT_COMMAND)일 때만 회전한다.
        self.robot_mode = "PATROL"

        # mission_manager 가 robot_mode 를 TRANSIENT_LOCAL 로 발행하므로,
        # 이 노드도 같은 durability 로 구독해야 늦게 떠도 마지막 상태를 즉시 받는다.
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, 'robot_mode', self.on_robot_mode, mode_qos)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Float32, 'aim/set_ori', self.on_set_ori, qos)

        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"AzimuthTracker Started (cmd_vel -> {cmd_vel_topic}, "
            f"활성 상태: {ACTIVE_MODES})"
        )

    def on_robot_mode(self, msg: String):
        prev = self.robot_mode
        self.robot_mode = msg.data

        if prev in ACTIVE_MODES and self.robot_mode not in ACTIVE_MODES:
            # 활성 상태를 벗어나는 순간, 잔여 회전이 남지 않도록 즉시 정지시킨다.
            self.current_angular = 0.0
            self._publish_twist(0.0)

    def on_set_ori(self, msg: Float32):
        """화면 중심 대비 좌우 오차(-1 ~ 1). 값과 수신 시각을 모두 기록."""
        self.error_x = msg.data
        self.last_msg_time = self.get_clock().now()

    def control_loop(self):
        """
        [버그 수정] 이전에는 비활성 상태에서도 target=0 을 계속 발행했다.
        값이 0 이어도 "발행 자체"가 계속 일어나면, Nav2/explore_lite 가 같은
        /robot5/cmd_vel 에 실제 주행 명령을 쏘는 순간과 겹쳐서 서로 덮어쓰기
        경쟁이 발생한다 (로봇이 순찰 중 미세하게 끊기거나 버벅이는 원인).

        그래서 비활성 상태(ACTIVE_MODES 가 아님)에서는 아예 발행하지 않고
        return 한다. Nav2/explore_lite 가 cmd_vel 을 100% 독점하게 된다.
        활성 상태를 "벗어나는 그 순간"의 정지 명령 1회는 on_robot_mode() 에서
        이미 별도로 보내고 있으므로, 여기서 또 신경 쓸 필요가 없다.
        """
        if self.robot_mode not in ACTIVE_MODES:
            self.current_angular = 0.0
            return   # 발행하지 않음 -> Nav2/explore_lite 가 cmd_vel 독점

        target = self._compute_target()

        # 슬루레이트 제한 (급격한 속도 변화 방지 -> 부드러운 회전)
        delta = target - self.current_angular
        delta = max(-self.max_delta_per_tick, min(self.max_delta_per_tick, delta))
        self.current_angular += delta

        self._publish_twist(self.current_angular)

    def _compute_target(self) -> float:
        """
        이번 tick 의 목표 각속도. 신호 없음/타임아웃/데드밴드 시 0.
        (robot_mode 활성 여부는 control_loop() 에서 이미 걸러지고 들어오므로
         여기서는 다시 체크하지 않는다 - 이 함수는 항상 ACTIVE_MODES 일 때만 호출됨)
        """
        if self.last_msg_time is None:
            return 0.0

        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            self.filtered_error = 0.0   # 필터 상태도 리셋
            return 0.0

        # EMA 저역통과 필터 (검출 노이즈 완화)
        self.filtered_error = (
            self.ema_alpha * self.error_x
            + (1.0 - self.ema_alpha) * self.filtered_error
        )

        e = self.filtered_error
        if abs(e) < self.deadband:
            return 0.0

        target = -self.kp * e
        return max(-self.max_angular, min(self.max_angular, target))

    def _publish_twist(self, angular_z: float):
        cmd = Twist()
        cmd.linear.x = 0.0    # 이동 없이 제자리 회전만
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = AzimuthTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_twist(0.0)   # 종료 시 반드시 정지
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()