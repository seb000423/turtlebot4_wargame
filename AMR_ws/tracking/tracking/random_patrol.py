#!/usr/bin/env python3
"""
random_patrol.py (고정 웨이포인트 버전)
========================================

기존 random_patrol.py 를 대체한다. 이전 버전은 로봇 주변 반경 1~2m 안에서
매번 완전히 무작위 좌표를 생성했지만, 이번 버전은 사용자가 미리 정해둔
5개의 고정 지점(GOAL_POSES)을 순서만 섞어서(shuffle) 순회한다.

[주의] GOAL_POSES 좌표는 예시값이다. 실제 맵에 맞는 좌표로 반드시
교체해야 한다 (RViz 에서 "2D Pose Estimate"로 원하는 지점을 찍어보고
좌표를 읽어서 넣는 걸 권장).

동작:
1. /robot_mode == PATROL 감시
2. /odom 이동량 감시 (기존 버전과 동일한 방식)
3. NO_PROGRESS_TIMEOUT 동안 충분히 못 움직이면(explore_lite 정체)
   -> explore_lite 일시정지 -> 고정 지점들을 랜덤 순서로 섞어서
      NavigateThroughPoses 액션 하나로 한 번에 순회 시작
4. 전체 경로를 완주하면 다시 섞어서 반복 (PATROL 상태인 동안 무한 반복)
5. PATROL 상태를 벗어나면 즉시 취소

[TurtleBot4Navigator 사용 범위 - 중요]
TurtleBot4Navigator 는 PoseStamped 를 만드는 용도(getPoseStamped)로만
딱 1번 사용하고 바로 정리(destroy_node)한다. TurtleBot4Navigator 의
startThroughPoses()/isTaskComplete() 같은 폴링 방식 메서드는 쓰지 않는다
(이 노드는 /robot_mode, /odom 을 동시에 계속 구독해야 하므로, 그 메서드들
내부의 spin 방식과 우리 노드의 콜백 처리가 서로 얽혀 멈출 위험이 있다).
실제 이동은 nav2_msgs/action/NavigateThroughPoses 를 raw ActionClient 로
비동기 콜백 방식으로 직접 호출한다 (mission_manager.py, target_nav.py 와
동일한 스타일).
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateThroughPoses

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)


# ==========================================================
# 고정 순찰 지점 (예시 좌표 - 반드시 실제 맵에 맞게 교체할 것)
# ==========================================================
GOAL_POSES = [
    ([-0.02, -1.39], TurtleBot4Directions.NORTH),
    ([-1.65, -1.10], TurtleBot4Directions.EAST),
    ([-2.77, -1.29], TurtleBot4Directions.SOUTH),
    ([-1.20, -2.00], TurtleBot4Directions.WEST),
    ([-0.50, -0.80], TurtleBot4Directions.NORTH),
]

# ==========================================================
# Topic / Action (상대경로 - mission_manager.py 와 동일한 방식)
# ==========================================================
ROBOT_MODE_TOPIC = "robot_mode"
ODOM_TOPIC = "odom"
EXPLORE_RESUME_TOPIC = "explore/resume"
NAV_THROUGH_POSES_ACTION = "navigate_through_poses"

# ==========================================================
# 자동 전환 조건 (기존 버전과 동일)
# ==========================================================
CHECK_INTERVAL = 2.0
NO_PROGRESS_TIMEOUT = 20.0    # PATROL 에서 이 시간(초) 동안
NO_PROGRESS_DISTANCE = 0.3    # 이 거리(m) 미만 이동이면 정체로 판단


class RandomPatrol(Node):

    def __init__(self):
        super().__init__("random_patrol")

        self.robot_mode = None
        self.random_active = False

        self.current_position = None
        self.progress_baseline = None
        self.no_progress_since = None

        self.nav_goal_handle = None
        self.nav_request_in_progress = False

        # ---- 미리 만들어둔 PoseStamped 목록 (좌표는 고정, 순서만 매번 섞음) ----
        self.all_poses = self._build_poses()

        # ---- Publisher / Subscriber ----
        self.resume_pub = self.create_publisher(Bool, EXPLORE_RESUME_TOPIC, 10)

        # mission_manager 가 robot_mode 를 TRANSIENT_LOCAL 로 발행하므로 맞춰서 구독
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, ROBOT_MODE_TOPIC, self.mode_callback, mode_qos)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)

        # ---- Nav2 액션 클라이언트 ----
        self.nav_client = ActionClient(self, NavigateThroughPoses, NAV_THROUGH_POSES_ACTION)

        self.create_timer(CHECK_INTERVAL, self.check_patrol_progress)

        self.get_logger().info(
            f"Random Patrol Started (고정 웨이포인트 {len(self.all_poses)}개, "
            f"정체 판단: {NO_PROGRESS_TIMEOUT:.0f}초 / {NO_PROGRESS_DISTANCE}m)"
        )

    # ==========================================================
    # 고정 웨이포인트 -> PoseStamped 변환 (TurtleBot4Navigator 는 여기서만 사용)
    # ==========================================================
    def _build_poses(self):
        """
        GOAL_POSES(좌표+방향)를 PoseStamped 리스트로 한 번만 변환한다.
        TurtleBot4Navigator 를 이 목적 하나로만 잠깐 만들었다가 바로 정리한다
        (내비게이션/폴링 기능은 전혀 쓰지 않음 - getPoseStamped() 변환만 사용).
        """
        helper = TurtleBot4Navigator()
        try:
            poses = [
                helper.getPoseStamped(position, direction)
                for position, direction in GOAL_POSES
            ]
        finally:
            helper.destroy_node()
        return poses

    # ==========================================================
    # Subscribers
    # ==========================================================
    def mode_callback(self, msg: String):
        previous_mode = self.robot_mode
        self.robot_mode = msg.data

        if self.robot_mode != "PATROL":
            if self.random_active:
                self.get_logger().info(f"PATROL 종료({self.robot_mode}) -> 랜덤 순찰 중지")
            self.stop_random_patrol(resume_explore=False)
            return

        if previous_mode != "PATROL":
            self.reset_progress()
            self.get_logger().info("PATROL 감지 -> explore_lite 진행 감시 시작")

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_position = (p.x, p.y)

    # ==========================================================
    # explore_lite 진행 감시 (기존 버전과 동일)
    # ==========================================================
    def reset_progress(self):
        self.progress_baseline = None
        self.no_progress_since = None

    def check_patrol_progress(self):
        if self.robot_mode != "PATROL":
            self.reset_progress()
            return
        if self.random_active:
            self.reset_progress()
            return
        if self.current_position is None:
            return

        if self.progress_baseline is None:
            self.progress_baseline = self.current_position
            self.no_progress_since = time.time()
            return

        dx = self.current_position[0] - self.progress_baseline[0]
        dy = self.current_position[1] - self.progress_baseline[1]
        moved = math.hypot(dx, dy)

        if moved >= NO_PROGRESS_DISTANCE:
            self.progress_baseline = self.current_position
            self.no_progress_since = time.time()
            return

        elapsed = time.time() - self.no_progress_since
        if elapsed < NO_PROGRESS_TIMEOUT:
            return

        self.get_logger().warn(
            f"PATROL에서 {elapsed:.0f}초 동안 {moved:.2f}m 이동 "
            "-> explore_lite 정지 후 고정 웨이포인트 순찰 시작"
        )
        self.start_random_patrol()

    # ==========================================================
    # explore_lite 제어
    # ==========================================================
    def set_explore_resume(self, resume: bool):
        msg = Bool()
        msg.data = resume
        self.resume_pub.publish(msg)
        self.get_logger().info(f"explore/resume = {resume}")

    # ==========================================================
    # 고정 웨이포인트 순찰
    # ==========================================================
    def start_random_patrol(self):
        if self.robot_mode != "PATROL":
            return
        if self.random_active:
            return

        self.random_active = True
        self.reset_progress()
        self.set_explore_resume(False)

        self.get_logger().info("=== 고정 웨이포인트 순찰 시작 ===")
        self.send_shuffled_route()

    def stop_random_patrol(self, resume_explore: bool):
        self.random_active = False
        self.nav_request_in_progress = False

        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.nav_goal_handle = None

        self.reset_progress()

        if resume_explore and self.robot_mode == "PATROL":
            self.set_explore_resume(True)

    def is_random_running(self) -> bool:
        return self.random_active and self.robot_mode == "PATROL"

    def send_shuffled_route(self):
        """저장해둔 PoseStamped 목록을 랜덤 순서로 섞어서 한 번에 순회 명령."""
        if not self.is_random_running():
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("NavigateThroughPoses 서버 연결 실패 -> 2초 후 재시도")
            self.create_timer(2.0, self._retry_route_once)
            return

        route = self.all_poses.copy()
        random.shuffle(route)

        goal = NavigateThroughPoses.Goal()
        goal.poses = route

        self.get_logger().info(f"=== 순찰 경로 전송 ({len(route)}개 지점, 순서 랜덤) ===")

        self.nav_request_in_progress = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_route_response)

    def _retry_route_once(self):
        if self.is_random_running():
            self.send_shuffled_route()

    def _on_route_response(self, future):
        self.nav_request_in_progress = False

        goal_handle = future.result()
        if not self.is_random_running():
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("순찰 경로 goal 거부됨 -> 재시도")
            self.send_shuffled_route()
            return

        self.nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_route_result)

    def _on_route_result(self, future):
        self.nav_goal_handle = None

        if not self.is_random_running():
            return

        self.get_logger().info("=== 순찰 경로 완주(또는 종료) -> 다시 섞어서 재시작 ===")
        self.send_shuffled_route()

    # ==========================================================
    # Shutdown
    # ==========================================================
    def destroy_node(self):
        self.stop_random_patrol(resume_explore=False)
        super().destroy_node()


def main():
    rclpy.init()
    node = RandomPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()