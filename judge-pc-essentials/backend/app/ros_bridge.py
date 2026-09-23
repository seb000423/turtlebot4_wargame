"""
FastAPI 프로세스 안에서 같이 도는 ROS2 노드.

역할 2가지:
1. REST로 받은 경기 제어 요청을 실제 ROS2 토픽 publish로 옮긴다 (/judge/match_state).
2. /judge/hit_event(judge_hit_detect.py가 publish)를 구독해서 HP를 깎고 /red|blue/status를
   publish한다 — 심판 UI 스코어보드의 HP 도트, 팀 PC의 HP 표시가 실제로 움직이려면 이
   status가 있어야 한다. 판정(HIT인지 아닌지) 자체는 이 노드가 아니라 judge_hit_detect.py의
   몫이고, 여기서는 "HIT 이벤트 1건 = 그 팀 HP -1"이라는 게임 규칙만 적용한다.

/judge/match_state, /red|blue/status 는 TRANSIENT_LOCAL(래치) QoS로 publish한다 — 경기
중간에 새로 붙는 구독자(팀 PC 새로고침, 심판 PC 재접속 등)도 마지막 상태를 바로 받아야
하기 때문(ROS1의 latched topic과 동일한 목적).
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .config import HIT_EVENT_TOPIC, MATCH_STATE_TOPIC, MAX_HP, STATE_IDLE, TEAM_COLORS, team_status_topic

_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# HitEvent/TeamStatus는 turtlebot4_wargame_msgs 패키지가 워크스페이스에 빌드되어 있어야 임포트된다
# (ros2_ws/src/turtlebot4_wargame_msgs). 없으면 HP 연동 없이 match_state publish만 동작하도록 degrade.
try:
    from turtlebot4_wargame_msgs.msg import HitEvent, TeamStatus

    _HAS_TURTLEBOT4_WARGAME_MSGS = True
except ImportError:
    _HAS_TURTLEBOT4_WARGAME_MSGS = False


class JudgeRosNode(Node):
    def __init__(self, on_hit: Optional[Callable[[str, int, int], None]] = None) -> None:
        """on_hit(victim_color, blue_hp_after, red_hp_after) — hit 처리 후 match_store에
        기록할 수 있도록 콜백을 받는다(순환 임포트 피하려고 여기서 store를 직접 안 씀)."""
        super().__init__("judge_backend")
        self._on_hit = on_hit
        self.hp = {c: MAX_HP for c in TEAM_COLORS}

        self._match_state_pub = self.create_publisher(String, MATCH_STATE_TOPIC, _LATCHED_QOS)
        self.publish_match_state(STATE_IDLE)

        self._status_pubs = {}
        if _HAS_TURTLEBOT4_WARGAME_MSGS:
            for color in TEAM_COLORS:
                self._status_pubs[color] = self.create_publisher(TeamStatus, team_status_topic(color), _LATCHED_QOS)
            self._hit_sub = self.create_subscription(HitEvent, HIT_EVENT_TOPIC, self._handle_hit, 10)
            self.reset_hp()
        else:
            self.get_logger().warn(
                "turtlebot4_wargame_msgs를 찾을 수 없음 — /red|blue/status publish와 hit_event 구독을 건너뜀"
                " (ros2_ws/src/turtlebot4_wargame_msgs를 colcon build 후 source 했는지 확인)"
            )

    def publish_match_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._match_state_pub.publish(msg)
        self.get_logger().info(f"{MATCH_STATE_TOPIC} -> {state}")

    def reset_hp(self) -> None:
        self.hp = {c: MAX_HP for c in TEAM_COLORS}
        for color in TEAM_COLORS:
            self._publish_status(color)

    def _publish_status(self, color: str) -> None:
        if color not in self._status_pubs:
            return
        msg = TeamStatus()
        msg.self_hp = self.hp[color]
        msg.battery = 100.0
        msg.ammo = 0
        msg.reload_remaining = 0.0
        msg.mission_state = "IDLE"
        msg.robot_destroyed = self.hp[color] <= 0
        msg.timestamp = time.time()
        self._status_pubs[color].publish(msg)
        self.get_logger().info(f"{team_status_topic(color)} -> self_hp={self.hp[color]}")

    def _handle_hit(self, msg) -> None:
        victim = msg.victim
        if victim not in self.hp:
            self.get_logger().warn(f"알 수 없는 victim '{victim}' — hit_event 무시")
            return
        self.hp[victim] = max(0, self.hp[victim] - 1)
        self._publish_status(victim)
        if self._on_hit is not None:
            self._on_hit(victim, self.hp["blue"], self.hp["red"])


class RosBridge:
    """rclpy.init/spin/shutdown 생명주기를 FastAPI lifespan에서 다루기 쉽게 감싼 래퍼.

    spin은 별도 스레드에서 돈다 — FastAPI(uvicorn)의 asyncio 이벤트 루프와 별개로 ROS2
    디스커버리/콜백을 계속 처리해야 하기 때문. publisher.publish()는 그 스레드가 아닌
    FastAPI 요청 핸들러(메인 스레드)에서 호출해도 안전하다(rclpy publisher는 스레드 세이프).
    """

    def __init__(self) -> None:
        self.node: JudgeRosNode | None = None
        self._thread: threading.Thread | None = None

    def start(self, on_hit: Optional[Callable[[str, int, int], None]] = None) -> JudgeRosNode:
        rclpy.init()
        self.node = JudgeRosNode(on_hit=on_hit)
        self._thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._thread.start()
        return self.node

    def stop(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        rclpy.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
