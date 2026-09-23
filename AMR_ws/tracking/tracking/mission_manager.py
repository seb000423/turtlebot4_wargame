#!/usr/bin/env python3
"""
mission_manager.py
===================

[병합 내역]
judge_state_manager.py 의 좋은 설계 두 가지를 이 파일에 합쳤다:
  1) 모든 토픽을 상대경로로 작성 -> 실행 시 네임스페이스만 지정하면
     (예: --ros-args -r __ns:=/robot5) 로봇이 몇 대여도 코드 수정 없이 그대로 씀.
  2) HP 입력값 범위(0~3) 검증 추가, /robot_mode 를 TRANSIENT_LOCAL QoS 로 발행해서
     azimuth_tracker/target_nav 가 늦게 떠도 마지막 상태를 즉시 받게 함.

HP 입력 토픽 이름은 기존 그대로 유지한다 (my_hp, enemy_hp).
GUI 1/2/3/4 선택 로직은 이 버전에 없다 (이미 제거된 상태).

[상태 결정 규칙]
    내 HP <= 0            -> RETURN  (사망) -- 최우선, 어느 상태에서든 즉시 적용
    상대 HP <= 0          -> RETURN  (미션 완료) -- 최우선, 어느 상태에서든 즉시 적용
    (그 외에는 아래 전체 시나리오를 따른다)

[전체 흐름]
    PATROL
      -> /set_ori 수신 (적 발견) -> WAIT_COMMAND
    WAIT_COMMAND
      -> 자체적으로 /set_ori 값을 보고 정렬 판정
      -> 정렬되면 "fire_ready" 발행 (사격 준비 알림, 실제 발사는 외부에서 처리)
      -> "shoot_complete" 수신 -> COOLDOWN
      -> (정렬 전에 적을 놓치면 ENEMY_LOST_IN_WAIT_TIMEOUT 후 PATROL 복귀)
    COOLDOWN (15초)
      -> 즉시 explore 재개, 15초 동안 /set_ori 완전히 무시
      -> enemy_hp==1 이 되는 순간, 15초를 안 채워도 즉시 TRACK 전이
      -> enemy_hp 가 2 또는 3 이면 15초를 다 채운 뒤에만 PATROL 복귀
    TRACK
      -> 실제 접근은 target_nav.py 가 담당, "track_complete" 수신 시 PATROL 복귀
    RETURN
      -> Nav2 로 시작 좌표 복귀

[역할 분담]
  - azimuth_tracker.py : WAIT_COMMAND 상태에서만 set_ori 보고 조준 회전
  - target_nav.py      : TRACK 상태에서 회전+거리 접근을 직접 계산해 cmd_vel 발행
  - mission_manager.py : 그 외 모든 상태 관리 + RETURN 주행
"""

import subprocess
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState
from turtlebot4_wargame_msgs.msg import TeamStatus
from std_msgs.msg import Bool, Float32, Int32, String


class State(Enum):
    PATROL = "PATROL"              # 순찰 (오토슬램)
    WAIT_COMMAND = "WAIT_COMMAND"  # 적 포착, 조준 + 사격 준비 알림 + 완료 대기
    COOLDOWN = "COOLDOWN"          # 발사 완료 직후 15초, 적 무시하고 순찰 유지
    TRACK = "TRACK"                # 상대 HP 1 -> 마지막 좌표로 1회 접근
    RETURN = "RETURN"              # 내 사망 또는 상대 HP 0 -> 복귀


# ----------------------------------------------------------
# 속도 제한 (%). 0.0 = 제한 해제 (Nav2 SpeedLimit 관례값)
# ----------------------------------------------------------
PATROL_SPEED = 70.0
MISSION_SPEED = 0.0

# ----------------------------------------------------------
# 시간/판정 관련 상수
# ----------------------------------------------------------
DETECT_TIMEOUT = 0.5              # PATROL 에서 set_ori 이내 수신 -> 적 발견
ENEMY_LOST_IN_WAIT_TIMEOUT = 5.0  # WAIT_COMMAND 중 이 시간 이상 set_ori 없으면 PATROL 복귀
COOLDOWN_DURATION = 15.0          # 발사 완료 후 쿨다운 시간
ALIGN_DEADBAND = 0.2              # WAIT_COMMAND 에서 "정렬 완료" 판정 임계값 (테스트용으로 넉넉하게)

# ----------------------------------------------------------
# 장전/재장전 (UI 쪽 MAX_AMMO 상수와 반드시 일치시킬 것: frontend-*/src/lib/topics.ts)
# ----------------------------------------------------------
MAX_AMMO = 5
RELOAD_SECONDS = 10.0

# HP 입력값 유효 범위 (judge_state_manager 방식 - 범위 밖 값은 무시하고 경고)
HP_MIN = 0
HP_MAX = 3

# ----------------------------------------------------------
# RETURN 목표 좌표 (map 프레임 기준 시작 위치)
# ----------------------------------------------------------
HOME_X = 0.0
HOME_Y = 0.0
MAP_FRAME = "map"

# ----------------------------------------------------------
# 토픽/액션 이름 - 전부 상대경로로 작성한다.
# 노드를 실행할 때 네임스페이스를 지정하면 (예: --ros-args -r __ns:=/robot5)
# 아래 이름 전체에 자동으로 /robot5/ 가 붙는다. 로봇이 몇 대여도
# 코드 수정 없이 네임스페이스만 다르게 실행하면 서로 겹치지 않는다.
# ----------------------------------------------------------
TOPIC_SET_ORI = "aim/set_ori"
TOPIC_SHOOT_COMPLETE = "shoot_complete"
TOPIC_TRACK_COMPLETE = "track_complete"
TOPIC_MY_HP = "my_hp"
TOPIC_ENEMY_HP = "enemy_hp"
TOPIC_ROBOT_MODE = "robot_mode"
TOPIC_FIRE_READY = "fire_ready"
TOPIC_SPEED_LIMIT = "speed_limit"
TOPIC_EXPLORE_RESUME = "explore/resume"

# ----------------------------------------------------------
# turtlebot4_wargame-ui 연동 토픽 (휴먼인더루프 발사 승인).
# 이름/타입은 frontend-team-pc/src/lib/topics.ts, backend/app/config.py 와
# 반드시 일치해야 한다. TOPIC_FIRE_READY/TOPIC_SHOOT_COMPLETE 는 로봇<->아두이노
# 내부 핸드셰이크 전용이라 UI 계약에는 없다 (그대로 유지).
# ----------------------------------------------------------
TOPIC_AIM_COMPLETE = "aim/complete"        # Bool, 정렬 여부 (UI 표시용)
TOPIC_SHOOT_APPROVAL = "shoot_approval"    # Bool, 사람이 UI에서 발사 승인 클릭 시 True
TOPIC_STATUS = "status"                    # turtlebot4_wargame_msgs/TeamStatus
TOPIC_BATTERY_STATE = "battery_state"      # sensor_msgs/BatteryState (Create3 표준 토픽)

NAV_ACTION = "navigate_to_pose"


class MissionManager(Node):

    def __init__(self):
        super().__init__("mission_manager")

        # ----------------------------
        # 상태 변수
        # ----------------------------
        self.state = State.PATROL

        # 언도킹이 끝나고 첫 PATROL 진입(explore_lite 시작)이 실제로 될 때까지는
        # state_machine() 이 아무 전이도 하지 않는다 (언도킹 전에 적을 만나도
        # WAIT_COMMAND 로 새치기해서 넘어가는 것을 방지).
        self.mission_started = False

        self.last_enemy_time = None     # set_ori 마지막 수신 시각
        self.last_error_x = None        # set_ori 마지막 값 (정렬 판정용)

        self.fire_ready_sent = False    # 이번 WAIT_COMMAND 세션에서 fire_ready 발행 여부
        self.cooldown_until = None      # COOLDOWN 종료 예정 시각

        # 휴먼인더루프: UI에서 "발사 승인" 버튼을 눌렀는지 (WAIT_COMMAND 진입 시 리셋)
        self.shoot_approval_received = False

        # 배터리 % (battery_state 수신 전까지의 기본값). 실제 로봇에 없으면
        # 콜백이 그냥 호출되지 않을 뿐이라 안전하다.
        self.battery_pct = 100.0

        # 장전/재장전 상태 (UI ammo/reload_remaining 표시 + 발사 가능 여부 판정용)
        self.ammo = MAX_AMMO
        self.reload_until = 0.0  # time.time() 이 이 값보다 작으면 재장전 중

        # 심판 PC 가 보내주는 최신 HP. 초기값은 "전투 전".
        # my_hp 는 enemy_hp 와 대칭으로, 3에서 시작해서 맞을 때마다 깎이고
        # 0이 되면 사망이다 (예전처럼 "피격 횟수"를 세는 방식이 아님).
        self.my_hp = 3
        self.enemy_hp = 3

        self.return_goal_handle = None

        # ----------------------------
        # QoS
        # ----------------------------
        # judge_state_manager 방식: 상태 발행은 TRANSIENT_LOCAL 로 해서,
        # azimuth_tracker/target_nav 가 mission_manager 보다 늦게 떠도
        # 마지막으로 발행된 상태를 즉시 받을 수 있게 한다.
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 심판 PC 입력용 QoS (judge_state_manager 와 동일한 스타일)
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ----------------------------
        # Publisher
        # ----------------------------
        self.mode_pub = self.create_publisher(String, TOPIC_ROBOT_MODE, state_qos)
        self.fire_ready_pub = self.create_publisher(Bool, TOPIC_FIRE_READY, 10)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, TOPIC_SPEED_LIMIT, 10)
        self.resume_pub = self.create_publisher(Bool, TOPIC_EXPLORE_RESUME, 10)
        self.aim_complete_pub = self.create_publisher(Bool, TOPIC_AIM_COMPLETE, 10)
        # UI가 늦게 접속해도 마지막 상태를 바로 받도록 TRANSIENT_LOCAL (mode_pub 과 동일 이유)
        self.status_pub = self.create_publisher(TeamStatus, TOPIC_STATUS, state_qos)

        # ----------------------------
        # Subscriber
        # ----------------------------
        self.create_subscription(Float32, TOPIC_SET_ORI, self.set_ori_callback, 10)
        self.create_subscription(Bool, TOPIC_SHOOT_COMPLETE, self.shoot_complete_callback, 10)
        self.create_subscription(Bool, TOPIC_TRACK_COMPLETE, self.track_complete_callback, 10)
        self.create_subscription(Int32, TOPIC_MY_HP, self.my_hp_callback, input_qos)
        self.create_subscription(Int32, TOPIC_ENEMY_HP, self.enemy_hp_callback, input_qos)
        self.create_subscription(Bool, TOPIC_SHOOT_APPROVAL, self.shoot_approval_callback, input_qos)
        self.create_subscription(BatteryState, TOPIC_BATTERY_STATE, self.battery_state_callback, input_qos)

        # ----------------------------
        # 도킹 상태 / 언도킹
        # ----------------------------
        # 도킹 스테이션에 물려있는 채로 explore_lite 가 주행을 시도하면
        # 바퀴만 헛돈다. 그래서 첫 PATROL 진입 전에 반드시 언도킹부터 한다.
        self.is_docked = None
        self.create_subscription(DockStatus, "dock_status", self.dock_status_callback, 10)
        self.undock_client = ActionClient(self, Undock, "undock")

        # ----------------------------
        # Nav2
        # ----------------------------
        self.nav_client = ActionClient(self, NavigateToPose, NAV_ACTION)

        # ----------------------------
        # Timer
        # ----------------------------
        self.create_timer(0.1, self.state_machine)

        # ----------------------------
        # 시작 (언도킹 먼저, 끝나면 explore_lite + PATROL 진입)
        # ----------------------------
        # dock_status 가 아직 한 번도 안 왔을 수 있으니(막 스핀 시작), 잠깐 기다렸다가
        # 판단한다. 1초 안에도 안 오면 "모르니 일단 언도킹 시도"로 안전하게 처리한다.
        self._undock_wait_timer = self.create_timer(1.0, self._begin_undock_sequence)

        self.get_logger().info(
            "Mission Manager Started "
            f"(COOLDOWN={COOLDOWN_DURATION}초, "
            f"WAIT_COMMAND 유실 타임아웃={ENEMY_LOST_IN_WAIT_TIMEOUT}초, "
            f"토픽 상대경로 적용됨 - 네임스페이스와 함께 실행하세요, "
            f"언도킹 후 PATROL 시작)"
        )

    # ==========================================
    # 언도킹 (최초 1회, PATROL 진입 전)
    # ==========================================
    def dock_status_callback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def _begin_undock_sequence(self):
        self._undock_wait_timer.cancel()
        self._undock_finished = False

        if self.is_docked is False:
            self.get_logger().info("이미 언도킹된 상태 -> 언도킹 생략")
            self._finish_undock_once()
            return

        self.get_logger().info("도킹 스테이션에서 언도킹 시작...")
        if not self.undock_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("Undock 액션 서버 연결 실패 -> 언도킹 없이 PATROL 진행")
            self._finish_undock_once()
            return

        # [버그 수정, 실측 확인됨] 로봇이 이미 도킹 해제된 상태로 Undock goal 을
        # 보내면, goal 은 accept 되지만 결과(get_result_async)가 영영 안 돌아오는
        # 경우가 있다 (몇 시간이 지나도 응답 없음). 결과 대기에 타임아웃을 걸어서,
        # 응답이 없으면 "이미 언도킹된 것"으로 보고 그냥 PATROL 로 진행한다.
        self._undock_timeout_timer = self.create_timer(10.0, self._on_undock_timeout)

        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._on_undock_goal_response)

    def _on_undock_timeout(self):
        if self._undock_finished:
            return
        self.get_logger().warn(
            "Undock 결과 응답이 10초 넘게 없음 (이미 언도킹된 상태로 추정) -> PATROL 진행"
        )
        self._finish_undock_once()

    def _on_undock_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Undock goal 오류: {exc} -> 언도킹 없이 PATROL 진행")
            self._finish_undock_once()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Undock goal 거부됨 -> 언도킹 없이 PATROL 진행")
            self._finish_undock_once()
            return

        goal_handle.get_result_async().add_done_callback(self._on_undock_result)

    def _on_undock_result(self, future):
        self.get_logger().info("언도킹 완료")
        self._finish_undock_once()

    def _finish_undock_once(self):
        """언도킹 시퀀스가 어떤 경로로 끝나든(성공/실패/타임아웃) 딱 한 번만 PATROL 로 넘어가게 한다."""
        if self._undock_finished:
            return
        self._undock_finished = True
        if hasattr(self, "_undock_timeout_timer"):
            self._undock_timeout_timer.cancel()
        self._start_patrol_after_undock()

    def _start_patrol_after_undock(self):
        self.start_explore_process()
        self.enter_patrol()
        self.mission_started = True

    # ==========================================
    # 콜백
    # ==========================================
    def set_ori_callback(self, msg: Float32):
        """적 검출 시에만 발행되는 좌우 오차(-1~1). 값과 수신 시각을 모두 기록."""
        self.last_error_x = msg.data
        self.last_enemy_time = time.time()

    def shoot_complete_callback(self, msg: Bool):
        """Shooter(라즈베리파이) 쪽에서 발사가 끝났음을 알려온다."""
        if self.state != State.WAIT_COMMAND:
            return
        if not msg.data:
            return

        self.get_logger().info("사격 완료 신호 수신 (shoot_complete) -> COOLDOWN")
        self.enter_cooldown()

    def shoot_approval_callback(self, msg: Bool):
        """
        휴먼인더루프: UI(팀 PC)에서 사람이 "발사 승인" 버튼을 눌렀을 때 True 로 온다.
        WAIT_COMMAND 가 아닐 때 온 승인은 무의미하므로 무시한다 (예: 늦게 도착한 이전
        교전의 승인 메시지가 다음 PATROL 중에 뒤늦게 들어오는 경우 방지).
        """
        if not msg.data:
            return
        if self.state != State.WAIT_COMMAND:
            return
        if self.shoot_approval_received:
            return
        self.shoot_approval_received = True
        self.get_logger().info("UI 발사 승인 수신 (shoot_approval)")

    def battery_state_callback(self, msg: BatteryState):
        # percentage 가 0~1 이 아니라 NaN/미지원인 드라이버도 있어 방어적으로 처리한다.
        pct = msg.percentage
        if pct is None or pct != pct:  # NaN 체크 (pct != pct 는 NaN 일 때만 True)
            return
        self.battery_pct = float(pct) * 100.0

    def track_complete_callback(self, msg: Bool):
        if self.state != State.TRACK:
            return
        if not msg.data:
            return
        self.get_logger().info("TRACK 완료 (track_complete) -> PATROL 복귀")
        self.enter_patrol()

    def my_hp_callback(self, msg: Int32):
        """
        [judge_state_manager 방식 반영] 범위(0~3) 밖 값은 무시하고 경고만 남긴다.
        """
        value = int(msg.data)
        if not HP_MIN <= value <= HP_MAX:
            self.get_logger().warning(
                f"잘못된 my_hp 수신: {value} (허용 범위 {HP_MIN}~{HP_MAX}) -> 무시"
            )
            return

        if value != self.my_hp:
            self.get_logger().info(f"[HP] my_hp: {self.my_hp} -> {value}")
        self.my_hp = value

    def enemy_hp_callback(self, msg: Int32):
        value = int(msg.data)
        if not HP_MIN <= value <= HP_MAX:
            self.get_logger().warning(
                f"잘못된 enemy_hp 수신: {value} (허용 범위 {HP_MIN}~{HP_MAX}) -> 무시"
            )
            return

        if value != self.enemy_hp:
            self.get_logger().info(f"[HP] enemy_hp: {self.enemy_hp} -> {value}")
        self.enemy_hp = value

    # ==========================================
    # 헬퍼
    # ==========================================
    def enemy_elapsed(self) -> float:
        if self.last_enemy_time is None:
            return float("inf")
        return time.time() - self.last_enemy_time

    def enemy_visible(self) -> bool:
        return self.enemy_elapsed() <= DETECT_TIMEOUT

    def is_aligned(self) -> bool:
        """지금 화면 중앙에 정렬됐는지(조준 완료) 판단. set_ori 값 자체를 본다."""
        if self.last_error_x is None:
            return False
        if not self.enemy_visible():
            return False
        return abs(self.last_error_x) < ALIGN_DEADBAND

    def in_cooldown(self) -> bool:
        if self.cooldown_until is None:
            return False
        return time.time() < self.cooldown_until

    def reload_remaining(self) -> float:
        return max(0.0, self.reload_until - time.time())

    def can_fire(self) -> bool:
        """탄약이 남아있고 재장전 중이 아닐 때만 발사 가능. UI가 이미 버튼을 막아주지만,
        승인 메시지가 그래도 들어올 경우를 대비한 서버(로봇) 쪽 안전장치."""
        return self.ammo > 0 and self.reload_remaining() <= 0.0

    def register_shot_fired(self):
        self.ammo = max(0, self.ammo - 1)
        self.reload_until = time.time() + RELOAD_SECONDS

    # ==========================================
    # explore_lite / 속도 / 상태 발행
    # ==========================================
    def start_explore_process(self):
        """
        explore_lite 를 최초 1회만 실행. 이후로는 resume 토픽으로만 제어한다.
        launch 자체의 namespace 인자는, 이 노드가 실행된 네임스페이스를
        그대로 가져와서 넘긴다 (로봇이 몇 대여도 자동으로 맞음).
        """
        robot_ns = self.get_namespace().strip("/")  # "/robot5" -> "robot5"

        self.get_logger().info(
            f"explore_lite 실행 (namespace={robot_ns or '(none)'}, 최초 1회)"
        )
        try:
            cmd = ["ros2", "launch", "explore_lite", "explore.launch.py"]
            if robot_ns:
                cmd.append(f"namespace:={robot_ns}")
            self.explore_process = subprocess.Popen(cmd)
        except Exception as exc:
            self.get_logger().error(f"explore_lite 실행 실패: {exc}")
            self.explore_process = None

    def set_explore_resume(self, resume: bool):
        msg = Bool()
        msg.data = resume
        self.resume_pub.publish(msg)
        self.get_logger().info(f"explore/resume = {resume}")

    def set_speed_limit(self, percentage: float):
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = percentage
        self.speed_limit_pub.publish(msg)
        self.get_logger().info(f"speed_limit = {percentage}%")

    def publish_mode(self):
        msg = String()
        msg.data = self.state.value
        self.mode_pub.publish(msg)

    def compute_ui_mission_state(self) -> str:
        """
        내부 State(PATROL/WAIT_COMMAND/COOLDOWN/TRACK/RETURN)를 turtlebot4_wargame-ui가
        아는 어휘(IDLE/CHASE/AIM/WAIT_APPROVAL/FIRE/RETURN/DISABLED)로 변환한다.
        AimPanel.tsx 는 정확히 "WAIT_APPROVAL" 문자열일 때만 승인 버튼을 띄우므로,
        이 매핑이 틀리면 버튼 자체가 안 뜬다 - 절대 오타 나면 안 되는 부분.
        """
        if self.my_hp <= 0:
            return "DISABLED"
        if not self.mission_started:
            return "IDLE"
        if self.state == State.PATROL:
            return "CHASE"
        if self.state == State.WAIT_COMMAND:
            if self.fire_ready_sent:
                return "FIRE"
            if self.is_aligned():
                return "WAIT_APPROVAL"
            return "AIM"
        if self.state == State.COOLDOWN:
            return "CHASE"
        if self.state == State.TRACK:
            return "CHASE"
        if self.state == State.RETURN:
            return "RETURN"
        return self.state.value

    def publish_status(self):
        msg = TeamStatus()
        msg.self_hp = int(self.my_hp)
        msg.battery = float(self.battery_pct)
        msg.ammo = int(self.ammo)
        msg.reload_remaining = float(self.reload_remaining())
        msg.mission_state = self.compute_ui_mission_state()
        msg.robot_destroyed = self.my_hp <= 0
        msg.timestamp = time.time()
        self.status_pub.publish(msg)

    def publish_aim_complete(self):
        """
        [버그 수정] 예전엔 WAIT_COMMAND 안에서만 발행해서, 발사 후 PATROL로
        돌아가도 UI에 "정렬됨" 배지가 낡은 값 그대로 남아있었다. 매 tick마다
        무조건 발행해서 WAIT_COMMAND 가 아니면 항상 False 로 리셋되게 한다.
        """
        msg = Bool()
        msg.data = self.state == State.WAIT_COMMAND and self.is_aligned()
        self.aim_complete_pub.publish(msg)

    # ==========================================
    # 상태 진입
    # ==========================================
    def enter_patrol(self):
        self.state = State.PATROL
        self.last_enemy_time = None
        self.last_error_x = None
        self.fire_ready_sent = False
        self.shoot_approval_received = False
        self.cooldown_until = None
        self.return_goal_handle = None

        self.set_explore_resume(True)
        self.set_speed_limit(PATROL_SPEED)

        self.get_logger().info("=== PATROL ===")

    def enter_wait_command(self):
        self.state = State.WAIT_COMMAND
        self.fire_ready_sent = False
        self.shoot_approval_received = False  # 매 교전마다 새로 승인받아야 함

        self.set_explore_resume(False)
        self.set_speed_limit(MISSION_SPEED)

        self.get_logger().info("=== WAIT_COMMAND (조준 시작) ===")

    def enter_cooldown(self):
        """
        발사 완료 직후 진입.
        - 즉시 explore_lite 재개해서 순찰 이동을 계속한다.
        - 15초 동안 set_ori 를 완전히 무시한다.
        """
        self.state = State.COOLDOWN
        self.cooldown_until = time.time() + COOLDOWN_DURATION

        self.set_explore_resume(True)
        self.set_speed_limit(PATROL_SPEED)

        self.get_logger().info(
            f"=== COOLDOWN 진입 ({COOLDOWN_DURATION}초, 적 무시하며 순찰 유지) ==="
        )

    def enter_track(self):
        """TRACK: 실제 접근은 target_nav.py 가 robot_mode 를 보고 수행."""
        self.state = State.TRACK
        self.cooldown_until = None

        self.set_explore_resume(False)
        self.set_speed_limit(MISSION_SPEED)

        self.get_logger().info("=== TRACK: 상대 HP 1, 마지막 좌표로 접근 ===")

    def enter_return(self, reason: str):
        self.state = State.RETURN
        self.cooldown_until = None

        self.set_explore_resume(False)
        self.set_speed_limit(MISSION_SPEED)

        self.get_logger().info(f"=== RETURN: {reason} -> ({HOME_X}, {HOME_Y}) 복귀 ===")
        self.send_return_goal()

    # ==========================================
    # RETURN: Nav2 goal
    # ==========================================
    def send_return_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 액션 서버 연결 실패 (RETURN goal 전송 못 함)")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = HOME_X
        goal.pose.pose.position.y = HOME_Y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_return_goal_response)

    def _on_return_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"RETURN goal 오류: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("RETURN goal 거부됨")
            return

        self.return_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_return_goal_result)

    def _on_return_goal_result(self, future):
        if self.state != State.RETURN:
            return
        self.get_logger().info("RETURN 완료 (홈 도착, HP 리셋 대기 중)")
        self.return_goal_handle = None

    # ==========================================
    # 상태 머신 (10Hz)
    # ==========================================
    def state_machine(self):

        # 언도킹 + 최초 PATROL 진입 전에는 아무 전이도 하지 않는다.
        if not self.mission_started:
            return

        # ---------- 최우선 규칙: 어느 상태든 무조건 RETURN ----------
        if self.my_hp <= 0 and self.state != State.RETURN:
            self.enter_return("내 HP 0 (사망)")
            self.publish_mode()
            self.publish_status()
            self.publish_aim_complete()
            return

        if self.enemy_hp <= 0 and self.state != State.RETURN:
            self.enter_return("상대 HP 0 (미션 완료)")
            self.publish_mode()
            self.publish_status()
            self.publish_aim_complete()
            return

        # ---------- PATROL ----------
        if self.state == State.PATROL:
            if self.enemy_visible():
                self.get_logger().info("적 발견 -> WAIT_COMMAND")
                self.enter_wait_command()

        # ---------- WAIT_COMMAND ----------
        elif self.state == State.WAIT_COMMAND:

            aligned = self.is_aligned()

            # [휴먼인더루프] 정렬됐다고 바로 쏘지 않는다. UI에서 사람이
            # shoot_approval 을 눌러야만 실제로 fire_ready 를 내보낸다.
            # (정렬만 되고 승인 대기 중인 구간 = compute_ui_mission_state() 의 WAIT_APPROVAL)
            # can_fire() 는 UI가 이미 버튼을 막아주는 것과 별개로 로봇 쪽에서도 한 번
            # 더 확인하는 안전장치 - 탄약 0이거나 재장전 중이면 승인이 와도 대기만 한다
            # (승인 자체를 취소하지 않으므로 재장전 끝나면 그 자리에서 바로 발사된다).
            if not self.fire_ready_sent and aligned and self.shoot_approval_received and self.can_fire():
                msg = Bool()
                msg.data = True
                self.fire_ready_pub.publish(msg)
                self.fire_ready_sent = True
                self.register_shot_fired()
                self.get_logger().info(
                    f"발사 승인 확인 -> fire_ready 발행 (사격 준비 알림, shoot_complete 대기, "
                    f"잔탄 {self.ammo}/{MAX_AMMO})"
                )

            if self.enemy_elapsed() >= ENEMY_LOST_IN_WAIT_TIMEOUT:
                self.get_logger().warn(
                    f"WAIT_COMMAND 중 적 유실 "
                    f"({self.enemy_elapsed():.1f}초 >= {ENEMY_LOST_IN_WAIT_TIMEOUT}초) "
                    "-> PATROL 복귀"
                )
                self.enter_patrol()

            # (COOLDOWN 전이는 shoot_complete_callback 에서 처리)

        # ---------- COOLDOWN ----------
        elif self.state == State.COOLDOWN:
            # 상대 HP가 1이 된 순간, 15초를 다 채우지 않고 즉시 TRACK 으로
            # 넘어간다. 단, "이 15초 동안은 재사격/재조준을 못 한다"는 성격
            # (set_ori 를 무시하고 WAIT_COMMAND 로 안 돌아가는 것)은 그대로 유지된다.
            # -> enemy_hp 가 2/3 인 일반적인 경우에는 이전과 동일하게 15초를
            #    꽉 채운 뒤에만 PATROL 로 복귀한다.
            if self.enemy_hp == 1:
                self.get_logger().info(
                    "COOLDOWN 중 상대 HP 1 감지 -> 대기 없이 즉시 TRACK"
                )
                self.enter_track()
            elif not self.in_cooldown():
                self.get_logger().info(
                    f"COOLDOWN 종료 (15초 경과), 상대 HP {self.enemy_hp} -> PATROL 복귀"
                )
                self.enter_patrol()

        # ---------- TRACK ----------
        elif self.state == State.TRACK:
            pass  # track_complete_callback 에서 전이 처리

        self.publish_mode()
        self.publish_status()
        self.publish_aim_complete()

    # ==========================================
    # 종료 처리
    # ==========================================
    def destroy_node(self):
        if self.return_goal_handle is not None:
            try:
                self.return_goal_handle.cancel_goal_async()
            except Exception:
                pass
        if getattr(self, "explore_process", None) is not None:
            try:
                self.explore_process.terminate()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MissionManager()
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