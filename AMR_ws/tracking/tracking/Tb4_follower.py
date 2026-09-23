#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
TurtleBot4 실시간 추종(Follower) 노드 - SLAM 버전 (검토/수정본)
==============================================================================
[개요]
  욜로(YOLO) 팀이 publish하는 상대 TurtleBot4의 map 기준 절대 좌표
  (geometry_msgs/PoseStamped)를 구독하여, 항상 약 1m 간격을 유지하며
  cmd_vel 직접 제어(P 제어기)로 부드럽게 추종하는 노드.

  ※ SLAM(slam_toolbox) 기반 버전입니다.
    - 사전 지도 없이 SLAM이 실시간으로 map을 만들며 map→odom→base_link TF를 발행
    - AMCL 전용 기능인 setInitialPose는 사용하지 않음
    - turtlebot4_navigation 패키지 의존성 제거됨 (도킹/언도킹 로직 없음)
      필요한 건 nav2_simple_commander(BasicNavigator)뿐이며, 이는 대부분
      Nav2 설치에 이미 포함되어 있음. 도킹 자동화가 필요하면 별도의
      mission_manager 노드 등에서 turtlebot4_navigation을 다시 붙이는 걸 권장.

[이번 수정에서 반영한 사항]
  (1) 네임스페이스 대응: map_frame / base_frame 을 파라미터로 분리
      (예: robot1/base_link 처럼 네임스페이스가 붙는 경우 대응)
  (2) QoS 선택 파라미터화: 욜로팀이 RELIABLE로 publish하면 BEST_EFFORT와
      불일치하여 메시지를 못 받는 흔한 함정 → reliable/best_effort 선택 가능
  (3) Nav2 활성화 대기 안전화: waitUntilNav2Active 실패/무한대기 대비 예외 처리
  (4) 거리 구간 로직 및 주석 정리 (후진/정지/추종 경계 명확화)
  (5) 첫 TF 조회 실패는 초기 정상 상황임을 안내
  (6) [디버깅용] 단계별 로그 태그 추가 — 아래 태그로 어디까지 진행됐는지 확인 가능
      - [MAIN STEP 0~3]   : main() 실행 순서 (초기화→SLAM대기→노드생성/spin→종료)
      - [STEP 1~6/READY]  : 노드(__init__) 초기화 순서 (파라미터→TF리스너→구독→발행→타이머)
      - [STEP CB]         : 상대 로봇 좌표 콜백에서 "최초 수신" 시 1회만 출력
      - [STEP TF-OK/FAIL] : TF(map→base_link) 조회 성공/실패 (최초 1회 + 상태 변화 시)
      - [STEP CL-1/1b]    : control_loop 조기 리턴 사유 (좌표 없음/타임아웃)
      - [STEP CL-5:분기명]: 매초 한 번, 어떤 거리 구간(BACKUP/STOP_ZONE/
                             ROTATE_IN_PLACE/FOLLOW)이 선택됐는지 표시
      실행 후 위 태그가 순서대로 안 뜨면 그 직전 단계에서 멈춘 것이므로
      바로 원인 지점을 좁힐 수 있습니다.

[실행 전제조건]
  1. SLAM 및 Nav2가 실행 중이어야 함 (map → base_link TF 필요)
     turtlebot4_navigation이 설치돼 있다면:
       ros2 launch turtlebot4_navigation slam.launch.py
       ros2 launch turtlebot4_navigation nav2.launch.py
     ※ 주의: turtlebot4_navigation 미설치 시 위 launch 파일도 없음
       (이 패키지는 코드 의존성뿐 아니라 launch 파일도 함께 담고 있음)
       이 경우 slam_toolbox/nav2_bringup을 직접 launch해야 함, 예:
       ros2 launch slam_toolbox online_async_launch.py
       ros2 launch nav2_bringup navigation_launch.py
  2. 욜로팀이 상대 로봇 좌표를 아래 토픽으로 publish 중이어야 함
     - 토픽 이름 : /target_robot_pose   (파라미터 target_pose_topic 으로 변경 가능)
     - 메시지    : geometry_msgs/msg/PoseStamped
     - frame_id  : 'map' (map 프레임 기준 절대 좌표)
  3. 실행 방법
     $ python3 tb4_follower.py
     네임스페이스가 robot1 인 경우 예시:
     $ python3 tb4_follower.py --ros-args \
         -p base_frame:=robot1/base_link \
         -p cmd_vel_topic:=/robot1/cmd_vel \
         -p target_pose_topic:=/target_robot_pose \
         -p target_qos:=reliable -p max_linear_speed:=0.15

[주요 파라미터]
  target_pose_topic  : 상대 로봇 좌표 토픽 이름            (기본 /target_robot_pose)
  cmd_vel_topic      : 속도 명령 토픽 이름                 (기본 /cmd_vel)
  map_frame          : map 프레임 이름                     (기본 map)
  base_frame         : 로봇 베이스 프레임 이름             (기본 base_link)
  target_qos         : 좌표 토픽 QoS (reliable/best_effort)(기본 best_effort)
  follow_distance    : 유지할 추종 간격 [m]                (기본 1.0)
  stop_distance      : 이보다 가까우면 전진 정지 [m]       (기본 0.8)
  backup_distance    : 이보다 가까우면 약간 후진 [m]       (기본 0.6)
  max_linear_speed   : 최대 전진 속도 [m/s]                (기본 0.3)
  max_angular_speed  : 최대 회전 속도 [rad/s]              (기본 1.0)
  backup_speed       : 후진 속도 [m/s] (양수로 입력)       (기본 0.1)
  kp_linear          : 거리 오차 P 게인                    (기본 0.7)
  kp_angular         : 각도 오차 P 게인                    (기본 1.5)
  control_rate       : 제어 루프 주기 [Hz]                 (기본 15.0)
  target_timeout     : 목표 좌표 미수신 시 정지 판정 [s]   (기본 1.0)
  wait_nav2          : 시작 시 Nav2 활성화 대기 여부        (기본 True)
==============================================================================
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from tf_transformations import euler_from_quaternion

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from nav2_simple_commander.robot_navigator import BasicNavigator
# turtlebot4_navigation 패키지 의존성 제거됨 (도킹 로직은 이 노드에서 다루지 않음)
# 도킹/언도킹이 필요해지면 mission_manager 등 별도 노드에서 처리 권장


# ---------------------------------------------------------------------------
# 유틸리티 함수
# ---------------------------------------------------------------------------
def normalize_angle(angle):
    """각도를 -pi ~ pi 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ---------------------------------------------------------------------------
# 추종 노드
# ---------------------------------------------------------------------------
class TurtleBot4Follower(Node):
    """상대 로봇 좌표를 구독하고 cmd_vel을 직접 publish하여 1m 간격을 유지하는 노드"""

    def __init__(self):
        super().__init__('tb4_follower')

        # ------------------- 파라미터 선언 -------------------
        self.declare_parameter('target_pose_topic', '/target_robot_pose')
        self.declare_parameter('cmd_vel_topic', '/robot5/cmd_vel')
        self.declare_parameter('map_frame', 'robot5/map')
        self.declare_parameter('base_frame', 'robot5/base_link')
        self.declare_parameter('target_qos', 'best_effort')  # 'reliable' 또는 'best_effort'
        self.declare_parameter('follow_distance', 1.0)   # 유지할 간격 [m]
        self.declare_parameter('stop_distance', 0.8)     # 전진 정지 거리 [m]
        self.declare_parameter('backup_distance', 0.6)   # 후진 시작 거리 [m]
        self.declare_parameter('max_linear_speed', 0.3)  # TB4 안전 상한 [m/s]
        self.declare_parameter('max_angular_speed', 1.0) # [rad/s]
        self.declare_parameter('backup_speed', 0.1)      # 후진 속도(양수) [m/s]
        self.declare_parameter('kp_linear', 0.7)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('control_rate', 15.0)     # 제어 루프 [Hz]
        self.declare_parameter('target_timeout', 1.0)    # 좌표 미수신 허용 시간 [s]

        gp = self.get_parameter
        self.map_frame = gp('map_frame').value
        self.base_frame = gp('base_frame').value
        self.follow_distance = gp('follow_distance').value
        self.stop_distance = gp('stop_distance').value
        self.backup_distance = gp('backup_distance').value
        self.max_lin = gp('max_linear_speed').value
        self.max_ang = gp('max_angular_speed').value
        self.backup_speed = abs(gp('backup_speed').value)  # 항상 양수로 사용
        self.kp_lin = gp('kp_linear').value
        self.kp_ang = gp('kp_angular').value
        self.target_timeout = gp('target_timeout').value
        control_rate = gp('control_rate').value

        target_topic = gp('target_pose_topic').value
        cmd_vel_topic = gp('cmd_vel_topic').value
        qos_mode = gp('target_qos').value.lower()

        # 파라미터 유효성 간단 검증 (거리 구간이 논리적으로 맞는지)
        if not (self.backup_distance <= self.stop_distance <= self.follow_distance):
            self.get_logger().warn(
                f'거리 파라미터 순서가 이상합니다: '
                f'backup({self.backup_distance}) <= stop({self.stop_distance}) '
                f'<= follow({self.follow_distance}) 가 되도록 권장합니다.')

        # [STEP 1] 파라미터가 실제로 어떤 값으로 로드됐는지 한눈에 확인
        self.get_logger().info(
            '[STEP 1] 파라미터 로드 완료:\n'
            f'  target_pose_topic = {target_topic}\n'
            f'  cmd_vel_topic     = {cmd_vel_topic}\n'
            f'  map_frame         = {self.map_frame}\n'
            f'  base_frame        = {self.base_frame}\n'
            f'  target_qos        = {qos_mode}\n'
            f'  follow/stop/backup distance = '
            f'{self.follow_distance}/{self.stop_distance}/{self.backup_distance} m\n'
            f'  max_linear/angular = {self.max_lin}/{self.max_ang}\n'
            f'  kp_linear/kp_angular = {self.kp_lin}/{self.kp_ang}\n'
            f'  control_rate = {control_rate} Hz, target_timeout = {self.target_timeout}s')

        # ------------------- 내부 상태 -------------------
        self.target_pose = None          # 최근 수신한 상대 로봇 좌표 (PoseStamped)
        self.last_target_time = None     # 마지막 좌표 수신 시각 (node clock 기준, 초)
        self.warned_no_target = False    # 반복 경고 로그 방지 플래그
        self.warned_no_tf = False
        self.tf_ready_logged = False     # TF 최초 성공 로그 1회용

        # ------------------- TF 리스너 (map → base_link) -------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('[STEP 2] TF 리스너 생성 완료 (아직 TF 수신 여부는 미확인)')

        # ------------------- Subscriber: 상대 로봇 좌표 -------------------
        # QoS: 욜로팀 publisher 설정과 반드시 일치해야 함.
        #  - 상대가 RELIABLE로 publish하는데 여기서 BEST_EFFORT면 수신은 되지만
        #    반대(여기 RELIABLE, 상대 BEST_EFFORT)면 아예 수신 불가.
        #  - 안전하게 파라미터로 노출. 모르면 best_effort로 두면 대부분 수신됨.
        if qos_mode == 'reliable':
            reliability = ReliabilityPolicy.RELIABLE
        else:
            reliability = ReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.target_sub = self.create_subscription(
            PoseStamped, target_topic, self.target_pose_callback, qos)
        self.get_logger().info(
            f"[STEP 3] Subscriber 생성 완료: '{target_topic}' (QoS={qos_mode}) "
            f"→ 아직 좌표 수신 전이면 아래 콜백 로그가 안 찍히는지 확인하세요.")

        # ------------------- Publisher: cmd_vel -------------------
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.get_logger().info(f"[STEP 4] Publisher 생성 완료: '{cmd_vel_topic}'")

        # ------------------- 제어 루프 타이머 -------------------
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.get_logger().info(
            f'[STEP 5] 제어 루프 타이머 시작 ({control_rate}Hz) '
            f'→ 이제부터 매 tick마다 control_loop()가 호출됩니다.')

        self.get_logger().info(
            f'[STEP 6/READY] 추종 노드 초기화 완료: target={target_topic}({qos_mode}), '
            f'cmd_vel={cmd_vel_topic}, TF={self.map_frame}->{self.base_frame}, '
            f'간격={self.follow_distance}m, 제어주기={control_rate}Hz')

    # -----------------------------------------------------------------
    # 콜백: 상대 로봇 좌표 수신 (목표 갱신)
    # -----------------------------------------------------------------
    def target_pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            # map 프레임이 아니면 경고만 하고 일단 사용 (욜로팀 설정 실수 대비)
            self.get_logger().warn(
                f"수신 좌표 frame_id='{msg.header.frame_id}' "
                f"(기대: '{self.map_frame}'). map 기준으로 가정하고 사용합니다.",
                throttle_duration_sec=5.0)
        is_first = self.target_pose is None
        self.target_pose = msg
        # 수신 시각 기준으로 타임아웃 판정 (좌표 header.stamp 대신 수신 시각을 쓰는 이유:
        # 욜로팀 stamp가 부정확하거나 0으로 올 수 있어, 수신 시각이 더 안전함)
        self.last_target_time = self.get_clock().now().nanoseconds * 1e-9
        self.warned_no_target = False
        if is_first:
            # [STEP] 최초 좌표 수신 확인용 — 이 로그가 안 뜨면 토픽 이름/QoS 불일치가 원인
            self.get_logger().info(
                f'[STEP CB] 상대 로봇 좌표 최초 수신: '
                f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, '
                f"frame_id='{msg.header.frame_id}'")

    # -----------------------------------------------------------------
    # 현재 내 로봇 위치 조회 (map → base_link TF)
    # 반환: (x, y, yaw) 또는 None
    # -----------------------------------------------------------------
    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())  # 최신 TF 사용
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.warned_no_tf = False
            if not self.tf_ready_logged:
                self.get_logger().info(
                    f'[STEP TF-OK] TF({self.map_frame}->{self.base_frame}) 정상 수신 시작: '
                    f'x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°')
                self.tf_ready_logged = True
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if not self.warned_no_tf:
                self.get_logger().warn(
                    f'[STEP TF-FAIL] TF({self.map_frame}->{self.base_frame}) 조회 실패: {e} '
                    f'(시작 직후 몇 초간은 정상일 수 있습니다. 계속 반복되면 '
                    f'프레임 이름/네임스페이스를 의심하세요)')
                self.warned_no_tf = True
            return None

    # -----------------------------------------------------------------
    # 제어 루프 (타이머): P 제어로 cmd_vel 계산 및 publish
    # -----------------------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # 1) 목표 좌표가 아직 없거나 오래됐으면 정지
        if self.target_pose is None or self.last_target_time is None:
            if not self.warned_no_target:
                self.get_logger().warn('[STEP CL-1] 상대 로봇 좌표 미수신 상태 → 대기(정지)')
                self.warned_no_target = True
            self.publish_stop()
            return

        if (now - self.last_target_time) > self.target_timeout:
            if not self.warned_no_target:
                self.get_logger().warn(
                    f'[STEP CL-1b] 좌표 수신이 {self.target_timeout:.1f}s 이상 끊김 → 안전 정지')
                self.warned_no_target = True
            self.publish_stop()
            return

        # 2) 내 위치 조회 실패 시 정지
        robot = self.get_robot_pose()
        if robot is None:
            # get_robot_pose() 내부에서 이미 [STEP] 경고를 찍으므로 여기선 추가 로그 없이 정지만
            self.publish_stop()
            return
        rx, ry, ryaw = robot

        # 3) 거리 / 각도 오차 계산 (map 프레임 기준)
        tx = self.target_pose.pose.position.x
        ty = self.target_pose.pose.position.y
        dx = tx - rx
        dy = ty - ry
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_target - ryaw)

        # 4) P 제어기: 선속도 / 각속도 계산
        cmd = Twist()

        # --- 각속도: 항상 상대 방향을 바라보도록 보정 ---
        cmd.angular.z = max(-self.max_ang,
                            min(self.max_ang, self.kp_ang * heading_error))

        # --- 선속도: 거리 구간별 처리 ---
        #   distance < backup_distance           : 너무 가까움 → 후진
        #   backup_distance <= d < stop_distance  : 정지(불감대)
        #   distance >= stop_distance             : P 제어로 전진
        if distance < self.backup_distance:
            # 너무 가까움 → 약간 후진 (후진 중에는 회전을 멈춰 안정성 확보)
            branch = 'BACKUP'
            cmd.linear.x = -self.backup_speed
            cmd.angular.z = 0.0
        elif distance < self.stop_distance:
            # 정지 불감대(backup~stop): 전진 정지, 방향만 유지
            branch = 'STOP_ZONE'
            cmd.linear.x = 0.0
        else:
            # 목표 간격(follow_distance)보다 멀면 P 제어로 전진
            dist_error = distance - self.follow_distance
            lin = self.kp_lin * dist_error
            lin = max(0.0, min(self.max_lin, lin))
            # 각도 오차가 크면(약 60도 이상) 제자리 회전 우선 → 부드러운 궤적
            if abs(heading_error) > math.radians(60):
                branch = 'ROTATE_IN_PLACE'
                lin = 0.0
            else:
                # 각도 오차에 비례해 감속 (코너에서 안정성 향상)
                branch = 'FOLLOW'
                lin *= max(0.0, math.cos(heading_error))
            cmd.linear.x = lin

        self.cmd_pub.publish(cmd)

        # 5) 상태 로그 (1초에 한 번만) — 어떤 거리 구간(branch)이 선택됐는지 함께 표시
        self.get_logger().info(
            f'[STEP CL-5:{branch}] 거리={distance:.2f}m, '
            f'각도오차={math.degrees(heading_error):.1f}°, '
            f'v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}',
            throttle_duration_sec=1.0)

    # -----------------------------------------------------------------
    # 정지 명령 publish
    # -----------------------------------------------------------------
    def publish_stop(self):
        self.cmd_pub.publish(Twist())  # 모든 필드 0 → 정지


# ---------------------------------------------------------------------------
# 메인: (SLAM 준비 대기) → 추종 시작
#   ※ 도킹/언도킹 로직은 turtlebot4_navigation 의존성 문제로 이 노드에서 제거함.
#     도킹 자동화가 필요하면 별도의 mission_manager 노드에서 처리 권장.
# ---------------------------------------------------------------------------
def main():
    rclpy.init()

    # SLAM 활성화 대기용 navigator (nav2_simple_commander만 사용, turtlebot4_navigation 불필요)
    nav_navigator = BasicNavigator(node_name='navigator_robot5',
                                   namespace='robot5')


    log = nav_navigator.get_logger()
    log.info('[MAIN STEP 0] rclpy.init() 완료, navigator 인스턴스 생성 완료 '
              '(도킹 관련 기능 없이 실행 중)')

    follower = None
    try:
        # 1. SLAM 모드에서는 setInitialPose가 필요 없음
        #    slam_toolbox와 Nav2가 활성화될 때까지 대기.
        #    (구성에 따라 localizer 노드 이름이 달라 무한 대기할 수 있으므로 예외 처리)
        log.info('[MAIN STEP 1] SLAM 및 Nav2 활성화 대기 시작...')
        time.sleep(1.0)  # slam_toolbox가 첫 map->odom TF를 발행할 시간 확보
        try:
            nav_navigator.waitUntilNav2Active(localizer='slam_toolbox')
            log.info('[MAIN STEP 1 완료] Nav2/slam_toolbox 활성화 확인됨')
        except Exception as e:
            # localizer 이름 불일치 등으로 대기에 실패해도 추종 자체는 TF만 있으면 가능.
            log.warn(
                f'[MAIN STEP 1 스킵] waitUntilNav2Active 실패: {e} '
                f'→ TF 기반으로 계속 진행합니다. (이 노드 자체는 정상)')

        # 2. 추종 노드 생성 및 실행 (도킹 확인/언도킹 단계 없이 바로 진행)
        log.info('[MAIN STEP 2] TurtleBot4Follower 노드 생성 중...')
        follower = TurtleBot4Follower()
        log.info('[MAIN STEP 2 완료] 노드 생성 완료 → rclpy.spin() 진입 '
                  '(여기서부터 위쪽 [STEP 1~6], [STEP CB], [STEP TF-*], '
                  '[STEP CL-*] 로그들이 순서대로 출력됩니다)')
        rclpy.spin(follower)

    except KeyboardInterrupt:
        if follower is not None:
            follower.get_logger().info('[MAIN] 사용자 종료 요청(Ctrl+C) 수신')
    finally:
        # 3. 종료 처리: 반드시 정지 명령 먼저 보낸 뒤 노드 정리
        log.info('[MAIN STEP 3] 종료 처리 시작 (정지 명령 → 노드 정리 → shutdown)')
        if follower is not None:
            try:
                follower.publish_stop()
                log.info('[MAIN STEP 3] 정지 명령 publish 완료')
            except Exception:
                pass
            follower.destroy_node()
        log.info('[MAIN STEP 3 완료] 노드 정리 시작 (이후 로그는 안 보일 수 있음)')
        nav_navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
