#!/bin/bash
# 전체 프로세스 한번에 실행 (순찰+탐지→조준→승인→발사 흐름용, 폰캠 버전)
# 로그는 /tmp/turtlebot4_wargame_logs/ 에 저장됨. 끌 땐 stop_all.sh 사용.
# [주의] set -u 쓰면 /opt/ros/humble/setup.bash 소싱 중 AMENT_TRACE_SETUP_FILES
# unbound variable 에러로 죽는다 (ROS 셋업 스크립트 자체가 strict mode 비호환).

LOGDIR=/tmp/turtlebot4_wargame_logs
mkdir -p "$LOGDIR"

# ---------------------------------------------------------------------------
# 환경 설정 — 아래 값들은 각자 환경에 맞게 수정하거나, 실행 전 export 로 덮어쓰세요.
#   예) ROBOT_IP=192.168.0.42 ./start_all.sh
# ---------------------------------------------------------------------------
# 이 스크립트가 있는 디렉터리(= 레포 루트)를 기준으로 워크스페이스 경로를 잡습니다.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
PATROL_WS="${PATROL_WS:-$REPO_DIR/patrol_ws/install/setup.bash}"
AMR_WS="${AMR_WS:-$REPO_DIR/AMR_ws/install/setup.bash}"
# TurtleBot4 표준 워크스페이스 — 이 레포에 포함되지 않음(공식 가이드대로 별도 설치)
TB4_WS="${TB4_WS:-$HOME/turtlebot4_ws/install/setup.bash}"
UI_DIR="${UI_DIR:-$REPO_DIR/turtlebot4_wargame-ui/frontend-team-pc}"

# 로봇(라즈베리파이) SSH 접속 정보 — 아두이노 슈터 노드 기동용
ROBOT_IP="${ROBOT_IP:-<ROBOT_IP>}"          # 예: 192.168.0.42
ROBOT_USER="${ROBOT_USER:-ubuntu}"
ROBOT_PW="${ROBOT_PW:-<ROBOT_PASSWORD>}"    # TurtleBot4 기본값은 turtlebot4
ROBOT_WS="${ROBOT_WS:-/home/$ROBOT_USER/rokey_ws/install/setup.bash}"
ROS_NS="${ROS_NS:-/robot4}"

echo "[1/10] SLAM"
( source "$ROS_SETUP" && source "$TB4_WS" && \
  nohup ros2 launch turtlebot4_navigation slam.launch.py namespace:="${ROS_NS#/}" \
    > "$LOGDIR/slam.log" 2>&1 & disown )
sleep 6

echo "[2/10] Nav2"
( source "$ROS_SETUP" && source "$TB4_WS" && \
  nohup ros2 launch turtlebot4_navigation nav2.launch.py namespace:="${ROS_NS#/}" \
    > "$LOGDIR/nav2.log" 2>&1 & disown )
echo "  -> Nav2 활성화 대기 중 (약 20초)..."
sleep 20

echo "[3/10] Iriun Webcam"
nohup iriunwebcam > "$LOGDIR/iriun.log" 2>&1 &
disown

echo "[4/10] phone_cam_detector"
( source "$ROS_SETUP" && source "$PATROL_WS" && \
  nohup ros2 run yolo_detector phone_cam_detector --ros-args -r __ns:="$ROS_NS" \
    > "$LOGDIR/phone_cam_detector.log" 2>&1 & disown )

echo "[5/10] mission_manager"
( source "$ROS_SETUP" && source "$PATROL_WS" && source "$AMR_WS" && \
  nohup ros2 run tracking mission_manager --ros-args -r __ns:="$ROS_NS" \
    > "$LOGDIR/mission_manager.log" 2>&1 & disown )

echo "[6/10] azimuth"
( source "$ROS_SETUP" && source "$AMR_WS" && \
  nohup ros2 run tracking azimuth --ros-args -r __ns:="$ROS_NS" \
    > "$LOGDIR/azimuth.log" 2>&1 & disown )

echo "[7/10] target"
( source "$ROS_SETUP" && source "$AMR_WS" && \
  nohup ros2 run tracking target --ros-args -r __ns:="$ROS_NS" \
    > "$LOGDIR/target.log" 2>&1 & disown )

echo "[8/10] rosbridge"
( source "$ROS_SETUP" && source "$PATROL_WS" && \
  nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    > "$LOGDIR/rosbridge.log" 2>&1 & disown )

echo "[9/10] frontend (npm run dev)"
( cd "$UI_DIR" && nohup npm run dev > "$LOGDIR/vite.log" 2>&1 & disown )

echo "[10/10] 아두이노 슈터 (로봇 파이, SSH)"
ARDUINO_PORT=$(sshpass -p "$ROBOT_PW" ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
  "$ROBOT_USER@$ROBOT_IP" "ls /dev/ttyACM* 2>/dev/null | head -1")
if [ -z "$ARDUINO_PORT" ]; then
  echo "  -> 아두이노 포트를 못 찾았습니다 (USB 연결 확인 필요). 슈터 노드는 건너뜁니다."
else
  echo "  -> 포트: $ARDUINO_PORT"
  sshpass -p "$ROBOT_PW" ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 "$ROBOT_USER@$ROBOT_IP" "
    export ROS_DOMAIN_ID=5
    export ROS_DISCOVERY_SERVER=';;;;127.0.0.1:11811;'
    export ROS_SUPER_CLIENT=False
    source /opt/ros/humble/setup.bash
    source $ROBOT_WS
    nohup ros2 run turtlebot4_beep arduino_shooter_node --ros-args -r __ns:="$ROS_NS" -p port:=$ARDUINO_PORT \
      > \$HOME/arduino_shooter.log 2>&1 < /dev/null &
    disown
  "
fi

sleep 3
echo
echo "=== 완료. 로그 확인: $LOGDIR ==="
echo "=== 브라우저: http://localhost:5173 (BLUE / blue1234) ==="
