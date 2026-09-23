#!/bin/bash
# start_all.sh 로 띄운 프로세스 전부 종료

# 로봇 SSH 접속 정보 — start_all.sh 와 동일하게 맞추세요 (export 로 덮어쓰기 가능)
ROBOT_IP="${ROBOT_IP:-<ROBOT_IP>}"          # 예: 192.168.0.42
ROBOT_USER="${ROBOT_USER:-ubuntu}"
ROBOT_PW="${ROBOT_PW:-<ROBOT_PASSWORD>}"    # TurtleBot4 기본값은 turtlebot4

echo "=== 이 노트북 프로세스 종료 ==="
ps aux | grep -E "phone_cam_detector|bbox4|mission_manager|tracking/lib/tracking|explore_lite/lib|rosbridge_websocket|iriunwebcam|slam_toolbox|nav2_|lifecycle_manager|node_modules/.bin/vite" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9

echo "=== 로봇 파이 아두이노 슈터 종료 ==="
sshpass -p "$ROBOT_PW" ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 "$ROBOT_USER@$ROBOT_IP" \
  "ps aux | grep '[a]rduino_shooter_node' | awk '{print \$2}' | xargs -r kill -SIGINT"

sleep 1
echo "=== 완료 ==="
