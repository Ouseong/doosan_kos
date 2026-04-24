#!/bin/bash
# ================================================
# 컨테이너 시작 시 자동 실행
# ✅ Isaac Sim 5.1.0 기준 (유저 1234로 실행)
# 1. ROS2 환경 소싱 (bootstrap 의 rosdep 실행에 필요)
# 2. doosan-robot2 빌드 (첫 실행 시만)
# 3. 워크스페이스 소싱
# ================================================

# user 1234 HOME 보장
export HOME="${HOME:-/isaac-sim}"

# ROS2 환경 먼저 소싱 (bootstrap_ws.sh 내 rosdep 이 ROS2 환경을 필요로 함)
source /opt/ros/jazzy/setup.bash 2>/dev/null || true

# doosan-robot2 빌드 (첫 실행 시만)
bash /usr/local/bin/bootstrap_ws.sh

# 빌드된 워크스페이스 소싱
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

echo "[KOS] 환경 준비 완료!"

exec "$@"
