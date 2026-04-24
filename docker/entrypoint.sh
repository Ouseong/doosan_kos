#!/bin/bash
# ================================================
# 컨테이너 시작 시 자동 실행
# ✅ Isaac Sim 5.1.0 기준 (유저 1234로 실행)
# 1. doosan-robot2 빌드 (첫 실행 시만)
# 2. ROS2 환경 소싱
# ================================================

# doosan-robot2 빌드 (첫 실행 시만)
bash /usr/local/bin/bootstrap_ws.sh

# ROS2 환경 소싱
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

echo "[KOS] 환경 준비 완료!"

exec "$@"
