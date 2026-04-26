#!/bin/bash
# ================================================
# M1013 시뮬 스택 부팅 + Telemetry GUI 실행
#
# Usage:
#   bash scripts/run_telemetry.sh             # Isaac Sim 포함
#   bash scripts/run_telemetry.sh --no-isaac  # 가볍게
#
# Jog GUI 와 동시에 띄울 수 있음. 두 스크립트 모두 idempotent 라서
# 한 쪽이 이미 스택을 띄워뒀으면 다른 쪽은 그대로 GUI 만 실행됨.
# ================================================

set -e
source "$(dirname "$0")/_ensure_stack.sh"
ensure_stack "$@"

echo ""
echo "[GUI] Telemetry 실행..."
echo "    창에는 ① 직접 측정값 ② DRCF 계산값 ③ ROS 측 계산값 3 섹션이 표시됩니다."
echo ""
docker exec doosan_kos bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    python3 /kos_workspace/scripts/telemetry.py
"
