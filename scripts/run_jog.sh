#!/bin/bash
# ================================================
# M1013 시뮬 스택 부팅 + Joint Jog GUI 실행
#
# Usage:
#   bash scripts/run_jog.sh             # Isaac Sim 포함
#   bash scripts/run_jog.sh --no-isaac  # 가볍게
# ================================================

set -e
source "$(dirname "$0")/_ensure_stack.sh"
ensure_stack "$@"

echo ""
echo "[GUI] Joint Jog 실행..."
echo "    (창이 뜨면 슬라이더로 관절 각도 지정 → Send 클릭)"
echo ""
docker exec doosan_kos bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    python3 /kos_workspace/scripts/joint_jog.py
"
