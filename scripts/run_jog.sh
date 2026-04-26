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
echo "[GUI] Control Console 실행..."
echo "    홈에서 6개 모드 중 선택. 각 모드 화면에서 ← Home 으로 복귀."
echo ""
docker exec doosan_kos bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    python3 /kos_workspace/scripts/control_console.py
"
