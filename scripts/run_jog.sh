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
echo "✓ 스택 + Control Console 모두 부팅됨."
echo "  홈에서 6개 모드 중 선택. 각 모드 화면에서 ← Home 으로 복귀."
