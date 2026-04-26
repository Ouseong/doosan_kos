#!/bin/bash
# ================================================
# M1013 시뮬 스택 일괄 부팅 + Jog GUI 실행
#
# Usage:
#   bash scripts/run_jog.sh             # 전부 켜고 GUI 실행 (Isaac Sim 포함)
#   bash scripts/run_jog.sh --no-isaac  # Isaac Sim 제외 (가벼운 모드)
#
# 이미 떠있는 컴포넌트는 건너뛰고 안 떠있는 것만 시작합니다 (idempotent).
# ================================================

set -e
cd "$(dirname "$0")/.."  # 프로젝트 루트로 이동

WITH_ISAAC=1
[ "${1:-}" = "--no-isaac" ] && WITH_ISAAC=0

# ── 1. X11 접근 권한 (컨테이너 GUI 가 호스트 디스플레이 쓰도록) ──
echo "[1/5] X11 access..."
if command -v xhost > /dev/null; then
    xhost +local: > /dev/null 2>&1 || true
fi

# ── 2. doosan_kos 컨테이너 ──
echo "[2/5] container..."
if docker ps --format '{{.Names}}' | grep -q '^doosan_kos$'; then
    echo "  ✓ 이미 실행 중"
else
    echo "  → 시작 중 (첫 빌드라면 30-45분 소요)..."
    bash docker/container.sh start
fi

# ── 3. DRCF 에뮬레이터 ──
echo "[3/5] emulator..."
if docker ps --format '{{.Names}}' | grep -q '^emulator$' && \
   ss -tln 2>/dev/null | grep -q ':12345'; then
    echo "  ✓ 이미 실행 중 (port 12345 LISTEN)"
else
    bash docker/run_emulator.sh
fi

# ── 4. ROS2 driver (dsr_bringup2) ──
echo "[4/5] ROS2 driver..."
if docker exec doosan_kos pgrep -f ros2_control_node > /dev/null 2>&1; then
    echo "  ✓ 이미 실행 중"
else
    docker exec -d doosan_kos bash -lc "
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
            model:=m1013 mode:=virtual host:=127.0.0.1 port:=12345 use_rviz:=false \
            > /tmp/driver.log 2>&1
    "
    echo -n "  → 서비스 준비 대기"
    READY=0
    for i in $(seq 1 30); do
        if docker exec doosan_kos bash -lc \
            "source /opt/ros/jazzy/setup.bash && timeout 1 ros2 service list 2>/dev/null | grep -q 'motion/move_joint$'"; then
            echo " ✓"
            READY=1
            break
        fi
        echo -n "."
        sleep 2
    done
    if [ "$READY" = "0" ]; then
        echo " ✗ 60초 안에 안 뜸. docker exec doosan_kos tail /tmp/driver.log 확인"
        exit 1
    fi
fi

# ── 5a. Isaac Sim 브리지 (선택) ──
if [ "$WITH_ISAAC" = "1" ]; then
    echo "[5a]  Isaac Sim bridge..."
    if docker exec doosan_kos pgrep -f m1013_ros2_bridge.py > /dev/null 2>&1; then
        echo "  ✓ 이미 실행 중"
    else
        docker exec -d doosan_kos bash -lc "
            export PYTHONUNBUFFERED=1
            export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:\$LD_LIBRARY_PATH
            /isaac-sim/python.sh /kos_workspace/isaac/m1013_ros2_bridge.py > /tmp/bridge.log 2>&1
        "
        echo -n "  → Isaac Sim 시작 대기 (보통 20-30초)"
        READY=0
        for i in $(seq 1 60); do
            if docker exec doosan_kos grep -q "시뮬레이션 루프 시작" /tmp/bridge.log 2>/dev/null; then
                echo " ✓"
                READY=1
                break
            fi
            if docker exec doosan_kos grep -qiE "Traceback|ERROR" /tmp/bridge.log 2>/dev/null; then
                echo " ✗ 에러 발생. docker exec doosan_kos tail /tmp/bridge.log 확인"
                exit 1
            fi
            echo -n "."
            sleep 3
        done
        if [ "$READY" = "0" ]; then
            echo " ✗ 3분 안에 준비 안 됨. docker exec doosan_kos tail /tmp/bridge.log 확인"
            exit 1
        fi
    fi
fi

# ── 5. Jog GUI 실행 (포그라운드, 종료시 GUI 만 닫힘) ──
echo "[5/5] Jog GUI 실행..."
echo "    (창이 뜨면 슬라이더로 관절 각도 지정 → Send 클릭)"
echo "    (GUI 닫아도 다른 컴포넌트는 계속 실행 중. 전부 끄려면 docker rm -f doosan_kos emulator)"
echo ""
docker exec doosan_kos bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    python3 /kos_workspace/scripts/joint_jog.py
"
