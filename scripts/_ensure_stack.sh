#!/bin/bash
# ================================================
# M1013 시뮬 스택 부팅 헬퍼 (source 해서 ensure_stack 호출)
#
#   xhost / 컨테이너 / emulator / driver / Isaac Sim bridge
#   순서대로 안 떠있는 것만 띄움 (idempotent).
#
# Usage in another script:
#   source "$(dirname "$0")/_ensure_stack.sh"
#   ensure_stack            # Isaac Sim 포함
#   ensure_stack --no-isaac # 가벼운 모드
# ================================================

ensure_stack() {
    local with_isaac=1
    [ "${1:-}" = "--no-isaac" ] && with_isaac=0

    local proj_root
    proj_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

    echo "[1/5] X11 access..."
    if command -v xhost > /dev/null; then
        xhost +local: > /dev/null 2>&1 || true
    fi

    echo "[2/5] container..."
    if docker ps --format '{{.Names}}' | grep -q '^doosan_kos$'; then
        echo "  ✓ 이미 실행 중"
    else
        echo "  → 시작 중 (첫 빌드라면 30-45분 소요)..."
        bash "$proj_root/docker/container.sh" start
    fi

    echo "[3/5] emulator..."
    if docker ps --format '{{.Names}}' | grep -q '^emulator$' && \
       ss -tln 2>/dev/null | grep -q ':12345'; then
        echo "  ✓ 이미 실행 중 (port 12345 LISTEN)"
    else
        bash "$proj_root/docker/run_emulator.sh"
    fi

    echo "[4/5] ROS2 driver..."
    if docker exec doosan_kos pgrep -f ros2_control_node > /dev/null 2>&1; then
        echo "  ✓ 이미 실행 중"
    else
        docker exec -d doosan_kos bash -lc "
            source /opt/ros/jazzy/setup.bash &&
            source /ros2_ws/install/setup.bash &&
            ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
                model:=m1013 mode:=virtual host:=127.0.0.1 port:=12345 gui:=false \
                > /tmp/driver.log 2>&1
        "
        echo -n "  → 서비스 준비 대기"
        local ready=0
        for i in $(seq 1 30); do
            if docker exec doosan_kos bash -lc \
                "source /opt/ros/jazzy/setup.bash && timeout 1 ros2 service list 2>/dev/null | grep -q 'motion/move_joint$'"; then
                echo " ✓"
                ready=1
                break
            fi
            echo -n "."
            sleep 2
        done
        if [ "$ready" = "0" ]; then
            echo " ✗ 60초 안에 안 뜸. docker exec doosan_kos tail /tmp/driver.log 확인"
            return 1
        fi
    fi

    if [ "$with_isaac" = "1" ]; then
        echo "[5/6] Isaac Sim bridge..."
        if docker exec doosan_kos pgrep -f m1013_ros2_bridge.py > /dev/null 2>&1; then
            echo "  ✓ 이미 실행 중"
        else
            docker exec -d doosan_kos bash -lc "
                export PYTHONUNBUFFERED=1
                export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:\$LD_LIBRARY_PATH
                /isaac-sim/python.sh /kos_workspace/isaac/m1013_ros2_bridge.py > /tmp/bridge.log 2>&1
            "
            echo -n "  → Isaac Sim 시작 대기 (보통 20-30초)"
            local ready=0
            for i in $(seq 1 60); do
                if docker exec doosan_kos grep -q "시뮬레이션 루프 시작" /tmp/bridge.log 2>/dev/null; then
                    echo " ✓"
                    ready=1
                    break
                fi
                if docker exec doosan_kos grep -qiE "Traceback|ERROR" /tmp/bridge.log 2>/dev/null; then
                    echo " ✗ 에러 발생. docker exec doosan_kos tail /tmp/bridge.log 확인"
                    return 1
                fi
                echo -n "."
                sleep 3
            done
            if [ "$ready" = "0" ]; then
                echo " ✗ 3분 안에 준비 안 됨. docker exec doosan_kos tail /tmp/bridge.log 확인"
                return 1
            fi
        fi

        # Isaac Sim 이 켜져있으면 telemetry 창도 같이 띄움
        echo "[6/6] Telemetry window..."
        if docker exec doosan_kos pgrep -f telemetry.py > /dev/null 2>&1; then
            echo "  ✓ 이미 실행 중"
        else
            docker exec -d doosan_kos bash -lc "
                source /opt/ros/jazzy/setup.bash &&
                source /ros2_ws/install/setup.bash &&
                python3 /kos_workspace/scripts/telemetry.py > /tmp/telem.log 2>&1
            "
            sleep 2
            if docker exec doosan_kos pgrep -f telemetry.py > /dev/null 2>&1; then
                echo "  ✓ 시작됨"
            else
                echo "  ⚠ 시작 실패. docker exec doosan_kos tail /tmp/telem.log 확인"
            fi
        fi
    fi

    return 0
}
