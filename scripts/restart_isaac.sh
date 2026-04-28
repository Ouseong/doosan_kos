#!/bin/bash
# Restart Isaac Sim bridge + telemetry + control console (kill → start, idempotent)
# Sends an auto-home movej so emulator/driver/sim all start at HOME_POSE.
# Pre-req: emulator + virtual driver running. For real-robot, use connect_real.sh.

set -u

echo "=== 기존 프로세스 종료 (bridge + telemetry + console) ==="
docker exec doosan_kos pkill -f m1013_gripper_bridge.py 2>/dev/null || true
docker exec doosan_kos pkill -f telemetry.py            2>/dev/null || true
docker exec doosan_kos pkill -f control_console.py      2>/dev/null || true

# Tk GUIs sometimes ignore SIGTERM; SIGKILL fallback after a beat.
sleep 4
remaining=$(docker exec doosan_kos bash -c "ps -ef | grep -E 'm1013_gripper_bridge|telemetry\\.py|control_console' | grep -v grep" 2>&1)
if [ -n "$remaining" ]; then
    echo "  ⚠ SIGTERM 후 살아남음, SIGKILL 송신:"
    echo "$remaining"
    docker exec doosan_kos pkill -9 -f m1013_gripper_bridge.py 2>/dev/null || true
    docker exec doosan_kos pkill -9 -f telemetry.py            2>/dev/null || true
    docker exec doosan_kos pkill -9 -f control_console.py      2>/dev/null || true
    sleep 2
fi
echo "  ✓ 종료됨"

echo ""
echo "=== Bridge GPU 2 재시작 ==="
docker exec doosan_kos bash -c '> /tmp/bridge.log'
docker exec -d doosan_kos bash -lc '
  export PYTHONUNBUFFERED=1
  export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:$LD_LIBRARY_PATH
  export CUDA_VISIBLE_DEVICES=2
  /isaac-sim/python.sh /kos_workspace/isaac/m1013_gripper_bridge.py > /tmp/bridge.log 2>&1
'
echo "  → 부팅 폴링 (max 150s)"
ready=0
for i in $(seq 1 75); do
    if docker exec doosan_kos grep -q "시뮬레이션 루프 시작" /tmp/bridge.log 2>/dev/null; then
        echo "  ✓ Bridge 부팅 완료 (~$((i*2))s)"
        ready=1; break
    fi
    if docker exec doosan_kos grep -q "Traceback (most recent call last)" /tmp/bridge.log 2>/dev/null; then
        echo "  ✗ Bridge 크래시"
        docker exec doosan_kos tail -25 /tmp/bridge.log
        exit 1
    fi
    sleep 2
done
if [ "$ready" = "0" ]; then
    echo "  ✗ 150s timeout"; docker exec doosan_kos tail -25 /tmp/bridge.log; exit 1
fi

echo ""
echo "=== Auto-home: emulator/driver/sim 모두 HOME_POSE 동기화 ==="
docker exec doosan_kos bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  timeout 30 ros2 service call /dsr01/dsr_controller2/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, -90.0, 0.0, -90.0, 0.0], vel: 30.0, acc: 30.0, time: 0.0, radius: 0.0, mode: 0, blend_type: 0, sync_type: 0}"
' 2>&1 | tail -10

echo ""
echo "=== Telemetry 시작 ==="
docker exec -d doosan_kos bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  python3 /kos_workspace/scripts/telemetry.py > /tmp/telem.log 2>&1
'

echo "=== Control Console 시작 ==="
docker exec -d doosan_kos bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  python3 /kos_workspace/scripts/control_console.py > /tmp/console.log 2>&1
'

sleep 4
echo ""
echo "=== 최종 상태 ==="
docker exec doosan_kos bash -c "
  pgrep -f m1013_gripper_bridge.py > /dev/null && echo '  ✓ bridge'    || echo '  ✗ bridge'
  pgrep -f telemetry.py            > /dev/null && echo '  ✓ telemetry'  || echo '  ✗ telemetry'
  pgrep -f control_console.py      > /dev/null && echo '  ✓ console'    || echo '  ✗ console'
"

echo ""
echo "=== /dsr01/joint_states 검증 ==="
docker exec doosan_kos bash -lc 'source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo --once /dsr01/joint_states 2>&1' | grep -E "name|position" | head -8
