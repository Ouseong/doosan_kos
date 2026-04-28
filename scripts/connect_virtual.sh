#!/bin/bash
# Switch back from real robot to emulator/sim mode.
#
# Recreates the emulator container (it's --rm so stop = gone) and brings up
# dsr_bringup2 in mode:=virtual against localhost:12345.

set -u

echo "=== ① real driver 종료 ==="
docker exec doosan_kos pkill -f "ros2 launch dsr_bringup2" 2>/dev/null || true
docker exec doosan_kos pkill -f ros2_control_node         2>/dev/null || true
sleep 4
docker exec doosan_kos pkill -9 -f "ros2 launch dsr_bringup2" 2>/dev/null || true
docker exec doosan_kos pkill -9 -f ros2_control_node         2>/dev/null || true
sleep 1
echo "  ✓ kill done"

echo ""
echo "=== ② emulator 재생성 ==="
PROJ_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
bash "$PROJ_ROOT/docker/run_emulator.sh" 2>&1 | tail -5

sleep 2
if ss -tln | grep -q ":12345 "; then
    echo "  ✓ 12345 listener 등장"
else
    echo "  ✗ emulator listener 안 뜸"
    exit 1
fi

echo ""
echo "=== ③ Virtual driver 부팅 ==="
docker exec doosan_kos bash -c '> /tmp/driver.log'
docker exec -d doosan_kos bash -lc '
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
        model:=m1013 mode:=virtual host:=127.0.0.1 port:=12345 gui:=false \
        > /tmp/driver.log 2>&1
'

echo "  → motion service 등장 폴링 (max 60s)"
for i in $(seq 1 30); do
    if docker exec doosan_kos bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 service list 2>/dev/null | grep -q "/dsr01/dsr_controller2/motion/move_joint$"'; then
        echo "  ✓ Virtual driver 준비됨 (~$((i*2))s)"
        break
    fi
    sleep 2
done

echo ""
echo "════════════════════════════════════════"
echo "Sim mode 복구 완료. Bridge/console/telemetry는 살아있다면 그대로 사용."
echo "재시작이 필요하면: scripts/restart_isaac.sh"
echo "════════════════════════════════════════"
