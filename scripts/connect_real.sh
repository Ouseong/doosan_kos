#!/bin/bash
# Switch the M1013 driver from emulator to real robot at $ROBOT_IP:$ROBOT_PORT.
#
# Critical: the emulator container listens on 0.0.0.0:12345 (host network mode),
# which short-circuits the driver to localhost even when host:= is set to the
# real IP. Stopping it first is mandatory.
#
# After this script:
#   ✓ TCP ESTAB host→192.168.137.100:12345
#   ✓ driver.log: "[dsr_hw_interface2]: Connected to DRCF"
#   ⚠ /dsr01/joint_states may stay empty until E-stop is released and servo activates.
#
# To return to sim afterward, run:  scripts/connect_virtual.sh

set -u
ROBOT_IP="${ROBOT_IP:-192.168.137.100}"
ROBOT_PORT="${ROBOT_PORT:-12345}"

echo "=== ① emulator 컨테이너 stop ==="
if docker ps --format '{{.Names}}' | grep -q '^emulator$'; then
    docker stop emulator 2>&1 | head -2
else
    echo "  (이미 안 떠있음)"
fi
sleep 3
if ss -tln | grep -q ":12345 "; then
    echo "  ⚠ 12345 listen 살아있음 (다른 listener? 확인 필요):"
    ss -tln | grep ":12345 "
    exit 1
else
    echo "  ✓ 12345 listener 없음"
fi

echo ""
echo "=== ② virtual driver 종료 ==="
docker exec doosan_kos pkill -f "ros2 launch dsr_bringup2" 2>/dev/null || true
docker exec doosan_kos pkill -f ros2_control_node         2>/dev/null || true
sleep 4
docker exec doosan_kos pkill -9 -f "ros2 launch dsr_bringup2" 2>/dev/null || true
docker exec doosan_kos pkill -9 -f ros2_control_node         2>/dev/null || true
sleep 1
echo "  ✓ kill done"

echo ""
echo "=== ③ Real driver 부팅 (mode:=real host:=$ROBOT_IP) ==="
docker exec doosan_kos bash -c '> /tmp/driver.log'
docker exec -d doosan_kos bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
        model:=m1013 mode:=real host:=$ROBOT_IP port:=$ROBOT_PORT gui:=false \
        > /tmp/driver.log 2>&1
"

echo "  → DRCF connection 폴링 (max 60s)"
ready=0
for i in $(seq 1 30); do
    if ss -tn 2>/dev/null | grep -q "$ROBOT_IP:$ROBOT_PORT"; then
        echo "  ✓ TCP ESTAB → $ROBOT_IP:$ROBOT_PORT (~$((i*2))s)"
        ready=1; break
    fi
    if docker exec doosan_kos grep -qiE "Connection refused|connect failed|cannot connect|Traceback" /tmp/driver.log 2>/dev/null; then
        echo "  ✗ 연결 실패 단서:"
        docker exec doosan_kos tail -20 /tmp/driver.log
        exit 1
    fi
    sleep 2
done
if [ "$ready" = "0" ]; then
    echo "  ✗ 60s timeout — driver.log 확인:"
    docker exec doosan_kos tail -25 /tmp/driver.log
    exit 1
fi

echo ""
echo "=== ④ DRCF 상태 (driver.log 발췌) ==="
docker exec doosan_kos grep -E "Connected to DRCF|host :|rt_host :|DRCF version|DRFL version" /tmp/driver.log 2>&1 | head -10

echo ""
echo "=== ⑤ 호스트 TCP ESTAB ==="
ss -tn 2>/dev/null | grep "$ROBOT_IP:$ROBOT_PORT" | head -3

echo ""
echo "=== ⑥ /dsr01/joint_states (E-stop 풀린 상태에서만 받힘) ==="
out=$(docker exec doosan_kos bash -lc 'source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo --once /dsr01/joint_states 2>&1')
if echo "$out" | grep -q "position:"; then
    echo "$out" | head -25
    echo "  ✓ state 수신 — robot 준비 완료"
else
    echo "  ⚠ state 비어있음 — E-stop ON 상태일 가능성 (해제하면 받혀야 함)"
fi

echo ""
echo "════════════════════════════════════════"
echo "Real driver 연결 완료. Sim으로 돌아가려면: scripts/connect_virtual.sh"
echo "════════════════════════════════════════"
