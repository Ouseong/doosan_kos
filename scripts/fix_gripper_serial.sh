#!/bin/bash
# ============================================================================
# Re-map the Dynamixel gripper serial port into the container and reconnect.
#
# WHY THIS EXISTS
#   The FT232H USB-serial adapter gets a NEW host device node every time it is
#   re-plugged (/dev/ttyUSB0 → ttyUSB1 → ...). The doosan_kos container's /dev
#   is fixed at container-start, so it never sees the new node and its old node
#   goes stale → the gripper node hits "Input/output error" and Open/Close die.
#   The console can't fix this: it runs as non-root (isaac-sim) inside the
#   container and cannot mknod. This script does it from the host as root
#   (the container is privileged, so a mknod with the right major:minor grants
#   real device access).
#
# WHAT IT DOES
#   1. find the live host FTDI ttyUSB* (name/minor is unstable across re-plugs)
#   2. (re)create the container's /dev/ttyUSB0 pointing at it  (stable path for
#      the node, which always opens /dev/ttyUSB0)
#   3. unless --map-only: restart the gripper node so it reopens the live port,
#      then verify the motor answers.
#
# USAGE
#   scripts/fix_gripper_serial.sh              # remap + restart node + verify
#   scripts/fix_gripper_serial.sh --map-only   # just remap the device node
# ============================================================================
set -uo pipefail
CONTAINER=doosan_kos
MAP_ONLY=0
[ "${1:-}" = "--map-only" ] && MAP_ONLY=1

# 1) live host serial device (one FT232H → one ttyUSB*; take the first)
host_ser="$(ls /dev/ttyUSB* 2>/dev/null | head -1)"
if [ -z "$host_ser" ]; then
    echo "✗ 호스트에 /dev/ttyUSB* 없음 — FT232H 어댑터 USB 연결/전원 확인"
    exit 1
fi
maj=$(( 16#$(stat -c '%t' "$host_ser") ))
min=$(( 16#$(stat -c '%T' "$host_ser") ))
echo "host serial: $host_ser  (c $maj:$min)"

# 2) (re)map container /dev/ttyUSB0 → live device, as root
if ! docker exec -u root "$CONTAINER" bash -lc \
        "rm -f /dev/ttyUSB0; mknod /dev/ttyUSB0 c $maj $min; chmod 666 /dev/ttyUSB0"; then
    echo "✗ 컨테이너 mknod 실패 (컨테이너 떠 있는지 확인)"
    exit 1
fi
echo "✓ container /dev/ttyUSB0 → $maj:$min"

if [ "$MAP_ONLY" = "1" ]; then
    exit 0
fi

# 3) restart the gripper node so it reopens the now-live port (an already-open
#    fd to the dead port won't recover on its own)
docker exec "$CONTAINER" pkill -9 -f gripper_node_v2.py 2>/dev/null
sleep 1
docker exec -d "$CONTAINER" bash -lc \
    'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash;
     nohup python3 /kos_workspace/Dynamixel_Control/gripper_node_v2.py \
         > /tmp/gripper_node.log 2>&1 &'

# verify the motor answered (fresh log: node start truncates it)
for _ in $(seq 1 10); do
    sleep 1
    if docker exec "$CONTAINER" grep -q "XM430 ID=.*ready" /tmp/gripper_node.log 2>/dev/null; then
        echo "✓ 그리퍼 연결됨:"
        docker exec "$CONTAINER" grep "XM430 ID=" /tmp/gripper_node.log | tail -1
        exit 0
    fi
done
echo "⚠ 노드는 떴지만 모터 응답 미확인 — 로그:"
docker exec "$CONTAINER" tail -8 /tmp/gripper_node.log
exit 1
