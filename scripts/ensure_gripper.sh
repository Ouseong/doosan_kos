#!/bin/bash
# ============================================================
# Make the Dynamixel gripper reachable from the container, then run its node.
#
# The FT232H USB adapter (vendor 0403) gets a NEW /dev/ttyUSB# minor every time
# it is re-plugged, and the privileged container does not track host hotplug —
# so a stale container /dev/ttyUSB0 points at a dead device and the gripper
# "won't connect". This finds the host adapter's char major/minor, recreates
# the container's /dev/ttyUSB0 to match (needs root → docker exec -u 0), and
# (re)starts gripper_node_v2.py.  Safe to run repeatedly.
# ============================================================
CONTAINER=doosan_kos

# 1) Locate the FTDI gripper adapter on the host (vendor 0403 = FTDI/FT232H)
dev=""
for d in /dev/ttyUSB*; do
    [ -e "$d" ] || continue
    vid=$(udevadm info -q property -n "$d" 2>/dev/null | sed -n 's/^ID_VENDOR_ID=//p')
    [ "$vid" = "0403" ] && { dev="$d"; break; }
done
if [ -z "$dev" ]; then
    echo "[gripper] FTDI adapter (0403) not found on host — skipping"
    exit 0
fi

# 2) Its char device major/minor (stat gives hex → convert to decimal)
hexmajor=$(stat -c '%t' "$dev"); hexminor=$(stat -c '%T' "$dev")
major=$((16#$hexmajor)); minor=$((16#$hexminor))
echo "[gripper] host $dev = char $major,$minor → recreating container /dev/ttyUSB0"
docker exec -u 0 "$CONTAINER" bash -lc \
    "rm -f /dev/ttyUSB0 && mknod /dev/ttyUSB0 c $major $minor && chmod 666 /dev/ttyUSB0" \
    || { echo "[gripper] mknod failed"; exit 1; }

# 3) (Re)start the gripper node so it opens the now-correct port
docker exec "$CONTAINER" bash -lc "pkill -9 -f gripper_node_v2.py 2>/dev/null; sleep 1"
docker exec -d "$CONTAINER" bash -lc '
    source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash &&
    python3 /kos_workspace/Dynamixel_Control/gripper_node_v2.py > /tmp/gripper_node.log 2>&1'
echo "[gripper] node started — XM430 should report 'ready' in ~6s (tail /tmp/gripper_node.log)"
