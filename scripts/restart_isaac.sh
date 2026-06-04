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
# ISAAC_GPU env override: defaults to 0 (works on single-GPU hosts like
# DGX Spark). On multi-GPU servers, set ISAAC_GPU=2 (or whichever index)
# before running this script. Picking a non-existent index makes Isaac
# Sim silently fall back to a CPU-only render that never opens a window.
# Default to the GPU actually wired to the display (display_active=Enabled) so
# the windowed Isaac Sim can present its window. Hardcoding 0 fails on this
# multi-GPU host where only GPU 2 drives the screen. Single-GPU hosts resolve
# to 0 anyway. Override with ISAAC_GPU=N.
if [ -z "${ISAAC_GPU:-}" ] && command -v nvidia-smi > /dev/null; then
    ISAAC_GPU="$(nvidia-smi --query-gpu=index,display_active --format=csv,noheader 2>/dev/null \
                 | awk -F', ' '$2=="Enabled"{print $1; exit}')"
fi
ISAAC_GPU="${ISAAC_GPU:-2}"
GUI_DISPLAY="${DISPLAY:-:2}"
echo "=== Bridge GPU $ISAAC_GPU / DISPLAY $GUI_DISPLAY 재시작 ==="
xhost +local: >/dev/null 2>&1 || true
docker exec doosan_kos bash -c '> /tmp/bridge.log'
docker exec -d -e "ISAAC_GPU=$ISAAC_GPU" -e "GUI_DISPLAY=$GUI_DISPLAY" doosan_kos bash -lc '
  export DISPLAY=$GUI_DISPLAY
  export PYTHONUNBUFFERED=1
  export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:$LD_LIBRARY_PATH
  export CUDA_VISIBLE_DEVICES=$ISAAC_GPU
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

# Tk GUIs (telemetry/console) need the *host user's* X display. The container
# image bakes DISPLAY=:1, which on a multi-seat host belongs to a DIFFERENT
# user (e.g. hckang) — connecting there fails with "Authorization required".
# Pass through the invoking user's $DISPLAY (oem == :2). Also open local X
# access so the container's ubuntu uid can connect to the socket.
GUI_DISPLAY="${DISPLAY:-:2}"
echo ""
echo "=== X display = $GUI_DISPLAY (xhost +local:) ==="
xhost +local: >/dev/null 2>&1 || true

echo "=== Telemetry 시작 ==="
docker exec -d doosan_kos bash -lc "
  export DISPLAY=$GUI_DISPLAY
  source /opt/ros/jazzy/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  python3 /kos_workspace/scripts/telemetry.py > /tmp/telem.log 2>&1
"

echo "=== Control Console 시작 ==="
docker exec -d doosan_kos bash -lc "
  export DISPLAY=$GUI_DISPLAY
  source /opt/ros/jazzy/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  python3 /kos_workspace/scripts/control_console.py > /tmp/console.log 2>&1
"

# Tk GUIs init rclpy first (~5s) THEN call tk.Tk(); a display-auth crash lands
# AFTER that. Checking too early sees the doomed process still in rclpy init and
# falsely reports ✓. Wait past the Tk handshake before judging.
sleep 8
echo ""
echo "=== 최종 상태 ==="
gui_status() {  # name -> ✓ only if process alive AND log has no Traceback
    local label="$1" pat="$2" log="$3"
    if docker exec doosan_kos pgrep -f "$pat" > /dev/null; then
        if docker exec doosan_kos grep -q "Traceback" "$log" 2>/dev/null; then
            echo "  ✗ $label (살아있으나 로그에 Traceback — $log 확인)"
        else
            echo "  ✓ $label"
        fi
    else
        echo "  ✗ $label (죽음 — $log 확인)"
        docker exec doosan_kos tail -4 "$log" 2>/dev/null | sed 's/^/      /'
    fi
}
docker exec doosan_kos pgrep -f m1013_gripper_bridge.py > /dev/null && echo '  ✓ bridge' || echo '  ✗ bridge'
gui_status telemetry telemetry.py       /tmp/telem.log
gui_status console   control_console.py  /tmp/console.log

echo ""
echo "=== /dsr01/joint_states 검증 ==="
# --once can emit a "message was lost" diagnostic that desyncs a naive grep.
# Echo a few samples and pull the first real position line instead.
js=$(docker exec doosan_kos bash -lc 'source /opt/ros/jazzy/setup.bash && timeout 6 ros2 topic echo --field position /dsr01/joint_states 2>/dev/null | grep -m1 "^array"')
if [ -n "$js" ]; then
    echo "  ✓ joint_states publishing: position=$js"
else
    echo "  ✗ joint_states 무응답 (bridge 발행 확인 필요)"
fi
