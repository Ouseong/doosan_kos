#!/bin/bash
# ============================================================
# Keeps cam_daemon alive so the Console "Open Camera" button always works.
#
# The console runs inside the container and only sends "open" to the host
# daemon on :19876 — it cannot start the host-side daemon itself. This loop
# (re)launches cam_daemon whenever the port is down, picking a python that
# actually has pyrealsense2. cam_daemon binds even with no camera attached
# (it answers "no_camera"), so it is safe to keep running unconditionally.
#
# Usage:  nohup scripts/cam/cam_watchdog.sh >/tmp/cam_watchdog.log 2>&1 &
# ============================================================
DIR="$(cd "$(dirname "$0")" && pwd)"

pick_py() {
    for c in python3.10 python3 python3.11 python3.12; do
        if command -v "$c" >/dev/null 2>&1 \
           && "$c" -c "import pyrealsense2" >/dev/null 2>&1; then
            echo "$c"; return
        fi
    done
}

is_up() { ss -tln 2>/dev/null | grep -q ':19876'; }

echo "[cam_watchdog] started — keeping cam_daemon alive on :19876"
while true; do
    if ! is_up; then
        py="$(pick_py)"
        if [ -n "$py" ]; then
            echo "[cam_watchdog] $(date '+%H:%M:%S') daemon down → starting with $py"
            nohup "$py" "$DIR/cam_daemon.py" >/tmp/cam_daemon.log 2>&1 &
            sleep 2
        else
            echo "[cam_watchdog] no python with pyrealsense2 found — retrying"
        fi
    fi
    sleep 3
done
