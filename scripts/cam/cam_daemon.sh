#!/bin/bash
# Host-side daemon: watches /kos_workspace/.cam_open flag file
# and starts/stops d435_rgb_view.py accordingly.
# Started by run_jog.sh in background.

FLAG="/home/oem/Desktop/doosan_kos/.cam_open"
SCRIPT="/home/oem/Desktop/doosan_kos/scripts/cam/d435_rgb_view.py"
DISPLAY="${DISPLAY:-:2}"
cam_pid=""

# Clean up flag on exit
trap 'rm -f "$FLAG"; [ -n "$cam_pid" ] && kill "$cam_pid" 2>/dev/null' EXIT

while true; do
    if [ -f "$FLAG" ]; then
        # Flag present — camera should be open
        if [ -z "$cam_pid" ] || ! kill -0 "$cam_pid" 2>/dev/null; then
            DISPLAY="$DISPLAY" python3 "$SCRIPT" > /tmp/d435_view.log 2>&1 &
            cam_pid=$!
        fi
    else
        # Flag absent — camera should be closed
        if [ -n "$cam_pid" ] && kill -0 "$cam_pid" 2>/dev/null; then
            kill "$cam_pid" 2>/dev/null
            cam_pid=""
        fi
    fi
    sleep 0.5
done
