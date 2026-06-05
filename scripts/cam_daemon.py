#!/usr/bin/env python3
"""Host-side camera daemon.

TCP 19876 commands (from console inside docker):
  open                          → start live viewer window
  close                         → stop live viewer window
  capture:<idx>:<color>:<area>  → grab D435 frame, detect blob, save thumbnail,
                                   respond "ok:<px_w>" or "none"

Started by run_jog.sh when Isaac Sim is launched.
"""
import os
import socket
import subprocess
import sys
import threading
import time

PORT = 19876
SERIAL = "043322073704"
SCRIPT = os.path.join(os.path.dirname(__file__), "d435_rgb_view.py")
WORKSPACE = os.path.dirname(os.path.dirname(__file__))  # doosan_kos root

PRESETS = {
    "orange": [((5, 110, 110), (22, 255, 255))],
    "red":    [((0, 120, 100), (10, 255, 255)), ((170, 120, 100), (179, 255, 255))],
    "green":  [((40, 80, 60), (85, 255, 255))],
    "blue":   [((95, 120, 60), (130, 255, 255))],
    "yellow": [((22, 110, 110), (35, 255, 255))],
}

cam_proc = None
lock = threading.Lock()


def _kill_viewer():
    """Kill any running d435_rgb_view.py (tracked or orphaned)."""
    global cam_proc
    # Kill tracked process
    if cam_proc is not None and cam_proc.poll() is None:
        cam_proc.kill()
        try:
            cam_proc.wait(timeout=3)
        except Exception:
            pass
    cam_proc = None
    # Also pkill any orphaned instances not tracked by cam_proc
    subprocess.run(["pkill", "-9", "-f", "d435_rgb_view.py"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)  # wait for device to release


def _detect_blob(img_bgr, color, min_area):
    import cv2
    import numpy as np
    ranges = PRESETS.get(color, PRESETS["orange"])
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = m if mask is None else (mask | m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0
    for c in cnts:
        area = cv2.contourArea(c)
        if area >= min_area and area > best_area:
            best, best_area = c, area
    if best is None:
        return None, None, None, img_bgr
    x, y, w, h = cv2.boundingRect(best)
    px_size = float(w)
    bx = x + w / 2.0   # blob center x
    by = y + h / 2.0   # blob center y
    cv2.drawContours(img_bgr, [best], -1, (0, 200, 255), 2)
    cv2.circle(img_bgr, (int(bx), int(by)), 4, (0, 200, 255), -1)
    cv2.putText(img_bgr, f"w={px_size:.0f}px",
                (x, y - 8 if y > 16 else y + h + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
    return px_size, bx, by, img_bgr


def _do_capture(idx, color, min_area):
    """Returns (px_w, fx, fy, ppx, ppy, bx, by)."""
    import cv2
    import numpy as np
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(SERIAL)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(cfg)

    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()
    fx, fy = intr.fx, intr.fy
    ppx, ppy = intr.ppx, intr.ppy

    try:
        for _ in range(15):
            pipeline.wait_for_frames()
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        img = np.asanyarray(color_frame.get_data()).copy()
    finally:
        pipeline.stop()

    px_w, bx, by, annotated = _detect_blob(img, color, min_area)

    thumb = cv2.resize(annotated, (200, 150))
    thumb_path = os.path.join(WORKSPACE, f".cap_{idx}.jpg")
    cv2.imwrite(thumb_path, thumb)

    return px_w, fx, fy, ppx, ppy, bx, by


def handle(conn: socket.socket):
    global cam_proc
    try:
        data = conn.recv(64).strip().decode()
    except Exception:
        conn.close()
        return

    with lock:
        if data == "open":
            _kill_viewer()
            env = {**os.environ, "DISPLAY": os.environ.get("DISPLAY", ":2")}
            cam_proc = subprocess.Popen(
                [sys.executable, SCRIPT], env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[cam_daemon] opened  pid={cam_proc.pid}", flush=True)
            conn.close()

        elif data == "close":
            _kill_viewer()
            conn.close()

        elif data.startswith("capture:"):
            # capture:<idx>:<color>:<min_area>
            parts = data.split(":")
            idx = int(parts[1])
            color = parts[2] if len(parts) > 2 else "orange"
            min_area = int(parts[3]) if len(parts) > 3 else 600

            # Pause live viewer so the pipeline can be opened
            was_open = cam_proc is not None and cam_proc.poll() is None
            _kill_viewer()

            try:
                px_w, fx, fy, ppx, ppy, bx, by = _do_capture(idx, color, min_area)
                base = f":{fx:.2f}:{fy:.2f}:{ppx:.2f}:{ppy:.2f}"
                if px_w is not None:
                    response = f"ok:{px_w:.1f}{base}:{bx:.1f}:{by:.1f}"
                else:
                    response = f"none{base}:0:0"
            except Exception as e:
                response = f"error:{e}"
                print(f"[cam_daemon] capture error: {e}", flush=True)

            try:
                conn.sendall((response + "\n").encode())
            except Exception:
                pass
            conn.close()

            # Restart live viewer if it was open
            if was_open:
                env = {**os.environ, "DISPLAY": os.environ.get("DISPLAY", ":2")}
                cam_proc = subprocess.Popen(
                    [sys.executable, SCRIPT], env=env,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"[cam_daemon] viewer restarted pid={cam_proc.pid}", flush=True)

        else:
            conn.close()


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", PORT))
        srv.listen()
        print(f"[cam_daemon] listening on :{PORT}", flush=True)
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=handle, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
