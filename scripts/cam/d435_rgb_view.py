#!/usr/bin/env python3
"""Live RGB viewer for D435 S/N 043322073704 (gripper-mounted camera).

Reads /home/oem/Desktop/doosan_kos/.cam_color each frame and draws a
bounding box around the detected blob of that color.

Usage:
    DISPLAY=:2 python3 scripts/d435_rgb_view.py
    python3 scripts/d435_rgb_view.py --serial 043322073704
"""
import argparse
import os
import signal
import sys

SERIAL = "043322073704"
COLOR_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))), ".cam_color")  # scripts/cam → scripts → root
# Latest per-frame detection, shared so cam_daemon can answer 'capture' from the
# live viewer WITHOUT killing it (the D435 pipeline is single-client).
LATEST_FILE = "/tmp/cam_latest.txt"

PRESETS = {
    "orange": [((5,  110, 110), (22,  255, 255))],
    "red":    [((0,  120, 100), (10,  255, 255)), ((170, 120, 100), (179, 255, 255))],
    "green":  [((40,  80,  60), (85,  255, 255))],
    "blue":   [((95, 120,  60), (130, 255, 255))],
    "yellow": [((22, 110, 110), (35,  255, 255))],
}

_running = True


def _sigterm(_sig, _frame):
    global _running
    _running = False


signal.signal(signal.SIGTERM, _sigterm)
signal.signal(signal.SIGINT, _sigterm)


def _read_color() -> str:
    try:
        return open(COLOR_FILE).read().strip()
    except Exception:
        return "orange"


def _draw_bbox(img, color_name: str):
    import cv2
    import numpy as np
    h_img, w_img = img.shape[:2]
    cx, cy = w_img // 2, h_img // 2          # dead center of the frame (≈ 320,240)

    # ── detect the largest blob (same target as cam_daemon) ──────────────
    ranges = PRESETS.get(color_name, PRESETS["orange"])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
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
        if area > 400 and area > best_area:
            best, best_area = c, area

    # ── always draw the center crosshair so the target is visible ────────
    cv2.drawMarker(img, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 22, 1)
    cv2.circle(img, (cx, cy), 7, (255, 255, 255), 1)

    if best is None:
        return img, None

    # Largest blob only (ignores stray same-colour specks) — centroid + size
    # match cam_daemon._detect_blob exactly so the shared measurement is valid.
    blob = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(blob, [best], -1, 255, -1)
    px_size = float(np.sqrt(cv2.countNonZero(blob)))
    M = cv2.moments(blob)
    bx = M["m10"] / M["m00"] if M["m00"] else cx
    by = M["m01"] / M["m00"] if M["m00"] else cy

    cv2.drawContours(img, [best], -1, (0, 220, 255), 2)
    cv2.circle(img, (int(bx), int(by)), 5, (0, 220, 255), -1)
    cv2.line(img, (cx, cy), (int(bx), int(by)), (0, 220, 255), 1)   # offset vector
    cv2.putText(img, f"{color_name}  off x{bx-cx:+.0f} y{by-cy:+.0f}px",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
    return img, (px_size, bx, by)


def main():
    global _running
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default=SERIAL)
    args = ap.parse_args()

    import cv2
    import numpy as np
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    cfg = rs.config()
    # Use the requested serial if connected, else whatever RealSense is present
    # (robust to swapping the camera / different unit).
    serials = [d.get_info(rs.camera_info.serial_number) for d in rs.context().devices]
    if args.serial in serials:
        cfg.enable_device(args.serial)
    elif serials:
        cfg.enable_device(serials[0])
        print(f"D435 {args.serial} not found — using {serials[0]}", flush=True)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(cfg)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    fx, fy, ppx, ppy = intr.fx, intr.fy, intr.ppx, intr.ppy
    base = f":{fx:.2f}:{fy:.2f}:{ppx:.2f}:{ppy:.2f}"

    def _publish(det):
        # write the latest detection in cam_daemon's response format (atomic)
        if det is not None:
            line = f"ok:{det[0]:.1f}{base}:{det[1]:.1f}:{det[2]:.1f}"
        else:
            line = f"none{base}:0:0"
        try:
            tmp = LATEST_FILE + ".tmp"
            with open(tmp, "w") as f:
                f.write(line)
            os.replace(tmp, LATEST_FILE)
        except Exception:
            pass

    print(f"D435 S/N:{args.serial} — q to quit", flush=True)

    title = f"D435  S/N:{args.serial}  (q to quit)"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 960, 720)

    frame_count = 0
    current_color = _read_color()

    try:
        for _ in range(10):
            pipeline.wait_for_frames()

        while _running:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            color = frames.get_color_frame()
            if not color:
                continue

            img = np.asanyarray(color.get_data()).copy()

            # Re-read color file every 15 frames (~2x/sec at 30fps)
            frame_count += 1
            if frame_count % 15 == 0:
                current_color = _read_color()

            img, det = _draw_bbox(img, current_color)
            _publish(det)

            cv2.imshow(title, img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            try:
                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    finally:
        _running = False
        pipeline.stop()
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except Exception:
            pass
        print("D435 viewer exited.", flush=True)


if __name__ == "__main__":
    sys.exit(main())
