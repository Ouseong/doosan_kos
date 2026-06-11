#!/usr/bin/env python3
"""D435: color-blob object detection (e.g. orange cube) with per-object depth.

YOLO/COCO has no "cube" class, so colored blocks go undetected. This finds
objects by HSV color instead — robust for known-color items — and samples the
aligned depth frame at each blob center to report distance in metres.

Usage:
    DISPLAY=:2 python3 scripts/d435_color.py                 # live, orange preset
    python3 scripts/d435_color.py --color red --min-area 800
    python3 scripts/d435_color.py --tune                     # interactive HSV sliders
    python3 scripts/d435_color.py --headless --save out

Press 'q'/ESC to quit. In --tune mode, drag the HSV trackbars until only the
cube is white in the mask, then read the printed values and bake them in.
"""
import argparse
import sys

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30

# HSV ranges (OpenCV H is 0-179). Orange wraps a bit toward red.
PRESETS = {
    "orange": [((5, 110, 110), (22, 255, 255))],
    "red":    [((0, 120, 100), (10, 255, 255)), ((170, 120, 100), (179, 255, 255))],
    "green":  [((40, 80, 60), (85, 255, 255))],
    "blue":   [((95, 120, 60), (130, 255, 255))],
    "yellow": [((22, 110, 110), (35, 255, 255))],
}


def depth_of_blob(depth_img_m, blob_mask, x, y, w, h, clip=6.0):
    """Median depth (m) over the blob's OWN pixels inside its bbox, rejecting
    holes (0) and saturated/invalid returns (>=clip, e.g. the 65.535 sat code).
    Sampling the color mask — not the whole box — avoids background bleed."""
    sub_d = depth_img_m[max(0, y):y + h, max(0, x):x + w]
    sub_m = blob_mask[max(0, y):y + h, max(0, x):x + w]
    vals = sub_d[(sub_m > 0) & (sub_d > 0.1) & (sub_d < clip)]
    return float(np.median(vals)) if vals.size else 0.0


def make_mask(hsv, ranges):
    mask = None
    for lo, hi in ranges:
        m = __import__("cv2").inRange(hsv, np.array(lo), np.array(hi))
        mask = m if mask is None else (mask | m)
    return mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--color", default="orange", choices=list(PRESETS))
    ap.add_argument("--min-area", type=int, default=600,
                    help="ignore blobs smaller than this many px")
    ap.add_argument("--tune", action="store_true", help="interactive HSV sliders")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--save", metavar="PREFIX")
    args = ap.parse_args()

    import cv2
    ranges = PRESETS[args.color]

    if args.tune:
        cv2.namedWindow("tune")
        for name, val in (("Hlo", ranges[0][0][0]), ("Hhi", ranges[0][1][0]),
                          ("Slo", ranges[0][0][1]), ("Shi", 255),
                          ("Vlo", ranges[0][0][2]), ("Vhi", 255)):
            cv2.createTrackbar(name, "tune", val, 255, lambda x: None)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    profile = pipeline.start(cfg)
    dev = profile.get_device()
    dsen = dev.first_depth_sensor()
    depth_scale = dsen.get_depth_scale()  # raw z16 -> metres
    # Flat/shiny/low-texture scenes (e.g. metal pegboard) need the IR projector
    # at full power or depth is mostly holes (~13% valid -> ~60%+ with this).
    try:
        if dsen.supports(rs.option.emitter_enabled):
            dsen.set_option(rs.option.emitter_enabled, 1)
        if dsen.supports(rs.option.laser_power):
            dsen.set_option(rs.option.laser_power,
                            dsen.get_option_range(rs.option.laser_power).max)
    except Exception as e:
        print("laser/emitter setup skipped:", e)
    print(f"D435 S/N {dev.get_info(rs.camera_info.serial_number)} | "
          f"color={args.color} min_area={args.min_area} depth_scale={depth_scale}")
    align = rs.align(rs.stream.color)
    # depth post-processing — fills holes and denoises the sparse stereo map
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    holefill = rs.hole_filling_filter(2)
    kernel = np.ones((5, 5), np.uint8)

    try:
        for _ in range(15):
            pipeline.wait_for_frames()

        while True:
            frames = align.process(pipeline.wait_for_frames())
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color or not depth:
                continue
            depth = spatial.process(depth)
            depth = temporal.process(depth)
            depth = holefill.process(depth)
            img = np.asanyarray(color.get_data())
            depth_m = np.asanyarray(depth.get_data()).astype(np.float32) * depth_scale
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            if args.tune:
                g = lambda n: cv2.getTrackbarPos(n, "tune")
                cur = [((g("Hlo"), g("Slo"), g("Vlo")),
                        (g("Hhi"), g("Shi"), g("Vhi")))]
                mask = make_mask(hsv, cur)
            else:
                mask = make_mask(hsv, ranges)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            lines = []
            for c in cnts:
                area = cv2.contourArea(c)
                if area < args.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                dist = depth_of_blob(depth_m, mask, x, y, w, h)
                dtxt = f"{dist:.2f}m" if dist > 0 else "--"
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 165, 255), 2)
                cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
                cv2.putText(img, f"{args.color} {dtxt}", (x, y - 6 if y > 16 else y + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)
                lines.append(f"  {args.color} area={int(area)} center=({cx},{cy}) depth {dtxt}")

            if lines:
                print("detections:\n" + "\n".join(lines))

            if args.save:
                cv2.imwrite(f"{args.save}.png", img)
                cv2.imwrite(f"{args.save}_mask.png", mask)
                print(f"saved {args.save}.png / {args.save}_mask.png")
            if args.headless or args.save:
                break

            cv2.imshow("D435 color detect (q to quit)", img)
            if args.tune:
                cv2.imshow("mask", mask)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                if args.tune:
                    print("final HSV:", cur)
                break
    finally:
        pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        print("stopped")


if __name__ == "__main__":
    sys.exit(main())
