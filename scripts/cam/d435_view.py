#!/usr/bin/env python3
"""Open Intel RealSense D435 and show live Color + Depth streams.

Usage:
    DISPLAY=:2 python3 scripts/d435_view.py            # live OpenCV window (q to quit)
    python3 scripts/d435_view.py --headless --save out  # no GUI, save one frame pair

Depth is colorized with a JET map and shown side-by-side with the color image.
"""
import argparse
import sys

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true",
                    help="no GUI window; capture frames and exit")
    ap.add_argument("--save", metavar="PREFIX",
                    help="save color/depth PNG with this path prefix")
    args = ap.parse_args()

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)

    profile = pipeline.start(cfg)
    dev = profile.get_device()
    print(f"Streaming D435  S/N {dev.get_info(rs.camera_info.serial_number)}  "
          f"FW {dev.get_info(rs.camera_info.firmware_version)}  @ {W}x{H}/{FPS}fps")

    align = rs.align(rs.stream.color)
    import cv2

    try:
        # let auto-exposure settle
        for _ in range(15):
            pipeline.wait_for_frames()

        while True:
            frames = align.process(pipeline.wait_for_frames())
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color or not depth:
                continue

            color_img = np.asanyarray(color.get_data())
            depth_img = np.asanyarray(depth.get_data())
            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
            combined = np.hstack((color_img, depth_vis))

            if args.save:
                cv2.imwrite(f"{args.save}_color.png", color_img)
                cv2.imwrite(f"{args.save}_depth.png", depth_vis)
                print(f"saved {args.save}_color.png / {args.save}_depth.png")

            if args.headless or args.save:
                break

            cv2.imshow("D435  Color | Depth   (q to quit)", combined)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
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
