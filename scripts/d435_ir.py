#!/usr/bin/env python3
"""D435: show the infrared (IR) camera stream(s).

The D435 has two IR imagers (left=1, right=2) that feed the stereo depth.
This displays them raw (y8). The IR projector dots are visible on surfaces —
that's normal and is what gives textureless scenes their depth.

Usage:
    DISPLAY=:2 python3 scripts/d435_ir.py            # both IR side-by-side
    python3 scripts/d435_ir.py --single              # left IR only
    python3 scripts/d435_ir.py --no-emitter          # turn the IR dot projector off
    python3 scripts/d435_ir.py --headless --save out  # one frame, no GUI
"""
import argparse
import sys

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--single", action="store_true", help="left IR only")
    ap.add_argument("--no-emitter", action="store_true", help="disable IR projector")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--save", metavar="PREFIX")
    args = ap.parse_args()

    import cv2

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, FPS)
    if not args.single:
        cfg.enable_stream(rs.stream.infrared, 2, W, H, rs.format.y8, FPS)
    profile = pipeline.start(cfg)
    dev = profile.get_device()
    dsen = dev.first_depth_sensor()
    try:
        if dsen.supports(rs.option.emitter_enabled):
            dsen.set_option(rs.option.emitter_enabled, 0 if args.no_emitter else 1)
    except Exception as e:
        print("emitter setup skipped:", e)
    print(f"D435 S/N {dev.get_info(rs.camera_info.serial_number)} | "
          f"IR {'left' if args.single else 'left+right'} | "
          f"emitter {'OFF' if args.no_emitter else 'ON'}")

    try:
        for _ in range(10):
            pipeline.wait_for_frames()
        while True:
            frames = pipeline.wait_for_frames()
            left = frames.get_infrared_frame(1)
            if not left:
                continue
            li = np.asanyarray(left.get_data())
            view = cv2.cvtColor(li, cv2.COLOR_GRAY2BGR)
            if not args.single:
                right = frames.get_infrared_frame(2)
                if right:
                    ri = cv2.cvtColor(np.asanyarray(right.get_data()),
                                      cv2.COLOR_GRAY2BGR)
                    cv2.putText(view, "L", (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (0, 255, 0), 2)
                    cv2.putText(ri, "R", (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (0, 255, 0), 2)
                    view = np.hstack((view, ri))

            if args.save:
                cv2.imwrite(f"{args.save}.png", view)
                print(f"saved {args.save}.png")
            if args.headless or args.save:
                break

            cv2.imshow("D435 IR (q to quit)", view)
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
