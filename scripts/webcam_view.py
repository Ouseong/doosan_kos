#!/usr/bin/env python3
"""Simple USB webcam viewer (non-RealSense).

Usage:
    python3 scripts/webcam_view.py                 # /dev/video0
    python3 scripts/webcam_view.py --dev /dev/video2
"""
import argparse
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/video0", help="V4L2 device path")
    args = ap.parse_args()

    import cv2

    cap = cv2.VideoCapture(args.dev)
    if not cap.isOpened():
        print(f"Cannot open {args.dev}", file=sys.stderr)
        sys.exit(1)

    title = f"Webcam ({args.dev})"
    print(f"{title} — press q to quit")
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow(title, frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
