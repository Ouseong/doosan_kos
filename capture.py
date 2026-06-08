#!/usr/bin/env python3
"""
RealSense D435 프레임 캡처 — 'c' 누르면 저장, 'q' 종료
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

OUT_DIR = "output/captures"
os.makedirs(OUT_DIR, exist_ok=True)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
pipeline.start(config)
print("카메라 연결됨.  'c' = 캡처  /  'q' = 종료")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data())

        cv2.imshow("Color", color)
        cv2.imshow("Depth", cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('c'):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"{OUT_DIR}/{ts}_color.png", color)
            cv2.imwrite(f"{OUT_DIR}/{ts}_depth.png",
                        cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03),
                                          cv2.COLORMAP_JET))
            np.save(f"{OUT_DIR}/{ts}_depth_raw.npy", depth)
            print(f"저장 → {OUT_DIR}/{ts}_*.png")
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
