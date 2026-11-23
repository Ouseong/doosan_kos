"""
카메라 프리셋 4개 (SPEC §18). eye/target 은 layout 기준으로 계산한다.

  overview     공장 작업구역 전체
  three_quarter 로봇과 용해로를 함께 보는 3/4 view (대표 장면)
  contact      그리퍼–판 접촉부 확대 (link_6 기준으로 런타임에 갱신)
  melt_side    액면과 splash 를 보는 측면(상부 비스듬히)
"""

import numpy as np


def presets(cfg, gripper_pos=None):
    lay = cfg["layout"]
    fx, fy = lay["furnace_xy"]
    rim = cfg["furnace"]["rim_height_from_floor_m"]
    c = {
        "overview": (np.array([-4.8, -5.2, 4.6]), np.array([1.2, 0.3, 0.9])),
        "three_quarter": (np.array([-0.9, -2.7, 2.35]), np.array([0.75, 0.0, 1.05])),
        "melt_side": (np.array([fx + 0.2, fy - 1.25, rim + 0.85]), np.array([fx, fy, rim - 0.15])),
    }
    g = np.array([0.6, 0.0, 1.3]) if gripper_pos is None else np.asarray(gripper_pos)
    c["contact"] = (g + np.array([-0.10, -0.33, 0.02]), g + np.array([0.04, 0.0, -0.06]))
    return c
