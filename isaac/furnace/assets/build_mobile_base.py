"""
두산로보틱스 Mobile Base (사용자가 SYSCON 이라 부르는 수동 이동 캐비닛) 임시 모델. 별도 USD reference.

치수 850(W) × 600(D) × 825(H) mm = 판매 페이지 값 (PLAN §1.4). 로봇 장착 위치(상판 중앙)·캐스터·풋 치수는 가정.
정확한 CAD 가 없어 외형만 근사한 임시 모델이다 (README·config 에 표시).
구성: 검은 하부 스커트, 흰 캐비닛, 스테인리스 상판, 앞문·서랍, 옆 손잡이, 캐스터 4 + 레벨링 풋 4(내린 상태),
      상판 케이블 구멍, 로봇 장착판, 비상정지 버튼·센서(개념적 외형).
물리: 정적 박스 하나 (고정 바디).
"""

import os
import sys

from pxr import UsdGeom, UsdPhysics

sys.path.insert(0, os.path.dirname(__file__))
import usd_util as U  # noqa: E402


def build(cfg, out_path):
    W_, D_, H_ = cfg["base"]["size_m"]
    st = U.new_stage(out_path, "MobileBase")
    B = "/MobileBase"
    st.GetRootLayer().customLayerData = {"note": "TEMPORARY model. Size from vendor page (Unchained Robotics / Cross Co.); "
                                                 "robot mount position, casters, feet are ASSUMED."}
    white = U.material(st, "cabinet_white", color=(0.86, 0.87, 0.86), rough=0.45)
    black = U.material(st, "skirt_black", color=(0.03, 0.03, 0.03), rough=0.7)
    ss = U.material(st, "stainless", color=(0.62, 0.63, 0.64), rough=0.28, metal=1.0)
    seam = U.material(st, "seam", color=(0.15, 0.15, 0.15), rough=0.6)
    alu = U.material(st, "aluminium", color=(0.7, 0.71, 0.72), rough=0.35, metal=1.0)
    red = U.material(st, "estop_red", color=(0.7, 0.03, 0.02), rough=0.4)
    ylw = U.material(st, "estop_yellow", color=(0.85, 0.65, 0.05), rough=0.5)
    rubber = U.material(st, "rubber", color=(0.05, 0.05, 0.05), rough=0.9)

    sk = 0.10
    top_t = 0.02
    U.box_mesh(st, B + "/skirt", (W_ - 0.04, D_ - 0.04, sk), t=(0, 0, sk / 2 + 0.03), mat=black)
    U.box_mesh(st, B + "/body", (W_ - 0.02, D_ - 0.02, H_ - sk - top_t - 0.03), t=(0, 0, sk + 0.03 + (H_ - sk - top_t - 0.03) / 2), mat=white)
    U.box_mesh(st, B + "/top", (W_, D_, top_t), t=(0, 0, H_ - top_t / 2), mat=ss)
    zc = sk + 0.03 + (H_ - sk - top_t - 0.03) / 2
    hz = H_ - sk - top_t - 0.03
    # 앞문(-y 면) 이음선 + 서랍
    U.box_mesh(st, B + "/door_seam_v", (0.006, 0.004, hz - 0.08), t=(0, -D_ / 2 + 0.008, zc - 0.03), mat=seam)
    U.box_mesh(st, B + "/drawer_seam", (W_ - 0.1, 0.004, 0.006), t=(0, -D_ / 2 + 0.008, H_ - 0.17), mat=seam)
    U.box_mesh(st, B + "/drawer_handle", (0.18, 0.02, 0.02), t=(0, -D_ / 2 - 0.004, H_ - 0.11), mat=alu)
    for sx in (-1, 1):
        U.box_mesh(st, B + f"/door_handle{sx}", (0.02, 0.02, 0.14), t=(sx * 0.05, -D_ / 2 - 0.004, zc), mat=alu)
        # 옆 손잡이 (±x 면)
        U.box_mesh(st, B + f"/side_handle{sx}", (0.03, 0.30, 0.03), t=(sx * (W_ / 2 + 0.04), 0, H_ - 0.12), mat=alu)
        for sy in (-1, 1):
            U.box_mesh(st, B + f"/side_handle_post{sx}{sy}", (0.05, 0.025, 0.025), t=(sx * (W_ / 2 + 0.02), sy * 0.14, H_ - 0.12), mat=alu)
    # 캐스터 + 레벨링 풋 (풋을 내린 정지 상태)
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * (W_ / 2 - 0.09), sy * (D_ / 2 - 0.09)
            U.cylinder(st, B + f"/caster{sx}{sy}", 0.05, 0.035, t=(cx, cy, 0.05), axis="X", mat=rubber)
            U.box_mesh(st, B + f"/caster_fork{sx}{sy}", (0.05, 0.06, 0.04), t=(cx, cy, 0.105), mat=alu)
            fx, fy = sx * (W_ / 2 - 0.035), sy * (D_ / 2 - 0.035)
            U.cylinder(st, B + f"/foot_pad{sx}{sy}", 0.03, 0.012, t=(fx, fy, 0.006), mat=rubber)
            U.cylinder(st, B + f"/foot_screw{sx}{sy}", 0.008, 0.03, t=(fx, fy, 0.027), mat=alu)
    # 상판: 케이블 구멍, 로봇 장착판 (가정: 중앙), 비상정지, 센서
    mx, my = cfg["base"]["robot_mount_xy_m"]
    U.cylinder(st, B + "/cable_hole", 0.035, 0.003, t=(W_ / 2 - 0.12, D_ / 2 - 0.1, H_ + 0.0005), mat=black)
    U.cylinder(st, B + "/robot_mount_plate", 0.13, 0.012, t=(mx, my, H_ + 0.006), mat=alu)
    U.box_mesh(st, B + "/estop_box", (0.08, 0.08, 0.07), t=(-W_ / 2 + 0.08, -D_ / 2 + 0.08, H_ + 0.035), mat=ylw)
    U.cylinder(st, B + "/estop_button", 0.025, 0.025, t=(-W_ / 2 + 0.08, -D_ / 2 + 0.08, H_ + 0.082), mat=red)
    U.box_mesh(st, B + "/sensor_concept", (0.06, 0.04, 0.05), t=(W_ / 2 - 0.06, -D_ / 2 + 0.06, H_ + 0.025), mat=black)

    c = UsdGeom.Cube.Define(st, B + "/Collision")
    c.CreateSizeAttr(1.0)
    U._set_xf(c, (0, 0, H_ / 2), (0, 0, 0), s=(W_, D_, H_))
    UsdGeom.Imageable(c).CreatePurposeAttr("guide")
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    st.GetRootLayer().Save()
    return {"top_z": H_, "mount_xy": (mx, my)}
