"""
핀어레이 그리퍼 (사용자 실물 CAD = usd/sim_gripper/*.stl, 설계판 v2 — PLAN §8-1 답 전까지).

add_visual(stage, parent, opening): link_6 아래에 스페이서 Ø80×35 mm + 그리퍼 시각 메시 (브리지와 같은 장착).
    반환: Body prim 경로와 판 Xform translate op (열림 갱신용).
물리 버전(핀 50 개 prismatic + 스프링)은 Phase 4 의 run_robot_charge_demo.py 에서 만든다.
"""

import json
import os
import sys

from pxr import Gf, UsdGeom

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
import usd_util as U  # noqa: E402
from grasp_geometry import MOUNT_SPACER, shift_from_opening  # noqa: E402

GRIPPER_DIR = os.path.join(U.ROOT, "usd", "sim_gripper")


def add_visual(st, parent, opening=0.067, root_name="PinGripper"):
    meta = json.load(open(os.path.join(GRIPPER_DIR, "meta.json")))
    root = f"{parent}/{root_name}"
    st.DefinePrim(root, "Xform")
    looks = root + "/Looks"
    colors = meta["colors"]
    spec = {"body": (0.0, 0.55), "house": (0.0, 0.55), "pcb": (0.0, 0.45), "pins": (0.85, 0.30), "mag": (0.10, 0.35)}
    mats = {k: U.material(st, f"grip_{k}", color=colors[k], metal=spec[k][0], rough=spec[k][1], scope=looks) for k in spec}
    spacer_m = U.material(st, "grip_spacer", color=(0.55, 0.56, 0.58), metal=0.8, rough=0.35, scope=looks)
    U.cylinder(st, root + "/Spacer", 0.040, MOUNT_SPACER, t=(0, 0, MOUNT_SPACER / 2), mat=spacer_m)
    body = root + "/Body"
    bx = UsdGeom.Xform.Define(st, body)
    xf = UsdGeom.Xformable(bx); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, MOUNT_SPACER))
    xf.AddOrientOp().Set(Gf.Quatf(0.0, 1.0, 0.0, 0.0))          # X 축 180°
    U.stl_mesh(st, body + "/BodyMesh", os.path.join(GRIPPER_DIR, "body.stl"), mat=mats["body"])
    ops = []
    s = shift_from_opening(opening)
    for side, sign in (("p", 1), ("n", -1)):
        pl = UsdGeom.Xform.Define(st, f"{body}/Plate_{side}")
        x = UsdGeom.Xformable(pl); x.ClearXformOpOrder()
        op = x.AddTranslateOp(); op.Set(Gf.Vec3d(sign * s, 0, 0))
        ops.append(op)
        for part in ("house", "pcb", "pins", "mag"):
            U.stl_mesh(st, f"{body}/Plate_{side}/{part}", os.path.join(GRIPPER_DIR, f"plate_{side}_{part}.stl"), mat=mats[part])
    return body, ops
