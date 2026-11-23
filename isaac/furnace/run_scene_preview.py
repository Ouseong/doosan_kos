"""
scene_preview (PLAN Phase 3): 주물공장 작업구역 + 용해로 + Mobile Base + M1013 + 핀어레이 그리퍼 + metal_plate.
외형·조명 확인용 정지 장면이다 (물리 시뮬레이션 결과 아님). 카메라 4 개 실제 렌더 PNG 저장.

  docker exec -e CUDA_VISIBLE_DEVICES=1 furnace_kos /isaac-sim/python.sh \
      /kos_workspace/isaac/furnace/run_scene_preview.py --headless [--preset dark_foundry|debug_bright]
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "assets"))
sys.path.insert(0, os.path.join(HERE, "sim"))

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(HERE, "config", "default.yaml"))
ap.add_argument("--headless", action="store_true")
ap.add_argument("--preset", default="dark_foundry")
ap.add_argument("--out", default=os.path.join(ROOT, "output", "furnace", "scene_preview"))
ap.add_argument("--res", type=int, nargs=2, default=[1600, 900])
ap.add_argument("--warmup", type=int, default=120)
ap.add_argument("--tilt_deg", type=float, default=25.0, help="대표 장면: 판 기울기")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting",
                     "width": args.res[0], "height": args.res[1]})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

import build_factory  # noqa: E402
import build_furnace  # noqa: E402
import build_mobile_base  # noqa: E402
import build_pin_gripper  # noqa: E402
import build_plate  # noqa: E402
import cameras  # noqa: E402
import lighting  # noqa: E402
import usd_util as U  # noqa: E402
from grasp_geometry import gripper_S_from_link6, link6_from_gripper_S, plate_in_S  # noqa: E402
from m1013_kinematics import M1013, make_T  # noqa: E402

cfg = yaml.safe_load(open(args.config))
os.makedirs(args.out, exist_ok=True)
lay = cfg["layout"]
fx, fy = lay["furnace_xy"]
rim = cfg["furnace"]["rim_height_from_floor_m"]

# ── 1. 자산 USD 생성 (usd/generated/furnace/, gitignore) ──────────────────────────
paths = {k: os.path.join(U.GEN, f"{k}.usda") for k in ("factory", "furnace", "mobile_base", "plate")}
build_factory.build(cfg, paths["factory"])
finfo = build_furnace.build(cfg, paths["furnace"])
build_mobile_base.build(cfg, paths["mobile_base"])
pinfo = build_plate.build(cfg, paths["plate"])
robot_usd = os.path.join(ROOT, "usd/generated/furnace/m1013/m1013_full.usda")

# ── 2. 장면 조립 ────────────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
add_reference_to_stage(paths["factory"], "/World/Factory")
add_reference_to_stage(paths["furnace"], "/World/Furnace")
U._set_xf(stage.GetPrimAtPath("/World/Furnace"), (fx, fy, 0), (0, 0, 0))
add_reference_to_stage(paths["mobile_base"], "/World/MobileBase")
add_reference_to_stage(robot_usd, "/World/m1013")
U._set_xf(stage.GetPrimAtPath("/World/m1013"), (0, 0, lay["robot_base_z"]), (0, 0, 0))
for _ in range(10):
    app.update()

# 대표 장면 자세: 판 중심을 용해로 위 rim+0.30 m, 앞쪽 가장자리가 낮게 tilt 기울임 (IK)
th = np.deg2rad(args.tilt_deg)
Pz = np.array([-np.cos(th), 0, np.sin(th)])        # 판 중심 → 파지 가장자리 (로봇 쪽 위)
Py = np.array([0.0, 1.0, 0.0])
Px = np.cross(Py, Pz)
T_WP = make_T(np.array([fx - 0.10, fy, rim + 0.30]), np.stack([Px, Py, Pz], 1))
T_SP = plate_in_S(cfg["plate"]["size_m"], cfg["grasp"]["insert_depth_m"])
T_W6 = link6_from_gripper_S(T_WP @ np.linalg.inv(T_SP))
kin = M1013()
base_T = make_T(np.array([0, 0, lay["robot_base_z"]]))
q, pe, re = kin.ik(T_W6, q0=np.deg2rad([0, -10, 100, 0, 60, 0]), base=base_T)
print(f"[preview] IK err pos={pe*1000:.2f} mm rot={np.degrees(re):.2f} deg q(deg)={np.degrees(q).round(1).tolist()}")

link6 = "/World/m1013/link_6"
body, plate_ops = build_pin_gripper.add_visual(stage, link6, opening=0.0)

art = None
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI) and p.GetPath().pathString.startswith("/World/m1013"):
        art = p.GetPath().pathString
        break
robot = SingleArticulation(prim_path=art, name="m1013")
world.scene.add(robot)
world.reset()
robot.set_joint_positions(q)
for _ in range(5):
    world.step(render=False)
m = UsdGeom.Xformable(stage.GetPrimAtPath(link6)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
T_W6_isaac = np.array(m).T
print("[preview] link_6 isaac vs fk (mm):", ((T_W6_isaac[:3, 3] - kin.fk(q, base_T)[:3, 3]) * 1000).round(2).tolist())

# 판: 그리퍼 S 프레임 기준 명목 파지 자세로 배치 (시각, 물리 정지)
T_WS = gripper_S_from_link6(T_W6_isaac)
T_WP_actual = T_WS @ T_SP
add_reference_to_stage(paths["plate"], "/World/metal_plate")
pp = stage.GetPrimAtPath("/World/metal_plate")
UsdPhysics.RigidBodyAPI(pp).CreateKinematicEnabledAttr(True)
xf = UsdGeom.Xformable(pp); xf.ClearXformOpOrder()
xf.AddTransformOp().Set(Gf.Matrix4d(T_WP_actual.T.tolist()))

# 조명
lighting.apply(stage, cfg, args.preset)
E = cfg["lighting"]["exposure"]
exp_before = lighting.read_exposure_settings()
if not E["auto"]:
    lighting.set_fixed_exposure(E["film_iso"], E["shutter_s"], E["f_number"])

# ── 3. 카메라 4 개 렌더 ─────────────────────────────────────────────────────────
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from PIL import Image  # noqa: E402

grip_pos = (T_WS @ np.array([0, 0.0157, -0.128, 1]))[:3]
shots = {}
cams = {}
cam_poses = cameras.presets(cfg, grip_pos)
for name, (eye, tgt) in cam_poses.items():
    cam = Camera(prim_path=f"/World/Cameras/{name}", resolution=tuple(args.res))
    cam.initialize()
    set_camera_view(eye=eye, target=tgt, camera_prim_path=f"/World/Cameras/{name}")
    cam.set_focal_length(1.8 if name != "overview" else 1.4)
    cam.set_clipping_range(0.01, 10000.0)        # 기본 near=1 m 이면 가까운 그리퍼가 잘린다
    cams[name] = cam
for _ in range(args.warmup):
    world.step(render=True)
for name, cam in cams.items():
    for _ in range(10):
        world.step(render=True)
    rgba = cam.get_rgba()
    if rgba is None or not rgba.size:
        shots[name] = None
        continue
    png = os.path.join(args.out, f"{args.preset}_{name}.png")
    Image.fromarray(rgba[..., :3].astype(np.uint8)).save(png)
    shots[name] = os.path.relpath(png, ROOT)
    print("[preview] saved", png, "mean", float(rgba[..., :3].mean()))

info = {"preset": args.preset, "renderer": "RTX RayTracedLighting (real-time)", "shots": shots,
        "ik": {"q_deg": np.degrees(q).round(2).tolist(), "pos_err_mm": pe * 1000, "rot_err_deg": float(np.degrees(re))},
        "plate_center_world": T_WP_actual[:3, 3].round(4).tolist(), "plate_mass_kg": pinfo["mass_kg"],
        "furnace": finfo, "gripper_grasp_center_world": grip_pos.round(3).tolist(),
        "cameras": {k: {"eye": v[0].round(3).tolist(), "target": v[1].round(3).tolist()} for k, v in cam_poses.items()}, "exposure_default": exp_before, "exposure_used": lighting.read_exposure_settings(),
        "note": "static preview (no physics stepping of plate); NOT a physics result"}
json.dump(info, open(os.path.join(args.out, f"{args.preset}_info.json"), "w"), indent=2, default=str)
print(json.dumps(info, indent=2, default=str))
app.close()
