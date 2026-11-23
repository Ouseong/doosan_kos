"""
Newton 녹화 결과 재생 렌더 (PLAN Phase 0-4 B-2 방식). Isaac Sim 은 물리를 계산하지 않고, Newton 이 저장한
trajectory.npz 의 입자 위치(부분 표본)·판 자세를 UsdGeom.Points / 박스로 장면에 넣어 렌더만 한다.
모든 출력 이미지에 "PLAYBACK of recorded Newton result" 를 표시한다. 물리 결과가 아니라 재생이다.

  docker exec -e CUDA_VISIBLE_DEVICES=1 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_playback_render.py \
      --headless --npz /kos_workspace/output/furnace/fluid_plate_test/phase1_suite/free_drop_trajectory.npz
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "assets"))
sys.path.insert(0, os.path.join(HERE, "sim"))

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(HERE, "config", "default.yaml"))
ap.add_argument("--headless", action="store_true")
ap.add_argument("--npz", required=True)
ap.add_argument("--times", type=float, nargs="+", default=[0.0, 0.25, 0.35, 0.6])
ap.add_argument("--out", default=os.path.join(ROOT, "output", "furnace", "playback"))
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting", "width": 1280, "height": 720})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
from pxr import Gf, UsdGeom, Vt  # noqa: E402

import build_factory  # noqa: E402
import build_furnace  # noqa: E402
import lighting  # noqa: E402
import usd_util as U  # noqa: E402

cfg = yaml.safe_load(open(args.config))
lay = cfg["layout"]
fx, fy = lay["furnace_xy"]
rim = cfg["furnace"]["rim_height_from_floor_m"]
floor_z = rim - cfg["furnace"]["crucible_depth_m"]
os.makedirs(args.out, exist_ok=True)
tag = os.path.basename(args.npz).replace("_trajectory.npz", "")

paths = {k: os.path.join(U.GEN, f"{k}.usda") for k in ("factory", "furnace")}
build_factory.build(cfg, paths["factory"])
build_furnace.build(cfg, paths["furnace"])
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
add_reference_to_stage(paths["factory"], "/World/Factory")
add_reference_to_stage(paths["furnace"], "/World/Furnace")
U._set_xf(stage.GetPrimAtPath("/World/Furnace"), (fx, fy, 0), (0, 0, 0))
for _ in range(5):
    app.update()
# 재생 중에는 정지 용탕 원판·찌꺼기를 숨긴다 (입자가 액체를 나타냄)
for p in stage.Traverse():
    n = p.GetPath().pathString
    if "/Furnace/Visual/melt_surface" in n or "/Furnace/Visual/crust" in n:
        UsdGeom.Imageable(p).MakeInvisible()

z = np.load(args.npz)
P = z["particles_sub"].astype(np.float32)
every, fps = int(z["every_n_frames"]), float(z["fps"])
plate = z["plate"]
lx, ly, lz = z["plate_size"]           # Newton 판 박스: x=긴 변, y=짧은 변, z=두께
off = np.array([fx, fy, floor_z], np.float32)

pts = UsdGeom.Points.Define(stage, "/World/Playback/fluid")
pts.CreateWidthsAttr(Vt.FloatArray([0.009] * P.shape[1]))
U.bind(pts, U.material(stage, "pb_fluid", color=(1.0, 0.4, 0.08), rough=0.7, emissive=(1.0, 0.35, 0.05),
                       emissive_intensity=0.35, scope="/World/Looks"))
plate_prim = None
if len(plate):
    plate_prim = UsdGeom.Cube.Define(stage, "/World/Playback/metal_plate")
    plate_prim.CreateSizeAttr(1.0)
    U.bind(plate_prim, U.material(stage, "pb_plate", color=(0.5, 0.5, 0.52), rough=0.5, metal=0.6, scope="/World/Looks"))

lighting.apply(stage, cfg, "dark_foundry")
E = cfg["lighting"]["exposure"]
lighting.set_fixed_exposure(E["film_iso"], E["shutter_s"], E["f_number"])
cam = Camera(prim_path="/World/Cameras/melt_side", resolution=(1280, 720))
cam.initialize()
set_camera_view(eye=np.array([fx - 0.35, fy - 1.7, rim + 1.0]), target=np.array([fx, fy, rim - 0.15]),
                camera_prim_path="/World/Cameras/melt_side")
cam.set_clipping_range(0.01, 1e4)
world.reset()
saved = []
for t in args.times:
    k = min(len(P) - 1, int(round(t * fps / every)))
    f = k * every
    pts.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(P[k] + off))
    if plate_prim is not None:
        p, q = plate[f, :3] + off, plate[f, 3:]          # q = xyzw
        xf = UsdGeom.Xformable(plate_prim); xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, p)))
        xf.AddOrientOp().Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))
        xf.AddScaleOp().Set(Gf.Vec3f(float(lx), float(ly), float(lz)))
    for _ in range(40):
        world.step(render=True)
    rgba = cam.get_rgba()
    im = Image.fromarray(rgba[..., :3].astype(np.uint8))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 1280, 26], fill=(0, 0, 0))
    d.text((8, 6), f"PLAYBACK of recorded Newton MPM result ({tag}, t={(f + 1) / fps:.2f}s) - rendered in Isaac Sim, NOT computed by Isaac. "
                   f"Fluid = isothermal water-like model, not molten metal. Particles = random subsample.", fill=(255, 255, 255))
    png = os.path.join(args.out, f"{tag}_t{int(round(t * 100)):03d}.png")
    im.save(png)
    saved.append(png)
    print("[playback] saved", png)
app.close()
