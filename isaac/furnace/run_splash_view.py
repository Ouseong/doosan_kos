"""
Newton splash 결과 여러 건을 Isaac Sim 공장 장면에 용해로를 나란히 놓고 반복 재생 (GUI) 하거나 mp4 로 저장한다.

이것은 재생(PLAYBACK)이다. 물리는 Newton (run_splash_case.py) 이 계산했고, Isaac Sim 은 저장된 입자 위치와
판 자세를 그대로 그린다. 유체는 물 수준 등온 모델이며 실제 용탕이 아니다.

  GUI 반복 재생:
    docker exec -e DISPLAY=:1 -e CUDA_VISIBLE_DEVICES=2 furnace_kos /isaac-sim/python.sh \
        /kos_workspace/isaac/furnace/run_splash_view.py --cases low_vertical high_flat
  mp4 저장 (headless):
    ... run_splash_view.py --cases low_vertical high_flat --headless --record
"""

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "assets"))
sys.path.insert(0, os.path.join(HERE, "sim"))

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(HERE, "config", "default.yaml"))
ap.add_argument("--cases", nargs="+", default=["low_vertical", "high_flat"])
ap.add_argument("--dir", default=os.path.join(ROOT, "output", "furnace", "splash_cases"))
ap.add_argument("--spacing", type=float, default=2.0, help="용해로 사이 간격 [m]")
ap.add_argument("--y0", type=float, default=0.3, help="첫 용해로 y [m] (판 랙 y=-0.8 을 피함)")
ap.add_argument("--speed", type=float, default=0.5, help="재생 속도 배율 (0.5 = 슬로모션 2 배)")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--record", action="store_true", help="프레임 PNG + mp4 저장 후 종료")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting", "width": 1600, "height": 900})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from pxr import Gf, UsdGeom, Vt  # noqa: E402

import build_factory  # noqa: E402
import build_furnace  # noqa: E402
import lighting  # noqa: E402
import usd_util as U  # noqa: E402

cfg = yaml.safe_load(open(args.config))
fx = cfg["layout"]["furnace_xy"][0]
rim = cfg["furnace"]["rim_height_from_floor_m"]
floor_z = rim - cfg["furnace"]["crucible_depth_m"]

paths = {k: os.path.join(U.GEN, f"{k}.usda") for k in ("factory", "furnace")}
build_factory.build(cfg, paths["factory"])
build_furnace.build(cfg, paths["furnace"])
omni.usd.get_context().new_stage()
for _ in range(3):
    app.update()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
add_reference_to_stage(paths["factory"], "/World/Factory")

n = len(args.cases)
ys = [args.y0 + i * args.spacing for i in range(n)]
yc = float(np.mean(ys))
cases = []
fluid_mat = plate_mat = None
for i, (tag, y) in enumerate(zip(args.cases, ys)):
    fp = f"/World/Furnace_{i}"
    add_reference_to_stage(paths["furnace"], fp)
    U._set_xf(stage.GetPrimAtPath(fp), (fx, y, 0), (0, 0, 0))
    z = np.load(os.path.join(args.dir, f"{tag}_trajectory.npz"))
    P = z["particles_sub"].astype(np.float32)
    off = np.array([fx, y, floor_z], np.float32)
    pts = UsdGeom.Points.Define(stage, f"/World/Playback_{i}/fluid")
    pts.CreateWidthsAttr(Vt.FloatArray([0.010] * P.shape[1]))
    if fluid_mat is None:
        fluid_mat = U.material(stage, "pb_fluid", color=(1.0, 0.42, 0.08), rough=0.6, emissive=(1.0, 0.35, 0.05),
                               emissive_intensity=0.6, scope="/World/Looks")
        plate_mat = U.material(stage, "pb_plate", color=(0.55, 0.55, 0.58), rough=0.45, metal=0.7, scope="/World/Looks")
    U.bind(pts, fluid_mat)
    plate = UsdGeom.Cube.Define(stage, f"/World/Playback_{i}/metal_plate")
    plate.CreateSizeAttr(1.0)
    U.bind(plate, plate_mat)
    xf = UsdGeom.Xformable(plate)
    ops = (xf.AddTranslateOp(), xf.AddOrientOp(), xf.AddScaleOp())
    ops[2].Set(Gf.Vec3f(*map(float, z["plate_size"])))
    cases.append(dict(tag=tag, P=P, plate=z["plate"], every=int(z["every_n_frames"]), fps=float(z["fps"]),
                      off=off, pts=pts, ops=ops))
    print(f"[view] {tag}: y={y:+.2f} m, snapshots={len(P)}, particles shown={P.shape[1]}", flush=True)

for _ in range(5):
    app.update()
# 정지 용탕 원판·찌꺼기는 숨긴다 (입자가 액체)
hidden = 0
for p in stage.Traverse():
    if p.GetName() == "melt_surface" or p.GetName().startswith("crust"):
        UsdGeom.Imageable(p).MakeInvisible()
        hidden += 1
print("[view] 정지 용탕 표면 숨김:", hidden, flush=True)

lighting.apply(stage, cfg, "dark_foundry")
E = cfg["lighting"]["exposure"]
lighting.set_fixed_exposure(E["film_iso"], E["shutter_s"], E["f_number"])
span = (n - 1) * args.spacing
eye = np.array([fx - 1.6 - 0.45 * span, yc, rim + 1.5 + 0.3 * span])
tgt = np.array([fx, yc, rim - 0.3])
cam_path = "/OmniverseKit_Persp"
set_camera_view(eye=eye, target=tgt, camera_prim_path=cam_path)
UsdGeom.Camera(stage.GetPrimAtPath(cam_path)).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1e4))
print("[view] 왼쪽→오른쪽 순서 (카메라 기준):", list(reversed(args.cases)), flush=True)


def show(k):
    for c in cases:
        kk = min(k, len(c["P"]) - 1)
        f = kk * c["every"]
        c["pts"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(c["P"][kk] + c["off"]))
        p, q = c["plate"][f, :3] + c["off"], c["plate"][f, 3:]
        c["ops"][0].Set(Gf.Vec3d(*map(float, p)))
        c["ops"][1].Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))


nsnap = max(len(c["P"]) for c in cases)
dt = cases[0]["every"] / cases[0]["fps"]
if args.record:
    from isaacsim.sensors.camera import Camera  # noqa: E402
    from PIL import Image, ImageDraw  # noqa: E402
    out = os.path.join(ROOT, "output", "furnace", "splash_view")
    os.makedirs(os.path.join(out, "frames"), exist_ok=True)
    import omni.timeline  # noqa: E402
    omni.timeline.get_timeline_interface().play()            # Camera annotator 는 타임라인이 돌아야 프레임을 낸다
    cam = Camera(prim_path="/World/RecCam", resolution=(1600, 900))
    cam.initialize()
    set_camera_view(eye=eye, target=tgt, camera_prim_path="/World/RecCam")
    cam.set_clipping_range(0.01, 1e4)
    _uc = UsdGeom.Camera(stage.GetPrimAtPath("/World/RecCam"))      # GUI 뷰포트와 같은 화각
    _uc.GetFocalLengthAttr().Set(18.147562)
    _uc.GetHorizontalApertureAttr().Set(20.955)
    _uc.GetVerticalApertureAttr().Set(20.955 * 900 / 1600)
    for _ in range(30):
        app.update()
    for k in range(nsnap):
        show(k)
        for _ in range(6):
            app.update()
        rgba = cam.get_rgba()
        tries = 0
        while (rgba is None or rgba.ndim != 3) and tries < 50:      # 카메라 버퍼 준비 대기
            app.update(); rgba = cam.get_rgba(); tries += 1
        im = Image.fromarray(rgba[..., :3].astype(np.uint8))
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, 1600, 28], fill=(0, 0, 0))
        d.text((8, 8), f"PLAYBACK of Newton MPM results (water-like fluid, not molten metal)  t={k * dt:.2f}s  "
                       f"left->right: {', '.join(reversed(args.cases))}", fill=(255, 255, 255))
        im.save(os.path.join(out, "frames", f"{k:04d}.png"))
    mp4 = os.path.join(out, "splash_compare.mp4")
    import shutil
    if shutil.which("ffmpeg") is None:
        print("[view] ffmpeg 없음 — 호스트에서: ffmpeg -framerate", round(args.speed / dt), "-i", os.path.join(out, "frames", "%04d.png"), mp4, flush=True)
    else:
      subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(round(args.speed / dt)), "-i",
                    os.path.join(out, "frames", "%04d.png"), "-pix_fmt", "yuv420p", mp4], check=False)
    print("[view] saved", mp4)
    app.close()
    sys.exit(0)

print("[view] 반복 재생 시작 — 창을 닫으면 종료", flush=True)
k, t_last = 0, time.time()
while app.is_running():
    show(k)
    app.update()
    if time.time() - t_last >= dt / args.speed:
        t_last = time.time()
        k = (k + 1) % (nsnap + int(0.5 / dt))          # 끝에서 0.5 s 멈춘 뒤 처음부터
app.close()
