"""
M1013 USD 생성 (furnace-charging 용).

eb1f4a4^ 에서 지워진 isaac/urdf_to_usd_v2.py + isaac/assemble_m1013.py 를 합쳐 정리한 것.
바뀐 점:
  - 결과물을 /tmp 대신 usd/generated/furnace/m1013/ 에 저장 (gitignore)
  - URDF 의 package:// mesh 경로를 assets_external/doosan-robot2 의 절대경로로 바꿔서 import
    (컨테이너의 /ros2_ws 워크스페이스 없이도 동작)
  - 생성 직후 빈 stage 에 로드해 joint 6개 / 한계 / link_6 / ArticulationRoot 를 확인하고
    output/furnace/phase0/m1013_check.json 에 기록. --screenshot 이면 렌더 PNG 도 저장

입력:  assets_external/doosan-robot2/dsr_description2/urdf/m1013.urdf (공식 doosan-robot2, SOURCE.txt 에 커밋)
출력:  usd/generated/furnace/m1013/m1013.usd            (URDF importer 결과)
       usd/generated/furnace/m1013/m1013_full.usda      (importer 의 base+physics sublayer 를 묶은 단일 진입점)

실행 (furnace_kos 컨테이너, 리포 루트 = /kos_workspace):
  /isaac-sim/python.sh /kos_workspace/isaac/furnace/assets/build_m1013.py [--screenshot]
"""

import argparse
import json
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

parser = argparse.ArgumentParser()
parser.add_argument("--urdf", default=os.path.join(
    ROOT, "assets_external/doosan-robot2/dsr_description2/urdf/m1013.urdf"))
parser.add_argument("--pkg_root", default=os.path.join(ROOT, "assets_external/doosan-robot2"),
                    help="package://<pkg>/... 를 <pkg_root>/<pkg>/... 로 바꾼다")
parser.add_argument("--out_dir", default=os.path.join(ROOT, "usd/generated/furnace/m1013"))
parser.add_argument("--report", default=os.path.join(ROOT, "output/furnace/phase0/m1013_check.json"))
parser.add_argument("--screenshot", action="store_true")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import omni.kit.commands
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdPhysics

enable_extension("isaacsim.asset.importer.urdf")
app.update()
from isaacsim.asset.importer.urdf import _urdf

os.makedirs(args.out_dir, exist_ok=True)
os.makedirs(os.path.dirname(args.report), exist_ok=True)

# ── 1. package:// → 절대경로 ────────────────────────────────────────────────
with open(args.urdf) as f:
    urdf_txt = f.read()
urdf_txt = re.sub(r"package://([^/\"]+)/", lambda m: f"{args.pkg_root}/{m.group(1)}/", urdf_txt)
missing = [p for p in re.findall(r'filename="([^"]+)"', urdf_txt) if not os.path.exists(p)]
if missing:
    raise SystemExit(f"[m1013] mesh 없음: {missing[:3]} ... (assets_external/doosan-robot2 확인)")
resolved_urdf = os.path.join(args.out_dir, "m1013_resolved.urdf")
with open(resolved_urdf, "w") as f:
    f.write(urdf_txt)

# ── 2. URDF import (설정은 예전 urdf_to_usd_v2.py 와 같다) ─────────────────────
usd_path = os.path.join(args.out_dir, "m1013.usd")
_, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = True
cfg.convex_decomp = False
cfg.import_inertia_tensor = True
cfg.fix_base = True
cfg.make_default_prim = True
cfg.self_collision = False
cfg.create_physics_scene = True
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
cfg.default_drive_strength = 1e7
cfg.default_position_drive_damping = 1e5
result = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=resolved_urdf,
                                   import_config=cfg, dest_path=usd_path)
print(f"[m1013] import result: {result}")

# ── 3. 단일 진입점 m1013_full.usda (importer 가 sublayer 로 나눠 만드는 경우 묶는다) ──
conf = os.path.join(args.out_dir, "configuration")
base, physics = (os.path.join(conf, f"m1013_{k}.usd") for k in ("base", "physics"))
full_path = os.path.join(args.out_dir, "m1013_full.usda")
if os.path.exists(base) and os.path.exists(physics):
    if os.path.exists(full_path):
        os.remove(full_path)
    dst = Usd.Stage.CreateNew(full_path)
    dst.SetDefaultPrim(dst.DefinePrim("/m1013", "Xform"))
    UsdGeom.SetStageUpAxis(dst, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(dst, 1.0)
    # 상대경로로 넣어 폴더째 옮겨도 열리게 한다
    dst.GetRootLayer().subLayerPaths.append("./configuration/m1013_base.usd")
    dst.GetRootLayer().subLayerPaths.append("./configuration/m1013_physics.usd")
    dst.GetRootLayer().Save()
    entry = full_path
else:
    entry = usd_path
print(f"[m1013] entry USD: {entry}")

# ── 4. 빈 stage 에 reference 로 로드해서 확인 ─────────────────────────────────
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
import omni.usd

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
add_reference_to_stage(usd_path=entry, prim_path="/World/m1013")
for _ in range(5):
    app.update()

report = {"urdf": os.path.relpath(args.urdf, ROOT), "entry_usd": os.path.relpath(entry, ROOT),
          "import_result": bool(result[0]) if isinstance(result, tuple) else bool(result)}
types = {}
art_root, link6, joints = None, None, []
for p in stage.Traverse():
    path = p.GetPath().pathString
    if not path.startswith("/World/m1013"):
        continue
    types[p.GetTypeName()] = types.get(p.GetTypeName(), 0) + 1
    if art_root is None and p.HasAPI(UsdPhysics.ArticulationRootAPI):
        art_root = path
    if p.GetName() == "link_6" and link6 is None:
        link6 = path
    if p.IsA(UsdPhysics.RevoluteJoint):
        j = UsdPhysics.RevoluteJoint(p)
        drive = UsdPhysics.DriveAPI.Get(p, "angular")
        joints.append({
            "name": p.GetName(), "axis": j.GetAxisAttr().Get(),
            "lower_deg": j.GetLowerLimitAttr().Get(), "upper_deg": j.GetUpperLimitAttr().Get(),
            "drive_stiffness": drive.GetStiffnessAttr().Get() if drive else None,
            "drive_damping": drive.GetDampingAttr().Get() if drive else None,
            "max_force": drive.GetMaxForceAttr().Get() if drive else None,
        })
report.update({"prim_types": types, "articulation_root": art_root, "link_6": link6,
               "revolute_joints": joints})

if art_root:
    robot = SingleArticulation(prim_path=art_root, name="m1013")
    world.scene.add(robot)
    world.reset()
    report["dof_names"] = list(robot.dof_names)
    report["num_dof"] = int(robot.num_dof)
    home = [0.0, 0.0, -1.5707963, 0.0, -1.5707963, 0.0]   # 실로봇 기본 자세와 같음
    robot.set_joint_positions(home)
    for _ in range(20):
        world.step(render=args.screenshot)
    report["home_joint_positions_rad"] = [round(float(v), 5) for v in robot.get_joint_positions()]
    if link6:
        m = UsdGeom.Xformable(stage.GetPrimAtPath(link6)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        report["link_6_world_pos_at_home_m"] = [round(v, 4) for v in m.ExtractTranslation()]

report["ok"] = bool(art_root and link6 and report.get("num_dof") == 6)

# ── 5. 스크린샷 (실제 렌더) ────────────────────────────────────────────────────
if args.screenshot and art_root:
    from isaacsim.sensors.camera import Camera
    from pxr import UsdLux
    import numpy as np
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(250)
    UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(1200)
    ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
    ground.CreatePointsAttr([(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateDisplayColorAttr([(0.25, 0.27, 0.3)])
    cam = Camera(prim_path="/World/Cam", resolution=(1280, 720),
                 position=np.array([2.4, -2.2, 1.5]))
    cam.initialize()
    # 카메라가 로봇(원점 위 0.6 m)을 보게 한다
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([2.4, -2.2, 1.5]), target=np.array([-0.2, 0.0, 0.5]),
                    camera_prim_path="/World/Cam")
    for _ in range(60):
        world.step(render=True)
    rgba = cam.get_rgba()
    if rgba is not None and rgba.size:
        from PIL import Image
        png = os.path.join(os.path.dirname(args.report), "m1013_home.png")
        Image.fromarray(rgba[..., :3].astype(np.uint8)).save(png)
        report["screenshot"] = os.path.relpath(png, ROOT)
    else:
        report["screenshot"] = None

with open(args.report, "w") as f:
    json.dump(report, f, indent=2, ensure_ascii=False)
print(json.dumps(report, indent=2, ensure_ascii=False))
app.close()
sys.exit(0 if report["ok"] else 1)
