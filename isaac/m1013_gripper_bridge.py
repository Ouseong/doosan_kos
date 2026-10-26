"""
M1013 + 그리퍼 통합 Isaac Sim 브릿지

- M1013 관절 제어: /dsr01/joint_states (sensor_msgs/JointState)
- 그리퍼 열기/닫기: /gripper_command   (control_msgs/GripperCommand)
- 그리퍼 상태 발행: /gripper_state     (sensor_msgs/JointState)

동작 방식:
  그리퍼는 Hall_Sensor/Code/Sim_Gripper.py 와 같은 CAD (usd/sim_gripper/*.stl,
  build_from_sim_gripper.py 로 생성). 물리 없는 시각 메시로 M1013 link_6 의 자식에
  붙인다. 플랜지 - 원기둥 스페이서(SPACER_*) - 그리퍼 순서로 고정되어 로봇과 한
  몸으로 움직인다 (예전처럼 매 프레임 따라가지 않으므로 지연·떨림이 없다).
  판 열림은 /gripper_command 를 받아 두 판의 X 이동만 바꾼다. 핀은 움직이지 않는다.

실행:
  docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/m1013_gripper_bridge.py
  docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/m1013_gripper_bridge.py --headless
"""

import argparse
import json
import os
import signal
import struct
import sys
import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

parser = argparse.ArgumentParser()
parser.add_argument("--topic",      default="dsr01/joint_states")
parser.add_argument("--real_topic", default="dsr01_real/joint_states")
parser.add_argument("--headless",   action="store_true")
parser.add_argument("--robot_usd",   default="/tmp/m1013_v2/m1013_full.usda")
parser.add_argument("--gripper_dir", default="/kos_workspace/usd/sim_gripper",
                    help="build_from_sim_gripper.py 가 만든 STL/meta.json 폴더")
args, unknown = parser.parse_known_args()

# ── 그리퍼 장착 (플랜지 → 원기둥 스페이서 → 그리퍼) ───────────────────────────
# link_6 원점 = 플랜지 면, +Z = 플랜지 바깥 방향 (예전 GUI 캘리브레이션과 같은 가정).
# 실물에서 플랜지와 그리퍼 장착면 사이가 3.5 cm (체결 부품 포함, 2026-10-26 실측) 라
# 그 자리를 원기둥으로 채운다.
SPACER_LEN_M = 0.035        # 플랜지 면 ~ 그리퍼 윗면 [m]
SPACER_DIA_M = 0.080        # 원기둥 지름 [m]. scripts/add_body_cylinder.py (그리퍼 윗면 원) 와 같다
# 그리퍼(Sim_Gripper 좌표: 윗면 Z=0, 손가락 -Z, 판은 ±X) → link_6.
# X 로 180° 뒤집어 손가락이 플랜지 바깥(+Z)을 향하게 하고, 판 여닫는 방향은 link_6 X 로
# 둔다. 실물이 플랜지 축으로 돌아가 붙어 있으면 MOUNT_YAW_DEG 만 바꾸면 된다.
MOUNT_YAW_DEG = 0.0

# /gripper_command (m, 0~0.067) 는 실 그리퍼 노드·콘솔과 같은 규약을 유지한다.
# 0.067 = 활짝 열림이고, 시뮬에서는 이를 판 하나 이동 0~plate_travel_mm 로 비례 변환한다.
CMD_MAX_OPENING_M = 0.067

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  HAMMERING SCENE CONFIG  (Step 0-2)                                     ║
# ║  Toggle HAMMERING_SCENE = False to run the plain bridge.                ║
# ╚══════════════════════════════════════════════════════════════════════════╝
HAMMERING_SCENE = True

# ── Nail / Workbench ─────────────────────────────────────────────────────────
# NAIL_POS: (x, y, z) meters in world frame.
# z = top surface of workbench = base of nail.  Edit this one variable to relocate.
# (x, y) 모두 음수 사분면 + bench bottom 이 바닥 (z=0) 에 닿도록 z = BENCH_THICK_M.
NAIL_POS        = (-0.65, -0.45, 0.05)
NAIL_DIAMETER_M = 0.003    # 3 mm
NAIL_HEIGHT_M   = 0.050    # 5 cm
BENCH_LENGTH_M  = 0.60
BENCH_WIDTH_M   = 0.40
BENCH_THICK_M   = 0.05

# ── Hammer ───────────────────────────────────────────────────────────────────
HAMMER_STL_PATH = "/kos_workspace/usd/hammer/hammer.stl"
HAMMER_SCALE    = 0.001    # STL unit: mm → m

# Two placement modes:
#   False → hammer rests on floor next to the robot (Step 1 시각 확인 단계)
#   True  → hammer rigidly follows the gripper (옵션 B / 망치질 단계)
HAMMER_FOLLOW_GRIPPER = False

# Rest pose (used when HAMMER_FOLLOW_GRIPPER = False).
# Strike face = local Z = -0.027 m → world Z = 0 (체크 바닥 표면).
# x = 작업대(0.5) 와 동축, y = -0.45 (작업대 좌측 빈 공간), 손잡이는 +Y(로봇 쪽).
HAMMER_REST_POS_M = np.array([-0.40, -0.45, 0.077])
HAMMER_REST_RPY   = np.array([0.0, 0.0, 0.0])

# 그리퍼 CAD 정보 (build_from_sim_gripper.py 출력)
with open(os.path.join(args.gripper_dir, "meta.json")) as _f:
    GRIPPER_META = json.load(_f)

# 잡는 점 = 두 판 핀 격자의 가운데, 그리퍼(Sim_Gripper) 좌표 [m].
# 망치 STL 원점을 여기에 둔다 (follow 모드) / auto-grasp 거리도 여기서 잰다.
HAMMER_GRIP_IN_GRIPPER = np.array(GRIPPER_META["grasp_center_mm"]) * 1e-3

# Hammer orientation relative to the gripper frame (RPY rad).
# 그리퍼 좌표계가 Sim_Gripper 기준으로 바뀌었으므로 follow 모드를 쓸 때 다시 맞출 것.
HAMMER_RPY_IN_GRIPPER  = np.array([0.0, 0.0, 0.0])

# Strike-point offset from STL origin in local frame (m).
# Computed from STL Z_min face centroid: circle r=15 mm, centre (0, -87, -27) mm.
STRIKE_POINT_LOCAL_M   = np.array([0.0, -0.087, -0.027])

# Auto-grasp 임계값 (HAMMER_FOLLOW_GRIPPER=False 모드 전용).
# 그리퍼 close 명령(opening<ATTACH) 시 plates ↔ 망치 grip 거리<DIST 면 부착.
# open 명령(opening>DETACH) 시 떼서 마지막 자세에 정지.  collision 없는
# kinematic 결합이라 풀물리 grasp 가 아닌 시각/trajectory 검증용 단축 경로.
HAMMER_ATTACH_OPENING_M = 0.040
HAMMER_DETACH_OPENING_M = 0.050
HAMMER_ATTACH_DIST_M    = 0.060

# ── Isaac Sim 앱 부팅 ──────────────────────────────────────────────────────
from isaacsim import SimulationApp

app = SimulationApp(
    {"renderer": "RayTracedLighting", "headless": args.headless,
     "width": 1280, "height": 720},
    experience="/isaac-sim/apps/isaacsim.exp.base.kit",
)

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics, UsdGeom, UsdLux, UsdShade, Gf, Usd, Sdf
import omni.isaac.core.utils.rotations as rot_utils

enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    app.update()

BUNDLED_RCLPY = "/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy"
if BUNDLED_RCLPY not in sys.path:
    sys.path.insert(0, BUNDLED_RCLPY)

# ── World + 씬 구성 ────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

ROBOT_PRIM = "/World/m1013"

add_reference_to_stage(usd_path=args.robot_usd, prim_path=ROBOT_PRIM)
print(f"[bridge] M1013 로드:  {args.robot_usd}")

for _ in range(10):
    app.update()

# ── tool0 경로 자동 탐색 ───────────────────────────────────────────────────
# URDF 임포트 결과에 따라 link6 / link_6 / tool0 등 명명이 다를 수 있어 자동 탐색
tool0_path = None
for _p in stage.Traverse():
    _pp = _p.GetPath().pathString
    if not _pp.startswith(ROBOT_PRIM):
        continue
    _name = _p.GetName().lower()
    if _name in ("link6", "link_6", "tool0", "flange", "tcp", "wrist3"):
        tool0_path = _pp
        break
if tool0_path is None:
    _link_prims = [p for p in stage.Traverse()
                   if p.GetPath().pathString.startswith(ROBOT_PRIM)
                   and p.GetName().startswith("link")]
    if _link_prims:
        tool0_path = max(_link_prims, key=lambda p: p.GetName()).GetPath().pathString
if tool0_path is None:
    raise SystemExit("[bridge] link_6 (그리퍼 장착 기준) 을 못 찾았다. --robot_usd 확인")
print(f"[bridge] 그리퍼 장착 기준: {tool0_path}")

# ── 체크 무늬 바닥 ─────────────────────────────────────────────────────────
def _setup_checker_floor(stage,
                         size_m=50.0, tile_m=0.5,
                         color_a=(20, 25, 45),
                         color_b=(70, 95, 175)):
    checker_png = "/tmp/m1013_checker_floor.png"
    if not os.path.exists(checker_png):
        from PIL import Image
        img_size = 256
        half = img_size // 2
        img = Image.new("RGB", (img_size, img_size), color_a)
        pix = img.load()
        for y in range(img_size):
            for x in range(img_size):
                if ((x // half) + (y // half)) % 2 == 0:
                    pix[x, y] = color_b
        img.save(checker_png)
        print(f"[bridge] 체크 바닥 텍스처 생성: {checker_png}")

    floor = "/World/CheckerFloor"
    half = size_m / 2.0
    mesh = UsdGeom.Mesh.Define(stage, floor)
    mesh.CreatePointsAttr([
        Gf.Vec3f(-half, -half, 0), Gf.Vec3f(half, -half, 0),
        Gf.Vec3f(half, half, 0),   Gf.Vec3f(-half, half, 0),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([(-half, -half, 0), (half, half, 0)])

    repeats = size_m / (2.0 * tile_m)
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
    uv.Set([Gf.Vec2f(0, 0), Gf.Vec2f(repeats, 0),
            Gf.Vec2f(repeats, repeats), Gf.Vec2f(0, repeats)])

    mat = UsdShade.Material.Define(stage, floor + "/Mat")
    surf = UsdShade.Shader.Define(stage, floor + "/Mat/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    reader = UsdShade.Shader.Define(stage, floor + "/Mat/UVReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader_out = reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(stage, floor + "/Mat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(checker_png)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader_out)
    tex_rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    surf.CreateInput("diffuseColor",
                     Sdf.ValueTypeNames.Color3f).ConnectToSource(tex_rgb)
    mat.CreateSurfaceOutput().ConnectToSource(
        surf.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)

    # Static collision (no RigidBodyAPI = static collider).  Triangle mesh OK.
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")
    print(f"[bridge] 체크 바닥: {size_m}m × {size_m}m, tile={tile_m*100:.0f}cm  (+ collision)")


# ── Step 2: Nail visual marker + workbench ────────────────────────────────────
def _setup_nail_workbench(stage):
    """Workbench slab + nail cylinder.  Nail has no collision — visual only."""
    nx, ny, nz = NAIL_POS

    # Workbench
    bench_path = "/World/Scene/Workbench"
    stage.DefinePrim("/World/Scene", "Xform")
    cube = UsdGeom.Cube.Define(stage, bench_path)
    cube.CreateSizeAttr(1.0)
    bxf = UsdGeom.Xformable(cube)
    bxf.ClearXformOpOrder()
    bxf.AddTranslateOp().Set(Gf.Vec3d(nx, ny, nz - BENCH_THICK_M / 2))
    bxf.AddScaleOp().Set(Gf.Vec3f(BENCH_LENGTH_M, BENCH_WIDTH_M, BENCH_THICK_M))
    b_mat = UsdShade.Material.Define(stage, bench_path + "/Mat")
    b_surf = UsdShade.Shader.Define(stage, bench_path + "/Mat/Surf")
    b_surf.CreateIdAttr("UsdPreviewSurface")
    b_surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55, 0.35, 0.15))
    b_surf.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(0.80)
    b_surf.CreateInput("metallic",     Sdf.ValueTypeNames.Float).Set(0.00)
    b_mat.CreateSurfaceOutput().ConnectToSource(
        b_surf.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI(cube).Bind(b_mat)

    # Static collision (Cube primitive — no approximation needed)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # Nail cylinder — no PhysicsCollisionAPI → visual only
    nail_path = "/World/Scene/Nail"
    cyl = UsdGeom.Cylinder.Define(stage, nail_path)
    cyl.CreateRadiusAttr(NAIL_DIAMETER_M / 2)
    cyl.CreateHeightAttr(NAIL_HEIGHT_M)
    nxf = UsdGeom.Xformable(cyl)
    nxf.ClearXformOpOrder()
    # USD Cylinder origin = geometric centre → shift up by half height
    nxf.AddTranslateOp().Set(Gf.Vec3d(nx, ny, nz + NAIL_HEIGHT_M / 2))
    n_mat = UsdShade.Material.Define(stage, nail_path + "/Mat")
    n_surf = UsdShade.Shader.Define(stage, nail_path + "/Mat/Surf")
    n_surf.CreateIdAttr("UsdPreviewSurface")
    n_surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.70, 0.72, 0.78))
    n_surf.CreateInput("metallic",     Sdf.ValueTypeNames.Float).Set(0.85)
    n_surf.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(0.25)
    n_mat.CreateSurfaceOutput().ConnectToSource(
        n_surf.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI(cyl).Bind(n_mat)

    print(f"[scene] Workbench : centre=({nx:.3f}, {ny:.3f}, {nz - BENCH_THICK_M/2:.3f}) m  "
          f"size=({BENCH_LENGTH_M:.2f}×{BENCH_WIDTH_M:.2f}×{BENCH_THICK_M:.2f}) m")
    print(f"[scene] Nail      : base=({nx:.3f}, {ny:.3f}, {nz:.3f}) m  "
          f"d={NAIL_DIAMETER_M*1000:.0f}mm  h={NAIL_HEIGHT_M*100:.0f}cm  (no collision)")


# ── Step 3: STL mesh loader (망치 + 그리퍼 공용) ───────────────────────────────
def _read_stl(stl_path):
    """binary STL → (N*3, 3) float32 꼭짓점. numpy 로 한 번에 읽는다
    (예전 struct 루프는 그리퍼 20만 삼각형에서 수십 초 걸린다)."""
    with open(stl_path, "rb") as f:
        f.read(80)
        n_tri = struct.unpack("<I", f.read(4))[0]
        rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        data = np.frombuffer(f.read(n_tri * rec.itemsize), dtype=rec, count=n_tri)
    return np.ascontiguousarray(data["v"].reshape(-1, 3))


def _make_material(stage, path, color, metallic=0.0, roughness=0.6):
    mat = UsdShade.Material.Define(stage, path)
    surf = UsdShade.Shader.Define(stage, path + "/Surf")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(c) for c in color]))
    surf.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(float(roughness))
    surf.CreateInput("metallic",     Sdf.ValueTypeNames.Float).Set(float(metallic))
    mat.CreateSurfaceOutput().ConnectToSource(
        surf.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def _stl_mesh(stage, stl_path, mesh_path, scale, material):
    """STL 하나를 USD Mesh 로. scale 은 mesh 자신에 넣어 부모 Xform 은 자세만 갖게 한다."""
    from pxr import Vt
    pts = _read_stl(stl_path)
    n_tri = len(pts) // 3
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(n_tri, 3, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(pts), dtype=np.int32)))
    mesh.CreateSubdivisionSchemeAttr("none")
    lo, hi = pts.min(0), pts.max(0)
    mesh.CreateExtentAttr([Gf.Vec3f(*lo.tolist()), Gf.Vec3f(*hi.tolist())])
    mxf = UsdGeom.Xformable(mesh)
    mxf.ClearXformOpOrder()
    mxf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    UsdShade.MaterialBindingAPI(mesh).Bind(material)
    return n_tri


def _load_stl_as_mesh(stage, stl_path, prim_path, scale=0.001):
    """망치: Xform + Mesh(dark rubber)."""
    stage.DefinePrim(prim_path, "Xform")
    mat = _make_material(stage, prim_path + "/Mat", (0.12, 0.12, 0.12), 0.0, 0.75)
    n_tri = _stl_mesh(stage, stl_path, prim_path + "/Mesh", scale, mat)
    print(f"[scene] Hammer STL: {n_tri} triangles loaded → {prim_path}  (scale={scale})")


# ── Step 4: 핀어레이 그리퍼 (Sim_Gripper CAD) 를 link_6 에 고정 ──────────────────
def _quat_xyzw_to_gf(q_wxyz):
    return Gf.Quatf(float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]))


def _setup_sim_gripper(stage, parent_path):
    """parent(link_6) 아래에 스페이서 원기둥 + 그리퍼를 만든다. 물리 없음(시각 전용).

    link_6 의 자식이라 렌더러가 로봇 자세를 그대로 상속한다. 예전처럼 매 프레임
    set_world_pose 로 따라 붙이지 않으므로 한 프레임 늦는 일도, 가벼운 떨림도 없다.
    반환: (+X 판 translate op, -X 판 translate op)
    """
    root = parent_path + "/PinGripper"
    stage.DefinePrim(root, "Xform")
    looks = root + "/Looks"
    stage.DefinePrim(looks, "Scope")

    # 스페이서 원기둥: 플랜지 면(z=0) ~ z=SPACER_LEN_M
    spacer = UsdGeom.Cylinder.Define(stage, root + "/Spacer")
    spacer.CreateAxisAttr("Z")
    spacer.CreateRadiusAttr(SPACER_DIA_M / 2.0)
    spacer.CreateHeightAttr(SPACER_LEN_M)
    r = SPACER_DIA_M / 2.0
    spacer.CreateExtentAttr([Gf.Vec3f(-r, -r, -SPACER_LEN_M / 2), Gf.Vec3f(r, r, SPACER_LEN_M / 2)])
    sxf = UsdGeom.Xformable(spacer)
    sxf.ClearXformOpOrder()
    sxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, SPACER_LEN_M / 2.0))
    UsdShade.MaterialBindingAPI(spacer).Bind(
        _make_material(stage, looks + "/Spacer", (0.55, 0.56, 0.58), 0.8, 0.35))

    # 그리퍼 본체 프레임: 스페이서 끝에 Sim_Gripper 좌표 원점(장착면)
    body = root + "/Body"
    bxf = UsdGeom.Xformable(stage.DefinePrim(body, "Xform"))
    bxf.ClearXformOpOrder()
    bxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, SPACER_LEN_M))
    bxf.AddOrientOp().Set(_quat_xyzw_to_gf(mount_quat))

    colors = GRIPPER_META["colors"]
    spec = {"body": (0.0, 0.55), "house": (0.0, 0.55), "pcb": (0.0, 0.45),
            "pins": (0.85, 0.30), "mag": (0.10, 0.35)}
    mats = {k: _make_material(stage, f"{looks}/{k}", colors[k], *spec[k]) for k in spec}

    d = args.gripper_dir
    n_tri = _stl_mesh(stage, os.path.join(d, "body.stl"), body + "/BodyMesh", 1e-3, mats["body"])
    ops = []
    for side in ("p", "n"):
        plate = f"{body}/Plate_{side}"
        pxf = UsdGeom.Xformable(stage.DefinePrim(plate, "Xform"))
        pxf.ClearXformOpOrder()
        ops.append(pxf.AddTranslateOp())
        for part in ("house", "pcb", "pins", "mag"):
            n_tri += _stl_mesh(stage, os.path.join(d, f"plate_{side}_{part}.stl"),
                               f"{plate}/{part}", 1e-3, mats[part])
    print(f"[bridge] 핀어레이 그리퍼: {n_tri:,} 삼각형 → {root}  "
          f"(스페이서 Ø{SPACER_DIA_M*1000:.0f} x {SPACER_LEN_M*1000:.0f} mm, yaw {MOUNT_YAW_DEG:g}°)")
    return ops[0], ops[1]


_setup_checker_floor(stage)

if HAMMERING_SCENE:
    _setup_nail_workbench(stage)
    _load_stl_as_mesh(stage, HAMMER_STL_PATH, "/World/HammerMesh", HAMMER_SCALE)

    # Hammer physics — RigidBody on root + dynamic mesh collision.
    # Start kinematic so it sits at rest pose; toggled to dynamic on detach.
    _h_root = stage.GetPrimAtPath("/World/HammerMesh")
    _h_rb   = UsdPhysics.RigidBodyAPI.Apply(_h_root)
    _h_rb.CreateKinematicEnabledAttr().Set(True)
    _h_mass = UsdPhysics.MassAPI.Apply(_h_root)
    _h_mass.CreateMassAttr(0.5)                                  # 500 g
    _h_mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, -0.014, 0.0))   # bbox 중앙
    _h_mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.001457, 0.000168, 0.001381))
    _h_mesh = stage.GetPrimAtPath("/World/HammerMesh/Mesh")
    UsdPhysics.CollisionAPI.Apply(_h_mesh)
    UsdPhysics.MeshCollisionAPI.Apply(_h_mesh).CreateApproximationAttr().Set("convexHull")
    print(f"[bridge] Hammer physics: RigidBody + 500g + convexHull collision  (kinematic)")

# ── 조명 ──────────────────────────────────────────────────────────────────
UsdLux.DistantLight.Define(stage, "/World/DistantLight").CreateIntensityAttr(3000)
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(500)

def rpy_to_quat_wxyz(rpy):
    """roll-pitch-yaw → quaternion [w, x, y, z]"""
    r, p, y = rpy
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y_ = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([w, x, y_, z])

def quat_multiply(q1, q2):
    """두 quaternion 곱 [w,x,y,z]"""
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def rotate_vec(q_wxyz, v):
    """quaternion으로 벡터 회전"""
    w,x,y,z = q_wxyz
    qv = np.array([0, *v])
    q_inv = np.array([w, -x, -y, -z])
    res = quat_multiply(quat_multiply(q_wxyz, qv), q_inv)
    return res[1:]

# 그리퍼 프레임(Sim_Gripper) 의 link_6 기준 자세: 먼저 X 180° (손가락 → +Z), 그다음
# 플랜지 축(Z) 으로 MOUNT_YAW_DEG.  MOUNT_XYZ = 스페이서 끝.
mount_quat = quat_multiply(rpy_to_quat_wxyz([0.0, 0.0, np.deg2rad(MOUNT_YAW_DEG)]),
                           rpy_to_quat_wxyz([np.pi, 0.0, 0.0]))
MOUNT_XYZ = np.array([0.0, 0.0, SPACER_LEN_M])

plate_op_p, plate_op_n = _setup_sim_gripper(stage, tool0_path)

# ── Articulation (로봇만. 그리퍼는 물리 없는 시각 메시) ────────────────────────
robot_art_path = None
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI) and \
            str(prim.GetPath()).startswith(ROBOT_PRIM):
        robot_art_path = str(prim.GetPath())
        break
print(f"[bridge] M1013 ArticulationRoot:  {robot_art_path}")

from isaacsim.core.prims import SingleArticulation

robot = SingleArticulation(prim_path=robot_art_path, name="m1013")
world.scene.add(robot)
world.reset()
print(f"[bridge] M1013  관절: {robot.num_dof}개 → {robot.dof_names}")

if HAMMERING_SCENE:
    _hammer_orient_local = rpy_to_quat_wxyz(HAMMER_RPY_IN_GRIPPER)
    _h_prim = stage.GetPrimAtPath("/World/HammerMesh")
    _h_xf = UsdGeom.Xformable(_h_prim)
    _h_xf.ClearXformOpOrder()
    _hammer_t_op = _h_xf.AddTranslateOp()
    _hammer_r_op = _h_xf.AddOrientOp()
    _h_kin_attr = UsdPhysics.RigidBodyAPI(_h_prim).GetKinematicEnabledAttr()

    # Auto-grasp 상태 머신 (HAMMER_FOLLOW_GRIPPER=False 일 때만 동작)
    _hammer_attached         = False
    _hammer_world_pos        = HAMMER_REST_POS_M.copy()
    _hammer_world_quat       = rpy_to_quat_wxyz(HAMMER_REST_RPY)
    _hammer_grip_offset_pos  = np.zeros(3)
    _hammer_grip_offset_quat = np.array([1.0, 0.0, 0.0, 0.0])
    _attach_diag_n           = 0

    if not HAMMER_FOLLOW_GRIPPER:
        _hammer_t_op.Set(Gf.Vec3d(*[float(v) for v in _hammer_world_pos]))
        _hammer_r_op.Set(Gf.Quatf(float(_hammer_world_quat[0]),
                                   float(_hammer_world_quat[1]),
                                   float(_hammer_world_quat[2]),
                                   float(_hammer_world_quat[3])))
        _strike_world = HAMMER_REST_POS_M + STRIKE_POINT_LOCAL_M  # identity rotation
        print(f"[scene] Hammer rest pose  pos={HAMMER_REST_POS_M.tolist()} m  "
              f"strike_world={_strike_world.tolist()} m")
        print(f"[scene] Auto-grasp 활성: opening<{HAMMER_ATTACH_OPENING_M*1000:.0f}mm "
              f"+ dist<{HAMMER_ATTACH_DIST_M*1000:.0f}mm 시 부착, "
              f"open>{HAMMER_DETACH_OPENING_M*1000:.0f}mm 시 분리")
    else:
        print(f"[scene] Hammer force-follow-gripper mode  "
              f"grip_offset={HAMMER_GRIP_IN_GRIPPER}  "
              f"strike_local={STRIKE_POINT_LOCAL_M} m")

def get_tool0_world_pose():
    """USD stage에서 tool0 world transform 읽기"""
    prim = stage.GetPrimAtPath(tool0_path)
    if not prim.IsValid():
        return np.zeros(3), np.array([1,0,0,0])
    xf = UsdGeom.Xformable(prim)
    mat = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # 위치
    t = mat.ExtractTranslation()
    pos = np.array([t[0], t[1], t[2]])
    # 회전 → quaternion
    rot = mat.ExtractRotationQuat()
    iv = rot.GetImaginary()
    quat = np.array([rot.GetReal(), iv[0], iv[1], iv[2]])  # wxyz
    return pos, quat


def get_prim_world_pose(prim_path):
    """USD stage 의 prim world pose (kinematic / dynamic 무관 live read)"""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    xf = UsdGeom.Xformable(prim)
    mat = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    pos = np.array([t[0], t[1], t[2]])
    rot = mat.ExtractRotationQuat()
    iv = rot.GetImaginary()
    quat = np.array([rot.GetReal(), iv[0], iv[1], iv[2]])
    return pos, quat

# ── 그리퍼 판 설정 ─────────────────────────────────────────────────────────
MAX_OPENING = CMD_MAX_OPENING_M                          # 명령 규약 (m)
PLATE_TRAVEL_M = GRIPPER_META["plate_travel_mm"] * 1e-3  # 판 하나 최대 이동
PLATE_OFFSET_M = GRIPPER_META["plate_offset_mm"] * 1e-3  # 닫힘에서도 벌어진 양

# Match the real Dynamixel gripper's traversal speed so the sim looks
# the same as the hardware. Settled on by visual comparison against the
# real motor (XM430, profile_velocity=100): direct timing of the motor
# itself gives ~0.046 m/s, but a chunk of that is end-of-motion settle
# the operator can't see, so 0.070 m/s lines up better with what the
# eye perceives. If you change the motor's profile_velocity, retune.
GRIPPER_OPENING_SPEED = 0.075   # m/s, |dopening/dt| cap

def opening_to_plate_shift(opening_m):
    """명령 열림(0~0.067 m) → 판 하나의 X 이동 [m] (Sim_Gripper._apply 와 같은 규칙)."""
    ratio = float(np.clip(opening_m, 0.0, MAX_OPENING)) / MAX_OPENING
    return ratio * PLATE_TRAVEL_M + PLATE_OFFSET_M


_last_shift = None


def set_plates(opening_m):
    """두 판을 민다. 값이 바뀔 때만 USD 를 건드린다 (정지 중엔 비용 0)."""
    global _last_shift
    s = opening_to_plate_shift(opening_m)
    if _last_shift is not None and abs(s - _last_shift) < 1e-6:
        return
    _last_shift = s
    # 판 Xform 은 Sim_Gripper 좌표(mm 메시를 1e-3 로 줄인 프레임) 안에 있다 → m 단위로 민다
    plate_op_p.Set(Gf.Vec3d(+s, 0.0, 0.0))
    plate_op_n.Set(Gf.Vec3d(-s, 0.0, 0.0))

# ── ROS2 ──────────────────────────────────────────────────────────────────
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Empty   # GripperCommand가 Isaac Sim Python 3.11 번들에 없어 대체

rclpy.init()

class BridgeNode(Node):
    def __init__(self):
        super().__init__("m1013_gripper_bridge")
        qos = QoSProfile(depth=10)
        # 관절 상태는 최신 한 개만 의미가 있다. depth 10 이면 밀린 옛 자세가 줄줄이
        # 그려져 화면이 실제보다 늦게 따라온다.
        qos_latest = QoSProfile(depth=1)

        self.robot_joints   = None
        self.target_opening = MAX_OPENING   # 시작 시 완전 열림
        # Actual opening tracked by the sim — ramps towards target_opening
        # at GRIPPER_OPENING_SPEED so the visual matches the real motor
        # speed instead of jumping instantly.
        self.current_opening = MAX_OPENING
        self._last_grip_pos = None           # IK 디버그용 (loop 에서 매 frame 갱신)
        self.hammer_reset_pending = False    # /hammer_reset 트리거

        # Dual-namespace mirror: subscribe both sim (/dsr01) and real (/dsr01_real)
        # joint_states. The viewport follows whichever side moved most recently
        # — so when the operator sends to sim it shows sim, and when the real
        # robot is jogged it shows the real robot.
        self._sources = {
            "sim":  {"joints": None, "last_change_t": 0.0},
            "real": {"joints": None, "last_change_t": 0.0},
        }
        self._last_active_src = None

        self.create_subscription(
            JointState, f"/{args.topic}",
            lambda m: self._joint_cb(m, "sim"), qos_latest)
        self.create_subscription(
            JointState, f"/{args.real_topic}",
            lambda m: self._joint_cb(m, "real"), qos_latest)
        self.create_subscription(
            Float32, "/gripper_command", self._gripper_cb, qos)
        self.create_subscription(
            Empty, "/hammer_reset", self._hammer_reset_cb, qos)
        self.state_pub = self.create_publisher(
            JointState, "/gripper_state", qos)

        self.get_logger().info(
            f"브릿지 시작. sim=/{args.topic}, real=/{args.real_topic}, dual-mirror.")

    def _joint_cb(self, msg, source):
        import time as _time
        if len(msg.position) == 0:
            return
        new_pos = np.array(msg.position)
        src = self._sources[source]
        # 자세 변화 감지 (1 mrad 이상이면 "방금 움직였다"로 판단)
        if src["joints"] is None or not np.allclose(new_pos, src["joints"], atol=1e-3):
            src["last_change_t"] = _time.time()
        src["joints"] = new_pos

        # 마지막에 움직인 source를 viewport에 반영
        sim_t  = self._sources["sim"]["last_change_t"]
        real_t = self._sources["real"]["last_change_t"]
        if real_t > sim_t and self._sources["real"]["joints"] is not None:
            active = "real"
        elif self._sources["sim"]["joints"] is not None:
            active = "sim"
        else:
            active = source
        self.robot_joints = self._sources[active]["joints"]
        if self._last_active_src != active:
            self.get_logger().info(f"[mirror] active source: {active}")
            self._last_active_src = active

    def _gripper_cb(self, msg):
        # std_msgs/Float32: data = 열림 너비 (m, 0.0~0.067)
        prev = self.target_opening
        self.target_opening = msg.data
        # 변화 큰 명령(>5mm)일 때만 로그 — IK 디버그용
        if abs(msg.data - prev) > 0.005 and self._last_grip_pos is not None:
            self.get_logger().info(
                f"그리퍼 명령: {msg.data*1000:.1f}mm  "
                f"@ gripper_pos={self._last_grip_pos.round(3).tolist()}")
        else:
            self.get_logger().info(f"그리퍼 명령: {msg.data*1000:.1f}mm")

    def _hammer_reset_cb(self, _msg):
        self.hammer_reset_pending = True
        self.get_logger().info("[hammer] reset 요청 수신 — 다음 frame 에서 rest pose 로 복구")

    def publish_gripper_state(self, opening_m):
        # 예전 articulation 관절값과 같은 규약: 판 하나가 닫힌 쪽으로 간 양 (0=활짝 열림)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['gripper_left_joint', 'gripper_right_joint']
        q = (MAX_OPENING - float(np.clip(opening_m, 0.0, MAX_OPENING))) / 2.0
        js.position = [q, q]
        self.state_pub.publish(js)


ros_node = BridgeNode()
executor = SingleThreadedExecutor()
executor.add_node(ros_node)


def spin_pending(max_callbacks=50):
    """쌓인 콜백을 한 프레임에 다 처리한다. spin_once 한 번은 콜백 하나뿐이라
    관절 상태(100 Hz+) 두 줄기를 60 Hz 루프로 받으면 점점 밀린다."""
    for _ in range(max_callbacks):
        executor.spin_once(timeout_sec=0)


# docker stop / kill 의 SIGTERM 도 Ctrl-C 처럼 정상 종료 경로를 타게 한다
# (예전엔 SIGTERM 이면 rclpy/앱 정리 없이 죽었다).
_stop = {"flag": False}
signal.signal(signal.SIGTERM, lambda *_: _stop.update(flag=True))

# ── 초기 포즈 ─────────────────────────────────────────────────────────────
HOME_POSE = np.deg2rad([0.0, 0.0, -90.0, 0.0, -90.0, 0.0])
robot.set_joint_positions(HOME_POSE)
set_plates(ros_node.current_opening)
for _ in range(10):
    world.step(render=True)
print(f"[bridge] M1013 초기 포즈 적용")

world.play()
print("[bridge] 시뮬레이션 루프 시작. Ctrl-C로 종료.")

# 60 Hz wall-clock throttle: physics_dt = 1/60 이라 step 호출 빈도와 sim 시간이 1:1 이 되어야
# 중력이 9.81 m/s² 그대로 보임.  throttle 없으면 GPU 가 빠른 만큼 sim 이 가속됨.
import time as _time
LOOP_DT = 1.0 / 60.0
_next_step_t = _time.time()

try:
    while app.is_running() and not _stop["flag"]:
        spin_pending()

        # ── M1013 관절 제어 ────────────────────────────────────────────
        if ros_node.robot_joints is not None:
            joints = ros_node.robot_joints
            if len(joints) == robot.num_dof and not np.allclose(joints, 0.0):
                robot.set_joint_positions(joints)

        # ── 그리퍼 자세 (계산만. 그리퍼 메시는 link_6 자식이라 알아서 따라간다) ──
        # 망치 auto-grasp / follow 에서만 쓴다.
        if HAMMERING_SCENE:
            tool0_pos, tool0_quat = get_tool0_world_pose()
            gripper_quat = quat_multiply(tool0_quat, mount_quat)
            gripper_pos  = tool0_pos + rotate_vec(tool0_quat, MOUNT_XYZ)
            ros_node._last_grip_pos = gripper_pos

        # ── 망치 reset 처리: 외부에서 /hammer_reset 받으면 rest pose 복구 ──
        if HAMMERING_SCENE and ros_node.hammer_reset_pending:
            ros_node.hammer_reset_pending = False
            _hammer_attached   = False
            _hammer_world_pos  = HAMMER_REST_POS_M.copy()
            _hammer_world_quat = rpy_to_quat_wxyz(HAMMER_REST_RPY)
            if not _h_kin_attr.Get():
                _h_kin_attr.Set(True)
            _hammer_t_op.Set(Gf.Vec3d(float(_hammer_world_pos[0]),
                                       float(_hammer_world_pos[1]),
                                       float(_hammer_world_pos[2])))
            _hammer_r_op.Set(Gf.Quatf(float(_hammer_world_quat[0]),
                                       float(_hammer_world_quat[1]),
                                       float(_hammer_world_quat[2]),
                                       float(_hammer_world_quat[3])))
            ros_node.get_logger().info(
                f"[hammer] reset → {HAMMER_REST_POS_M.tolist()}  (kinematic)")

        # ── 망치: 강제 follow (옵션 B) 또는 auto-grasp 상태머신 ─────────
        if HAMMERING_SCENE:
            if HAMMER_FOLLOW_GRIPPER:
                _hammer_world_pos  = gripper_pos + rotate_vec(gripper_quat, HAMMER_GRIP_IN_GRIPPER)
                _hammer_world_quat = quat_multiply(gripper_quat, _hammer_orient_local)
                _hammer_t_op.Set(Gf.Vec3d(float(_hammer_world_pos[0]),
                                           float(_hammer_world_pos[1]),
                                           float(_hammer_world_pos[2])))
                _hammer_r_op.Set(Gf.Quatf(float(_hammer_world_quat[0]),
                                           float(_hammer_world_quat[1]),
                                           float(_hammer_world_quat[2]),
                                           float(_hammer_world_quat[3])))
            else:
                _t_open = ros_node.target_opening
                if not _hammer_attached:
                    # Live-read actual pose (dynamic 단계에서 PhysX 가 움직였을 수 있음)
                    _hammer_world_pos, _hammer_world_quat = get_prim_world_pose("/World/HammerMesh")

                    _grasp_world = gripper_pos + rotate_vec(
                        gripper_quat, HAMMER_GRIP_IN_GRIPPER)
                    _dist = float(np.linalg.norm(_grasp_world - _hammer_world_pos))
                    if _t_open < HAMMER_ATTACH_OPENING_M and _dist < HAMMER_ATTACH_DIST_M:
                        # 현재 actual world pose 기준으로 offset 계산
                        _qw, _qx, _qy, _qz = gripper_quat
                        _inv_q = np.array([_qw, -_qx, -_qy, -_qz])
                        _hammer_grip_offset_pos  = rotate_vec(
                            _inv_q, _hammer_world_pos - gripper_pos)
                        _hammer_grip_offset_quat = quat_multiply(_inv_q, _hammer_world_quat)
                        _hammer_attached = True
                        # dynamic → kinematic 전환. xform op 도 actual pose 로 동기화하여
                        # 첫 프레임 jump 회피.
                        if not _h_kin_attr.Get():
                            _h_kin_attr.Set(True)
                        _hammer_t_op.Set(Gf.Vec3d(float(_hammer_world_pos[0]),
                                                   float(_hammer_world_pos[1]),
                                                   float(_hammer_world_pos[2])))
                        _hammer_r_op.Set(Gf.Quatf(float(_hammer_world_quat[0]),
                                                   float(_hammer_world_quat[1]),
                                                   float(_hammer_world_quat[2]),
                                                   float(_hammer_world_quat[3])))
                        _attach_diag_n = 5
                        _joint_log = (np.rad2deg(ros_node.robot_joints).round(1).tolist()
                                      if ros_node.robot_joints is not None else None)
                        ros_node.get_logger().info(
                            f"[hammer] attach  hammer_actual={_hammer_world_pos.round(4).tolist()}  "
                            f"gripper_pos={gripper_pos.round(4).tolist()}  "
                            f"offset={_hammer_grip_offset_pos.round(4).tolist()}  "
                            f"joints_deg={_joint_log}  dist={_dist*1000:.0f}mm")
                else:
                    _hammer_world_pos  = gripper_pos + rotate_vec(
                        gripper_quat, _hammer_grip_offset_pos)
                    _hammer_world_quat = quat_multiply(gripper_quat, _hammer_grip_offset_quat)
                    _hammer_t_op.Set(Gf.Vec3d(float(_hammer_world_pos[0]),
                                               float(_hammer_world_pos[1]),
                                               float(_hammer_world_pos[2])))
                    _hammer_r_op.Set(Gf.Quatf(float(_hammer_world_quat[0]),
                                               float(_hammer_world_quat[1]),
                                               float(_hammer_world_quat[2]),
                                               float(_hammer_world_quat[3])))
                    if _attach_diag_n > 0:
                        _actual_pos, _ = get_prim_world_pose("/World/HammerMesh")
                        _drift = np.linalg.norm(_actual_pos - _hammer_world_pos)
                        ros_node.get_logger().info(
                            f"[hammer-diag] f={_attach_diag_n}  "
                            f"cmd={_hammer_world_pos.round(4).tolist()}  "
                            f"actual={_actual_pos.round(4).tolist()}  "
                            f"drift={_drift*1000:.1f}mm")
                        _attach_diag_n -= 1
                    if _t_open > HAMMER_DETACH_OPENING_M:
                        _hammer_attached = False
                        _h_kin_attr.Set(False)   # 중력 ON, 자유 낙하
                        ros_node.get_logger().info(
                            f"[hammer] detached → dynamic  target={_t_open*1000:.0f}mm")

        # ── 그리퍼 판 ─────────────────────────────────────────────────
        # Velocity-limited ramp toward target_opening so the sim animation
        # matches the real motor's traversal time. Without this the plates
        # snap to target instantly, which doesn't look like the hardware.
        delta = ros_node.target_opening - ros_node.current_opening
        step = GRIPPER_OPENING_SPEED * LOOP_DT
        if abs(delta) <= step:
            ros_node.current_opening = ros_node.target_opening
        else:
            ros_node.current_opening += step if delta > 0 else -step
        set_plates(ros_node.current_opening)
        ros_node.publish_gripper_state(ros_node.current_opening)

        world.step(render=True)

        # Wall-clock throttle to physics rate.  Drop-back if 0.5s 이상 밀리면 reset.
        _next_step_t += LOOP_DT
        _sleep = _next_step_t - _time.time()
        if _sleep > 0.0:
            _time.sleep(_sleep)
        elif _sleep < -0.5:
            _next_step_t = _time.time()

except KeyboardInterrupt:
    pass
finally:
    # 이 브리지는 로봇에 명령을 보내지 않는다 (관절 상태를 받아 그리기만 함).
    # 그러니 여기서 끄는 것은 화면만 닫는 것이고 실제 로봇 정지와 무관하다.
    print("[bridge] 종료 중...")
    executor.shutdown()
    ros_node.destroy_node()
    rclpy.shutdown()
    app.close()
