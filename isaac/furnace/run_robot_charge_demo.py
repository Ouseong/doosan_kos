"""
robot_charge_demo (PLAN Phase 4): Mobile Base 위 M1013 + 물리 핀어레이 그리퍼(핀 50 개 prismatic + 스프링)가
랙에 세워진 metal_plate 의 긴 변 가장자리를 두께 방향으로 끼워 잡고 들어 올려 용해로 위로 옮긴다. PhysX (Isaac Sim 5.1).

- 로봇: usd/generated/furnace/m1013 (공식 URDF 변환), 고정 베이스 z = layout.robot_base_z
- 그리퍼 물리: 몸체(link_6 에 fixed joint) / jaw 2 개 prismatic (S.x, 위치 drive, 최대 힘 제한) /
  핀 50 개 prismatic (S.x, 스프링 = drive stiffness·damping, 한계 0–stroke). 핀 변위 = joint position (sim GT)
- 판: RigidBody + box collision + density (후보값). 마찰 파지를 먼저 시도하고 미끄러지면 결과에 그대로 기록.
  --attach_fallback 이면 "접촉 후 fixed joint 부착" 대안으로 실행 (결과에 방식 표시)
- 경로: IK 웨이포인트 (m1013_kinematics) + 관절공간 보간. 관절 위치 drive 로 추종
- 로그: output/furnace/robot_charge_demo/<tag>/log.csv (핀 변위 50, jaw, 관절, 판 GT pose — GT 는 평가용 열 gt_*)
"""

import argparse
import csv
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
ap.add_argument("--tag", default="demo")
ap.add_argument("--pin_k", type=float, default=None, help="핀 스프링 강성 덮어쓰기 [N/m]")
ap.add_argument("--attach_fallback", action="store_true")
ap.add_argument("--drop_height", type=float, default=None,
                help="판 최저점을 액면 위 이 높이까지 내린다 [m]. 없으면 예전처럼 rim 위 0.15 m 에서 멈춤")
ap.add_argument("--drop_offset", type=float, default=0.30, help="판 중심을 도가니 중심보다 로봇 쪽으로 [m]")
ap.add_argument("--release", action="store_true", help="마지막에 그리퍼를 열어 판을 놓는다")
ap.add_argument("--release_json", default=None, help="놓는 순간 판 상태(Newton 도가니 좌표)를 저장할 경로")
ap.add_argument("--splash_npz", default=None, help="놓는 순간부터 재생할 Newton splash npz (run_splash_case.py --release_json)")
ap.add_argument("--keep_open", action="store_true", help="GUI: 데모 끝난 뒤 창 유지 (닫을 때까지)")
ap.add_argument("--frames_every", type=int, default=0, help="N 스텝마다 three_quarter/contact 프레임 PNG (0=끔)")
ap.add_argument("--grasp_err", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                help="랙 위 판 초기 위치 오차 (S 기준 dx, dz, ry_deg) — 파지 오차 생성용")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting", "width": 1280, "height": 720})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

import build_factory  # noqa: E402
import build_furnace  # noqa: E402
import build_mobile_base  # noqa: E402
import lighting  # noqa: E402
import usd_util as U  # noqa: E402
from grasp_geometry import (MOUNT_SPACER, PIN_HALF, PIN_YZ, gripper_S_from_link6, link6_from_gripper_S,  # noqa: E402
                            plate_in_S, rot, shift_from_opening)
from m1013_kinematics import M1013, make_T  # noqa: E402

cfg = yaml.safe_load(open(args.config))
lay, gcf, pcf = cfg["layout"], cfg["gripper"], cfg["plate"]
out = os.path.join(ROOT, "output", "furnace", "robot_charge_demo", args.tag)
os.makedirs(out, exist_ok=True)
fx, fy = lay["furnace_xy"]
rim = cfg["furnace"]["rim_height_from_floor_m"]
L_, W_, t_ = pcf["size_m"]
rho = pcf["density_candidates"][pcf["density_index"]]
DT = 1.0 / 240.0

# ── 장면 ────────────────────────────────────────────────────────────────────────
paths = {k: os.path.join(U.GEN, f"{k}.usda") for k in ("factory", "furnace", "mobile_base")}
build_factory.build(cfg, paths["factory"])
build_furnace.build(cfg, paths["furnace"])
build_mobile_base.build(cfg, paths["mobile_base"])
world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT * 4)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
add_reference_to_stage(paths["factory"], "/World/Factory")
add_reference_to_stage(paths["furnace"], "/World/Furnace")
U._set_xf(stage.GetPrimAtPath("/World/Furnace"), (fx, fy, 0), (0, 0, 0))
add_reference_to_stage(paths["mobile_base"], "/World/MobileBase")
add_reference_to_stage(os.path.join(ROOT, "usd/generated/furnace/m1013/m1013_full.usda"), "/World/m1013")
U._set_xf(stage.GetPrimAtPath("/World/m1013"), (0, 0, lay["robot_base_z"]), (0, 0, 0))
scene = UsdPhysics.Scene.Get(stage, "/physicsScene") or UsdPhysics.Scene.Define(stage, "/physicsScene")
ps = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
ps.CreateEnableCCDAttr(True)
for _ in range(5):
    app.update()

# 로봇 관절 drive: importer 기본 stiffness 625 는 하중에서 처짐이 커서 올린다 (값은 결과 JSON 에 기록)
ROBOT_DRIVE = {"stiffness": 2.0e4, "damping": 2.0e3}
joint_paths = []
for p in stage.Traverse():
    if p.IsA(UsdPhysics.RevoluteJoint) and p.GetPath().pathString.startswith("/World/m1013"):
        d = UsdPhysics.DriveAPI.Get(p, "angular")
        d.GetStiffnessAttr().Set(ROBOT_DRIVE["stiffness"])
        d.GetDampingAttr().Set(ROBOT_DRIVE["damping"])
        joint_paths.append(p.GetPath().pathString)

kin = M1013()
base_T = make_T(np.array([0, 0, lay["robot_base_z"]]))

# ── 판 (랙 슬롯에 세워 둠). 판 법선 = world y, 긴 변 = world x, 짧은 변 = world z ─────────────
rx, ry = lay["rack_xy"]
rack_h = lay["rack_height"]
slot_y = ry - 0.15
R_WP = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)     # 열: P.x(법선)=+y, P.y(긴 변)=-x, P.z=+z
T_WP0 = make_T(np.array([rx, slot_y, rack_h + W_ / 2 + 0.003]), R_WP)
dx_e, dz_e, ry_e = args.grasp_err
T_WP0[:3, 3] += R_WP @ np.array([dx_e, 0.0, 0.0]) + np.array([0, 0, 0])
plate_path = "/World/metal_plate"
pl = UsdGeom.Cube.Define(stage, plate_path)
pl.CreateSizeAttr(1.0)
xf = UsdGeom.Xformable(pl); xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(*T_WP0[:3, 3]))
xf.AddOrientOp().Set(Gf.Quatf(Gf.Matrix4d(T_WP0.T.tolist()).ExtractRotationQuat()))
xf.AddScaleOp().Set(Gf.Vec3f(t_, L_, W_))
U.bind(pl, U.material(stage, "plate_mat", color=(0.5, 0.5, 0.52), rough=0.5, metal=0.6, scope="/World/Looks"))
UsdPhysics.RigidBodyAPI.Apply(pl.GetPrim())
UsdPhysics.CollisionAPI.Apply(pl.GetPrim())
UsdPhysics.MassAPI.Apply(pl.GetPrim()).CreateMassAttr(float(rho * L_ * W_ * t_))
pmat = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim("/World/PhysMat/plate", "Material"))
pmat.CreateStaticFrictionAttr(gcf["grasp_mu"] + 0.2); pmat.CreateDynamicFrictionAttr(gcf["grasp_mu"])
from pxr import UsdShade  # noqa: E402
UsdShade.MaterialBindingAPI.Apply(pl.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath("/World/PhysMat/plate")),
                                                   UsdShade.Tokens.weakerThanDescendants, "physics")
PhysxSchema.PhysxRigidBodyAPI.Apply(pl.GetPrim()).CreateEnableCCDAttr(True)
# 랙 충돌 (factory 의 랙은 시각만): 테이블 + 슬롯 칸막이
for k, (cx, cy, cz, sx, sy, sz) in enumerate([(rx, ry, rack_h - 0.02, 0.9, 0.5, 0.04)] +
                                             [(rx, ry - 0.2 + 0.1 * j, rack_h + 0.06, 0.8, 0.012, 0.12) for j in range(5)]):
    c = UsdGeom.Cube.Define(stage, f"/World/RackCollision/b{k}")
    c.CreateSizeAttr(1.0)
    U._set_xf(c, (cx, cy, cz), (0, 0, 0), s=(sx, sy, sz))
    UsdGeom.Imageable(c).CreatePurposeAttr("guide")
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())

# ── 경로 웨이포인트 (IK) ──────────────────────────────────────────────────────────
insert = cfg["grasp"]["insert_depth_m"]
T_SP = plate_in_S(pcf["size_m"], insert)


def S_target(T_WP, lift=0.0):
    T = T_WP @ np.linalg.inv(T_SP)
    T[2, 3] += lift
    return T


q_home = np.deg2rad([0, 0, -90, 0, -90, 0])
q_seed = np.deg2rad([-60, 10, 100, 0, 70, 30])
wps = {}
for name, T_S in (("pre_grasp", S_target(T_WP0, 0.15)), ("grasp", S_target(T_WP0, 0.0)),
                  ("lift", S_target(T_WP0, 0.25))):
    q, pe, re = kin.ik(link6_from_gripper_S(T_S), q0=q_seed, base=base_T)
    wps[name] = (q, pe, re)
    q_seed = q
# 용해로 위: 판을 세운 그대로(법선 수평). 긴 변은 접선 방향(y) — 작업반경 1.3 m 안에 들도록 판 중심을
# 도가니 중심보다 로봇 쪽으로 drop_offset 만큼, 판 하단은 rim 위 drop_clear 만큼
drop_offset, drop_clear = args.drop_offset, 0.15     # 오프라인 IK 확인: offset 0.15–0.30, 액면 위 0.03 m 까지 도달 가능
surf_z = rim - cfg["furnace"]["crucible_depth_m"] + cfg["furnace"]["melt_depth_m"]
if args.drop_height is not None:
    drop_clear = 0.08                                   # 낮게 넣을 때 경유점: rim 위 0.08 m (offset 0.25 에서 IK 해 있음)
z_above = rim + drop_clear + W_ / 2
z_drop = z_above if args.drop_height is None else surf_z + args.drop_height + W_ / 2
T_WPf = make_T(np.array([fx - drop_offset, fy, z_drop]), rot("z", np.pi / 2) @ R_WP)
for name, T_S in (("transit", S_target(T_WPf, z_above - z_drop + (0.06 if args.drop_height is not None else 0.08))), ("above", S_target(T_WPf, z_above - z_drop)),
                  ("over_furnace", S_target(T_WPf, 0.0))):
    q, pe, re = kin.ik(link6_from_gripper_S(T_S), q0=q_seed, base=base_T)
    wps[name] = (q, pe, re)
    q_seed = q
ik_report = {k: {"q_deg": np.degrees(v[0]).round(2).tolist(), "pos_err_mm": v[1] * 1000, "rot_err_deg": float(np.degrees(v[2]))}
             for k, v in wps.items()}
print("[demo] IK", json.dumps({k: (round(v["pos_err_mm"], 2), round(v["rot_err_deg"], 2)) for k, v in ik_report.items()}))

# ── 그리퍼 물리 (link_6 에 붙인다) ───────────────────────────────────────────────────
q_init = wps["pre_grasp"][0]
T_W6 = kin.fk(q_init, base_T)
T_WS = gripper_S_from_link6(T_W6)
G = "/World/Gripper"
UsdGeom.Xform.Define(stage, G)
grip_mat = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim("/World/PhysMat/pin", "Material"))
grip_mat.CreateStaticFrictionAttr(gcf["grasp_mu"] + 0.2); grip_mat.CreateDynamicFrictionAttr(gcf["grasp_mu"])
pin_mat = UsdShade.Material(stage.GetPrimAtPath("/World/PhysMat/pin"))
dark = U.material(stage, "grip_dark", color=(0.13, 0.13, 0.14), rough=0.55, scope="/World/Looks")
steel = U.material(stage, "grip_steel", color=(0.92, 0.93, 0.95), metal=0.35, rough=0.25,   # 은색 핀: 금속성 높으면 어두운 공장에서 검게 보임
                   emissive=(0.55, 0.56, 0.58), emissive_intensity=0.15, scope="/World/Looks")


def body(path, T_world, mass, boxes, mat=None, vis_mat=None):
    """boxes: [(center_in_body, size)]. body frame = T_world (S 방향과 같게)."""
    x = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(x); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*T_world[:3, 3]))
    xf.AddOrientOp().Set(Gf.Quatf(Gf.Matrix4d(T_world.T.tolist()).ExtractRotationQuat()))
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(float(mass))
    for i, (c, s) in enumerate(boxes):
        b = UsdGeom.Cube.Define(stage, f"{path}/col{i}")
        b.CreateSizeAttr(1.0)
        U._set_xf(b, c, (0, 0, 0), s=s)
        UsdPhysics.CollisionAPI.Apply(b.GetPrim())
        if mat is not None:
            UsdShade.MaterialBindingAPI.Apply(b.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        if vis_mat is not None:
            U.bind(b, vis_mat)
    return x


def joint(kind, path, b0, b1, T_W0, T_W1, T_Wj, axis="X", lower=None, upper=None):
    """joint frame = T_Wj (world). local0/1 = 각 바디 프레임 기준."""
    J = (UsdPhysics.FixedJoint if kind == "fixed" else UsdPhysics.PrismaticJoint).Define(stage, path)
    J.CreateBody0Rel().SetTargets([b0]); J.CreateBody1Rel().SetTargets([b1])
    for T_Wb, setp, setr in ((T_W0, J.CreateLocalPos0Attr, J.CreateLocalRot0Attr), (T_W1, J.CreateLocalPos1Attr, J.CreateLocalRot1Attr)):
        T = np.linalg.inv(T_Wb) @ T_Wj
        setp().Set(Gf.Vec3f(*T[:3, 3]))
        setr().Set(Gf.Quatf(Gf.Matrix4d(T.T.tolist()).ExtractRotationQuat()))
    if kind == "prismatic":
        J.CreateAxisAttr(axis)
        J.CreateLowerLimitAttr(float(lower)); J.CreateUpperLimitAttr(float(upper))
    return J


# 몸체: body.stl bbox (x ±80, y -40..50.5, z -87.6..0 mm) → 박스. 스페이서 포함
body_T = T_WS
body("/World/Gripper/Body", body_T, gcf["mass_kg"] * 0.7,
     [((0, 0.005, -0.044), (0.16, 0.09, 0.087)), ((0, 0, 0.0175), (0.08, 0.08, 0.035))], vis_mat=dark)
link6_path = "/World/m1013/link_6"
joint("fixed", G + "/mount", link6_path, G + "/Body", T_W6, body_T, body_T)

s0 = shift_from_opening(gcf["max_opening_m"])
JAW_DRIVE = {"stiffness": 2.0e5, "damping": 2.0e3, "max_force": 150.0}    # max_force: [가정] 모터 파지력 상한
PIN_DRIVE = {"stiffness": args.pin_k or gcf["pin_spring_k"], "damping": gcf["pin_damping"]}
stroke = gcf["pin_stroke_m"]
jaw_joints, pin_joints = [], []
for side, sign in (("p", 1), ("n", -1)):
    T_jaw = body_T @ make_T(np.array([sign * s0, 0, 0]))
    # 하우징: 핀 끝에서 16.5 mm 바깥 (house STL x 16.5..55.5 mm, 닫힘 기준)
    jp = f"{G}/Jaw_{side}"
    body(jp, T_jaw, gcf["mass_kg"] * 0.12,
         [((sign * 0.036, 0.009, -0.1246), (0.039, 0.079, 0.074))], mat=pin_mat, vis_mat=dark)
    J = joint("prismatic", f"{G}/jaw_joint_{side}", G + "/Body", jp, body_T, T_jaw, T_jaw, "X", -0.05, 0.05)
    d = UsdPhysics.DriveAPI.Apply(J.GetPrim(), "linear")
    d.CreateTypeAttr("force"); d.CreateTargetPositionAttr(0.0)
    d.CreateStiffnessAttr(JAW_DRIVE["stiffness"]); d.CreateDampingAttr(JAW_DRIVE["damping"])
    d.CreateMaxForceAttr(JAW_DRIVE["max_force"])
    jaw_joints.append(J.GetPrim().GetPath().pathString)
    for i, (y, z) in enumerate(PIN_YZ):
        T_pin = T_jaw @ make_T(np.array([0, y, z]))
        pp = f"{jp}_pin{i:02d}"
        body(pp, T_pin, 0.004, [((sign * 0.008, 0, 0), (0.016, 2 * PIN_HALF * 0.95, 2 * PIN_HALF * 0.95))],
             mat=pin_mat, vis_mat=steel)
        lo, hi = (0.0, stroke) if sign > 0 else (-stroke, 0.0)
        J = joint("prismatic", f"{jp}_pinjoint{i:02d}", jp, pp, T_jaw, T_pin, T_pin, "X", lo, hi)
        d = UsdPhysics.DriveAPI.Apply(J.GetPrim(), "linear")
        d.CreateTypeAttr("force"); d.CreateTargetPositionAttr(0.0)
        d.CreateStiffnessAttr(PIN_DRIVE["stiffness"]); d.CreateDampingAttr(PIN_DRIVE["damping"])
        pin_joints.append(J.GetPrim().GetPath().pathString)

# 핀-핀·핀-jaw 자기 충돌은 끄고 (articulation self-collision off), 판·랙 과만 충돌
lighting.apply(stage, cfg, "dark_foundry")
E = cfg["lighting"]["exposure"]
lighting.set_fixed_exposure(E["film_iso"], E["shutter_s"], E["f_number"])

art = None
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI) and p.GetPath().pathString.startswith("/World/m1013"):
        art = p.GetPath().pathString
        break
PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(art)).CreateSolverPositionIterationCountAttr(64)
robot = SingleArticulation(prim_path=art, name="m1013")
world.scene.add(robot)
plate = SingleRigidPrim(prim_path=plate_path, name="plate")
world.scene.add(plate)
world.reset()
names = list(robot.dof_names)
print(f"[demo] articulation dofs={robot.num_dof} (robot 6 + jaw 2 + pin 50 = 58 이어야 함)")
idx_robot = [names.index(f"joint_{i}") for i in range(1, 7)]
idx_jaw = [names.index(os.path.basename(p)) for p in jaw_joints]
idx_pin = [names.index(os.path.basename(p)) for p in pin_joints]

q_full = np.zeros(robot.num_dof)
q_full[idx_robot] = q_init
robot.set_joint_positions(q_full)
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

ctrl = robot.get_articulation_controller()


def set_targets(q_arm, opening):
    tgt = robot.get_joint_positions().copy()
    tgt[idx_robot] = q_arm
    s = shift_from_opening(opening) - s0                  # jaw joint = 초기(열림) 위치 기준 이동량
    tgt[idx_jaw] = [s, -s]
    tgt[idx_pin] = 0.0
    ctrl.apply_action(ArticulationAction(joint_positions=tgt))


# ── 시나리오 ───────────────────────────────────────────────────────────────────
plan = [("settle", None, None, 0.5, gcf["max_opening_m"]),
        ("approach", "pre_grasp", "grasp", 2.0, gcf["max_opening_m"]),
        ("close", "grasp", "grasp", 1.5, 0.0),
        ("lift", "grasp", "lift", 2.0, 0.0),
        ("transit", "lift", "transit", 3.0, 0.0),
        ("lower", "transit", "above", 1.5, 0.0),
        ("descend", "above", "over_furnace", 2.0 if args.drop_height is not None else 0.01, 0.0),
        ("hold", "over_furnace", "over_furnace", 1.0, 0.0)]
if args.release:
    plan += [("release", "over_furnace", "over_furnace", 0.6, gcf["max_opening_m"]),
             ("after", "over_furnace", "above", 2.0, gcf["max_opening_m"])]

# ── 놓는 순간 판 상태 / Newton splash 재생 ────────────────────────────────────────────
floor_z = rim - cfg["furnace"]["crucible_depth_m"]
N_ORIGIN = np.array([fx, fy, floor_z])                     # Newton 도가니 좌표 원점 = 도가니 바닥 중심
P_TO_N = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)  # 열 = Newton 판 박스 축(x=L, y=W, z=t) 을 P(x=t, y=L, z=W)로
release_t = None
pb = None
if args.splash_npz:
    from pxr import Vt  # noqa: E402
    zz = np.load(args.splash_npz)
    pb = {"P": zz["particles_sub"].astype(np.float32), "plate": zz["plate"], "every": int(zz["every_n_frames"]),
          "fps": float(zz["fps"]), "size": zz["plate_size"]}
    pts = UsdGeom.Points.Define(stage, "/World/Splash/fluid")
    pts.CreateWidthsAttr(Vt.FloatArray([0.010] * pb["P"].shape[1]))
    U.bind(pts, U.material(stage, "pb_fluid", color=(1.0, 0.42, 0.08), rough=0.6, emissive=(1.0, 0.35, 0.05),
                           emissive_intensity=0.6, scope="/World/Looks"))
    pts.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pb["P"][0] + N_ORIGIN.astype(np.float32)))
    pbp = UsdGeom.Cube.Define(stage, "/World/Splash/metal_plate")
    pbp.CreateSizeAttr(1.0)
    U.bind(pbp, U.material(stage, "pb_plate", color=(0.55, 0.55, 0.58), rough=0.45, metal=0.7, scope="/World/Looks"))
    _x = UsdGeom.Xformable(pbp)
    pb["ops"] = (_x.AddTranslateOp(), _x.AddOrientOp(), _x.AddScaleOp())
    pb["ops"][2].Set(Gf.Vec3f(*map(float, pb["size"])))
    UsdGeom.Imageable(pbp).MakeInvisible()
    pb["pts"], pb["pbp"] = pts, pbp
    for p in stage.Traverse():                               # 정지 용탕 원판은 숨기고 입자를 액체로 보인다
        if p.GetName() == "melt_surface" or p.GetName().startswith("crust"):
            UsdGeom.Imageable(p).MakeInvisible()


def show_splash(tau):
    """tau = 놓은 뒤 경과 시간 [s]. Newton 기록의 해당 프레임을 그린다 (재생)."""
    k = min(int(tau * pb["fps"] / pb["every"]), len(pb["P"]) - 1)
    f = min(k * pb["every"], len(pb["plate"]) - 1)
    pb["pts"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pb["P"][k] + N_ORIGIN.astype(np.float32)))
    p, q = pb["plate"][f, :3] + N_ORIGIN, pb["plate"][f, 3:]
    pb["ops"][0].Set(Gf.Vec3d(*map(float, p)))
    pb["ops"][1].Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))
    return k >= len(pb["P"]) - 1


def capture_release():
    from scipy.spatial.transform import Rotation as Rot
    pos, qw = plate.get_world_pose()                         # (w, x, y, z)
    R_WP_now = Rot.from_quat([qw[1], qw[2], qw[3], qw[0]]).as_matrix()
    R_WN = R_WP_now @ P_TO_N
    st = {"t_release": t, "pos": (np.asarray(pos) - N_ORIGIN).tolist(),
          "quat_xyzw": Rot.from_matrix(R_WN).as_quat().tolist(),
          "lin_vel": np.asarray(plate.get_linear_velocity()).tolist(),
          "ang_vel": np.asarray(plate.get_angular_velocity()).tolist(),
          "frame": "Newton crucible frame: origin = crucible floor centre, z up; plate box axes x=L, y=W, z=t",
          "source": "Isaac Sim PhysX plate state at the first step of the release phase (gripper starts opening)"}
    if args.release_json:
        json.dump(st, open(args.release_json, "w"), indent=2)
    print("[demo] release state", json.dumps({k: (np.round(v, 4).tolist() if isinstance(v, list) else v)
                                              for k, v in st.items() if k in ("t_release", "pos", "lin_vel")}), flush=True)
    return st
rows = []
attached = None
t = 0.0
cams = None
frame_k = 0
if args.frames_every:
    from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
    from isaacsim.sensors.camera import Camera  # noqa: E402
    import cameras  # noqa: E402
    cams = {}
    for nm in ("three_quarter",):
        e, tg = cameras.presets(cfg)[nm]
        cams[nm] = Camera(prim_path=f"/World/Cameras/{nm}", resolution=(960, 540))
        cams[nm].initialize()
        set_camera_view(eye=e, target=tg, camera_prim_path=f"/World/Cameras/{nm}")
        cams[nm].set_clipping_range(0.01, 1e4)
    os.makedirs(os.path.join(out, "frames"), exist_ok=True)

if not args.headless:
    import cameras  # noqa: E402
    from isaacsim.core.utils.viewports import set_camera_view as _scv  # noqa: E402
    _e, _tg = cameras.presets(cfg)["three_quarter"]
    if args.drop_height is not None:                          # 용해로 안이 보이는 시점
        _e, _tg = np.array([fx - 0.9, fy - 2.3, rim + 1.5]), np.array([fx - 0.25, fy, rim - 0.05])
    _scv(eye=_e, target=_tg)
    for _ in range(30):
        app.update()

for phase, a, b, dur, opening in plan:
    n = int(round(dur / DT))
    qa = wps[a][0] if a else q_init
    qb = wps[b][0] if b else q_init
    for k in range(n):
        u = (k + 1) / n
        u = 3 * u * u - 2 * u ** 3                         # smoothstep
        set_targets(qa + (qb - qa) * u, opening)
        if phase == "release" and k == 0:
            capture_release()
            release_t = t
            if pb is not None:                                # 여기부터 판·액체는 Newton 기록 재생
                plate.disable_rigid_body_physics()
                UsdGeom.Imageable(stage.GetPrimAtPath(plate_path)).MakeInvisible()
                UsdGeom.Imageable(pb["pbp"]).MakeVisible()
        if pb is not None and release_t is not None:
            show_splash(t - release_t)
        world.step(render=(bool(cams) or not args.headless) and (k % 4 == 0))
        t += DT
        if args.attach_fallback and phase == "close" and k == n - 1 and attached is None:
            # 대안: 접촉 후 fixed joint 로 판을 그리퍼 몸체에 부착 (결과에 표시)
            Tb = np.array(UsdGeom.Xformable(stage.GetPrimAtPath(G + "/Body")).ComputeLocalToWorldTransform(0)).T
            Tp = np.array(UsdGeom.Xformable(stage.GetPrimAtPath(plate_path)).ComputeLocalToWorldTransform(0)).T
            Tp_noscale = Tp.copy(); Tp_noscale[:3, :3] /= np.linalg.norm(Tp[:3, :3], axis=0)
            joint("fixed", "/World/attach_joint", G + "/Body", plate_path, Tb, Tp_noscale, Tb)
            attached = t
        if k % 6 == 0:
            q = robot.get_joint_positions()
            pos, quat = plate.get_world_pose()
            Tb = np.array(UsdGeom.Xformable(stage.GetPrimAtPath(G + "/Body")).ComputeLocalToWorldTransform(0)).T
            Rb = Tb[:3, :3] / np.linalg.norm(Tb[:3, :3], axis=0)
            rel = Rb.T @ (np.asarray(pos) - Tb[:3, 3])        # 판 중심을 그리퍼(S) 프레임으로 (평가용 GT)
            r = {"t": round(t, 4), "phase": phase, "opening_cmd": opening,
                 **{f"q{i + 1}": float(q[j]) for i, j in enumerate(idx_robot)},
                 "jaw_p": float(q[idx_jaw[0]]), "jaw_n": float(q[idx_jaw[1]]),
                 **{f"pin_p{i:02d}": float(abs(q[idx_pin[i]])) for i in range(25)},
                 **{f"pin_n{i:02d}": float(abs(q[idx_pin[25 + i]])) for i in range(25)},
                 "gt_plate_x": float(pos[0]), "gt_plate_y": float(pos[1]), "gt_plate_z": float(pos[2]),
                 "gt_plate_qw": float(quat[0]), "gt_plate_qx": float(quat[1]), "gt_plate_qy": float(quat[2]),
                 "gt_plate_qz": float(quat[3]),
                 "gt_plate_in_S_x": float(rel[0]), "gt_plate_in_S_y": float(rel[1]), "gt_plate_in_S_z": float(rel[2])}
            rows.append(r)
        if cams and k % args.frames_every == 0:
            for nm, cam in cams.items():
                rgba = cam.get_rgba()
                if rgba is not None and rgba.size:
                    from PIL import Image
                    Image.fromarray(rgba[..., :3].astype(np.uint8)).save(os.path.join(out, "frames", f"{nm}_{frame_k:04d}.png"))
            frame_k += 1
    print(f"[demo] {phase:10s} t={t:5.2f}s plate_z={rows[-1]['gt_plate_z']:.3f} "
          f"pins_p={sum(rows[-1][f'pin_p{i:02d}'] > 5e-4 for i in range(25))} "
          f"pins_n={sum(rows[-1][f'pin_n{i:02d}'] > 5e-4 for i in range(25))} "
          f"rel_S=({rows[-1]['gt_plate_in_S_x']*1000:.1f},{rows[-1]['gt_plate_in_S_y']*1000:.1f},{rows[-1]['gt_plate_in_S_z']*1000:.1f})mm", flush=True)

with open(os.path.join(out, "log.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)

# 판정: 들어 올린 뒤 판이 그리퍼를 따라왔는가 (판 z 상승, S 기준 상대 위치 변화)
def at(phase):
    return [r for r in rows if r["phase"] == phase]


z_rack = T_WP0[2, 3]
rel_close = np.array([[r[f"gt_plate_in_S_{c}"] for c in "xyz"] for r in at("close")[-3:]]).mean(0)
rel_hold = np.array([[r[f"gt_plate_in_S_{c}"] for c in "xyz"] for r in at("hold")[-3:]]).mean(0)
res = {"grasp_method": "fixed-joint attach after contact (fallback)" if args.attach_fallback else "friction (pin springs + jaw force)",
       "physics": "PhysX (Isaac Sim 5.1), dt=1/240", "robot_drive": ROBOT_DRIVE, "jaw_drive": JAW_DRIVE, "pin_drive": PIN_DRIVE,
       "pin_stroke_m": stroke, "plate_mass_kg": rho * L_ * W_ * t_, "plate_density_candidate": rho,
       "ik": ik_report, "articulation_dofs": int(robot.num_dof),
       "plate_z_rack": float(z_rack), "plate_z_end": rows[-1]["gt_plate_z"],
       "plate_lifted": bool(at("hold")[-1]["gt_plate_z"] > z_rack + 0.2),
       "released": release_t is not None, "drop_height_m": args.drop_height, "drop_offset_m": args.drop_offset,
       "plate_in_S_after_close_mm": (rel_close * 1000).round(2).tolist(),
       "plate_in_S_at_hold_mm": (rel_hold * 1000).round(2).tolist(),
       "slip_during_transport_mm": float(np.linalg.norm(rel_hold - rel_close) * 1000),
       "pins_pressed_at_hold": {"p": int(sum(at("hold")[-1][f"pin_p{i:02d}"] > 5e-4 for i in range(25))),
                                "n": int(sum(at("hold")[-1][f"pin_n{i:02d}"] > 5e-4 for i in range(25)))},
       "note": "pin displacement = prismatic joint position (simulation ground truth). gt_* columns for evaluation only"}
json.dump(res, open(os.path.join(out, "result.json"), "w"), indent=2)
print(json.dumps(res, indent=2))
if args.keep_open and not args.headless:
    print("[demo] 완료 — 창을 닫으면 종료")
    import time as _time
    _t0 = _time.time()
    while app.is_running():
        if pb is not None and release_t is not None:          # splash 를 0.5 배속으로 반복 재생
            tau = ((_time.time() - _t0) * 0.5) % (len(pb["P"]) * pb["every"] / pb["fps"] + 0.5)
            show_splash(tau)
        app.update()
app.close()
