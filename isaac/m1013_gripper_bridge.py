"""
M1013 + 그리퍼 통합 Isaac Sim 브릿지

- M1013 관절 제어: /dsr01/joint_states (sensor_msgs/JointState)
- 그리퍼 열기/닫기: /gripper_command   (control_msgs/GripperCommand)
- 그리퍼 상태 발행: /gripper_state     (sensor_msgs/JointState)

동작 방식:
  매 프레임마다 M1013 tool0의 world transform을 읽어
  그리퍼 루트를 해당 위치/방향으로 이동 (kinematic follow)

실행:
  docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/m1013_gripper_bridge.py
  docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/m1013_gripper_bridge.py --headless
"""

import argparse
import os
import sys
import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

parser = argparse.ArgumentParser()
parser.add_argument("--topic",    default="dsr01/joint_states")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--robot_usd",   default="/tmp/m1013_v2/m1013_full.usda")
parser.add_argument("--gripper_usd", default="/kos_workspace/usd/parts/gripper_assembly_physics.usd")
args, unknown = parser.parse_known_args()

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

ROBOT_PRIM   = "/World/m1013"
GRIPPER_PRIM = "/World/Gripper"

add_reference_to_stage(usd_path=args.robot_usd, prim_path=ROBOT_PRIM)
gripper_prim = stage.DefinePrim(GRIPPER_PRIM, "Xform")
gripper_prim.GetReferences().AddReference(assetPath=args.gripper_usd, primPath="/World/Gripper")
print(f"[bridge] M1013 로드:  {args.robot_usd}")
print(f"[bridge] 그리퍼 로드: {args.gripper_usd}")

# 그리퍼 collision approximation 패치 (PhysX가 dynamic body의 triangle mesh를 못 다루므로
# convexHull로 명시. 안 하면 첫 step에서 prim 삭제 → tensor view 무효화 → 시뮬 죽음)
_coll_patched = 0
for _p in stage.Traverse():
    if _p.GetPath().pathString.startswith(GRIPPER_PRIM) \
       and _p.HasAPI(UsdPhysics.CollisionAPI) and _p.IsA(UsdGeom.Mesh):
        UsdPhysics.MeshCollisionAPI.Apply(_p).CreateApproximationAttr().Set("convexHull")
        _coll_patched += 1
print(f"[bridge] 그리퍼 collision 패치: {_coll_patched} prims → convexHull")

# right_joint axis 반전 + limit 정상화
# (소스 USD가 axis=X / upper=-0.0335 로 설정해서 PhysX가 무시했음)
_rj = stage.GetPrimAtPath("/World/Gripper/right_joint")
if _rj.IsValid():
    _j = UsdPhysics.PrismaticJoint(_rj)
    _j.GetLowerLimitAttr().Set(0.0)
    _j.GetUpperLimitAttr().Set(0.0335)
    _flip_y = Gf.Quatf(0, 0, 1, 0)   # Y축 180°
    _j.GetLocalRot0Attr().Set(_flip_y)
    _j.GetLocalRot1Attr().Set(_flip_y)
    print(f"[bridge] right_joint axis 반전 + limit [0, 0.0335]")

# 옵션 A 3차: articulation 유지 + body_world_joint 비활성 + 중력 OFF
# 이유:
#   - articulation 유지 → SingleArticulation API 그대로 사용 가능 (set_world_pose, set_joint_positions)
#   - body_world_joint 비활성 → set_world_pose가 world fixed joint와 충돌 안 함
#   - 중력 OFF → floating articulation이지만 떨어지지 않음 (떠다니지 않음)
from pxr import PhysxSchema
_fj_world = stage.GetPrimAtPath("/World/Gripper/body_world_joint")
if _fj_world.IsValid():
    _fj_world.SetActive(False)
    print(f"[bridge] body_world_joint 비활성")

for _bp in ["/World/Gripper/body", "/World/Gripper/left_plate", "/World/Gripper/right_plate"]:
    _p = stage.GetPrimAtPath(_bp)
    if _p.IsValid():
        PhysxSchema.PhysxRigidBodyAPI.Apply(_p).CreateDisableGravityAttr().Set(True)
print(f"[bridge] 그리퍼 rigid body 중력 비활성")

# 옵션 A: 그리퍼 collision 비활성화 (kinematic follow 중에 외력으로 튕기는 거 방지)
# 옵션 B (망치질) 단계에서 다시 켜야 함
_coll_off = 0
for _p in stage.Traverse():
    _pp = _p.GetPath().pathString
    if _pp.startswith(GRIPPER_PRIM) and _p.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI(_p).CreateCollisionEnabledAttr().Set(False)
        _coll_off += 1
print(f"[bridge] 그리퍼 collision 비활성: {_coll_off} prims (옵션 B 단계에서 재활성화 필요)")

for _ in range(10):
    app.update()

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
    print(f"[bridge] 체크 바닥: {size_m}m × {size_m}m, tile={tile_m*100:.0f}cm")

_setup_checker_floor(stage)

# ── 조명 ──────────────────────────────────────────────────────────────────
UsdLux.DistantLight.Define(stage, "/World/DistantLight").CreateIntensityAttr(3000)
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(500)

# ── ArticulationRoot 탐지 ─────────────────────────────────────────────────
robot_art_path = None
gripper_art_path = None
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        continue
    p = str(prim.GetPath())
    if p.startswith(ROBOT_PRIM) and robot_art_path is None:
        robot_art_path = p
    if p.startswith(GRIPPER_PRIM) and gripper_art_path is None:
        gripper_art_path = p

print(f"[bridge] M1013 ArticulationRoot:  {robot_art_path}")
print(f"[bridge] 그리퍼 ArticulationRoot: {gripper_art_path}")

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
print(f"[bridge] 그리퍼 장착 기준: {tool0_path}")

# ── Articulation 객체 ──────────────────────────────────────────────────────
from isaacsim.core.prims import SingleArticulation

robot   = SingleArticulation(prim_path=robot_art_path,   name="m1013")
gripper = SingleArticulation(prim_path=gripper_art_path, name="gripper")
world.scene.add(robot)
world.scene.add(gripper)
world.reset()

print(f"[bridge] M1013  관절: {robot.num_dof}개 → {robot.dof_names}")
print(f"[bridge] 그리퍼 관절: {gripper.num_dof}개 → {gripper.dof_names}")

# ── 그리퍼 장착 오프셋 (2026-04-27 GUI 캘리브레이션 결과) ──────────────────
MOUNT_RPY = np.array([-np.pi / 2, 0.0, 0.0])   # tool0 +Y에 그리퍼 정면
MOUNT_XYZ = np.array([0.0, 0.012, 0.0])        # link_6 +Y로 12mm (15→12로 더 붙임)

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

mount_quat = rpy_to_quat_wxyz(MOUNT_RPY)

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

# ── 그리퍼 관절 설정 ───────────────────────────────────────────────────────
MAX_OPENING = 0.067   # m
MAX_TRAVEL  = 0.0335  # m

def opening_to_joint(opening_m):
    opening_m = float(np.clip(opening_m, 0.0, MAX_OPENING))
    return (MAX_OPENING - opening_m) / 2.0

# ── ROS2 ──────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32   # GripperCommand가 Isaac Sim Python 3.11 번들에 없어 대체

rclpy.init()

class BridgeNode(Node):
    def __init__(self):
        super().__init__("m1013_gripper_bridge")
        qos = QoSProfile(depth=10)

        self.robot_joints   = None
        self.target_opening = MAX_OPENING   # 시작 시 완전 열림

        self.create_subscription(
            JointState, f"/{args.topic}", self._robot_cb, qos)
        self.create_subscription(
            Float32, "/gripper_command", self._gripper_cb, qos)
        self.state_pub = self.create_publisher(
            JointState, "/gripper_state", qos)

        self.get_logger().info("브릿지 시작. M1013 + 그리퍼 대기 중...")

    def _robot_cb(self, msg):
        if len(msg.position) > 0:
            self.robot_joints = np.array(msg.position)

    def _gripper_cb(self, msg):
        # std_msgs/Float32: data = 열림 너비 (m, 0.0~0.067)
        self.target_opening = msg.data
        self.get_logger().info(
            f"그리퍼 명령: {msg.data*1000:.1f}mm")

    def publish_gripper_state(self, dof_pos):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['gripper_left_joint', 'gripper_right_joint']
        if len(dof_pos) >= 2:
            js.position = [float(dof_pos[0]), float(dof_pos[1])]
        elif len(dof_pos) == 1:
            js.position = [float(dof_pos[0]), float(dof_pos[0])]
        self.state_pub.publish(js)


ros_node = BridgeNode()

# ── 초기 포즈 ─────────────────────────────────────────────────────────────
HOME_POSE = np.deg2rad([0.0, 0.0, -90.0, 0.0, -90.0, 0.0])
robot.set_joint_positions(HOME_POSE)
for _ in range(10):
    world.step(render=True)
print(f"[bridge] M1013 초기 포즈 적용")

world.play()
print("[bridge] 시뮬레이션 루프 시작. Ctrl-C로 종료.")

try:
    while app.is_running():
        rclpy.spin_once(ros_node, timeout_sec=0)

        # ── M1013 관절 제어 ────────────────────────────────────────────
        if ros_node.robot_joints is not None:
            joints = ros_node.robot_joints
            if len(joints) == robot.num_dof and not np.allclose(joints, 0.0):
                robot.set_joint_positions(joints)

        # ── 그리퍼를 tool0에 붙이기 (kinematic follow) ─────────────────
        tool0_pos, tool0_quat = get_tool0_world_pose()
        gripper_quat = quat_multiply(tool0_quat, mount_quat)
        offset_world = rotate_vec(tool0_quat, MOUNT_XYZ)
        gripper_pos  = tool0_pos + offset_world

        gripper.set_world_pose(position=gripper_pos, orientation=gripper_quat)

        # ── 그리퍼 관절 제어 ───────────────────────────────────────────
        joint_target = opening_to_joint(ros_node.target_opening)
        gripper.set_joint_positions(np.full(gripper.num_dof, joint_target))

        # ── 그리퍼 상태 발행 ───────────────────────────────────────────
        cur_pos = gripper.get_joint_positions()
        ros_node.publish_gripper_state(cur_pos)

        world.step(render=True)

except KeyboardInterrupt:
    print("[bridge] 종료 중...")
finally:
    rclpy.shutdown()
    app.close()
