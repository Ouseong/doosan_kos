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
from pxr import UsdPhysics, UsdGeom, UsdLux, Gf, Usd, Sdf
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

add_reference_to_stage(usd_path=args.robot_usd,   prim_path=ROBOT_PRIM)
add_reference_to_stage(usd_path=args.gripper_usd, prim_path=GRIPPER_PRIM)
print(f"[bridge] M1013 로드:  {args.robot_usd}")
print(f"[bridge] 그리퍼 로드: {args.gripper_usd}")

for _ in range(10):
    app.update()

# ── 조명 ──────────────────────────────────────────────────────────────────
UsdLux.DistantLight.Define(stage, "/World/DistantLight").CreateIntensityAttr(3000)
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(500)

# ── M1013 ArticulationRoot 탐지 ────────────────────────────────────────────
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

# ── tool0 경로 설정 ────────────────────────────────────────────────────────
# urdf_to_usd_v2.py에서 merge_fixed_joints=True로 임포트했기 때문에
# tool0는 link6에 병합됨 → 마지막 링크 = link6
tool0_path = ROBOT_PRIM + "/link6"
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

# ── 그리퍼 장착 오프셋 설정 ────────────────────────────────────────────────
# M1013 tool0 기준 그리퍼 장착 회전
# 그리퍼 플랜지가 -Y 방향 → tool0 +Z 방향에 맞추려면 X축 +90° 회전
# ⚠️ 실제로 Isaac Sim에서 확인 후 rpy 조정 필요
MOUNT_RPY = np.array([np.pi / 2, 0.0, 0.0])   # (roll, pitch, yaw) rad
MOUNT_XYZ = np.array([0.0, 0.0, 0.0])          # tool0 기준 오프셋 (m)

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
from control_msgs.msg import GripperCommand

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
            GripperCommand, "/gripper_command", self._gripper_cb, qos)
        self.state_pub = self.create_publisher(
            JointState, "/gripper_state", qos)

        self.get_logger().info("브릿지 시작. M1013 + 그리퍼 대기 중...")

    def _robot_cb(self, msg):
        if len(msg.position) > 0:
            self.robot_joints = np.array(msg.position)

    def _gripper_cb(self, msg):
        self.target_opening = msg.position
        self.get_logger().info(
            f"그리퍼 명령: {msg.position*1000:.1f}mm")

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
HOME_POSE = np.array([0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0])
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

        # 장착 회전 합성: tool0_quat * mount_quat
        gripper_quat = quat_multiply(tool0_quat, mount_quat)
        # 장착 오프셋 (tool0 방향으로 회전된 오프셋)
        offset_world = rotate_vec(tool0_quat, MOUNT_XYZ)
        gripper_pos  = tool0_pos + offset_world

        gripper.set_world_pose(
            position=gripper_pos,
            orientation=gripper_quat,
        )

        # ── 그리퍼 관절 제어 ───────────────────────────────────────────
        joint_target = opening_to_joint(ros_node.target_opening)
        n = gripper.num_dof
        targets = np.full(n, joint_target)
        gripper.set_joint_positions(targets)

        # ── 그리퍼 상태 발행 ───────────────────────────────────────────
        cur_pos = gripper.get_joint_positions()
        ros_node.publish_gripper_state(cur_pos)

        world.step(render=True)

except KeyboardInterrupt:
    print("[bridge] 종료 중...")
finally:
    rclpy.shutdown()
    app.close()
