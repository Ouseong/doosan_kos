"""
Isaac Sim 5.1.0 standalone app
M1013 USD 로드 + ROS2 joint_states 구독 → 관절 제어
✅ Isaac Sim bundled rclpy (Python 3.11) 사용 → 시스템 ROS2 Jazzy (Python 3.12) 와 호환 문제 해결

실행: docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/m1013_ros2_bridge.py
"""

import argparse
import os
import sys

# ROS2 discovery 환경변수
os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

parser = argparse.ArgumentParser()
parser.add_argument("--topic",    default="dsr01/joint_states", help="구독할 joint_states 토픽")
parser.add_argument("--headless", action="store_true",          help="헤드리스 모드")
args, unknown = parser.parse_known_args()

# ── Isaac Sim 앱 부팅 ──
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "renderer": "RayTracedLighting",
        "headless": args.headless,
        "width":    1280,
        "height":   720,
    },
    experience="/isaac-sim/apps/isaacsim.exp.base.kit",
)

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics

# ── ROS2 Bridge 확장 활성화 (번들 rclpy 경로 설정) ──
enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    app.update()

# 번들 rclpy 경로를 sys.path 에 추가 (Isaac Sim Python 3.11 호환)
BUNDLED_RCLPY = "/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy"
if BUNDLED_RCLPY not in sys.path:
    sys.path.insert(0, BUNDLED_RCLPY)

# ── USD 경로 ──
# M1013: assemble_m1013.py 로 base.usd (메시) + physics.usd (조인트) 합쳐서 만든 self-contained
USD_PATH   = "/tmp/m1013_v2/m1013_full.usda"
ROBOT_PRIM = "/World/m1013"

# ── World 생성 ──
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# ── M1013 USD 로드 ──
add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT_PRIM)
print(f"[bridge] USD 로드: {USD_PATH}")

for _ in range(5):
    app.update()

# ── ArticulationRoot 는 URDF 임포터가 root_joint 에 붙인 그대로 유지 ──
# (옮기면 fixed base 고정이 풀려 로봇이 중력에 쓰러짐)
articulation_root_path = None
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        articulation_root_path = str(prim.GetPath())
        print(f"[bridge] ArticulationRoot 위치 (유지): {articulation_root_path}")
        break

# base_link 자동 탐지
base_link_path = None
for prim in stage.Traverse():
    p = str(prim.GetPath())
    if p.endswith("/base_link") and p.startswith(ROBOT_PRIM):
        base_link_path = p
        break

if base_link_path is None:
    raise RuntimeError(f"base_link 를 {ROBOT_PRIM} 밑에서 찾을 수 없음")

print(f"[bridge] base_link 경로: {base_link_path}")

for _ in range(5):
    app.update()

# ── Articulation 로봇 객체 ──
from isaacsim.core.prims import SingleArticulation

# Articulation prim_path 는 ArticulationRootAPI 가 있는 prim (원래 root_joint)
robot = SingleArticulation(prim_path=articulation_root_path, name="robot")
world.scene.add(robot)

# ── 씬 보강: 지면 + 조명 + 카메라 ──
from pxr import UsdGeom, UsdLux, Gf, Sdf

# 지면 (ground plane)
from isaacsim.core.api.objects.ground_plane import GroundPlane
GroundPlane(prim_path="/World/GroundPlane", size=5.0)

# Distant light
light_prim = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
light_prim.CreateIntensityAttr(3000)

# Dome light (환경광)
dome_prim = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
dome_prim.CreateIntensityAttr(500)

# 카메라: 로봇을 정면에서 비스듬히 바라보도록 (LookAt 방식)
cam_path = "/World/PerspectiveCam"
cam = UsdGeom.Camera.Define(stage, Sdf.Path(cam_path))
cam_xform = UsdGeom.Xformable(cam)
cam_xform.ClearXformOpOrder()

# 카메라 위치 + 타겟 (로봇 중간 높이)
cam_pos = Gf.Vec3d(3.0, -3.0, 1.8)
target  = Gf.Vec3d(0.0, 0.0, 0.6)
up      = Gf.Vec3d(0.0, 0.0, 1.0)

# LookAt 행렬 계산 → translate + rotate
gf_mat = Gf.Matrix4d().SetLookAt(cam_pos, target, up).GetInverse()
cam_xform.AddTransformOp().Set(gf_mat)

# viewport 를 이 카메라로 전환
try:
    from omni.kit.viewport.utility import get_active_viewport
    vp = get_active_viewport()
    if vp:
        vp.camera_path = cam_path
        print(f"[bridge] viewport 카메라 설정: {cam_path}")
except Exception as e:
    print(f"[bridge] viewport 카메라 설정 실패: {e}")

world.reset()

print(f"[bridge] 관절 수: {robot.num_dof}")
print(f"[bridge] 관절명: {robot.dof_names}")

# ── ROS2 구독 (bundled rclpy, Python 3.11 호환) ──
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

rclpy.init()

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("isaac_bridge")
        self.latest = None
        qos = QoSProfile(depth=10)
        self.create_subscription(JointState, f"/{args.topic}", self._cb, qos)

    def _cb(self, msg):
        self.latest = msg

ros_node = JointStateSubscriber()
print(f"[bridge] ROS2 구독 시작: /{args.topic}")
print("[bridge] 시뮬레이션 루프 시작. Ctrl-C로 종료.")

world.play()

import numpy as np

# ── 초기 포즈: "서있는" 자세 (rad) ──
# 모든 관절 0 = URDF 캘리브레이션 자세 (수평으로 누움)
# 아래는 일반적인 Doosan "ready" 포즈
HOME_POSE = np.array([0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0])  # [rad]
robot.set_joint_positions(HOME_POSE)
for _ in range(10):
    world.step(render=True)
print(f"[bridge] 초기 포즈 적용: {HOME_POSE}")

try:
    while app.is_running():
        rclpy.spin_once(ros_node, timeout_sec=0)

        msg = ros_node.latest
        if msg is not None and len(msg.position) > 0:
            positions = np.array(msg.position)
            if len(positions) == robot.num_dof:
                # DRCF 미초기화 상태 (모든 관절 정확히 0) 는 무시 → HOME_POSE 유지
                if not np.allclose(positions, 0.0):
                    robot.set_joint_positions(positions)

        world.step(render=True)

except KeyboardInterrupt:
    print("[bridge] 종료 중...")
finally:
    rclpy.shutdown()
    app.close()
