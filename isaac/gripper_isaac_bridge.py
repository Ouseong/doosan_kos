"""
그리퍼 Isaac Sim 브릿지

Subscribe: /gripper_command  (control_msgs/GripperCommand)
  position = 0.067 → 완전 열림
  position = 0.0   → 완전 닫힘
  position = 0~0.067 → 중간 위치
  max_effort > 0 → 향후 토크 제한 (현재 미구현)

Publish:   /gripper_state   (sensor_msgs/JointState)
  name: ['gripper_left_joint', 'gripper_right_joint']
  position: 각 플레이트 열림량 (m)

실행:
  docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/gripper_isaac_bridge.py
  docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/gripper_isaac_bridge.py --headless
"""

import argparse
import os
import sys
import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default="/kos_workspace/usd/parts/gripper_assembly_physics.usd")
args, unknown = parser.parse_known_args()

# ── Isaac Sim 앱 부팅 ──────────────────────────────────────────────────────
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "renderer": "RayTracedLighting",
        "headless": args.headless,
        "width":  1280,
        "height":  720,
    },
    experience="/isaac-sim/apps/isaacsim.exp.base.kit",
)

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics, UsdGeom, UsdLux, Gf, Sdf

# ROS2 브릿지 확장 활성화
enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    app.update()

BUNDLED_RCLPY = "/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy"
if BUNDLED_RCLPY not in sys.path:
    sys.path.insert(0, BUNDLED_RCLPY)

# ── World 생성 + USD 로드 ───────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

add_reference_to_stage(usd_path=args.usd, prim_path="/World/Gripper")
print(f"[gripper] USD 로드: {args.usd}")

for _ in range(5):
    app.update()

# ── 조명 ──────────────────────────────────────────────────────────────────
UsdLux.DistantLight.Define(stage, "/World/DistantLight").CreateIntensityAttr(3000)
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(500)

# ── ArticulationRoot 탐지 ──────────────────────────────────────────────────
articulation_root_path = None
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        articulation_root_path = str(prim.GetPath())
        print(f"[gripper] ArticulationRoot: {articulation_root_path}")
        break

if articulation_root_path is None:
    raise RuntimeError("ArticulationRootAPI를 찾을 수 없습니다. USD를 확인하세요.")

# ── Articulation 객체 ──────────────────────────────────────────────────────
from isaacsim.core.prims import SingleArticulation

gripper_art = SingleArticulation(prim_path=articulation_root_path, name="gripper")
world.scene.add(gripper_art)
world.reset()

print(f"[gripper] 관절 수:  {gripper_art.num_dof}")
print(f"[gripper] 관절명:   {gripper_art.dof_names}")

MAX_OPENING = 0.067   # m (67mm)
MAX_TRAVEL  = 0.0335  # m (33.5mm per plate)


def opening_to_joint(opening_m: float) -> float:
    """목표 열림 너비(m) → 각 플레이트 관절 변위(m)"""
    opening_m = float(np.clip(opening_m, 0.0, MAX_OPENING))
    return (MAX_OPENING - opening_m) / 2.0


# ── ROS2 ──────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from control_msgs.msg import GripperCommand
from sensor_msgs.msg import JointState

rclpy.init()


class GripperBridgeNode(Node):
    def __init__(self):
        super().__init__("gripper_isaac_bridge")
        qos = QoSProfile(depth=10)

        self.create_subscription(GripperCommand, "/gripper_command", self._cmd_cb, qos)
        self.state_pub = self.create_publisher(JointState, "/gripper_state", qos)

        self.target_opening = MAX_OPENING   # 시작 시 완전 열림
        self.get_logger().info("그리퍼 브릿지 시작. /gripper_command 대기 중...")

    def _cmd_cb(self, msg: GripperCommand):
        self.target_opening = msg.position
        joint_pos = opening_to_joint(msg.position)
        self.get_logger().info(
            f"명령 수신: opening={msg.position*1000:.1f}mm → joint={joint_pos*1000:.1f}mm"
        )

    def publish_state(self, dof_positions):
        """현재 관절 위치 → /gripper_state 발행"""
        # left_joint, right_joint 순서로 존재한다고 가정
        if len(dof_positions) >= 2:
            left_pos  = float(dof_positions[0])
            right_pos = float(dof_positions[1])
        elif len(dof_positions) == 1:
            left_pos = right_pos = float(dof_positions[0])
        else:
            return

        current_opening = MAX_OPENING - (left_pos + right_pos)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['gripper_left_joint', 'gripper_right_joint']
        js.position = [left_pos, right_pos]
        self.state_pub.publish(js)


ros_node = GripperBridgeNode()
print("[gripper] 시뮬레이션 루프 시작. Ctrl-C로 종료.")

world.play()

try:
    while app.is_running():
        rclpy.spin_once(ros_node, timeout_sec=0)

        # 목표 관절 위치 설정
        joint_target = opening_to_joint(ros_node.target_opening)
        target_positions = np.array([joint_target, joint_target])

        if gripper_art.num_dof >= 2:
            gripper_art.set_joint_positions(target_positions)
        elif gripper_art.num_dof == 1:
            gripper_art.set_joint_positions(np.array([joint_target]))

        # 현재 상태 발행
        current_pos = gripper_art.get_joint_positions()
        ros_node.publish_state(current_pos)

        world.step(render=True)

except KeyboardInterrupt:
    print("[gripper] 종료 중...")
finally:
    rclpy.shutdown()
    app.close()
