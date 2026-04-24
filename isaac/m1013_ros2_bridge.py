"""
Isaac Sim 5.1.0 standalone app
M1013 USD 로드 + ROS2 OmniGraph 브리지
✅ x86_64 + aarch64 (DGX Spark) 호환

실행: /isaac-sim/python.sh isaac/m1013_ros2_bridge.py --topic dsr01/joint_states
"""

import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--topic", default="dsr01/joint_states", help="구독할 joint_states 토픽")
parser.add_argument("--headless", action="store_true", help="헤드리스 모드 (Spark 원격 사용 시)")
args, unknown = parser.parse_known_args()

# ── Isaac Sim 앱 부팅 ──
from isaacsim import SimulationApp

CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": args.headless,
    "width": 1280,
    "height": 720,
}
app = SimulationApp(CONFIG)

import omni
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from pxr import UsdPhysics, Usd
import carb

# ── ROS2 브리지 익스텐션 활성화 ──
# 5.1.0에서는 isaacsim.ros2.bridge 사용
enable_extension("isaacsim.ros2.bridge")
app.update()

# ── USD 경로 ──
USD_PATH = "/ros2_ws/src/doosan-robot2/dsr_description2/usd/m1013.usd"

# ── World 생성 ──
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# ── M1013 USD 로드 ──
root_prim = stage.DefinePrim("/World/m1013", "Xform")
root_prim.GetReferences().AddReference(USD_PATH)
print(f"[bridge] referencing USD: {USD_PATH}")

app.update()

# ── ArticulationRootAPI 패치 ──
# 원래 USD에서 root_joint(fixed joint)에 잘못 적용된 API를
# base_link로 이동시켜 PhysX가 올바른 아티큘레이션을 인식하도록 수정
wrong_prim = stage.GetPrimAtPath("/World/m1013/m1013/root_joint")
if wrong_prim and wrong_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
    wrong_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    print("[bridge] removed ArticulationRootAPI from /World/m1013/m1013/root_joint")

base_link = stage.GetPrimAtPath("/World/m1013/m1013/base_link")
if base_link:
    UsdPhysics.ArticulationRootAPI.Apply(base_link)
    print("[bridge] applied ArticulationRootAPI to /World/m1013/m1013/base_link")

ROBOT_PATH = "/World/m1013/m1013/base_link"
print(f"[bridge] articulation robotPath: {ROBOT_PATH}")

# ── OmniGraph 구성 ──
# OnPlaybackTick → ROS2SubscribeJointState → IsaacArticulationController
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick",      "omni.graph.action.OnPlaybackTick"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationCtrl",    "isaacsim.core.nodes.IsaacArticulationController"),
            ("PublishJointState",   "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("PublishClock",        "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("SubscribeJointState.inputs:topicName",        args.topic),
            ("PublishJointState.inputs:topicName",          "isaac_joint_states"),
            ("ArticulationCtrl.inputs:robotPath",           ROBOT_PATH),
            ("ArticulationCtrl.inputs:usePath",             True),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick",                 "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",                 "ArticulationCtrl.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",                 "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",                 "PublishClock.inputs:execIn"),
            ("SubscribeJointState.outputs:jointNames",      "ArticulationCtrl.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationCtrl.inputs:positionCommand"),
        ],
    },
)

print(f"[bridge] graph ready. Subscribing to '/{args.topic}', echoing on '/isaac_joint_states'")
print("[bridge] entering main loop. Ctrl-C to quit.")

world.play()

while app.is_running():
    world.step(render=True)

app.close()
