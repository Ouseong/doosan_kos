"""
Isaac Sim 5.1.0 URDF → USD 변환 스크립트
M1013 URDF를 USD로 변환

실행:
  docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/urdf_to_usd.py
"""

import os

# package:// 경로 해석을 위해 ROS2 환경 주입 (importer 가 ament_index 참조)
os.environ["AMENT_PREFIX_PATH"] = "/opt/ros/jazzy:/ros2_ws/install/dsr_description2"
os.environ["ROS_PACKAGE_PATH"] = "/ros2_ws/install/dsr_description2/share"

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import omni
import omni.kit.commands
from omni.isaac.core.utils.extensions import enable_extension

enable_extension("isaacsim.asset.importer.urdf")
app.update()

from isaacsim.asset.importer.urdf import _urdf

URDF_PATH = "/tmp/m1013_abs.urdf"
USD_PATH  = "/ros2_ws/src/doosan-robot2/dsr_description2/usd/m1013.usd"

print(f"[urdf_to_usd] URDF: {URDF_PATH}")
print(f"[urdf_to_usd] USD:  {USD_PATH}")

_, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints         = False
import_config.convex_decomp              = False
import_config.import_inertia_tensor      = True
import_config.fix_base                   = True
import_config.make_default_prim          = True
import_config.self_collision             = False
import_config.create_physics_scene       = True
import_config.default_drive_type         = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_drive_strength     = 1e7
import_config.default_position_drive_damping = 1e5

omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    dest_path=USD_PATH,
)

print("[urdf_to_usd] 변환 완료!")
app.close()
