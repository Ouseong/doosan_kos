"""
Isaac Sim 5.1.0 URDF → USD 변환 (v2)
- 표준 urdf/m1013.urdf 사용 (usd/ 하위 버전 대신)
- merge_fixed_joints=True 시도
- 새 output 경로 /tmp/m1013_v2/ 에 저장 (기존 파일 충돌 방지)

실행:
  docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/urdf_to_usd_v2.py
"""

import os

# package:// 경로 해석용 ROS2 env
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

URDF_PATH = "/ros2_ws/src/doosan-robot2/dsr_description2/urdf/m1013.urdf"
OUT_DIR   = "/tmp/m1013_v2"
USD_PATH  = f"{OUT_DIR}/m1013.usd"

os.makedirs(OUT_DIR, exist_ok=True)

print(f"[v2] URDF: {URDF_PATH}")
print(f"[v2] USD:  {USD_PATH}")

_, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints         = True    # ← 이번엔 True (world_fixed 병합)
import_config.convex_decomp              = False
import_config.import_inertia_tensor      = True
import_config.fix_base                   = True
import_config.make_default_prim          = True
import_config.self_collision             = False
import_config.create_physics_scene       = True
import_config.default_drive_type         = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_drive_strength     = 1e7
import_config.default_position_drive_damping = 1e5

result = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    dest_path=USD_PATH,
)
print(f"[v2] import result: {result}")

# 결과 검증
from pxr import Usd
stage = Usd.Stage.Open(USD_PATH)
if stage:
    stage.Load()
    mesh_cnt = sum(1 for p in stage.Traverse() if p.GetTypeName() == "Mesh")
    xform_cnt = sum(1 for p in stage.Traverse() if p.GetTypeName() == "Xform")
    print(f"[v2] ✓ Mesh={mesh_cnt}, Xform={xform_cnt}")
else:
    print(f"[v2] ✗ USD failed to open")

print("[v2] done")
app.close()
