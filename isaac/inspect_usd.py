"""
USD 구조 확인 스크립트 - prim 경로 출력
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd

USD_PATH   = "/ros2_ws/src/doosan-robot2/dsr_description2/usd/m1013.usd"
ROBOT_PRIM = "/World/m1013"

add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT_PRIM)
for _ in range(3):
    app.update()

stage = omni.usd.get_context().get_stage()
print("\n=== USD 프림 구조 ===")
for prim in stage.Traverse():
    path = str(prim.GetPath())
    apis = prim.GetAppliedSchemas()
    api_str = f"  [{', '.join(apis)}]" if apis else ""
    print(f"  {path}{api_str}")

app.close()
