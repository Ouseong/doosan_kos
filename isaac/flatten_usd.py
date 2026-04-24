"""
m1013.usd 의 모든 sublayer/reference 를 펼쳐서 단일 파일로 저장
→ 깨진 참조 문제 해결
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd

SRC = "/ros2_ws/src/doosan-robot2/dsr_description2/usd/m1013.usd"
DST = "/ros2_ws/src/doosan-robot2/dsr_description2/usd/m1013_flat.usda"

stage = Usd.Stage.Open(SRC)
print(f"[flat] Loaded: {SRC}")

# 모든 payload 로드
stage.Load()
print("[flat] Payloads loaded")

flat = stage.Flatten()
flat.Export(DST)
print(f"[flat] Exported flat USD: {DST}")

# 결과 검증
flat_stage = Usd.Stage.Open(DST)
mesh_count = sum(1 for p in flat_stage.Traverse() if p.GetTypeName() == "Mesh")
print(f"[flat] Mesh prims in flattened file: {mesh_count}")

app.close()
