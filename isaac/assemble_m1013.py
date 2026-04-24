"""
M1013 강제 조립: base.usd (메시) + physics.usd (관절) 수동 병합
→ 단일 self-contained m1013_full.usda 생성
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Sdf

BASE    = "/tmp/m1013_v2/configuration/m1013_base.usd"
PHYSICS = "/tmp/m1013_v2/configuration/m1013_physics.usd"
ROBOT   = "/tmp/m1013_v2/configuration/m1013_robot.usd"
OUT     = "/tmp/m1013_v2/m1013_full.usda"

# ── 새 stage 생성 + base 를 sublayer 로 추가 ──
dst = Usd.Stage.CreateNew(OUT)
dst.SetDefaultPrim(dst.DefinePrim("/m1013", "Xform"))
UsdGeom.SetStageUpAxis(dst, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(dst, 1.0)

# 방식 변경: sublayers 로 base + physics 를 둘 다 얹어서 /m1013, /visuals, /meshes 등 전부 가져옴
root_layer = dst.GetRootLayer()
root_layer.subLayerPaths.append(BASE)
root_layer.subLayerPaths.append(PHYSICS)

# 저장 후 재로드
dst.GetRootLayer().Save()
print(f"[assemble] saved: {OUT}")

# 검증
check = Usd.Stage.Open(OUT)
check.Load()
types = {}
for p in check.Traverse():
    t = p.GetTypeName()
    types[t] = types.get(t, 0) + 1
print("[assemble] resulting types:")
for t, n in sorted(types.items(), key=lambda x: -x[1]):
    print(f"  {t}: {n}")

app.close()
