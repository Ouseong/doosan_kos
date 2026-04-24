"""
m1013 v2 를 완전 평탄화 — 모든 reference/sublayer 를 단일 stage 로 흡수
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd, UsdUtils

SRC = "/tmp/m1013_v2/m1013.usd"
DST = "/tmp/m1013_v2/m1013_single.usda"

# 모든 dependency 찾기
deps = UsdUtils.ComputeAllDependencies(SRC)
print(f"[flat] dependencies: {len(deps[0])} files")
for d in deps[0]:
    print(f"  - {d}")

# open + load payloads
stage = Usd.Stage.Open(SRC)
stage.Load()

# Flatten
flat = stage.Flatten()
flat.Export(DST)
print(f"[flat] exported: {DST}")

# 검증
fs = Usd.Stage.Open(DST)
fs.Load()
types = {}
for p in fs.Traverse():
    t = p.GetTypeName()
    types[t] = types.get(t, 0) + 1
print("[flat] resulting types:")
for t, n in sorted(types.items(), key=lambda x: -x[1]):
    print(f"  {t}: {n}")

app.close()
