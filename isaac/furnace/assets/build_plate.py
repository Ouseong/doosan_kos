"""
metal_plate (재질 미확정). 치수·밀도는 config (plate.size_m, plate.density_candidates[density_index]).
판 프레임 P: 원점 = 중심, P.x = 두께(10 mm), P.y = 긴 변(500), P.z = 짧은 변(300). grasp_geometry.py 와 같다.
물리: RigidBody + Collision(박스) + MassAPI(density). 같은 박스를 시각 형상으로도 쓴다.
"""

import os
import sys

from pxr import Gf, UsdGeom, UsdPhysics

sys.path.insert(0, os.path.dirname(__file__))
import usd_util as U  # noqa: E402


def build(cfg, out_path, density=None):
    pl = cfg["plate"]
    L, W, t = pl["size_m"]
    rho = density if density is not None else pl["density_candidates"][pl["density_index"]]
    st = U.new_stage(out_path, "metal_plate")
    root = st.GetPrimAtPath("/metal_plate")
    m = U.material(st, "metal_plate", color=(0.50, 0.50, 0.52), rough=0.5, metal=0.6, scope="/metal_plate/Looks")
    b = U.box_mesh(st, "/metal_plate/geom", (t, L, W), mat=m, uv_scale=1.0)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateDensityAttr(float(rho))
    UsdPhysics.CollisionAPI.Apply(b.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(b.GetPrim()).CreateApproximationAttr("boundingCube")
    st.GetRootLayer().customLayerData = {"name": "metal_plate", "material": "UNDECIDED",
                                         "density_kg_m3": float(rho), "density_note": "candidate value, not a material choice",
                                         "size_m": f"{L} x {W} x {t}"}
    st.GetRootLayer().Save()
    return {"mass_kg": rho * L * W * t}
