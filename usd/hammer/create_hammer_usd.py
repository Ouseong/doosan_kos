"""
Hammer 물리 USD 생성 스크립트 (Isaac Sim 외부에서 실행)

출력: usd/hammer/hammer_physics.usd
  - DefaultPrim: /World/Hammer  (그리퍼 USD 의 referencing 함정 회피)
  - RigidBody: /World/Hammer
  - Visual mesh: /World/Hammer/visual   (원본 444 tri)
  - Collision mesh: /World/Hammer/collision  (그대로 — 이미 저폴리)
  - 좌표계: Z-up, meters (STL mm → 0.001)

규약:
  hammer_link 원점(0,0,0) = 그리퍼 grip origin
  strike 면 중심        = (0, -0.087, -0.027) m, r=0.015
  Y 축 = 손잡이 길이 방향 (Y- 가 머리, Y+ 가 손잡이 끝)
"""
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Vt, Sdf
import open3d as o3d
import numpy as np
import os

# ── 경로 (Windows / Docker / 로컬 자동 감지) ───────────────────────────────
if os.path.exists('/kos_workspace'):
    BASE = '/kos_workspace/usd/hammer/'
else:
    BASE = os.path.normpath(os.path.join(os.path.dirname(__file__))) + os.sep

USD_OUT = os.path.join(BASE, 'hammer_physics.usd')
STL_IN  = os.path.join(BASE, 'hammer.stl')

SCALE = 0.001  # mm → m

# 질량/관성 (rubber mallet 일반치, 실측 시 교체)
MASS_HAMMER = 0.5   # kg
# bounding box 0.0335 × 0.179 × 0.054 m 균질 근사
IXX = 0.001457
IYY = 0.000168
IZZ = 0.001381
COM = (0.0, -0.014, 0.0)   # bounding box 중앙


# ── 메쉬 로드 ──────────────────────────────────────────────────────────────
def load_stl(path):
    mesh = o3d.io.read_triangle_mesh(path)
    mesh.compute_vertex_normals()
    verts = np.array(mesh.vertices) * SCALE
    faces = np.array(mesh.triangles)
    return verts, faces


def add_mesh_prim(stage, prim_path, verts, faces, collision=False):
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*v) for v in verts]))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(faces.flatten().tolist()))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(faces)))
    mesh.GetSubdivisionSchemeAttr().Set('none')
    if collision:
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        # 임팩트 시 안정적인 contact 위해 convex decomposition 권장
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        mesh_api.CreateApproximationAttr('convexDecomposition')
    return mesh


# ── 스테이지 생성 ──────────────────────────────────────────────────────────
print('메쉬 로딩...')
verts, faces = load_stl(STL_IN)
print(f'  {len(faces)} faces, bbox(m): '
      f'{verts.min(0).round(4)} → {verts.max(0).round(4)}')

stage = Usd.Stage.CreateNew(USD_OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
stage.SetMetadata('metersPerUnit', 1.0)

UsdGeom.Xform.Define(stage, '/World')
hammer = UsdGeom.Xform.Define(stage, '/World/Hammer')
stage.SetDefaultPrim(hammer.GetPrim())

# RigidBody + Mass
rb = UsdPhysics.RigidBodyAPI.Apply(hammer.GetPrim())
mass_api = UsdPhysics.MassAPI.Apply(hammer.GetPrim())
mass_api.CreateMassAttr(MASS_HAMMER)
mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*COM))
mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(IXX, IYY, IZZ))

# Visual + Collision
add_mesh_prim(stage, '/World/Hammer/visual',    verts, faces, collision=False)
add_mesh_prim(stage, '/World/Hammer/collision', verts, faces, collision=True)

# Strike frame — 머리 아래쪽 면 중앙, +Z 가 타격 normal (hammer -Z) 을 가리킴
strike = UsdGeom.Xform.Define(stage, '/World/Hammer/strike_frame')
strike.AddTranslateOp().Set(Gf.Vec3f(0.0, -0.087, -0.027))
strike.AddRotateXOp().Set(180.0)            # +Z(local) → -Z(hammer)
# 시각 확인용 작은 빨간 sphere (반경 15mm 의 strike disc 표시)
disc = UsdGeom.Sphere.Define(stage, '/World/Hammer/strike_frame/marker')
disc.GetRadiusAttr().Set(0.005)
disc.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.13, 0.13)]))
UsdGeom.Imageable(disc).CreatePurposeAttr('guide')   # 시뮬에서 숨김, 뷰포트 guide

stage.GetRootLayer().Save()
print(f'\nHammer 물리 USD 저장: {USD_OUT}')
print(f'  mass = {MASS_HAMMER} kg, COM = {COM}')
print(f'  diag inertia = ({IXX}, {IYY}, {IZZ})')
print('  defaultPrim = /World/Hammer  → AddReference(asset, "/World/Hammer") 로 임포트')
