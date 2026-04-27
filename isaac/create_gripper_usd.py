"""
그리퍼 물리 USD 생성 스크립트 (Isaac Sim 없이 실행 가능)

출력: usd/parts/gripper_assembly_physics.usd
  - ArticulationRoot: /World/Gripper
  - RigidBody: body / left_plate / right_plate
  - PrismaticJoint: left_joint / right_joint (X축 슬라이딩)
  - 좌표계: Z-up, 단위 meters (STL은 mm → 0.001 스케일)
  - 기준: 플랜지 면 = 원점 (gripper_*_urdf.stl 기준)

관절 규약:
  joint position = 0      → 완전 열림 (현재 STL 위치)
  joint position = 0.0335 → 완전 닫힘 (플레이트가 중심 방향으로 33.5mm 이동)
"""

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Vt, Sdf
import open3d as o3d
import numpy as np

# ── 경로 설정 ──────────────────────────────────────────────────────────────
BASE      = 'C:/Users/oskwo/Desktop/doosan_kos/usd/parts/'
USD_OUT   = BASE + 'gripper_assembly_physics.usd'
STL_BODY  = BASE + 'gripper_body_urdf.stl'
STL_LEFT  = BASE + 'gripper_left_urdf.stl'
STL_RIGHT = BASE + 'gripper_right_urdf.stl'

SCALE = 0.001  # mm → m

# 질량 (추정값 — 알루미늄 기준 나중에 측정 후 교체)
MASS_BODY  = 0.8   # kg
MASS_PLATE = 0.3   # kg

# 관절 위치 (m, 플랜지 원점 기준)
LEFT_JOINT_POS  = Gf.Vec3f(-0.0445, -0.128, -0.0002)
RIGHT_JOINT_POS = Gf.Vec3f( 0.0445, -0.128, -0.0017)
MAX_TRAVEL      = 0.0335  # m (33.5mm)


# ── 메쉬 로드 헬퍼 ─────────────────────────────────────────────────────────
def load_stl(path, decimate=None):
    mesh = o3d.io.read_triangle_mesh(path)
    if decimate:
        mesh = mesh.simplify_quadric_decimation(decimate)
    mesh.compute_vertex_normals()
    verts = np.array(mesh.vertices) * SCALE   # mm → m
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
    return mesh


# ── 스테이지 생성 ──────────────────────────────────────────────────────────
stage = Usd.Stage.CreateNew(USD_OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
stage.SetMetadata('metersPerUnit', 1.0)

world  = UsdGeom.Xform.Define(stage, '/World')
gripper = UsdGeom.Xform.Define(stage, '/World/Gripper')
UsdPhysics.ArticulationRootAPI.Apply(gripper.GetPrim())

# ── 메쉬 로드 ──────────────────────────────────────────────────────────────
print('메쉬 로딩...')
body_v,  body_f  = load_stl(STL_BODY)
left_v,  left_f  = load_stl(STL_LEFT)
right_v, right_f = load_stl(STL_RIGHT)
body_vc, body_fc   = load_stl(STL_BODY,  decimate=3000)   # 충돌용 단순화
left_vc, left_fc   = load_stl(STL_LEFT,  decimate=3000)
right_vc, right_fc = load_stl(STL_RIGHT, decimate=3000)
print(f'  body:  {len(body_f)} / 충돌: {len(body_fc)} faces')
print(f'  left:  {len(left_f)} / 충돌: {len(left_fc)} faces')
print(f'  right: {len(right_f)} / 충돌: {len(right_fc)} faces')


# ── 링크 생성 함수 ──────────────────────────────────────────────────────────
def make_link(prim_path, verts, faces, coll_v, coll_f, mass, com):
    xform = UsdGeom.Xform.Define(stage, prim_path)
    rb = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr(mass)
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))

    add_mesh_prim(stage, prim_path + '/visual',    verts, faces, collision=False)
    add_mesh_prim(stage, prim_path + '/collision', coll_v, coll_f, collision=True)
    return xform


body_com  = (float(body_v[:,0].mean()),  float(body_v[:,1].mean()),  float(body_v[:,2].mean()))
left_com  = (float(left_v[:,0].mean()),  float(left_v[:,1].mean()),  float(left_v[:,2].mean()))
right_com = (float(right_v[:,0].mean()), float(right_v[:,1].mean()), float(right_v[:,2].mean()))

body_link  = make_link('/World/Gripper/body',        body_v,  body_f,  body_vc,  body_fc,  MASS_BODY,  body_com)
left_link  = make_link('/World/Gripper/left_plate',  left_v,  left_f,  left_vc,  left_fc,  MASS_PLATE, left_com)
right_link = make_link('/World/Gripper/right_plate', right_v, right_f, right_vc, right_fc, MASS_PLATE, right_com)


# ── Fixed joint: body를 World에 고정 ───────────────────────────────────────
fixed = UsdPhysics.FixedJoint.Define(stage, '/World/Gripper/body_world_joint')
fixed.CreateBody1Rel().SetTargets([Sdf.Path('/World/Gripper/body')])


# ── Prismatic joint 생성 함수 ──────────────────────────────────────────────
def make_prismatic_joint(prim_path, body0_path, body1_path, joint_pos, direction):
    """
    direction: 1.0 = +X 방향으로 닫힘 (left),  -1.0 = -X 방향으로 닫힘 (right)
    joint 0 = 완전 열림, joint MAX_TRAVEL = 완전 닫힘
    """
    joint = UsdPhysics.PrismaticJoint.Define(stage, prim_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])

    # 관절 프레임: X축 방향
    joint.CreateAxisAttr('X')

    # 관절 기준 위치 (body 로컬 프레임 = body Xform이 world origin이므로 world 위치 그대로)
    joint.CreateLocalPos0Attr(joint_pos)
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalPos1Attr(joint_pos)
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    # 관절 범위: 열림(0) ~ 닫힘(MAX_TRAVEL)
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(MAX_TRAVEL * direction)

    # Drive (위치 제어)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear')
    drive.CreateTypeAttr('position')
    drive.CreateStiffnessAttr(1000.0)   # N/m
    drive.CreateDampingAttr(100.0)      # N·s/m
    drive.CreateMaxForceAttr(50.0)      # N

    return joint


make_prismatic_joint(
    '/World/Gripper/left_joint',
    '/World/Gripper/body',
    '/World/Gripper/left_plate',
    LEFT_JOINT_POS,
    direction=1.0    # +X 방향으로 닫힘
)

make_prismatic_joint(
    '/World/Gripper/right_joint',
    '/World/Gripper/body',
    '/World/Gripper/right_plate',
    RIGHT_JOINT_POS,
    direction=-1.0   # -X 방향으로 닫힘
)

stage.GetRootLayer().Save()
print(f'\n그리퍼 물리 USD 저장 완료: {USD_OUT}')
print('관절 규약:')
print('  joint = 0.000  → 완전 열림 (67mm)')
print('  joint = 0.0335 → 완전 닫힘 (0mm)')
