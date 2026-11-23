"""
산업용 유도 용해로 (절차적 생성). 시각 형상과 충돌 형상을 분리한다.

시각: 내화재 라이닝(도가니) / 코일 하우징 강판 셸 + 리브 / 내화재 상부 테두리 + 출탕구 / 기울임 유압실린더 /
      철골 프레임·작업 발판 / 냉각수 배관(급·배수) / 전력 버스바 / 제어반 / 집진 후드 + 덕트 / 용탕(발광) + 조명
충돌: 도가니 벽 = convex 박스 N 조각 + 바닥 박스 (Phase 1 Newton 용기와 같은 치수). 단일 convex hull 금지.
     충돌 prim 은 purpose=guide 로 렌더에서 숨긴다.

좌표: 용해로 중심 바닥 = 로컬 원점, rim 높이 = furnace.rim_height_from_floor_m.
"""

import os
import sys

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics

sys.path.insert(0, os.path.dirname(__file__))
import usd_util as U  # noqa: E402


def build(cfg, out_path):
    f = cfg["furnace"]
    R = f["crucible_inner_d_m"] / 2
    D = f["crucible_depth_m"]
    rim = f["rim_height_from_floor_m"]
    melt_z = rim - D + f["melt_depth_m"]
    floor_z = rim - D
    T = f["wall_thickness_m"]
    st = U.new_stage(out_path, "Furnace")
    W = "/Furnace"

    refr = U.material(st, "refractory", texture=U.texture("refractory", "refractory", seed=3), tex_scale=(1, 1), rough=0.95)
    refr_hot = U.material(st, "refractory_hot", color=(0.55, 0.32, 0.18), rough=0.9, emissive=(1.0, 0.35, 0.08),
                          emissive_intensity=1.5)
    shell = U.material(st, "shell_paint", color=(0.13, 0.16, 0.18), rough=0.55, metal=0.4)
    rust = U.material(st, "rust", texture=U.texture("rust", "rust", seed=5), rough=0.85, metal=0.3)
    steel = U.material(st, "steel_frame", color=(0.22, 0.22, 0.2), rough=0.5, metal=0.7)
    yellow = U.material(st, "safety_yellow", color=(0.75, 0.55, 0.05), rough=0.5)
    copper = U.material(st, "copper", color=(0.72, 0.42, 0.25), rough=0.35, metal=1.0)
    blue_p = U.material(st, "pipe_blue", color=(0.08, 0.2, 0.45), rough=0.45, metal=0.3)
    red_p = U.material(st, "pipe_red", color=(0.45, 0.06, 0.04), rough=0.45, metal=0.3)
    cab = U.material(st, "cabinet", color=(0.42, 0.44, 0.42), rough=0.5, metal=0.3)
    dark = U.material(st, "dark", color=(0.03, 0.03, 0.03), rough=0.8)
    # 용탕: 발광 + 거친 산화 피막 느낌 (빨간 플라스틱처럼 보이지 않게 roughness 높이고 색을 황-주황으로)
    melt = U.material(st, "melt", color=(1.0, 0.35, 0.06), rough=0.75, metal=0.0, emissive=(1.0, 0.30, 0.04),
                      emissive_intensity=float(cfg["lighting"]["melt_emissive"]))
    crust = U.material(st, "melt_crust", color=(0.12, 0.05, 0.03), rough=0.95, emissive=(0.6, 0.12, 0.02),
                       emissive_intensity=0.4)
    lamp_g = U.material(st, "lamp_green", color=(0.1, 1, 0.2), emissive=(0.1, 1.0, 0.2), emissive_intensity=20)
    lamp_r = U.material(st, "lamp_red", color=(1, 0.1, 0.05), emissive=(1, 0.1, 0.05), emissive_intensity=20)

    V = W + "/Visual"
    U.xform(st, V)
    # 도가니 내화재 라이닝 + 바닥
    U.ring_mesh(st, V + "/lining", R, R + 0.10, floor_z, rim - 0.05, seg=96, mat=refr, uv_scale=1.5)
    U.cylinder(st, V + "/lining_floor", R + 0.10, 0.12, t=(0, 0, floor_z - 0.06), mat=refr)
    # 코일 하우징 (강판 셸) + 세로 리브 + 하부 받침
    U.ring_mesh(st, V + "/shell", R + 0.10, R + 0.24, 0.12, rim - 0.06, seg=96, mat=shell, uv_scale=1.0)
    for k in range(16):
        a = 2 * np.pi * k / 16
        U.box_mesh(st, V + f"/rib{k}", (0.05, 0.03, rim - 0.2), t=((R + 0.25) * np.cos(a), (R + 0.25) * np.sin(a), (rim - 0.06 + 0.12) / 2),
                   rpy_deg=(0, 0, np.degrees(a)), mat=rust)
    for zz in (0.25, 0.55, rim - 0.18):                          # 띠 보강
        U.ring_mesh(st, V + f"/band{int(zz*100)}", R + 0.24, R + 0.28, zz - 0.03, zz + 0.03, seg=96, mat=steel)
    U.box_mesh(st, V + "/plinth", (2 * R + 0.8, 2 * R + 0.8, 0.12), t=(0, 0, 0.06), mat=steel)
    # 내화재 상부 테두리 (뜨거운 안쪽 가장자리) + 출탕구
    U.ring_mesh(st, V + "/top_rim", R, R + 0.30, rim - 0.06, rim, seg=96, mat=refr, uv_scale=1.5)
    U.ring_mesh(st, V + "/top_rim_hot", R, R + 0.03, rim - 0.30, rim + 0.002, seg=96, mat=refr_hot)
    # 출탕구는 로봇 반대쪽(+y)에 둔다 (로봇은 -x 쪽에서 접근)
    U.box_mesh(st, V + "/spout", (0.16, 0.30, 0.08), t=(0, R + 0.28, rim - 0.03), mat=refr)
    U.box_mesh(st, V + "/spout_channel", (0.06, 0.30, 0.02), t=(0, R + 0.28, rim + 0.005), mat=refr_hot)
    # 용탕 표면 (발광 원판) + 표면 산화 찌꺼기 몇 조각
    U.cylinder(st, V + "/melt_surface", R - 0.005, 0.02, t=(0, 0, melt_z - 0.01), mat=melt)
    rng = np.random.default_rng(7)
    for k in range(9):
        rr, aa = R * 0.75 * np.sqrt(rng.random()), 2 * np.pi * rng.random()
        U.cylinder(st, V + f"/crust{k}", 0.02 + 0.04 * rng.random(), 0.006,
                   t=(rr * np.cos(aa), rr * np.sin(aa), melt_z + 0.002), mat=crust)
    # 기울임 유압실린더 (출탕 반대쪽 피벗)
    for s in (-1, 1):
        U.pipe_path(st, V + f"/hyd_cyl{'L' if s < 0 else 'R'}", [(R + 0.5, s * (R + 0.15), 0.12), (R + 0.32, s * (R + 0.15), 0.75)],
                    0.06, mat=yellow)
        U.pipe_path(st, V + f"/hyd_rod{'L' if s < 0 else 'R'}", [(R + 0.32, s * (R + 0.15), 0.75), (R + 0.26, s * (R + 0.15), 0.95)],
                    0.03, mat=steel)
        U.cylinder(st, V + f"/pivot{'L' if s < 0 else 'R'}", 0.08, 0.2, t=(-(R + 0.1), s * (R + 0.32), rim - 0.25),
                   axis="Y", mat=steel)
    # 철골 프레임 — 로봇이 -x 쪽에서 접근하므로 기둥은 뒤(+x) 두 개만 세우고 후드 보는 캔틸레버
    c = R + 0.75
    hood_z = rim + 1.35
    for sy in (-1, 1):
        U.box_mesh(st, V + f"/col_{sy}", (0.18, 0.18, hood_z + 0.1), t=(c, sy * c, (hood_z + 0.1) / 2), mat=steel)
        U.box_mesh(st, V + f"/brace_{sy}", (0.08, 0.08, 1.2), t=(c - 0.35, sy * c, hood_z - 0.4), rpy_deg=(0, -35, 0), mat=steel)
        U.box_mesh(st, V + f"/beam_y{sy}", (2 * c - 0.3, 0.14, 0.2), t=(0.15, sy * c, hood_z), mat=steel)
    U.box_mesh(st, V + "/beam_x", (0.14, 2 * c + 0.16, 0.2), t=(c, 0, hood_z), mat=steel)
    # 집진 후드 (사각 깔때기) + 덕트 (천장 방향)
    U.box_mesh(st, V + "/hood_lower", (1.7, 1.7, 0.08), t=(0, 0, hood_z + 0.12), mat=rust)
    U.box_mesh(st, V + "/hood_upper", (1.0, 1.0, 0.45), t=(0, 0, hood_z + 0.38), mat=rust)
    for k, (dx, dy, w, h) in enumerate(((0.62, 0, 0.3, 1.7), (-0.62, 0, 0.3, 1.7), (0, 0.62, 1.7, 0.3), (0, -0.62, 1.7, 0.3))):
        U.box_mesh(st, V + f"/hood_skirt{k}", (w if dy == 0 else w, h if dx == 0 else h, 0.3),
                   t=(dx, dy, hood_z + 0.3), rpy_deg=((20 if dy > 0 else -20) if dy else 0, (-20 if dx > 0 else 20) if dx else 0, 0), mat=rust)
    U.cylinder(st, V + "/duct_up", 0.32, 4.5, t=(0, 0, hood_z + 0.6 + 2.25), mat=steel)
    # 냉각수 배관 (급수 파랑 / 배수 빨강), 전력 버스바 (구리), 제어반
    for i, (mat, y0) in enumerate(((blue_p, 0.18), (red_p, 0.32))):
        U.pipe_path(st, V + f"/cool{i}", [(R + 0.28, y0, 0.35 + 0.15 * i), (R + 0.9, y0, 0.35 + 0.15 * i),
                                           (R + 0.9, y0, 0.05), (R + 2.2, y0, 0.05)], 0.035, mat=mat)
    for i in range(3):
        U.box_mesh(st, V + f"/busbar{i}", (1.6, 0.03, 0.12), t=(R + 1.05, -0.25 - 0.06 * i, 0.30), mat=copper)
    cx = R + 1.95
    U.box_mesh(st, V + "/cabinet", (0.6, 1.2, 2.0), t=(cx, -0.6, 1.0), mat=cab)
    U.box_mesh(st, V + "/cabinet_door_seam", (0.005, 0.01, 1.9), t=(cx - 0.302, -0.6, 1.0), mat=dark)
    for i, m in enumerate((lamp_g, lamp_r, lamp_g)):
        s = UsdGeom.Sphere.Define(st, V + f"/lamp{i}")
        s.CreateRadiusAttr(0.025)
        U._set_xf(s, (cx - 0.31, -0.9 + 0.12 * i, 1.75), (0, 0, 0))
        U.bind(s, m)
    U.box_mesh(st, V + "/hmi", (0.01, 0.35, 0.25), t=(cx - 0.305, -0.4, 1.45),
               mat=U.material(st, "hmi", color=(0.05, 0.12, 0.2), emissive=(0.2, 0.5, 0.9), emissive_intensity=3))

    # ── 조명 (용탕 빛이 판·그리퍼 하부·로봇·바닥에 실제로 비치도록 실제 광원으로) ──────────
    L = W + "/Lights"
    U.xform(st, L)
    U.disk_light(st, L + "/melt_glow", R * 0.95, cfg["lighting"]["melt_disk_intensity"], (1.0, 0.45, 0.12),
                 t=(0, 0, melt_z + 0.03), rpy_deg=(0, 0, 0))            # DiskLight 는 -Z 로 비춘다 → 아래로
    lt = U.disk_light(st, L + "/melt_glow_up", R * 0.95, cfg["lighting"]["melt_up_intensity"], (1.0, 0.45, 0.12),
                      t=(0, 0, rim + 0.01), rpy_deg=(180, 0, 0))        # 개구부에서 위로 (판·그리퍼 하부·로봇)
    if cfg["lighting"]["melt_sphere_intensity"] > 0:
        U.sphere_light(st, L + "/rim_glow", 0.25, cfg["lighting"]["melt_sphere_intensity"], (1.0, 0.5, 0.15),
                       t=(0, 0, melt_z + 0.15))

    # ── 충돌 (Newton 용기와 같은 치수, 렌더 숨김) ────────────────────────────────────
    C = W + "/Collision"
    cp = UsdGeom.Xform.Define(st, C)
    UsdGeom.Imageable(cp).CreatePurposeAttr("guide")
    n = f["wall_segments"]
    half_tan = np.pi * (R + T) / n * 1.08
    for k in range(n):
        a = 2 * np.pi * k / n
        b = UsdGeom.Cube.Define(st, C + f"/wall{k}")
        b.CreateSizeAttr(1.0)
        U._set_xf(b, ((R + T / 2) * np.cos(a), (R + T / 2) * np.sin(a), floor_z + D / 2), (0, 0, np.degrees(a)),
                  s=(T, 2 * half_tan, D + T))
        UsdPhysics.CollisionAPI.Apply(b.GetPrim())
    fl = UsdGeom.Cube.Define(st, C + "/floor")
    fl.CreateSizeAttr(1.0)
    U._set_xf(fl, (0, 0, floor_z - T / 2), (0, 0, 0), s=(2 * (R + T), 2 * (R + T), T))
    UsdPhysics.CollisionAPI.Apply(fl.GetPrim())
    # 외곽(하우징·프레임) 은 로봇 충돌 확인용 단순 박스
    for nm, size, t in (("housing", (2 * R + 0.6, 2 * R + 0.6, rim), (0, 0, rim / 2 - 0.0)),):
        pass   # 하우징 박스는 도가니 내부를 막으므로 넣지 않는다. 벽 조각이 하우징 역할

    st.GetRootLayer().customLayerData = {"furnace": {"R": R, "depth": D, "rim_z": rim, "melt_z": melt_z,
                                                     "floor_z": floor_z, "source": "procedural, assumed dims"}}
    st.GetRootLayer().Save()
    return {"R": R, "rim_z": rim, "melt_z": melt_z, "floor_z": floor_z, "hood_z": hood_z, "frame_half": c}
