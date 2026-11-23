"""
주물공장 작업구역 (절차적 생성, 외부 자산 없음). 공장 전체가 아니라 로봇·원료 보관대·용해로 주변 구역.

철골 기둥(H형강)·지붕 보·트러스 / 천장 주행 크레인 레일 + 거더 + 호이스트 / 금속 패널 벽 + 하부 콘크리트 /
고창(희미한 하늘빛) / 벽면 배관 / 대형 덕트 / 내화벽돌 구역 / 황색 안전선·난간 / 금속판 보관 랙 / 팔레트·드럼 /
사용감 있는 콘크리트 바닥.

물리: 바닥만 충돌. 나머지 배경 소품은 물리 없음 (SPEC §17).
좌표: 원점 = Mobile Base 바닥 투영 중심. 용해로 중심 = layout.furnace_xy.
"""

import os
import sys

import numpy as np
from pxr import UsdGeom, UsdPhysics

sys.path.insert(0, os.path.dirname(__file__))
import usd_util as U  # noqa: E402


def hbeam(st, path, length, axis, t, size=0.35, mat=None):
    """H 형강: 웹 + 플랜지 2 장. axis = 'z' | 'x' | 'y'."""
    U.xform(st, path, t=t)
    fl, web = 0.025, 0.018
    if axis == "z":
        U.box_mesh(st, path + "/web", (web, size, length), mat=mat)
        U.box_mesh(st, path + "/fl1", (size, fl, length), t=(0, size / 2, 0), mat=mat)
        U.box_mesh(st, path + "/fl2", (size, fl, length), t=(0, -size / 2, 0), mat=mat)
    elif axis == "x":
        U.box_mesh(st, path + "/web", (length, web, size), mat=mat)
        U.box_mesh(st, path + "/fl1", (length, size, fl), t=(0, 0, size / 2), mat=mat)
        U.box_mesh(st, path + "/fl2", (length, size, fl), t=(0, 0, -size / 2), mat=mat)
    else:
        U.box_mesh(st, path + "/web", (web, length, size), mat=mat)
        U.box_mesh(st, path + "/fl1", (size, length, fl), t=(0, 0, size / 2), mat=mat)
        U.box_mesh(st, path + "/fl2", (size, length, fl), t=(0, 0, -size / 2), mat=mat)


def build(cfg, out_path):
    lay = cfg["layout"]
    fx, fy = lay["furnace_xy"]
    x0, x1, y0, y1 = lay["hall_x"][0], lay["hall_x"][1], lay["hall_y"][0], lay["hall_y"][1]
    H = lay["hall_height"]
    st = U.new_stage(out_path, "Factory")
    W = "/Factory"

    concrete = U.material(st, "concrete_floor", texture=U.texture("concrete", "concrete", 1024, seed=1),
                          tex_scale=(1, 1), rough=0.85)
    panel = U.material(st, "metal_panel", texture=U.texture("metal_panel", "metal_panel", seed=2), rough=0.6, metal=0.5)
    wall_conc = U.material(st, "wall_concrete", texture=U.texture("concrete_wall", "concrete", seed=4), rough=0.9)
    brick = U.material(st, "fire_brick", texture=U.texture("brick", "brick", seed=6), rough=0.9)
    refr = U.material(st, "refractory_brick", texture=U.texture("refractory", "refractory", seed=3), rough=0.95)
    steel = U.material(st, "structural_steel", color=(0.16, 0.17, 0.17), rough=0.55, metal=0.6)
    rust = U.material(st, "rust", texture=U.texture("rust", "rust", seed=5), rough=0.85, metal=0.3)
    yellow = U.material(st, "safety_yellow", color=(0.8, 0.58, 0.04), rough=0.5)
    hazard = U.material(st, "hazard", texture=U.texture("hazard", "hazard", seed=0), rough=0.6)
    window = U.material(st, "window_sky", color=(0.3, 0.38, 0.5), emissive=(0.35, 0.45, 0.6), emissive_intensity=4)
    pipe_g = U.material(st, "pipe_green", color=(0.12, 0.25, 0.15), rough=0.5, metal=0.3)
    pipe_gr = U.material(st, "pipe_grey", color=(0.35, 0.36, 0.36), rough=0.4, metal=0.7)
    duct = U.material(st, "duct", color=(0.3, 0.31, 0.32), rough=0.45, metal=0.8)
    wood = U.material(st, "pallet_wood", color=(0.42, 0.31, 0.2), rough=0.9)
    drum = U.material(st, "drum_blue", color=(0.08, 0.16, 0.35), rough=0.5, metal=0.4)
    plate_m = U.material(st, "metal_plate_stock", color=(0.55, 0.56, 0.57), rough=0.45, metal=0.9)

    # 바닥 (유일한 충돌 배경)
    fl = U.box_mesh(st, W + "/floor", (x1 - x0, y1 - y0, 0.2), t=((x0 + x1) / 2, (y0 + y1) / 2, -0.1), mat=concrete,
                    uv_scale=0.25)
    UsdPhysics.CollisionAPI.Apply(fl.GetPrim())

    # 기둥·보·트러스
    S = W + "/Structure"
    U.xform(st, S)
    xs = np.arange(x0 + 1.0, x1, 6.0)
    for i, x in enumerate(xs):
        for side, y in (("N", y1 - 0.4), ("S", y0 + 0.4)):
            hbeam(st, S + f"/col_{side}{i}", H, "z", (x, y, H / 2), 0.4, steel)
        hbeam(st, S + f"/roofbeam{i}", y1 - y0, "y", (x, (y0 + y1) / 2, H + 0.3), 0.5, steel)
        for k in range(6):                                          # 트러스 사재
            yy = y0 + (k + 0.5) * (y1 - y0) / 6
            U.box_mesh(st, S + f"/truss{i}_{k}", (0.08, (y1 - y0) / 6 * 1.05, 0.08), t=(x, yy, H - 0.4),
                       rpy_deg=(28 * (1 if k % 2 else -1), 0, 0), mat=steel)
    for side, y in (("N", y1 - 0.4), ("S", y0 + 0.4)):              # 처마 보
        hbeam(st, S + f"/eave_{side}", x1 - x0, "x", ((x0 + x1) / 2, y, H), 0.4, steel)
    # 천장 크레인: 주행 레일 2 줄 + 거더 + 호이스트
    for side, y in (("N", y1 - 1.0), ("S", y0 + 1.0)):
        hbeam(st, S + f"/crane_rail_{side}", x1 - x0, "x", ((x0 + x1) / 2, y, H - 2.0), 0.45, steel)
        for i, x in enumerate(xs):
            U.box_mesh(st, S + f"/rail_bracket_{side}{i}", (0.3, 0.6, 0.3), t=(x, y + (0.3 if side == "N" else -0.3), H - 2.0), mat=steel)
    gx = fx - 3.5
    U.box_mesh(st, S + "/crane_girder", (0.7, y1 - y0 - 1.6, 0.9), t=(gx, (y0 + y1) / 2, H - 1.35), mat=yellow)
    U.box_mesh(st, S + "/crane_girder_stripe", (0.71, y1 - y0 - 1.6, 0.12), t=(gx, (y0 + y1) / 2, H - 1.75), mat=hazard, uv_scale=2)
    U.box_mesh(st, S + "/hoist", (0.9, 0.9, 0.7), t=(gx, -1.5, H - 2.2), mat=yellow)
    U.cylinder(st, S + "/hoist_cable", 0.02, 3.0, t=(gx, -1.5, H - 4.0), mat=steel)
    U.box_mesh(st, S + "/hook_block", (0.3, 0.2, 0.4), t=(gx, -1.5, H - 5.7), mat=yellow)

    # 벽: 하부 콘크리트 + 상부 금속 패널 + 고창
    Wl = W + "/Walls"
    U.xform(st, Wl)
    for nm, (cx, cy, sx, sy) in {"N": ((x0 + x1) / 2, y1, x1 - x0, 0.2), "S": ((x0 + x1) / 2, y0, x1 - x0, 0.2),
                                 "W": (x0, (y0 + y1) / 2, 0.2, y1 - y0), "E": (x1, (y0 + y1) / 2, 0.2, y1 - y0)}.items():
        U.box_mesh(st, Wl + f"/lower_{nm}", (sx, sy, 1.6), t=(cx, cy, 0.8), mat=wall_conc, uv_scale=0.4)
        U.box_mesh(st, Wl + f"/panel_{nm}", (sx, sy, H - 1.6), t=(cx, cy, 1.6 + (H - 1.6) / 2), mat=panel, uv_scale=0.25)
    for i, x in enumerate(np.arange(x0 + 2.5, x1 - 1, 6.0)):
        U.box_mesh(st, Wl + f"/window_N{i}", (3.0, 0.05, 1.0), t=(x, y1 - 0.12, H - 1.6), mat=window)
        U.box_mesh(st, Wl + f"/window_S{i}", (3.0, 0.05, 1.0), t=(x, y0 + 0.12, H - 1.6), mat=window)

    # 배관 (북쪽 벽) + 대형 덕트 (천장)
    P = W + "/Piping"
    U.xform(st, P)
    for i, (z, r, m) in enumerate(((3.2, 0.08, pipe_g), (3.55, 0.06, pipe_gr), (3.85, 0.1, pipe_gr), (2.6, 0.05, rust))):
        U.pipe_path(st, P + f"/wall_pipe{i}", [(x0 + 0.5, y1 - 0.5 - 0.2 * i, z), (x1 - 0.5, y1 - 0.5 - 0.2 * i, z)], r, mat=m)
    U.pipe_path(st, P + "/drop0", [(fx + 2.4, y1 - 0.5, 3.2), (fx + 2.4, 1.0, 3.2), (fx + 2.4, 1.0, 0.4)], 0.08, mat=pipe_g)
    U.pipe_path(st, P + "/main_duct", [(fx, 0.0, H - 2.9), (fx, 0.0, H - 2.4), (fx, y1 - 1.2, H - 2.4), (x1 - 0.6, y1 - 1.2, H - 2.4)],
                0.55, mat=duct)
    U.pipe_path(st, P + "/duct2", [(x0 + 0.6, y1 - 1.2, H - 2.4), (fx, y1 - 1.2, H - 2.4)], 0.45, mat=duct)

    # 내화벽돌 구역 (용해로 뒤) + 벽돌 팔레트
    B = W + "/RefractoryZone"
    U.xform(st, B)
    U.box_mesh(st, B + "/brick_wall", (0.5, 4.5, 1.8), t=(fx + 3.6, fy, 0.9), mat=brick, uv_scale=1.5)
    U.box_mesh(st, B + "/brick_wall_side", (2.5, 0.5, 1.2), t=(fx + 2.6, fy + 2.5, 0.6), mat=brick, uv_scale=1.5)
    for k in range(3):
        U.box_mesh(st, B + f"/pallet{k}", (1.0, 1.0, 0.14), t=(fx + 2.5, fy - 2.2 + 1.2 * k, 0.07), mat=wood)
        U.box_mesh(st, B + f"/bricks{k}", (0.9, 0.9, 0.5 + 0.15 * k), t=(fx + 2.5, fy - 2.2 + 1.2 * k, 0.14 + (0.5 + 0.15 * k) / 2),
                   mat=refr, uv_scale=2.0)

    # 안전선 (용해로 구역 둘레 + 통로)
    Y = W + "/Safety"
    U.xform(st, Y)
    hz = 2.2
    for k, (cx, cy, sx, sy) in enumerate(((fx, fy + hz, 2 * hz, 0.1), (fx, fy - hz, 2 * hz, 0.1),
                                          (fx + hz, fy, 0.1, 2 * hz), (fx - hz - 0.6, fy, 0.1, 2 * hz))):
        U.box_mesh(st, Y + f"/line{k}", (sx, sy, 0.004), t=(cx, cy, 0.002), mat=hazard, uv_scale=4)
    for k, y in enumerate((-3.4, 3.4)):
        U.box_mesh(st, Y + f"/walkway{k}", (x1 - x0 - 2, 0.12, 0.004), t=((x0 + x1) / 2, y, 0.002), mat=yellow)
    # 난간 (용해로 구역 좌우, 로봇 접근쪽은 개방)
    for side, y in (("N", fy + hz), ("S", fy - hz)):
        for k, x in enumerate(np.linspace(fx - hz, fx + hz, 6)):
            U.box_mesh(st, Y + f"/post_{side}{k}", (0.05, 0.05, 1.1), t=(x, y, 0.55), mat=yellow)
        for z in (0.5, 1.05):
            U.box_mesh(st, Y + f"/rail_{side}{int(z*100)}", (2 * hz, 0.04, 0.04), t=(fx, y, z), mat=yellow)

    # 금속판 보관 랙 (판을 세워 꽂는 슬롯형) — 로봇 오른쪽
    rx, ry = lay["rack_xy"]
    R_ = W + "/PlateRack"
    U.xform(st, R_, t=(rx, ry, 0))
    pl = cfg["plate"]["size_m"]
    U.box_mesh(st, R_ + "/base", (0.9, 0.5, 0.08), t=(0, 0, 0.04), mat=steel)
    for sx in (-1, 1):
        U.box_mesh(st, R_ + f"/end{sx}", (0.06, 0.5, lay["rack_height"]), t=(sx * 0.42, 0, lay["rack_height"] / 2), mat=yellow)
    U.box_mesh(st, R_ + "/table", (0.9, 0.5, 0.04), t=(0, 0, lay["rack_height"] - 0.02), mat=steel)
    for k in range(5):                                             # 슬롯 칸막이
        y = -0.2 + 0.1 * k
        U.box_mesh(st, R_ + f"/slot{k}", (0.8, 0.012, 0.12), t=(0, y, lay["rack_height"] + 0.06), mat=steel)
    for k in range(3):                                             # 꽂힌 재고 판 (배경, 물리 없음). 집을 판은 따로 생성
        y = -0.15 + 0.1 * (k + 1)
        U.box_mesh(st, R_ + f"/stock_plate{k}", (pl[0], pl[2], pl[1]), t=(0, y, lay["rack_height"] + pl[1] / 2 + 0.002),
                   mat=plate_m)

    # 기타 소품: 판 더미 팔레트, 드럼, 공구 카트
    M = W + "/Props"
    U.xform(st, M)
    for k in range(2):
        U.box_mesh(st, M + f"/stack_pallet{k}", (1.2, 1.0, 0.14), t=(-2.4, -2.5 + 1.4 * k, 0.07), mat=wood)
        for j in range(6 + 3 * k):
            U.box_mesh(st, M + f"/stack{k}_{j}", (pl[0], pl[1], pl[2]),
                       t=(-2.4 + 0.02 * ((j % 3) - 1), -2.5 + 1.4 * k + 0.02 * ((j % 2) - 0.5), 0.14 + pl[2] * (j + 0.5)),
                       rpy_deg=(0, 0, 3 * ((j % 3) - 1)), mat=plate_m)
    for k in range(4):
        U.cylinder(st, M + f"/drum{k}", 0.29, 0.88, t=(x0 + 1.2 + 0.65 * k, y1 - 1.6, 0.44), mat=drum if k % 2 else rust)
    U.box_mesh(st, M + "/cart", (0.8, 0.5, 0.05), t=(-1.6, 1.8, 0.85), mat=steel)
    for sx in (-1, 1):
        for sy in (-1, 1):
            U.box_mesh(st, M + f"/cart_leg{sx}{sy}", (0.04, 0.04, 0.85), t=(-1.6 + sx * 0.37, 1.8 + sy * 0.22, 0.425), mat=steel)
    U.box_mesh(st, M + "/control_desk", (1.4, 0.7, 1.0), t=(-3.5, 2.6, 0.5), mat=U.material(st, "desk", color=(0.35, 0.36, 0.33), rough=0.6))

    st.GetRootLayer().Save()
