"""
핀어레이 그리퍼 – 금속판 파지 기하 (순수 numpy). PLAN §2.

좌표: S = Sim_Gripper 프레임 (usd/sim_gripper/meta.json): 장착면 Z=0, 손가락 -Z, 판은 ±X 로 열림.
  - G.x = S.x = jaw 가 닫히는 방향 = 판 법선, G.z 손가락 방향 = -S.z, G.y = S.y
  - 각 jaw 의 5×5 핀 (피치 12 mm). 핀 중심 (y, z) = (15.67 + 12 i, -128.4 + 12 k) mm, i,k ∈ {-2..2}
    (plate_{p,n}_pins.stl bbox 와 PLAN 의 5×5·12 mm 로부터. 중심 z 는 meta.json grasp_center 와 일치)
  - 핀 끝 x = ±shift (판이 shift 만큼 벌어진 위치). 핀은 바깥쪽(±X)으로만 눌려 들어간다. 행정 0–stroke.
판(plate) 프레임 P: 원점 = 판 중심, P.x = 두께(법선), P.y = 긴 변, P.z = 짧은 변. 파지 가장자리 = P.z = +W/2 면.
명목 파지: P 를 S 와 같은 방향으로 두고, 가장자리가 손가락 끝 쪽 핀 열에서 insert_depth 만큼 들어온 위치.

이 모듈의 핀 변위 모델은 "접촉 깊이 기하 모델" 이다 (강체 판, 핀 = 축 방향 선 + 단면 ±4.5 mm 겹침 판정). 물리 시뮬레이션
(Phase 4 의 prismatic 핀) 과 별개이며 Phase 5 합성 데이터와 추정기 검증에 쓴다.
"""

from __future__ import annotations

import numpy as np

PITCH = 0.012
PIN_Y = 0.01567 + PITCH * np.arange(-2, 3)              # m, S 프레임
PIN_Z = -0.1284 + PITCH * np.arange(-2, 3)
PIN_HALF = 0.0045                                       # 핀 단면 반폭 (9 mm)
PIN_YZ = np.array([(y, z) for z in PIN_Z for y in PIN_Y])   # (25,2), 행 = z(손가락 끝→몸체), 열 = y
MAX_OPENING = 0.067
PLATE_TRAVEL = 0.030
PLATE_OFFSET = 0.001
MOUNT_SPACER = 0.035


def rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def shift_from_opening(opening):
    """명령 열림(0–0.067 m) → 판 하나의 X 이동 (브리지 opening_to_plate_shift 와 같은 규칙)."""
    return float(np.clip(opening, 0, MAX_OPENING)) / MAX_OPENING * PLATE_TRAVEL + PLATE_OFFSET


def nominal_plate_in_S(plate_size, insert_depth):
    L, W, t = plate_size
    z_low = PIN_Z[0] - PIN_HALF                         # 손가락 끝 쪽 핀 열의 바깥 경계
    z_edge = z_low + insert_depth
    T = np.eye(4)
    T[:3, 3] = (0.0, PIN_Y[2], z_edge - W / 2)
    return T


def offset_T(dx=0.0, dy=0.0, dz=0.0, rx=0.0, ry=0.0, rz=0.0):
    """파지 오차 (판 프레임 기준이 아니라 S 축 기준, 회전 중심 = 가장자리 중앙의 핀 배열 중심). 반환: 4×4 (S 에서 왼쪽 곱)."""
    T = np.eye(4)
    T[:3, :3] = rot("z", rz) @ rot("y", ry) @ rot("x", rx)
    T[:3, 3] = (dx, dy, dz)
    return T


def plate_in_S(plate_size, insert_depth, err=None):
    """err = dict(dx,dy,dz,rx,ry,rz). 핀 배열 중심(0, PIN_Y[2], PIN_Z[2]) 을 축으로 회전."""
    Tn = nominal_plate_in_S(plate_size, insert_depth)
    if not err:
        return Tn
    c = np.eye(4); c[:3, 3] = (0.0, PIN_Y[2], PIN_Z[2])
    ci = np.eye(4); ci[:3, 3] = -c[:3, 3]
    return c @ offset_T(**err) @ ci @ Tn


def pin_displacements(T_SP, plate_size, opening, stroke=0.012):
    """각 핀 축선(x 방향, (y,z) 고정)이 판과 만나면 판 면까지 눌린 깊이. 반환 (dp(25), dn(25), contact_p, contact_n).
    +X jaw 핀 끝은 x=+shift, 판의 +면 x 가 shift 보다 크면(판이 핀 끝을 넘어오면) 그만큼 눌린다."""
    L, W, t = plate_size
    s = shift_from_opening(opening)
    R, p = T_SP[:3, :3], T_SP[:3, 3]
    n = R[:, 0]                                          # 판 법선 (S)
    dp, dn = np.zeros(25), np.zeros(25)
    cp, cn = np.zeros(25, bool), np.zeros(25, bool)
    if abs(n[0]) < 1e-6:
        return dp, dn, cp, cn
    for i, (y, z) in enumerate(PIN_YZ):
        for sign, out, cont in ((+1, dp, cp), (-1, dn, cn)):
            # 판의 sign 쪽 면: n·(X - (p + sign*t/2*n)) = 0, X=(x,y,z) → x
            c = p + sign * (t / 2) * n * np.sign(n[0])
            x_face = c[0] - (n[1] * (y - c[1]) + n[2] * (z - c[2])) / n[0]
            X = np.array([x_face, y, z])
            q = R.T @ (X - p)                            # 판 프레임 좌표
            # 핀 단면(±PIN_HALF)이 판과 조금이라도 겹치면 접촉 (실제 핀·Phase 4 물리 핀과 같은 규칙)
            inside = abs(q[1]) <= L / 2 + PIN_HALF and abs(q[2]) <= W / 2 + PIN_HALF
            if not inside:
                continue
            depth = sign * x_face - s                    # 핀 끝 너머로 판 면이 들어온 양
            if depth > 0:
                out[i] = min(depth, stroke)
                cont[i] = True
    return dp, dn, cp, cn


def gripper_S_from_link6(T_W6):
    """link_6 world → S(world). 브리지와 같은 장착: 스페이서 35 mm 후 X 축 180°."""
    M = np.eye(4); M[:3, :3] = rot("x", np.pi); M[:3, 3] = (0, 0, MOUNT_SPACER)
    return T_W6 @ M


def link6_from_gripper_S(T_WS):
    M = np.eye(4); M[:3, :3] = rot("x", np.pi); M[:3, 3] = (0, 0, MOUNT_SPACER)
    return T_WS @ np.linalg.inv(M)


def S_for_plate_target(T_WP_target, T_SP):
    """판 목표 world 자세 + 판의 S 내 자세 → S world 자세."""
    return T_WP_target @ np.linalg.inv(T_SP)
