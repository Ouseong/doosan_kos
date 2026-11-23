"""
Release 제어 (PLAN Phase 6). 목표 투입 자세(판 기준)를 파지 추정값으로 보정해 그리퍼 목표 자세를 계산한다.

  조건 A  고정 투입: 파지 오차 무시 (명목 파지 가정)
  조건 B  핀 변위 추정 보정: grasp_estimator 결과 (관측 불가 성분 = 사전값 0)
  조건 C  oracle: 판 ground truth 파지 오차 사용 (상한 기준, 평가 전용)
  조건 D  낮은 높이에서 조심스럽게 놓기: A 와 같되 release 높이를 낮춤 (단순 baseline)

순서: 접근 → 감속 → jaw 열기. 여기서는 release 순간의 그리퍼 자세와 속도를 계산하고, 판의 실제 release
자세 = 그리퍼 명령 자세 ∘ 실제 파지(T_SP(e_true)) 로 둔다 (jaw 가 열리는 동안 판에 주는 교란은 모델링하지 않음 — 한계).
"""

from __future__ import annotations

import numpy as np

import grasp_geometry as G


def target_plate_pose(center, pitch_deg=0.0, roll_deg=0.0, yaw_deg=0.0):
    """목표 판 자세 (world). 기본 = 판을 세운 상태: P.x(법선) = -world x, P.y(긴 변) = world y, P.z = world z.
    pitch: P.y(긴 변) 축 회전 → 판면이 수직에서 기울어짐 (90° = 수평 판). roll: P.x(법선) 축 회전 → 긴 변이 기울어
    모서리부터 진입. yaw: world z 축 회전."""
    R0 = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], float)      # 열 = P.x, P.y, P.z  (det=+1)
    R = G.rot("z", np.deg2rad(yaw_deg)) @ R0 @ G.rot("y", np.deg2rad(pitch_deg)) @ G.rot("x", np.deg2rad(roll_deg))
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = center
    return T


def command(T_WP_target, plate_size, insert_depth, err_used):
    """그리퍼 S 의 world 명령 자세."""
    T_SP = G.plate_in_S(plate_size, insert_depth, err_used)
    return T_WP_target @ np.linalg.inv(T_SP)


def released_plate(T_WS_cmd, plate_size, insert_depth, err_true):
    return T_WS_cmd @ G.plate_in_S(plate_size, insert_depth, err_true)


def pose_error(T_a, T_b):
    dp = float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    R = T_a[:3, :3].T @ T_b[:3, :3]
    ang = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    return dp, ang


def quat_xyzw(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()        # x, y, z, w
