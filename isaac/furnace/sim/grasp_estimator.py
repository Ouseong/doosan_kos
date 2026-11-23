"""
핀 변위 기반 파지 자세 추정 (PLAN Phase 5, SPEC §12). 해석/기하 방법. AI/RL 없음.

입력 (관측만): 핀 변위 dp(25), dn(25) [m], jaw 이동량 shift_p, shift_n [m] (판 하나가 중심에서 벌어진 양),
              (관절 상태는 그리퍼 기준 추정에는 필요 없다. world 변환할 때만 쓴다)
금지 입력: 판 ground truth pose. 이 함수는 판 pose 를 인자로 받지 않는다.

출력 (그리퍼 S 프레임, 명목 파지 대비 오차, grasp_geometry.offset 규약과 같음):
  dx  판 중심면의 x 오프셋 (jaw 사이 치우침)     — 관측 가능, 연속
  ry  판 법선의 S.y 축 회전 (행 방향 기울기)       — 관측 가능 (눌린 행이 2 개 이상일 때)
  rz  판 법선의 S.z 축 회전 (열 방향 기울기)       — 관측 가능
  dz  삽입 깊이 오차                              — 관측 가능하지만 핀 피치(12 mm) 단위로 양자화
  rx  가장자리 각도 (S.x 축 회전)                  — 거칠게 (열마다 경계 행이 다를 때만)
  dy  가장자리 따라 미끄러짐                       — 관측 불가 (가장자리가 배열보다 길어 패턴 불변) → 사전값 0
  thickness  판 두께 (검증용)
"""

from __future__ import annotations

import numpy as np

from grasp_geometry import PIN_HALF, PIN_Y, PIN_YZ, PIN_Z, PITCH

CONTACT_EPS = 2e-4      # 이보다 작은 변위는 비접촉 [m]


def estimate(dp, dn, shift_p, shift_n, insert_depth_nominal, plate_thickness=None, noise_std=1e-4):
    dp, dn = np.asarray(dp, float), np.asarray(dn, float)
    cp, cn = dp > CONTACT_EPS, dn > CONTACT_EPS
    both = cp & cn
    out = {"observable": {"dx": False, "ry": False, "rz": False, "dz": False, "rx": False, "dy": False},
           "dy": 0.0, "dy_note": "unobservable: plate edge longer than pin array; prior (commanded) value used"}
    if both.sum() < 3:
        out.update({"ok": False, "reason": "fewer than 3 pins in contact on both jaws"})
        return out
    # 판 면 위치: +면 x = shift_p + dp, -면 x = -(shift_n + dn)
    yz = PIN_YZ
    xp = shift_p + dp
    xn = -(shift_n + dn)
    xm = 0.5 * (xp + xn)
    th = xp - xn
    y0, z0 = PIN_Y[2], PIN_Z[2]
    A = np.c_[np.ones(both.sum()), yz[both, 0] - y0, yz[both, 1] - z0]
    rows_used = np.unique(yz[both, 1])
    cols_used = np.unique(yz[both, 0])
    if len(rows_used) >= 2 and len(cols_used) >= 2:
        coef, res, rank, _ = np.linalg.lstsq(A, xm[both], rcond=None)
        c, a, b = coef
        cov = noise_std ** 2 / 2 * np.linalg.inv(A.T @ A)          # 중심면 = 두 면 평균 → 분산 /2
        out["observable"].update({"dx": True, "ry": True, "rz": True})
    else:
        c, a, b = xm[both].mean(), 0.0, 0.0
        cov = np.diag([noise_std ** 2 / 2 / both.sum(), np.inf, np.inf])
        out["observable"]["dx"] = True
    out["dx"] = float(c)
    out["rz"] = float(-np.arctan(a))
    out["ry"] = float(np.arctan(b))
    out["sigma"] = {"dx": float(np.sqrt(cov[0, 0])), "rz": float(np.sqrt(cov[1, 1])), "ry": float(np.sqrt(cov[2, 2]))}
    out["thickness"] = float(np.median(th[both]))
    # 삽입 깊이: 열마다 "가장 위(몸체 쪽)의 접촉 행" 과 그 위 비접촉 행 사이에 가장자리가 있다
    contact = (cp | cn).reshape(5, 5)                  # [행 k(z 오름차순), 열 i(y)]
    z_edges = []
    for i in range(5):
        col = contact[:, i]
        if not col.any():
            continue
        k = np.max(np.where(col)[0])
        # 핀 단면이 겹치면 접촉 → 가장자리 ∈ (z_k - h, z_{k+1} - h)
        lo = PIN_Z[k] - PIN_HALF
        hi = PIN_Z[k + 1] - PIN_HALF if k + 1 < 5 else np.nan
        z_edges.append((PIN_Y[i], lo, hi))
    z_low = PIN_Z[0] - PIN_HALF
    if z_edges:
        ze = np.array(z_edges)
        mids = np.where(np.isnan(ze[:, 2]), np.nan, 0.5 * (ze[:, 1] + ze[:, 2]))
        if np.all(np.isnan(mids)):
            out["dz"] = float(PIN_Z[-1] - z_low - insert_depth_nominal)
            out["dz_note"] = "all rows in contact: insertion depth only a lower bound"
            out["sigma"]["dz"] = np.inf
        else:
            z_edge = np.nanmean(mids)
            out["dz"] = float(z_edge - z_low - insert_depth_nominal)
            out["sigma"]["dz"] = PITCH / np.sqrt(12)      # 양자화 오차 (균일분포)
            out["observable"]["dz"] = True
        # 가장자리 각도: 열마다 경계 행이 다르면 기울기 추정 (거침)
        ok = ~np.isnan(mids)
        # 합성 평가(Phase 5)에서 경계 행 기울기로 구한 rx 는 사전값(0)보다 오차가 컸다 (RMSE 3.5° vs 1.7°,
        # 최대 15.8°) → 관측 불가로 두고 거친 값만 참고로 남긴다
        out["rx"] = 0.0
        out["rx_coarse"] = float(np.arctan(np.polyfit(ze[ok, 0], mids[ok], 1)[0])) if ok.sum() >= 2 else 0.0
        out["rx_note"] = "treated as unobservable (12 mm pitch over 48 mm span); prior 0 used"
        out["sigma"]["rx"] = float(np.arctan(PITCH / (PIN_Y[-1] - PIN_Y[0])))
    # 판 중심면이 핀 배열 중심에서 x = c 이다. 파라미터 dx 는 회전 중심 기준 이동이므로
    # c = dx - a*dy - b*dz (기울어진 판을 면 안에서 옮기면 c 가 변함). dy 는 관측 불가 → 사전값 0 으로 둔다.
    # 남는 오차 a*dy_true 는 관측 불가 성분의 영향이며 sigma 에 넣지 않고 note 로 남긴다.
    out["plane_offset_at_pin_center"] = float(c)
    out["dx"] = float(c + b * out.get("dz", 0.0))
    out["dx_note"] = "dx couples with unobservable dy by tan(rz)*dy"
    out["ok"] = True
    if plate_thickness is not None:
        out["thickness_error"] = out["thickness"] - plate_thickness
    return out


def as_err(est):
    """추정 결과 → grasp_geometry.plate_in_S(err=...) 형식. 관측 불가 성분은 0 (사전값)."""
    return {k: (est.get(k, 0.0) if est["observable"].get(k, False) else 0.0) for k in ("dx", "dy", "dz", "rx", "ry", "rz")}
