#!/usr/bin/env python3
"""
Cubic Hermite spline swing trajectory — iLQR 없는 단순 직접 생성기.

Pipeline:
  Stage 1: q1 = 임팩트 자세 IK (J4 lock=0; 나머지 free; R_strike_target + NAIL_HEAD)
  Stage 2: q0 = q1 에서 J2/J3/J5 만 hand-tuned offset (백스윙 자세)
  Stage 3: q(t) = cubic Hermite spline, q̇(0) = q̇(T) = 0 (정지 시작/끝)
                  → swing arc 가 자연스럽게 단조 감소 보장

Why cubic 만 쓰는가:
  - iLQR 의 cost trade-off + IK 분기 모호성을 제거
  - q0, q1 둘 다 직접 결정 → 그리퍼 자세, 책상 충돌, swing 형태 모두 직접 제어
  - 사람의 망치질 (단조 감소 z) 자연스럽게 만들어짐
  - vel peak = 1.5 × 평균 vel → swing time 만 적절히 잡으면 한계 안

산출:
  output/swing_traj.csv  (rows × 6, deg)
"""

import os
import numpy as np
import pinocchio as pin

URDF = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
OUT_CSV = "/home/kos/Desktop/Code/doosan_kos/output/swing_traj.csv"

NAIL_HEAD = np.array([-0.65, -0.45, 0.10])
T_TOTAL = 1.0   # swing time (s) — 사람 망치질에 가까운 속도
N = 100         # number of intervals (101 frames, dt=10ms)

# Backswing offset: q1 에서 q0 가는 방향.
# 검증 결과 (FK perturbation): 모두 NEGATIVE offset 이 hammer_strike z 를 위로 올림.
#   J2 -30° → Δz +0.40    (어깨 들어올림)
#   J3 -30° → Δz +0.30    (팔꿈치 펴기)
#   J5 -30° → Δz +0.27    (손목 앞으로)
# 합산 ~ +0.4m 위 backswing 기대.  J1/J4/J6 락 (swing 평면 유지).
# 핵심 통찰: J2/J3/J5 모두 한 방향 회전이 hammer z 와 grip +Y z 를 동시에
# 영향 → 한 효과 늘리려면 다른 효과도 늘어남.  해결 = J 부호 반대 조합으로
# strike z 위 효과 합치고 grip +Y 효과 상쇄.
#
# Per-J FK 분석 (q1 baseline, ±30°):
#   J2: Δstrike_z=∓0.40, Δgrip_y_z=±0.50
#   J3: Δstrike_z=∓0.14, Δgrip_y_z=±0.50
#   J5: Δstrike_z=∓0.17, Δgrip_y_z=±0.50
# Linear combination 해: ΔJ2=-38, ΔJ3=+20, ΔJ5=+18
#   → Δstrike_z ≈ +0.29 (위 30cm),  Δgrip_y_z ≈ 0 (그리퍼 자세 유지)
#   → J3=134.87 + 20 = 154.87° (한계 ±155° 안)
BACKSWING_OFFSET_DEG = {
    1: -80.0,    # J2 — 어깨 크게 들어올림 (가장 큰 변동, 모터 한계 100% T=1.0s)
    2: +20.0,    # J3 — 팔꿈치 굽힘 (J3 한계 ±155° 직전)
    4: +60.0,    # J5 — 손목 큰 회전 (그리퍼 보정 + 추가 swing)
}


def solve_impact_ik(model, data, n_iter=800, step=0.2, tol=5e-4):
    """J4 lock=0;  J1, J2, J3, J5, J6 free.  R_target + NAIL_HEAD 6D 도달.

    R_target: strike face down (+Z = world -Z), handle horizontal back (= -ê_h).
    """
    sid = model.getFrameId("hammer_strike")

    eh = np.array([NAIL_HEAD[0], NAIL_HEAD[1]])
    eh = eh / np.linalg.norm(eh)
    R_target = np.array([
        [-eh[1], -eh[0], 0.0],
        [ eh[0], -eh[1], 0.0],
        [ 0.0,    0.0,   1.0],
    ]) @ np.diag([1.0, -1.0, -1.0])
    M_target = pin.SE3(R_target, NAIL_HEAD)

    free = [0, 1, 2, 4, 5]   # J1, J2, J3, J5, J6
    q = np.zeros(model.nq)
    init = {0: -150.0, 1: -30.0, 2: 80.0, 4: 30.0, 5: -98.0}
    for j, v in init.items():
        q[j] = np.deg2rad(v)

    for k in range(n_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        M_err = M_target.actInv(data.oMf[sid])
        e = pin.log(M_err).vector
        if np.linalg.norm(e) < tol:
            break
        J = pin.computeFrameJacobian(model, data, q, sid, pin.LOCAL)[:, free]
        dq = -J.T @ np.linalg.solve(J @ J.T + 1e-2 * np.eye(6), e)
        for i, j in enumerate(free):
            q[j] += step * dq[i]

    # Normalize J1, J6 to [-180, 180] for short-way movej
    q[0] = (q[0] + np.pi) % (2 * np.pi) - np.pi
    q[5] = (q[5] + np.pi) % (2 * np.pi) - np.pi

    pos_err = np.linalg.norm(e[:3]) * 1000
    ori_err = np.rad2deg(np.linalg.norm(e[3:]))
    print(f"[ik]   |pos|={pos_err:.2f}mm, |ori|={ori_err:.2f}°")
    return q


def cubic_hermite_spline(q0, q1, T_total, dt, v_end=None):
    """Cubic Hermite spline with q̇(0)=0 and q̇(T)=v_end.

    v_end=None → q̇(T)=0 (예전 동작, 끝 정지)
    v_end=array → 끝점 속도 강제 → 임팩트 momentum 보장 (망치질!)
    """
    n_steps = int(round(T_total / dt)) + 1
    qs = np.zeros((n_steps, len(q0)))
    if v_end is None:
        v_end = np.zeros(len(q0))
    for k in range(n_steps):
        u = (k * dt) / T_total
        h00 = 2*u**3 - 3*u**2 + 1.0
        h01 = -2*u**3 + 3*u**2
        h11 = u**3 - u**2
        qs[k] = h00 * q0 + h01 * q1 + h11 * T_total * v_end
    return qs


def main():
    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()

    print(f"\n=== Stage 1: 임팩트 자세 IK ===")
    q1 = solve_impact_ik(model, data)
    print(f"q1 (deg) = {np.rad2deg(q1).round(2).tolist()}")
    pin.forwardKinematics(model, data, q1)
    pin.updateFramePlacements(model, data)
    sid = model.getFrameId("hammer_strike")
    gid = model.getFrameId("gripper_full_link")
    p1 = data.oMf[sid].translation
    n1 = data.oMf[sid].rotation @ [0, 0, 1]
    gy1 = data.oMf[gid].rotation @ [0, 1, 0]
    print(f"   hammer_strike p   = {p1.round(4)}    (목표 {NAIL_HEAD})")
    print(f"   strike normal     = {n1.round(3)}    (목표 [0,0,-1])")
    print(f"   gripper +Y world  = {gy1.round(3)}    (z>0 = 위 향함)")

    print(f"\n=== Stage 2: 백스윙 자세 (q1 + offset) ===")
    q0 = q1.copy()
    for j, off_deg in BACKSWING_OFFSET_DEG.items():
        q0[j] += np.deg2rad(off_deg)
    print(f"q0 (deg) = {np.rad2deg(q0).round(2).tolist()}")
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)
    p0 = data.oMf[sid].translation
    gy0 = data.oMf[gid].rotation @ [0, 1, 0]
    print(f"   hammer_strike p   = {p0.round(4)}    (z={p0[2]:.3f}, nail 위 +{(p0[2]-0.10)*100:.0f}cm)")
    print(f"   gripper +Y world  = {gy0.round(3)}    (z>0 = 위 향함)")

    print(f"\n=== Stage 3: Cubic spline (T={T_TOTAL}s, {N+1} frames) ===")
    dt = T_TOTAL / N
    qs = cubic_hermite_spline(q0, q1, T_TOTAL, dt)

    # Verification
    print(f"\n=== Verification ===")
    zs = np.zeros(len(qs))
    flips = 0
    for k in range(len(qs)):
        pin.forwardKinematics(model, data, qs[k])
        pin.updateFramePlacements(model, data)
        zs[k] = data.oMf[sid].translation[2]
        if (data.oMf[gid].rotation @ [0, 1, 0])[2] < 0:
            flips += 1
    diffs = np.diff(zs)
    print(f"hammer_strike z: 시작 {zs[0]:.3f} → 끝 {zs[-1]:.3f}    (max {zs.max():.3f}, min {zs.min():.3f})")
    asc = np.sum(diffs > 0)
    desc = np.sum(diffs < 0)
    monotonic = '✓ 단조 감소 (사람 망치질)' if asc == 0 else f'⚠ 올라가는 frame {asc} 개'
    print(f"  내려가는 {desc}, 올라가는 {asc}    {monotonic}")
    print(f"그리퍼 +Y 아래 향한 frame: {flips}/{len(qs)}    {'✓' if flips == 0 else '⚠'}")

    # Velocity check
    v_deg = np.rad2deg(np.diff(qs, axis=0)) / dt
    limits = [120, 120, 180, 225, 225, 225]
    print(f"Velocity peak (M1013 한계 대비):")
    all_ok = True
    for j in range(6):
        vp = np.max(np.abs(v_deg[:, j]))
        pct = 100 * vp / limits[j]
        flag = '✓' if vp <= limits[j] else '⚠'
        if vp > limits[j]:
            all_ok = False
        print(f"  J{j+1}  peak={vp:6.1f}°/s  ({pct:5.0f}% limit) {flag}")

    # Link z check
    print(f"각 link world z 최저값 (책상 표면 0.05):")
    fids = {n: model.getFrameId(n) for n in
            ['link_3', 'link_4', 'link_5', 'link_6', 'gripper_full_link', 'hammer_link', 'hammer_strike']}
    for n, fid in fids.items():
        z_min = 1e6
        for q in qs:
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            z_min = min(z_min, data.oMf[fid].translation[2])
        flag = '✓' if z_min >= 0.05 else f'⚠ {(0.05-z_min)*100:.1f}cm 통과'
        print(f"  {n:22s}  z_min = {z_min:+.3f}    {flag}")

    # Save CSV
    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    np.savetxt(OUT_CSV, np.rad2deg(qs), delimiter=',', fmt='%.4f',
               header=f'J1,J2,J3,J4,J5,J6  (deg, cubic spline, dt={dt:.4f}s, rows={len(qs)})',
               comments='')
    print(f"\nSaved → {OUT_CSV}")


if __name__ == "__main__":
    main()
