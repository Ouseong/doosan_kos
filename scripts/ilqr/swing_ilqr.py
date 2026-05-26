#!/usr/bin/env python3
"""
swing_optimal.py — Optimal hammering trajectory with iLQR (Crocoddyl FDDP).

설계:
  Stage 1: q1 = 임팩트 자세 IK (J4 lock=0; J1/J2/J3/J5/J6 free; R_target + nail)
  Stage 2: q0 = q1 + BACKSWING_OFFSET (직접 hand-tune, 그리퍼 안 뒤집힘 보장)
  Stage 3: cubic spline q_ref(t) — iLQR warm start (분기 모호성 회피)
  Stage 4: iLQR (FDDP):
              terminal: pos + orientation + STRONG velocity (impact momentum)
              running: state reg (J1/J4/J6 락, cubic ref 가이드), barriers

핵심 차이점 vs swing_cubic.py:
  - cubic 은 끝점 q̇=0 강제 → 임팩트 vel 0 (타격력 X)
  - iLQR 은 terminal velocity cost 로 끝점 q̇ 큰 값 강제 → 임팩트 vel 큼 (진짜 망치질)

산출:
  output/swing_traj.csv (rows × 6, deg)
"""

import os
import numpy as np
import pinocchio as pin
import crocoddyl

URDF = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
OUT_CSV = "/home/kos/Desktop/Code/doosan_kos/output/swing_ilqr.csv"

NAIL_HEAD = np.array([-0.65, -0.45, 0.10])

# Backswing offset (q1 + offset = q0).  swing_cubic.py 와 동일한 grip-cancellation.
BACKSWING_OFFSET_DEG = {
    1: -80.0,    # J2 — 어깨 들기
    2: +20.0,    # J3 — 팔꿈치 굽힘 (한계 안)
    4: +60.0,    # J5 — 손목 (그리퍼 보정)
}

# OCP horizon
DT = 0.01
T  = 100   # 1.0s swing

# Terminal cost weights
W_POS         = 5.0e3    # impact position
W_ROT         = 5.0e2    # impact orientation
W_VEL         = 1.0e3    # impact velocity ← 강하게! momentum 최대화
V_DES_DOWN    = 3.0      # m/s, 임팩트 시 -z 방향 목표 속도

# Running cost weights
W_LOCK_JOINT  = 1.0e5    # J1/J4/J6 변동 (사실상 hard)
W_FREE_JOINT  = 1.0e-3   # J2/J3/J5 자유
W_VEL_REG     = 1.0e-1   # q̇ 정규화
W_TAU         = 1.0e-3   # 토크 정규화
W_BOUNDS      = 1.0e4    # joint position barrier (J3)
W_VEL_BARRIER = 5.0e3    # joint velocity barrier
W_LINK_Z      = 5.0e5    # link world z 책상 barrier
W_REF_TRACK   = 1.0e1    # cubic reference tracking (작게, momentum cost 가 우세하도록)

# M1013 한계
JOINT_POS_LIMIT_DEG = np.array([355.0, 355.0, 155.0, 355.0, 355.0, 355.0])
JOINT_VEL_LIMIT_DEG = np.array([108.0, 108.0, 162.0, 200.0, 200.0, 200.0])  # 90% 마진
LINK_Z_MIN_ARM = 0.10   # link_3/4/5 origin: 책상 위 5cm
LINK_Z_MIN_TIP = 0.05   # gripper, hammer: 책상 표면 (임팩트 시 nail head=0.10)


def solve_impact_ik(model, data, n_iter=800, step=0.2, tol=5e-4):
    """J4 lock=0; J1, J2, J3, J5, J6 free.  R_target + NAIL_HEAD."""
    sid = model.getFrameId("hammer_strike")
    eh = np.array([NAIL_HEAD[0], NAIL_HEAD[1]])
    eh = eh / np.linalg.norm(eh)
    R_target = np.array([
        [-eh[1], -eh[0], 0.0],
        [ eh[0], -eh[1], 0.0],
        [ 0.0,    0.0,   1.0],
    ]) @ np.diag([1.0, -1.0, -1.0])
    M_target = pin.SE3(R_target, NAIL_HEAD)

    free = [0, 1, 2, 4, 5]
    q = np.zeros(model.nq)
    init = {0: -150, 1: -30, 2: 80, 4: 30, 5: -98}
    for j, v in init.items():
        q[j] = np.deg2rad(v)
    for k in range(n_iter):
        pin.forwardKinematics(model, data, q); pin.updateFramePlacements(model, data)
        M_err = M_target.actInv(data.oMf[sid])
        e = pin.log(M_err).vector
        if np.linalg.norm(e) < tol: break
        J = pin.computeFrameJacobian(model, data, q, sid, pin.LOCAL)[:, free]
        dq = -J.T @ np.linalg.solve(J @ J.T + 1e-2 * np.eye(6), e)
        for i, j in enumerate(free): q[j] += step * dq[i]
    q[0] = (q[0] + np.pi) % (2 * np.pi) - np.pi
    q[5] = (q[5] + np.pi) % (2 * np.pi) - np.pi
    return q, R_target


def cubic_hermite_warm(q0, q1, T_total, dt):
    """Cubic Hermite with q̇=0 at both ends — only used as iLQR warm start."""
    n_steps = int(round(T_total / dt)) + 1
    qs = np.zeros((n_steps, len(q0)))
    qdots = np.zeros((n_steps, len(q0)))
    for k in range(n_steps):
        u = (k * dt) / T_total
        h00 = 2*u**3 - 3*u**2 + 1.0
        h01 = -2*u**3 + 3*u**2
        dh00 = (6*u**2 - 6*u) / T_total
        dh01 = (-6*u**2 + 6*u) / T_total
        qs[k] = h00 * q0 + h01 * q1
        qdots[k] = dh00 * q0 + dh01 * q1
    return qs, qdots


def main():
    model = pin.buildModelFromUrdf(URDF)
    state = crocoddyl.StateMultibody(model)
    actuation = crocoddyl.ActuationModelFull(state)
    nu = actuation.nu
    data = model.createData()
    sid = model.getFrameId("hammer_strike")

    # ── Stage 1: 임팩트 자세 IK ──
    print("\n=== Stage 1: 임팩트 자세 IK ===")
    q1, R_target = solve_impact_ik(model, data)
    print(f"q1 (deg) = {np.rad2deg(q1).round(2).tolist()}")

    # ── Stage 2: 백스윙 자세 ──
    print("\n=== Stage 2: 백스윙 자세 (q1 + offset) ===")
    q0 = q1.copy()
    for j, off in BACKSWING_OFFSET_DEG.items():
        q0[j] += np.deg2rad(off)
    print(f"q0 (deg) = {np.rad2deg(q0).round(2).tolist()}")

    # ── Stage 3: cubic warm start ──
    print(f"\n=== Stage 3: Cubic spline warm-start (T={T*DT}s) ===")
    q_ref, qdot_ref = cubic_hermite_warm(q0, q1, T*DT, DT)
    xs_warm = [np.concatenate([q_ref[k], qdot_ref[k]]) for k in range(T+1)]
    us_warm = [np.zeros(nu) for _ in range(T)]
    print(f"warm start: {len(xs_warm)} states, {len(us_warm)} controls")

    # ── Stage 4: iLQR (Crocoddyl) ──
    print("\n=== Stage 4: iLQR OCP 빌드 ===")
    x0 = np.concatenate([q0, np.zeros(model.nv)])

    # Running cost: state regularization (J1/J4/J6 락 + cubic ref tracking)
    # Reference 가 매 step 다른 q_ref(t) 라서 step-wise cost.
    # 단순화: 그냥 J1/J4/J6 만 q0 값 락 + J2/J3/J5 자유.
    state_w_q = np.array([W_LOCK_JOINT, W_FREE_JOINT, W_FREE_JOINT,
                          W_LOCK_JOINT, W_FREE_JOINT, W_LOCK_JOINT])
    state_w_v = W_VEL_REG * np.ones(model.nv)
    state_w   = np.concatenate([state_w_q, state_w_v])
    state_act = crocoddyl.ActivationModelWeightedQuad(state_w ** 2)
    state_res = crocoddyl.ResidualModelState(state, x0, nu)
    state_cost = crocoddyl.CostModelResidual(state, state_act, state_res)

    ctrl_res  = crocoddyl.ResidualModelControl(state, nu)
    ctrl_cost = crocoddyl.CostModelResidual(state, ctrl_res)

    # Joint pos barrier (J3 ±155°)
    q_ub = np.deg2rad(JOINT_POS_LIMIT_DEG)
    state_lb = np.concatenate([-q_ub, -1e6 * np.ones(model.nv)])
    state_ub = np.concatenate([ q_ub,  1e6 * np.ones(model.nv)])
    bnd_act = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(state_lb, state_ub))
    bnd_res = crocoddyl.ResidualModelState(state, np.zeros(state.nx), nu)
    bnd_cost = crocoddyl.CostModelResidual(state, bnd_act, bnd_res)

    # Joint vel barrier
    v_ub = np.deg2rad(JOINT_VEL_LIMIT_DEG)
    vel_state_lb = np.concatenate([-1e6 * np.ones(model.nq), -v_ub])
    vel_state_ub = np.concatenate([ 1e6 * np.ones(model.nq),  v_ub])
    vel_act = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(vel_state_lb, vel_state_ub))
    vel_res = crocoddyl.ResidualModelState(state, np.zeros(state.nx), nu)
    vel_cost = crocoddyl.CostModelResidual(state, vel_act, vel_res)

    running_cost = crocoddyl.CostModelSum(state, nu)
    running_cost.addCost("state",   state_cost, 1.0)
    running_cost.addCost("ctrl",    ctrl_cost,  W_TAU)
    running_cost.addCost("bounds",  bnd_cost,   W_BOUNDS)
    running_cost.addCost("vel_lim", vel_cost,   W_VEL_BARRIER)

    # Link world-z barriers
    z_lb_arm = np.array([-1e6, -1e6, LINK_Z_MIN_ARM])
    z_ub = np.array([1e6, 1e6, 1e6])
    z_act_arm = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(z_lb_arm, z_ub))
    for ln in ("link_3", "link_4", "link_5"):
        fid = model.getFrameId(ln)
        zr = crocoddyl.ResidualModelFrameTranslation(state, fid, np.zeros(3), nu)
        zc = crocoddyl.CostModelResidual(state, z_act_arm, zr)
        running_cost.addCost(f"z_{ln}", zc, W_LINK_Z)
    z_lb_tip = np.array([-1e6, -1e6, LINK_Z_MIN_TIP])
    z_act_tip = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(z_lb_tip, z_ub))
    for ln in ("gripper_full_link", "hammer_link", "hammer_strike"):
        fid = model.getFrameId(ln)
        zr = crocoddyl.ResidualModelFrameTranslation(state, fid, np.zeros(3), nu)
        zc = crocoddyl.CostModelResidual(state, z_act_tip, zr)
        running_cost.addCost(f"z_{ln}", zc, W_LINK_Z)

    # Terminal cost: position + orientation + STRONG velocity
    pos_res = crocoddyl.ResidualModelFrameTranslation(state, sid, NAIL_HEAD, nu)
    pos_cost = crocoddyl.CostModelResidual(state, pos_res)

    rot_res = crocoddyl.ResidualModelFrameRotation(state, sid, R_target, nu)
    rot_cost = crocoddyl.CostModelResidual(state, rot_res)

    vref = pin.Motion(np.array([0.0, 0.0, -V_DES_DOWN]), np.zeros(3))
    vel_t_res = crocoddyl.ResidualModelFrameVelocity(
        state, sid, vref, pin.LOCAL_WORLD_ALIGNED, nu)
    vel_t_cost = crocoddyl.CostModelResidual(state, vel_t_res)

    terminal_cost = crocoddyl.CostModelSum(state, nu)
    terminal_cost.addCost("strike_pos", pos_cost,   W_POS)
    terminal_cost.addCost("strike_rot", rot_cost,   W_ROT)
    terminal_cost.addCost("strike_vel", vel_t_cost, W_VEL)

    # Action models
    run_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuation, running_cost)
    term_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuation, terminal_cost)
    run_IAM = crocoddyl.IntegratedActionModelEuler(run_DAM, DT)
    term_IAM = crocoddyl.IntegratedActionModelEuler(term_DAM, 0.0)

    problem = crocoddyl.ShootingProblem(x0, [run_IAM] * T, term_IAM)
    solver = crocoddyl.SolverFDDP(problem)

    print(f"\n=== Solving FDDP (T={T}, DT={DT}, horizon={T*DT}s, V_DES={V_DES_DOWN} m/s) ===")
    converged = solver.solve(xs_warm, us_warm, 300, False, 1e-9)
    print(f"converged={converged}, iters={solver.iter}, cost={solver.cost:.4f}")

    xs = np.array(solver.xs)
    qs    = xs[:, :model.nq]   # (T+1, nq)
    qdots = xs[:, model.nq:]   # (T+1, nv)

    # Acceleration via central difference (rad/s²)
    qddots = np.zeros_like(qdots)
    qddots[1:-1] = (qdots[2:] - qdots[:-2]) / (2 * DT)
    qddots[0]    = qddots[1]
    qddots[-1]   = qddots[-2]

    # ── Verification ──
    print("\n=== Verification ===")
    pin.forwardKinematics(model, data, qs[-1], qdots[-1])
    pin.updateFramePlacements(model, data)
    pT = data.oMf[sid].translation
    nT = data.oMf[sid].rotation @ [0, 0, 1]
    ang = np.rad2deg(np.arccos(np.clip(np.dot(nT, [0, 0, -1]), -1, 1)))
    vT = pin.getFrameVelocity(model, data, sid, pin.LOCAL_WORLD_ALIGNED)
    print(f"final pos       = {pT.round(4)}    err={np.linalg.norm(pT-NAIL_HEAD)*1000:.1f} mm")
    print(f"final ori       = {nT.round(3)}    vs -Z = {ang:.1f}°")
    print(f"final lin vel   = {vT.linear.round(3)} m/s   |v|={np.linalg.norm(vT.linear):.2f} m/s")
    print(f"   ↑ momentum  = m × v = 0.5 × {np.linalg.norm(vT.linear):.2f} = {0.5*np.linalg.norm(vT.linear):.2f} kg·m/s (망치 0.5kg)")

    # Trajectory shape
    zs = []; gy = []
    for q in qs:
        pin.forwardKinematics(model, data, q); pin.updateFramePlacements(model, data)
        zs.append(data.oMf[sid].translation[2])
        gy.append((data.oMf[model.getFrameId("gripper_full_link")].rotation @ [0,1,0])[2])
    zs = np.array(zs); diffs = np.diff(zs)
    print(f"hammer_strike z 시작/max/min/끝 = {zs[0]:.3f}/{zs.max():.3f}/{zs.min():.3f}/{zs[-1]:.3f}")
    print(f"  내려가는 frame: {np.sum(diffs<0)},  올라가는: {np.sum(diffs>0)}")
    print(f"그리퍼 +Y z<0 frames: {sum(1 for v in gy if v<0)}/{len(qs)}")

    # Vel check
    v_deg = np.rad2deg(np.diff(qs, axis=0)) / DT
    limits = [120, 120, 180, 225, 225, 225]
    print("Velocity peak:")
    for j in range(6):
        vp = np.max(np.abs(v_deg[:, j]))
        print(f"  J{j+1}  peak={vp:6.1f}°/s  ({100*vp/limits[j]:.0f}% limit)")

    # Save CSV: t, q (deg), qdot (deg/s), qddot (deg/s²)  — 19 columns
    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    t_col = np.arange(len(qs))[:, None] * DT
    data_out = np.hstack([t_col, np.rad2deg(qs), np.rad2deg(qdots), np.rad2deg(qddots)])
    header = ('t,'
              'J1,J2,J3,J4,J5,J6,'
              'J1_dot,J2_dot,J3_dot,J4_dot,J5_dot,J6_dot,'
              'J1_ddot,J2_ddot,J3_ddot,J4_ddot,J5_ddot,J6_ddot')
    np.savetxt(OUT_CSV, data_out, delimiter=',', fmt='%.4f', header=header, comments='')
    print(f"\nSaved → {OUT_CSV}  ({len(qs)} rows, 19 cols: t + pos + vel + acc)")


if __name__ == "__main__":
    main()
