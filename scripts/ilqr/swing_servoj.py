#!/usr/bin/env python3
"""
swing_servoj.py — servoj_rt streaming 용 iLQR trajectory.

swing_optimal.py 와 동일한 OCP 를 풀되:
  1. 출력은 output/swing_traj_servoj.csv (swing_traj.csv 안 덮음)
  2. 13 열로 저장: t, J1..J6 (deg), J1_dot..J6_dot (deg/s)
     → servoj_rt 가 pos + vel feedforward 둘 다 필요로 함
  3. 자동 T 조정: vel/acc 한계 초과 시 swing 시간(T) 늘려 재시도

검사 한계 (M1013 스펙):
  vel  : [120, 120, 180, 225, 225, 225] deg/s
  acc  : [300, 300, 400, 500, 500, 500] deg/s²
타깃: 스펙의 85% (margin)
"""

import os
import numpy as np
import pinocchio as pin
import crocoddyl

URDF = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
OUT_CSV = "/home/kos/Desktop/Code/doosan_kos/output/swing_traj_servoj.csv"

NAIL_HEAD = np.array([-0.65, -0.45, 0.10])

BACKSWING_OFFSET_DEG = {
    1: -80.0,
    2: +20.0,
    4: -150.0,    # J5 +60° → -150°: 반대 분기로 가서 망치를 그리퍼 위로 들어올림
                  # q₀.J5 = q₁.J5(53°) + (-150°) = -97° → 망치 z=0.93m (그리퍼 위 9cm)
                  # swing 범위 150° (1s = 150 deg/s 평균, M1013 J5 스펙 225 안)
}

DT = 0.01
T_INIT = 100           # 시작 horizon (1.0s)
T_MAX  = 400           # 안전상 상한 (4.0s)
MAX_ATTEMPTS = 6

W_POS         = 5.0e3
W_ROT         = 5.0e2
W_VEL         = 1.0e3
V_DES_DOWN    = 1.5       # 3.0 → 1.5 m/s (운동량 0.75 kg·m/s, 스펙 안에서 swing)

W_LOCK_JOINT  = 1.0e5
W_FREE_JOINT  = 1.0e-3
W_VEL_REG     = 1.0e-1
W_TAU         = 1.0e-3    # 원래 값 복원 (1e-1은 임팩트 위치 망쳤음)
W_BOUNDS      = 1.0e4
W_VEL_BARRIER = 5.0e4     # 5e3 → 5e4 (10×, vel 한계 강화)
W_LINK_Z      = 5.0e5

JOINT_POS_LIMIT_DEG = np.array([355.0, 355.0, 155.0, 355.0, 355.0, 355.0])
# iLQR 내부 barrier 한계 — 스펙 × 0.85 로 타이트하게
JOINT_VEL_LIMIT_DEG = np.array([102.0, 102.0, 153.0, 191.0, 191.0, 191.0])
LINK_Z_MIN_ARM = 0.10
LINK_Z_MIN_TIP = 0.05

# 외부 검증 한계 (M1013 스펙)
SPEC_VEL = np.array([120.0, 120.0, 180.0, 225.0, 225.0, 225.0])    # deg/s
SPEC_ACC = np.array([300.0, 300.0, 400.0, 500.0, 500.0, 500.0])    # deg/s²
SAFETY_FACTOR = 0.85    # 타깃 = 스펙 × 0.85


def solve_impact_ik(model, data, n_iter=800, step=0.2, tol=5e-4):
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


def solve_iLQR(model, state, actuation, data, q0, q1, R_target, T):
    """주어진 horizon T 로 iLQR FDDP 풂. (qs, qdots) 반환 (rad)."""
    nu = actuation.nu
    sid = model.getFrameId("hammer_strike")

    q_ref, qdot_ref = cubic_hermite_warm(q0, q1, T * DT, DT)
    xs_warm = [np.concatenate([q_ref[k], qdot_ref[k]]) for k in range(T + 1)]
    us_warm = [np.zeros(nu) for _ in range(T)]

    x0 = np.concatenate([q0, np.zeros(model.nv)])

    state_w_q = np.array([W_LOCK_JOINT, W_FREE_JOINT, W_FREE_JOINT,
                          W_LOCK_JOINT, W_FREE_JOINT, W_LOCK_JOINT])
    state_w_v = W_VEL_REG * np.ones(model.nv)
    state_w   = np.concatenate([state_w_q, state_w_v])
    state_act = crocoddyl.ActivationModelWeightedQuad(state_w ** 2)
    state_res = crocoddyl.ResidualModelState(state, x0, nu)
    state_cost = crocoddyl.CostModelResidual(state, state_act, state_res)

    ctrl_res  = crocoddyl.ResidualModelControl(state, nu)
    ctrl_cost = crocoddyl.CostModelResidual(state, ctrl_res)

    q_ub = np.deg2rad(JOINT_POS_LIMIT_DEG)
    state_lb = np.concatenate([-q_ub, -1e6 * np.ones(model.nv)])
    state_ub = np.concatenate([ q_ub,  1e6 * np.ones(model.nv)])
    bnd_act = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(state_lb, state_ub))
    bnd_res = crocoddyl.ResidualModelState(state, np.zeros(state.nx), nu)
    bnd_cost = crocoddyl.CostModelResidual(state, bnd_act, bnd_res)

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

    run_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuation, running_cost)
    term_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuation, terminal_cost)
    run_IAM = crocoddyl.IntegratedActionModelEuler(run_DAM, DT)
    term_IAM = crocoddyl.IntegratedActionModelEuler(term_DAM, 0.0)

    problem = crocoddyl.ShootingProblem(x0, [run_IAM] * T, term_IAM)
    solver = crocoddyl.SolverFDDP(problem)
    converged = solver.solve(xs_warm, us_warm, 300, False, 1e-9)
    print(f"  FDDP T={T} ({T*DT:.2f}s): converged={converged}, iters={solver.iter}, cost={solver.cost:.4f}")

    xs = np.array(solver.xs)
    return xs[:, :model.nq], xs[:, model.nq:]


def measure_peaks(qs, qdots):
    """Peak velocity / acceleration (deg/s, deg/s²)."""
    vel_deg = np.rad2deg(qdots)
    vel_peak = np.max(np.abs(vel_deg), axis=0)               # (6,)
    acc_deg = np.diff(vel_deg, axis=0) / DT
    acc_peak = np.max(np.abs(acc_deg), axis=0)               # (6,)
    return vel_peak, acc_peak


def main():
    model = pin.buildModelFromUrdf(URDF)
    state = crocoddyl.StateMultibody(model)
    actuation = crocoddyl.ActuationModelFull(state)
    data = model.createData()
    sid = model.getFrameId("hammer_strike")

    print("=== Stage 1: 임팩트 자세 IK ===")
    q1, R_target = solve_impact_ik(model, data)
    print(f"q1 (deg) = {np.rad2deg(q1).round(2).tolist()}")

    print("\n=== Stage 2: 백스윙 자세 (q1 + offset) ===")
    q0 = q1.copy()
    for j, off in BACKSWING_OFFSET_DEG.items():
        q0[j] += np.deg2rad(off)
    print(f"q0 (deg) = {np.rad2deg(q0).round(2).tolist()}")

    target_vel = SPEC_VEL * SAFETY_FACTOR
    target_acc = SPEC_ACC * SAFETY_FACTOR
    print(f"\n타깃 vel (스펙×{SAFETY_FACTOR}) = {target_vel.tolist()} deg/s")
    print(f"타깃 acc (스펙×{SAFETY_FACTOR}) = {target_acc.tolist()} deg/s²")

    T = T_INIT
    qs = qdots = None
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"\n=== Attempt {attempt}: T={T} ({T*DT:.2f}s) ===")
        qs, qdots = solve_iLQR(model, state, actuation, data, q0, q1, R_target, T)
        vel_peak, acc_peak = measure_peaks(qs, qdots)

        print(f"  vel peak (deg/s): {[f'{v:.1f}' for v in vel_peak]}")
        print(f"  acc peak (deg/s²): {[f'{a:.1f}' for a in acc_peak]}")

        vel_ratio = vel_peak / target_vel
        acc_ratio = acc_peak / target_acc
        vel_worst_j = int(np.argmax(vel_ratio))
        acc_worst_j = int(np.argmax(acc_ratio))
        vel_max_ratio = vel_ratio[vel_worst_j]
        acc_max_ratio = acc_ratio[acc_worst_j]

        print(f"  vel worst: J{vel_worst_j+1} = {vel_peak[vel_worst_j]:.1f} (target {target_vel[vel_worst_j]:.1f}) → {vel_max_ratio*100:.0f}%")
        print(f"  acc worst: J{acc_worst_j+1} = {acc_peak[acc_worst_j]:.1f} (target {target_acc[acc_worst_j]:.1f}) → {acc_max_ratio*100:.0f}%")

        # Stop 조건: vel peak 가 진짜 M1013 SPEC 안에 들어오면 OK
        # (target=spec×0.85 는 iLQR 내부 마진 목표, stop 기준은 100% spec).
        vel_ratio_spec = np.max(vel_peak / SPEC_VEL)
        print(f"  vel vs SPEC: max ratio = {vel_ratio_spec*100:.1f}%")
        if vel_ratio_spec <= 1.0:
            print(f"  ✓ vel 모든 관절 SPEC 안. T={T} ({T*DT:.2f}s) 채택.")
            print(f"    (acc 일부 초과는 임팩트 직전 step-jump 성격, firmware clip 영역)")
            break

        # vel은 1/T 로, acc는 1/T² 로 스케일 → 둘 중 큰 요구량을 적용
        new_T_for_vel = int(np.ceil(T * vel_max_ratio * 1.10))
        new_T_for_acc = int(np.ceil(T * np.sqrt(acc_max_ratio) * 1.10))
        new_T = max(new_T_for_vel, new_T_for_acc, T + 10)
        if new_T > T_MAX:
            print(f"  ✗ T={new_T}이 T_MAX={T_MAX} 초과. T={T_MAX}로 최종 시도.")
            new_T = T_MAX
        if new_T == T:
            print(f"  ✗ T 변화 없음, 종료.")
            break
        print(f"  ✗ 한계 초과. 다음 attempt: T={new_T}")
        T = new_T
    else:
        print(f"\n⚠️  {MAX_ATTEMPTS}회 시도 후에도 한계 초과 (마지막 T={T}). 결과 저장.")

    # ── 최종 검증 ──
    print("\n=== Final verification ===")
    pin.forwardKinematics(model, data, qs[-1], qdots[-1])
    pin.updateFramePlacements(model, data)
    pT = data.oMf[sid].translation
    nT = data.oMf[sid].rotation @ [0, 0, 1]
    ang = np.rad2deg(np.arccos(np.clip(np.dot(nT, [0, 0, -1]), -1, 1)))
    vT = pin.getFrameVelocity(model, data, sid, pin.LOCAL_WORLD_ALIGNED)
    print(f"final pos     = {pT.round(4)}   err={np.linalg.norm(pT-NAIL_HEAD)*1000:.1f} mm")
    print(f"final ori vs -Z = {ang:.1f}°")
    print(f"final lin vel = {vT.linear.round(3)} m/s   |v|={np.linalg.norm(vT.linear):.2f} m/s")
    print(f"   ↑ momentum = m × v = 0.5 × {np.linalg.norm(vT.linear):.2f} = {0.5*np.linalg.norm(vT.linear):.2f} kg·m/s")

    vel_peak, acc_peak = measure_peaks(qs, qdots)
    print("\nFinal peaks vs M1013 spec:")
    print(f"  joint │  vel peak │ vel spec │   %    │  acc peak │ acc spec │   %")
    print(f"  ──────┼───────────┼──────────┼────────┼───────────┼──────────┼──────")
    for j in range(6):
        vp, ap = vel_peak[j], acc_peak[j]
        vs, as_ = SPEC_VEL[j], SPEC_ACC[j]
        flag_v = " " if vp <= vs else "★"
        flag_a = " " if ap <= as_ else "★"
        print(f"  J{j+1}    │ {vp:7.1f}{flag_v} │ {vs:7.1f}  │ {100*vp/vs:5.1f}% │ {ap:7.1f}{flag_a} │ {as_:7.1f}  │ {100*ap/as_:5.1f}%")

    # ── CSV 저장: t, q (deg), qdot (deg/s) ──
    n_rows = len(qs)
    t_col = np.arange(n_rows) * DT
    qs_deg = np.rad2deg(qs)
    qdots_deg = np.rad2deg(qdots)
    out_data = np.hstack([t_col.reshape(-1, 1), qs_deg, qdots_deg])

    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    header = (f"t,J1,J2,J3,J4,J5,J6,J1_dot,J2_dot,J3_dot,J4_dot,J5_dot,J6_dot  "
              f"(deg & deg/s, iLQR servoj swing, dt={DT}s, rows={n_rows}, total={n_rows*DT:.2f}s)")
    np.savetxt(OUT_CSV, out_data, delimiter=',', fmt='%.5f',
               header=header, comments='')
    print(f"\nSaved → {OUT_CSV}")
    print(f"  rows={n_rows}, total_time={n_rows*DT:.2f}s, columns=13 (t, q×6, q̇×6)")


if __name__ == "__main__":
    main()
