#!/usr/bin/env python3
"""
M1013 + 그리퍼 + 망치 평면 swing 궤적 최적화 (Crocoddyl FDDP).

설계 요약 (사용자 요구 반영):
  - swing plane = J1 으로 정한 vertical plane.  J1/J4/J6 는 락 (큰 state weight).
  - 임팩트 시 hammer_strike 가 nail_head 좌표에 정확히 도달, 동시에 -z 방향 5 m/s 속도.
  - J2/J3/J5 자유 → 어깨/팔꿈치/손목으로 큰 호 그리며 swing.

산출:
  output/swing_traj.csv    (T+1 rows × 6 cols, joint angles in DEG)
"""

import os
import numpy as np
import pinocchio as pin
import crocoddyl

URDF_PATH = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
OUT_CSV   = "/home/kos/Desktop/Code/doosan_kos/output/swing_traj.csv"

NAIL_HEAD = np.array([-0.65, -0.45, 0.10])    # base + 5cm height

# 백스윙 정점 hammer_strike 위치 — nail 위 BACKSWING_HEIGHT m
BACKSWING_HEIGHT = 0.50   # nail 위 50cm — forearm 이 책상 위 충분히 들리도록
BACKSWING_TARGET = NAIL_HEAD + np.array([0.0, 0.0, BACKSWING_HEIGHT])

# IK 락: J4 = 0 (forearm twist).
# J1, J6 도 IK 단계에서 free → impact IK 결과로 swing 락 값 자동 결정.
# 즉 IK 가 R_target 도달 가능한 J1/J6 자동 찾음 → swing 중 그 값으로 락.
IK_LOCKED = {3: 0.0}                                  # J4 only
IK_FREE   = [0, 1, 2, 4, 5]                           # J1, J2, J3, J5, J6
IK_INIT_DEG = {0: -150.0, 1: -30.0, 2: 80.0, 4: 30.0, 5: -98.0}

# OCP 파라미터.
# 주의: T*DT 가 emulator 의 joint velocity 한계 (J1/J2/J3=120°/s, J4/J5/J6=225°/s)
# 안에서 풀릴 수 있어야 함.  너무 짧으면 emulator 가 finalTime 알람 띄우며
# 자체 시간으로 늘려서 swing 이 매우 느려짐.
DT = 0.01
T  = 500   # 5.0 s 총 swing — vel barrier 약한 강제력 보완 위해 시간 충분히

# Cost weights
W_POS         = 3.0e3     # terminal: hammer_strike → nail_head  (위치 강조)
W_ROT         = 5.0e2     # terminal: strike face down + handle horizontal back
W_VEL_TARGET  = 5.0e1     # terminal: hammer_strike v == (0,0,-V_DES)
V_DES_DOWN    = 1.5       # m/s — M1013 모터 한계 안에서 가능한 strike speed
W_LOCK_JOINT  = 1.0e5     # running: J1/J4/J6 변동 강한 페널티 (사실상 hard constraint)
W_FREE_JOINT  = 1.0e-3    # running: J2/J3/J5 자유 (q0 와 떨어져도 OK)
W_VEL_REG     = 1.0e-1    # running: q̇ 정규화
W_TAU         = 1.0e-3    # running: 토크²
W_BOUNDS      = 1.0e4     # running: joint position limit (J3 ±160°)
W_VEL_BARRIER = 1.0e5     # running: joint velocity limit (강하게 — emulator 매우 보수적)
W_LINK_Z      = 5.0e5     # running: link world z 하한 (책상 충돌 방지, 강하게)
LINK_Z_MIN    = 0.10      # m, bench top 0.05 + 5cm 마진 (link_3/4/5 origin)

# M1013 joint position limits (URDF 기준).  J3 만 ±160°, 나머지는 ±360°.
# 안전 마진 5° 빼고 사용.
JOINT_POS_LIMIT_DEG = np.array([355.0, 355.0, 155.0, 355.0, 355.0, 355.0])
# M1013 joint velocity limits — emulator 가 매우 보수적이라 60% 만 사용.
JOINT_VEL_LIMIT_DEG = np.array([72.0, 72.0, 108.0, 135.0, 135.0, 135.0])


def solve_backswing_ik(model, data, target_pos, R_target, n_iter=800, step=0.2, tol=5e-4):
    """J4/J6 LOCKED, J1/J2/J3/J5 free, 6D residual (pos + ori).

    Damped least-squares Newton.  6 residual vs 4 DOF → underdetermined,
    DLS 로 근사 (정확도 ~1mm, ~1° 정도).
    """
    strike_id = model.getFrameId("hammer_strike")
    q = np.zeros(model.nq)
    for j, v in IK_LOCKED.items():
        q[j] = np.deg2rad(v)
    for j, v in IK_INIT_DEG.items():
        q[j] = np.deg2rad(v)

    free = IK_FREE
    M_target = pin.SE3(R_target, target_pos)
    for k in range(n_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        M_cur = data.oMf[strike_id]
        M_err = M_target.actInv(M_cur)               # current relative to target
        e6 = pin.log(M_err).vector                    # 6D twist residual (linear, angular)
        err = np.linalg.norm(e6)
        if err < tol:
            print(f"[ik]   converged @ iter {k}, |pos|={np.linalg.norm(e6[:3])*1000:.2f}mm, |ori|={np.rad2deg(np.linalg.norm(e6[3:])):.2f}°")
            return q
        J = pin.computeFrameJacobian(model, data, q, strike_id, pin.LOCAL)
        Jp = J[:, free]                               # (6 × 4)
        # DLS:  dq = -J^T (J J^T + λI)^-1 e
        lam = 1e-2
        dq_free = -Jp.T @ np.linalg.solve(Jp @ Jp.T + lam * np.eye(6), e6)
        for idx, ji in enumerate(free):
            q[ji] += step * dq_free[idx]
    print(f"[ik]   max iter reached, |pos|={np.linalg.norm(e6[:3])*1000:.2f}mm, |ori|={np.rad2deg(np.linalg.norm(e6[3:])):.2f}°")
    return q


def main():
    model = pin.buildModelFromUrdf(URDF_PATH)
    state = crocoddyl.StateMultibody(model)
    actuation = crocoddyl.ActuationModelFull(state)
    nu = actuation.nu
    nx = state.nx
    print(f"[ocp] state nx={nx}, control nu={nu}")

    strike_id = model.getFrameId("hammer_strike")
    print(f"[ocp] hammer_strike frame id = {strike_id}")

    data = model.createData()

    # ───── 백스윙 자세 q0 자동 풀기 ─────
    # Swing geometry: hammer_strike 는 그리퍼 (= 손목, 회전 중심) 기준 호를 그림.
    # 임팩트 자세에서 그리퍼 위치 P_grip 을 IK로 먼저 풀고,
    # 그 P_grip 기준으로 backswing 시작 위치를 swing_normal axis 기준 회전.
    eh3 = np.array([NAIL_HEAD[0], NAIL_HEAD[1], 0.0])
    eh3 = eh3 / np.linalg.norm(eh3)
    swing_normal = np.cross(eh3, np.array([0, 0, 1.0]))
    swing_normal = swing_normal / np.linalg.norm(swing_normal)
    BACKSWING_ARC_DEG = -90.0   # swing arc — grip 뒤집히지 않는 분기
    R_swing_back = pin.exp3(np.deg2rad(BACKSWING_ARC_DEG) * swing_normal)
    R_strike_target_for_ik = np.array([
        [-eh3[1], -eh3[0], 0.0],
        [ eh3[0], -eh3[1], 0.0],
        [ 0.0,    0.0,    1.0],
    ]) @ np.diag([1.0, -1.0, -1.0])
    R_backswing_target = R_swing_back @ R_strike_target_for_ik

    # 임팩트 자세 IK (J6 도 free) → J1/J6 락 값 자동 결정 + 그리퍼 위치 추출
    print(f"\n[ocp] Step1: 임팩트 자세 IK (J6 도 free 로 풀어서 락 값 자동 결정)")
    q_impact = solve_backswing_ik(model, data, NAIL_HEAD, R_strike_target_for_ik)
    q_impact[0] = (q_impact[0] + np.pi) % (2 * np.pi) - np.pi
    q_impact[5] = (q_impact[5] + np.pi) % (2 * np.pi) - np.pi
    print(f"[ocp]   q_impact (deg, normalized) = {np.rad2deg(q_impact).round(2).tolist()}")
    J1_LOCK = float(q_impact[0])
    J6_LOCK = float(q_impact[5])
    print(f"[ocp]   → swing 중 락:  J1={np.rad2deg(J1_LOCK):.2f}°, J4=0°, J6={np.rad2deg(J6_LOCK):.2f}°")
    pin.forwardKinematics(model, data, q_impact); pin.updateFramePlacements(model, data)
    P_grip = data.oMf[model.getFrameId("gripper_full_link")].translation.copy()

    # Backswing strike 위치 = swing arc geometry (그리퍼 회전 중심 기준).
    P_backswing_strike = P_grip + R_swing_back @ (NAIL_HEAD - P_grip)
    print(f"\n[ocp] Step2: Backswing IK (J1/J4/J6 자동 락)")
    print(f"[ocp]   backswing strike pos target = {P_backswing_strike.round(4)}")
    # IK 락 / free 동적 변경
    global IK_LOCKED, IK_FREE, IK_INIT_DEG
    IK_LOCKED = {0: np.rad2deg(J1_LOCK), 3: 0.0, 5: np.rad2deg(J6_LOCK)}
    IK_FREE   = [1, 2, 4]
    IK_INIT_DEG = {1: float(np.rad2deg(q_impact[1])), 2: float(np.rad2deg(q_impact[2])),
                   4: float(np.rad2deg(q_impact[4]))}
    q0 = solve_backswing_ik(model, data, P_backswing_strike, R_backswing_target)
    Q0_DEG = np.rad2deg(q0)
    x0 = np.concatenate([q0, np.zeros(model.nv)])

    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)
    p_strike0 = data.oMf[strike_id].translation
    print(f"[ocp] q0 (deg)         = {Q0_DEG.round(2)}")
    print(f"[ocp] hammer_strike0   = {p_strike0.round(4)} (백스윙 정점)")
    print(f"[ocp] nail_head        = {NAIL_HEAD}")
    print(f"[ocp] |start − nail|   = {np.linalg.norm(p_strike0 - NAIL_HEAD)*100:.1f} cm")

    # State residual weight: q 부분에 per-joint, q̇ 부분에 균일
    state_w_q = np.array([
        W_LOCK_JOINT,  # J1
        W_FREE_JOINT,  # J2
        W_FREE_JOINT,  # J3
        W_LOCK_JOINT,  # J4
        W_FREE_JOINT,  # J5
        W_LOCK_JOINT,  # J6
    ])
    state_w_v = W_VEL_REG * np.ones(model.nv)
    state_w   = np.concatenate([state_w_q, state_w_v])
    state_act = crocoddyl.ActivationModelWeightedQuad(state_w ** 2)
    state_res = crocoddyl.ResidualModelState(state, x0, nu)
    state_cost = crocoddyl.CostModelResidual(state, state_act, state_res)

    ctrl_res  = crocoddyl.ResidualModelControl(state, nu)
    ctrl_cost = crocoddyl.CostModelResidual(state, ctrl_res)

    # ───── Joint position barrier (J3 ±160° 등 한계 보호) ─────
    q_ub = np.deg2rad(JOINT_POS_LIMIT_DEG)
    q_lb = -q_ub
    state_lb = np.concatenate([q_lb, -1e6 * np.ones(model.nv)])
    state_ub = np.concatenate([q_ub,  1e6 * np.ones(model.nv)])
    bounds_act = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(state_lb, state_ub))
    bounds_res = crocoddyl.ResidualModelState(state, np.zeros(state.nx), nu)
    bounds_cost = crocoddyl.CostModelResidual(state, bounds_act, bounds_res)

    # ───── Joint VELOCITY barrier (M1013 모터 한계 — 가장 중요) ─────
    v_ub = np.deg2rad(JOINT_VEL_LIMIT_DEG)
    v_lb = -v_ub
    vel_state_lb = np.concatenate([-1e6 * np.ones(model.nq), v_lb])
    vel_state_ub = np.concatenate([ 1e6 * np.ones(model.nq), v_ub])
    vel_act = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(vel_state_lb, vel_state_ub))
    vel_res = crocoddyl.ResidualModelState(state, np.zeros(state.nx), nu)
    vel_cost = crocoddyl.CostModelResidual(state, vel_act, vel_res)

    running_cost = crocoddyl.CostModelSum(state, nu)
    running_cost.addCost("state",     state_cost,  1.0)
    running_cost.addCost("ctrl",      ctrl_cost,   W_TAU)
    running_cost.addCost("bounds",    bounds_cost, W_BOUNDS)
    running_cost.addCost("vel_lim",   vel_cost,    W_VEL_BARRIER)

    # ───── Link world-z barrier (책상 충돌 방지) ─────
    # 강한 barrier: link_3/4/5 = 책상 위 15cm  (팔이 책상 위 일정 거리 유지)
    z_lb_arm = np.array([-1e6, -1e6, LINK_Z_MIN])
    z_ub_arm = np.array([ 1e6,  1e6,  1e6])
    z_act_arm = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(z_lb_arm, z_ub_arm))
    for link_name in ("link_3", "link_4", "link_5"):
        link_id = model.getFrameId(link_name)
        z_res = crocoddyl.ResidualModelFrameTranslation(
            state, link_id, np.zeros(3), nu)
        z_cost = crocoddyl.CostModelResidual(state, z_act_arm, z_res)
        running_cost.addCost(f"z_{link_name}", z_cost, W_LINK_Z)

    # 약한 barrier: gripper, hammer_link = 책상 표면 z=0.05 위 5cm 마진
    # hammer_strike 는 임팩트 시점에만 z=0.10 정확 도달, 그 외엔 위
    z_lb_low = np.array([-1e6, -1e6, 0.10])
    z_ub_low = np.array([ 1e6,  1e6,  1e6])
    z_act_low = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(z_lb_low, z_ub_low))
    for link_name in ("gripper_full_link", "hammer_link"):
        link_id = model.getFrameId(link_name)
        z_res = crocoddyl.ResidualModelFrameTranslation(
            state, link_id, np.zeros(3), nu)
        z_cost = crocoddyl.CostModelResidual(state, z_act_low, z_res)
        running_cost.addCost(f"z_{link_name}", z_cost, W_LINK_Z)

    # hammer_strike: z=0.05 (책상 표면) 만 안 통과
    z_lb_str = np.array([-1e6, -1e6, 0.05])
    z_act_str = crocoddyl.ActivationModelQuadraticBarrier(
        crocoddyl.ActivationBounds(z_lb_str, z_ub_low))
    z_res_str = crocoddyl.ResidualModelFrameTranslation(
        state, model.getFrameId("hammer_strike"), np.zeros(3), nu)
    z_cost_str = crocoddyl.CostModelResidual(state, z_act_str, z_res_str)
    running_cost.addCost("z_hammer_strike", z_cost_str, W_LINK_Z)

    # Terminal: 위치 + 속도 + orientation (strike face down)
    pos_res  = crocoddyl.ResidualModelFrameTranslation(state, strike_id, NAIL_HEAD, nu)
    pos_cost = crocoddyl.CostModelResidual(state, pos_res)

    vref = pin.Motion(np.array([0.0, 0.0, -V_DES_DOWN]), np.zeros(3))
    vel_res = crocoddyl.ResidualModelFrameVelocity(
        state, strike_id, vref, pin.LOCAL_WORLD_ALIGNED, nu)
    vel_cost = crocoddyl.CostModelResidual(state, vel_res)

    # ── Strike orientation target ─────────────────────────────────────────
    # 임팩트 순간 hammer_strike 자세:
    #   +Z (strike normal) = world -Z   ← 못 머리(XY 평면) 와 평행하게 타격
    #   handle direction = robot back direction (= -ê_h)
    #   head top = world +Z
    # ê_h = base→nail XY 단위벡터.  hammer_link 좌표계에서 +Y=handle.
    # hammer_strike 는 hammer_link 에 Rx(π) 적용된 child → strike Y/Z 부호 반전.
    eh_xy = np.array([NAIL_HEAD[0], NAIL_HEAD[1]])
    eh = eh_xy / np.linalg.norm(eh_xy)            # base→nail unit XY
    R_hammer_link_target = np.array([
        [-eh[1], -eh[0], 0.0],
        [ eh[0], -eh[1], 0.0],
        [ 0.0,    0.0,   1.0],
    ])
    R_x_pi = np.diag([1.0, -1.0, -1.0])
    R_strike_target = R_hammer_link_target @ R_x_pi
    print(f"[ocp] R_strike_target =\n{R_strike_target.round(3)}")
    print(f"[ocp]   strike +Z (impact normal) world = {(R_strike_target @ [0,0,1]).round(3)}  (목표 [0,0,-1])")
    rot_res  = crocoddyl.ResidualModelFrameRotation(state, strike_id, R_strike_target, nu)
    rot_cost = crocoddyl.CostModelResidual(state, rot_res)

    terminal_cost = crocoddyl.CostModelSum(state, nu)
    terminal_cost.addCost("strike_pos", pos_cost, W_POS)
    terminal_cost.addCost("strike_rot", rot_cost, W_ROT)
    terminal_cost.addCost("strike_vel", vel_cost, W_VEL_TARGET)

    # Action models
    run_DAM  = crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuation, running_cost)
    term_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuation, terminal_cost)
    run_IAM  = crocoddyl.IntegratedActionModelEuler(run_DAM, DT)
    term_IAM = crocoddyl.IntegratedActionModelEuler(term_DAM, 0.0)

    problem = crocoddyl.ShootingProblem(x0, [run_IAM] * T, term_IAM)
    solver  = crocoddyl.SolverFDDP(problem)
    solver.setCallbacks([crocoddyl.CallbackVerbose()])

    xs0 = [x0.copy() for _ in range(T + 1)]
    us0 = [np.zeros(nu) for _ in range(T)]

    print(f"\n[ocp] Solving FDDP (T={T}, dt={DT}, horizon={T*DT}s) ...")
    converged = solver.solve(xs0, us0, 200, False, 1e-9)
    print(f"[ocp] converged={converged}, iters={solver.iter}, cost={solver.cost:.4f}")

    xs = np.array(solver.xs)
    qs = xs[:, :model.nq]
    qdots = xs[:, model.nq:]

    # 최종 자세 FK / velocity 검증
    pin.forwardKinematics(model, data, qs[-1], qdots[-1])
    pin.updateFramePlacements(model, data)
    p_strike_T = data.oMf[strike_id].translation
    v_str = pin.getFrameVelocity(model, data, strike_id, pin.LOCAL_WORLD_ALIGNED)
    print(f"\n[ocp] final hammer_strike  = {p_strike_T.round(4)}")
    print(f"[ocp] error vs nail head    = {np.linalg.norm(p_strike_T - NAIL_HEAD)*100:.2f} cm")
    print(f"[ocp] final linear vel      = {v_str.linear.round(3)} m/s   (target=[0,0,{-V_DES_DOWN}])")
    print(f"[ocp] final angular vel     = {v_str.angular.round(3)} rad/s")
    print(f"[ocp] final speed magnitude = {np.linalg.norm(v_str.linear):.2f} m/s")

    # 락된 관절 변동 확인
    dq16_max = np.max(np.abs(np.rad2deg(qs[:, [0, 3, 5]] - q0[[0, 3, 5]])), axis=0)
    dq235_max = np.max(np.abs(np.rad2deg(qs[:, [1, 2, 4]] - q0[[1, 2, 4]])), axis=0)
    print(f"[ocp] J1/J4/J6 max swing |Δ| = {dq16_max.round(2)} deg  (작아야 함)")
    print(f"[ocp] J2/J3/J5 max swing |Δ| = {dq235_max.round(2)} deg")

    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    np.savetxt(OUT_CSV, np.rad2deg(qs), delimiter=',', fmt='%.4f',
               header=f'J1,J2,J3,J4,J5,J6  (deg, dt={DT}s, rows={T+1})',
               comments='')
    print(f"\n[ocp] Saved trajectory → {OUT_CSV}")


if __name__ == "__main__":
    main()
