#!/usr/bin/env python3
"""
2D side-view viewer for FULL hammer demo (pickup → lift → backswing → strike).

hammer_demo.py 의 일련의 모션을 IK + linear joint interpolation 으로 재구성:
  HOME → PRE_ALIGN → APPROACH → AT_HAMMER → CLOSE → LIFT → BACKSWING → STRIKE

각 Cartesian waypoint 은 Pinocchio Damped Least-Squares Newton IK 로 풀고,
joint segment 사이는 등속 보간 (DURATION 에 비례한 frame 수).

망치 상태:
  - 'rest' : 책상 위 (-0.40, -0.45, 0.077), identity rotation
  - 'attached': gripper 에 kinematic attach (URDF 자세 그대로 따라감)

사용:
    ~/ilqr_venv/bin/python3 scripts/ilqr/view_demo_full.py
    ~/ilqr_venv/bin/python3 scripts/ilqr/view_demo_full.py --slider
"""

import argparse
import csv

import matplotlib
matplotlib.use('QtAgg')

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from matplotlib.widgets import Slider

URDF = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
SWING_CSV = "/home/kos/Desktop/Code/doosan_kos/output/swing_traj_optimal.csv"

# Hammer rest pose (책상 위)
HAMMER_REST_POS = np.array([-0.40, -0.45, 0.077])
HAMMER_REST_RPY = np.array([0.0, 0.0, 0.0])

NAIL_BASE   = np.array([-0.65, -0.45, 0.05])
NAIL_HEAD_Z = 0.10
BENCH_Z     = 0.05
BENCH_H_RANGE = (-0.4, 1.2)

LINKS = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4',
         'link_5', 'link_6', 'gripper_full_link', 'hammer_link', 'hammer_strike']

# 데모 각 segment 정의: (label, type, target, duration_s, grip, ham_state)
# type:  'home'  : initial joints (deg)
#        'cart'  : ((x,y,z) m, R 3x3) for link_6 IK target
#        'joint' : target joints (deg) via direct interp
#        'csv'   : load swing CSV (already discrete, just append)
#        'wait'  : stay at current joints, no motion (for grasp/release)

R_TCP_DOWN = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])   # Ry(180°), tool +Z = world -Z

SEGMENTS = [
    ('HOME',       'home',  np.array([0.0, 0.0, 90.0, 0.0, 90.0, 0.0]), 0.0,  'open', 'rest'),
    ('PRE_ALIGN',  'joint', np.array([-134.0, 0.0, 90.0, 0.0, 90.0, -134.0]), 1.5, 'open', 'rest'),
    ('APPROACH',   'cart',  (np.array([-0.400, -0.462, 0.305]), R_TCP_DOWN), 1.5, 'open', 'rest'),
    ('AT_HAMMER',  'cart',  (np.array([-0.400, -0.462, 0.205]), R_TCP_DOWN), 1.0, 'open', 'rest'),
    ('CLOSE',      'wait',  None, 0.5, 'closed', 'attached'),   # 정지 + auto-grasp
    ('LIFT',       'cart',  (np.array([-0.400, -0.462, 0.400]), R_TCP_DOWN), 1.5, 'closed', 'attached'),
    # BACKSWING + STRIKE 는 CSV 가 알아서 처리
    ('BACKSWING',  'csv_first',   SWING_CSV, 1.0, 'closed', 'attached'),
    ('STRIKE',     'csv_strike',  SWING_CSV, 1.5, 'closed', 'attached'),   # 실제 demo: single movej q0→q100, T=1.5s
]

DT = 0.04   # ~25 fps for non-swing portions (swing CSV 자체는 0.01s)


def make_projector():
    eh = np.array([NAIL_BASE[0], NAIL_BASE[1]])
    eh = eh / np.linalg.norm(eh)
    def proj(p):
        return float(eh[0]*p[0] + eh[1]*p[1]), float(p[2])
    def proj_dir(d):
        return float(eh[0]*d[0] + eh[1]*d[1]), float(d[2])
    return proj, proj_dir


def solve_ik(model, data, target_pos, target_R, q_init, n_iter=400, step=0.3, tol=5e-4):
    """6D IK to link_6 frame (= TCP)."""
    target_id = model.getFrameId("link_6")
    M_target = pin.SE3(target_R, target_pos)
    q = q_init.copy()
    last_e = None
    for k in range(n_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        M_err = M_target.actInv(data.oMf[target_id])
        e = pin.log(M_err).vector
        last_e = e
        if np.linalg.norm(e) < tol:
            return q, True
        J = pin.computeFrameJacobian(model, data, q, target_id, pin.LOCAL)
        dq = -J.T @ np.linalg.solve(J @ J.T + 1e-2 * np.eye(6), e)
        q += step * dq
    pos_mm = np.linalg.norm(last_e[:3]) * 1000
    ori_deg = np.rad2deg(np.linalg.norm(last_e[3:]))
    print(f"[ik]   max iter, |pos|={pos_mm:.1f}mm, |ori|={ori_deg:.1f}°")
    return q, False


def load_csv(path):
    with open(path) as f:
        r = csv.reader(f); next(r)
        rows = [[float(x) for x in row[:6]] for row in r if row]
    return np.deg2rad(np.array(rows))


def build_full_trajectory():
    """Pinocchio FK + IK 로 세그먼트 별 joint trajectory 만들고 합침.

    반환: qs (T×6 rad), labels (T,), grip_state (T,), ham_state (T,), seg_starts (segment 시작 frame index list).
    """
    model = pin.buildModelFromUrdf(URDF)
    data  = model.createData()

    qs = []
    labels = []
    grip_state = []
    ham_state = []
    seg_starts = []

    q_cur = None
    for seg in SEGMENTS:
        label, typ, tgt, dur, grip, ham = seg
        seg_starts.append(len(qs))
        if typ == 'home':
            q_cur = np.deg2rad(tgt)
            qs.append(q_cur.copy())
            labels.append(label); grip_state.append(grip); ham_state.append(ham)
        elif typ == 'joint':
            q_target = np.deg2rad(tgt)
            n = max(2, int(round(dur / DT)))
            for k in range(1, n+1):
                u = k / n
                q_interp = (1-u) * q_cur + u * q_target
                qs.append(q_interp)
                labels.append(label); grip_state.append(grip); ham_state.append(ham)
            q_cur = q_target
        elif typ == 'cart':
            target_pos, target_R = tgt
            q_tgt, ok = solve_ik(model, data, target_pos, target_R, q_cur)
            print(f"[seg] {label:11s} IK {'OK' if ok else 'FAIL'}  q={np.rad2deg(q_tgt).round(1).tolist()}")
            n = max(2, int(round(dur / DT)))
            for k in range(1, n+1):
                u = k / n
                q_interp = (1-u) * q_cur + u * q_tgt
                qs.append(q_interp)
                labels.append(label); grip_state.append(grip); ham_state.append(ham)
            q_cur = q_tgt
        elif typ == 'wait':
            n = max(2, int(round(dur / DT)))
            for k in range(n):
                qs.append(q_cur.copy())
                labels.append(label); grip_state.append(grip); ham_state.append(ham)
        elif typ == 'csv_first':
            # BACKSWING = swing CSV row 0 — q_cur 에서 q_csv0 로 보간
            csv_qs = load_csv(tgt)
            q_target = csv_qs[0]
            n = max(2, int(round(dur / DT)))
            for k in range(1, n+1):
                u = k / n
                q_interp = (1-u) * q_cur + u * q_target
                qs.append(q_interp)
                labels.append(label); grip_state.append(grip); ham_state.append(ham)
            q_cur = q_target
        elif typ == 'csv_full':
            csv_qs = load_csv(tgt)
            for q in csv_qs:
                qs.append(q.copy())
                labels.append(label); grip_state.append(grip); ham_state.append(ham)
            q_cur = csv_qs[-1]
        elif typ == 'csv_strike':
            # 실제 demo 의 single movej (BACKSWING q0 → IMPACT q100) trapezoidal-like
            # 보간.  여기선 단순화로 cosine smoothstep 사용 (vel 시작/끝 0).
            csv_qs = load_csv(tgt)
            q_target = csv_qs[-1]
            n = max(2, int(round(dur / DT)))
            for k in range(1, n+1):
                u = k / n
                # smoothstep 으로 trapezoidal 비슷한 vel 프로파일 (시작/끝 vel ≈ 0)
                s = 3*u*u - 2*u*u*u
                q_interp = (1-s) * q_cur + s * q_target
                qs.append(q_interp)
                labels.append(label); grip_state.append(grip); ham_state.append(ham)
            q_cur = q_target

    return (model, np.array(qs), labels, grip_state, ham_state, seg_starts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slider', action='store_true')
    ap.add_argument('--interval', type=int, default=40)
    args = ap.parse_args()

    print("\n=== Building full demo trajectory ===")
    model, qs, labels, grip_state, ham_state, seg_starts = build_full_trajectory()
    T = len(qs)
    data = model.createData()
    proj, proj_dir = make_projector()
    fids = {n: model.getFrameId(n) for n in LINKS}
    h_nail, _ = proj(NAIL_BASE)
    h_rest, _ = proj(HAMMER_REST_POS)

    print(f"Total trajectory: {T} frames\n")
    for i, lab in enumerate(set(labels)):
        cnt = labels.count(lab)
        print(f"  {lab:11s} {cnt:4d} frames")

    # FK precompute — robot 자세 + hammer (rest 또는 attached)
    grip_fid = fids['gripper_full_link']
    hammer_fid = fids['hammer_link']
    strike_fid = fids['hammer_strike']
    HANDLE_LEN = 0.087

    all_h = np.zeros((T, len(LINKS)))
    all_v = np.zeros((T, len(LINKS)))
    grip_up_dir = np.zeros((T, 2))
    strike_face_dir = np.zeros((T, 2))
    head_end_h = np.zeros(T)
    head_end_v = np.zeros(T)
    # Hammer rendered position: rest 일 때 fixed, attached 일 때 FK
    ham_link_h = np.zeros(T)
    ham_link_v = np.zeros(T)
    ham_strike_h = np.zeros(T)
    ham_strike_v = np.zeros(T)

    # rest pose 의 hammer 위치 (변하지 않음)
    rest_link_pos = HAMMER_REST_POS.copy()
    # rest 는 identity rot, hammer +Y axis = world +Y, head 끝 (-Y direction): rest_link + (0, -0.087, 0)
    rest_head_world = rest_link_pos + np.array([0.0, -HANDLE_LEN, 0.0])
    rest_strike_world = rest_link_pos + np.array([0.0, -0.087, -0.027])
    rest_face_dir_world = np.array([0.0, 0.0, -1.0])    # strike +Z = -Z (face down)
    h_rest_link, v_rest_link = proj(rest_link_pos)
    h_rest_head, v_rest_head = proj(rest_head_world)
    h_rest_strike, v_rest_strike = proj(rest_strike_world)
    rest_face_h, rest_face_v = proj_dir(rest_face_dir_world)

    for k in range(T):
        pin.forwardKinematics(model, data, qs[k])
        pin.updateFramePlacements(model, data)
        for j, name in enumerate(LINKS):
            h, v = proj(data.oMf[fids[name]].translation)
            all_h[k, j] = h
            all_v[k, j] = v
        gy = data.oMf[grip_fid].rotation @ np.array([0.0, 1.0, 0.0])
        grip_up_dir[k] = proj_dir(gy)
        sz = data.oMf[strike_fid].rotation @ np.array([0.0, 0.0, 1.0])
        strike_face_dir[k] = proj_dir(sz)

        if ham_state[k] == 'attached':
            # FK 로 그대로
            ham_link_h[k] = all_h[k, 8]
            ham_link_v[k] = all_v[k, 8]
            head_world = data.oMf[hammer_fid].translation + \
                         data.oMf[hammer_fid].rotation @ np.array([0.0, -HANDLE_LEN, 0.0])
            head_end_h[k], head_end_v[k] = proj(head_world)
            ham_strike_h[k] = all_h[k, 9]
            ham_strike_v[k] = all_v[k, 9]
        else:
            # rest 위치 사용
            ham_link_h[k] = h_rest_link
            ham_link_v[k] = v_rest_link
            head_end_h[k] = h_rest_head
            head_end_v[k] = v_rest_head
            ham_strike_h[k] = h_rest_strike
            ham_strike_v[k] = v_rest_strike
            strike_face_dir[k] = (rest_face_h, rest_face_v)

    # Plot setup
    h_min = min(all_h.min(), h_nail, h_rest) - 0.1
    h_max = max(all_h.max(), h_nail, h_rest) + 0.1
    v_min = -0.05
    v_max = max(all_v.max(), 1.2) + 0.1

    fig, ax = plt.subplots(figsize=(12, 8))
    fig.subplots_adjust(bottom=0.18 if args.slider else 0.10)
    ax.set_xlabel('h  [base→nail direction]  (m)')
    ax.set_ylabel('z  (m)')
    ax.set_aspect('equal')
    ax.set_xlim(h_min, h_max)
    ax.set_ylim(v_min, v_max)
    ax.grid(True, alpha=0.3)

    # 책상
    ax.fill_between(BENCH_H_RANGE, -0.05, BENCH_Z, color='peru', alpha=0.35)
    ax.plot(BENCH_H_RANGE, [BENCH_Z]*2, color='saddlebrown', lw=2, label='Bench (z=0.05)')

    # 못
    ax.plot([h_nail, h_nail], [BENCH_Z, NAIL_HEAD_Z], color='dimgray', lw=3, label='Nail')
    ax.plot([h_nail], [NAIL_HEAD_Z], 'ko', ms=10, zorder=5, label='Nail head')

    # 망치 rest 위치 표시 (반투명, 배경)
    ax.plot([h_rest_link, h_rest_head], [v_rest_link, v_rest_head],
            '-', color='crimson', lw=4, alpha=0.18,
            label='Hammer rest pose')

    # M1013 arm
    arm_line, = ax.plot([], [], 'o-', color='steelblue', lw=7, ms=10,
                        label='M1013 (links ~10cm)')
    # 망치 본체
    hammer_line, = ax.plot([], [], '-', color='crimson', lw=4,
                           solid_capstyle='round', label='Hammer (handle)')
    strike_dot, = ax.plot([], [], 'o', color='red', ms=10, zorder=6)
    STRIKE_ARROW_LEN = 0.07
    strike_face_arrow = ax.annotate(
        '', xy=(0,0), xytext=(0,0),
        arrowprops=dict(arrowstyle='-|>', color='red',
                        lw=2.5, mutation_scale=18), zorder=8)
    strike_face_label = ax.text(0,0,'strike face', fontsize=8.5,
                                color='red', fontweight='bold',
                                ha='left', va='bottom', zorder=11)
    strike_face_line, = ax.plot([], [], '-', color='red', lw=4,
                                solid_capstyle='butt', zorder=7,
                                label='Strike face')

    # 그리퍼 π
    GRIP_BODY_OFFSET = 0.0
    GRIP_BODY_HALF   = 0.02
    GRIP_PLATE_LEN   = 0.128
    GRIP_OPEN_HALF   = 0.034   # 그리퍼 열림 시 plate 끝 사이 거리 절반 (~67mm 열림)
    grip_body_line,  = ax.plot([], [], '-', color='royalblue', lw=2.0,
                               solid_capstyle='round', zorder=7,
                               label='Gripper π (body↑/plates↓)')
    grip_plate_l,    = ax.plot([], [], '-', color='royalblue', lw=2.0,
                               solid_capstyle='round', zorder=7)
    grip_plate_r,    = ax.plot([], [], '-', color='royalblue', lw=2.0,
                               solid_capstyle='round', zorder=7)

    # Phase 라벨 (큰 텍스트, 좌상단)
    phase_text = ax.text(0.02, 0.96, '', transform=ax.transAxes,
                         fontsize=14, fontweight='bold', va='top',
                         bbox=dict(boxstyle='round', facecolor='lightyellow',
                                   alpha=0.9, edgecolor='gray'))

    title = ax.set_title('')
    ax.legend(loc='upper right', fontsize=8)

    def render(frame):
        # arm
        arm_line.set_data(all_h[frame, :7], all_v[frame, :7])

        # 망치
        hammer_line.set_data([ham_link_h[frame], head_end_h[frame]],
                             [ham_link_v[frame], head_end_v[frame]])
        strike_dot.set_data([ham_strike_h[frame]], [ham_strike_v[frame]])
        # strike face arrow
        sh, sv = ham_strike_h[frame], ham_strike_v[frame]
        dh_s, dv_s = strike_face_dir[frame]
        end_s = (sh + STRIKE_ARROW_LEN * dh_s, sv + STRIKE_ARROW_LEN * dv_s)
        strike_face_arrow.set_position((sh, sv))
        strike_face_arrow.xy = end_s
        strike_face_label.set_position((end_s[0]+0.005, end_s[1]+0.005))
        perp_sh, perp_sv = -dv_s, dh_s
        sf1 = (sh - 0.025 * perp_sh, sv - 0.025 * perp_sv)
        sf2 = (sh + 0.025 * perp_sh, sv + 0.025 * perp_sv)
        strike_face_line.set_data([sf1[0], sf2[0]], [sf1[1], sf2[1]])

        # 그리퍼 π
        gh, gv = all_h[frame, 7], all_v[frame, 7]
        dh_g, dv_g = grip_up_dir[frame]
        n = np.hypot(dh_g, dv_g)
        if n > 1e-6: dh_g, dv_g = dh_g/n, dv_g/n
        bh = gh + GRIP_BODY_OFFSET * dh_g
        bv = gv + GRIP_BODY_OFFSET * dv_g
        # 그리퍼 열림: open 이면 plate 가 양옆으로 더 벌어짐
        is_open = (grip_state[frame] == 'open')
        half_w = GRIP_OPEN_HALF if is_open else GRIP_BODY_HALF
        perp_h, perp_v = -dv_g, dh_g
        bl_h = bh - half_w * perp_h
        bl_v = bv - half_w * perp_v
        br_h = bh + half_w * perp_h
        br_v = bv + half_w * perp_v
        # body bar 는 항상 좁게 (4cm), plate 만 벌어짐
        bbody_l_h = bh - GRIP_BODY_HALF * perp_h
        bbody_l_v = bv - GRIP_BODY_HALF * perp_v
        bbody_r_h = bh + GRIP_BODY_HALF * perp_h
        bbody_r_v = bv + GRIP_BODY_HALF * perp_v
        grip_body_line.set_data([bbody_l_h, bbody_r_h], [bbody_l_v, bbody_r_v])
        # plate 끝점 = body bar 끝 + plate 가 옆으로 벌어진 만큼 변 + 길이 -Y
        # 단순화: plate 시작점은 body bar end (좁은) → plate 끝점은 (body bar end + 양옆 추가 offset)
        # 여기선 그냥 plate 가 body bar 끝에서 곧장 -Y 로 내려가되 끝부분만 살짝 오므림 또는 벌어짐
        pl_top_h = bbody_l_h + (half_w - GRIP_BODY_HALF) * (-perp_h)
        pl_top_v = bbody_l_v + (half_w - GRIP_BODY_HALF) * (-perp_v)
        pr_top_h = bbody_r_h + (half_w - GRIP_BODY_HALF) * perp_h
        pr_top_v = bbody_r_v + (half_w - GRIP_BODY_HALF) * perp_v
        # plate 다리: top → -Y direction
        pl_h_end = pl_top_h - GRIP_PLATE_LEN * dh_g
        pl_v_end = pl_top_v - GRIP_PLATE_LEN * dv_g
        pr_h_end = pr_top_h - GRIP_PLATE_LEN * dh_g
        pr_v_end = pr_top_v - GRIP_PLATE_LEN * dv_g
        grip_plate_l.set_data([pl_top_h, pl_h_end], [pl_top_v, pl_v_end])
        grip_plate_r.set_data([pr_top_h, pr_h_end], [pr_top_v, pr_v_end])

        # 라벨
        lab = labels[frame]
        ham_lab = ham_state[frame]
        grip_lab = grip_state[frame]
        phase_text.set_text(f'{lab}\ngrip: {grip_lab}\nhammer: {ham_lab}')
        title.set_text(f'frame {frame:4d}/{T-1}')

    if args.slider:
        ax_s = fig.add_axes([0.15, 0.05, 0.7, 0.03])
        sl = Slider(ax_s, 'frame', 0, T-1, valinit=0, valstep=1)
        sl.on_changed(lambda v: (render(int(v)), fig.canvas.draw_idle()))
        render(0)
    else:
        anim = animation.FuncAnimation(
            fig, render, frames=T, interval=args.interval,
            blit=False, repeat=True)

    plt.show()


if __name__ == '__main__':
    main()
