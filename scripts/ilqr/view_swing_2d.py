#!/usr/bin/env python3
"""
2D side-view viewer for iLQR swing trajectory.

망치질은 J1/J4/J6 락 + J2/J3/J5 협응 평면 운동이라, base→nail vertical plane
으로 projection 해서 stick figure 로 swing 보여주면 직관적.

가로축 h : base→nail 방향 거리 (m)
세로축 z : world Z (m)

사용:
    ~/ilqr_venv/bin/python3 scripts/ilqr/view_swing_2d.py
    ~/ilqr_venv/bin/python3 scripts/ilqr/view_swing_2d.py --slider     # 시간 슬라이더
"""

import argparse
import csv

import matplotlib
matplotlib.use('QtAgg')   # PySide6 자동 감지

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from matplotlib.widgets import Slider

URDF = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
CSV  = "/home/kos/Desktop/Code/doosan_kos/output/swing_traj.csv"

NAIL_BASE   = np.array([-0.65, -0.45, 0.05])
NAIL_HEAD_Z = 0.10
BENCH_Z     = 0.05
BENCH_H_RANGE = (0.0, 1.2)    # 측면 뷰에서 책상이 보이는 h 범위

LINKS = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4',
         'link_5', 'link_6', 'gripper_full_link', 'hammer_link', 'hammer_strike']

# 화면용 짧은 라벨 (link 이름)
LINK_LABELS = ['Base', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6', 'Grip', 'Ham', 'Strike']

# 각 link 로 들어오는 joint 이름 + 락/자유 (swing 중 변동 여부)
# base_link 는 들어오는 joint 없음.  J1/J4/J6 락, J2/J3/J5 자유 (망치질 평면 운동).
JOINT_LABELS = [None,
                'J1 (lock)',  # base→link_1
                'J2 (free)',  # link_1→link_2
                'J3 (free)',  # link_2→link_3
                'J4 (lock)',  # link_3→link_4
                'J5 (free)',  # link_4→link_5
                'J6 (lock)',  # link_5→link_6
                'fixed',       # gripper
                'fixed',       # hammer
                'fixed']       # strike face


def load_traj(path):
    with open(path) as f:
        r = csv.reader(f); next(r)
        rows = [[float(x) for x in row[:6]] for row in r if row]
    return np.deg2rad(np.array(rows))


def make_projector():
    """ê_h = base→nail XY 방향 단위벡터.  h = ê_h · (x,y), z = z."""
    eh = np.array([NAIL_BASE[0], NAIL_BASE[1]])
    eh = eh / np.linalg.norm(eh)
    def proj(p):  # p : 3-vec world
        return float(eh[0]*p[0] + eh[1]*p[1]), float(p[2])
    def proj_dir(d):  # 3-vec direction → 2-vec (Δh, Δz)
        return float(eh[0]*d[0] + eh[1]*d[1]), float(d[2])
    return proj, proj_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slider', action='store_true',
                    help='시간 슬라이더 (자동 재생 대신)')
    ap.add_argument('--interval', type=int, default=33,
                    help='프레임 간격 ms (기본 33 = ~30 fps)')
    ap.add_argument('--loop', action='store_true', default=True,
                    help='반복 재생')
    args = ap.parse_args()

    qs = load_traj(CSV)
    T = len(qs)
    print(f'[view] loaded {T} frames from {CSV}')

    model = pin.buildModelFromUrdf(URDF)
    data  = model.createData()
    proj, proj_dir = make_projector()
    fids  = {n: model.getFrameId(n) for n in LINKS}
    grip_fid = fids['gripper_full_link']
    strike_fid = fids['hammer_strike']
    h_nail, _ = proj(NAIL_BASE)

    hammer_fid = fids['hammer_link']
    HANDLE_LEN = 0.087   # hammer_link → head 끝 (hammer 의 local -Y 방향)

    # 미리 모든 프레임 FK 계산 (T=150 정도라 빠름)
    all_h = np.zeros((T, len(LINKS)))
    all_v = np.zeros((T, len(LINKS)))
    grip_up_dir = np.zeros((T, 2))   # gripper +Y axis (body up) projected (Δh, Δz)
    strike_face_dir = np.zeros((T, 2))   # hammer_strike +Z (impact face normal) projected
    head_end_h = np.zeros(T)         # hammer head 끝 위치 (h 좌표)
    head_end_v = np.zeros(T)         # hammer head 끝 위치 (z 좌표)
    for k in range(T):
        pin.forwardKinematics(model, data, qs[k])
        pin.updateFramePlacements(model, data)
        for j, name in enumerate(LINKS):
            h, v = proj(data.oMf[fids[name]].translation)
            all_h[k, j] = h
            all_v[k, j] = v
        # +Y of gripper = body 방향 (위)
        gy = data.oMf[grip_fid].rotation @ np.array([0.0, 1.0, 0.0])
        grip_up_dir[k] = proj_dir(gy)
        # +Z of hammer_strike = 타격면 normal
        sz = data.oMf[strike_fid].rotation @ np.array([0.0, 0.0, 1.0])
        strike_face_dir[k] = proj_dir(sz)
        # Hammer head 끝 = hammer_link world pos + R_hammer @ (0, -HANDLE_LEN, 0)
        head_world = data.oMf[hammer_fid].translation + \
                     data.oMf[hammer_fid].rotation @ np.array([0.0, -HANDLE_LEN, 0.0])
        head_end_h[k], head_end_v[k] = proj(head_world)

    # plot 범위
    h_min = min(all_h.min(), 0.0) - 0.1
    h_max = max(all_h.max(), h_nail) + 0.1
    v_min = -0.05
    v_max = max(all_v.max(), 1.2) + 0.1

    fig, ax = plt.subplots(figsize=(10, 7))
    fig.subplots_adjust(bottom=0.18 if args.slider else 0.10)
    ax.set_xlabel('h  [base→nail direction]  (m)')
    ax.set_ylabel('z  (m)')
    ax.set_aspect('equal')
    ax.set_xlim(h_min, h_max)
    ax.set_ylim(v_min, v_max)
    ax.grid(True, alpha=0.3)

    # 책상 (수평선 + 면 채움)
    ax.fill_between(BENCH_H_RANGE, -0.05, BENCH_Z,
                    color='peru', alpha=0.35)
    ax.plot(BENCH_H_RANGE, [BENCH_Z]*2, color='saddlebrown', lw=2,
            label='Bench (z=0.05)')

    # 못
    ax.plot([h_nail, h_nail], [BENCH_Z, NAIL_HEAD_Z],
            color='dimgray', lw=3, label='Nail')
    ax.plot([h_nail], [NAIL_HEAD_Z], 'ko', ms=10, zorder=5,
            label='Nail head (target)')

    # 로봇 (base ~ link_6) — link 0..6 (그리퍼는 별도로 π 모양 그림)
    # M1013 link 단면 ~10cm → 굵게 표시 (그리퍼 plate 와 시각적 비율 맞춤)
    arm_line, = ax.plot([], [], 'o-', color='steelblue', lw=7,
                        ms=10, label='M1013 (links ~10cm thick)')
    # 망치 본체 (hammer_link → head 끝, 일자 handle, 직경 ~3cm)
    hammer_line, = ax.plot([], [], '-', color='crimson', lw=4,
                           solid_capstyle='round', label='Hammer (handle)')
    # 망치 strike face (hammer_strike 점 강조)
    strike_dot, = ax.plot([], [], 'o', color='red', ms=10, zorder=6)
    STRIKE_ARROW_LEN = 0.07
    # 망치 strike face normal 화살표 (+Z of hammer_strike)
    strike_face_arrow = ax.annotate(
        '', xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle='-|>', color='red',
                        lw=2.5, mutation_scale=18), zorder=8,
    )
    strike_face_label = ax.text(0, 0, 'strike face', fontsize=8.5,
                                color='red', fontweight='bold',
                                ha='left', va='bottom', zorder=11)
    # 망치 strike face 본체 (수직 짧은 막대)
    strike_face_line, = ax.plot([], [], '-', color='red', lw=4,
                                solid_capstyle='butt', zorder=7,
                                label='Strike face')

    # ── 그리퍼 π 모양 (정면뷰 가정) ──────────────────────────────────────
    # gripper_full_link origin = body top (link_6 부착쪽).  bbox Y=0..-0.128.
    # plate tips 가 Y=-0.128 = hammer_link 위치 → 점선 없이 plate 가 직접 망치 잡음.
    # bbox 0.04 × 0.13 × 0.04 — 가로(폭) 4cm, 세로(길이) 12.8cm.
    GRIP_BODY_OFFSET = 0.0    # body bar 가 gripper origin 에 위치 (top)
    GRIP_BODY_HALF   = 0.02   # body bar 절반 폭 (m, 총 40mm = bbox X)
    GRIP_PLATE_LEN   = 0.128  # plate 길이 — gripper bbox 와 일치, hammer_link 까지 닿음
    # 얇은 metal plate → lw 2.0
    grip_body_line,  = ax.plot([], [], '-', color='royalblue', lw=2.0,
                               solid_capstyle='round', zorder=7,
                               label='Gripper π (body↑/plates↓)')
    grip_plate_l,    = ax.plot([], [], '-', color='royalblue', lw=2.0,
                               solid_capstyle='round', zorder=7)
    grip_plate_r,    = ax.plot([], [], '-', color='royalblue', lw=2.0,
                               solid_capstyle='round', zorder=7)

    # ── 라벨: 점마다 link 이름 (위), joint 이름+lock/free (아래) ──
    link_lbls  = []
    joint_lbls = []
    for j, (lname, jname) in enumerate(zip(LINK_LABELS, JOINT_LABELS)):
        lt = ax.text(0, 0, lname, fontsize=8.5, color='navy',
                     ha='left', va='bottom', zorder=11,
                     fontweight='bold')
        link_lbls.append(lt)
        if jname is None:
            joint_lbls.append(None)
        else:
            color = 'darkred' if 'lock' in jname else 'darkgreen'
            jt = ax.text(0, 0, jname, fontsize=7, color=color,
                         ha='left', va='top', zorder=11)
            joint_lbls.append(jt)

    title = ax.set_title('')
    ax.legend(loc='upper right', fontsize=9)

    OFF_LINK = 0.02   # 라벨 점 옆 offset
    OFF_JNT  = 0.025

    def render(frame):
        # arm: 0..6 (base_link ~ link_6) — 그리퍼는 π 모양으로 별도 그림
        arm_line.set_data(all_h[frame, :7], all_v[frame, :7])
        # 망치 본체: hammer_link(8) → head 끝 (handle 일자)
        hammer_line.set_data([all_h[frame, 8], head_end_h[frame]],
                             [all_v[frame, 8], head_end_v[frame]])
        strike_dot.set_data([all_h[frame, 9]], [all_v[frame, 9]])

        # ── 그리퍼 π 모양 ──
        gh, gv = all_h[frame, 7], all_v[frame, 7]   # gripper origin
        dh_g, dv_g = grip_up_dir[frame]              # +Y 단위벡터 (2D)
        # 단위 벡터 정규화 (projection 후 길이 < 1 일 수 있음 — Y 가 평면 밖일 때)
        n = np.hypot(dh_g, dv_g)
        if n > 1e-6:
            dh_g, dv_g = dh_g/n, dv_g/n
        # body bar 위치 = gripper 위치 + GRIP_BODY_OFFSET * +Y
        bh = gh + GRIP_BODY_OFFSET * dh_g
        bv = gv + GRIP_BODY_OFFSET * dv_g
        # body bar 양 끝 (perp 방향)
        perp_h, perp_v = -dv_g, dh_g
        bl_h = bh - GRIP_BODY_HALF * perp_h
        bl_v = bv - GRIP_BODY_HALF * perp_v
        br_h = bh + GRIP_BODY_HALF * perp_h
        br_v = bv + GRIP_BODY_HALF * perp_v
        grip_body_line.set_data([bl_h, br_h], [bl_v, br_v])
        # plates: body bar 양 끝에서 -Y 방향으로 GRIP_PLATE_LEN 내려감
        pl_h_end = bl_h - GRIP_PLATE_LEN * dh_g
        pl_v_end = bl_v - GRIP_PLATE_LEN * dv_g
        pr_h_end = br_h - GRIP_PLATE_LEN * dh_g
        pr_v_end = br_v - GRIP_PLATE_LEN * dv_g
        grip_plate_l.set_data([bl_h, pl_h_end], [bl_v, pl_v_end])
        grip_plate_r.set_data([br_h, pr_h_end], [br_v, pr_v_end])

        # 망치 strike face 화살표 (hammer_strike 위치에서 +Z 방향)
        sh, sv = all_h[frame, 9], all_v[frame, 9]
        dh_s, dv_s = strike_face_dir[frame]
        end_s = (sh + STRIKE_ARROW_LEN * dh_s, sv + STRIKE_ARROW_LEN * dv_s)
        strike_face_arrow.set_position((sh, sv))
        strike_face_arrow.xy = end_s
        strike_face_label.set_position((end_s[0] + 0.005, end_s[1] + 0.005))
        # 타격면 본체 (face normal 의 수직방향, 짧은 막대)
        perp_sh, perp_sv = -dv_s, dh_s
        sf1 = (sh - 0.025 * perp_sh, sv - 0.025 * perp_sv)
        sf2 = (sh + 0.025 * perp_sh, sv + 0.025 * perp_sv)
        strike_face_line.set_data([sf1[0], sf2[0]], [sf1[1], sf2[1]])

        # 라벨 위치 업데이트
        for j in range(len(LINKS)):
            h, v = all_h[frame, j], all_v[frame, j]
            link_lbls[j].set_position((h + OFF_LINK, v + OFF_LINK*0.5))
            if joint_lbls[j] is not None:
                joint_lbls[j].set_position((h + OFF_JNT, v - OFF_JNT*0.5))
        # 헤드와 못 거리
        d = np.hypot(all_h[frame, 9] - h_nail,
                     all_v[frame, 9] - NAIL_HEAD_Z)
        flip = ' ⚠FLIP' if grip_up_dir[frame, 1] < 0 else ''
        title.set_text(
            f't = {frame*0.01:5.2f} s   '
            f'frame {frame:3d}/{T-1}   '
            f'|strike − nail head| = {d*1000:6.1f} mm{flip}')

    if args.slider:
        ax_slider = fig.add_axes([0.15, 0.05, 0.7, 0.03])
        sl = Slider(ax_slider, 'frame', 0, T-1, valinit=0, valstep=1)
        def on_slide(v):
            render(int(v)); fig.canvas.draw_idle()
        sl.on_changed(on_slide)
        render(0)
    else:
        anim = animation.FuncAnimation(
            fig, render, frames=T, interval=args.interval,
            blit=False, repeat=args.loop)

    plt.show()


if __name__ == '__main__':
    main()
