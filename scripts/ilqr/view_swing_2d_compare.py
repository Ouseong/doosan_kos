#!/usr/bin/env python3
"""
3-panel comparison viewer: swing_ocp / swing_cubic / swing_optimal 동시 재생.

가로축 h : base→nail 방향 거리 (m)
세로축 z : world Z (m)

세 알고리즘이 frame 수가 달라서 (ocp 501, 나머지 101) 진행률 u ∈ [0,1] 로
동기화. 같은 u 에서 각 panel 이 자기 trajectory 의 비례 frame 을 그림.

사용:
    ~/ilqr_venv/bin/python3 scripts/ilqr/view_swing_2d_compare.py
    ~/ilqr_venv/bin/python3 scripts/ilqr/view_swing_2d_compare.py --slider
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

PANELS = [
    {
        'csv':   "/home/kos/Desktop/Code/doosan_kos/output/swing_traj_ocp.csv",
        'title': "(1) swing_ocp.py — Pure iLQR (T=5s, V*=1.5m/s)",
        'color': 'darkorange',
    },
    {
        'csv':   "/home/kos/Desktop/Code/doosan_kos/output/swing_traj_cubic.csv",
        'title': "(2) swing_cubic.py — Cubic Hermite (T=1s, q̇_T=0)",
        'color': 'seagreen',
    },
    {
        'csv':   "/home/kos/Desktop/Code/doosan_kos/output/swing_traj_optimal.csv",
        'title': "(3) swing_optimal.py — cubic-warmstart + iLQR (T=1s, V*=3m/s)",
        'color': 'crimson',
    },
]

NAIL_BASE   = np.array([-0.65, -0.45, 0.05])
NAIL_HEAD_Z = 0.10
BENCH_Z     = 0.05
BENCH_H_RANGE = (0.0, 1.2)

LINKS = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4',
         'link_5', 'link_6', 'gripper_full_link', 'hammer_link', 'hammer_strike']


def load_traj(path):
    with open(path) as f:
        r = csv.reader(f); next(r)
        rows = [[float(x) for x in row[:6]] for row in r if row]
    return np.deg2rad(np.array(rows))


def make_projector():
    eh = np.array([NAIL_BASE[0], NAIL_BASE[1]])
    eh = eh / np.linalg.norm(eh)
    def proj(p):
        return float(eh[0]*p[0] + eh[1]*p[1]), float(p[2])
    return proj


def precompute_panel(model, qs, proj):
    """모든 frame 에 대해 link world 위치 (h, z) 와 grip+Y world z 미리 계산."""
    data = model.createData()
    fids = {n: model.getFrameId(n) for n in LINKS}
    grip_fid = fids['gripper_full_link']
    T = len(qs)
    H = np.zeros((T, len(LINKS)))
    Z = np.zeros((T, len(LINKS)))
    grip_y_z = np.zeros(T)
    strike_z = np.zeros(T)
    for k in range(T):
        pin.forwardKinematics(model, data, qs[k])
        pin.updateFramePlacements(model, data)
        for j, name in enumerate(LINKS):
            h, z = proj(data.oMf[fids[name]].translation)
            H[k, j] = h; Z[k, j] = z
        grip_y_z[k] = (data.oMf[grip_fid].rotation @ [0, 1, 0])[2]
        strike_z[k] = data.oMf[fids['hammer_strike']].translation[2]
    return H, Z, grip_y_z, strike_z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slider', action='store_true')
    ap.add_argument('--interval', type=int, default=33)
    args = ap.parse_args()

    model = pin.buildModelFromUrdf(URDF)
    proj  = make_projector()
    h_nail, _ = proj(NAIL_BASE)

    # Load + precompute all 3
    panels = []
    for cfg in PANELS:
        qs = load_traj(cfg['csv'])
        H, Z, gy_z, sz = precompute_panel(model, qs, proj)
        panels.append({**cfg, 'qs': qs, 'H': H, 'Z': Z,
                       'gy_z': gy_z, 'sz': sz, 'T': len(qs)})
        print(f"[load] {cfg['csv'].split('/')[-1]:25s}  frames={len(qs)}")

    # Plot range — 모든 panel 합쳐서 같은 range
    h_min = min(p['H'].min() for p in panels) - 0.1
    h_max = max(max(p['H'].max() for p in panels), h_nail) + 0.1
    z_min = -0.05
    z_max = max(max(p['Z'].max() for p in panels), 1.2) + 0.1

    fig, axes = plt.subplots(1, 3, figsize=(20, 7), sharey=True)
    fig.subplots_adjust(left=0.05, right=0.98, top=0.92,
                        bottom=0.18 if args.slider else 0.10,
                        wspace=0.08)
    fig.suptitle('Swing trajectory comparison — synchronized progress u ∈ [0, 1]',
                 fontsize=13, fontweight='bold')

    # 각 panel artist
    arts = []
    for ax, p in zip(axes, panels):
        ax.set_title(p['title'], fontsize=10)
        ax.set_xlabel('h  [base→nail]  (m)')
        ax.set_aspect('equal')
        ax.set_xlim(h_min, h_max)
        ax.set_ylim(z_min, z_max)
        ax.grid(True, alpha=0.3)

        ax.fill_between(BENCH_H_RANGE, -0.05, BENCH_Z,
                        color='peru', alpha=0.35)
        ax.plot(BENCH_H_RANGE, [BENCH_Z]*2, color='saddlebrown', lw=2)
        ax.plot([h_nail, h_nail], [BENCH_Z, NAIL_HEAD_Z],
                color='dimgray', lw=3)
        ax.plot([h_nail], [NAIL_HEAD_Z], 'ko', ms=9, zorder=5)

        # Trajectory trail of strike point (faint)
        ax.plot(p['H'][:, 9], p['Z'][:, 9],
                '-', color=p['color'], lw=1.0, alpha=0.25)

        arm,    = ax.plot([], [], 'o-', color='steelblue', lw=2.4, ms=5)
        hammer, = ax.plot([], [], '-',  color=p['color'], lw=4.5,
                          solid_capstyle='round')
        strike, = ax.plot([], [], 'o',  color=p['color'], ms=9, zorder=6)
        info    = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                          fontsize=9, va='top', family='monospace',
                          bbox=dict(boxstyle='round', facecolor='white',
                                    alpha=0.85, edgecolor='gray'))
        arts.append({'arm': arm, 'hammer': hammer, 'strike': strike,
                     'info': info, 'ax': ax})

    axes[0].set_ylabel('z  (m)')

    def render(u):
        """u ∈ [0, 1] — 진행률 동기화."""
        for p, a in zip(panels, arts):
            k = int(round(u * (p['T'] - 1)))
            H, Z = p['H'], p['Z']
            a['arm'].set_data(H[k, :8], Z[k, :8])
            a['hammer'].set_data(H[k, 7:10], Z[k, 7:10])
            a['strike'].set_data([H[k, 9]], [Z[k, 9]])

            # 임팩트 vel 추정 (frame diff)
            if k > 0:
                dt = 0.01
                v_z = (Z[k, 9] - Z[k-1, 9]) / dt
            else:
                v_z = 0.0

            grip_flip = '↓ FLIP' if p['gy_z'][k] < 0 else '↑ ok'
            d_mm = np.hypot(H[k, 9] - h_nail, Z[k, 9] - NAIL_HEAD_Z) * 1000
            a['info'].set_text(
                f"u={u:.2f}  frame {k:>3d}/{p['T']-1}\n"
                f"strike z   = {Z[k, 9]:+.3f} m\n"
                f"|to nail|  = {d_mm:6.1f} mm\n"
                f"v_strike_z = {v_z:+.2f} m/s\n"
                f"grip +Y z  = {p['gy_z'][k]:+.3f}  {grip_flip}"
            )

    if args.slider:
        ax_s = fig.add_axes([0.15, 0.05, 0.7, 0.03])
        sl = Slider(ax_s, 'progress u', 0.0, 1.0, valinit=0.0, valstep=0.005)
        sl.on_changed(lambda v: (render(float(v)), fig.canvas.draw_idle()))
        render(0.0)
    else:
        N = 100  # 100 frames per loop
        anim = animation.FuncAnimation(
            fig, lambda i: render(i / (N - 1)),
            frames=N, interval=args.interval, blit=False, repeat=True)

    plt.show()


if __name__ == '__main__':
    main()
