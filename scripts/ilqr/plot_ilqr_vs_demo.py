#!/usr/bin/env python3
"""
iLQR 가 계산한 궤적 vs 실제 demo 가 emulator 에 보내서 따라가는 궤적 비교.

iLQR optimal CSV (101 frame, 1s) — 빨강 (실제로는 안 쓰임, J5 whip 포함)
Demo trapezoidal (q0→q100, 1.5s) — 파랑 (실제로 emulator 가 따라감, 직선 보간)
"""

import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pinocchio as pin

URDF = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
CSV  = "/home/kos/Desktop/Code/doosan_kos/output/swing_traj.csv"
OUT  = "/home/kos/Desktop/Code/doosan_kos/output/ilqr_vs_demo.png"

NAIL_BASE   = np.array([-0.65, -0.45, 0.05])
NAIL_HEAD_Z = 0.10
BENCH_Z     = 0.05
HANDLE_LEN  = 0.087


def proj(p, eh):
    return float(eh[0]*p[0] + eh[1]*p[1]), float(p[2])


def trapezoidal(q0, q1, T, dt, acc=600.0):
    """Per-joint trapezoidal interpolation (deg/s)."""
    n = int(round(T/dt)) + 1
    qs = np.zeros((n, 6))
    for j in range(6):
        d = q1[j] - q0[j]
        sign = 1.0 if d >= 0 else -1.0
        D = abs(d)
        if D < 1e-9:
            qs[:, j] = q0[j]
            continue
        # peak vel solving D = T*v - v²/acc → v² - acc*T*v + acc*D = 0
        disc = (acc*T)**2 - 4*acc*D
        if disc < 0:
            v_p = D / T   # triangular fallback
        else:
            v_p = (acc*T - np.sqrt(disc)) / 2
        t_a = v_p / acc
        t_c = T - 2*t_a
        for k in range(n):
            t = k*dt
            if t < t_a:
                disp = 0.5 * acc * t * t
            elif t < t_a + t_c:
                disp = 0.5*acc*t_a*t_a + v_p*(t - t_a)
            else:
                tau = T - t
                disp = D - 0.5*acc*tau*tau
            qs[k, j] = q0[j] + sign * disp
    return qs


def main():
    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()
    sid = model.getFrameId("hammer_strike")
    hid = model.getFrameId("hammer_link")

    eh_xy = NAIL_BASE[:2]; eh = eh_xy / np.linalg.norm(eh_xy)
    h_nail, _ = proj(NAIL_BASE, eh)

    # iLQR CSV 로드
    with open(CSV) as f:
        r = csv.reader(f); next(r)
        ilqr_qs = np.array([[float(x) for x in row[:6]] for row in r if row])
    ilqr_qs_rad = np.deg2rad(ilqr_qs)

    # Demo trapezoidal 재구성 (q0, q100, T=1.5s, dt=0.01)
    demo_qs = trapezoidal(ilqr_qs[0], ilqr_qs[-1], T=1.5, dt=0.01, acc=600.0)
    demo_qs_rad = np.deg2rad(demo_qs)

    # FK 로 hammer head 끝 위치 trajectory 두 개 계산
    def fk_strike_path(qs_rad):
        path_h, path_z, vel_z = [], [], []
        prev_z = None
        for k, q in enumerate(qs_rad):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            p = data.oMf[sid].translation
            h, z = proj(p, eh)
            path_h.append(h); path_z.append(z)
            if prev_z is not None:
                vel_z.append((z - prev_z) / 0.01)
            else:
                vel_z.append(0.0)
            prev_z = z
        return np.array(path_h), np.array(path_z), np.array(vel_z)

    ih, iz, ivz = fk_strike_path(ilqr_qs_rad)
    dh, dz, dvz = fk_strike_path(demo_qs_rad)

    # Plot — 2 panel: side view + impact velocity profile
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # ── Panel 1: side-view trajectory ──
    ax1.fill_between([-0.4, 1.2], -0.05, BENCH_Z, color='peru', alpha=0.3)
    ax1.plot([-0.4, 1.2], [BENCH_Z]*2, color='saddlebrown', lw=2)
    ax1.plot([h_nail, h_nail], [BENCH_Z, NAIL_HEAD_Z], color='dimgray', lw=3)
    ax1.plot([h_nail], [NAIL_HEAD_Z], 'ko', ms=12, zorder=5, label='Nail')

    # iLQR path (red — discarded)
    ax1.plot(ih, iz, '-', color='crimson', lw=2.5, alpha=0.9,
             label='iLQR optimal (101 frames, 1s) -- COMPUTED BUT DISCARDED')
    ax1.plot(ih[::10], iz[::10], 'o', color='crimson', ms=4)

    # Demo path (blue — actual motion)
    ax1.plot(dh, dz, '-', color='royalblue', lw=2.5, alpha=0.9,
             label='Demo trapezoidal (2 endpoints, 1.5s) -- ACTUAL emulator')
    ax1.plot(dh[::10], dz[::10], 's', color='royalblue', ms=4)

    # 시작/끝 강조
    ax1.plot([ih[0]], [iz[0]], '^', color='black', ms=14, zorder=6, label='q₀ (BACKSWING)')
    ax1.plot([ih[-1]], [iz[-1]], 'v', color='black', ms=14, zorder=6, label='q₁₀₀ (IMPACT)')

    ax1.set_xlabel('h  [base→nail direction]  (m)', fontsize=11)
    ax1.set_ylabel('z  (m)', fontsize=11)
    ax1.set_aspect('equal')
    ax1.set_xlim(-0.3, 1.0); ax1.set_ylim(-0.05, 0.85)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=9)
    ax1.set_title('Hammer strike face trajectory (side view)', fontsize=12, fontweight='bold')

    # ── Panel 2: vertical velocity at strike face vs time ──
    t_ilqr = np.arange(len(ivz)) * 0.01
    t_demo = np.arange(len(dvz)) * 0.01
    ax2.plot(t_ilqr, ivz, '-', color='crimson', lw=2.5, label='iLQR (impact vel = -3 m/s)')
    ax2.plot(t_demo, dvz, '-', color='royalblue', lw=2.5,
             label='Demo trapezoidal (impact vel = 0)')
    ax2.axhline(0, color='gray', lw=0.5)
    ax2.set_xlabel('t  (s)', fontsize=11)
    ax2.set_ylabel('strike face  $v_z$  (m/s)', fontsize=11)
    ax2.set_title('Impact velocity profile (= hammering momentum)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='lower left', fontsize=10)

    fig.suptitle(
        'What "iLQR demo" actually does: only endpoints survive, emulator linearly interpolates between them',
        fontsize=13, fontweight='bold', y=1.00)
    fig.tight_layout()
    fig.savefig(OUT, dpi=140, bbox_inches='tight')
    print(f'Saved → {OUT}')

    # 요약 수치
    impact_v_ilqr = ivz[-1]
    impact_v_demo = dvz[-1]
    print(f'\n임팩트 시점 strike face vertical velocity:')
    print(f'  iLQR:  v_z = {impact_v_ilqr:+.2f} m/s')
    print(f'  Demo:  v_z = {impact_v_demo:+.2f} m/s   ← 실제 로봇이 내리치는 속도')


if __name__ == '__main__':
    main()
