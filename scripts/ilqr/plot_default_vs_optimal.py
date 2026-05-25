#!/usr/bin/env python3
"""
Console dropdown 의 두 옵션 비교 plot — 망치 끝(hammer_strike) trajectory.

  default (movej, no CSV)        — hardcoded BACKSWING→IMPACT, firmware trapezoidal
  optimal (swing_traj_optimal)   — iLQR 3-DOF, 101 frame 1s swing

생성: output/default_vs_optimal.png
"""

import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pinocchio as pin

URDF = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
OPTIMAL_CSV = "/home/kos/Desktop/Code/doosan_kos/output/swing_traj_optimal.csv"
OUT = "/home/kos/Desktop/Code/doosan_kos/output/default_vs_optimal.png"

# hammer_demo_2dof.py 의 현재 default (movej, no CSV) — hardcoded BACKSWING/IMPACT
# α=20° Jacobian-역수 + J5 -75° (망치 위로) — 임팩트 접근 거의 수직
DEFAULT_BACKSWING = np.array([-148.06, 15.80,  94.84, 0.00,  -5.69, -92.76])
DEFAULT_IMPACT    = np.array([-148.06, 25.58, 101.30, 0.00,  53.10, -92.76])
DEFAULT_VEL       = 225.0   # M1013 J5/J6 SPEC
DEFAULT_ACC       = 500.0   # J5/J6 SPEC
# T 자동: J5 swing 58.79° vel=225 acc=500 → triangular T ≈ √(4·58.79/500) = 0.69s
DEFAULT_T_SWING   = 0.69

NAIL_BASE   = np.array([-0.65, -0.45, 0.05])
NAIL_HEAD_Z = 0.10
BENCH_Z     = 0.05


def proj(p, eh):
    return float(eh[0]*p[0] + eh[1]*p[1]), float(p[2])


def trapezoidal(q0, q1, T, dt=0.01, acc_deg=500.0):
    """Per-joint trapezoidal interpolation (deg, deg/s²).  movej firmware 근사."""
    n = int(round(T / dt)) + 1
    qs = np.zeros((n, 6))
    for j in range(6):
        d = q1[j] - q0[j]
        sign = 1.0 if d >= 0 else -1.0
        D = abs(d)
        if D < 1e-9:
            qs[:, j] = q0[j]
            continue
        # solve D = v_p*T - v_p²/acc  → v_p² - acc*T*v_p + acc*D = 0
        disc = (acc_deg*T)**2 - 4*acc_deg*D
        if disc < 0:
            v_p = D / T   # triangular fallback (no cruise)
        else:
            v_p = (acc_deg*T - np.sqrt(disc)) / 2
        t_a = v_p / acc_deg
        t_c = T - 2*t_a
        for k in range(n):
            t = k*dt
            if t < t_a:
                disp = 0.5 * acc_deg * t*t
            elif t < t_a + t_c:
                disp = 0.5*acc_deg*t_a*t_a + v_p*(t - t_a)
            else:
                tau = T - t
                disp = D - 0.5*acc_deg*tau*tau
            qs[k, j] = q0[j] + sign * disp
    return qs


def main():
    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()
    sid = model.getFrameId("hammer_strike")

    eh = NAIL_BASE[:2] / np.linalg.norm(NAIL_BASE[:2])
    h_nail, _ = proj(NAIL_BASE, eh)

    # ── default (movej) trajectory ──
    default_qs = trapezoidal(DEFAULT_BACKSWING, DEFAULT_IMPACT,
                              T=DEFAULT_T_SWING, acc_deg=DEFAULT_ACC)
    default_qs_rad = np.deg2rad(default_qs)

    # ── optimal (iLQR) trajectory ──
    with open(OPTIMAL_CSV) as f:
        r = csv.reader(f); header = next(r)
        # 6-col format (J1..J6) or 13-col (t,J1..J6,J1_dot..J6_dot) 자동 감지
        is_13col = header[0].strip().startswith('t')
        sl = slice(1, 7) if is_13col else slice(0, 6)
        optimal_qs = np.array([[float(x) for x in row[sl]] for row in r if row])
    optimal_qs_rad = np.deg2rad(optimal_qs)

    # FK 로 hammer_strike world position trajectory
    def fk_path(qs_rad):
        hs, zs, vzs = [], [], []
        prev_z = None
        for q in qs_rad:
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            p = data.oMf[sid].translation
            h, z = proj(p, eh)
            hs.append(h); zs.append(z)
            if prev_z is not None:
                vzs.append((z - prev_z) / 0.01)
            else:
                vzs.append(0.0)
            prev_z = z
        return np.array(hs), np.array(zs), np.array(vzs)

    d_h, d_z, d_vz = fk_path(default_qs_rad)
    o_h, o_z, o_vz = fk_path(optimal_qs_rad)

    # ── Plot ──
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Panel 1: side view trajectory
    ax1.fill_between([-0.4, 1.2], -0.05, BENCH_Z, color='peru', alpha=0.3)
    ax1.plot([-0.4, 1.2], [BENCH_Z]*2, color='saddlebrown', lw=2)
    ax1.plot([h_nail, h_nail], [BENCH_Z, NAIL_HEAD_Z], color='dimgray', lw=3)
    ax1.plot([h_nail], [NAIL_HEAD_Z], 'ko', ms=12, zorder=5, label='Nail')

    ax1.plot(o_h, o_z, '-', color='crimson', lw=2.5, alpha=0.9,
             label='optimal — iLQR 3-DOF (101 frame, 1s, swing_traj_optimal.csv)')
    ax1.plot(o_h[::10], o_z[::10], 'o', color='crimson', ms=4)

    ax1.plot(d_h, d_z, '-', color='forestgreen', lw=2.5, alpha=0.9,
             label=f'default — movej (α=20°+J5-75°, vel={DEFAULT_VEL:.0f} acc={DEFAULT_ACC:.0f}, T≈{DEFAULT_T_SWING}s)')
    ax1.plot(d_h[::10], d_z[::10], 's', color='forestgreen', ms=4)

    ax1.plot([o_h[0]], [o_z[0]], '^', color='crimson', ms=12, zorder=6)
    ax1.plot([o_h[-1]], [o_z[-1]], 'v', color='crimson', ms=12, zorder=6)
    ax1.plot([d_h[0]], [d_z[0]], '^', color='forestgreen', ms=12, zorder=6)
    ax1.plot([d_h[-1]], [d_z[-1]], 'v', color='forestgreen', ms=12, zorder=6)

    ax1.set_xlabel('h  [base→nail direction]  (m)', fontsize=11)
    ax1.set_ylabel('z  (m)', fontsize=11)
    ax1.set_aspect('equal')
    ax1.set_xlim(0.0, 1.2); ax1.set_ylim(-0.05, 1.2)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=9)
    ax1.set_title('Hammer strike face trajectory (side view, base→nail plane)',
                  fontsize=12, fontweight='bold')

    # Panel 2: impact velocity profile
    t_d = np.arange(len(d_vz)) * 0.01
    t_o = np.arange(len(o_vz)) * 0.01
    ax2.plot(t_o, o_vz, '-', color='crimson', lw=2.5,
             label=f'optimal — iLQR  (impact $v_z$ = {o_vz[-1]:+.2f} m/s)')
    ax2.plot(t_d, d_vz, '-', color='forestgreen', lw=2.5,
             label=f'default — movej (impact $v_z$ = {d_vz[-1]:+.2f} m/s)')
    ax2.axhline(0, color='gray', lw=0.5)
    ax2.set_xlabel('t  (s)', fontsize=11)
    ax2.set_ylabel('strike face  $v_z$  (m/s)', fontsize=11)
    ax2.set_title('Vertical velocity at hammer strike face',
                  fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='lower left', fontsize=10)

    fig.suptitle('Console demo comparison — default (movej) vs optimal (iLQR)',
                 fontsize=13, fontweight='bold', y=1.00)
    fig.tight_layout()
    fig.savefig(OUT, dpi=140, bbox_inches='tight')
    print(f'Saved → {OUT}')
    print()
    print(f'임팩트 시점 hammer strike face vertical velocity:')
    print(f'  optimal (iLQR):    v_z = {o_vz[-1]:+.2f} m/s')
    print(f'  default (movej):   v_z = {d_vz[-1]:+.2f} m/s   ← firmware trapezoidal 끝점')


if __name__ == '__main__':
    main()
