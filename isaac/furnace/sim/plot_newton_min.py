"""newton_plate_drop_min.py 결과(trajectory.npz)의 측면 단면 그림. 시뮬레이션 데이터 플롯이며 렌더가 아니다.

  ~/venvs/furnace_newton/bin/python isaac/furnace/sim/plot_newton_min.py output/furnace/phase0/newton_min
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

d = sys.argv[1]
z = np.load(f"{d}/trajectory.npz")
snaps = sorted(int(k.split("_")[1]) for k in z.files if k.startswith("snap_"))
hx, hy = z["tank_half"]
H = float(z["tank_height"])
lx, ly, lz = z["plate_size"]
P, Q = z["plate_pos"], z["plate_quat"]


def rot(q, v):
    u, w = q[:3], q[3]
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


fig, axs = plt.subplots(1, len(snaps), figsize=(4 * len(snaps), 4.2), sharey=True)
for ax, f in zip(np.atleast_1d(axs), snaps):
    x = z[f"snap_{f}"]
    m = np.abs(x[:, 1]) < 0.03                      # y≈0 단면
    out = (np.abs(x[:, 0]) > hx) | (np.abs(x[:, 1]) > hy) | (x[:, 2] < 0)
    ax.scatter(x[m & ~out, 0], x[m & ~out, 2], s=0.3, c="tab:blue")
    ax.scatter(x[m & out, 0], x[m & out, 2], s=2, c="tab:red")
    ax.plot([-hx, -hx, hx, hx], [H, 0, 0, H], "k-", lw=2)
    if len(P):
        cs = np.array([[-lx / 2, 0, 0], [lx / 2, 0, 0]])
        pts = np.array([rot(Q[f], c) + P[f] for c in cs])
        ax.plot(pts[:, 0], pts[:, 2], "-", c="darkorange", lw=3)
    ax.set_title(f"frame {f}  (t={(f + 1) / 100:.2f}s)")
    ax.set_aspect("equal"); ax.set_xlim(-0.45, 0.45); ax.set_ylim(-0.1, 0.8)
fig.suptitle(f"{d}: y≈0 slice, blue=inside tank, red=outside (incl. leak), orange=metal_plate  [Newton MPM data plot, not a render]",
             fontsize=8)
fig.tight_layout()
fig.savefig(f"{d}/side_view.png", dpi=110)
print(f"{d}/side_view.png")
