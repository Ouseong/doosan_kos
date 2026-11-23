"""run_fluid_plate_test.py 의 <tag>_trajectory.npz 측면 단면 그림. 시뮬레이션 데이터 플롯이며 렌더가 아니다.

  ~/venvs/furnace_newton/bin/python isaac/furnace/sim/plot_fluid.py <npz> [<npz> ...] --out side_view.png
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def rot(q, v):
    u, w = q[:3], q[3]
    v = np.asarray(v, dtype=float)
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


ap = argparse.ArgumentParser()
ap.add_argument("npz", nargs="+")
ap.add_argument("--out", required=True)
ap.add_argument("--times", type=float, nargs="+", default=[0.0, 0.2, 0.35, 1.45])
a = ap.parse_args()
fig, axs = plt.subplots(len(a.npz), len(a.times), figsize=(3.6 * len(a.times), 3.4 * len(a.npz)), squeeze=False)
for row, path in enumerate(a.npz):
    z = np.load(path)
    P = z["particles_sub"].astype(np.float32)
    every, fps = int(z["every_n_frames"]), float(z["fps"])
    R, H, T, melt = float(z["radius"]), float(z["depth"]), float(z["wall"]), float(z["melt"])
    lx, ly, lz = z["plate_size"]
    plate = z["plate"]
    for col, t in enumerate(a.times):
        ax = axs[row, col]
        k = min(len(P) - 1, int(round(t * fps / every)))
        f = k * every
        x = P[k]
        m = np.abs(x[:, 1]) < 0.03
        ax.scatter(x[m, 0], x[m, 2], s=0.4, c="tab:blue")
        ax.plot([-R, -R, R, R], [H, 0, 0, H], "k-", lw=2)
        ax.axhline(melt, color="gray", lw=0.5, ls="--")
        if len(plate):
            p, q = plate[f, :3], plate[f, 3:]
            cs = np.array([rot(q, c) + p for c in ([-lx / 2, 0, 0], [lx / 2, 0, 0])])
            ax.plot(cs[:, 0], cs[:, 2], "-", c="darkorange", lw=3)
        ax.set_aspect("equal"); ax.set_xlim(-R - 0.1, R + 0.1); ax.set_ylim(-0.05, H + 0.15)
        ax.set_title(f"{os.path.basename(path).replace('_trajectory.npz', '')}  t={(f + 1) / fps:.2f}s", fontsize=8)
fig.suptitle("Newton MPM (isothermal water-like fluid, NOT molten metal) — y≈0 slice. blue=fluid particles, "
             "orange=metal_plate. Simulation data plot, not a render.", fontsize=8)
fig.tight_layout()
fig.savefig(a.out, dpi=100)
print(a.out)
