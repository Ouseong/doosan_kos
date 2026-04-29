import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import art3d

path = r"C:\Users\oskwo\Desktop\doosan_kos\usd\hammer\hammer.stl"

with open(path, "rb") as f:
    f.read(80)
    n_tri = struct.unpack("<I", f.read(4))[0]
    tris = []
    for _ in range(n_tri):
        f.read(12)
        tri = [struct.unpack("<fff", f.read(12)) for _ in range(3)]
        f.read(2)
        tris.append(tri)

tris  = np.array(tris)
verts = tris.reshape(-1, 3)
mn, mx = verts.min(0), verts.max(0)
sz  = mx - mn
ctr = (mn + mx) / 2

# ── Keypoints ──────────────────────────────────────────────────────────────
GRIP_PT    = np.array([0.0, 0.0, 0.0])          # gripper clamps here
STRIKE_PT  = np.array([0.0, -87.0, -27.0])      # Z_min face centre
STRIKE_R   = 15.0                                # radius of strike circle (mm)

# Strike circle in 3D (flat in XY at Z = -27)
theta = np.linspace(0, 2 * np.pi, 64)
circle_x = STRIKE_PT[0] + STRIKE_R * np.cos(theta)
circle_y = STRIKE_PT[1] + STRIKE_R * np.sin(theta)
circle_z = np.full_like(theta, STRIKE_PT[2])

pad = 5
lims = [(mn[i] - pad, mx[i] + pad) for i in range(3)]
sx, sy, sz_r = sz[0], sz[1], sz[2]

# ── colour mesh by Y (handle warm, head dark) ──────────────────────────────
y_vals = tris[:, :, 1].mean(axis=1)
y_norm = (y_vals - y_vals.min()) / (y_vals.max() - y_vals.min() + 1e-9)
face_colors = plt.cm.YlOrBr(0.25 + 0.55 * y_norm)

def draw(ax, elev, azim, title, show_legend=False):
    poly = Poly3DCollection(tris, alpha=0.88, linewidth=0)
    poly.set_facecolor(face_colors)
    poly.set_edgecolor("none")
    ax.add_collection3d(poly)

    # Strike circle
    ax.plot(circle_x, circle_y, circle_z,
            color="#ff2222", linewidth=2.0, zorder=12, label="Strike circle (r=15mm)")
    # Strike centre
    ax.scatter(*STRIKE_PT, color="#ff2222", s=120, zorder=15,
               marker="x", linewidths=2.5)
    # Grip point
    ax.scatter(*GRIP_PT, color="#00e5ff", s=100, zorder=15,
               marker="o", label="Grip origin (0,0,0)")

    # Annotation arrow from label to strike centre (only on perspective view)
    if show_legend:
        ax.text(STRIKE_PT[0] + 20, STRIKE_PT[1] + 5, STRIKE_PT[2] - 8,
                "Strike\n(0, -87, -27) mm", color="#ff6666",
                fontsize=7.5, ha="left", va="top")
        ax.text(GRIP_PT[0] + 18, GRIP_PT[1] + 2, GRIP_PT[2] + 6,
                "Grip\n(0, 0, 0) mm", color="#00e5ff",
                fontsize=7.5, ha="left", va="bottom")

    ax.set_xlim(*lims[0])
    ax.set_ylim(*lims[1])
    ax.set_zlim(*lims[2])
    ax.set_box_aspect([sx, sy, sz_r])

    ax.set_xlabel("X (mm)", color="#aaa", fontsize=8, labelpad=2)
    ax.set_ylabel("Y (mm)", color="#aaa", fontsize=8, labelpad=2)
    ax.set_zlabel("Z (mm)", color="#aaa", fontsize=8, labelpad=2)
    ax.tick_params(colors="#777", labelsize=7)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor("#1e1e30")
    ax.grid(True, color="#1e1e30", linewidth=0.4)
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, color="white", fontsize=10, pad=4)

fig = plt.figure(figsize=(22, 7), facecolor="#13131f")

views = [
    ("Side  (XZ plane)",    0, -90),
    ("Top   (XY plane)",   90, -90),
    ("Front (YZ plane)",    0,   0),
    ("Perspective",        22,  -50),
]

for col, (title, elev, azim) in enumerate(views, 1):
    ax = fig.add_subplot(1, 4, col, projection="3d", facecolor="#0f0f1e")
    draw(ax, elev, azim, title, show_legend=(col == 4))

# Shared legend
red_line  = mpatches.Patch(color="#ff2222", label=f"Strike circle  (0, -87, -27) mm  r=15mm")
cyan_dot  = mpatches.Patch(color="#00e5ff", label=f"Grip origin    (0,  0,   0) mm")
fig.legend(handles=[red_line, cyan_dot],
           loc="lower center", ncol=2, fontsize=9,
           facecolor="#1a1a2e", edgecolor="#334", labelcolor="white",
           bbox_to_anchor=(0.5, 0.01))

fig.suptitle(
    f"hammer.stl  (rubber mallet)   X={sz[0]:.1f}  Y={sz[1]:.1f}  Z={sz[2]:.1f} mm   "
    f"triangles={n_tri}",
    color="white", fontsize=11, y=1.01,
)
plt.tight_layout(pad=1.5, rect=[0, 0.07, 1, 1])
out = r"C:\Users\oskwo\Desktop\doosan_kos\usd\hammer\hammer_preview.png"
plt.savefig(out, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor())
print(f"saved: {out}")
