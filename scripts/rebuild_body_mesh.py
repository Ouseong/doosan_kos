#!/usr/bin/env python3
"""Idempotently rebuild the gripper body visual+collision mesh:
  1. load gripper_body_camera_urdf.stl  (mm → m)
  2. shift -0.0202 m in Y so the flange face sits at Y=0 (matches old body)
  3. append a cylinder spacer (top circle, +Y toward robot arm)

Usage:
  python3 scripts/rebuild_body_mesh.py            # default cylinder height 20mm
  python3 scripts/rebuild_body_mesh.py --h 15     # cylinder height 15mm
  python3 scripts/rebuild_body_mesh.py --h 0      # no cylinder

Because it rebuilds from the STL each run, re-running with a new --h does
NOT stack cylinders — it always produces body = STL + one cylinder.
Restart the Isaac stack afterward for the change to load.
"""
import argparse
import struct

import numpy as np
from pxr import Usd, UsdGeom, Vt, Gf

USD_PATH = "/home/oem/Desktop/doosan_kos/usd/parts/gripper_assembly_physics.usd"
STL_PATH = "/home/oem/Desktop/doosan_kos/usd/parts/gripper_body_camera_urdf.stl"
SCALE = 0.001          # mm → m
Y_SHIFT = -0.0202      # bring flange face to Y=0

# cylinder geometry (gripper top circle)
RADIUS = 0.040         # 80mm diameter
CX, CZ = -0.0001, -0.0155
SEGMENTS = 48


def parse_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        tris = []
        for _ in range(n):
            f.read(12)
            v = [struct.unpack("<3f", f.read(12)) for _ in range(3)]
            f.read(2)
            tris.append(v)
    flat = np.array([v for tri in tris for v in tri], dtype=np.float32)
    unique, inverse = np.unique(flat, axis=0, return_inverse=True)
    return unique * SCALE, inverse.reshape(-1, 3).astype(np.int32)


def make_cylinder(height):
    y0, y1 = 0.0, height
    pts = []
    for y in (y0, y1):
        for i in range(SEGMENTS):
            a = 2 * np.pi * i / SEGMENTS
            pts.append((CX + RADIUS * np.cos(a), y, CZ + RADIUS * np.sin(a)))
    c_bot = len(pts); pts.append((CX, y0, CZ))
    c_top = len(pts); pts.append((CX, y1, CZ))
    pts = np.array(pts, dtype=np.float32)
    faces = []
    for i in range(SEGMENTS):
        j = (i + 1) % SEGMENTS
        faces.append((i, j, SEGMENTS + j))
        faces.append((i, SEGMENTS + j, SEGMENTS + i))
        faces.append((c_bot, j, i))
        faces.append((c_top, SEGMENTS + i, SEGMENTS + j))
    return pts, np.array(faces, dtype=np.int32)


def set_mesh(stage, path, pts, faces):
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
    mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p.tolist()) for p in pts]))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(faces.flatten().tolist()))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(faces)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--h", type=float, default=20.0, help="cylinder height in mm")
    args = ap.parse_args()
    h = args.h / 1000.0

    pts, faces = parse_stl(STL_PATH)
    pts[:, 1] += Y_SHIFT
    print(f"[1] STL+shift: {len(pts)} pts, {len(faces)} faces  "
          f"Y=[{pts[:,1].min():.4f}, {pts[:,1].max():.4f}]")

    if h > 0:
        cpts, cfaces = make_cylinder(h)
        off = len(pts)
        pts = np.vstack([pts, cpts])
        faces = np.vstack([faces, cfaces + off])
        print(f"[2] +cylinder h={args.h:.0f}mm: now {len(pts)} pts, {len(faces)} faces  "
              f"Y max={pts[:,1].max():.4f}")
    else:
        print("[2] no cylinder")

    stage = Usd.Stage.Open(USD_PATH)
    set_mesh(stage, "/World/Gripper/body/visual", pts, faces)
    set_mesh(stage, "/World/Gripper/body/collision", pts, faces)
    stage.Save()
    print("[3] Saved. Restart Isaac stack to load.")


if __name__ == "__main__":
    main()
