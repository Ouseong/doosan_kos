#!/usr/bin/env python3
"""Replace gripper body visual+collision mesh in gripper_assembly_physics.usd
with data from gripper_body_camera_urdf.stl (mm → m scale).

Only /World/Gripper/body/visual and /World/Gripper/body/collision are touched.
"""
import os
import struct
import sys

import numpy as np

USD_PATH = "/home/oem/Desktop/doosan_kos/usd/parts/gripper_assembly_physics.usd"
STL_PATH = "/home/oem/Desktop/doosan_kos/usd/parts/gripper_body_camera_urdf.stl"
SCALE = 0.001  # mm → m


def parse_stl_binary(path):
    """Return (vertices Nx3 float32, faces Mx3 int32) from binary STL."""
    with open(path, "rb") as f:
        f.read(80)  # header
        n_tri = struct.unpack("<I", f.read(4))[0]
        print(f"  STL triangles: {n_tri}")
        tris = []
        for _ in range(n_tri):
            f.read(12)  # normal (skip)
            v0 = struct.unpack("<3f", f.read(12))
            v1 = struct.unpack("<3f", f.read(12))
            v2 = struct.unpack("<3f", f.read(12))
            f.read(2)   # attribute
            tris.append((v0, v1, v2))
    # flat list of all vertices (duplicated per triangle)
    flat = np.array([(v[0], v[1], v[2]) for tri in tris for v in tri], dtype=np.float32)
    # deduplicate
    unique, inverse = np.unique(flat, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    return unique * SCALE, faces


def parse_stl(path):
    with open(path, "rb") as f:
        header = f.read(80)
    if header[:5] == b"solid" and b"\n" in header:
        raise NotImplementedError("ASCII STL — convert to binary first")
    return parse_stl_binary(path)


def update_mesh_prim(stage, prim_path, points_np, faces_np):
    from pxr import UsdGeom, Vt, Gf

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"  WARNING: prim not found: {prim_path}")
        return

    mesh = UsdGeom.Mesh(prim)

    pts_vt = Vt.Vec3fArray([Gf.Vec3f(*p.tolist()) for p in points_np])
    fvi_vt = Vt.IntArray(faces_np.flatten().tolist())
    fvc_vt = Vt.IntArray([3] * len(faces_np))

    mesh.GetPointsAttr().Set(pts_vt)
    mesh.GetFaceVertexIndicesAttr().Set(fvi_vt)
    mesh.GetFaceVertexCountsAttr().Set(fvc_vt)

    print(f"  {prim_path}: {len(points_np)} pts, {len(faces_np)} faces")


def main():
    from pxr import Usd

    print("[1] Parsing STL...")
    pts, faces = parse_stl(STL_PATH)
    print(f"  Unique vertices: {len(pts)}, Triangles: {len(faces)}")
    print(f"  X range: [{pts[:,0].min():.4f}, {pts[:,0].max():.4f}]")
    print(f"  Y range: [{pts[:,1].min():.4f}, {pts[:,1].max():.4f}]")
    print(f"  Z range: [{pts[:,2].min():.4f}, {pts[:,2].max():.4f}]")

    print("[2] Opening USD stage...")
    stage = Usd.Stage.Open(USD_PATH)

    print("[3] Updating body/visual mesh...")
    update_mesh_prim(stage, "/World/Gripper/body/visual", pts, faces)

    print("[4] Updating body/collision mesh...")
    update_mesh_prim(stage, "/World/Gripper/body/collision", pts, faces)

    print("[5] Saving USD...")
    stage.Save()
    print("Done.")


if __name__ == "__main__":
    main()
