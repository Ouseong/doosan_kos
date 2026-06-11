#!/usr/bin/env python3
"""Add a cylinder spacer at the top of the gripper body, filling the gap
between the gripper flange (Y=0) and the robot arm.

Cylinder:
  - diameter 80mm  (matches gripper top circle)
  - height   30mm
  - axis along +Y (toward robot), spanning Y=[0, +0.030]
  - centered at X=-0.0001, Z=-0.0155  (gripper top-circle center)

The cylinder mesh is merged into body/visual and body/collision so it
moves with the body and shares its (disabled) collision treatment.
Run again to re-add only if the body mesh has been freshly replaced.
"""
import numpy as np
from pxr import Usd, UsdGeom, Vt, Gf

USD_PATH = "/home/oem/Desktop/doosan_kos/usd/parts/gripper_assembly_physics.usd"

# Top-circle geometry (meters)
RADIUS = 0.040          # 80mm diameter
HEIGHT = 0.030          # 30mm tall
CX, CZ = -0.0001, -0.0155
Y0, Y1 = 0.0, HEIGHT    # span +Y toward robot
SEGMENTS = 48


def make_cylinder():
    """Return (points Nx3, faces Mx3) for a capped cylinder along +Y."""
    pts = []
    # bottom ring (Y0), top ring (Y1)
    for y in (Y0, Y1):
        for i in range(SEGMENTS):
            ang = 2 * np.pi * i / SEGMENTS
            pts.append((CX + RADIUS * np.cos(ang), y, CZ + RADIUS * np.sin(ang)))
    c_bot = len(pts); pts.append((CX, Y0, CZ))
    c_top = len(pts); pts.append((CX, Y1, CZ))
    pts = np.array(pts, dtype=np.float32)

    faces = []
    bot = lambda i: i
    top = lambda i: SEGMENTS + i
    for i in range(SEGMENTS):
        j = (i + 1) % SEGMENTS
        # side (two triangles per quad)
        faces.append((bot(i), bot(j), top(j)))
        faces.append((bot(i), top(j), top(i)))
        # bottom cap
        faces.append((c_bot, bot(j), bot(i)))
        # top cap
        faces.append((c_top, top(i), top(j)))
    return pts, np.array(faces, dtype=np.int32)


def append_to_mesh(stage, path, cyl_pts, cyl_faces):
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float32)
    fvi = np.array(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
    fvc = np.array(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int32)

    offset = len(pts)
    new_pts = np.vstack([pts, cyl_pts])
    new_fvi = np.concatenate([fvi, (cyl_faces + offset).flatten()])
    new_fvc = np.concatenate([fvc, np.full(len(cyl_faces), 3, dtype=np.int32)])

    mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p.tolist()) for p in new_pts]))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(new_fvi.tolist()))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(new_fvc.tolist()))
    print(f"  {path}: +{len(cyl_pts)} pts, +{len(cyl_faces)} faces "
          f"(now {len(new_pts)} pts, {len(new_fvc)} faces)")


def main():
    cyl_pts, cyl_faces = make_cylinder()
    print(f"[1] Cylinder: r={RADIUS*1000:.0f}mm h={HEIGHT*1000:.0f}mm "
          f"Y=[{Y0:.3f},{Y1:.3f}] center=({CX*1000:.1f},{CZ*1000:.1f})mm "
          f"→ {len(cyl_pts)} pts, {len(cyl_faces)} faces")

    stage = Usd.Stage.Open(USD_PATH)
    print("[2] Merging into body/visual...")
    append_to_mesh(stage, "/World/Gripper/body/visual", cyl_pts, cyl_faces)
    print("[3] Merging into body/collision...")
    append_to_mesh(stage, "/World/Gripper/body/collision", cyl_pts, cyl_faces)
    print("[4] Saving...")
    stage.Save()
    print("Done.")


if __name__ == "__main__":
    main()
