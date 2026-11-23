"""
절차적 USD 생성 공용 함수 (pxr 만 사용. Isaac Sim 번들 python 에서 실행).

- 모든 길이 m, Z up, metersPerUnit 1.0
- 시각 형상과 충돌 형상은 따로 만든다: visual=True 인 prim 에는 충돌 API 를 붙이지 않는다.
- 텍스처는 PIL 로 직접 만든 PNG (외부 자산 아님) → usd/generated/furnace/textures/
"""

from __future__ import annotations

import math
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
GEN = os.path.join(ROOT, "usd", "generated", "furnace")
TEX = os.path.join(GEN, "textures")


def _sp(path):
    """prim 경로에 쓸 수 없는 문자('-', '.') 를 바꾼다 (예: end-1 → endm1)."""
    head, _, tail = path.rpartition("/")
    return f"{head}/{tail.replace('-', 'm').replace('.', '_')}" if head else path


def new_stage(path, default="World"):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if os.path.exists(path):
        os.remove(path)
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = st.DefinePrim(f"/{default}", "Xform")
    st.SetDefaultPrim(root)
    return st


def xform(st, path, t=(0, 0, 0), rpy_deg=(0, 0, 0), s=None):
    x = UsdGeom.Xform.Define(st, _sp(path))
    _set_xf(x, t, rpy_deg, s)
    return x


def _set_xf(prim, t, rpy_deg, s=None):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, t)))
    if any(rpy_deg):
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*map(float, rpy_deg)))
    if s is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))


_MAT_CACHE: dict = {}


def material(st, name, color=(0.5, 0.5, 0.5), rough=0.6, metal=0.0, emissive=None, emissive_intensity=1.0,
             texture=None, tex_scale=(1, 1), opacity=1.0, scope=None):
    # reference 하면 default prim 아래만 따라오므로 재질도 default prim 아래에 둔다
    if scope is None:
        scope = f"{st.GetDefaultPrim().GetPath()}/Looks"
    key = (st.GetRootLayer().identifier, name)
    if key in _MAT_CACHE:
        return _MAT_CACHE[key]
    if not st.GetPrimAtPath(scope):
        st.DefinePrim(scope, "Scope")
    path = f"{scope}/{name}"
    m = UsdShade.Material.Define(st, _sp(path))
    sh = UsdShade.Shader.Define(st, path + "/Surf")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
    if opacity < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    if emissive is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*[float(c) * emissive_intensity for c in emissive]))
    if texture:
        rd = UsdShade.Shader.Define(st, path + "/UV")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tr = UsdShade.Shader.Define(st, path + "/Xf")
        tr.CreateIdAttr("UsdTransform2d")
        tr.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.CreateOutput("result", Sdf.ValueTypeNames.Float2))
        tr.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*map(float, tex_scale)))
        tx = UsdShade.Shader.Define(st, path + "/Tex")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(os.path.relpath(texture, os.path.dirname(st.GetRootLayer().realPath)))
        tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(tr.CreateOutput("result", Sdf.ValueTypeNames.Float2))
        rgb = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, color)))
    m.CreateSurfaceOutput().ConnectToSource(sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    _MAT_CACHE[key] = m
    return m


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim() if hasattr(prim, "GetPrim") else prim).Bind(mat)


def box_mesh(st, path, size, t=(0, 0, 0), rpy_deg=(0, 0, 0), mat=None, uv_scale=1.0, collide=False):
    """size = 전체 치수 (x,y,z). UV 는 면 크기(m) × uv_scale 로 줘서 텍스처가 늘어나지 않게 한다."""
    hx, hy, hz = [float(v) / 2 for v in size]
    P, N, UV, idx = [], [], [], []
    faces = [((1, 0, 0), (0, 1, 0), (0, 0, 1), hx, hy, hz), ((-1, 0, 0), (0, -1, 0), (0, 0, 1), hx, hy, hz),
             ((0, 1, 0), (-1, 0, 0), (0, 0, 1), hy, hx, hz), ((0, -1, 0), (1, 0, 0), (0, 0, 1), hy, hx, hz),
             ((0, 0, 1), (1, 0, 0), (0, 1, 0), hz, hx, hy), ((0, 0, -1), (-1, 0, 0), (0, 1, 0), hz, hx, hy)]
    for n, u, v, d, a, b in faces:
        n, u, v = np.array(n), np.array(u), np.array(v)
        c = n * d
        base = len(P)
        for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            P.append(c + su * a * u + sv * b * v)
            N.append(n)
            UV.append(((su + 1) * a * uv_scale, (sv + 1) * b * uv_scale))
        idx += [base, base + 1, base + 2, base + 3]
    m = UsdGeom.Mesh.Define(st, _sp(path))
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in P])
    m.CreateNormalsAttr([Gf.Vec3f(*map(float, n)) for n in N])
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set([Gf.Vec2f(*map(float, uv)) for uv in UV])
    m.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    _set_xf(m, t, rpy_deg)
    if mat is not None:
        bind(m, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    return m


def cylinder(st, path, r, h, t=(0, 0, 0), rpy_deg=(0, 0, 0), mat=None, axis="Z", collide=False):
    c = UsdGeom.Cylinder.Define(st, _sp(path))
    c.CreateRadiusAttr(float(r)); c.CreateHeightAttr(float(h)); c.CreateAxisAttr(axis)
    e = [float(r), float(r), float(h) / 2]
    if axis == "X":
        e = [float(h) / 2, float(r), float(r)]
    elif axis == "Y":
        e = [float(r), float(h) / 2, float(r)]
    c.CreateExtentAttr([Gf.Vec3f(*[-v for v in e]), Gf.Vec3f(*e)])
    _set_xf(c, t, rpy_deg)
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def ring_mesh(st, path, r_in, r_out, z0, z1, seg=64, t=(0, 0, 0), mat=None, uv_scale=1.0):
    """두꺼운 원통 고리 (속 빈 관). 시각용. 위/아래 면, 안/바깥 벽."""
    a = np.linspace(0, 2 * np.pi, seg + 1)
    P, idx, cnt, UV = [], [], [], []

    def quad(p0, p1, p2, p3, uvs):
        b = len(P)
        P.extend([p0, p1, p2, p3]); UV.extend(uvs)
        idx.extend([b, b + 1, b + 2, b + 3]); cnt.append(4)

    for i in range(seg):
        c0, s0, c1, s1 = np.cos(a[i]), np.sin(a[i]), np.cos(a[i + 1]), np.sin(a[i + 1])
        L0, L1 = a[i] * r_out * uv_scale, a[i + 1] * r_out * uv_scale
        hz = (z1 - z0) * uv_scale
        quad((r_out * c0, r_out * s0, z0), (r_out * c1, r_out * s1, z0), (r_out * c1, r_out * s1, z1),
             (r_out * c0, r_out * s0, z1), [(L0, 0), (L1, 0), (L1, hz), (L0, hz)])
        quad((r_in * c1, r_in * s1, z0), (r_in * c0, r_in * s0, z0), (r_in * c0, r_in * s0, z1),
             (r_in * c1, r_in * s1, z1), [(L1, 0), (L0, 0), (L0, hz), (L1, hz)])
        w = (r_out - r_in) * uv_scale
        quad((r_in * c0, r_in * s0, z1), (r_out * c0, r_out * s0, z1), (r_out * c1, r_out * s1, z1),
             (r_in * c1, r_in * s1, z1), [(L0, 0), (L0, w), (L1, w), (L1, 0)])
        quad((r_in * c1, r_in * s1, z0), (r_out * c1, r_out * s1, z0), (r_out * c0, r_out * s0, z0),
             (r_in * c0, r_in * s0, z0), [(L1, 0), (L1, w), (L0, w), (L0, 0)])
    m = UsdGeom.Mesh.Define(st, _sp(path))
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in P])
    m.CreateFaceVertexCountsAttr(cnt)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set([Gf.Vec2f(*map(float, uv)) for uv in UV])
    m.CreateExtentAttr([Gf.Vec3f(-r_out, -r_out, z0), Gf.Vec3f(r_out, r_out, z1)])
    _set_xf(m, t, (0, 0, 0))
    if mat is not None:
        bind(m, mat)
    return m


def pipe_path(st, path, pts, r, mat=None):
    """점 목록을 잇는 직선 배관 (구간마다 원통 + 꺾이는 곳에 구)."""
    xform(st, path)
    pts = [np.array(p, dtype=float) for p in pts]
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        d = b - a
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            continue
        mid = (a + b) / 2
        z = d / L
        # Z 축을 d 방향으로 돌리는 RPY (XYZ 순서) 대신 axis-angle → 쿼터니언 orient op 사용
        c = UsdGeom.Cylinder.Define(st, _sp(f"{path}/seg{i}"))
        c.CreateRadiusAttr(float(r)); c.CreateHeightAttr(L); c.CreateAxisAttr("Z")
        c.CreateExtentAttr([Gf.Vec3f(-r, -r, -L / 2), Gf.Vec3f(r, r, L / 2)])
        xf = UsdGeom.Xformable(c); xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*mid))
        q = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*z)).GetQuat()
        xf.AddOrientOp().Set(Gf.Quatf(q))
        if mat is not None:
            bind(c, mat)
        if 0 < i:
            s = UsdGeom.Sphere.Define(st, _sp(f"{path}/elbow{i}"))
            s.CreateRadiusAttr(float(r) * 1.05)
            s.CreateExtentAttr([Gf.Vec3f(-r * 1.05), Gf.Vec3f(r * 1.05)])
            _set_xf(s, a, (0, 0, 0))
            if mat is not None:
                bind(s, mat)


def stl_mesh(st, path, stl_path, scale=1e-3, mat=None, t=(0, 0, 0), rpy_deg=(0, 0, 0)):
    import struct
    with open(stl_path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        d = np.frombuffer(f.read(n * rec.itemsize), dtype=rec, count=n)
    pts = np.ascontiguousarray(d["v"].reshape(-1, 3))
    from pxr import Vt
    m = UsdGeom.Mesh.Define(st, _sp(path))
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(n, 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(pts), dtype=np.int32)))
    m.CreateSubdivisionSchemeAttr("none")
    lo, hi = pts.min(0), pts.max(0)
    m.CreateExtentAttr([Gf.Vec3f(*lo.tolist()), Gf.Vec3f(*hi.tolist())])
    xf = UsdGeom.Xformable(m); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, t)))
    if any(rpy_deg):
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*map(float, rpy_deg)))
    xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    if mat is not None:
        bind(m, mat)
    return m, n


# ── 절차적 텍스처 (PIL) ─────────────────────────────────────────────────────────
def _noise(h, w, rng, scales=(64, 16, 4)):
    from PIL import Image
    acc = np.zeros((h, w))
    for s in scales:
        small = rng.random((max(2, h // s), max(2, w // s)))
        acc += np.asarray(Image.fromarray((small * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)) / 255.0 * s
    return acc / sum(scales)


def texture(name, kind, size=512, seed=0):
    """kind: concrete | brick | metal_panel | rust | refractory | hazard. 반환: PNG 경로."""
    from PIL import Image, ImageDraw
    os.makedirs(TEX, exist_ok=True)
    p = os.path.join(TEX, f"{name}.png")
    if os.path.exists(p):
        return p
    rng = np.random.default_rng(seed)
    n = _noise(size, size, rng)
    if kind == "concrete":
        base = 0.30 + 0.18 * n
        stains = _noise(size, size, rng, (128, 32)) > 0.62
        base = np.where(stains, base * 0.7, base)
        img = np.stack([base * 0.98, base * 0.97, base * 0.95], -1)
        im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
        d = ImageDraw.Draw(im)
        for i in range(0, size, size // 2):          # 바닥 줄눈
            d.line([(i, 0), (i, size)], fill=(40, 40, 40), width=2)
            d.line([(0, i), (size, i)], fill=(40, 40, 40), width=2)
    elif kind in ("brick", "refractory"):
        im = Image.new("RGB", (size, size), (70, 62, 55))
        d = ImageDraw.Draw(im)
        bw, bh = size // 4, size // 8
        for r in range(8):
            off = (bw // 2) * (r % 2)
            for c in range(-1, 5):
                x0, y0 = c * bw + off + 3, r * bh + 3
                if kind == "brick":
                    col = np.array([125, 58, 38]) * (0.75 + 0.35 * rng.random())
                else:
                    col = np.array([196, 170, 130]) * (0.7 + 0.3 * rng.random())
                d.rectangle([x0, y0, x0 + bw - 6, y0 + bh - 6], fill=tuple(int(v) for v in col))
        arr = np.asarray(im) / 255.0 * (0.8 + 0.3 * n[..., None])
        im = Image.fromarray((np.clip(arr, 0, 1) * 255).astype(np.uint8))
    elif kind == "metal_panel":
        x = np.linspace(0, 8 * np.pi, size)
        rib = 0.5 + 0.5 * np.sign(np.sin(x))[None, :]
        base = 0.22 + 0.06 * rib + 0.12 * n
        grime = _noise(size, size, rng, (128, 32))
        base = base * (0.75 + 0.3 * (1 - np.linspace(0, 1, size)[:, None] * grime))
        img = np.stack([base * 0.95, base * 1.0, base * 1.05], -1)
        im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    elif kind == "rust":
        base = n
        img = np.stack([0.30 + 0.25 * base, 0.18 + 0.12 * base, 0.10 + 0.06 * base], -1)
        im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    elif kind == "hazard":
        im = Image.new("RGB", (size, size), (20, 20, 20))
        d = ImageDraw.Draw(im)
        w = size // 4
        for k in range(-4, 8):
            d.polygon([(k * w, 0), (k * w + w // 2, 0), (k * w + w // 2 + size, size), (k * w + size, size)],
                      fill=(220, 170, 20))
    else:
        raise ValueError(kind)
    im.save(p)
    return p


def rect_light(st, path, w, h, intensity, color, t, rpy_deg=(0, 0, 0)):
    l = UsdLux.RectLight.Define(st, _sp(path))
    l.CreateWidthAttr(float(w)); l.CreateHeightAttr(float(h))
    l.CreateIntensityAttr(float(intensity)); l.CreateColorAttr(Gf.Vec3f(*map(float, color)))
    _set_xf(l, t, rpy_deg)
    return l


def sphere_light(st, path, r, intensity, color, t):
    l = UsdLux.SphereLight.Define(st, _sp(path))
    l.CreateRadiusAttr(float(r)); l.CreateIntensityAttr(float(intensity)); l.CreateColorAttr(Gf.Vec3f(*map(float, color)))
    _set_xf(l, t, (0, 0, 0))
    return l


def disk_light(st, path, r, intensity, color, t, rpy_deg=(0, 0, 0)):
    l = UsdLux.DiskLight.Define(st, _sp(path))
    l.CreateRadiusAttr(float(r)); l.CreateIntensityAttr(float(intensity)); l.CreateColorAttr(Gf.Vec3f(*map(float, color)))
    _set_xf(l, t, rpy_deg)
    return l
