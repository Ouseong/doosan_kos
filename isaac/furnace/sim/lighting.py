"""
조명 프리셋. Isaac Sim 런타임에서 호출 (pxr + carb.settings).

dark_foundry: 어두운 청회색 dome + 희미한 천장 조명 + 약한 보조광. 용탕 발광·용해로 내부 주황 광원은
              build_furnace.py 가 용해로 USD 안에 넣는다 (실제 광원이라 판·그리퍼 하부·로봇·바닥을 비춘다).
debug_bright: 형상 확인용 밝은 조명.

자동 노출: /rtx/post/histogram/enabled = False 로 끄고 고정 노출 (/rtx/post/tonemap/filmIso, exposureTime,
fNumber). 이 키 이름은 설치본 Isaac Sim 5.1 확장(omni.kit.viewport.menubar.camera auto_exposure.py,
omni.kit.waypoint.core renderer.py)에서 확인했다.
"""

from pxr import Gf, UsdGeom, UsdLux

EXPOSURE_KEYS = ["/rtx/post/histogram/enabled", "/rtx/post/tonemap/filmIso", "/rtx/post/tonemap/exposureTime",
                 "/rtx/post/tonemap/fNumber", "/rtx/post/tonemap/op"]


def read_exposure_settings():
    import carb.settings
    s = carb.settings.get_settings()
    return {k: s.get(k) for k in EXPOSURE_KEYS}


def set_fixed_exposure(film_iso, shutter_s, f_number):
    import carb.settings
    s = carb.settings.get_settings()
    s.set("/rtx/post/histogram/enabled", False)
    s.set("/rtx/post/tonemap/filmIso", float(film_iso))
    s.set("/rtx/post/tonemap/exposureTime", float(shutter_s))
    s.set("/rtx/post/tonemap/fNumber", float(f_number))


def apply(stage, cfg, preset="dark_foundry", root="/World/Lights"):
    lay = cfg["layout"]
    L = cfg["lighting"]
    if stage.GetPrimAtPath(root):
        stage.RemovePrim(root)
    UsdGeom.Xform.Define(stage, root)
    if preset == "debug_bright":
        d = UsdLux.DomeLight.Define(stage, root + "/dome")
        d.CreateIntensityAttr(1000.0)
        UsdLux.DistantLight.Define(stage, root + "/sun").CreateIntensityAttr(3000.0)
        return
    d = UsdLux.DomeLight.Define(stage, root + "/dome")
    d.CreateIntensityAttr(float(L["dome_intensity"]))
    d.CreateColorAttr(Gf.Vec3f(*L["dome_color"]))
    x0, x1 = lay["hall_x"]
    y0, y1 = lay["hall_y"]
    H = lay["hall_height"]
    k = 0
    for x in (x0 + 3.0, (x0 + x1) / 2, x1 - 3.0):
        for y in (y0 + 3.0, y1 - 3.0):
            r = UsdLux.RectLight.Define(stage, root + f"/ceiling{k}")
            r.CreateWidthAttr(1.2); r.CreateHeightAttr(0.4)
            r.CreateIntensityAttr(float(L["ceiling_rect_intensity"]))
            r.CreateColorAttr(Gf.Vec3f(*L["ceiling_rect_color"]))
            xf = UsdGeom.Xformable(r); xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(x, y, H - 0.6))
            k += 1
    # 약한 보조광 (장비가 검게 묻히지 않도록) — 로봇 쪽 위에서
    f = UsdLux.RectLight.Define(stage, root + "/fill")
    f.CreateWidthAttr(2.0); f.CreateHeightAttr(2.0)
    f.CreateIntensityAttr(float(L["fill_intensity"]))
    f.CreateColorAttr(Gf.Vec3f(0.7, 0.78, 0.9))
    xf = UsdGeom.Xformable(f); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(-1.5, -2.0, 4.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(30.0, -20.0, 0.0))
    # 작업등 (로봇·그리퍼 위, 약한 백색)
    w = UsdLux.SphereLight.Define(stage, root + "/work_light")
    w.CreateRadiusAttr(0.15)
    w.CreateIntensityAttr(float(L.get("work_light_intensity", 0.0)))
    w.CreateColorAttr(Gf.Vec3f(0.85, 0.88, 0.95))
    xf = UsdGeom.Xformable(w); xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.2, -1.0, 2.6))
