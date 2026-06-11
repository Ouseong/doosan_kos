from console.base import *  # noqa: F401,F403
from console.base import _zyz_to_R, gripper_tip_world, camera_axis_world


class DistanceEstimatorScreen(ModeScreen):
    """Two-point monocular distance estimation.

    User jogs to two TCP positions and captures one image at each.
    Formula: h1 = |ΔZ| × w2 / (w2 − w1)
    No depth sensor, no focal length, no prior object size needed.
    """

    PRESETS = {
        "orange": [((5, 110, 110), (22, 255, 255))],
        "red":    [((0, 120, 100), (10, 255, 255)), ((170, 120, 100), (179, 255, 255))],
        "green":  [((40, 80, 60), (85, 255, 255))],
        "blue":   [((95, 120, 60), (130, 255, 255))],
        "yellow": [((22, 110, 110), (35, 255, 255))],
    }

    def __init__(self, parent, app):
        super().__init__(parent, app, "3. Distance Estimator (monocular)", T.CAM)

        self.add_intro(
            "Jog to Position 1 → Capture 1.  Jog to Position 2 → Capture 2.  "
            "Then click Estimate.    h = |ΔZ| × w2 / (w2 − w1)    "
            "(RGB only — no depth sensor, no focal length, no prior knowledge of object size)")

        # ── top controls ──────────────────────────────────────────────────────
        top = tk.Frame(self.body, bg=T.BG)
        top.pack(fill="x", pady=(0, 8))
        COLOR_HEX = {
            "orange": "#e8820c", "red": "#e84040",
            "green":  "#3ab06e", "blue": "#3a7ce8", "yellow": "#d4c800",
        }
        self._color_hex = COLOR_HEX
        self.color_var = tk.StringVar(value="orange")
        self._color_btn = tk.Button(
            top, text="orange", bg=COLOR_HEX["orange"], fg="white",
            activebackground=COLOR_HEX["orange"],
            font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold"),
            relief="flat", bd=0, padx=10, pady=4,
            command=self._show_color_menu)
        self._color_btn.pack(side="left", padx=6)
        self._select_color("orange")

        tk.Label(top, text="Min area (px):", bg=T.BG, fg=T.LABEL).pack(side="left", padx=(16, 0))
        self.min_area_var = tk.IntVar(value=600)
        tk.Entry(top, textvariable=self.min_area_var, width=6, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat").pack(side="left", padx=4)

        tk.Label(top, text="Size scale:", bg=T.BG, fg=T.LABEL).pack(side="left", padx=(16, 0))
        self.scale_var = tk.DoubleVar(value=1.0)
        tk.Entry(top, textvariable=self.scale_var, width=5, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat").pack(side="left", padx=4)
        tk.Label(top, text="(1.0 — tilt auto-handled)", bg=T.BG, fg=T.DIM,
                 font=tkfont.Font(family="DejaVu Sans", size=8)).pack(side="left")

        # ── two capture panels ────────────────────────────────────────────────
        panels_row = tk.Frame(self.body, bg=T.BG)
        panels_row.pack(fill="x", pady=8)
        panels_row.columnconfigure(0, weight=1)
        panels_row.columnconfigure(1, weight=1)

        self._cap = [None, None]
        self._panels = []

        for i in range(2):
            pf = tk.Frame(panels_row, bg=T.PANEL, highlightthickness=1,
                          highlightbackground=T.BORDER)
            pf.grid(row=0, column=i, padx=6, pady=4, sticky="nsew")

            tk.Label(pf, text=f"📍 Point {i+1}", bg=T.PANEL, fg=T.CAM,
                     font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold")
                     ).pack(anchor="w", padx=12, pady=(10, 4))

            z_lbl = tk.Label(pf, text="TCP Z:  ---  mm", bg=T.PANEL, fg=T.VAL,
                             font=tkfont.Font(family="DejaVu Sans Mono", size=10))
            z_lbl.pack(anchor="w", padx=12, pady=2)

            px_lbl = tk.Label(pf, text="Object: ---  px", bg=T.PANEL, fg=T.VAL,
                              font=tkfont.Font(family="DejaVu Sans Mono", size=10))
            px_lbl.pack(anchor="w", padx=12, pady=2)

            big_button(pf, f"Capture {i+1}", lambda idx=i: self._capture(idx),
                       bg=T.CAM, fg=T.BG, height=2
                       ).pack(padx=12, pady=(8, 12), fill="x")

            pf._z_lbl = z_lbl
            pf._px_lbl = px_lbl
            self._panels.append(pf)

        # ── Z jog / estimate / reset / camera ────────────────────────────────
        mid = tk.Frame(self.body, bg=T.BG)
        mid.pack(fill="x", pady=4)

        # Z ±10 cm jog buttons
        big_button(mid, "↑", lambda: self._z_jog(+100.0),
                   bg=T.PANEL_HI, fg=T.TITLE, height=2, width=3).pack(side="left", padx=2)
        big_button(mid, "↓", lambda: self._z_jog(-100.0),
                   bg=T.PANEL_HI, fg=T.TITLE, height=2, width=3).pack(side="left", padx=2)

        # View jog: move the TCP in the camera IMAGE PLANE so the object shifts
        # left/right (◀▶) / up/down (▲▼) in the view. Directions are the same
        # image-plane axes the Fetch alignment uses.
        vj = tk.Frame(mid, bg=T.BG)
        vj.pack(side="left", padx=(10, 4))
        tk.Label(vj, text="View:", bg=T.BG, fg=T.LABEL,
                 font=tkfont.Font(family="DejaVu Sans", size=8)).pack(side="left")
        for txt, ax, sgn in [("◀", "h", -1), ("▶", "h", +1),
                             ("▲", "v", -1), ("▼", "v", +1)]:
            big_button(vj, txt, lambda a=ax, s=sgn: self._img_jog(a, s),
                       bg=T.PANEL_HI, fg=T.TITLE, height=1, width=2
                       ).pack(side="left", padx=1)
        self._img_jog_step = tk.DoubleVar(value=10.0)
        tk.Entry(vj, textvariable=self._img_jog_step, width=4, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat").pack(side="left", padx=(3, 0))
        tk.Label(vj, text="mm", bg=T.BG, fg=T.DIM,
                 font=tkfont.Font(family="DejaVu Sans", size=8)).pack(side="left")

        big_button(mid, "Estimate Distance", self._estimate,
                   bg=T.CAM, fg=T.BG, height=2, width=22).pack(side="left", padx=4)

        # Fetch toggle button: click to start moving, click again to stop
        self._fetch_moving = False
        self._fetch_btn = tk.Button(
            mid, text="Fetch\n(start)", bg="#a6e3a1", fg=T.BG,
            activebackground="#74c47a", relief="flat", bd=0,
            width=8, height=2, cursor="hand2",
            font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold"),
            state="normal", command=self._fetch_toggle)
        self._fetch_btn.pack(side="left", padx=4)

        # Vel / Acc entries for fetch
        vel_acc_frame = tk.Frame(mid, bg=T.BG)
        vel_acc_frame.pack(side="left", padx=(6, 4))
        for label, var_name, default in [("vel", "_fetch_vel", 30.0),
                                          ("acc", "_fetch_acc", 30.0)]:
            row = tk.Frame(vel_acc_frame, bg=T.BG)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=f"{label}:", bg=T.BG, fg=T.LABEL,
                     font=tkfont.Font(family="DejaVu Sans", size=8), width=3
                     ).pack(side="left")
            dv = tk.DoubleVar(value=default)
            setattr(self, var_name, dv)
            tk.Entry(row, textvariable=dv, width=5, bg=T.PANEL, fg=T.VAL,
                     insertbackground=T.VAL, relief="flat"
                     ).pack(side="left", padx=2)

        # Second button row (keeps the first row from overflowing off-screen)
        mid2 = tk.Frame(self.body, bg=T.BG)
        mid2.pack(fill="x", pady=2)

        big_button(mid2, "Reset", self._reset,
                   bg=T.PANEL_HI, fg=T.TITLE, height=2).pack(side="left", padx=4)

        self._cam_open = False
        self._cam_btn = big_button(mid2, "Open Camera", self._toggle_camera,
                                   bg=T.PANEL_HI, fg=T.TITLE, height=2, width=14)
        self._cam_btn.pack(side="left", padx=4)

        self._obj_world = None   # estimated object world position [x,y,z,a,b,c]

        # ── result panel ──────────────────────────────────────────────────────
        res = tk.Frame(self.body, bg=T.PANEL)
        res.pack(fill="x", pady=(4, 0), ipady=10)
        self._result_lbls = {}
        for key, text in [("dz",   "Δ along optic axis (Δang):"),
                          ("h1",   "Distance at Pt.1:"),
                          ("h2",   "Distance at Pt.2:"),
                          ("xy",   "X/Y offset (cam):"),
                          ("size", "Object size (L):")]:
            row = tk.Frame(res, bg=T.PANEL)
            row.pack(fill="x", padx=16, pady=3)
            tk.Label(row, text=text, width=22, bg=T.PANEL, fg=T.LABEL, anchor="w",
                     font=tkfont.Font(family="DejaVu Sans", size=10)
                     ).pack(side="left")
            lbl = tk.Label(row, text="---", bg=T.PANEL, fg=T.VAL,
                           font=tkfont.Font(family="DejaVu Sans Mono", size=12, weight="bold"))
            lbl.pack(side="left")
            self._result_lbls[key] = lbl

    def _show_color_menu(self):
        menu = tk.Menu(self, tearoff=0, bg=T.PANEL, fg=T.VAL,
                       activebackground=T.PANEL_HI, activeforeground=T.TITLE,
                       font=tkfont.Font(family="DejaVu Sans", size=10))
        for name in self.PRESETS:
            menu.add_command(label=f"  {name}  ",
                             command=lambda n=name: self._select_color(n))
        btn = self._color_btn
        menu.tk_popup(btn.winfo_rootx(),
                      btn.winfo_rooty() + btn.winfo_height())

    def _select_color(self, name: str):
        self.color_var.set(name)
        hex_col = self._color_hex[name]
        self._color_btn.config(text=name, bg=hex_col, activebackground=hex_col)
        # Write selected color to shared file so live viewer shows bbox
        try:
            with open("/kos_workspace/.cam_color", "w") as f:
                f.write(name)
        except Exception:
            pass

    def _img_jog(self, axis, sign):
        """Jog the TCP along the camera IMAGE-PLANE axes (perpendicular to the
        optical axis): axis 'h' = horizontal (◀▶), 'v' = vertical (▲▼). Moves
        the object left/right/up/down in the view — same axes Fetch centers on.
        If an arrow goes the opposite way on screen, the sign just needs a flip."""
        def worker():
            cur = self.robot.get_current_posx(0)
            if not cur:
                self.after(0, lambda: self.set_status("could not read TCP pose", T.BAD))
                return
            n = np.array(camera_axis_world(cur))
            u = np.cross(n, np.array([0.0, 0.0, 1.0]))
            if np.linalg.norm(u) < 1e-6:
                u = np.cross(n, np.array([1.0, 0.0, 0.0]))
            u = u / (np.linalg.norm(u) or 1.0)
            v = np.cross(n, u)
            v = v / (np.linalg.norm(v) or 1.0)
            ax = u if axis == "h" else v
            try:
                step = float(self._img_jog_step.get())
            except Exception:
                step = 10.0
            delta = (ax * sign * step).tolist()
            target = [cur[0] + delta[0], cur[1] + delta[1], cur[2] + delta[2],
                      cur[3], cur[4], cur[5]]
            ok_ws, why = RobotInterface._check_workspace(target, ref=0)
            if not ok_ws:
                self.after(0, lambda: self.set_status(f"View jog blocked — {why}", T.BAD))
                return
            self.after(0, lambda: self.set_status(
                f"View jog {'◀▶'[sign > 0] if axis == 'h' else '▲▼'[sign > 0]} {step:.0f}mm", T.LABEL))
            self.robot.movel(target, vel_lin=40.0, vel_ang=40.0,
                             acc_lin=80.0, acc_ang=80.0, sync=0,
                             confirm_real_callback=self.app.confirm_real_modal)
        threading.Thread(target=worker, daemon=True).start()

    def _z_jog(self, delta_mm: float):
        """Move TCP by delta_mm along Z axis (world frame)."""
        def worker():
            cur = self.robot.get_current_posx(0)
            if not cur:
                self.after(0, lambda: self.set_status("could not read TCP pose", T.BAD))
                return
            target = list(cur)
            target[2] += delta_mm
            direction = "Up" if delta_mm > 0 else "Down"
            self.after(0, lambda: self.set_status(
                f"Z {direction} {abs(delta_mm):.0f}mm → movel", T.LABEL))
            ok = self.robot.movel(target, vel_lin=50.0, acc_lin=100.0,
                                  confirm_real_callback=self.app.confirm_real_modal)
            self.after(0, lambda: self.set_status(
                "OK" if ok else "FAILED", T.OK if ok else T.BAD))
        threading.Thread(target=worker, daemon=True).start()

    def _get_host_ip(self):
        import subprocess
        try:
            out = subprocess.check_output(["ip", "route"], text=True)
            return next(l.split()[2] for l in out.splitlines() if l.startswith("default"))
        except Exception:
            return "172.17.0.1"

    def _capture(self, idx):
        self.set_status(f"capturing point {idx + 1} ...", T.LABEL)

        def worker():
            # 1. TCP 위치 읽기 (ROS — docker 안에서 동작)
            posx = self.robot.get_current_posx(0)
            if not posx:
                self.after(0, lambda: self.set_status("get_current_posx failed", T.BAD))
                return
            tcp_z = posx[2]
            tcp_posx = list(posx)

            # 2. 호스트 cam_daemon 에 캡처 요청 (pyrealsense2 는 호스트에만 있음)
            color = self.color_var.get()
            min_area = self.min_area_var.get()
            try:
                import socket as _sock
                host_ip = self._get_host_ip()
                with _sock.create_connection((host_ip, 19876), timeout=15) as s:
                    s.sendall(f"capture:{idx}:{color}:{min_area}".encode())
                    response = s.makefile().readline().strip()
            except Exception as exc:
                err = str(exc)
                self.after(0, lambda: self.set_status(f"cam daemon error: {err}", T.BAD))
                return

            # response: ok:<px_w>:<fx>:<fy>:<ppx>:<ppy>:<bx>:<by>
            #        or none:<fx>:<fy>:<ppx>:<ppy>:0:0
            px_w = None
            fx = fy = ppx = ppy = bx = by = None
            parts = response.split(":")
            try:
                if parts[0] == "ok":
                    px_w = float(parts[1])
                    fx, fy = float(parts[2]), float(parts[3])
                    ppx, ppy = float(parts[4]), float(parts[5])
                    bx, by = float(parts[6]), float(parts[7])
                elif parts[0] == "none":
                    fx, fy = float(parts[1]), float(parts[2])
                    ppx, ppy = float(parts[3]), float(parts[4])
            except (IndexError, ValueError):
                pass

            def update():
                self._cap[idx] = {
                    "tcp_z": tcp_z, "tcp_posx": tcp_posx,
                    "px_w": px_w, "fx": fx, "fy": fy,
                    "ppx": ppx, "ppy": ppy, "bx": bx, "by": by,
                }
                pf = self._panels[idx]
                pf._z_lbl.config(text=f"TCP Z:  {tcp_z:+.1f}  mm")
                if px_w is not None:
                    pf._px_lbl.config(text=f"Object: {px_w:.0f}  px", fg=T.OK)
                else:
                    pf._px_lbl.config(text="Object: not detected", fg=T.BAD)
                self.set_status(
                    f"point {idx+1}: TCP Z={tcp_z:.1f}mm, "
                    f"obj={'not found' if px_w is None else f'{px_w:.0f}px'}",
                    T.OK if px_w else T.WARN)
            self.after(0, update)

        threading.Thread(target=worker, daemon=True).start()

    def _estimate(self):
        c0, c1 = self._cap
        if not c0 or not c1:
            self.set_status("capture both points first", T.WARN)
            return
        if c0["px_w"] is None or c1["px_w"] is None:
            self.set_status("object not detected in one or both captures", T.BAD)
            return

        w1, w2 = c0["px_w"], c1["px_w"]

        # ── Baseline projected onto the camera optical axis ──────────────────
        # The camera is tilted ~45° from the TCP Z axis, so only the COMPONENT
        # of the robot's motion ALONG the optical axis changes the apparent
        # pixel size — a sideways move barely changes it. So instead of the raw
        # |ΔZ| we project the TCP world displacement between the two captures
        # onto the (world) optical axis. This folds cos(tilt) in automatically,
        # which is exactly the old hand-tuned 0.707 factor — now derived, not
        # hard-coded.  Optical axis = CAMERA_AXIS_TOOL0 rotated by each capture's
        # TCP orientation.
        p0  = c0.get("tcp_posx")
        pc1 = c1.get("tcp_posx")
        if not p0 or not pc1:
            self.set_status("a capture is missing its TCP pose — re-capture", T.BAD)
            return
        ax0 = np.array(camera_axis_world(p0))     # world optical axis @ capture 0
        ax1 = np.array(camera_axis_world(pc1))    # world optical axis @ capture 1
        ax_ang = math.degrees(math.acos(
            float(np.clip(np.dot(ax0, ax1), -1.0, 1.0))))
        axis = ax0 + ax1
        axis = axis / (float(np.linalg.norm(axis)) or 1.0)   # mean view direction
        dvec = np.array(pc1[:3]) - np.array(p0[:3])          # TCP world travel
        d_optical = abs(float(np.dot(dvec, axis)))           # baseline along optic axis

        if d_optical < 1.0:
            self.set_status(
                f"optical-axis baseline < 1mm (got {d_optical:.2f}) — "
                "move farther ALONG the camera view", T.WARN)
            return
        if abs(w2 - w1) < 0.5:
            self.set_status("pixel sizes too similar — move robot closer/farther", T.WARN)
            return

        # closer capture has the larger pixel size; keep w2 the larger one
        if w1 > w2:
            w1, w2 = w2, w1

        # Distance from camera to object (parallax along the optical axis)
        h2 = d_optical * w1 / (w2 - w1)
        h1 = h2 + d_optical

        # Warn if the viewing direction swung between captures — the single-axis
        # projection assumes a roughly constant optical axis.
        axis_warn = (f"⚠ optic axis moved {ax_ang:.1f}° between captures "
                     "(>5°): keep the same orientation. "
                     if ax_ang > 5.0 else "")

        # Use closer capture (c_near) for size and world position
        c_near = c1 if c1["px_w"] == w2 else c0
        fx   = c_near.get("fx") or 608.67
        fy   = c_near.get("fy") or 608.75
        ppx  = c_near.get("ppx") or 319.33
        ppy  = c_near.get("ppy") or 239.98
        bx   = c_near.get("bx")
        by   = c_near.get("by")
        posx_near = c_near.get("tcp_posx")

        try:
            scale = float(self.scale_var.get())
        except Exception:
            scale = 1.0

        # Optional residual fudge factor (default 1.0). The 45° camera tilt is
        # now handled by the optical-axis projection above, so this no longer
        # needs the old ~0.7 — leave it at 1.0 unless a separate bias remains.
        h1 = h1 * scale
        h2 = h2 * scale

        # Object size
        obj_size = (w2 * h2 / fx)  # h2 already scaled

        # X/Y offset in camera frame (mm) — use scaled h2
        x_cam = ((bx - ppx) * h2 / fx) if bx is not None else None
        y_cam = ((by - ppy) * h2 / fy) if by is not None else None

        # Transform camera offset → world frame using TCP rotation
        self._obj_world = None
        if posx_near and x_cam is not None:
            px, py, pz, A, B, C = posx_near[:6]
            R = _zyz_to_R(A, B, C)
            dx = R[0][0]*x_cam + R[0][1]*y_cam + R[0][2]*h2
            dy = R[1][0]*x_cam + R[1][1]*y_cam + R[1][2]*h2
            dz_w = R[2][0]*x_cam + R[2][1]*y_cam + R[2][2]*h2
            self._obj_world = [px+dx, py+dy, pz+dz_w, A, B, C]
            self._fetch_btn.config(state="normal")

        # Update result labels (dz row now shows the optical-axis baseline)
        self._result_lbls["dz"].config(
            text=f"{d_optical:.1f} mm  ({ax_ang:.1f}°)", fg=T.VAL)
        self._result_lbls["h1"].config(text=f"{h1:.1f} mm", fg=T.VAL)
        self._result_lbls["h2"].config(text=f"{h2:.1f} mm", fg=T.OK)
        if x_cam is not None:
            self._result_lbls["xy"].config(
                text=f"X={x_cam:+.1f}  Y={y_cam:+.1f} mm", fg=T.OK)
        else:
            self._result_lbls["xy"].config(text="---", fg=T.VAL)
        self._result_lbls["size"].config(text=f"{obj_size:.1f} mm", fg=T.OK)

        base = (f"h2={h2:.1f}mm  X={x_cam:+.1f}mm  Y={y_cam:+.1f}mm  "
                f"size={obj_size:.1f}mm  → Fetch ready" if x_cam is not None else
                f"h2={h2:.1f}mm  size={obj_size:.1f}mm")
        self.set_status(axis_warn + base, T.WARN if axis_warn else T.OK)

    def _fetch_toggle(self):
        """Click once to start moving along the camera direction, click again
        to stop (replaces the old hold-to-move behaviour)."""
        if self._fetch_moving:
            self._fetch_stop()
        else:
            self._fetch_start()

    # ── object-centering (proportional image-plane servo) ───────────────────
    CENTER_TOL_PX  = 3.0     # consider centered when pixel offset < this (tight)
    ALIGN_ITERS    = 30      # max centering steps (far objects need many)
    MAX_CENTER_MM  = 250.0   # safety cap on a single centering move
    ALIGN_VEL      = 30.0    # slow, precise moves while aligning

    def _grab_px(self):
        """Blocking single capture via cam_daemon. Returns (bx, by, ppx, ppy)
        pixel center of the object, or None if not detected / no daemon."""
        import socket as _sock
        try:
            host_ip = self._get_host_ip()
            color = self.color_var.get()
            min_area = self.min_area_var.get()
            with _sock.create_connection((host_ip, 19876), timeout=15) as s:
                s.sendall(f"capture:9:{color}:{min_area}".encode())
                resp = s.makefile().readline().strip()
        except Exception:
            return None
        parts = resp.split(":")
        if len(parts) >= 8 and parts[0] == "ok":
            try:
                return (float(parts[6]), float(parts[7]),   # bx, by
                        float(parts[4]), float(parts[5]))    # ppx, ppy
            except ValueError:
                return None
        return None

    def _grab_px_avg(self, n=3):
        """Average n captures spaced ~40ms apart (distinct 30fps frames) to beat
        per-frame centroid noise — used for the precise final approach so a tight
        tolerance is actually reachable. Returns (bx,by,ppx,ppy) or None."""
        import time as _t
        xs, ys, ppx, ppy = [], [], None, None
        for i in range(n):
            m = self._grab_px()
            if m:
                xs.append(m[0]); ys.append(m[1]); ppx, ppy = m[2], m[3]
            if i < n - 1:
                _t.sleep(0.04)
        if not xs:
            return None
        return (sum(xs) / len(xs), sum(ys) / len(ys), ppx, ppy)

    def _move_world(self, dx, dy, dz):
        """Move the TCP by a world-frame delta (mm), blocking until done.
        Returns True on success, False if blocked/failed."""
        cur = self.robot.get_current_posx(0)
        if not cur:
            return False
        target = [cur[0] + dx, cur[1] + dy, cur[2] + dz, cur[3], cur[4], cur[5]]
        ok_ws, why = RobotInterface._check_workspace(target, ref=0)
        if not ok_ws:
            self.after(0, lambda: self.set_status(f"move blocked — {why}", T.BAD))
            return False
        return bool(self.robot.movel(
            target, vel_lin=self.ALIGN_VEL, vel_ang=self.ALIGN_VEL,
            acc_lin=self.ALIGN_VEL * 2, acc_ang=self.ALIGN_VEL * 2,
            sync=1, confirm_real_callback=self.app.confirm_real_modal))

    def _align_to_object(self, alive):
        """Center the object by repeatedly jogging the TCP TOWARD center along
        the camera image plane — the very same ◀▶▲▼ directions as the View jog
        (confirmed correct), proportional to the pixel offset, re-measuring each
        step. No probe, no Jacobian: object right → jog ◀, object down → jog ▲,
        etc., shrinking the step (gain) if it ever overshoots. Returns (ok,msg)."""
        cur = self.robot.get_current_posx(0)
        if not cur:
            return False, "could not read TCP pose"
        m0 = self._grab_px()
        if m0 is None:
            return False, "object not detected (open camera / check color)"
        ppx, ppy = m0[2], m0[3]

        # image-plane basis (same as View jog): ◀▶ = ∓u, ▲▼ = ∓v
        n = np.array(camera_axis_world(cur))
        u = np.cross(n, np.array([0.0, 0.0, 1.0]))
        if np.linalg.norm(u) < 1e-6:
            u = np.cross(n, np.array([1.0, 0.0, 0.0]))
        u = u / (np.linalg.norm(u) or 1.0)
        v = np.cross(n, u)
        v = v / (np.linalg.norm(v) or 1.0)

        MAXSTEP    = 25.0    # mm per coarse step
        ACCEPT     = 8.0     # close enough to proceed if fine can't do better
        FINE_ENTER = 12.0    # hand off to the 1mm fine approach below this offset
        TOL        = self.CENTER_TOL_PX   # concentric when both axes < TOL (3px)

        # ── Phase 1: coarse proportional servo → get within ~12px fast ──────
        gain_scale = 1.0
        prev = None
        for it in range(self.ALIGN_ITERS):
            if not alive():
                return False, "stopped"
            m = self._grab_px()
            if not m:
                return False, "lost object"
            ox, oy = m[0] - ppx, m[1] - ppy
            mag = (ox * ox + oy * oy) ** 0.5
            if mag < FINE_ENTER:
                break                       # close → switch to fine 1mm steps
            base = 0.5 if mag > 25 else 0.3
            if prev is not None and mag > prev * 1.3 and mag > prev + 4.0:
                gain_scale = max(0.3, gain_scale * 0.6)
            Kp = base * gain_scale
            self.robot.get_logger().info(
                f"[align] coarse it{it}: ({ox:+.0f},{oy:+.0f}) {mag:.0f}px Kp={Kp:.2f}")
            prev = mag
            du = max(-MAXSTEP, min(MAXSTEP, ox * Kp))
            dv = max(-MAXSTEP, min(MAXSTEP, oy * Kp))
            moved = False
            for shrink in (1.0, 0.5, 0.25):
                if self._move_world(*(u * du * shrink + v * dv * shrink).tolist()):
                    moved = True
                    break
            if not moved:
                break                       # blocked → finish with fine steps

        # ── Phase 2: fine 1mm jogs (like tapping ◀▶▲▼) until concentric ─────
        # On each axis still off-center, nudge a fixed 1mm toward the crosshair,
        # re-measuring with a 3-frame average, until the yellow centroid sits
        # inside the centre (both axes < TOL). Stops if 1mm granularity stalls.
        STEP  = 1.0
        best  = 1e9
        stall = 0
        for it in range(40):
            if not alive():
                return False, "stopped"
            m = self._grab_px_avg(3)
            if not m:
                return False, "lost object"
            ox, oy = m[0] - ppx, m[1] - ppy
            mag = (ox * ox + oy * oy) ** 0.5
            self.robot.get_logger().info(
                f"[align] fine it{it}: ({ox:+.1f},{oy:+.1f}) {mag:.1f}px")
            if abs(ox) < TOL and abs(oy) < TOL:
                return True, f"concentric ({mag:.1f}px, fine {it})"
            if mag < best - 0.4:
                best, stall = mag, 0
            else:
                stall += 1
            if stall >= 3:                  # 1mm granularity can't improve further
                return (mag < 2 * TOL), f"fine limit {mag:.1f}px (1mm step)"
            # 1mm toward center on each axis that is still outside TOL
            du = STEP if ox > TOL else (-STEP if ox < -TOL else 0.0)
            dv = STEP if oy > TOL else (-STEP if oy < -TOL else 0.0)
            moved = False
            for shrink in (1.0, 0.5):
                if self._move_world(*(u * du * shrink + v * dv * shrink).tolist()):
                    moved = True
                    break
            if not moved:
                return (mag < 2 * TOL), f"blocked {mag:.1f}px"

        m = self._grab_px_avg(3)
        mag = ((m[0] - ppx) ** 2 + (m[1] - ppy) ** 2) ** 0.5 if m else 999
        return (mag < 2 * TOL), f"closest {mag:.1f}px"

    def _fetch_start(self):
        """Fetch = (1) center the object on the optical axis with a quick image
        Jacobian, then (2) drive straight along the camera viewing axis toward
        it. Click again to stop at any phase."""
        self._fetch_moving = True

        def fail(msg):
            self._fetch_moving = False
            self.after(0, lambda: self.set_status(msg, T.BAD))
            self._fetch_btn.after(0, lambda: self._fetch_btn.config(
                text="Fetch\n(start)", bg="#a6e3a1"))

        def worker():
            self._fetch_btn.after(0, lambda: self._fetch_btn.config(
                text="Fetch\n(stop)", bg="#f38ba8"))

            # ── Phase 1: center the object (zero X/Y pixel offset) ──────────
            self.after(0, lambda: self.set_status(
                "Fetch ①: centering object (image Jacobian)…", T.LABEL))
            try:
                ok, msg = self._align_to_object(lambda: self._fetch_moving)
            except Exception as e:
                fail(f"align error: {e}")
                return
            if not self._fetch_moving:        # user pressed stop mid-align
                return
            if not ok:
                fail(f"Fetch align failed — {msg}")
                return
            self.after(0, lambda: self.set_status(
                f"Fetch ②: {msg} → approaching along optic axis…", T.LABEL))

            # ── Phase 2: approach along the camera optical axis ─────────────
            cur = self.robot.get_current_posx(0)
            if not cur:
                fail("could not read TCP pose")
                return
            ux, uy, uz = camera_axis_world(cur)

            # CLAMP reach so the finger-tip stops at the floor (tip moves
            # parallel to the TCP: tip_z(d) = tip_z0 + uz*d).
            REACH = 800.0
            tip_z0 = gripper_tip_world(cur)[2]
            FLOOR = TCP_Z_MIN_MM + 1.0
            reach = REACH
            if uz < -1e-6:
                reach = max(0.0, min(REACH, (FLOOR - tip_z0) / uz))
            if reach < 1.0:
                fail(f"Fetch blocked — tip already at floor (tip Z={tip_z0:.0f}mm)")
                return

            target = [cur[0] + ux * reach, cur[1] + uy * reach,
                      cur[2] + uz * reach, cur[3], cur[4], cur[5]]
            ok_ws, why = RobotInterface._check_workspace(target, ref=0)
            if not ok_ws:
                fail(f"Fetch blocked — {why}")
                return
            if not self._fetch_moving:
                return

            vel = self._fetch_vel.get()
            acc = self._fetch_acc.get()
            self.after(0, lambda: self.set_status(
                f"Fetch → cam dir [{ux:+.2f},{uy:+.2f},{uz:+.2f}]  reach={reach:.0f}mm", T.LABEL))
            self.robot.movel(target, vel_lin=vel, vel_ang=vel,
                             acc_lin=acc, acc_ang=acc, sync=0,
                             confirm_real_callback=self.app.confirm_real_modal)

        threading.Thread(target=worker, daemon=True).start()

    def _fetch_stop(self):
        self._fetch_moving = False
        self.robot.cancel_motion(stop_mode=3)
        self._fetch_btn.config(text="Fetch\n(start)", bg="#a6e3a1")   # back to green
        self.set_status("Fetch stopped", T.LABEL)

    def _reset(self):
        self._cap = [None, None]
        self._obj_world = None
        self._fetch_btn.config(state="normal")  # camera-direction fetch needs no estimate
        for pf in self._panels:
            pf._z_lbl.config(text="TCP Z:  ---  mm")
            pf._px_lbl.config(text="Object: ---  px", fg=T.VAL)
        for lbl in self._result_lbls.values():
            lbl.config(text="---", fg=T.VAL)
        self.set_status("reset — jog to point 1, capture, jog to point 2, capture, estimate", T.LABEL)

    def _cam_send(self, cmd: str) -> str:
        """Send a command to host cam_daemon (port 19876); return its reply ('' if none)."""
        import socket, subprocess
        # Host IP = default gateway inside docker
        try:
            out = subprocess.check_output(["ip", "route"], text=True)
            host_ip = next(
                line.split()[2] for line in out.splitlines()
                if line.startswith("default"))
        except Exception:
            host_ip = "172.17.0.1"
        with socket.create_connection((host_ip, 19876), timeout=3) as s:
            s.sendall(cmd.encode())
            try:
                s.settimeout(5)
                return s.recv(64).decode().strip()
            except Exception:
                return ""

    def _toggle_camera(self):
        if self._cam_open:
            # Closing — fire-and-forget
            try:
                self._cam_send("close")
            except Exception as e:
                self.set_status(f"cam daemon unreachable: {e}", T.BAD)
                return
            self._cam_open = False
            self._cam_btn.config(text="Open Camera", bg=T.PANEL_HI, fg=T.TITLE)
            self.set_status("Camera closed", T.LABEL)
            return

        # Opening — the daemon replies whether the camera is connected
        try:
            reply = self._cam_send("open")
        except Exception as e:
            self.set_status(f"cam daemon unreachable: {e}", T.BAD)
            return
        if reply == "no_camera":
            self.set_status("Camera not connected", T.BAD)
            return
        self._cam_open = True
        self._cam_btn.config(text="Close Camera", bg=T.CAM, fg=T.BG)
        self.set_status("Camera opened", T.OK)


# ──────── Home (dashboard) ─────────────────────────────────
