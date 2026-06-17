from console.base import *  # noqa: F401,F403
from console.base import _zyz_to_R, gripper_tip_world, camera_axis_world
from console.joystick import Joystick, deadzone


class TaskSpaceScreen(ModeScreen):
    # Joystick teleop: throttle lever scales the linear speed between these.
    JOY_SPEED_MIN = 15.0    # mm/s  (throttle pulled back)
    JOY_SPEED_MAX = 200.0   # mm/s  (throttle pushed forward)
    JOY_TICK_MS = 50        # 20 Hz, matches the speedl publish rate
    JOY_GRIPPER_BTN = 1     # side thumb (gray) button → toggle gripper open/close

    def __init__(self, parent, app):
        super().__init__(parent, app, "2. Task Space Move (TCP)", T.TASK)

        # joystick teleop state (built lazily on arm)
        self._joy = None
        self._joy_armed = False
        self._joy_grip_open = True       # gripper homes to open
        self._joy_prev_grip_btn = False  # for rising-edge detection

        self.add_intro(
            "Specify the end-effector pose: position (X,Y,Z) in mm and "
            "orientation (Rx,Ry,Rz) in degrees. The controller computes IK "
            "internally. MoveL keeps the TCP on a straight line in the chosen "
            "frame (BASE / TOOL / WORLD). MoveJX targets the same pose but "
            "lets joints take the shortest path (faster, but the TCP path is "
            "curved). If the target is unreachable or hits a joint limit, the "
            "driver returns FAILED — pick a closer pose.")

        # speed
        top = tk.Frame(self.body, bg=T.BG)
        top.pack(fill="x", pady=(0, 8))
        tk.Label(top, text="vel (mm/s, °/s):", bg=T.BG, fg=T.LABEL).pack(side="left")
        self.vlin = tk.DoubleVar(value=100.0); self.vang = tk.DoubleVar(value=30.0)
        tk.Entry(top, textvariable=self.vlin, width=6, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat").pack(side="left", padx=4)
        tk.Entry(top, textvariable=self.vang, width=6, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat").pack(side="left", padx=4)
        tk.Label(top, text="ref:", bg=T.BG, fg=T.LABEL).pack(side="left", padx=(12, 0))
        self.ref_var = tk.StringVar(value="BASE")
        ref_menu = tk.OptionMenu(top, self.ref_var, "BASE", "TOOL", "WORLD")
        ref_menu.config(bg=T.PANEL, fg=T.VAL, activebackground=T.PANEL_HI,
                        relief="flat", highlightthickness=0)
        ref_menu["menu"].config(bg=T.PANEL, fg=T.VAL)
        ref_menu.pack(side="left", padx=4)

        # target rows
        rows = tk.Frame(self.body, bg=T.PANEL)
        rows.pack(fill="x", pady=4, ipady=8)
        self.entries = []
        self.cur_lbls = []
        units = ["mm", "mm", "mm", "°", "°", "°"]
        for i, ax in enumerate(TCP_AXES):
            row = tk.Frame(rows, bg=T.PANEL)
            row.pack(fill="x", padx=12, pady=3)
            tk.Label(row, text=ax, width=3, bg=T.PANEL, fg=T.TASK,
                     font=tkfont.Font(family="DejaVu Sans Mono", size=11, weight="bold")
                     ).pack(side="left")
            cur = tk.Label(row, text="   ---", width=10, bg=T.PANEL, fg=T.VAL,
                           font=tkfont.Font(family="DejaVu Sans Mono", size=10))
            cur.pack(side="left", padx=6)
            self.cur_lbls.append(cur)
            tk.Label(row, text="→", bg=T.PANEL, fg=T.DIM).pack(side="left", padx=4)
            v = tk.DoubleVar(value=0.0)
            e = tk.Entry(row, textvariable=v, width=12, bg=T.PANEL_HI, fg=T.VAL,
                         insertbackground=T.VAL, relief="flat",
                         font=tkfont.Font(family="DejaVu Sans Mono", size=11))
            e.pack(side="left", padx=4)
            tk.Label(row, text=units[i], width=4, bg=T.PANEL, fg=T.DIM,
                     anchor="w").pack(side="left")
            self.entries.append(v)

        btns = tk.Frame(self.body, bg=T.BG)
        btns.pack(fill="x", pady=10)
        big_button(btns, "Sync to current", self._sync, bg=T.PANEL_HI, fg=T.TITLE).pack(side="left", padx=4)
        big_button(btns, "MoveL (linear)", self._movel, bg=T.TASK).pack(side="left", padx=4)
        big_button(btns, "MoveJX (joint)", self._movejx, bg=T.PANEL_HI, fg=T.TITLE).pack(side="left", padx=4)

        self._build_joystick_panel()

        info = tk.Label(self.body, bg=T.BG, fg=T.DIM, justify="left", anchor="w",
                        font=tkfont.Font(family="DejaVu Sans", size=9),
                        text=("MoveL: TCP travels in a straight line in the chosen frame.\n"
                              "MoveJX: targets the same TCP pose but lets joints take the\n"
                              "           shortest joint-space path (faster, curved TCP path)."))
        info.pack(fill="x", pady=(8, 0))

        self.after(500, self._refresh)

    def _ref_int(self):
        return {"BASE": 0, "TOOL": 1, "WORLD": 2}[self.ref_var.get()]

    def _refresh(self):
        # Poll posx every 500ms
        ref = self._ref_int()
        def fetch():
            return self.robot.get_current_posx(ref=ref)
        def update(p):
            if p:
                for i, lbl in enumerate(self.cur_lbls):
                    lbl.config(text=f"{p[i]:+9.2f}")
        self.run_async(fetch, on_done=update)
        self.after(500, self._refresh)

    def on_show(self):
        self._sync()

    def _sync(self):
        def fetch():
            return self.robot.get_current_posx(ref=self._ref_int())
        def update(p):
            if p:
                for i, e in enumerate(self.entries):
                    e.set(round(p[i], 2))
                self.set_status("synced to current TCP pose", T.LABEL)
            else:
                self.set_status("get_current_posx failed", T.BAD)
        self.run_async(fetch, on_done=update)

    def _movel(self):
        target = [e.get() for e in self.entries]
        ref = self._ref_int()
        self.set_status(f"movel → {target}", T.LABEL)
        self.run_async(
            lambda: self.robot.movel(target, vel_lin=self.vlin.get(),
                                     vel_ang=self.vang.get(), ref=ref,
                                     confirm_real_callback=self.app.confirm_real_modal),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))

    def _movejx(self):
        target = [e.get() for e in self.entries]
        ref = self._ref_int()
        self.set_status(f"movejx → {target}", T.LABEL)
        self.run_async(
            lambda: self.robot.movejx(target, vel=self.vang.get(),
                                      acc=60.0, ref=ref,
                                      confirm_real_callback=self.app.confirm_real_modal),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))

    # ──────── Joystick teleop (Logitech Extreme 3D Pro) ────────
    # Toggle to arm. While armed, motion only streams when the trigger
    # (button 0) is HELD (deadman). speedl velocity (BASE frame):
    #   stick X (axis0) → X,  stick Y (axis1) → Y,  twist (axis2) → Z.
    #   throttle (axis3) → speed scale,  trigger (button0) → deadman.
    # Streams to the SIM controller only, like the Speed Control screen.
    def _build_joystick_panel(self):
        panel = tk.Frame(self.body, bg=T.PANEL)
        panel.pack(fill="x", pady=(4, 0), ipady=6)

        head = tk.Frame(panel, bg=T.PANEL)
        head.pack(fill="x", padx=12, pady=(4, 2))
        tk.Label(head, text="🕹 Joystick teleop  (Extreme 3D Pro)",
                 bg=T.PANEL, fg=T.TASK,
                 font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold")
                 ).pack(side="left")
        self.joy_conn_lbl = tk.Label(head, text="disarmed", bg=T.PANEL, fg=T.DIM,
                                     font=tkfont.Font(family="DejaVu Sans", size=9))
        self.joy_conn_lbl.pack(side="right")

        row = tk.Frame(panel, bg=T.PANEL)
        row.pack(fill="x", padx=12, pady=2)
        self.joy_btn = big_button(row, "🕹 ARM JOYSTICK", self._joy_toggle,
                                  bg=T.TASK, height=1, width=18)
        self.joy_btn.pack(side="left")
        self.joy_dead_lbl = tk.Label(row, text="○ disarmed", bg=T.PANEL, fg=T.DIM,
                                     width=22, anchor="w",
                                     font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold"))
        self.joy_dead_lbl.pack(side="left", padx=10)
        self.joy_gain_lbl = tk.Label(row, text="speed   -- mm/s", bg=T.PANEL, fg=T.VAL,
                                     font=tkfont.Font(family="DejaVu Sans Mono", size=10))
        self.joy_gain_lbl.pack(side="right")

        row2 = tk.Frame(panel, bg=T.PANEL)
        row2.pack(fill="x", padx=12, pady=2)
        self.joy_vel_lbl = tk.Label(row2, text="vx     +0  vy     +0  vz     +0 mm/s",
                                    bg=T.PANEL, fg=T.VAL,
                                    font=tkfont.Font(family="DejaVu Sans Mono", size=10))
        self.joy_vel_lbl.pack(side="left")

        # invert checkboxes — calibrate sign against the robot BASE frame live
        self.inv_x = tk.BooleanVar(value=False)
        self.inv_y = tk.BooleanVar(value=True)   # forward push → +Y by default
        self.inv_z = tk.BooleanVar(value=False)
        inv = tk.Frame(row2, bg=T.PANEL)
        inv.pack(side="right")
        tk.Label(inv, text="invert:", bg=T.PANEL, fg=T.DIM,
                 font=tkfont.Font(family="DejaVu Sans", size=9)).pack(side="left")
        for txt, var in (("X", self.inv_x), ("Y", self.inv_y), ("Z", self.inv_z)):
            tk.Checkbutton(inv, text=txt, variable=var, bg=T.PANEL, fg=T.VAL,
                           selectcolor=T.PANEL_HI, activebackground=T.PANEL,
                           font=tkfont.Font(family="DejaVu Sans", size=9)
                           ).pack(side="left")

        tk.Label(panel,
                 text="HOLD the trigger (button 0) to move — release = STOP.  "
                      "Twist = Z up/down.  Throttle lever = speed.  "
                      "Side gray button (1) = gripper open/close toggle.",
                 bg=T.PANEL, fg=T.DIM, anchor="w",
                 font=tkfont.Font(family="DejaVu Sans", size=8)
                 ).pack(fill="x", padx=12, pady=(2, 0))

    def _joy_set_conn(self, text, color=T.DIM):
        self.joy_conn_lbl.config(text=text, fg=color)

    def _joy_toggle(self):
        if self._joy_armed:
            self._joy_disarm()
            self._joy_set_conn("disarmed", T.LABEL)
        else:
            self._joy_arm()

    def _joy_arm(self):
        if self._joy_armed:
            return
        import os as _os
        # On ARM: (re)scan for the joystick, confirm it is connected and
        # actually sending data, THEN connect. If not connected, report clearly
        # and do NOT start the velocity stream.
        self._joy_set_conn("checking joystick…", T.WARN)
        self.update_idletasks()        # paint the label before the blocking probe

        js, last_err = None, "no joystick device"
        for _attempt in range(2):      # one retry: some devices stay silent on first probe
            cand = Joystick()
            if not cand.start():       # opens device + waits for joydev init events
                last_err = cand.error or "joystick not found"
                continue
            if cand.ready:
                js = cand
                break
            last_err = cand.error or "detected but no data"
            cand.stop()                # opened but silent → treat as not connected

        if js is None:
            self._joy_set_conn(f"not connected — {last_err}", T.BAD)
            return

        self._joy = js
        self._joy_armed = True
        self.robot.speedl_start()
        self.joy_btn.config(text="■ DISARM JOYSTICK", bg=T.BAD)
        self._joy_set_conn(f"connected — {_os.path.basename(js.path)}", T.OK)
        self.after(self.JOY_TICK_MS, self._joy_tick)

    def _joy_disarm(self):
        self._joy_armed = False
        self._joy_prev_grip_btn = False   # avoid a stale edge on next arm
        try:
            self.robot.speedl_stop()
        except Exception:
            pass
        if self._joy:
            self._joy.stop()
            self._joy = None
        self.joy_btn.config(text="🕹 ARM JOYSTICK", bg=T.TASK)
        self.joy_dead_lbl.config(text="○ disarmed", fg=T.DIM)
        self.joy_vel_lbl.config(text="vx     +0  vy     +0  vz     +0 mm/s")
        self.joy_gain_lbl.config(text="speed   -- mm/s")

    def _joy_tick(self):
        if not self._joy_armed:
            return
        js = self._joy
        if js is None or not js.connected:
            self._joy_set_conn("joystick disconnected", T.BAD)
            self._joy_disarm()
            return

        # Side (gray) button → toggle gripper open/close on each press.
        # Independent of the deadman, so the gripper works without moving.
        grip_btn = js.button(self.JOY_GRIPPER_BTN)
        if grip_btn and not self._joy_prev_grip_btn:     # rising edge
            self._joy_grip_open = not self._joy_grip_open
            if self._joy_grip_open:
                self.gripper_panel._open()
            else:
                self.gripper_panel._close()
        self._joy_prev_grip_btn = grip_btn

        deadman = js.button(0)               # trigger
        ax = deadzone(js.axis(0))            # stick L/R  → X
        ay = deadzone(js.axis(1))            # stick F/B  → Y
        az = deadzone(js.axis(2))            # twist (Rz) → Z
        thr = js.axis(3)                     # throttle lever (absolute)

        # throttle forward (negative) = fast; map -1..1 → 1..0 → speed range
        t01 = min(1.0, max(0.0, (1.0 - thr) / 2.0))
        gain = self.JOY_SPEED_MIN + t01 * (self.JOY_SPEED_MAX - self.JOY_SPEED_MIN)

        sx = -1.0 if self.inv_x.get() else 1.0
        sy = -1.0 if self.inv_y.get() else 1.0
        sz = -1.0 if self.inv_z.get() else 1.0
        vx, vy, vz = sx * ax * gain, sy * ay * gain, sz * az * gain
        if not deadman:
            vx = vy = vz = 0.0

        self.robot.speedl_set_target([vx, vy, vz, 0.0, 0.0, 0.0],
                                     acc_lin=400.0, acc_ang=80.0)

        self.joy_dead_lbl.config(
            text="● MOVING (trigger held)" if deadman else "○ hold trigger to move",
            fg=T.OK if deadman else T.WARN)
        self.joy_vel_lbl.config(
            text=f"vx {vx:+6.0f}  vy {vy:+6.0f}  vz {vz:+6.0f} mm/s")
        self.joy_gain_lbl.config(text=f"speed {gain:4.0f} mm/s")
        self.after(self.JOY_TICK_MS, self._joy_tick)

    def on_hide(self):
        self._joy_disarm()
        super().on_hide()


# ──────── 3. Incremental ───────────────────────────────────
