from console.base import *  # noqa: F401,F403
from console.base import _zyz_to_R, gripper_tip_world, camera_axis_world


class JointSliderScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "1. Joint Slider Jog", T.JOINT)

        self.add_intro(
            "Set absolute angles for each of the 6 joints, then click Send. "
            "movej (joint-space move) interpolates all joints together; the "
            "TCP follows a curved path. Slider ranges are pre-clamped to the "
            "URDF/controller limits so out-of-range commands can't be sent.")

        top = tk.Frame(self.body, bg=T.BG)
        top.pack(fill="x", pady=(0, 8))
        tk.Label(top, text="vel (°/s)", bg=T.BG, fg=T.LABEL).pack(side="left")
        self.vel_var = tk.DoubleVar(value=30.0)
        tk.Entry(top, textvariable=self.vel_var, width=6, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat").pack(side="left", padx=4)
        tk.Label(top, text="acc (°/s²)", bg=T.BG, fg=T.LABEL).pack(side="left", padx=(12, 0))
        self.acc_var = tk.DoubleVar(value=30.0)
        tk.Entry(top, textvariable=self.acc_var, width=6, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat").pack(side="left", padx=4)

        self.sliders = []
        self.cur_lbls = []
        rows = tk.Frame(self.body, bg=T.PANEL)
        rows.pack(fill="x", pady=4, ipady=8)
        for i in range(6):
            row = tk.Frame(rows, bg=T.PANEL)
            row.pack(fill="x", padx=12, pady=3)
            tk.Label(row, text=f"J{i+1}", width=3, bg=T.PANEL, fg=T.JOINT,
                     font=tkfont.Font(family="DejaVu Sans Mono", size=11, weight="bold")
                     ).pack(side="left")
            cur = tk.Label(row, text="   0.0°", width=8, bg=T.PANEL, fg=T.VAL,
                           font=tkfont.Font(family="DejaVu Sans Mono", size=10))
            cur.pack(side="left", padx=4)
            self.cur_lbls.append(cur)
            tk.Label(row, text="→", bg=T.PANEL, fg=T.DIM).pack(side="left", padx=2)
            lo, hi = JOINT_LIMITS[i]
            sl = tk.Scale(row, from_=lo, to=hi, orient="horizontal", length=380,
                          resolution=0.5, bg=T.PANEL, fg=T.LABEL,
                          troughcolor=T.PANEL_HI, highlightthickness=0,
                          activebackground=T.JOINT)
            sl.pack(side="left", expand=True, fill="x", padx=4)
            self.sliders.append(sl)

        btns = tk.Frame(self.body, bg=T.BG)
        btns.pack(fill="x", pady=10)
        big_button(btns, "Sync to current", self._sync, bg=T.PANEL_HI, fg=T.TITLE).pack(side="left", padx=4)
        big_button(btns, "Send (blocking)", lambda: self._send(0), bg=T.JOINT).pack(side="left", padx=4)
        big_button(btns, "Send (async)", lambda: self._send(1), bg=T.PANEL_HI, fg=T.TITLE).pack(side="left", padx=4)
        big_button(btns, "Home", self._home, bg=T.WARN).pack(side="left", padx=4)

        self.after(300, self._refresh)

    def _refresh(self):
        for i, lbl in enumerate(self.cur_lbls):
            lbl.config(text=f"{self.robot.joint_pos_deg[i]:+7.1f}°")
        self.after(300, self._refresh)

    def on_show(self):
        self._sync()

    def _sync(self):
        for i, sl in enumerate(self.sliders):
            sl.set(self.robot.joint_pos_deg[i])
        self.set_status("sliders synced to current pose", T.LABEL)

    def _send(self, sync):
        target = [sl.get() for sl in self.sliders]
        self.set_status(f"movej → {[f'{x:.1f}' for x in target]}", T.LABEL)
        self.run_async(
            lambda: self.robot.movej(target, vel=self.vel_var.get(),
                                     acc=self.acc_var.get(), sync=sync,
                                     confirm_real_callback=self.app.confirm_real_modal),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))

    def _home(self):
        for sl, deg in zip(self.sliders, HOME_POSE_DEG):
            sl.set(deg)
        self._send(0)


# ──────── 2. Task Space ────────────────────────────────────
