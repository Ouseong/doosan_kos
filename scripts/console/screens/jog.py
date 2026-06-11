from console.base import *  # noqa: F401,F403
from console.base import _zyz_to_R, gripper_tip_world, camera_axis_world


class IncrementalJogScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "3. Incremental Jog (step)", T.INCR)

        self.add_intro(
            "Click +/- to nudge by exactly the selected step. Joint mode "
            "moves one joint by N degrees (clamped to limits). Task mode "
            "moves the TCP by N mm (X,Y,Z) or N degrees (Rx,Ry,Rz) in the "
            "BASE frame. Best for fine alignment / teaching positions.")

        top = tk.Frame(self.body, bg=T.BG)
        top.pack(fill="x", pady=(0, 8))

        tk.Label(top, text="mode:", bg=T.BG, fg=T.LABEL).pack(side="left")
        self.mode_var = tk.StringVar(value="joint")
        for m, lbl in [("joint", "Joint"), ("task", "Task")]:
            tk.Radiobutton(top, text=lbl, variable=self.mode_var, value=m,
                           bg=T.BG, fg=T.LABEL, selectcolor=T.PANEL_HI,
                           activebackground=T.BG, command=self._rebuild
                           ).pack(side="left", padx=4)

        tk.Label(top, text="step:", bg=T.BG, fg=T.LABEL).pack(side="left", padx=(20, 4))
        self.step_var = tk.DoubleVar(value=5.0)
        for s in [1, 5, 10, 30]:
            tk.Radiobutton(top, text=f"{s}", variable=self.step_var, value=float(s),
                           bg=T.BG, fg=T.LABEL, selectcolor=T.PANEL_HI,
                           activebackground=T.BG).pack(side="left", padx=2)
        self.unit_lbl = tk.Label(top, text="°", bg=T.BG, fg=T.DIM)
        self.unit_lbl.pack(side="left", padx=4)

        tk.Label(top, text="vel:", bg=T.BG, fg=T.LABEL).pack(side="left", padx=(20, 4))
        self.vel_var = tk.DoubleVar(value=30.0)
        tk.Entry(top, textvariable=self.vel_var, width=6, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat").pack(side="left")

        self.grid_holder = tk.Frame(self.body, bg=T.PANEL)
        self.grid_holder.pack(fill="x", pady=8, ipady=8)
        self.cur_lbls = []
        self._rebuild()

        info = tk.Label(self.body, bg=T.BG, fg=T.DIM, justify="left", anchor="w",
                        font=tkfont.Font(family="DejaVu Sans", size=9),
                        text=("Click +/- to nudge by the selected step.\n"
                              "Joint mode: J1..J6 in degrees → movej.\n"
                              "Task mode:  X/Y/Z (mm) and Rx/Ry/Rz (°) → movel relative to BASE."))
        info.pack(fill="x", pady=(8, 0))

        self.after(300, self._refresh)

    def _rebuild(self):
        for w in self.grid_holder.winfo_children():
            w.destroy()
        self.cur_lbls = []
        if self.mode_var.get() == "joint":
            self.unit_lbl.config(text="°")
            names = JOINT_NAMES
        else:
            self.unit_lbl.config(text="mm/°")
            names = TCP_AXES

        for i, name in enumerate(names):
            row = tk.Frame(self.grid_holder, bg=T.PANEL)
            row.pack(fill="x", padx=12, pady=2)
            tk.Label(row, text=name, width=4, bg=T.PANEL, fg=T.INCR,
                     font=tkfont.Font(family="DejaVu Sans Mono", size=11, weight="bold")
                     ).pack(side="left")
            cur = tk.Label(row, text="   ---", width=10, bg=T.PANEL, fg=T.VAL,
                           font=tkfont.Font(family="DejaVu Sans Mono", size=10))
            cur.pack(side="left", padx=6)
            self.cur_lbls.append(cur)
            big_button(row, f"− step", lambda i=i: self._jog(i, -1),
                       bg=T.PANEL_HI, fg=T.TITLE, height=1, width=8).pack(side="left", padx=2)
            big_button(row, f"+ step", lambda i=i: self._jog(i, +1),
                       bg=T.INCR, height=1, width=8).pack(side="left", padx=2)

    def _refresh(self):
        if self.mode_var.get() == "joint":
            for i, lbl in enumerate(self.cur_lbls):
                lbl.config(text=f"{self.robot.joint_pos_deg[i]:+7.1f}°")
        else:
            def fetch():
                return self.robot.get_current_posx(0)
            def update(p):
                if p and self.cur_lbls and self.mode_var.get() == "task":
                    for i, lbl in enumerate(self.cur_lbls):
                        lbl.config(text=f"{p[i]:+9.2f}")
            self.run_async(fetch, on_done=update)
        self.after(300, self._refresh)

    def _jog(self, idx, sign):
        step = self.step_var.get() * sign
        if self.mode_var.get() == "joint":
            target = list(self.robot.joint_pos_deg)
            target[idx] += step
            # clamp to limits
            lo, hi = JOINT_LIMITS[idx]
            target[idx] = max(lo, min(hi, target[idx]))
            self.set_status(f"J{idx+1} {step:+.1f}° → movej", T.LABEL)
            self.run_async(
                lambda: self.robot.movej(target, vel=self.vel_var.get(), acc=60.0,
                                         confirm_real_callback=self.app.confirm_real_modal),
                on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                                   T.OK if ok else T.BAD))
        else:
            cur = self.robot.get_current_posx(0)
            if not cur:
                self.set_status("could not read TCP pose", T.BAD)
                return
            target = list(cur)
            target[idx] += step
            self.set_status(f"{TCP_AXES[idx]} {step:+.1f} → movel", T.LABEL)
            self.run_async(
                lambda: self.robot.movel(target, vel_lin=self.vel_var.get(),
                                         vel_ang=self.vel_var.get(),
                                         confirm_real_callback=self.app.confirm_real_modal),
                on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                                   T.OK if ok else T.BAD))


# ──────── 4. Waypoint Recorder ─────────────────────────────
