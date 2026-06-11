from console.base import *  # noqa: F401,F403
from console.base import _zyz_to_R, gripper_tip_world, camera_axis_world


class TaskSpaceScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "2. Task Space Move (TCP)", T.TASK)

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


# ──────── 3. Incremental ───────────────────────────────────
