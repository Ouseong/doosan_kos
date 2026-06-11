from console.base import *  # noqa: F401,F403
from console.base import _zyz_to_R, gripper_tip_world, camera_axis_world


class SpeedControlScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "5. Speed Control (deadman)", T.SPEED)

        self.add_intro(
            "Continuous joint-space velocity, like a teach-pendant jog. While "
            "you HOLD a +/- button, the joint spins at the selected magnitude "
            "(°/s). Releasing the button — or moving the cursor off — sends "
            "speedj 0 immediately. Closing this window also stops the stream. "
            "Each joint is independent. Stops automatically when joint nears its limit.")

        warn = tk.Label(self.body,
                        text="⚠ HOLD a button to move at that velocity. Release = STOP.",
                        bg=T.BAD, fg=T.BG, anchor="w",
                        font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold"))
        warn.pack(fill="x", pady=(0, 10), ipady=4)

        # global speed setting
        top = tk.Frame(self.body, bg=T.BG)
        top.pack(fill="x", pady=(0, 8))
        tk.Label(top, text="velocity magnitude (°/s):", bg=T.BG, fg=T.LABEL).pack(side="left")
        self.vmag = tk.DoubleVar(value=15.0)
        tk.Scale(top, from_=1.0, to=60.0, orient="horizontal", length=300,
                 resolution=1.0, variable=self.vmag,
                 bg=T.BG, fg=T.LABEL, troughcolor=T.PANEL_HI,
                 highlightthickness=0).pack(side="left", padx=8)

        # 6 joint rows
        rows = tk.Frame(self.body, bg=T.PANEL)
        rows.pack(fill="x", pady=4, ipady=8)
        for i in range(6):
            row = tk.Frame(rows, bg=T.PANEL)
            row.pack(fill="x", padx=12, pady=4)
            tk.Label(row, text=f"J{i+1}", width=4, bg=T.PANEL, fg=T.SPEED,
                     font=tkfont.Font(family="DejaVu Sans Mono", size=11, weight="bold")
                     ).pack(side="left")
            self._make_hold_btn(row, "− hold", i, -1)
            self._make_hold_btn(row, "+ hold", i, +1)

        # status / running indicator
        self.indicator = tk.Label(self.body, text="● IDLE", bg=T.BG, fg=T.DIM,
                                  font=tkfont.Font(family="DejaVu Sans", size=14, weight="bold"))
        self.indicator.pack(pady=12)

        info = tk.Label(self.body, bg=T.BG, fg=T.DIM, justify="left", anchor="w",
                        font=tkfont.Font(family="DejaVu Sans", size=9),
                        text=("Mouse-down on +/- starts speedj stream for that joint.\n"
                              "Mouse-up or leaving the screen sends speedj 0 immediately.\n"
                              "speedj_stream topic publishes at 20 Hz while held."))
        info.pack(fill="x", pady=(8, 0))

    def _make_hold_btn(self, parent, text, idx, sign):
        b = tk.Button(parent, text=text, bg=T.SPEED, fg=T.BG,
                      activebackground=T.WARN, relief="flat", bd=0,
                      width=10, height=1,
                      font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold"),
                      cursor="hand2")
        b.pack(side="left", padx=4)
        b.bind("<ButtonPress-1>", lambda e: self._press(idx, sign))
        b.bind("<ButtonRelease-1>", lambda e: self._release())
        b.bind("<Leave>", lambda e: self._release())  # safety: cursor leaves button = stop

    def _press(self, idx, sign):
        v = [0.0] * 6
        v[idx] = self.vmag.get() * sign
        self.robot.speedj_set_target(v, acc=30.0)
        self.robot.speedj_start()
        self.indicator.config(text=f"● MOVING J{idx+1} {sign:+d}", fg=T.SPEED)
        self.set_status(f"speedj J{idx+1} = {v[idx]:+.1f}°/s", T.SPEED)

    def _release(self):
        self.robot.speedj_stop()
        self.indicator.config(text="● IDLE", fg=T.DIM)
        self.set_status("stopped", T.LABEL)

    def on_hide(self):
        self._release()
        super().on_hide()


# ──────── 6. MoveIt2 ───────────────────────────────────────
