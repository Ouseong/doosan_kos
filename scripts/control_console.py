#!/usr/bin/env python3
"""M1013 Control Console — entry point.
Shared infra is in console/base.py; each mode screen in console/screens/.
"""
from console.base import *  # noqa: F401,F403
from console.screens.joint import JointSliderScreen
from console.screens.task import TaskSpaceScreen
from console.screens.jog import IncrementalJogScreen
from console.screens.waypoint import WaypointScreen
from console.screens.speed import SpeedControlScreen
from console.screens.moveit import MoveItScreen
from console.screens.distance import DistanceEstimatorScreen


class HomeScreen(tk.Frame):
    CARDS = [
        ("joint",  "Joint Slider Jog",  T.JOINT,
         "Drag 6 sliders → set joint angles → movej.",
         "Best for getting an exact joint configuration."),
        ("task",   "Task Space Move",   T.TASK,
         "Type X/Y/Z + Rx/Ry/Rz → MoveL or MoveJX.",
         "Most natural for tool-position-based tasks."),
        ("dist",   "Distance Estimator", T.CAM,
         "Capture at 2 TCP heights → estimate distance.",
         "RGB only — no depth sensor, no focal length needed."),
    ]

    def __init__(self, parent, app):
        super().__init__(parent, bg=T.BG)
        self.app = app

        # title bar
        title = tk.Frame(self, bg=T.BG)
        title.pack(fill="x", padx=20, pady=(20, 4))
        tk.Label(title, text="M1013 Control Console", bg=T.BG, fg=T.TITLE,
                 font=tkfont.Font(family="DejaVu Sans", size=20, weight="bold")
                 ).pack(side="left")

        # Mode button — top-right, opens connection settings dialog
        self.mode_btn = tk.Button(title, text="🟢 SIM",
                                  command=self._open_settings,
                                  bg=T.OK, fg=T.BG, relief="flat", bd=0,
                                  cursor="hand2", padx=14, pady=4,
                                  font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold"))
        self.mode_btn.pack(side="right", padx=(8, 0))

        self.live_lbl = tk.Label(title, text="● connecting", bg=T.BG, fg=T.DIM,
                                 font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold"))
        self.live_lbl.pack(side="right", padx=(0, 8))

        sub = tk.Label(self, text="Pick a control mode below.", bg=T.BG, fg=T.LABEL,
                       font=tkfont.Font(family="DejaVu Sans", size=11))
        sub.pack(anchor="w", padx=20, pady=(0, 10))

        # current pose strip
        strip = tk.Frame(self, bg=T.PANEL)
        strip.pack(fill="x", padx=20, pady=4, ipady=8)
        tk.Label(strip, text="current J:", bg=T.PANEL, fg=T.LABEL,
                 font=tkfont.Font(family="DejaVu Sans Mono", size=10)
                 ).pack(side="left", padx=(12, 4))
        self.pose_lbl = tk.Label(strip, text="...", bg=T.PANEL, fg=T.VAL,
                                 font=tkfont.Font(family="DejaVu Sans Mono", size=10))
        self.pose_lbl.pack(side="left")

        # safety strip
        safe = tk.Frame(self, bg=T.PANEL_HI)
        safe.pack(fill="x", padx=20, pady=(2, 4), ipady=6)
        tk.Label(safe, text="🛡 Safety:", bg=T.PANEL_HI, fg=T.OK,
                 font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold")
                 ).pack(side="left", padx=(12, 4))
        self.safety_lbl = tk.Label(safe, text="configuring ...", bg=T.PANEL_HI,
                                   fg=T.LABEL,
                                   font=tkfont.Font(family="DejaVu Sans", size=9))
        self.safety_lbl.pack(side="left")

        # 2x3 grid
        grid = tk.Frame(self, bg=T.BG)
        grid.pack(fill="both", expand=True, padx=20, pady=10)
        for r in range(2):
            grid.rowconfigure(r, weight=1)
        for c in range(3):
            grid.columnconfigure(c, weight=1)

        for idx, (key, ttl, accent, line1, line2) in enumerate(self.CARDS):
            r, c = divmod(idx, 3)
            self._make_card(grid, r, c, key, ttl, accent, line1, line2)

        # footer
        ft = tk.Label(self,
                      text="Tip: keep the Telemetry window open in parallel to watch live numbers.",
                      bg=T.BG, fg=T.DIM,
                      font=tkfont.Font(family="DejaVu Sans", size=9))
        ft.pack(side="bottom", pady=10)

        # Gripper footer
        self.gripper_panel = GripperPanel(self, app)
        self.gripper_panel.pack(side="bottom", fill="x", padx=20, pady=(2, 6))

        self.after(500, self._refresh)

    def _make_card(self, parent, row, col, key, title, accent, line1, line2):
        card = tk.Frame(parent, bg=T.PANEL, bd=0, highlightthickness=2,
                        highlightbackground=T.PANEL, highlightcolor=accent,
                        cursor="hand2")
        card.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")

        # accent bar
        tk.Frame(card, bg=accent, height=4).pack(fill="x")

        body = tk.Frame(card, bg=T.PANEL, padx=14, pady=12)
        body.pack(fill="both", expand=True)

        # number
        num_frame = tk.Frame(body, bg=T.PANEL)
        num_frame.pack(fill="x")
        tk.Label(num_frame, text=str(self.CARDS.index(
            next(c for c in self.CARDS if c[0] == key)) + 1),
                 bg=accent, fg=T.BG,
                 font=tkfont.Font(family="DejaVu Sans", size=12, weight="bold"),
                 width=3, height=1).pack(side="left")
        tk.Label(num_frame, text=" "+title, bg=T.PANEL, fg=T.TITLE,
                 font=tkfont.Font(family="DejaVu Sans", size=13, weight="bold")
                 ).pack(side="left", padx=4)

        tk.Label(body, text=line1, bg=T.PANEL, fg=T.VAL,
                 font=tkfont.Font(family="DejaVu Sans", size=10),
                 wraplength=240, justify="left", anchor="w"
                 ).pack(fill="x", pady=(8, 2))
        tk.Label(body, text=line2, bg=T.PANEL, fg=T.DIM,
                 font=tkfont.Font(family="DejaVu Sans", size=9),
                 wraplength=240, justify="left", anchor="w"
                 ).pack(fill="x")

        # whole card click → navigate
        def click(_e=None):
            self.app.show_screen(key)

        for w in [card, body, num_frame] + list(body.winfo_children()) + list(num_frame.winfo_children()):
            w.bind("<Button-1>", click)

        # hover effect
        def enter(_e):
            card.config(highlightbackground=accent)
        def leave(_e):
            card.config(highlightbackground=T.PANEL)
        card.bind("<Enter>", enter)
        card.bind("<Leave>", leave)

    def _open_settings(self):
        dlg = tk.Toplevel(self.app.root)
        dlg.title("Connection Settings")
        dlg.configure(bg=T.BG)
        dlg.geometry("440x340")
        dlg.transient(self.app.root)

        tk.Label(dlg, text="Target Mode", bg=T.BG, fg=T.TITLE,
                 font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold")
                 ).pack(anchor="w", padx=16, pady=(14, 4))
        mode_var = tk.StringVar(value=self.app.robot.target_mode)
        modes = [
            ("sim",     "🟢 SIM only       — commands go to the emulator (safe)"),
            ("real",    "🔴 REAL only      — commands go straight to the real robot"),
            ("preview", "🎯 PREVIEW→REAL — sim first, ask, then real (recommended for new programs)"),
        ]
        for val, lbl in modes:
            tk.Radiobutton(dlg, text=lbl, variable=mode_var, value=val,
                           bg=T.BG, fg=T.LABEL, selectcolor=T.PANEL_HI,
                           activebackground=T.BG, anchor="w", justify="left",
                           font=tkfont.Font(family="DejaVu Sans", size=10)
                           ).pack(fill="x", padx=24, pady=2)

        tk.Frame(dlg, height=1, bg=T.BORDER).pack(fill="x", padx=16, pady=10)

        tk.Label(dlg, text="Real Robot Connection", bg=T.BG, fg=T.TITLE,
                 font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold")
                 ).pack(anchor="w", padx=16, pady=(0, 4))

        ip_row = tk.Frame(dlg, bg=T.BG); ip_row.pack(fill="x", padx=24, pady=4)
        tk.Label(ip_row, text="Robot IP:", bg=T.BG, fg=T.LABEL, width=10, anchor="w"
                 ).pack(side="left")
        ip_var = tk.StringVar(value=self.app.robot.real_ip)
        tk.Entry(ip_row, textvariable=ip_var, width=18, bg=T.PANEL, fg=T.VAL,
                 insertbackground=T.VAL, relief="flat",
                 font=tkfont.Font(family="DejaVu Sans Mono", size=10)
                 ).pack(side="left", padx=4)

        def describe_real_state():
            """Resolve the true real-robot status for the status_lbl text.
            Returns (text, color). ROS-service registration alone is shown
            distinct from DRCF actually being in a motion-ready state."""
            r = self.app.robot
            if not r.real_driver_started:
                return "not started", T.DIM
            code, name = r.query_real_robot_state(timeout=1.0)
            if code is None:
                return "✓ ros2 connected (servo state unknown)", T.WARN
            if code in ROBOT_STATE_READY:
                return f"ready - {name}", T.OK
            if code == 3:    # SAFE_OFF
                return "connected, SERVO OFF - press 'Start Real Driver' to recover", T.WARN
            if code == 6:    # EMERGENCY_STOP
                return "EMERGENCY_STOP - clear alarm on pendant first", T.BAD
            if code in (5, 9):  # SAFE_STOP variants
                return f"{name} - clear on pendant", T.WARN
            return f"connected - robot_state = {name}", T.WARN

        status_text, status_color = describe_real_state()
        status_lbl = tk.Label(dlg, text=f"Real driver: {status_text}",
                              bg=T.BG, fg=status_color,
                              font=tkfont.Font(family="DejaVu Sans", size=9))
        status_lbl.pack(anchor="w", padx=24, pady=(4, 8))

        def refresh_status():
            # Dialog may have been closed by the user while a background worker
            # is still updating; the Tk widget then no longer exists.
            try:
                if not status_lbl.winfo_exists():
                    return
            except tk.TclError:
                return
            text, color = describe_real_state()
            try:
                status_lbl.config(text=f"Real driver: {text}", fg=color)
            except tk.TclError:
                pass

        def start_real():
            ip = ip_var.get().strip()
            if not ip:
                status_lbl.config(text="enter an IP first", fg=T.BAD)
                return
            status_lbl.config(text=f"Real driver: starting at {ip} ...", fg=T.WARN)

            def safe_set_status(text, color):
                """Update status_lbl from a worker thread without crashing
                if the user has already closed the dialog (Tk destroys the
                widget; subsequent .config() calls raise TclError)."""
                def apply():
                    try:
                        if status_lbl.winfo_exists():
                            status_lbl.config(text=text, fg=color)
                    except tk.TclError:
                        pass
                self.app.root.after(0, apply)

            def worker():
                # start_real_driver now ALWAYS kills any existing real driver
                # first and waits for STATE_STANDBY before returning True, so
                # success here implies servo is on (motor torque is clicking).
                ok = self.app.robot.start_real_driver(ip)
                if not ok:
                    safe_set_status(
                        f"Real driver: failed to reach {ip} (no STATE_STANDBY in 30s)",
                        T.BAD)
                    return
                self.app.robot.query_real_robot_state(timeout=1.5)
                self.app.root.after(0, refresh_status)
            threading.Thread(target=worker, daemon=True).start()

        tk.Button(dlg, text="Start Real Driver", command=start_real,
                  bg=T.BAD, fg=T.BG, relief="flat", bd=0,
                  cursor="hand2", padx=14, pady=4,
                  font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold")
                  ).pack(anchor="w", padx=24)

        tk.Frame(dlg, height=1, bg=T.BORDER).pack(fill="x", padx=16, pady=14)

        def show_not_connected_modal():
            sub = tk.Toplevel(dlg)
            sub.title("Real Robot Not Connected")
            sub.configure(bg=T.BG)
            sub.transient(dlg)
            sub.grab_set()
            sub.resizable(False, False)
            w, h = 460, 280
            dlg.update_idletasks()
            x = dlg.winfo_rootx() + (dlg.winfo_width()  - w) // 2
            y = dlg.winfo_rooty() + (dlg.winfo_height() - h) // 2
            sub.geometry(f"{w}x{h}+{max(0, x)}+{max(0, y)}")

            header = tk.Frame(sub, bg=T.WARN, height=64)
            header.pack(fill="x"); header.pack_propagate(False)
            tk.Label(header, text="⚠  REAL ROBOT NOT CONNECTED",
                     bg=T.WARN, fg=T.BG,
                     font=tkfont.Font(family="DejaVu Sans", size=14, weight="bold")
                    ).pack(pady=18)

            body = tk.Frame(sub, bg=T.BG)
            body.pack(fill="both", expand=True, padx=24, pady=18)

            tk.Label(body,
                     text="Real driver is not connected yet.\nOnly SIM mode is available right now.",
                     bg=T.BG, fg=T.VAL, justify="center",
                     font=tkfont.Font(family="DejaVu Sans", size=11)
                    ).pack(pady=(2, 10))
            tk.Label(body,
                     text="To use REAL / PREVIEW mode, click\n'Start Real Driver' first.",
                     bg=T.BG, fg=T.DIM, justify="center",
                     font=tkfont.Font(family="DejaVu Sans", size=9)
                    ).pack(pady=(0, 14))

            def close():
                sub.destroy()

            tk.Button(body, text="OK", command=close,
                      bg=T.JOINT, fg=T.BG, activebackground=T.PANEL_HI,
                      font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold"),
                      relief="flat", padx=28, pady=10, cursor="hand2", bd=0,
                     ).pack(side="bottom")

            sub.protocol("WM_DELETE_WINDOW", close)
            sub.bind("<Escape>", lambda _e: close())
            sub.bind("<Return>", lambda _e: close())
            sub.wait_window()

        def apply_and_close():
            new_mode = mode_var.get()
            if new_mode in ("real", "preview"):
                r = self.app.robot
                if not r.real_driver_started:
                    show_not_connected_modal()
                    return
                # Re-query state right before committing — DRCF can drop to
                # SAFE_OFF mid-session (e.g. mid-demo collision), and the
                # cached value would silently let commands go to a robot
                # that won't move.
                code, name = r.query_real_robot_state(timeout=1.0)
                if code is not None and code not in ROBOT_STATE_READY:
                    refresh_status()
                    status_lbl.config(
                        text=f"Real driver: {name} - mode switch blocked, check pendant",
                        fg=T.BAD)
                    return
            self.app.robot.target_mode = new_mode
            self.app.robot.real_ip = ip_var.get().strip()
            dlg.destroy()

        btns = tk.Frame(dlg, bg=T.BG); btns.pack(pady=4)
        tk.Button(btns, text="Apply", command=apply_and_close,
                  bg=T.JOINT, fg=T.BG, relief="flat", bd=0, cursor="hand2",
                  padx=18, pady=4,
                  font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold")
                  ).pack(side="left", padx=4)
        tk.Button(btns, text="Cancel", command=dlg.destroy,
                  bg=T.PANEL_HI, fg=T.TITLE, relief="flat", bd=0, cursor="hand2",
                  padx=18, pady=4,
                  font=tkfont.Font(family="DejaVu Sans", size=10)
                  ).pack(side="left", padx=4)

    def _refresh(self):
        # mode button
        mode = self.app.robot.target_mode
        if mode == "sim":
            self.mode_btn.config(text="🟢 SIM", bg=T.OK)
        elif mode == "real":
            self.mode_btn.config(text="🔴 REAL", bg=T.BAD)
        else:
            self.mode_btn.config(text="🎯 PREVIEW→REAL", bg=T.WARN)

        pos = self.app.robot.joint_pos_deg
        self.pose_lbl.config(text="  ".join(f"J{i+1}:{p:+7.1f}°" for i, p in enumerate(pos)))
        # service ready check
        ready = self.app.robot.cli_movej.service_is_ready()
        if ready:
            self.live_lbl.config(text="● connected", fg=T.OK)
        else:
            self.live_lbl.config(text="● waiting for driver", fg=T.WARN)
        # safety status
        sm = self.app.robot.singularity_mode
        cs = self.app.robot.collision_sensitivity
        sing_txt = ["AVOID", "TASK_STOP", "VAR_VEL"][sm] if sm in (0, 1, 2) else "—"
        self.safety_lbl.config(text=(
            f"singularity={sing_txt}   "
            f"collision_sensitivity={cs if cs is not None else '—'}   "
            f"J2 ±95° (datasheet)   "
            f"fkin TCP-Z ≥ {TCP_Z_MIN_MM:.0f}mm"))
        self.after(500, self._refresh)


# ──────── App ──────────────────────────────────────────────
class App:
    def __init__(self):
        rclpy.init()
        self.robot = RobotInterface()
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.robot,), daemon=True)
        self.spin_thread.start()
        # ensure autonomous + enable safety features (best-effort)
        def _setup():
            self.robot.ensure_autonomous()
            self.robot.configure_safety()
        threading.Thread(target=_setup, daemon=True).start()

        self.root = tk.Tk()
        self.root.title("M1013 Control Console")
        self.root.configure(bg=T.BG)
        self.root.geometry("960x680")
        self.root.minsize(860, 600)

        # ttk styling for the OptionMenu/Scrollbar where used
        self.container = tk.Frame(self.root, bg=T.BG)
        self.container.pack(fill="both", expand=True)

        self.home = HomeScreen(self.container, self)
        self.screens = {
            "joint":  JointSliderScreen(self.container, self),
            "task":   TaskSpaceScreen(self.container, self),
            "dist":   DistanceEstimatorScreen(self.container, self),
        }
        for s in [self.home, *self.screens.values()]:
            s.place(x=0, y=0, relwidth=1, relheight=1)

        self.current = None
        self.show_home()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def show_home(self):
        if self.current and hasattr(self.current, "on_hide"):
            self.current.on_hide()
        self.home.tkraise()
        self.current = self.home

    def show_screen(self, key):
        if self.current and hasattr(self.current, "on_hide"):
            self.current.on_hide()
        s = self.screens[key]
        s.tkraise()
        s.on_show()
        self.current = s

    def _on_close(self):
        # safety: cancel any motion before closing
        try:
            self.robot.cancel_motion(stop_mode=3)
        except Exception:
            pass
        self.root.destroy()

    def confirm_real_modal(self, target_pos, kind="movej"):
        """Called from a worker thread; pops a Tk modal asking the user
        to confirm sending the same command to the real robot. Blocks
        until the user answers (or 5 min timeout). `kind` is the motion
        type ("movej" | "movejx" | "movel") and controls how the target
        is labeled in the UI."""
        event = threading.Event()
        answer = [False]

        def show():
            dlg = tk.Toplevel(self.root)
            dlg.title("Apply to Real Robot?")
            dlg.configure(bg=T.BG)
            dlg.transient(self.root)
            dlg.grab_set()
            dlg.resizable(False, False)
            w, h = 520, 360
            self.root.update_idletasks()
            x = self.root.winfo_x() + (self.root.winfo_width()  - w) // 2
            y = self.root.winfo_y() + (self.root.winfo_height() - h) // 2
            dlg.geometry(f"{w}x{h}+{max(0, x)}+{max(0, y)}")

            # Warning header bar
            header = tk.Frame(dlg, bg=T.BAD, height=72)
            header.pack(fill="x"); header.pack_propagate(False)
            tk.Label(header, text="⚠  REAL ROBOT MOTION",
                     bg=T.BAD, fg="#ffffff",
                     font=tkfont.Font(family="DejaVu Sans", size=18, weight="bold")
                    ).pack(pady=18)

            body = tk.Frame(dlg, bg=T.BG)
            body.pack(fill="both", expand=True, padx=22, pady=14)

            tk.Label(body,
                     text="Sim execution finished. Send the same command to the REAL robot?",
                     bg=T.BG, fg=T.LABEL, anchor="w", justify="left",
                     font=tkfont.Font(family="DejaVu Sans", size=11)
                    ).pack(fill="x", pady=(0, 12))

            # Info panel: command kind / robot IP / target values
            panel = tk.Frame(body, bg=T.PANEL)
            panel.pack(fill="x", pady=(0, 4))

            def _info_row(parent, label, value, value_color=T.VAL, bold=False):
                row = tk.Frame(parent, bg=T.PANEL)
                row.pack(fill="x", padx=14, pady=4)
                tk.Label(row, text=label, bg=T.PANEL, fg=T.DIM, width=10, anchor="w",
                         font=tkfont.Font(family="DejaVu Sans Mono", size=10)
                        ).pack(side="left")
                tk.Label(row, text=value, bg=T.PANEL, fg=value_color, anchor="w",
                         justify="left", wraplength=380,
                         font=tkfont.Font(family="DejaVu Sans Mono", size=11,
                                          weight=("bold" if bold else "normal"))
                        ).pack(side="left", fill="x", expand=True)

            _info_row(panel, "Command:", kind, value_color=T.WARN, bold=True)
            _info_row(panel, "Robot IP:", self.robot.real_ip)

            if kind == "movej":
                labels, unit = JOINT_NAMES, "°"
                target_str = "   ".join(
                    f"{labels[i]}={v:+.1f}{unit}" for i, v in enumerate(target_pos))
            else:  # movejx / movel — TCP pose: mm for X/Y/Z, ° for Rx/Ry/Rz
                target_str = "   ".join(
                    f"{TCP_AXES[i]}={target_pos[i]:+.1f}{'mm' if i < 3 else '°'}"
                    for i in range(min(6, len(target_pos))))

            row = tk.Frame(panel, bg=T.PANEL)
            row.pack(fill="x", padx=14, pady=(4, 10))
            tk.Label(row, text="Target:", bg=T.PANEL, fg=T.DIM, width=10, anchor="nw",
                     font=tkfont.Font(family="DejaVu Sans Mono", size=10)
                    ).pack(side="left")
            tk.Label(row, text=target_str, bg=T.PANEL, fg=T.VAL, anchor="w",
                     justify="left", wraplength=380,
                     font=tkfont.Font(family="DejaVu Sans Mono", size=10)
                    ).pack(side="left", fill="x", expand=True)

            # Buttons
            btns = tk.Frame(body, bg=T.BG)
            btns.pack(fill="x", side="bottom", pady=(16, 0))

            def on_send():   answer[0] = True;  dlg.destroy()
            def on_cancel(): answer[0] = False; dlg.destroy()

            tk.Button(btns, text="Cancel  (sim only)", command=on_cancel,
                      bg=T.PANEL_HI, fg=T.TITLE, activebackground=T.PANEL,
                      font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold"),
                      relief="flat", padx=18, pady=12, cursor="hand2"
                     ).pack(side="left", expand=True, fill="x", padx=(0, 6))

            tk.Button(btns, text="▶  Send to REAL", command=on_send,
                      bg=T.BAD, fg="#ffffff", activebackground="#a02530",
                      font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold"),
                      relief="flat", padx=18, pady=12, cursor="hand2"
                     ).pack(side="right", expand=True, fill="x", padx=(6, 0))

            dlg.protocol("WM_DELETE_WINDOW", on_cancel)
            dlg.bind("<Escape>", lambda _e: on_cancel())

            dlg.wait_window()
            event.set()

        self.root.after(0, show)
        event.wait(timeout=300)
        return answer[0]

    def run(self):
        self.root.mainloop()


def main():
    app = App()
    try:
        app.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

