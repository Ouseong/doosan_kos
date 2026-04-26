#!/usr/bin/env python3
"""
M1013 Control Console — multi-mode control dashboard.

Home screen has 6 cards. Click a card → that mode's screen opens.
Click "← Home" on any screen → back to dashboard.

Modes:
  1. Joint Slider Jog   — 6 sliders, target joint angles, movej
  2. Task Space Move    — X/Y/Z/Rx/Ry/Rz, movel (linear) or movejx
  3. Incremental Jog    — +/- buttons, joint or task, fixed step
  4. Waypoint Recorder  — save current pose, replay sequence
  5. Speed Control      — hold-to-jog, speedj velocity stream (deadman)
  6. MoveIt2            — launch full MoveIt2 stack (separate window)
"""

import json
import math
import os
import subprocess
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
import tkinter.filedialog as filedialog
import tkinter.messagebox as messagebox
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.msg import SpeedjStream
from dsr_msgs2.srv import (
    MoveJoint,
    MoveJointx,
    MoveLine,
    GetCurrentPosj,
    GetCurrentPosx,
    SetRobotMode,
)

ROBOT_ID = "dsr01"
SVC = f"/{ROBOT_ID}/dsr_controller2"
WAYPOINTS_FILE = Path("/tmp/m1013_waypoints.json")


# ──────── Theme ────────────────────────────────────────────
class T:
    BG          = "#1e1e2e"
    PANEL       = "#252537"
    PANEL_HI    = "#313244"
    BORDER      = "#45475a"
    TITLE       = "#cdd6f4"
    LABEL       = "#a6adc8"
    VAL         = "#f5f5f5"
    DIM         = "#6c7086"
    OK          = "#a6e3a1"
    WARN        = "#f9e2af"
    BAD         = "#f38ba8"
    # mode accents
    JOINT       = "#74c7ec"  # cyan
    TASK        = "#cba6f7"  # purple
    INCR        = "#f9e2af"  # yellow
    WP          = "#a6e3a1"  # green
    SPEED       = "#fab387"  # orange
    MOVEIT      = "#f38ba8"  # red


# ──────── M1013 joint limits (deg) — conservative ──────────
JOINT_LIMITS = [
    (-360, 360),  # J1
    (-95,   95),  # J2
    (-135, 135),  # J3
    (-360, 360),  # J4
    (-135, 135),  # J5
    (-360, 360),  # J6
]

JOINT_NAMES = [f"J{i+1}" for i in range(6)]
TCP_AXES = ["X", "Y", "Z", "Rx", "Ry", "Rz"]


# ──────── Robot Interface ──────────────────────────────────
class RobotInterface(Node):
    """All ROS2 motion plumbing in one place."""

    def __init__(self):
        super().__init__("control_console_node", namespace=ROBOT_ID)

        # subscriptions
        self.create_subscription(JointState, f"/{ROBOT_ID}/joint_states", self._on_js, 10)
        self.joint_pos_deg = [0.0] * 6
        self.joint_vel_deg = [0.0] * 6

        # service clients
        self.cli_movej = self.create_client(MoveJoint, f"{SVC}/motion/move_joint")
        self.cli_movejx = self.create_client(MoveJointx, f"{SVC}/motion/move_jointx")
        self.cli_movel = self.create_client(MoveLine, f"{SVC}/motion/move_line")
        self.cli_posj = self.create_client(GetCurrentPosj, f"{SVC}/aux_control/get_current_posj")
        self.cli_posx = self.create_client(GetCurrentPosx, f"{SVC}/aux_control/get_current_posx")
        self.cli_mode = self.create_client(SetRobotMode, f"{SVC}/system/set_robot_mode")

        # speedj publisher
        self.pub_speedj = self.create_publisher(SpeedjStream, f"{SVC}/speedj_stream", 10)
        self._speedj_running = False
        self._speedj_thread = None
        self._speedj_target = [0.0] * 6
        self._speedj_acc = 30.0

    def _on_js(self, msg: JointState):
        if len(msg.position) >= 6:
            self.joint_pos_deg = [math.degrees(p) for p in msg.position[:6]]
        if len(msg.velocity) >= 6:
            self.joint_vel_deg = [math.degrees(v) for v in msg.velocity[:6]]

    def _wait(self, future, timeout):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return None
        return future.result()

    def ensure_autonomous(self) -> bool:
        if not self.cli_mode.wait_for_service(timeout_sec=2.0):
            return False
        req = SetRobotMode.Request()
        req.robot_mode = 1
        r = self._wait(self.cli_mode.call_async(req), 3.0)
        return bool(r and r.success)

    # ── service-backed motion ──
    def movej(self, pos_deg, vel=30.0, acc=30.0, sync=0) -> bool:
        if not self.cli_movej.wait_for_service(timeout_sec=2.0):
            return False
        req = MoveJoint.Request()
        req.pos = [float(p) for p in pos_deg]
        req.vel = float(vel); req.acc = float(acc)
        req.time = 0.0; req.radius = 0.0
        req.mode = 0; req.blend_type = 0; req.sync_type = int(sync)
        r = self._wait(self.cli_movej.call_async(req), 120.0)
        return bool(r and r.success)

    def movejx(self, posx, vel=30.0, acc=30.0, ref=0, sol=2, sync=0) -> bool:
        if not self.cli_movejx.wait_for_service(timeout_sec=2.0):
            return False
        req = MoveJointx.Request()
        req.pos = [float(p) for p in posx]
        req.vel = float(vel); req.acc = float(acc)
        req.time = 0.0; req.radius = 0.0
        req.ref = int(ref); req.mode = 0
        req.blend_type = 0; req.sol = int(sol); req.sync_type = int(sync)
        r = self._wait(self.cli_movejx.call_async(req), 120.0)
        return bool(r and r.success)

    def movel(self, posx, vel_lin=100.0, vel_ang=30.0, acc_lin=200.0, acc_ang=60.0,
              ref=0, sync=0) -> bool:
        if not self.cli_movel.wait_for_service(timeout_sec=2.0):
            return False
        req = MoveLine.Request()
        req.pos = [float(p) for p in posx]
        req.vel = [float(vel_lin), float(vel_ang)]
        req.acc = [float(acc_lin), float(acc_ang)]
        req.time = 0.0; req.radius = 0.0
        req.ref = int(ref); req.mode = 0
        req.blend_type = 0; req.sync_type = int(sync)
        r = self._wait(self.cli_movel.call_async(req), 120.0)
        return bool(r and r.success)

    def get_current_posj(self):
        if not self.cli_posj.wait_for_service(timeout_sec=1.0):
            return None
        r = self._wait(self.cli_posj.call_async(GetCurrentPosj.Request()), 2.0)
        if r and r.success:
            return list(r.pos)
        return None

    def get_current_posx(self, ref=0):
        if not self.cli_posx.wait_for_service(timeout_sec=1.0):
            return None
        req = GetCurrentPosx.Request(); req.ref = int(ref)
        r = self._wait(self.cli_posx.call_async(req), 2.0)
        if r and r.success and r.task_pos_info:
            data = list(r.task_pos_info[0].data)
            if len(data) >= 6:
                return data[:6]
        return None

    # ── speedj continuous stream ──
    def speedj_set_target(self, vel_deg, acc=30.0):
        self._speedj_target = list(vel_deg[:6])
        self._speedj_acc = float(acc)

    def speedj_start(self):
        if self._speedj_running:
            return
        self._speedj_running = True

        def loop():
            while self._speedj_running:
                msg = SpeedjStream()
                msg.vel = list(self._speedj_target)
                msg.acc = [self._speedj_acc] * 6
                msg.time = 0.05  # 50ms blend
                self.pub_speedj.publish(msg)
                time.sleep(0.05)

        self._speedj_thread = threading.Thread(target=loop, daemon=True)
        self._speedj_thread.start()

    def speedj_stop(self):
        self._speedj_running = False
        # send zeros once to cleanly stop
        msg = SpeedjStream()
        msg.vel = [0.0] * 6
        msg.acc = [self._speedj_acc] * 6
        msg.time = 0.0
        for _ in range(3):
            self.pub_speedj.publish(msg)
            time.sleep(0.02)


# ──────── GUI helpers ──────────────────────────────────────
def big_button(parent, text, command, bg, fg=T.BG, height=2, width=14, **kw):
    b = tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                  activebackground=bg, activeforeground=fg,
                  relief="flat", bd=0, height=height, width=width,
                  font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold"),
                  cursor="hand2", **kw)
    return b


def section_label(parent, text, accent=T.TITLE):
    return tk.Label(parent, text=text, bg=T.BG, fg=accent,
                    font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold"))


# ──────── Base screen ──────────────────────────────────────
class ModeScreen(tk.Frame):
    def __init__(self, parent, app, title, accent):
        super().__init__(parent, bg=T.BG)
        self.app = app
        self.robot = app.robot
        self.title = title
        self.accent = accent
        self._build_header()
        self.body = tk.Frame(self, bg=T.BG)
        self.body.pack(fill="both", expand=True, padx=14, pady=(0, 14))

    def _build_header(self):
        hdr = tk.Frame(self, bg=T.BG)
        hdr.pack(fill="x", padx=14, pady=(14, 6))

        back = big_button(hdr, "← Home", self.app.show_home,
                          bg=T.PANEL_HI, fg=T.TITLE, height=1, width=10)
        back.pack(side="left")

        # accent bar + title
        bar = tk.Frame(hdr, bg=self.accent, width=4)
        bar.pack(side="left", fill="y", padx=(20, 8))
        tk.Label(hdr, text=self.title, bg=T.BG, fg=T.TITLE,
                 font=tkfont.Font(family="DejaVu Sans", size=15, weight="bold")
                 ).pack(side="left")

        self.status_lbl = tk.Label(hdr, text="ready", bg=T.BG, fg=T.DIM,
                                   font=tkfont.Font(family="DejaVu Sans", size=9))
        self.status_lbl.pack(side="right")

    def set_status(self, msg, color=T.DIM):
        self.status_lbl.config(text=msg, fg=color)

    def on_show(self):
        """Override to refresh state when screen activated."""
        pass

    def on_hide(self):
        """Override to release resources when leaving."""
        pass

    # helper: run in worker thread, status updates marshalled to main
    def run_async(self, work_fn, on_done=None):
        def worker():
            try:
                result = work_fn()
            except Exception as e:
                self.after(0, lambda: self.set_status(f"error: {e}", T.BAD))
                return
            if on_done:
                self.after(0, lambda: on_done(result))
        threading.Thread(target=worker, daemon=True).start()


# ──────── 1. Joint Slider ──────────────────────────────────
class JointSliderScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "1. Joint Slider Jog", T.JOINT)

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
        big_button(btns, "Home (all 0°)", self._home, bg=T.WARN).pack(side="left", padx=4)

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
                                     acc=self.acc_var.get(), sync=sync),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))

    def _home(self):
        for sl in self.sliders:
            sl.set(0.0)
        self._send(0)


# ──────── 2. Task Space ────────────────────────────────────
class TaskSpaceScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "2. Task Space Move (TCP)", T.TASK)

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
                                     vel_ang=self.vang.get(), ref=ref),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))

    def _movejx(self):
        target = [e.get() for e in self.entries]
        ref = self._ref_int()
        self.set_status(f"movejx → {target}", T.LABEL)
        self.run_async(
            lambda: self.robot.movejx(target, vel=self.vang.get(),
                                      acc=60.0, ref=ref),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))


# ──────── 3. Incremental ───────────────────────────────────
class IncrementalJogScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "3. Incremental Jog (step)", T.INCR)

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
                lambda: self.robot.movej(target, vel=self.vel_var.get(), acc=60.0),
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
                                         vel_ang=self.vel_var.get()),
                on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                                   T.OK if ok else T.BAD))


# ──────── 4. Waypoint Recorder ─────────────────────────────
class WaypointScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "4. Waypoint Recorder", T.WP)
        self.waypoints = []  # list of dicts {posj: [...], posx: [...]}

        top = tk.Frame(self.body, bg=T.BG)
        top.pack(fill="x", pady=(0, 8))
        big_button(top, "+ Save Current", self._save_current, bg=T.WP).pack(side="left", padx=2)
        big_button(top, "↑ Move Up", self._move_up, bg=T.PANEL_HI, fg=T.TITLE,
                   height=1).pack(side="left", padx=2)
        big_button(top, "↓ Move Down", self._move_down, bg=T.PANEL_HI, fg=T.TITLE,
                   height=1).pack(side="left", padx=2)
        big_button(top, "✕ Delete", self._delete, bg=T.BAD, fg=T.BG,
                   height=1).pack(side="left", padx=2)
        big_button(top, "→ Go to Selected", self._goto, bg=T.PANEL_HI, fg=T.TITLE
                   ).pack(side="left", padx=10)
        big_button(top, "▶ Play All", self._play_all, bg=T.WP).pack(side="left", padx=2)

        files = tk.Frame(self.body, bg=T.BG)
        files.pack(fill="x", pady=4)
        big_button(files, "Save to file...", self._save_file, bg=T.PANEL_HI, fg=T.TITLE,
                   height=1).pack(side="left", padx=2)
        big_button(files, "Load from file...", self._load_file, bg=T.PANEL_HI, fg=T.TITLE,
                   height=1).pack(side="left", padx=2)
        tk.Label(files, text="(default location: /tmp/m1013_waypoints.json)",
                 bg=T.BG, fg=T.DIM, font=tkfont.Font(family="DejaVu Sans", size=8)
                 ).pack(side="left", padx=10)

        # listbox
        lstwrap = tk.Frame(self.body, bg=T.PANEL)
        lstwrap.pack(fill="both", expand=True, pady=8)
        scroll = tk.Scrollbar(lstwrap, bg=T.PANEL)
        scroll.pack(side="right", fill="y")
        self.listbox = tk.Listbox(lstwrap, bg=T.PANEL, fg=T.VAL,
                                  selectbackground=T.WP, selectforeground=T.BG,
                                  font=tkfont.Font(family="DejaVu Sans Mono", size=10),
                                  yscrollcommand=scroll.set, relief="flat",
                                  highlightthickness=0, activestyle="none")
        self.listbox.pack(fill="both", expand=True, padx=4, pady=4)
        scroll.config(command=self.listbox.yview)

        info = tk.Label(self.body, bg=T.BG, fg=T.DIM, anchor="w",
                        font=tkfont.Font(family="DejaVu Sans", size=9),
                        text=("Each waypoint stores BOTH joint angles (posj) and TCP pose (posx).\n"
                              "Playback uses movej (joint-space). Use 'Go to Selected' for single jumps."))
        info.pack(fill="x")

    def on_show(self):
        # auto-load if file exists
        if not self.waypoints and WAYPOINTS_FILE.exists():
            try:
                with open(WAYPOINTS_FILE) as f:
                    data = json.load(f)
                self.waypoints = data.get("waypoints", [])
                self._refresh_list()
            except Exception:
                pass

    def _refresh_list(self):
        self.listbox.delete(0, tk.END)
        for i, wp in enumerate(self.waypoints):
            posj = [f"{x:+6.1f}" for x in wp["posj"]]
            self.listbox.insert(tk.END, f"WP{i+1:02d}  posj=[{', '.join(posj)}]")

    def _selected_index(self):
        sel = self.listbox.curselection()
        return sel[0] if sel else None

    def _save_current(self):
        def fetch():
            return self.robot.get_current_posj(), self.robot.get_current_posx(0)
        def update(result):
            posj, posx = result
            if not posj or not posx:
                self.set_status("could not capture pose", T.BAD)
                return
            self.waypoints.append({"posj": posj, "posx": posx})
            self._refresh_list()
            self.set_status(f"saved WP{len(self.waypoints)}", T.OK)
        self.run_async(fetch, on_done=update)

    def _move_up(self):
        i = self._selected_index()
        if i is None or i == 0: return
        self.waypoints[i-1], self.waypoints[i] = self.waypoints[i], self.waypoints[i-1]
        self._refresh_list()
        self.listbox.selection_set(i-1)

    def _move_down(self):
        i = self._selected_index()
        if i is None or i >= len(self.waypoints)-1: return
        self.waypoints[i], self.waypoints[i+1] = self.waypoints[i+1], self.waypoints[i]
        self._refresh_list()
        self.listbox.selection_set(i+1)

    def _delete(self):
        i = self._selected_index()
        if i is None: return
        del self.waypoints[i]
        self._refresh_list()
        self.set_status(f"deleted WP at index {i+1}", T.LABEL)

    def _goto(self):
        i = self._selected_index()
        if i is None:
            self.set_status("select a waypoint first", T.WARN)
            return
        wp = self.waypoints[i]
        self.set_status(f"movej → WP{i+1}", T.LABEL)
        self.run_async(
            lambda: self.robot.movej(wp["posj"], vel=30.0, acc=30.0),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))

    def _play_all(self):
        if not self.waypoints:
            self.set_status("no waypoints to play", T.WARN); return
        wps = list(self.waypoints)
        self.set_status(f"playing {len(wps)} waypoints...", T.LABEL)
        def worker():
            for idx, wp in enumerate(wps):
                self.after(0, lambda i=idx: self.set_status(
                    f"playing WP{i+1}/{len(wps)}", T.LABEL))
                ok = self.robot.movej(wp["posj"], vel=30.0, acc=30.0, sync=0)
                if not ok:
                    self.after(0, lambda: self.set_status("FAILED mid-sequence", T.BAD))
                    return
            self.after(0, lambda: self.set_status("playback complete", T.OK))
        threading.Thread(target=worker, daemon=True).start()

    def _save_file(self):
        try:
            with open(WAYPOINTS_FILE, "w") as f:
                json.dump({"waypoints": self.waypoints}, f, indent=2)
            self.set_status(f"saved to {WAYPOINTS_FILE}", T.OK)
        except Exception as e:
            self.set_status(f"save error: {e}", T.BAD)

    def _load_file(self):
        try:
            with open(WAYPOINTS_FILE) as f:
                data = json.load(f)
            self.waypoints = data.get("waypoints", [])
            self._refresh_list()
            self.set_status(f"loaded {len(self.waypoints)} waypoints", T.OK)
        except Exception as e:
            self.set_status(f"load error: {e}", T.BAD)


# ──────── 5. Speed Control ─────────────────────────────────
class SpeedControlScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "5. Speed Control (deadman)", T.SPEED)

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


# ──────── 6. MoveIt2 ───────────────────────────────────────
class MoveItScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "6. MoveIt2 Planning", T.MOVEIT)

        big = tk.Label(self.body,
                       text="MoveIt2: motion planning with collision avoidance.",
                       bg=T.BG, fg=T.TITLE,
                       font=tkfont.Font(family="DejaVu Sans", size=12, weight="bold"))
        big.pack(anchor="w", pady=(0, 6))

        desc = tk.Label(self.body, bg=T.BG, fg=T.LABEL, justify="left", anchor="w",
                        font=tkfont.Font(family="DejaVu Sans", size=10),
                        text=(
                            "MoveIt2 launches a separate RViz window with motion-planning panels.\n"
                            "Drag the orange interactive marker on the end-effector to set a goal,\n"
                            "then click 'Plan & Execute' in the panel. Path planning, collision\n"
                            "checking, and trajectory execution all happen automatically.\n"
                            "\n"
                            "Note: dsr_moveit2 is part of the upstream doosan-robot2 source but is\n"
                            "not built by default. The button below builds and launches it."))
        desc.pack(fill="x", pady=4)

        self.build_status = tk.Label(self.body, text="checking build status...",
                                     bg=T.BG, fg=T.DIM, anchor="w",
                                     font=tkfont.Font(family="DejaVu Sans", size=10))
        self.build_status.pack(fill="x", pady=(12, 4))

        btns = tk.Frame(self.body, bg=T.BG)
        btns.pack(pady=10)
        big_button(btns, "Build dsr_moveit2", self._build,
                   bg=T.PANEL_HI, fg=T.TITLE).pack(side="left", padx=4)
        big_button(btns, "Launch MoveIt2", self._launch, bg=T.MOVEIT).pack(side="left", padx=4)

        self.log = tk.Text(self.body, bg=T.PANEL, fg=T.VAL, height=12,
                           font=tkfont.Font(family="DejaVu Sans Mono", size=9),
                           relief="flat", insertbackground=T.VAL)
        self.log.pack(fill="both", expand=True, pady=8)

    def on_show(self):
        self._check_build()

    def _check_build(self):
        def fetch():
            r = subprocess.run(
                ["docker", "exec", "doosan_kos", "bash", "-c",
                 "[ -d /ros2_ws/install/dsr_moveit2 ] && echo built || echo missing"],
                capture_output=True, text=True, timeout=5)
            return r.stdout.strip()
        def update(s):
            if s == "built":
                self.build_status.config(text="✓ dsr_moveit2 already built", fg=T.OK)
            else:
                self.build_status.config(text="✗ dsr_moveit2 not built — click 'Build' first",
                                         fg=T.WARN)
        self.run_async(fetch, on_done=update)

    def _append_log(self, s):
        self.log.insert(tk.END, s)
        self.log.see(tk.END)

    def _build(self):
        self._append_log("\n[build] starting (5-15min) ...\n")
        self.set_status("building dsr_moveit2 ...", T.LABEL)

        def worker():
            proc = subprocess.Popen(
                ["docker", "exec", "doosan_kos", "bash", "-lc",
                 "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && "
                 "colcon build --packages-select dsr_moveit2 --cmake-args -DCMAKE_BUILD_TYPE=Release"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                self.after(0, lambda l=line: self._append_log(l))
            rc = proc.wait()
            self.after(0, lambda: self.set_status(
                "build OK" if rc == 0 else "build FAILED",
                T.OK if rc == 0 else T.BAD))
            self.after(0, self._check_build)
        threading.Thread(target=worker, daemon=True).start()

    def _launch(self):
        self._append_log("\n[launch] starting MoveIt2 ...\n")
        self.set_status("launching MoveIt2 ...", T.LABEL)
        # check it's built first
        def fetch():
            r = subprocess.run(
                ["docker", "exec", "doosan_kos", "bash", "-c",
                 "ls /ros2_ws/install/dsr_moveit2/share/dsr_moveit2/launch/ 2>/dev/null"],
                capture_output=True, text=True, timeout=5)
            return r.stdout.strip().splitlines()
        def update(launch_files):
            if not launch_files:
                self._append_log("[launch] dsr_moveit2 not built. Click 'Build' first.\n")
                self.set_status("not built", T.BAD)
                return
            chosen = launch_files[0]
            self._append_log(f"[launch] using {chosen}\n")
            subprocess.Popen(
                ["docker", "exec", "-d", "doosan_kos", "bash", "-lc",
                 f"source /opt/ros/jazzy/setup.bash && "
                 f"source /ros2_ws/install/setup.bash && "
                 f"ros2 launch dsr_moveit2 {chosen} > /tmp/moveit.log 2>&1"])
            self._append_log("[launch] background process started. Check separate RViz window.\n")
            self._append_log("[launch] log: docker exec doosan_kos tail -f /tmp/moveit.log\n")
            self.set_status("launched (separate window)", T.OK)
        self.run_async(fetch, on_done=update)


# ──────── Home (dashboard) ─────────────────────────────────
class HomeScreen(tk.Frame):
    CARDS = [
        ("joint",  "Joint Slider Jog",  T.JOINT,
         "Drag 6 sliders → set joint angles → movej.",
         "Best for getting an exact joint configuration."),
        ("task",   "Task Space Move",   T.TASK,
         "Type X/Y/Z + Rx/Ry/Rz → MoveL or MoveJX.",
         "Most natural for tool-position-based tasks."),
        ("incr",   "Incremental Jog",   T.INCR,
         "+/- buttons with selectable step size.",
         "Quick small adjustments, both joint and TCP."),
        ("wp",     "Waypoint Recorder", T.WP,
         "Save current pose, replay a sequence.",
         "Build teach-and-playback programs."),
        ("speed",  "Speed Control",     T.SPEED,
         "Hold-to-jog (deadman) at given velocity.",
         "Continuous motion, like a teach pendant."),
        ("moveit", "MoveIt2 Planning",  T.MOVEIT,
         "Launch MoveIt2 RViz with planning panel.",
         "Goal-based planning with collision avoidance."),
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
        self.live_lbl = tk.Label(title, text="● connecting", bg=T.BG, fg=T.DIM,
                                 font=tkfont.Font(family="DejaVu Sans", size=11, weight="bold"))
        self.live_lbl.pack(side="right")

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

    def _refresh(self):
        pos = self.app.robot.joint_pos_deg
        self.pose_lbl.config(text="  ".join(f"J{i+1}:{p:+7.1f}°" for i, p in enumerate(pos)))
        # service ready check
        ready = self.app.robot.cli_movej.service_is_ready()
        if ready:
            self.live_lbl.config(text="● connected", fg=T.OK)
        else:
            self.live_lbl.config(text="● waiting for driver", fg=T.WARN)
        self.after(500, self._refresh)


# ──────── App ──────────────────────────────────────────────
class App:
    def __init__(self):
        rclpy.init()
        self.robot = RobotInterface()
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.robot,), daemon=True)
        self.spin_thread.start()
        # ensure autonomous (best-effort)
        threading.Thread(target=self.robot.ensure_autonomous, daemon=True).start()

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
            "incr":   IncrementalJogScreen(self.container, self),
            "wp":     WaypointScreen(self.container, self),
            "speed":  SpeedControlScreen(self.container, self),
            "moveit": MoveItScreen(self.container, self),
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
        # safety: stop any speedj stream
        try:
            self.robot.speedj_stop()
        except Exception:
            pass
        self.root.destroy()

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
