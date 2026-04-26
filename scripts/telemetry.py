#!/usr/bin/env python3
"""
M1013 Telemetry Window — three sections, all live:

  [1] Raw Sensors        : joint position, joint torque
                           (motor current is not exposed by dsr_bringup2)
  [2] DRCF Firmware      : joint velocity, external torque,
                           TCP position / velocity / force-torque
  [3] ROS-Computed       : joint acceleration, TCP acceleration
                           (numerical derivative of velocity, EMA smoothed)

Update rates:
  - joint_states subscription : ~100 Hz (firmware native)
  - service polling           : ~10 Hz (best-effort, async, non-blocking)
  - GUI repaint               : 10 Hz
"""

import math
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import (
    GetExternalTorque,
    GetCurrentPosx,
    GetCurrentVelx,
    GetToolForce,
)

ROBOT_ID = "dsr01"
SVC = f"/{ROBOT_ID}/dsr_controller2/aux_control"

JOINT_NAMES = [f"J{i+1}" for i in range(6)]
TCP_AXES = ["X", "Y", "Z", "Rx", "Ry", "Rz"]

# ── color theme ─────────────────────────────────────────────
BG = "#1e1e2e"        # main background
PANEL = "#252537"     # section background
PANEL_RAISE = "#2c2c40"  # alternate row
TITLE = "#cdd6f4"     # section title
LABEL = "#a6adc8"     # row label
VAL = "#f5f5f5"       # numeric value
DIM = "#6c7086"       # n/a / inactive
ACCENT_RAW = "#f38ba8"      # red-ish for section 1
ACCENT_FW = "#74c7ec"       # cyan for section 2
ACCENT_ROS = "#a6e3a1"      # green for section 3
HIGHLIGHT_MOTION = "#fab387"  # orange when moving
HIGHLIGHT_LOAD = "#f9e2af"    # yellow for high torque
HIGHLIGHT_EXT = "#f38ba8"     # red for external collision

# thresholds for color coding
VEL_HIGHLIGHT_DEG = 1.0   # |°/s| above this → orange
TORQUE_HIGH_NM = 25.0     # |N·m| above → yellow
EXT_TORQUE_NM = 3.0       # |external N·m| above → red


def fmt(v, prec=2, width=8):
    return f"{v:+{width}.{prec}f}"


# ────────────────────────────────────────────────────────────
class TelemetryNode(Node):
    def __init__(self):
        super().__init__("telemetry_node", namespace=ROBOT_ID)
        self.create_subscription(JointState, f"/{ROBOT_ID}/joint_states", self._on_js, 50)
        self.cli_ext = self.create_client(GetExternalTorque, f"{SVC}/get_external_torque")
        self.cli_posx = self.create_client(GetCurrentPosx, f"{SVC}/get_current_posx")
        self.cli_velx = self.create_client(GetCurrentVelx, f"{SVC}/get_current_velx")
        self.cli_force = self.create_client(GetToolForce, f"{SVC}/get_tool_force")

        self.pos_deg = [0.0] * 6
        self.vel_deg = [0.0] * 6
        self.torque = [0.0] * 6
        self.ext_torque = [0.0] * 6
        self.tcp_pos = [0.0] * 6
        self.tcp_vel = [0.0] * 6
        self.tcp_force = [0.0] * 6
        self.acc_deg = [0.0] * 6
        self.tcp_acc = [0.0] * 6

        self._prev_vel = None
        self._prev_vel_t = None
        self._prev_tcp_vel = None
        self._prev_tcp_vel_t = None
        self._f_ext = None
        self._f_posx = None
        self._f_velx = None
        self._f_force = None

    def _on_js(self, msg: JointState):
        if len(msg.position) >= 6:
            self.pos_deg = [math.degrees(p) for p in msg.position[:6]]
        if len(msg.velocity) >= 6:
            new_vel = [math.degrees(v) for v in msg.velocity[:6]]
            now = time.time()
            if self._prev_vel is not None and self._prev_vel_t is not None:
                dt = now - self._prev_vel_t
                if dt > 1e-3:
                    raw = [(v - pv) / dt for v, pv in zip(new_vel, self._prev_vel)]
                    self.acc_deg = [0.2 * r + 0.8 * a for r, a in zip(raw, self.acc_deg)]
            self._prev_vel = new_vel
            self._prev_vel_t = now
            self.vel_deg = new_vel
        if len(msg.effort) >= 6:
            self.torque = list(msg.effort[:6])

    def poll_services(self):
        if self._f_ext is None or self._f_ext.done():
            if self._f_ext and self._f_ext.done():
                r = self._f_ext.result()
                if r and r.success:
                    self.ext_torque = list(r.ext_torque)
            if self.cli_ext.service_is_ready():
                self._f_ext = self.cli_ext.call_async(GetExternalTorque.Request())

        if self._f_posx is None or self._f_posx.done():
            if self._f_posx and self._f_posx.done():
                r = self._f_posx.result()
                if r and r.success and r.task_pos_info:
                    data = list(r.task_pos_info[0].data)
                    if len(data) >= 6:
                        self.tcp_pos = data[:6]
            if self.cli_posx.service_is_ready():
                req = GetCurrentPosx.Request()
                req.ref = 0
                self._f_posx = self.cli_posx.call_async(req)

        if self._f_velx is None or self._f_velx.done():
            if self._f_velx and self._f_velx.done():
                r = self._f_velx.result()
                if r and r.success:
                    new_v = list(r.vel)
                    now = time.time()
                    if self._prev_tcp_vel is not None and self._prev_tcp_vel_t is not None:
                        dt = now - self._prev_tcp_vel_t
                        if dt > 1e-3:
                            raw = [(a - b) / dt for a, b in zip(new_v, self._prev_tcp_vel)]
                            self.tcp_acc = [0.3 * r + 0.7 * a for r, a in zip(raw, self.tcp_acc)]
                    self._prev_tcp_vel = new_v
                    self._prev_tcp_vel_t = now
                    self.tcp_vel = new_v
            if self.cli_velx.service_is_ready():
                req = GetCurrentVelx.Request()
                req.ref = 0
                self._f_velx = self.cli_velx.call_async(req)

        if self._f_force is None or self._f_force.done():
            if self._f_force and self._f_force.done():
                r = self._f_force.result()
                if r and r.success:
                    self.tcp_force = list(r.tool_force)
            if self.cli_force.service_is_ready():
                req = GetToolForce.Request()
                req.ref = 0
                self._f_force = self.cli_force.call_async(req)


# ────────────────────────────────────────────────────────────
class TelemetryGUI:
    def __init__(self, node: TelemetryNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("M1013 Telemetry")
        self.root.configure(bg=BG)

        # fonts
        self.font_label = tkfont.Font(family="DejaVu Sans", size=10)
        self.font_label_bold = tkfont.Font(family="DejaVu Sans", size=10, weight="bold")
        self.font_section = tkfont.Font(family="DejaVu Sans", size=12, weight="bold")
        self.font_value = tkfont.Font(family="DejaVu Sans Mono", size=11)
        self.font_header = tkfont.Font(family="DejaVu Sans Mono", size=10, weight="bold")
        self.font_title = tkfont.Font(family="DejaVu Sans", size=14, weight="bold")
        self.font_status = tkfont.Font(family="DejaVu Sans", size=9)

        # title bar
        title_bar = tk.Frame(self.root, bg=BG)
        title_bar.pack(fill="x", padx=14, pady=(12, 4))
        tk.Label(title_bar, text="M1013 Telemetry", bg=BG, fg=TITLE,
                 font=self.font_title).pack(side="left")
        self.live_lbl = tk.Label(title_bar, text="● LIVE", bg=BG, fg=ACCENT_ROS,
                                 font=self.font_label_bold)
        self.live_lbl.pack(side="right")
        self.clock_lbl = tk.Label(title_bar, text="--:--:--", bg=BG, fg=DIM,
                                  font=self.font_label)
        self.clock_lbl.pack(side="right", padx=(0, 12))

        # sections
        self.r_pos, self.r_torq, self.r_curr = self._build_section_raw()
        self.r_vel, self.r_ext, self.t_pos, self.t_vel, self.t_force = self._build_section_fw()
        self.r_acc, self.t_acc = self._build_section_ros()

        # status bar
        self.status = tk.Label(self.root, text="connecting...", bg=BG, fg=DIM,
                               anchor="w", font=self.font_status)
        self.status.pack(fill="x", padx=14, pady=(0, 10))

        self.root.after(100, self._tick)

    # ── section builders ───────────────────────────────────────
    def _section_frame(self, num, title, accent):
        wrap = tk.Frame(self.root, bg=BG)
        wrap.pack(fill="x", padx=14, pady=4)
        # header strip
        hdr = tk.Frame(wrap, bg=PANEL)
        hdr.pack(fill="x")
        tk.Label(hdr, text=f"  {num}  ", bg=accent, fg=BG,
                 font=self.font_section).pack(side="left", ipady=4)
        tk.Label(hdr, text=f"  {title}", bg=PANEL, fg=TITLE,
                 font=self.font_section).pack(side="left", ipady=4)
        # body
        body = tk.Frame(wrap, bg=PANEL, padx=14, pady=10)
        body.pack(fill="x")
        return body

    def _grid_table(self, parent, col_headers, row_specs, start_row=0):
        """row_specs = [(label, unit, n_cols, row_bg), ...].
        Returns list of cell-label-lists for each row."""
        # column headers
        tk.Label(parent, text="", bg=PANEL, width=14).grid(row=start_row, column=0, padx=2)
        for c, h in enumerate(col_headers):
            tk.Label(parent, text=h, bg=PANEL, fg=LABEL,
                     font=self.font_header, width=10, anchor="e").grid(
                row=start_row, column=c + 1, padx=1, sticky="e")
        tk.Label(parent, text="unit", bg=PANEL, fg=DIM,
                 font=self.font_label, width=8, anchor="w").grid(
            row=start_row, column=len(col_headers) + 1, padx=4, sticky="w")

        cells_list = []
        for i, (lab, unit, ncols, row_bg) in enumerate(row_specs):
            r = start_row + 1 + i
            tk.Label(parent, text=lab, bg=row_bg, fg=LABEL,
                     font=self.font_label_bold, width=14, anchor="w").grid(
                row=r, column=0, sticky="ew", padx=(2, 0), ipady=2)
            cells = []
            for c in range(ncols):
                lbl = tk.Label(parent, text="     ---", bg=row_bg, fg=VAL,
                               font=self.font_value, width=10, anchor="e")
                lbl.grid(row=r, column=c + 1, sticky="ew", padx=1, ipady=2)
                cells.append(lbl)
            tk.Label(parent, text=unit, bg=row_bg, fg=DIM,
                     font=self.font_label, width=8, anchor="w").grid(
                row=r, column=ncols + 1, sticky="ew", padx=4)
            cells_list.append(cells)
        return cells_list

    def _build_section_raw(self):
        body = self._section_frame("1", "Raw Sensors  (directly measured by the robot)", ACCENT_RAW)
        rows = [
            ("Position",     "°",   6, PANEL),
            ("Torque",       "N·m", 6, PANEL_RAISE),
            ("Motor Current","A",   6, PANEL),
        ]
        cells = self._grid_table(body, JOINT_NAMES, rows)
        # mark motor current as N/A
        for c in cells[2]:
            c.config(text="     n/a", fg=DIM)
        # footnote
        tk.Label(body, text="Motor current is not exposed by the dsr_bringup2 driver.",
                 bg=PANEL, fg=DIM, font=self.font_status).grid(
            row=len(rows) + 1, column=0, columnspan=8, sticky="w", pady=(6, 0))
        return cells[0], cells[1], cells[2]

    def _build_section_fw(self):
        body = self._section_frame("2", "DRCF Firmware  (computed inside the controller)", ACCENT_FW)
        # joint rows
        joint_rows = [
            ("Velocity",       "°/s", 6, PANEL),
            ("External Torque","N·m", 6, PANEL_RAISE),
        ]
        joint_cells = self._grid_table(body, JOINT_NAMES, joint_rows, start_row=0)

        # divider
        tk.Frame(body, height=1, bg=PANEL_RAISE).grid(
            row=len(joint_rows) + 1, column=0, columnspan=8, sticky="ew", pady=8)

        # tcp rows
        tcp_rows = [
            ("TCP Position",      "mm,°",       6, PANEL),
            ("TCP Velocity",      "mm/s,°/s",   6, PANEL_RAISE),
            ("TCP Force/Moment",  "N, N·m",     6, PANEL),
        ]
        tcp_cells = self._grid_table(body, TCP_AXES, tcp_rows, start_row=len(joint_rows) + 2)

        return joint_cells[0], joint_cells[1], tcp_cells[0], tcp_cells[1], tcp_cells[2]

    def _build_section_ros(self):
        body = self._section_frame("3", "ROS-Computed  (numerical derivative of velocity, EMA-smoothed)", ACCENT_ROS)
        joint_rows = [("Joint Acceleration", "°/s²", 6, PANEL)]
        joint_cells = self._grid_table(body, JOINT_NAMES, joint_rows)

        tk.Frame(body, height=1, bg=PANEL_RAISE).grid(
            row=2, column=0, columnspan=8, sticky="ew", pady=8)

        tcp_rows = [("TCP Acceleration", "mm/s², °/s²", 6, PANEL_RAISE)]
        tcp_cells = self._grid_table(body, TCP_AXES, tcp_rows, start_row=3)

        return joint_cells[0], tcp_cells[0]

    # ── live update tick ───────────────────────────────────────
    def _tick(self):
        self.node.poll_services()

        # Raw sensors
        for c, v in zip(self.r_pos, self.node.pos_deg):
            c.config(text=fmt(v), fg=VAL)
        for c, v in zip(self.r_torq, self.node.torque):
            color = HIGHLIGHT_LOAD if abs(v) > TORQUE_HIGH_NM else VAL
            c.config(text=fmt(v), fg=color)

        # FW velocities
        for c, v in zip(self.r_vel, self.node.vel_deg):
            color = HIGHLIGHT_MOTION if abs(v) > VEL_HIGHLIGHT_DEG else VAL
            c.config(text=fmt(v), fg=color)
        for c, v in zip(self.r_ext, self.node.ext_torque):
            color = HIGHLIGHT_EXT if abs(v) > EXT_TORQUE_NM else VAL
            c.config(text=fmt(v), fg=color)
        for c, v in zip(self.t_pos, self.node.tcp_pos):
            c.config(text=fmt(v), fg=VAL)
        for c, v in zip(self.t_vel, self.node.tcp_vel):
            color = HIGHLIGHT_MOTION if abs(v) > 0.5 else VAL
            c.config(text=fmt(v), fg=color)
        for c, v in zip(self.t_force, self.node.tcp_force):
            color = HIGHLIGHT_EXT if abs(v) > 0.5 else VAL
            c.config(text=fmt(v, prec=3, width=9), fg=color)

        # ROS-computed
        for c, v in zip(self.r_acc, self.node.acc_deg):
            c.config(text=fmt(v, prec=1), fg=VAL)
        for c, v in zip(self.t_acc, self.node.tcp_acc):
            c.config(text=fmt(v, prec=1), fg=VAL)

        # status
        self.clock_lbl.config(text=time.strftime("%H:%M:%S"))
        ready = all(cli.service_is_ready() for cli in
                    [self.node.cli_ext, self.node.cli_posx,
                     self.node.cli_velx, self.node.cli_force])
        if ready:
            self.live_lbl.config(text="● LIVE", fg=ACCENT_ROS)
            self.status.config(
                text="connected — joint_states 100Hz, services ~10Hz, GUI 10Hz", fg=DIM)
        else:
            self.live_lbl.config(text="○ WAIT", fg=HIGHLIGHT_LOAD)
            self.status.config(text="waiting for driver services...", fg=HIGHLIGHT_LOAD)

        self.root.after(100, self._tick)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = TelemetryNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    gui = TelemetryGUI(node)
    try:
        gui.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
