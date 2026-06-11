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
import socket
import subprocess
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
import tkinter.filedialog as filedialog
import tkinter.messagebox as messagebox
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Empty
from dsr_msgs2.msg import SpeedjStream
from dsr_msgs2.srv import (
    MoveJoint,
    MoveJointx,
    MoveLine,
    MoveStop,
    Fkin,
    GetCurrentPosj,
    GetCurrentPosx,
    GetRobotState,
    SetRobotMode,
    SetSafetyMode,
    SetSingularityHandling,
    ChangeCollisionSensitivity,
)

# DRCF robot_state enum (from doosan-robot2 src). Used to distinguish
# "ROS service is connected" from "DRCF actually accepts motion commands".
ROBOT_STATE_NAMES = {
    0: "INITIALIZING", 1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF",
    4: "TEACHING", 5: "SAFE_STOP", 6: "EMERGENCY_STOP", 7: "HOMING",
    8: "RECOVERY", 9: "SAFE_STOP2", 10: "SAFE_OFF2",
}
# Codes where the robot will actually execute motion commands.
ROBOT_STATE_READY = {1, 2, 7}  # STANDBY, MOVING, HOMING

# ──────── Workspace safety limits ──────────────────────────
TCP_Z_MIN_MM = 28.9          # finger-tip floor set to the user's taught position (TCP Z=187mm @ real posx 2026-06-04); tip can't go below this
COLLISION_SENSITIVITY = 50   # 0-100 ; controller-side reactive stop threshold
SINGULARITY_MODE = 0         # 0=AVOID, 1=TASK_STOP, 2=VAR_VEL

# Gripper finger-tip in tool0 frame (mm). Derived from URDF + bridge mount:
#   bridge attaches the gripper at tool0 with translation (0, +12mm, 0) and Rx(-90°);
#   URDF places the finger-tip at gripper-base + (0, -158.1mm, 0).
#   Composing: tip_in_tool0 = (0, +12, 0) + Rx(-90°)·(0, -158.1, 0) = (0, +12, +158.1).
GRIPPER_TIP_TOOL0_MM = (0.0, 12.0, 158.1)

ROBOT_ID = "dsr01"
SVC = f"/{ROBOT_ID}/dsr_controller2"

# Real-robot deployment defaults
REAL_ROBOT_ID = "dsr01_real"
DEFAULT_REAL_IP = "192.168.137.100"
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
    CAM         = "#89dceb"  # sky blue — distance estimator


# ──────── M1013 joint limits (deg) — datasheet-matched practical bounds ──────────
# Empirically verified via fkin: with all other joints at 0, J2 outside ±90°
# drives the TCP below the floor (e.g. J2=120° → TCP Z ≈ -500mm). Datasheet
# says ±95°. Using ±95° as the slider/clamp limit. fkin TCP-Z check below
# catches compound poses where the wrist would still dive below the floor.
# (Elbow / upper-arm collision in some compound poses is not caught here —
#  use MoveIt2 mode 6 for full geometric collision avoidance.)
JOINT_LIMITS = [
    (-360, 360),  # J1  — base rotation, no floor risk
    (-95,   95),  # J2  — datasheet ±95° matches the empirical floor-safe range
    (-160, 160),  # J3  — URDF/controller limit (datasheet is tighter at ±125°)
    (-360, 360),  # J4  — wrist roll
    (-135, 135),  # J5  — datasheet, prevents wrist over-rotation
    (-360, 360),  # J6
]

JOINT_NAMES = [f"J{i+1}" for i in range(6)]
TCP_AXES = ["X", "Y", "Z", "Rx", "Ry", "Rz"]

# Real-robot slider-jog default pose (mirrors HOME_POSE in m1013_gripper_bridge.py).
HOME_POSE_DEG = [0.0, 0.0, -90.0, 0.0, -90.0, 0.0]


# ──────── Robot Interface ──────────────────────────────────
def _zyz_to_R(deg_a, deg_b, deg_c):
    """Doosan posx orientation (A, B, C) is intrinsic ZYZ Euler in degrees.
    Returns the 3x3 rotation matrix R = Rz(A) · Ry(B) · Rz(C)."""
    a, b, c = (math.radians(d) for d in (deg_a, deg_b, deg_c))
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    return (
        ( ca*cb*cc - sa*sc, -ca*cb*sc - sa*cc,  ca*sb),
        ( sa*cb*cc + ca*sc, -sa*cb*sc + ca*cc,  sa*sb),
        (            -sb*cc,             sb*sc,     cb),
    )


def gripper_tip_world(posx):
    """World coords (mm) of the gripper finger-tip from a TCP posx
    (XYZ in mm, ABC as ZYZ-Euler degrees, expressed in BASE/WORLD ref)."""
    R = _zyz_to_R(posx[3], posx[4], posx[5])
    tx, ty, tz = GRIPPER_TIP_TOOL0_MM
    return (
        posx[0] + R[0][0]*tx + R[0][1]*ty + R[0][2]*tz,
        posx[1] + R[1][0]*tx + R[1][1]*ty + R[1][2]*tz,
        posx[2] + R[2][0]*tx + R[2][1]*ty + R[2][2]*tz,
    )


# Fetch motion direction (camera viewing axis) expressed in the TCP/tool0 frame.
# Calibrated (2026-06-09) at the operator's working pose: with the gripper
# pointing down, tool +Z → world straight-down and tool +Y → world horizontal,
# so CAMERA_AXIS_TOOL0 = (0, +1/√2, +1/√2) yields world ≈ (forward + 45° down),
# moving the camera toward what it sees. (The earlier (0,-1/√2,+1/√2) drove it
# back+down; flipping only Y — keeping Z positive — turns "back" into "forward"
# while staying DOWN. Negating Z too would point it up.)
# NOTE: this is a tool-frame vector rotated by R(posx); if the gripper is
# re-oriented a lot the world direction changes. Re-calibrate if needed:
# CAMERA_AXIS_TOOL0 = Rᵀ(posx) · desired_world.
_CAM_S = 1.0 / math.sqrt(2.0)
CAMERA_AXIS_TOOL0 = (0.0, _CAM_S, _CAM_S)


def camera_axis_world(posx):
    """Unit vector (world frame) of the camera viewing direction for a TCP posx.
    Rotates CAMERA_AXIS_TOOL0 by the TCP orientation (ZYZ Euler ABC)."""
    R = _zyz_to_R(posx[3], posx[4], posx[5])
    vx, vy, vz = CAMERA_AXIS_TOOL0
    wx = R[0][0]*vx + R[0][1]*vy + R[0][2]*vz
    wy = R[1][0]*vx + R[1][1]*vy + R[1][2]*vz
    wz = R[2][0]*vx + R[2][1]*vy + R[2][2]*vz
    n = (wx*wx + wy*wy + wz*wz) ** 0.5 or 1.0
    return (wx / n, wy / n, wz / n)


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
        self.cli_stop = self.create_client(MoveStop, f"{SVC}/motion/move_stop")
        self.cli_sing = self.create_client(SetSingularityHandling, f"{SVC}/motion/set_singularity_handling")
        self.cli_coll = self.create_client(ChangeCollisionSensitivity, f"{SVC}/system/change_collision_sensitivity")
        self.cli_fkin = self.create_client(Fkin, f"{SVC}/motion/fkin")
        self.cli_safety = self.create_client(SetSafetyMode, f"{SVC}/system/set_safety_mode")

        # safety state (for status display)
        self.singularity_mode = None
        self.collision_sensitivity = None

        # target mode: "sim" | "real" | "preview"
        # preview = run on sim first, ask user confirm, then run on real.
        self.target_mode = "sim"
        self.real_ip = DEFAULT_REAL_IP
        self.real_driver_started = False
        # Re-entry lock for start_real_driver: rapid double-clicks otherwise
        # race past the idempotence guard and spawn duplicate launches that
        # fight over DRCF's TCP port, leaving zombie ros2_control_nodes.
        self._real_starting = False
        # real-robot service clients (lazily created when real driver is started)
        self._real_clients = {}
        # DRCF robot_state code of the real robot, cached from the last
        # get_robot_state query. None = not yet queried / driver not up.
        self._real_robot_state_code = None
        self._real_robot_state_name = None
        # Auto-detect a real driver launched outside this process
        # (e.g. by an external script). Cancels itself once detected.
        self._real_detect_timer = self.create_timer(2.0, self._poll_real_driver)

        # speedj publisher
        self.pub_speedj = self.create_publisher(SpeedjStream, f"{SVC}/speedj_stream", 10)
        # Gripper: Float32 (m, 0=closed, 0.067=fully open). 시뮬 브릿지 + 실 Dynamixel 노드 공통.
        self.pub_gripper = self.create_publisher(Float32, "/gripper_command", 10)
        # Recover trigger — Empty to the real Dynamixel node to reboot out of
        # an overload + re-home (see gripper_node_v2.py).
        self.pub_gripper_recover = self.create_publisher(Empty, "/gripper_recover", 10)
        self._speedj_running = False
        self._speedj_thread = None
        self._speedj_target = [0.0] * 6
        self._speedj_acc = 30.0

    def gripper_set(self, opening_m: float):
        msg = Float32()
        msg.data = float(max(0.0, min(0.067, opening_m)))
        self.pub_gripper.publish(msg)

    def gripper_recover(self):
        """Tell the real gripper node to reboot the motor out of an overload
        and re-home/recalibrate."""
        self.pub_gripper_recover.publish(Empty())

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

    def configure_safety(self, sing_mode=SINGULARITY_MODE,
                         coll_sens=COLLISION_SENSITIVITY) -> bool:
        """Enable singularity avoidance + reactive collision stop."""
        ok = True
        if self.cli_sing.wait_for_service(timeout_sec=2.0):
            req = SetSingularityHandling.Request()
            req.mode = int(sing_mode)
            r = self._wait(self.cli_sing.call_async(req), 2.0)
            if r and r.success:
                self.singularity_mode = sing_mode
            else:
                ok = False
        if self.cli_coll.wait_for_service(timeout_sec=2.0):
            req = ChangeCollisionSensitivity.Request()
            req.sensitivity = int(coll_sens)
            r = self._wait(self.cli_coll.call_async(req), 2.0)
            if r and r.success:
                self.collision_sensitivity = coll_sens
            else:
                ok = False
        return ok

    @staticmethod
    def _clamp_joint_targets(pos_deg):
        """Clamp each joint to its URDF/controller limit."""
        out = []
        clamped = False
        for p, (lo, hi) in zip(pos_deg, JOINT_LIMITS):
            cp = max(lo, min(hi, p))
            if cp != p:
                clamped = True
            out.append(cp)
        return out, clamped

    def _send_movej_one(self, pos_deg, vel, acc, sync, real):
        cli = self._client_for("movej", real=real)
        if not cli or not cli.wait_for_service(timeout_sec=2.0):
            return False
        req = MoveJoint.Request()
        req.pos = [float(p) for p in pos_deg]
        req.vel = float(vel); req.acc = float(acc)
        req.time = 0.0; req.radius = 0.0
        req.mode = 0; req.blend_type = 0; req.sync_type = int(sync)
        r = self._wait(cli.call_async(req), 120.0)
        return bool(r and r.success)

    # ── service-backed motion ──
    def movej(self, pos_deg, vel=30.0, acc=30.0, sync=0,
              confirm_real_callback=None) -> bool:
        clamped, was_clamped = self._clamp_joint_targets(pos_deg)
        if was_clamped:
            self.get_logger().warn(f"movej target clamped to limits: {pos_deg} → {clamped}")
        # workspace check via forward kinematics
        ok, why = self._fkin_check(clamped, ref=0)
        if not ok:
            self.get_logger().warn(f"movej blocked: predicted {why}")
            return False

        mode = self.target_mode
        if mode == "sim":
            return self._send_movej_one(clamped, vel, acc, sync, real=False)
        if mode == "real":
            if not self.real_driver_started:
                self.get_logger().warn("real mode requested but real driver not started")
                return False
            return self._send_movej_one(clamped, vel, acc, sync, real=True)
        # preview: sim → confirm → real
        ok_sim = self._send_movej_one(clamped, vel, acc, sync=0, real=False)
        if not ok_sim:
            return False
        if not self.real_driver_started:
            self.get_logger().warn("preview mode: sim done but real driver not started; skipping real")
            return ok_sim
        proceed = True
        if confirm_real_callback:
            proceed = bool(confirm_real_callback(clamped, kind="movej"))
        if not proceed:
            return ok_sim
        return self._send_movej_one(clamped, vel, acc, sync, real=True)

    @staticmethod
    def _check_workspace(posx, ref):
        """Reject targets where the gripper finger-tip would dip below the
        floor margin. Operates on the tip in world coords (computed from TCP
        pose + tool0→tip offset), not on the bare TCP, because the gripper
        sticks out ~158mm past the flange."""
        if ref not in (0, 2):  # 0=BASE, 2=WORLD; tool/relative refs skip the check
            return True, ""
        tip_x, tip_y, tip_z = gripper_tip_world(posx)
        if tip_z < TCP_Z_MIN_MM:
            return False, (
                f"gripper tip Z={tip_z:.1f}mm < floor {TCP_Z_MIN_MM:.0f}mm "
                f"(TCP Z={posx[2]:.1f}mm)"
            )
        return True, ""

    def _fkin_check(self, posj_deg, ref=0):
        """Predict TCP via Doosan fkin service then run workspace check.
        Catches the case where movej target joints would put the TCP /
        wrist below the floor."""
        if not self.cli_fkin.wait_for_service(timeout_sec=0.5):
            return True, ""  # can't check, allow (don't block on missing service)
        req = Fkin.Request()
        req.pos = [float(p) for p in posj_deg]
        req.ref = int(ref)
        r = self._wait(self.cli_fkin.call_async(req), 1.5)
        if r is None or not r.success:
            return True, ""  # fkin failed, allow
        posx = list(r.conv_posx)
        return self._check_workspace(posx, ref)

    def _send_movejx_one(self, posx, vel, acc, ref, sol, sync, real):
        cli = self._client_for("movejx", real=real)
        if not cli or not cli.wait_for_service(timeout_sec=2.0):
            return False
        req = MoveJointx.Request()
        req.pos = [float(p) for p in posx]
        req.vel = float(vel); req.acc = float(acc)
        req.time = 0.0; req.radius = 0.0
        req.ref = int(ref); req.mode = 0
        req.blend_type = 0; req.sol = int(sol); req.sync_type = int(sync)
        r = self._wait(cli.call_async(req), 120.0)
        return bool(r and r.success)

    def movejx(self, posx, vel=30.0, acc=30.0, ref=0, sol=2, sync=0,
               confirm_real_callback=None) -> bool:
        ok, why = self._check_workspace(posx, ref)
        if not ok:
            self.get_logger().warn(f"movejx blocked: {why}")
            return False
        mode = self.target_mode
        if mode == "sim":
            return self._send_movejx_one(posx, vel, acc, ref, sol, sync, real=False)
        if mode == "real":
            if not self.real_driver_started:
                self.get_logger().warn("real mode requested but real driver not started")
                return False
            return self._send_movejx_one(posx, vel, acc, ref, sol, sync, real=True)
        # preview: sim → confirm → real
        ok_sim = self._send_movejx_one(posx, vel, acc, ref, sol, sync=0, real=False)
        if not ok_sim:
            return False
        if not self.real_driver_started:
            self.get_logger().warn("preview mode: sim done but real driver not started; skipping real")
            return ok_sim
        proceed = True
        if confirm_real_callback:
            proceed = bool(confirm_real_callback(posx, kind="movejx"))
        if not proceed:
            return ok_sim
        return self._send_movejx_one(posx, vel, acc, ref, sol, sync, real=True)

    def _send_movel_one(self, posx, vel_lin, vel_ang, acc_lin, acc_ang, ref, sync, real):
        cli = self._client_for("movel", real=real)
        if not cli or not cli.wait_for_service(timeout_sec=2.0):
            return False
        req = MoveLine.Request()
        req.pos = [float(p) for p in posx]
        req.vel = [float(vel_lin), float(vel_ang)]
        req.acc = [float(acc_lin), float(acc_ang)]
        req.time = 0.0; req.radius = 0.0
        req.ref = int(ref); req.mode = 0
        req.blend_type = 0; req.sync_type = int(sync)
        r = self._wait(cli.call_async(req), 120.0)
        return bool(r and r.success)

    def movel(self, posx, vel_lin=100.0, vel_ang=30.0, acc_lin=200.0, acc_ang=60.0,
              ref=0, sync=0, confirm_real_callback=None) -> bool:
        ok, why = self._check_workspace(posx, ref)
        if not ok:
            self.get_logger().warn(f"movel blocked: {why}")
            return False
        mode = self.target_mode
        if mode == "sim":
            return self._send_movel_one(posx, vel_lin, vel_ang, acc_lin, acc_ang, ref, sync, real=False)
        if mode == "real":
            if not self.real_driver_started:
                self.get_logger().warn("real mode requested but real driver not started")
                return False
            return self._send_movel_one(posx, vel_lin, vel_ang, acc_lin, acc_ang, ref, sync, real=True)
        # preview: sim → confirm → real
        ok_sim = self._send_movel_one(posx, vel_lin, vel_ang, acc_lin, acc_ang, ref, sync=0, real=False)
        if not ok_sim:
            return False
        if not self.real_driver_started:
            self.get_logger().warn("preview mode: sim done but real driver not started; skipping real")
            return ok_sim
        proceed = True
        if confirm_real_callback:
            proceed = bool(confirm_real_callback(posx, kind="movel"))
        if not proceed:
            return ok_sim
        return self._send_movel_one(posx, vel_lin, vel_ang, acc_lin, acc_ang, ref, sync, real=True)

    def _query_client(self, kind):
        """Pick sim or real query client based on target_mode.
        Real for 'real' / 'preview' modes (when real driver is up), sim otherwise."""
        use_real = (self.target_mode in ("real", "preview")
                    and self.real_driver_started)
        if use_real:
            cli = self._real_clients.get(kind)
            if cli is not None:
                return cli
        return {"posj": self.cli_posj, "posx": self.cli_posx}[kind]

    def get_current_posj(self):
        cli = self._query_client("posj")
        if not cli.wait_for_service(timeout_sec=1.0):
            return None
        r = self._wait(cli.call_async(GetCurrentPosj.Request()), 2.0)
        if r and r.success:
            return list(r.pos)
        return None

    def get_current_posx(self, ref=0):
        cli = self._query_client("posx")
        if not cli.wait_for_service(timeout_sec=1.0):
            return None
        req = GetCurrentPosx.Request(); req.ref = int(ref)
        r = self._wait(cli.call_async(req), 2.0)
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

        def publish_loop():
            margin = 5.0  # stop this many degrees before joint limit
            while self._speedj_running:
                safe = list(self._speedj_target)
                for i, v in enumerate(safe):
                    if abs(v) < 0.01:
                        continue
                    lo, hi = JOINT_LIMITS[i]
                    cur = self.joint_pos_deg[i]
                    if (v > 0 and cur >= hi - margin) or (v < 0 and cur <= lo + margin):
                        safe[i] = 0.0
                msg = SpeedjStream()
                msg.vel = safe
                msg.acc = [self._speedj_acc] * 6
                msg.time = 0.05
                self.pub_speedj.publish(msg)
                time.sleep(0.05)

        # Floor watchdog: live jog bypasses pre-flight workspace checks, so
        # poll fkin while streaming and zero-out the moment the gripper tip
        # would cross below the floor margin. Runs in its own thread because
        # the fkin service call could block the publish loop otherwise.
        def floor_watchdog():
            while self._speedj_running:
                time.sleep(0.3)
                if not self._speedj_running:
                    return
                try:
                    ok, why = self._fkin_check(self.joint_pos_deg, ref=0)
                except Exception:
                    continue
                if not ok and self._speedj_running:
                    self.get_logger().warn(f"speedj auto-stop: {why}")
                    self._speedj_running = False
                    stop = SpeedjStream()
                    stop.vel = [0.0] * 6
                    stop.acc = [self._speedj_acc] * 6
                    stop.time = 0.0
                    for _ in range(3):
                        self.pub_speedj.publish(stop)
                        time.sleep(0.02)
                    return

        self._speedj_thread = threading.Thread(target=publish_loop, daemon=True)
        self._speedj_thread.start()
        threading.Thread(target=floor_watchdog, daemon=True).start()

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

    # ── real-robot driver management ──
    def start_real_driver(self, robot_ip: str) -> bool:
        """Launch a second dsr_bringup2 driver pointing at the real robot,
        in namespace /dsr01_real. Returns True only when the driver actually
        connects to DRCF — service-name registration alone is not enough,
        because ros2_control_node registers services even when the host is
        unreachable, which would mis-report a fake 'connected' state."""
        if self._real_starting:
            self.get_logger().info(
                "start_real_driver: already in progress, ignoring duplicate request")
            return False
        self._real_starting = True
        try:
            return self._start_real_driver_impl(robot_ip)
        finally:
            self._real_starting = False

    def _start_real_driver_impl(self, robot_ip: str) -> bool:
        self.real_ip = robot_ip

        # Always force a fresh boot. Previously this attached to an existing
        # driver if one was already up, but that left users stuck when DRCF
        # had drifted to SAFE_OFF after a demo tripped a soft fault — the
        # driver only auto-retries CONTROL_SERVO_ON during its init phase.
        # By killing first, we guarantee that "Start Real Driver" always
        # results in STATE_STANDBY (servo on) or a clear failure.
        # Unconditionally tear down ANY existing /dsr01_real driver before
        # launching. This must NOT be gated on service discovery: when the DDS
        # layer is polluted the services become undiscoverable, and skipping the
        # kill in that state is exactly what let stale drivers pile up (28+ over
        # days) and corrupt discovery → "servo state unknown".
        #
        # Critical: match the *child* namespace `__ns:=/dsr01_real`. The worker
        # nodes (ros2_control_node, robot_state_publisher, controller spawners)
        # carry that token; `name:=dsr01_real` only matches the `ros2 launch`
        # PARENT. The old pattern killed only the parent and orphaned every
        # worker — the root cause of the pileup.
        self.get_logger().info(
            "start_real_driver: tearing down any existing real driver (fresh boot)")
        kill_patterns = (f"__ns:=/{REAL_ROBOT_ID}", f"name:={REAL_ROBOT_ID}")
        for sig in ([], ["-9"]):
            for pat in kill_patterns:
                try:
                    subprocess.run(["pkill", *sig, "-f", pat],
                                   check=False, timeout=3)
                except Exception as e:
                    self.get_logger().warn(f"pkill {sig} {pat} failed: {e}")
            time.sleep(3 if not sig else 2)
        # Clear orphaned FastDDS shared-memory segments left behind by the
        # SIGKILLs so they don't break discovery for the fresh driver.
        # `fastdds shm clean` only removes segments with no live owner, so it is
        # safe for the still-running sim driver / console / Isaac bridge.
        try:
            subprocess.run(
                ["bash", "-lc",
                 "source /opt/ros/jazzy/setup.bash && fastdds shm clean"],
                check=False, timeout=10)
        except Exception as e:
            self.get_logger().warn(f"fastdds shm clean failed: {e}")
        # Stale ROS clients pointed at the now-dead node; clear so they get
        # recreated against the new instance.
        self.real_driver_started = False
        self._real_clients = {}
        self._real_robot_state_code = None
        self._real_robot_state_name = None
        self._real_proc = None

        log_path = "/tmp/real_driver.log"
        try:
            open(log_path, "w").close()
        except Exception:
            pass
        cmd = [
            "bash", "-lc",
            "source /opt/ros/jazzy/setup.bash && "
            "source /ros2_ws/install/setup.bash && "
            f"exec ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py "
            f"name:={REAL_ROBOT_ID} model:=m1013 mode:=real "
            f"host:={robot_ip} port:=12345 gui:=false",
        ]
        try:
            logf = open(log_path, "ab")
            self._real_proc = subprocess.Popen(
                cmd, stdout=logf, stderr=subprocess.STDOUT)
        except Exception as e:
            self.get_logger().error(f"failed to launch real driver: {e}")
            return False

        # We wait for STATE_STANDBY, not just "Connected to DRCF".
        # "Connected" fires before the driver has tried CONTROL_SERVO_ON;
        # STATE_STANDBY is the line that prints once servo_on succeeded,
        # which is the actual readiness signal the user can hear (motor
        # torque click) and observe via robot_state=1.
        success_marker = "STATE_STANDBY"
        fail_keywords = (
            "Connection refused", "connect failed", "cannot connect",
            "No route to host", "Network is unreachable", "Traceback",
        )
        deadline = time.time() + 30.0
        while time.time() < deadline:
            try:
                with open(log_path, "r") as f:
                    log = f.read()
            except Exception:
                log = ""
            if success_marker in log:
                self.real_driver_started = True
                self._lazy_create_real_clients()
                return True
            if any(k in log for k in fail_keywords):
                last = log.strip().splitlines()[-1] if log.strip() else "no log"
                self.get_logger().warn(
                    f"real driver: connection failure — {last}")
                self._kill_real_driver()
                return False
            if self._real_proc.poll() is not None:
                self.get_logger().warn(
                    f"real driver: process exited (rc={self._real_proc.returncode})")
                return False
            time.sleep(1)

        self.get_logger().warn(
            f"real driver: 30s timeout — no DRCF connection at {robot_ip}:12345")
        self._kill_real_driver()
        return False

    def _kill_real_driver(self):
        proc = getattr(self, "_real_proc", None)
        if proc and proc.poll() is None:
            try:
                proc.terminate()
                try:
                    proc.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2)
            except Exception:
                pass
        # Kill the worker nodes (__ns:=/dsr01_real) AND the launch parent
        # (name:=dsr01_real). Matching only name:= orphans every worker.
        for sig in ([], ["-9"]):
            for pat in (f"__ns:=/{REAL_ROBOT_ID}", f"name:={REAL_ROBOT_ID}"):
                try:
                    subprocess.run(["pkill", *sig, "-f", pat],
                                   check=False, timeout=3)
                except Exception:
                    pass
            time.sleep(2 if not sig else 1)
        self.real_driver_started = False
        self._real_clients = {}

    @staticmethod
    def _robot_reachable(ip: str, port: int, timeout: float = 1.0) -> bool:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        try:
            s.connect((ip, port))
            return True
        except Exception:
            return False
        finally:
            try:
                s.close()
            except Exception:
                pass

    def _lazy_create_real_clients(self):
        """Create service clients for the real-robot namespace."""
        if self._real_clients:
            return
        real_svc = f"/{REAL_ROBOT_ID}/dsr_controller2"
        self._real_clients = {
            "movej": self.create_client(MoveJoint, f"{real_svc}/motion/move_joint"),
            "movejx": self.create_client(MoveJointx, f"{real_svc}/motion/move_jointx"),
            "movel": self.create_client(MoveLine, f"{real_svc}/motion/move_line"),
            "stop": self.create_client(MoveStop, f"{real_svc}/motion/move_stop"),
            "posj": self.create_client(GetCurrentPosj, f"{real_svc}/aux_control/get_current_posj"),
            "posx": self.create_client(GetCurrentPosx, f"{real_svc}/aux_control/get_current_posx"),
            "get_state": self.create_client(GetRobotState, f"{real_svc}/system/get_robot_state"),
            "safety": self.create_client(SetSafetyMode, f"{real_svc}/system/set_safety_mode"),
            "mode": self.create_client(SetRobotMode, f"{real_svc}/system/set_robot_mode"),
        }

    def query_real_robot_state(self, timeout: float = 1.5):
        """Query DRCF's current robot_state via the real driver. Used to
        distinguish "ROS connected" from "robot actually ready to move":
        a driver that's up but whose DRCF dropped to SAFE_OFF/EMERGENCY_STOP
        still has a live service, so service-existence alone is misleading.

        Returns (code, name). Both None if the query times out or fails.
        Side-effect: caches the result on the node for later UI access."""
        if not self.real_driver_started:
            self._real_robot_state_code = None
            self._real_robot_state_name = None
            return None, None
        cli = self._real_clients.get("get_state")
        if cli is None or not cli.wait_for_service(timeout_sec=0.5):
            return None, None
        resp = self._wait(cli.call_async(GetRobotState.Request()), timeout)
        if resp is None or not getattr(resp, "success", False):
            return None, None
        code = resp.robot_state
        name = ROBOT_STATE_NAMES.get(code, f"UNKNOWN({code})")
        self._real_robot_state_code = code
        self._real_robot_state_name = name
        return code, name

    def _poll_real_driver(self):
        """Detect a real driver started outside this process and attach to it.
        We only check ROS service registration here — DRCF's TCP socket is
        single-client and refuses a second probe while the driver holds it,
        so a TCP test from this node falsely reads as 'unreachable'."""
        if self.real_driver_started:
            self._real_detect_timer.cancel()
            return
        target = f"/{REAL_ROBOT_ID}/dsr_controller2/motion/move_joint"
        if not any(name == target for name, _ in self.get_service_names_and_types()):
            return
        self.real_driver_started = True
        self._lazy_create_real_clients()
        self.get_logger().info(f"Detected external real driver at {target}")
        self._real_detect_timer.cancel()

    def _client_for(self, kind: str, real: bool):
        """Pick sim or real client for the given motion kind."""
        if real and self.real_driver_started:
            return self._real_clients.get(kind)
        return {
            "movej": self.cli_movej,
            "movejx": self.cli_movejx,
            "movel": self.cli_movel,
            "stop": self.cli_stop,
        }[kind]

    def cancel_motion(self, stop_mode=3):
        """Cancel any in-progress motion. stop_mode 3 = HOLD (graceful).

        Safety-critical: a motion started with sync=0 keeps running until a
        move_stop reaches the SAME driver that is executing it. We therefore
        fire move_stop at BOTH the sim and (if up) the real driver, so release
        always halts the robot no matter which mode launched the move."""
        # 1. stop speedj stream if any — never let a publisher error here block
        #    the move_stop service calls below (those actually halt movel).
        try:
            self.speedj_stop()
        except Exception as e:
            self.get_logger().warn(f"speedj_stop during cancel failed (ignored): {e}")

        # 2. send move_stop to every available driver
        clients = [self.cli_stop]
        if self.real_driver_started:
            real_stop = self._real_clients.get("stop")
            if real_stop is not None:
                clients.append(real_stop)

        stopped = False
        for cli in clients:
            try:
                if cli.wait_for_service(timeout_sec=0.5):
                    req = MoveStop.Request()
                    req.stop_mode = int(stop_mode)
                    cli.call_async(req)   # fire-and-forget
                    stopped = True
            except Exception as e:
                self.get_logger().warn(f"move_stop failed on a client (continuing): {e}")
        return stopped

    def recover(self) -> bool:
        """Try to bring the robot out of a faulted/stuck state.

        Sequence:
          1. cancel any in-flight motion (move_stop HOLD)
          2. cycle safety mode RECOVERY → AUTONOMOUS+MOVE (clears soft faults)
          3. ensure robot mode = AUTONOMOUS
        """
        self.cancel_motion(stop_mode=3)
        time.sleep(0.3)
        if self.cli_safety.wait_for_service(timeout_sec=1.0):
            for safety_mode, safety_event in [(2, 1), (1, 1)]:
                req = SetSafetyMode.Request()
                req.safety_mode = int(safety_mode)
                req.safety_event = int(safety_event)
                self._wait(self.cli_safety.call_async(req), 2.0)
                time.sleep(0.2)
        return self.ensure_autonomous()

    def recover_real(self) -> bool:
        """Same as recover() but targets the real-robot namespace via
        self._real_clients. Used after start_real_driver if DRCF is sitting
        in SAFE_OFF (e.g. a prior demo tripped collision/limit and the
        driver doesn't auto-retry servo_on outside of init)."""
        if not self.real_driver_started:
            return False
        cli_safety = self._real_clients.get("safety")
        cli_mode = self._real_clients.get("mode")
        if cli_safety is None or cli_mode is None:
            return False
        if cli_safety.wait_for_service(timeout_sec=1.0):
            for safety_mode, safety_event in [(2, 1), (1, 1)]:
                req = SetSafetyMode.Request()
                req.safety_mode = int(safety_mode)
                req.safety_event = int(safety_event)
                self._wait(cli_safety.call_async(req), 2.0)
                time.sleep(0.2)
        if not cli_mode.wait_for_service(timeout_sec=2.0):
            return False
        req = SetRobotMode.Request()
        req.robot_mode = 1  # AUTONOMOUS
        r = self._wait(cli_mode.call_async(req), 3.0)
        return bool(r and r.success)


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
        # Gripper footer (모든 모드 공통)
        self.gripper_panel = GripperPanel(self, app)
        self.gripper_panel.pack(side="bottom", fill="x", padx=14, pady=(0, 8))
        self.body = tk.Frame(self, bg=T.BG)
        self.body.pack(fill="both", expand=True, padx=14, pady=(0, 14))

    def _build_header(self):
        hdr = tk.Frame(self, bg=T.BG)
        hdr.pack(fill="x", padx=14, pady=(14, 6))

        back = big_button(hdr, "← Home", self.app.show_home,
                          bg=T.PANEL_HI, fg=T.TITLE, height=1, width=10)
        back.pack(side="left")

        # Robot Home Pose button — every mode gets the same shortcut so the
        # operator can always recover to a safe known pose (HOME_POSE_DEG)
        # via movej. Respects target_mode (sim / real / preview).
        home_pose = big_button(hdr, "🏠 Home Pose", self._go_home_pose,
                               bg=T.WARN, height=1, width=12)
        home_pose.pack(side="left", padx=(8, 0))

        # accent bar + title
        bar = tk.Frame(hdr, bg=self.accent, width=4)
        bar.pack(side="left", fill="y", padx=(20, 8))
        tk.Label(hdr, text=self.title, bg=T.BG, fg=T.TITLE,
                 font=tkfont.Font(family="DejaVu Sans", size=15, weight="bold")
                 ).pack(side="left")

        self.status_lbl = tk.Label(hdr, text="ready", bg=T.BG, fg=T.DIM,
                                   font=tkfont.Font(family="DejaVu Sans", size=9),
                                   cursor="hand2")
        self.status_lbl.pack(side="right")
        self.status_lbl.bind("<Button-1>", lambda e: self._on_status_click())

    def _go_home_pose(self):
        """Send robot to HOME_POSE_DEG via movej. Respects target_mode —
        in 'preview' the confirm dialog is shown before touching the real arm."""
        self.set_status("Home Pose → movej " + str(HOME_POSE_DEG) + "°", T.LABEL)
        self.run_async(
            lambda: self.robot.movej(
                HOME_POSE_DEG, vel=30.0, acc=30.0,
                confirm_real_callback=self.app.confirm_real_modal),
            on_done=lambda ok: self.set_status("OK" if ok else "FAILED",
                                               T.OK if ok else T.BAD))

    def set_status(self, msg, color=T.DIM):
        self.status_lbl.config(text=msg, fg=color)
        # show "click to recover" hint when in failed state
        if color == T.BAD:
            self.status_lbl.config(text=f"⟳  {msg}  (click to recover)")

    def _on_status_click(self):
        """If the status is in a failed state, run recover()."""
        cur_text = self.status_lbl.cget("text")
        if "click to recover" not in cur_text:
            return  # only react when failed
        self.set_status("recovering...", T.WARN)
        def work():
            return self.robot.recover()
        def done(ok):
            self.set_status("recovered — try again" if ok else "recover FAILED",
                            T.OK if ok else T.BAD)
        self.run_async(work, on_done=done)

    def on_show(self):
        """Override to refresh state when screen activated."""
        pass

    def on_hide(self):
        """Cancel any in-flight motion when leaving the screen.
        Subclasses can override but should call super().on_hide()."""
        self.robot.cancel_motion(stop_mode=3)  # HOLD = graceful

    def add_intro(self, text):
        """Pin a 'how it works' panel at the top of the body."""
        intro = tk.Frame(self.body, bg=T.PANEL_HI)
        intro.pack(fill="x", pady=(0, 10), before=None)
        tk.Label(intro, text="ⓘ  HOW IT WORKS",
                 bg=T.PANEL_HI, fg=self.accent, anchor="w",
                 font=tkfont.Font(family="DejaVu Sans", size=9, weight="bold")
                 ).pack(fill="x", padx=12, pady=(6, 2))
        tk.Label(intro, text=text, bg=T.PANEL_HI, fg=T.LABEL,
                 anchor="w", justify="left", wraplength=860,
                 font=tkfont.Font(family="DejaVu Sans", size=9)
                 ).pack(fill="x", padx=12, pady=(0, 8))

    # helper: run in worker thread, status updates marshalled to main
    def run_async(self, work_fn, on_done=None):
        def worker():
            try:
                result = work_fn()
            except Exception as exc:
                msg = f"error: {exc}"  # capture as plain string before lambda
                self.after(0, lambda m=msg: self.set_status(m, T.BAD))
                return
            if on_done:
                self.after(0, lambda r=result: on_done(r))
        threading.Thread(target=worker, daemon=True).start()


# ──────── 1. Joint Slider ──────────────────────────────────


class GripperPanel(tk.Frame):
    """모든 화면 하단에 embed 되는 그리퍼 제어 위젯.
    /gripper_command (std_msgs/Float32, 단위 m) 발행."""
    MAX_M = 0.067

    def __init__(self, parent, app):
        super().__init__(parent, bg=T.PANEL)
        self.app = app

        tk.Label(self, text="🤏 Gripper", bg=T.PANEL, fg=T.TITLE,
                 font=tkfont.Font(family="DejaVu Sans", size=10, weight="bold")
                 ).pack(side="left", padx=(12, 8), pady=8)

        tk.Button(self, text="Open", command=self._open,
                  bg=T.OK, fg=T.BG, relief="flat", bd=0,
                  cursor="hand2", padx=10, pady=2,
                  font=tkfont.Font(family="DejaVu Sans", size=9, weight="bold")
                  ).pack(side="left", padx=2, pady=8)

        tk.Button(self, text="Close", command=self._close,
                  bg=T.WARN, fg=T.BG, relief="flat", bd=0,
                  cursor="hand2", padx=10, pady=2,
                  font=tkfont.Font(family="DejaVu Sans", size=9, weight="bold")
                  ).pack(side="left", padx=2, pady=8)

        # Recover — reboot the motor out of an overload + re-home/recalibrate
        tk.Button(self, text="↻ Recover", command=self._recover,
                  bg=T.CAM, fg=T.BG, relief="flat", bd=0,
                  cursor="hand2", padx=10, pady=2,
                  font=tkfont.Font(family="DejaVu Sans", size=9, weight="bold")
                  ).pack(side="left", padx=2, pady=8)

        # Slider: 0~67mm (release 시에만 publish)
        self.var = tk.DoubleVar(value=self.MAX_M * 1000)
        sl = tk.Scale(self, from_=0, to=self.MAX_M * 1000, orient="horizontal",
                      resolution=1, variable=self.var, showvalue=True,
                      bg=T.PANEL, fg=T.LABEL, troughcolor=T.PANEL_HI,
                      activebackground=T.OK, highlightthickness=0,
                      length=200, sliderlength=18,
                      font=tkfont.Font(family="DejaVu Sans Mono", size=8))
        sl.pack(side="left", padx=(8, 4), pady=4)
        sl.bind("<ButtonRelease-1>", lambda e: self._publish(self.var.get() / 1000.0))

        tk.Label(self, text="mm", bg=T.PANEL, fg=T.DIM,
                 font=tkfont.Font(family="DejaVu Sans", size=9)
                 ).pack(side="left", padx=(0, 8))

        self.status_lbl = tk.Label(self, text="ready", bg=T.PANEL, fg=T.DIM,
                                   font=tkfont.Font(family="DejaVu Sans", size=9))
        self.status_lbl.pack(side="right", padx=12)

        # Hammer demo button — runs scripts/ilqr/hammer_demo_2dof.py as subprocess
        self.demo_btn = tk.Button(self, text="🔨 Hammer Demo",
                                  command=self._run_hammer_demo,
                                  bg=T.WP, fg=T.BG, relief="flat", bd=0,
                                  cursor="hand2", padx=12, pady=2,
                                  font=tkfont.Font(family="DejaVu Sans",
                                                    size=9, weight="bold"))
        self.demo_btn.pack(side="right", padx=(8, 4), pady=8)
        self._demo_running = False

        # Trajectory dropdown — CSV 골라서 demo 에 적용 (default = hardcoded movej)
        import glob as _glob
        csvs = sorted(_glob.glob('/kos_workspace/output/swing_traj*.csv'))
        self._traj_paths = {"default (movej, no CSV)": None}
        for p in csvs:
            short = os.path.basename(p).replace('.csv', '').replace('swing_traj_', '')
            self._traj_paths[short] = p
        self._traj_var = tk.StringVar(value="default (movej, no CSV)")
        self.traj_menu = tk.OptionMenu(self, self._traj_var, *self._traj_paths.keys())
        self.traj_menu.config(bg=T.PANEL, fg=T.WP, relief="flat",
                              font=tkfont.Font(family="DejaVu Sans", size=8),
                              highlightthickness=0)
        self.traj_menu.pack(side="right", padx=(0, 4), pady=8)

    def _publish(self, opening_m):
        try:
            self.app.robot.gripper_set(opening_m)
            self.status_lbl.config(text=f"sent: {opening_m*1000:.0f}mm", fg=T.OK)
        except Exception as e:
            self.status_lbl.config(text=f"err: {e}"[:40], fg=T.BAD)

    def _open(self):
        self.var.set(self.MAX_M * 1000)
        self._publish(self.MAX_M)

    def _close(self):
        self.var.set(0)
        self._publish(0.0)

    def _recover(self):
        try:
            self.app.robot.gripper_recover()
            self.status_lbl.config(
                text="recover: reboot + re-home (~8s)…", fg=T.CAM)
        except Exception as e:
            self.status_lbl.config(text=f"err: {e}"[:40], fg=T.BAD)

    def _run_hammer_demo(self):
        if self._demo_running:
            return
        self._demo_running = True
        self.demo_btn.config(state="disabled")
        self.status_lbl.config(text="hammer demo running ...", fg=T.WP)
        threading.Thread(target=self._hammer_demo_worker, daemon=True).start()

    def _hammer_demo_worker(self):
        # Pick namespace by current target_mode: real → /dsr01_real, otherwise → /dsr01.
        # "preview" stays on sim namespace; the demo doesn't have its own preview gate.
        ns = "dsr01_real" if self.app.robot.target_mode == "real" else "dsr01"
        # 선택된 trajectory CSV — None 이면 default (hardcoded movej)
        csv_path = self._traj_paths.get(self._traj_var.get())
        csv_arg = f' --csv {csv_path}' if csv_path else ''
        cmd = ['/bin/bash', '-c',
               f'export DSR_NS={ns} && '
               'source /opt/ros/jazzy/setup.bash && '
               'source /ros2_ws/install/setup.bash && '
               f'python3 /kos_workspace/scripts/ilqr/hammer_demo_2dof.py{csv_arg}']
        rc = -1
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
            for line in p.stdout:
                line = line.rstrip()
                # 진행 상태 캡처: "→ ..." 패턴
                if '→' in line:
                    msg = line.split('→', 1)[1].strip()
                    short = msg[:50]
                    self.app.root.after(0, lambda m=short:
                                         self.status_lbl.config(text=m, fg=T.WP))
            rc = p.wait()
        except Exception as e:
            err = str(e)[:50]
            self.app.root.after(0, lambda:
                                 self.status_lbl.config(text=f"err: {err}", fg=T.BAD))
        finally:
            if rc == 0:
                self.app.root.after(0, lambda:
                                     self.status_lbl.config(text="demo done", fg=T.OK))
            else:
                self.app.root.after(0, lambda:
                                     self.status_lbl.config(text=f"✗ exit {rc}", fg=T.BAD))
            self._demo_running = False
            self.app.root.after(0, lambda: self.demo_btn.config(state="normal"))


# ──────── 3. Distance Estimator ───────────────────────────
