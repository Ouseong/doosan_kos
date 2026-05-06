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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from dsr_msgs2.msg import SpeedjStream
from dsr_msgs2.srv import (
    MoveJoint,
    MoveJointx,
    MoveLine,
    MoveStop,
    Fkin,
    GetCurrentPosj,
    GetCurrentPosx,
    SetRobotMode,
    SetSafetyMode,
    SetSingularityHandling,
    ChangeCollisionSensitivity,
)

# ──────── Workspace safety limits ──────────────────────────
TCP_Z_MIN_MM = 50.0          # don't let the gripper finger-tip drop below this (floor + margin)
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
        # real-robot service clients (lazily created when real driver is started)
        self._real_clients = {}
        # Auto-detect a real driver launched outside this process
        # (e.g. by an external script). Cancels itself once detected.
        self._real_detect_timer = self.create_timer(2.0, self._poll_real_driver)

        # speedj publisher
        self.pub_speedj = self.create_publisher(SpeedjStream, f"{SVC}/speedj_stream", 10)
        # Gripper: Float32 (m, 0=closed, 0.067=fully open). 시뮬 브릿지 + 실 Dynamixel 노드 공통.
        self.pub_gripper = self.create_publisher(Float32, "/gripper_command", 10)
        self._speedj_running = False
        self._speedj_thread = None
        self._speedj_target = [0.0] * 6
        self._speedj_acc = 30.0

    def gripper_set(self, opening_m: float):
        msg = Float32()
        msg.data = float(max(0.0, min(0.067, opening_m)))
        self.pub_gripper.publish(msg)

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
        self.real_ip = robot_ip

        # Idempotence guard: if an external driver is already publishing the
        # /dsr01_real motion services and the robot is reachable, just attach
        # to it instead of spawning a duplicate launch (multiple drivers race
        # for the same TCP and starve each other).
        target = f"/{REAL_ROBOT_ID}/dsr_controller2/motion/move_joint"
        if any(name == target for name, _ in self.get_service_names_and_types()):
            if self._robot_reachable(robot_ip, 12345):
                self.real_driver_started = True
                self._lazy_create_real_clients()
                self.get_logger().info(
                    f"start_real_driver: attaching to existing external driver at {target}")
                return True

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

        success_marker = "Connected to DRCF"
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
        try:
            subprocess.run(
                ["pkill", "-f", f"name:={REAL_ROBOT_ID}"],
                check=False, timeout=3)
        except Exception:
            pass
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
        }

    def _poll_real_driver(self):
        """Detect a real driver started outside this process and attach to it.
        Service registration alone is not enough — also require that the robot
        IP:port is actually reachable so we don't latch onto a driver that's
        still hung on an unreachable host."""
        if self.real_driver_started:
            self._real_detect_timer.cancel()
            return
        target = f"/{REAL_ROBOT_ID}/dsr_controller2/motion/move_joint"
        if not any(name == target for name, _ in self.get_service_names_and_types()):
            return
        if not self._robot_reachable(self.real_ip, 12345):
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
        """Cancel any in-progress motion. stop_mode 3 = HOLD (graceful)."""
        # 1. stop speedj stream if any
        self.speedj_stop()
        # 2. send move_stop service
        if not self.cli_stop.wait_for_service(timeout_sec=0.5):
            return False
        req = MoveStop.Request()
        req.stop_mode = int(stop_mode)
        # fire-and-forget — we don't care about the response
        self.cli_stop.call_async(req)
        return True

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
class WaypointScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "4. Waypoint Recorder", T.WP)
        self.waypoints = []  # list of dicts {posj: [...], posx: [...]}

        self.add_intro(
            "Teach-and-playback. Drive the robot to a pose using any other "
            "mode, then come here and click '+ Save Current' — the current "
            "joint angles are recorded. Build a sequence, reorder with ↑↓, "
            "then '▶ Play All' executes them in order via movej. Save the "
            "list to a JSON file to reuse later.")

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
            lambda: self.robot.movej(wp["posj"], vel=30.0, acc=30.0,
                                     confirm_real_callback=self.app.confirm_real_modal),
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
                ok = self.robot.movej(wp["posj"], vel=30.0, acc=30.0, sync=0,
                                      confirm_real_callback=self.app.confirm_real_modal)
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
class MoveItScreen(ModeScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app, "6. MoveIt2 Planning", T.MOVEIT)

        self.add_intro(
            "MoveIt2 is the ROS-standard motion-planning framework: drag an "
            "interactive marker on the TCP in RViz, click 'Plan & Execute', "
            "and MoveIt finds a collision-free joint trajectory automatically. "
            "Strong for cluttered workspaces and goal-based programming. "
            "Heavier than the other modes (separate window, takes time to "
            "launch). The dsr_moveit2 package needs a one-time colcon build.")

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


# ──────── Gripper footer (Home + 모든 모드 화면 공통) ─────
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

        # Hammer demo button — runs scripts/hammer_demo.py as subprocess
        self.demo_btn = tk.Button(self, text="🔨 Hammer Demo",
                                  command=self._run_hammer_demo,
                                  bg=T.WP, fg=T.BG, relief="flat", bd=0,
                                  cursor="hand2", padx=12, pady=2,
                                  font=tkfont.Font(family="DejaVu Sans",
                                                    size=9, weight="bold"))
        self.demo_btn.pack(side="right", padx=(8, 4), pady=8)
        self._demo_running = False

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

    def _run_hammer_demo(self):
        if self._demo_running:
            return
        self._demo_running = True
        self.demo_btn.config(state="disabled")
        self.status_lbl.config(text="hammer demo 실행 중 …", fg=T.WP)
        threading.Thread(target=self._hammer_demo_worker, daemon=True).start()

    def _hammer_demo_worker(self):
        # Pick namespace by current target_mode: real → /dsr01_real, otherwise → /dsr01.
        # "preview" stays on sim namespace; the demo doesn't have its own preview gate.
        ns = "dsr01_real" if self.app.robot.target_mode == "real" else "dsr01"
        cmd = ['/bin/bash', '-c',
               f'export DSR_NS={ns} && '
               'source /opt/ros/jazzy/setup.bash && '
               'source /ros2_ws/install/setup.bash && '
               'python3 /kos_workspace/scripts/hammer_demo.py']
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
                                     self.status_lbl.config(text="✓ demo 완료", fg=T.OK))
            else:
                self.app.root.after(0, lambda:
                                     self.status_lbl.config(text=f"✗ exit {rc}", fg=T.BAD))
            self._demo_running = False
            self.app.root.after(0, lambda: self.demo_btn.config(state="normal"))


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

        status_color = T.OK if self.app.robot.real_driver_started else T.DIM
        status_text = ("✓ connected" if self.app.robot.real_driver_started
                       else "not started")
        status_lbl = tk.Label(dlg, text=f"Real driver: {status_text}",
                              bg=T.BG, fg=status_color,
                              font=tkfont.Font(family="DejaVu Sans", size=9))
        status_lbl.pack(anchor="w", padx=24, pady=(4, 8))

        def start_real():
            ip = ip_var.get().strip()
            if not ip:
                status_lbl.config(text="enter an IP first", fg=T.BAD)
                return
            status_lbl.config(text=f"Real driver: starting at {ip} ...", fg=T.WARN)

            def worker():
                ok = self.app.robot.start_real_driver(ip)
                msg = (f"✓ connected to {ip}" if ok
                       else f"✗ failed to reach {ip}")
                color = T.OK if ok else T.BAD
                self.app.root.after(0, lambda: status_lbl.config(
                    text=f"Real driver: {msg}", fg=color))
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
                     text="Real driver가 아직 연결돼있지 않아요.\n지금은 SIM 모드만 사용할 수 있어요.",
                     bg=T.BG, fg=T.VAL, justify="center",
                     font=tkfont.Font(family="DejaVu Sans", size=11)
                    ).pack(pady=(2, 10))
            tk.Label(body,
                     text="REAL / PREVIEW 모드를 쓰려면 먼저\n'Start Real Driver' 버튼으로 연결하세요.",
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
            if new_mode in ("real", "preview") and not self.app.robot.real_driver_started:
                show_not_connected_modal()
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
