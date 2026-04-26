#!/usr/bin/env python3
"""
M1013 Joint Jog GUI
6관절 슬라이더로 자세 지정 → Send 누르면 movej 실행.
현재 관절 각도는 /dsr01/joint_states 구독해서 실시간 표시.

Run inside container:
  docker exec doosan_kos bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    python3 /kos_workspace/scripts/joint_jog.py
  "

전제:
  1) doosan_kos 컨테이너 + dsr_bringup2 driver 가 떠있어야 함
  2) DRCF emulator 가 port 12345 에서 LISTEN 중이어야 함
  3) 보고싶으면 Isaac Sim bridge 도 띄워두기
"""

import math
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import MoveJoint, SetRobotMode

ROBOT_ID = "dsr01"
SVC_PREFIX = f"/{ROBOT_ID}/dsr_controller2"

# M1013 datasheet 기준 보수적 관절 한계 (deg)
JOINT_LIMITS_DEG = [
    (-360, 360),  # J1
    (-95,   95),  # J2
    (-135, 135),  # J3
    (-360, 360),  # J4
    (-135, 135),  # J5
    (-360, 360),  # J6
]


class JogNode(Node):
    def __init__(self):
        super().__init__("jog_gui_node", namespace=ROBOT_ID)
        self.move_cli = self.create_client(MoveJoint, f"{SVC_PREFIX}/motion/move_joint")
        self.mode_cli = self.create_client(SetRobotMode, f"{SVC_PREFIX}/system/set_robot_mode")
        self.current_pos_deg = [0.0] * 6
        self.create_subscription(JointState, f"/{ROBOT_ID}/joint_states", self._on_js, 10)

    def _on_js(self, msg: JointState):
        if len(msg.position) >= 6:
            self.current_pos_deg = [math.degrees(p) for p in msg.position[:6]]

    def _wait_future(self, future, timeout):
        # 백그라운드 executor 가 future 를 채우길 기다림
        # (직접 spin 하면 다른 스레드와 충돌하므로 폴링)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.done()

    def ensure_autonomous(self) -> bool:
        if not self.mode_cli.wait_for_service(timeout_sec=2.0):
            return False
        req = SetRobotMode.Request()
        req.robot_mode = 1  # AUTONOMOUS
        future = self.mode_cli.call_async(req)
        if not self._wait_future(future, 3.0):
            future.cancel()
            return False
        r = future.result()
        return bool(r and r.success)

    def send_movej(self, pos_deg, vel=30.0, acc=30.0, sync=0) -> bool:
        if not self.move_cli.wait_for_service(timeout_sec=2.0):
            return False
        req = MoveJoint.Request()
        req.pos = [float(p) for p in pos_deg]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = int(sync)
        future = self.move_cli.call_async(req)
        if not self._wait_future(future, 120.0):
            future.cancel()
            return False
        r = future.result()
        return bool(r and r.success)


class JogGUI:
    def __init__(self, node: JogNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("M1013 Joint Jog")

        top = tk.Frame(self.root)
        top.pack(fill="x", padx=8, pady=6)
        tk.Label(top, text="vel(deg/s):").pack(side="left")
        self.vel_var = tk.DoubleVar(value=30.0)
        tk.Entry(top, textvariable=self.vel_var, width=6).pack(side="left")
        tk.Label(top, text="acc(deg/s²):").pack(side="left", padx=(8, 0))
        self.acc_var = tk.DoubleVar(value=30.0)
        tk.Entry(top, textvariable=self.acc_var, width=6).pack(side="left")

        self.sliders = []
        self.cur_lbls = []
        for i in range(6):
            row = tk.Frame(self.root)
            row.pack(fill="x", padx=8, pady=2)
            tk.Label(row, text=f"J{i+1}", width=3, font=("TkDefaultFont", 10, "bold")).pack(side="left")
            tk.Label(row, text="cur", fg="gray").pack(side="left")
            cur = tk.Label(row, text="   0.0°", width=8, fg="blue", font=("TkFixedFont", 10))
            cur.pack(side="left")
            self.cur_lbls.append(cur)
            tk.Label(row, text="→ target").pack(side="left", padx=(6, 0))
            lo, hi = JOINT_LIMITS_DEG[i]
            sl = tk.Scale(row, from_=lo, to=hi, orient="horizontal",
                          length=350, resolution=0.5, tickinterval=(hi - lo) / 4)
            sl.pack(side="left", expand=True, fill="x")
            self.sliders.append(sl)

        btns = tk.Frame(self.root)
        btns.pack(pady=10)
        tk.Button(btns, text="Sync sliders ← current",
                  command=self._sync_sliders).pack(side="left", padx=4)
        tk.Button(btns, text="Send (blocking)", bg="lightgreen", width=14,
                  command=lambda: self._send(sync=0)).pack(side="left", padx=4)
        tk.Button(btns, text="Send (async)", width=12,
                  command=lambda: self._send(sync=1)).pack(side="left", padx=4)
        tk.Button(btns, text="Home (all 0°)", bg="lightyellow",
                  command=self._home).pack(side="left", padx=4)

        self.status = tk.Label(self.root, text="ready", anchor="w", relief="sunken")
        self.status.pack(fill="x", padx=8, pady=4)

        self._sync_sliders()
        self.root.after(300, self._refresh_current)

    def _refresh_current(self):
        for i, lbl in enumerate(self.cur_lbls):
            lbl.config(text=f"{self.node.current_pos_deg[i]:+7.1f}°")
        self.root.after(300, self._refresh_current)

    def _sync_sliders(self):
        for i, sl in enumerate(self.sliders):
            sl.set(self.node.current_pos_deg[i])
        self.status.config(text="sliders synced to current pose", fg="black")

    def _send(self, sync=0):
        target = [sl.get() for sl in self.sliders]
        mode = "blocking" if sync == 0 else "async"
        self.status.config(text=f"sending ({mode}): {[f'{x:.1f}' for x in target]}", fg="black")

        def worker():
            ok = self.node.send_movej(target, vel=self.vel_var.get(),
                                      acc=self.acc_var.get(), sync=sync)
            self.root.after(0, lambda: self.status.config(
                text="OK" if ok else "FAILED — check driver/emulator",
                fg="green" if ok else "red"))
        threading.Thread(target=worker, daemon=True).start()

    def _home(self):
        for sl in self.sliders:
            sl.set(0.0)
        self._send(sync=0)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = JogNode()

    # spin 을 먼저 띄워야 service call 의 future 가 완료될 수 있음
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if not node.ensure_autonomous():
        node.get_logger().warn("set_robot_mode AUTONOMOUS failed (driver not up?)")

    gui = JogGUI(node)
    try:
        gui.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
