#!/usr/bin/env python3
"""
M1013 hammer demo (iLQR planar swing 버전).

흐름:
  HOME → 망치 위 → 잡기 → 들어올림 → 백스윙 자세 → iLQR swing 으로 못 타격

핵심 변경 vs legacy (hammer_demo_legacy.py):
  - J5 단독 swing 폐기.
  - swing 은 scripts/ilqr/swing_ocp.py 가 미리 산출한 output/swing_traj.csv (101 step,
    dt=10ms) 의 J1..J6 궤적을 MoveSplineJoint 한 방으로 발행.
  - swing plane 락(J1/J4/J6 변동 0°), J2/J3/J5 협응 → 평면 망치질 보장.

좌표 (m1013_gripper_bridge.py 와 일치):
  hammer rest          : (-0.40, -0.45, 0.077)  identity orient
  nail base / head     : (-0.65, -0.45, 0.05) / (-0.65, -0.45, 0.10)
  hammer strike face   : hammer_link 기준 (0, -0.087, -0.027)
  TCP→그리퍼→망치 chain: link_6 → +(0,0.012,0) → gripper → +(0,-0.128,0) → hammer
"""

import math
import os
import time
import csv

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint, MoveJointx, MoveLine, MoveSplineJoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, Empty


# ── Cartesian waypoints (mm + ZYZ deg) — pick/grasp/lift ─────────────────────
WP_APPROACH_HAMMER = [-400.0, -462.0, 305.0, 0.0, 180.0, 0.0]
WP_AT_HAMMER       = [-400.0, -462.0, 205.0, 0.0, 180.0, 0.0]
WP_LIFT            = [-400.0, -462.0, 400.0, 0.0, 180.0, 0.0]

# Backswing pose: swing_traj.csv 의 첫 row 를 자동으로 사용 (load_swing_csv 후 결정)
BACKSWING_DEG = None   # 런타임에 CSV 첫 row 로 채움

# ── Swing trajectory CSV (iLQR 산출, dt=10 ms, 101 row) ──────────────────────
SWING_CSV = "/kos_workspace/output/swing_traj.csv"
SWING_DURATION_S = 1.5   # emulator J2 한계(120°/s) 안 — 1.0s 면 RC_ERROR 로 reject 됨
N_STRIKES        = 3     # 망치질 반복 횟수 — 각 strike 후 backswing 으로 돌아감


def main():
    # Namespace selectable via env var: DSR_NS=dsr01 (sim, default) or dsr01_real (real robot).
    ns = os.environ.get('DSR_NS', 'dsr01')

    rclpy.init()
    node = Node('hammer_demo_ilqr')
    node.get_logger().info(f"Using namespace /{ns}/")

    cli_movej   = node.create_client(MoveJoint,        f'/{ns}/dsr_controller2/motion/move_joint')
    cli_movejx  = node.create_client(MoveJointx,       f'/{ns}/dsr_controller2/motion/move_jointx')
    cli_movel   = node.create_client(MoveLine,         f'/{ns}/dsr_controller2/motion/move_line')
    cli_spline  = node.create_client(MoveSplineJoint,  f'/{ns}/dsr_controller2/motion/move_spline_joint')

    node.get_logger().info("Waiting for motion services ...")
    for c, name in ((cli_movej, 'movej'), (cli_movejx, 'movejx'),
                    (cli_movel, 'movel'), (cli_spline, 'move_spline_joint')):
        if not c.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(f"{name} service not ready"); return

    joints_state = {'pos_deg': None}
    def _js_cb(msg):
        joints_state['pos_deg'] = [math.degrees(p) for p in msg.position]
    node.create_subscription(JointState, f'/{ns}/joint_states', _js_cb, 10)

    gp_pub    = node.create_publisher(Float32, '/gripper_command', 10)
    reset_pub = node.create_publisher(Empty,   '/hammer_reset',    10)

    def reset_hammer():
        node.get_logger().info("→ RESET hammer to rest pose")
        for _ in range(3):
            reset_pub.publish(Empty()); time.sleep(0.1)
        time.sleep(0.5)

    def movejx(label, posx, vel=30.0, acc=30.0, sol=2):
        node.get_logger().info(
            f"→ {label}: posx=({posx[0]:+.0f}, {posx[1]:+.0f}, {posx[2]:+.0f}) mm")
        req = MoveJointx.Request()
        req.pos = [float(p) for p in posx]
        req.vel = float(vel); req.acc = float(acc)
        req.time = 0.0; req.radius = 0.0
        req.ref = 0; req.sol = int(sol)
        req.mode = 0; req.blend_type = 0; req.sync_type = 0
        fut = cli_movejx.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        time.sleep(0.5)

    def movej(label, joints_deg, vel=30.0, acc=30.0):
        node.get_logger().info(
            f"→ {label} (movej): {[round(j,1) for j in joints_deg]}  v={vel:.0f}°/s")
        req = MoveJoint.Request()
        req.pos = [float(a) for a in joints_deg]
        req.vel = float(vel); req.acc = float(acc)
        req.time = 0.0; req.radius = 0.0
        req.mode = 0; req.blend_type = 0; req.sync_type = 0
        fut = cli_movej.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        time.sleep(0.3)

    def capture_joints(timeout=2.0):
        deadline = time.time() + timeout
        joints_state['pos_deg'] = None
        while joints_state['pos_deg'] is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return joints_state['pos_deg']

    def grip(opening_m, label):
        node.get_logger().info(f"→ {label}: gripper {opening_m*1000:.0f}mm")
        msg = Float32(); msg.data = float(opening_m)
        for _ in range(5):
            gp_pub.publish(msg); time.sleep(0.1)
        time.sleep(1.5)

    def load_swing_csv(path):
        rows = []
        with open(path) as f:
            r = csv.reader(f)
            next(r)   # header
            for line in r:
                if not line: continue
                rows.append([float(x) for x in line[:6]])
        return rows

    def move_spline(label, waypoints, total_time_s):
        """MoveSplineJoint with cascading fallback (21pt → 11pt → single movej).

        QEMU 에서 N=100 Catmull spline 계산이 16s 걸려 client timeout.
        N≈20 은 ~0.6s 예상 (QEMU 비용이 N²로 스케일한다는 가정 하).
        실패 시 11pt → 단일 movej 로 자동 후퇴."""
        vel_lim = [120.0, 120.0, 180.0, 200.0, 200.0, 200.0]   # M1013 한계 ~90%
        acc_lim = [300.0, 300.0, 400.0, 500.0, 500.0, 500.0]

        for stride, tag in [(5, "21pt"), (10, "11pt")]:
            sub = waypoints[::stride]
            if sub[-1] is not waypoints[-1]:
                sub.append(waypoints[-1])

            node.get_logger().info(
                f"→ {label} ({tag}, {len(sub)} pts, T={total_time_s:.1f}s)")

            req = MoveSplineJoint.Request()
            req.pos = []
            for row in sub:
                arr = Float64MultiArray()
                arr.data = [float(j) for j in row]
                req.pos.append(arr)
            req.pos_cnt = len(sub)
            req.vel = vel_lim
            req.acc = acc_lim
            req.time = float(total_time_s)
            req.mode = 0
            req.sync_type = 0

            fut = cli_spline.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=25.0)

            if fut.done() and fut.result() is not None and fut.result().success:
                time.sleep(total_time_s + 0.3)
                node.get_logger().info(f"   ✓ {tag} OK")
                return
            node.get_logger().warn(f"   ✗ {tag} 실패/timeout — 다음 단계로")

        # Final fallback: single movej (iLQR 형상 잃음, 양 끝 자세만 매칭)
        node.get_logger().warn(f"→ {label} (single-movej fallback, T={total_time_s:.1f}s)")
        req = MoveJoint.Request()
        req.pos = [float(a) for a in waypoints[-1]]
        req.vel = 120.0; req.acc = 300.0
        req.time = float(total_time_s); req.radius = 0.0
        req.mode = 0; req.blend_type = 0; req.sync_type = 0
        fut = cli_movej.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        time.sleep(total_time_s + 0.3)

    # ─────────────────────────── DEMO ──────────────────────────────────────
    node.get_logger().info("=== Hammer demo (iLQR planar swing) start ===")

    # CSV 미리 읽기 (없으면 demo 실패)
    if not os.path.exists(SWING_CSV):
        node.get_logger().error(
            f"Swing trajectory CSV not found: {SWING_CSV}\n"
            f"   → 먼저 ~/ilqr_venv/bin/python3 scripts/ilqr/swing_ocp.py 실행하세요.")
        return
    swing_pts = load_swing_csv(SWING_CSV)
    node.get_logger().info(f"   loaded {len(swing_pts)} swing waypoints from {SWING_CSV}")
    nonlocal_backswing = swing_pts[0]   # CSV 첫 row = iLQR q0
    node.get_logger().info(
        f"   backswing pose (CSV row 0) = {[round(x,2) for x in nonlocal_backswing]}")

    reset_hammer()
    grip(0.067, "OPEN")

    # PRE_ALIGN: HOME → APPROACH 의 long-way J1 회전 회피.
    cur0 = capture_joints()
    if cur0 is not None:
        pre_align = [-134.0, cur0[1], cur0[2], cur0[3], cur0[4], -134.0]
        movej('PRE_ALIGN (short J1/J6)', pre_align, vel=40.0, acc=40.0)

    # Pick & lift
    movejx('APPROACH', WP_APPROACH_HAMMER)
    movejx('AT_HAMMER', WP_AT_HAMMER, vel=20.0, acc=20.0)
    grip(0.020, "CLOSE + auto-grasp")
    movejx('LIFT', WP_LIFT, vel=20.0, acc=20.0)

    # Backswing pose (iLQR q₀ = CSV first row) — 첫 진입은 큰 자세 변화이므로 천천히
    movej('BACKSWING 1', nonlocal_backswing, vel=40.0, acc=60.0)

    # iLQR swing × N (각 strike 후 backswing 으로 돌아가서 다시 swing)
    for i in range(N_STRIKES):
        move_spline(f'STRIKE {i+1}/{N_STRIKES}', swing_pts, SWING_DURATION_S)
        if i < N_STRIKES - 1:
            # 임팩트 직후 자세 → 백스윙 자세 복귀.  이미 가까운 자세이므로 빠르게.
            movej(f'RECOVER → BACKSWING {i+2}', nonlocal_backswing,
                  vel=80.0, acc=120.0)

    node.get_logger().info(
        f"=== Demo done — {N_STRIKES} iLQR planar strikes 완료 ===")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
