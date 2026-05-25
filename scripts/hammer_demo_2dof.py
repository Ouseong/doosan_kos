#!/usr/bin/env python3
"""
M1013 hammer demo — 2-DOF (J3 + J5 only) iLQR swing.

흐름:
  HOME → 망치 위 → 잡기 → 들어올림 → 백스윙 자세 → 2-DOF swing 으로 못 타격

vs hammer_demo.py:
  - J2 락 (어깨 고정 25.58°), J3+J5 만 swing → mechanically aligned posture
  - 백스윙에서 망치가 그리퍼 위로 들림 (J5 = -67°) → 진짜 "위에서 내려치기"
  - CSV: output/swing_traj_servoj_2dof_J2+26.csv  (13열: t, q, q̇)
  - swing_servoj_2dof.py 산출, T=100 (1초), J3 -40° + J5 -120° backswing offset

좌표 (m1013_gripper_bridge.py 와 일치):
  hammer rest          : (-0.40, -0.45, 0.077)  identity orient
  nail base / head     : (-0.65, -0.45, 0.05) / (-0.65, -0.45, 0.10)
  hammer strike face   : hammer_link 기준 (0, -0.087, -0.027)
  TCP→그리퍼→망치 chain: link_6 → +(0,0.012,0) → gripper → +(0,-0.128,0) → hammer
"""

import argparse
import math
import os
import time
import csv
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint, MoveJointx, MoveLine, MoveSplineJoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, Empty


# ── Cartesian waypoints (mm + ZYZ deg) — pick/grasp/lift ─────────────────────
WP_APPROACH_HAMMER = [-400.0, -462.0, 305.0, 0.0, 180.0, 0.0]
WP_AT_HAMMER       = [-400.0, -462.0, 205.0, 0.0, 180.0, 0.0]
WP_LIFT            = [-400.0, -462.0, 400.0, 0.0, 180.0, 0.0]

# ── Backswing / Impact 자세 (수동 정의, J1/J4/J6 동일 → swing 평면 자동 락) ───
# J3, J5 만 변동.  J2 도 동일 (락).
BACKSWING_DEG = [-148.06, 15.80,  94.84, 0.00,  -5.69, -92.76]  # α=20° + J5 -75°
IMPACT_DEG    = [-148.06, 25.58, 101.30, 0.00,  53.10, -92.76]
# α=20° Jacobian-역수 backswing 의 J5 만 -75° 회전.
# J1/J4/J6 동일 (락), J2/J3/J5 만 swing.

# STRIKE 의 vel/acc — time=0 으로 firmware 가 자동으로 최단 시간 계산
# vel 225 = J5 SPEC, acc 500 = SPEC.  →  swing ~0.85s (vel-limited)
STRIKE_VEL = 225.0   # J5/J6 SPEC 한계 (M1013 URDF)
STRIKE_ACC = 500.0   # J5/J6 acc SPEC

N_STRIKES  = 5       # 망치질 반복 횟수

# 디버그 dump (단일 movej 라 굳이 필요 없지만 유지)
DEBUG_DUMP_PATH = "/tmp/hammer_demo_2dof_sent.jsonl"


def main():
    # ── CLI: --csv 로 trajectory CSV 지정 가능 ──
    # 없으면 default (BACKSWING_DEG/IMPACT_DEG hardcoded + movej STRIKE) ─ 교수님 demo 용 baseline.
    # 있으면 CSV 의 첫 row = BACKSWING, 끝 row = IMPACT, STRIKE 는 MoveSplineJoint(전체 CSV) 로 발행.
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=None,
                    help='CSV path (없으면 hardcoded default 사용)')
    args = ap.parse_args()

    # ── BACKSWING/IMPACT 결정 ──
    global BACKSWING_DEG, IMPACT_DEG  # 상단 hardcoded 값을 덮어쓰기 가능하게
    swing_pts = None   # CSV mode 일 때만 채워짐
    if args.csv is not None and os.path.exists(args.csv):
        with open(args.csv) as f:
            r = csv.reader(f); header = next(r)
            is_13col = header[0].strip().startswith('t')
            sl = slice(1, 7) if is_13col else slice(0, 6)
            swing_pts = [[float(x) for x in line[sl]] for line in r if line]
        BACKSWING_DEG = swing_pts[0]
        IMPACT_DEG    = swing_pts[-1]
        demo_mode = f"CSV ({os.path.basename(args.csv)}, {len(swing_pts)} rows)"
    else:
        demo_mode = "default (hardcoded movej)"

    # Namespace selectable via env var: DSR_NS=dsr01 (sim, default) or dsr01_real (real robot).
    ns = os.environ.get('DSR_NS', 'dsr01')

    rclpy.init()
    node = Node('hammer_demo_ilqr')
    node.get_logger().info(f"Using namespace /{ns}/")
    node.get_logger().info(f"Mode: {demo_mode}")

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

    def movej(label, joints_deg, vel=30.0, acc=30.0, time_s=0.0):
        node.get_logger().info(
            f"→ {label} (movej): {[round(j,1) for j in joints_deg]}  v={vel:.0f}°/s  t={time_s:.2f}s")
        req = MoveJoint.Request()
        req.pos = [float(a) for a in joints_deg]
        req.vel = float(vel); req.acc = float(acc)
        req.time = float(time_s); req.radius = 0.0
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
        """13열 (t, J1..J6, J1_dot..J6_dot) 자동 감지 — 6열 구버전도 지원."""
        rows = []
        with open(path) as f:
            r = csv.reader(f)
            header = next(r)
            is_13col = header[0].strip().startswith('t')
            sl = slice(1, 7) if is_13col else slice(0, 6)
            for line in r:
                if not line: continue
                rows.append([float(x) for x in line[sl]])
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

            with open(DEBUG_DUMP_PATH, "a") as f:
                f.write(json.dumps({
                    "ts": datetime.now().isoformat(timespec="milliseconds"),
                    "label": label,
                    "tag": tag,
                    "service": "MoveSplineJoint",
                    "pos_cnt": req.pos_cnt,
                    "pos": [list(arr.data) for arr in req.pos],
                    "vel": list(req.vel),
                    "acc": list(req.acc),
                    "time": req.time,
                    "mode": req.mode,
                    "sync_type": req.sync_type,
                }) + "\n")

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

        with open(DEBUG_DUMP_PATH, "a") as f:
            f.write(json.dumps({
                "ts": datetime.now().isoformat(timespec="milliseconds"),
                "label": label,
                "tag": "single-movej-fallback",
                "service": "MoveJoint",
                "pos": list(req.pos),
                "vel": req.vel, "acc": req.acc, "time": req.time,
                "mode": req.mode, "blend_type": req.blend_type, "sync_type": req.sync_type,
            }) + "\n")

        fut = cli_movej.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        time.sleep(total_time_s + 0.3)

    # ─────────────────────────── DEMO ──────────────────────────────────────
    node.get_logger().info("=== Hammer demo (simple movej swing) start ===")
    node.get_logger().info(f"   BACKSWING = {BACKSWING_DEG}")
    node.get_logger().info(f"   IMPACT    = {IMPACT_DEG}")
    node.get_logger().info(f"   ※ J1/J2/J4/J6 동일 → swing 평면 자동 락")

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
    grip(0.0, "CLOSE + auto-grasp")
    movejx('LIFT', WP_LIFT, vel=20.0, acc=20.0)

    # Backswing pose — 첫 진입은 큰 자세 변화이므로 천천히
    movej('BACKSWING 1', BACKSWING_DEG, vel=40.0, acc=60.0)

    # STRIKE 단계 — 모드 분기:
    #   default (no --csv): 단일 movej(BACKSWING → IMPACT), firmware trapezoidal.
    #   CSV mode: MoveSplineJoint 로 전체 trajectory 보냄 (21pt 다운샘플, 내부 shape 유지).
    for i in range(N_STRIKES):
        if swing_pts is not None:
            move_spline(f'STRIKE {i+1}/{N_STRIKES}', swing_pts, 1.5)
        else:
            movej(f'STRIKE {i+1}/{N_STRIKES}', IMPACT_DEG,
                  vel=STRIKE_VEL, acc=STRIKE_ACC, time_s=0.0)
        if i < N_STRIKES - 1:
            movej(f'RECOVER → BACKSWING {i+2}', BACKSWING_DEG,
                  vel=STRIKE_VEL, acc=STRIKE_ACC)

    node.get_logger().info(
        f"=== Demo done — {N_STRIKES} simple swing 완료 ===")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
