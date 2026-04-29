#!/usr/bin/env python3
"""
M1013 hammer demo:  HOME → 망치 위 → 잡기 → 들어올림 → 못 위 strike 정렬

Cartesian (movejx) 으로 명령 — Doosan controller 가 IK 처리.
posx = (x_mm, y_mm, z_mm, A_deg, B_deg, C_deg)  ZYZ Euler.
B=180 → tool +Z 가 world -Z (아래로 향함), gripper plates 가 아래.

기하 (m1013_gripper_bridge.py 와 일치):
  hammer rest          : (-0.40, -0.45, 0.077)  identity orient
  nail base            : (-0.65, -0.45, 0.05)
  hammer strike local  : (0, -0.087, -0.027)
  gripper grasp center : TCP + (0, 0.012, 0)  +  rotate(gripper, (0,-0.128,0))
                        = TCP + (0, 0.012, -0.128)  when B=180
"""

import math
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint, MoveJointx, MoveLine
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Empty
import time


# Cartesian waypoints (TCP world pose, mm + ZYZ deg).
WP_APPROACH_HAMMER = [-400.0, -462.0, 305.0, 0.0, 180.0, 0.0]
WP_AT_HAMMER       = [-400.0, -462.0, 205.0, 0.0, 180.0, 0.0]
WP_LIFT            = [-400.0, -462.0, 400.0, 0.0, 180.0, 0.0]
WP_ABOVE_NAIL      = [-579.0, -412.0, 355.0, 0.0, 180.0, 0.0]   # J6 += 55° 회전 보정.
                                                                  # 회전 후 strike 가 (-0.65,-0.45,0.20) 못 바로 위.
WP_STRIKE_DOWN     = [-579.0, -412.0, 255.0, 0.0, 180.0, 0.0]   # cartesian fallback (J6 회전 후 nail XY 보정)

# J5-only swing 셋업.  목표: handle 의 긴 축이 vertical plane 에서 up-down,
# strike face 가 nail 머리에 닿음.
#
# 기하:
#   IK 가 ABOVE_NAIL 에서 J1 ≈ world arm angle (-145°).
#   J5 axis 는 arm plane 에 수직 (= horizontal direction perpendicular to arm).
#   Handle (hammer +Y, world +Y at identity) 이 J5 swing plane (= arm plane) 에 있어야
#   J5 회전이 handle 의 up-down 운동이 됨.
#
#   Arm plane 의 horizontal direction = (-0.824, -0.570, 0) (nail 방향).
#   Handle 을 이 vertical plane 안으로 회전: world Z 축 기준 -55.3° 회전.
#   J6 (tool axis = world -Y when B=180) 회전이 handle 을 vertical plane 안으로 들임.
#   IK J6 = ~207°, hammer 회전 -55.3° 위해 J6 += 55.3° → J6 ≈ 262°.
#
# 모든 값은 추정.  None 이면 IK 값 유지.
SWING_J1_DEG     = None      # IK 의 nail 방향 J1 유지
SWING_J4_DEG     = None      # IK = 0 유지: J5 axis 가 arm plane 에 수직 (= horizontal,
                              # nail 방향 vertical plane 의 normal).  J5 swing 은
                              # 이 vertical plane 안에서 일어남 → handle 앞뒤 운동.
# J6 override: handle (default world +Y) 을 arm plane 안으로 회전.
# arm plane 의 horizontal direction = (-0.824, -0.570) (nail 향함).  handle 을 반대 방향
# (+0.824, +0.570 = world +27°) 으로 정렬하면 head 가 nail 향함, handle 이 plane 안.
# IK J6 = -152.7°, +55° 회전 필요 → J6 = -98°.
SWING_J6_DEG     = -98.0
J5_DELTA_STRIKE_DEG = -45.0  # 큰 swing (handle 앞뒤 흔들림 강조).
                              # 주의: arc 가 nail 정확히 닿지 못함 (wrist z=0.48 vs nail z=0.10
                              # 거리 차이로 같은 arc 불가능).  근사 strike 시연용.


def main():
    rclpy.init()
    node = Node('hammer_demo')

    cli         = node.create_client(MoveJointx, '/dsr01/dsr_controller2/motion/move_jointx')
    cli_movej   = node.create_client(MoveJoint,  '/dsr01/dsr_controller2/motion/move_joint')
    cli_movel   = node.create_client(MoveLine,   '/dsr01/dsr_controller2/motion/move_line')
    node.get_logger().info("Waiting for motion services ...")
    for c, name in ((cli, 'movejx'), (cli_movej, 'movej'), (cli_movel, 'movel')):
        if not c.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(f"{name} service not ready"); return

    # /dsr01/joint_states 캡처용
    joints_state = {'pos_deg': None}
    def _js_cb(msg):
        joints_state['pos_deg'] = [math.degrees(p) for p in msg.position]
    node.create_subscription(JointState, '/dsr01/joint_states', _js_cb, 10)

    gp_pub    = node.create_publisher(Float32, '/gripper_command', 10)
    reset_pub = node.create_publisher(Empty,   '/hammer_reset',    10)

    def reset_hammer():
        node.get_logger().info("→ RESET hammer to rest pose")
        for _ in range(3):
            reset_pub.publish(Empty()); time.sleep(0.1)
        time.sleep(0.5)

    def movejx(label, posx, vel=30.0, acc=30.0, sol=2):
        node.get_logger().info(
            f"→ {label}: posx=({posx[0]:+.0f}, {posx[1]:+.0f}, {posx[2]:+.0f}) "
            f"mm  ABC=({posx[3]:.0f}, {posx[4]:.0f}, {posx[5]:.0f})°")
        req = MoveJointx.Request()
        req.pos = [float(p) for p in posx]
        req.vel = float(vel); req.acc = float(acc)
        req.time = 0.0; req.radius = 0.0
        req.ref = 0   # base coord
        req.sol = int(sol)
        req.mode = 0; req.blend_type = 0; req.sync_type = 0
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        r = fut.result()
        if r is None or not getattr(r, 'success', False):
            node.get_logger().warn(f"   {label} movejx returned failure")
        time.sleep(0.5)

    def movel(label, posx, vel_lin=200.0, acc_lin=400.0, vel_ang=30.0, acc_ang=60.0):
        node.get_logger().info(
            f"→ {label} (movel): TCP_z={posx[2]:+.0f}mm  v={vel_lin:.0f}mm/s  a={acc_lin:.0f}mm/s²")
        req = MoveLine.Request()
        req.pos = [float(p) for p in posx]
        req.vel = [float(vel_lin), float(vel_ang)]
        req.acc = [float(acc_lin), float(acc_ang)]
        req.time = 0.0; req.radius = 0.0
        req.ref = 0; req.mode = 0
        req.blend_type = 0; req.sync_type = 0
        fut = cli_movel.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        r = fut.result()
        if r is None or not getattr(r, 'success', False):
            node.get_logger().warn(f"   {label} movel returned failure")
        time.sleep(0.2)

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
        # 최신 joint_state 받을 때까지 spin
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

    node.get_logger().info("=== Hammer demo start ===")
    reset_hammer()
    grip(0.067,        "OPEN")

    # PRE_ALIGN: HOME(J1=0) → APPROACH(IK J1≈+226°) 가 long-way 회전이 되는 문제 회피.
    # J1, J6 를 short-way 등가값(-134°)으로 먼저 돌려놓고 movejx 호출하면
    # IK 가 가까운 분기를 선택.
    cur0 = capture_joints()
    if cur0 is not None:
        pre_align = [-134.0, cur0[1], cur0[2], cur0[3], cur0[4], -134.0]
        movej('PRE_ALIGN (short J1/J6)', pre_align, vel=40.0, acc=40.0)

    movejx('APPROACH', WP_APPROACH_HAMMER)
    movejx('AT_HAMMER', WP_AT_HAMMER, vel=20.0, acc=20.0)
    grip(0.020,        "CLOSE + auto-grasp")
    movejx('LIFT',     WP_LIFT, vel=20.0, acc=20.0)
    movejx('ABOVE_NAIL', WP_ABOVE_NAIL)

    # ── J5-only swing 셋업 ──
    cur = capture_joints()
    if cur is None:
        node.get_logger().error("joint_state 못 받음 — strike 생략"); return
    node.get_logger().info(
        f"   IK joints: J1={cur[0]:+.1f}, J2={cur[1]:+.1f}, J3={cur[2]:+.1f}, "
        f"J4={cur[3]:+.1f}, J5={cur[4]:+.1f}, J6={cur[5]:+.1f}")

    # Swing-ready: SWING_*_DEG override 가 있으면 그 값, 없으면 IK 유지
    overrides = [SWING_J1_DEG, None, None, SWING_J4_DEG, None, SWING_J6_DEG]
    ready = [(o if o is not None else c) for o, c in zip(overrides, cur)]
    if ready != cur:
        movej('SWING_READY (override align)', ready, vel=30.0, acc=30.0)
    else:
        node.get_logger().info("→ SWING_READY: override 없음, IK 자세 유지")

    # 정렬 후 다시 capture (J1 변경으로 IK 재계산 안 되므로 ready 와 동일하지만,
    # emulator 가 reach 못해 다른 값으로 클램프했을 수도 있음)
    cur2 = capture_joints()
    if cur2 is None:
        cur2 = ready
    node.get_logger().info(
        f"   ready joints: {[round(j,1) for j in cur2]}")

    # ── Strike: J5 ONLY ──
    strike = list(cur2)
    strike[4] = cur2[4] + J5_DELTA_STRIKE_DEG
    movej('STRIKE (J5 only)', strike, vel=200.0, acc=400.0)
    time.sleep(0.3)
    movej('RETURN (J5 only)', cur2, vel=80.0, acc=120.0)

    node.get_logger().info("=== Demo done — J5 swing 완료 ===")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
