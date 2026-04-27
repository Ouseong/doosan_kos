#!/usr/bin/env python3
"""
Dynamixel 그리퍼 ROS2 노드

Subscribe: /gripper_command  (control_msgs/GripperCommand)
  - position > 0  → 열기
  - position == 0 → 닫기 (천천히 닫다가 토크 임계값 초과 시 유지)
  - max_effort    → 토크 임계값 (mNm), 0이면 파라미터값 사용

Publish:   /gripper_state   (sensor_msgs/JointState)
  - name:     ['gripper_left_joint', 'gripper_right_joint']
  - position: 각 플레이트 변위 (m)
  - velocity: RPM → m/s 변환값
  - effort:   토크 (Nm)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.msg import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from dynamixel_sdk import PortHandler, PacketHandler

# ── 제어 테이블 주소 ──────────────────────────────────
ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_VELOCITY_LIMIT   = 44
ADDR_GOAL_VELOCITY    = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_LOAD     = 126

# ── 단위 변환 ─────────────────────────────────────────
UNIT_TO_MNM = 3.0    # load unit → mNm
UNIT_TO_RPM = 0.229  # velocity unit → RPM
MAX_OPENING_M = 0.067  # 최대 열림 67mm

CLOSE_VEL_UNIT =  22   # ~5 RPM, 닫히는 방향 (음수면 방향 반대)
OPEN_VEL_UNIT  = -22   # 열리는 방향 (실제 방향은 하드웨어 확인 후 부호 조정)


def twos_complement(val, byte_size):
    if byte_size == 2:
        return val if val < 32768 else val - 65536
    if byte_size == 4:
        return val if val < 2147483648 else val - 4294967296
    return val


class GripperNode(Node):

    def __init__(self):
        super().__init__('gripper_node')

        # ── ROS2 파라미터 ────────────────────────────────
        self.declare_parameter('device',           'COM5')       # Linux: /dev/ttyUSB0
        self.declare_parameter('baudrate',         57600)
        self.declare_parameter('dxl_id',           3)
        self.declare_parameter('torque_threshold', 200.0)        # mNm
        self.declare_parameter('close_velocity',   CLOSE_VEL_UNIT)
        self.declare_parameter('publish_rate',     20.0)         # Hz

        device    = self.get_parameter('device').value
        baudrate  = self.get_parameter('baudrate').value
        self.dxl_id          = self.get_parameter('dxl_id').value
        self.torque_threshold = self.get_parameter('torque_threshold').value
        self.close_vel        = self.get_parameter('close_velocity').value
        publish_rate          = self.get_parameter('publish_rate').value

        # ── Dynamixel 초기화 ─────────────────────────────
        self.port = PortHandler(device)
        self.pkt  = PacketHandler(2.0)

        if not self.port.openPort() or not self.port.setBaudRate(baudrate):
            self.get_logger().fatal(f'Dynamixel 연결 실패: {device}')
            raise RuntimeError('Dynamixel 연결 실패')

        self._init_motor()
        self.get_logger().info(f'Dynamixel 연결 성공 (ID={self.dxl_id}, {device})')

        # ── 상태 변수 ────────────────────────────────────
        self.state = 'idle'   # idle | closing | opening | holding

        # ── ROS2 Pub/Sub ─────────────────────────────────
        self.cmd_sub = self.create_subscription(
            GripperCommand, '/gripper_command', self._cmd_cb, 10)

        self.state_pub = self.create_publisher(JointState, '/gripper_state', 10)

        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._publish_state)

    # ── 모터 초기화 ──────────────────────────────────────
    def _init_motor(self):
        self.pkt.write1ByteTxRx(self.port, self.dxl_id, ADDR_TORQUE_ENABLE,  0)
        self.pkt.write1ByteTxRx(self.port, self.dxl_id, ADDR_OPERATING_MODE, 1)   # 속도 제어
        self.pkt.write4ByteTxRx(self.port, self.dxl_id, ADDR_VELOCITY_LIMIT, 240) # ~55 RPM
        self.pkt.write1ByteTxRx(self.port, self.dxl_id, ADDR_TORQUE_ENABLE,  1)

    def _set_velocity(self, vel_unit: int):
        self.pkt.write4ByteTxRx(self.port, self.dxl_id, ADDR_GOAL_VELOCITY, int(vel_unit))

    # ── /gripper_command 콜백 ────────────────────────────
    def _cmd_cb(self, msg: GripperCommand):
        threshold = msg.max_effort if msg.max_effort > 0 else self.torque_threshold

        if msg.position > 0.0:
            # 열기
            self.state = 'opening'
            self._set_velocity(OPEN_VEL_UNIT)
            self.get_logger().info(f'그리퍼 열기')

        else:
            # 닫기 — 토크 임계값 도달 시 유지
            self.state = 'closing'
            self.torque_threshold = threshold
            self._set_velocity(self.close_vel)
            self.get_logger().info(f'그리퍼 닫기 (임계 토크: {threshold:.0f} mNm)')

    # ── 상태 발행 + 토크 감지 루프 ──────────────────────
    def _publish_state(self):
        try:
            load_raw = self.pkt.read2ByteTxRx(self.port, self.dxl_id, ADDR_PRESENT_LOAD)[0]
            pos_raw  = self.pkt.read4ByteTxRx(self.port, self.dxl_id, ADDR_PRESENT_POSITION)[0]
            vel_raw  = self.pkt.read4ByteTxRx(self.port, self.dxl_id, ADDR_PRESENT_VELOCITY)[0]
        except Exception as e:
            self.get_logger().warn(f'센서 읽기 실패: {e}')
            return

        torque_mnm = twos_complement(load_raw, 2) * UNIT_TO_MNM
        vel_rpm    = twos_complement(vel_raw,   4) * UNIT_TO_RPM
        pos_norm   = (pos_raw % 4096) / 4096.0   # 0~1 정규화

        # ── 토크 기반 파악 감지 ──────────────────────────
        if self.state == 'closing' and abs(torque_mnm) > self.torque_threshold:
            self._set_velocity(0)
            self.state = 'holding'
            self.get_logger().info(
                f'물체 파악! 토크 {torque_mnm:.1f} mNm → 위치 유지')

        # ── /gripper_state 발행 ──────────────────────────
        # 모터 1개가 양쪽 플레이트를 대칭으로 구동하므로 각 플레이트 변위는 절반
        plate_pos = pos_norm * MAX_OPENING_M / 2.0   # 각 플레이트 변위 (m)
        plate_vel = (vel_rpm / 60.0) * MAX_OPENING_M # m/s 근사

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['gripper_left_joint', 'gripper_right_joint']
        js.position = [plate_pos,          plate_pos]
        js.velocity = [plate_vel,          plate_vel]
        js.effort   = [torque_mnm / 1000.0, torque_mnm / 1000.0]  # mNm → Nm

        self.state_pub.publish(js)

    # ── 종료 처리 ────────────────────────────────────────
    def destroy_node(self):
        self._set_velocity(0)
        self.port.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GripperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
    finally:
        if 'node' in dir():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
