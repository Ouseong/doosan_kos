#!/usr/bin/env python3
"""
Dynamixel gripper ROS2 node — v2 (position mode).

Topic interface mirrors the Isaac Sim bridge so control_console.py's
gripper panel drives sim and real motor with the same publish:
  Subscribe: /gripper_command  (std_msgs/Float32, opening in meters, 0~0.067)
  Publish:   /gripper_state    (sensor_msgs/JointState)

Mechanism: position mode (operating_mode=3). User confirmed motor
position 0 == gripper fully open. The opening width is mapped linearly:

  opening_m = 0.067 (Open)  → goal_position = OPEN_TICKS  (= 0)
  opening_m = 0.0   (Close) → goal_position = CLOSE_TICKS
  in-between values are interpolated.

Both endpoints are ROS2 parameters so they can be calibrated without
editing the file. Start with a conservative CLOSE_TICKS and grow it
while watching for mechanical hard-stop.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from dynamixel_sdk import PortHandler, PacketHandler

DEVICE = '/dev/ttyUSB0'
BAUD = 57600
DXL_ID = 3
PROTOCOL = 2.0

# Control table — XM430-W350, Protocol 2.0
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_LOAD     = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

# Unit conversions (XM430)
UNIT_TO_MNM   = 3.0
UNIT_TO_RPM   = 0.229
MAX_OPENING_M = 0.067

# Default endpoints — captured by hand-positioning the gripper against its
# actual mechanical range (torque off, read present_position):
#   fully open  -> 3974 ticks
#   fully close -> 2280 ticks
# Direction: closing DECREASES ticks (3974 -> 2280), span ≈ 1694 ticks.
# The previous open=0 was wrong: driving toward 0 jammed the finger into a
# hard stop and tripped the overload error (hardware_error bit5), which
# auto-disabled torque — that is why "open" appeared to do nothing.
# Extended-position mode (4) honors the signed delta either direction.
DEFAULT_OPEN_TICKS  = 3974
DEFAULT_CLOSE_TICKS = 2280

# Motion profile. profile_velocity=0 means "unlimited" (capped by the
# Velocity Limit reg, default 1023 ≈ 234 RPM) which is what the Dynamixel
# Wizard uses, so the motor can power through the gripper's mechanical
# friction. A nonzero value caps speed but lets friction stall the motor
# at low values; raise this if you see overload errors mid-travel.
DEFAULT_PROFILE_VELOCITY = 100   # ≈ 23 RPM, plenty to beat mechanism friction


def twos_complement(val, byte_size):
    if byte_size == 2:
        return val if val < 32768 else val - 65536
    if byte_size == 4:
        return val if val < 2147483648 else val - 4294967296
    return val


class GripperNode(Node):

    def __init__(self):
        super().__init__('gripper_node')

        # Tunable from outside without editing the file
        self.declare_parameter('open_ticks',  DEFAULT_OPEN_TICKS)
        self.declare_parameter('close_ticks', DEFAULT_CLOSE_TICKS)
        self.declare_parameter('profile_velocity', DEFAULT_PROFILE_VELOCITY)
        self.open_ticks  = int(self.get_parameter('open_ticks').value)
        self.close_ticks = int(self.get_parameter('close_ticks').value)
        prof_vel         = int(self.get_parameter('profile_velocity').value)

        self.port = PortHandler(DEVICE)
        self.pkt = PacketHandler(PROTOCOL)
        if not self.port.openPort() or not self.port.setBaudRate(BAUD):
            self.get_logger().fatal(f'cannot open {DEVICE}@{BAUD}')
            raise RuntimeError('open failed')

        # Extended Position mode (4) instead of plain Position mode (3):
        # mode 3 wraps at 4095 → 0 and may pick the short-path direction
        # for a 0→4095 command, which can rotate the wrong way. Mode 4
        # is multi-turn and always honors the signed delta direction.
        self.pkt.write1ByteTxRx(self.port, DXL_ID, ADDR_TORQUE_ENABLE, 0)
        self.pkt.write1ByteTxRx(self.port, DXL_ID, ADDR_OPERATING_MODE, 4)
        self.pkt.write4ByteTxRx(self.port, DXL_ID, ADDR_PROFILE_VELOCITY, prof_vel)
        self.pkt.write1ByteTxRx(self.port, DXL_ID, ADDR_TORQUE_ENABLE, 1)

        self.get_logger().info(
            f'XM430 ID={DXL_ID} ready on {DEVICE} '
            f'(extended-position mode, open={self.open_ticks}, close={self.close_ticks})')

        self.create_subscription(
            Float32, '/gripper_command', self._cmd_cb, 10)
        self.pub = self.create_publisher(JointState, '/gripper_state', 10)
        self.create_timer(0.05, self._tick)  # 20 Hz

    def _opening_to_ticks(self, opening_m: float) -> int:
        """Map opening width [0, MAX_OPENING_M] to goal position ticks."""
        opening_m = max(0.0, min(MAX_OPENING_M, opening_m))
        ratio = 1.0 - (opening_m / MAX_OPENING_M)   # 0 at open, 1 at close
        return int(round(
            self.open_ticks + ratio * (self.close_ticks - self.open_ticks)))

    def _set_goal(self, ticks: int):
        self.pkt.write4ByteTxRx(
            self.port, DXL_ID, ADDR_GOAL_POSITION, int(ticks))

    def _cmd_cb(self, msg: Float32):
        ticks = self._opening_to_ticks(msg.data)
        self._set_goal(ticks)
        self.get_logger().info(
            f'cmd {msg.data*1000:.1f}mm -> goal_position {ticks}')

    def _tick(self):
        try:
            load_raw = self.pkt.read2ByteTxRx(
                self.port, DXL_ID, ADDR_PRESENT_LOAD)[0]
            pos_raw = self.pkt.read4ByteTxRx(
                self.port, DXL_ID, ADDR_PRESENT_POSITION)[0]
            vel_raw = self.pkt.read4ByteTxRx(
                self.port, DXL_ID, ADDR_PRESENT_VELOCITY)[0]
        except Exception as e:
            self.get_logger().warn(f'sensor read failed: {e}')
            return

        torque_mnm = twos_complement(load_raw, 2) * UNIT_TO_MNM
        vel_rpm    = twos_complement(vel_raw, 4) * UNIT_TO_RPM

        # Map current motor position back to opening width (m). Uses the
        # same linear interpolation as _opening_to_ticks, inverted.
        span = self.close_ticks - self.open_ticks
        if span != 0:
            ratio = (pos_raw - self.open_ticks) / span
            ratio = max(0.0, min(1.0, ratio))
            opening_m = MAX_OPENING_M * (1.0 - ratio)
        else:
            opening_m = MAX_OPENING_M
        plate_pos = opening_m / 2.0
        plate_vel = (vel_rpm / 60.0) * MAX_OPENING_M

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['gripper_left_joint', 'gripper_right_joint']
        js.position = [plate_pos, plate_pos]
        js.velocity = [plate_vel, plate_vel]
        js.effort = [torque_mnm / 1000.0, torque_mnm / 1000.0]
        self.pub.publish(js)

    def destroy_node(self):
        try:
            self.pkt.write1ByteTxRx(
                self.port, DXL_ID, ADDR_TORQUE_ENABLE, 0)
            self.port.closePort()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
