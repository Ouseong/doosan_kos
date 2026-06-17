#!/usr/bin/env python3
"""Set the Doosan M1013 safety mode to RECOVERY (or back to AUTONOMOUS).

Usage:
    python3 recovery_mode.py                 # enter RECOVERY on sim (/dsr01)
    python3 recovery_mode.py --real          # enter RECOVERY on real (/dsr01_real)
    python3 recovery_mode.py --auto          # back to AUTONOMOUS
    python3 recovery_mode.py --real --auto   # back to AUTONOMOUS on real
    python3 recovery_mode.py --ns dsr01      # custom namespace

Run inside the doosan_kos container where dsr_msgs2 is sourced:
    docker exec -it doosan_kos bash -lc \\
        "source /opt/ros/humble/setup.bash && \\
         source /root/ros2_ws/install/setup.bash && \\
         python3 /workspace/scripts/recovery_mode.py --real"
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from dsr_msgs2.srv import SetSafetyMode, GetRobotState


# DRCF safety_mode enum (from doosan-robot2 dsr_common2/include/DRFL.h)
SAFETY_MODE_MANUAL     = 0
SAFETY_MODE_AUTONOMOUS = 1
SAFETY_MODE_RECOVERY   = 2
SAFETY_MODE_SAFE_OFF   = 3
SAFETY_MODE_SAFE_STOP  = 4

# safety_event: 0=NONE, 1=MOVE (apply now), 2=STOP
SAFETY_EVENT_MOVE = 1

ROBOT_STATE_NAMES = {
    0: "INITIALIZING", 1: "STANDBY",      2: "MOVING",       3: "SAFE_OFF",
    4: "TEACHING",     5: "SAFE_STOP",    6: "EMERGENCY_STOP", 7: "HOMING",
    8: "RECOVERY",     9: "SAFE_STOP2",  10: "SAFE_OFF2",
}


class RecoveryModeClient(Node):
    def __init__(self, namespace: str):
        super().__init__("recovery_mode_client")
        base = f"/{namespace}/dsr_controller2/system"
        self.cli_safety = self.create_client(SetSafetyMode, f"{base}/set_safety_mode")
        self.cli_state  = self.create_client(GetRobotState, f"{base}/get_robot_state")
        self.namespace  = namespace

    def _wait_future(self, fut, timeout: float):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def query_state(self) -> str:
        if not self.cli_state.wait_for_service(timeout_sec=2.0):
            return "service unavailable"
        resp = self._wait_future(self.cli_state.call_async(GetRobotState.Request()), 2.0)
        if resp is None:
            return "no response"
        code = int(resp.robot_state)
        return f"{ROBOT_STATE_NAMES.get(code, '?')} ({code})"

    def set_safety_mode(self, mode: int) -> bool:
        if not self.cli_safety.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                f"set_safety_mode service not found on /{self.namespace}. "
                f"Is the driver running?"
            )
            return False
        req = SetSafetyMode.Request()
        req.safety_mode  = int(mode)
        req.safety_event = SAFETY_EVENT_MOVE
        resp = self._wait_future(self.cli_safety.call_async(req), 3.0)
        if resp is None:
            self.get_logger().error("set_safety_mode timed out")
            return False
        ok = bool(getattr(resp, "success", True))
        if not ok:
            self.get_logger().error("driver returned success=False")
        return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--real", action="store_true",
                        help="target real driver namespace (/dsr01_real)")
    parser.add_argument("--auto", action="store_true",
                        help="restore AUTONOMOUS instead of entering RECOVERY")
    parser.add_argument("--ns", default=None,
                        help="explicit namespace (overrides --real)")
    args = parser.parse_args()

    namespace = args.ns or ("dsr01_real" if args.real else "dsr01")
    target_mode = SAFETY_MODE_AUTONOMOUS if args.auto else SAFETY_MODE_RECOVERY
    target_name = "AUTONOMOUS" if args.auto else "RECOVERY"

    rclpy.init()
    node = RecoveryModeClient(namespace)
    try:
        before = node.query_state()
        print(f"[/{namespace}] robot_state before: {before}")

        print(f"[/{namespace}] requesting safety_mode -> {target_name} ({target_mode})...")
        ok = node.set_safety_mode(target_mode)
        if not ok:
            print(f"[/{namespace}] FAILED to set {target_name}")
            sys.exit(2)

        time.sleep(0.5)
        after = node.query_state()
        print(f"[/{namespace}] robot_state after:  {after}")
        print(f"[/{namespace}] OK — safety_mode is now {target_name}.")

        if target_mode == SAFETY_MODE_RECOVERY:
            print()
            print("Robot is in RECOVERY. You can now move it out of singular/collision pose:")
            print("  - jog with move_joint at low velocity")
            print("  - DRCF caps motion speed in RECOVERY regardless of vel/acc args")
            print("When done, restore normal operation with:")
            print(f"  python3 {sys.argv[0]} {'--real ' if args.real else ''}--auto")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
