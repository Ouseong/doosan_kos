# Real Dynamixel Gripper + Real-Driver Console Workflow

Summary of the changes made on 2026-05-13 that bring up the real
XM430-based pin-array gripper end-to-end, harden the real-robot driver
flow in `control_console.py`, and align Isaac Sim's gripper animation
to the actual motor speed.

---

## 1. control_console.py — real driver workflow

The previous `start_real_driver` flow had three latent failure modes that
manifested as `console says "connected" but the robot won't move`.

### Patch a — re-entry lock

Rapid double-clicks on **Start Real Driver** used to race past the
idempotence guard and spawn duplicate `ros2 launch` instances that
fought for DRCF's TCP port, leaving zombie `ros2_control_node`s.
Now wrapped in a `self._real_starting` lock; concurrent calls log
`already in progress, ignoring duplicate request` and return immediately.

### Patch b — drop the DRCF TCP probe

The old idempotence guard did a fresh TCP connect to `192.168.137.100:12345`
to decide whether to attach to an existing driver. DRCF doesn't always
accept a second client during an active driver session, so this probe
falsely failed and triggered a duplicate launch that then collided on
the same port. The guard now relies on ROS service-name registration
alone, which is the correct primitive. The same TCP probe was removed
from `_poll_real_driver` for consistency.

### Patch c — Start Real Driver always means "fresh boot, servo on"

After a demo trips a soft fault, DRCF drops to `STATE_SAFE_OFF` and the
driver doesn't auto-retry `CONTROL_SERVO_ON` (it only does that during
its init phase). Idempotent attach silently left the user stuck with
a live ROS connection to a robot that refused all motion commands.

`_start_real_driver_impl` now:

1. SIGTERMs any existing `/dsr01_real` driver, waits 3 s, SIGKILLs anything
   still alive, waits 2 s for TCP cleanup.
2. Launches a fresh `ros2 launch dsr_bringup2 ... name:=dsr01_real mode:=real`.
3. Polls `/tmp/real_driver.log` for the literal substring **`STATE_STANDBY`**
   (not "Connected to DRCF" — that fires before servo_on succeeds).
4. Returns `True` only when servo is actually on (you hear the motor torque
   click) and motion commands will be honored.

Click cost is ~15 s but the user always gets a guaranteed STANDBY robot
or a clear failure. Don't add an idempotent fast-path back.

### Patch d — state-aware UI + auto-recover

The Connection Settings dialog now calls `query_real_robot_state` and
shows the actual DRCF state:

| robot_state code | UI label |
|---|---|
| 1/2/7 (STANDBY/MOVING/HOMING) | `ready - STANDBY` |
| 3 (SAFE_OFF) | `connected, SERVO OFF — press 'Start Real Driver' to recover` |
| 6 (EMERGENCY_STOP) | `EMERGENCY_STOP — clear alarm on pendant first` |
| 5/9 (SAFE_STOP variants) | `<name> — clear on pendant` |

`recover_real()` adds a real-namespace variant of the existing `recover()`
helper: cycles `set_safety_mode` RECOVERY→NORMAL and then asserts
`set_robot_mode AUTONOMOUS`. This is what the bare ROS calls do that we
verified by hand recover a SAFE_OFF robot.

`apply_and_close` now re-queries state right before committing a mode
switch to `real`/`preview`, so the user can't drive commands at a robot
that drifted into SAFE_OFF after the dialog was first opened.

### Patch e — Korean → English UI strings

The status labels, modals, and demo-status text were Korean. The
container ships without a CJK font (`fonts-nanum`/`noto-cjk` are not
installed), so anything Korean rendered as tofu boxes. Strings the
operator sees are now English.

---

## 2. Dynamixel_Control/gripper_node_v2.py — real gripper ROS2 node

New file. ROS2 node that subscribes `/gripper_command` (Float32 m,
0.0~0.067) and drives the user's XM430-W350 over `/dev/ttyUSB0`. Topic
contract is identical to the Isaac Sim bridge so `control_console.py`
publishes once and both react.

### Hardware

- USB adapter: FTDI FT232H (`0403:6014`, serial FTAO51QS) — passed through
  to the `doosan_kos` container via `/dev/ttyUSB0`, no extra docker args
  needed. Note this is **not** a ROBOTIS U2D2 (which uses FT232RL); it
  was working as a serial transport but isn't a name-brand adapter.
- Motor: XM430-W350, model_number 1130, ID 3, baudrate 57600, Protocol 2.0.
- Container needs `dynamixel-sdk` (Python). Already installed system-wide
  via `pip install --break-system-packages` (root). Survives container
  restart, not image rebuild.

### Calibration (measured with Dynamixel Wizard)

| | Position |
|---|---|
| Fully open  | **0°** (motor home) → 0 ticks |
| Fully close | **−175°** (mechanical hard stop) → −1991 ticks |
| Direction   | Closing is **NEGATIVE** ticks (CW from user's frame) |

Extended Position Mode (`operating_mode = 4`) is mandatory because the
single-revolution Position mode (3) chooses short-path direction and
will rotate the wrong way for a 0 → −1991 command.

`profile_velocity = 100` (≈ 22.9 RPM) gives a 1.27 s nominal traversal;
measured at 1.46 s end-to-end. Lower values let mechanism friction
stall the motor before completing the stroke — don't drop below ~80.

### How to run

```bash
docker exec -d doosan_kos bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  python3 /kos_workspace/Dynamixel_Control/gripper_node_v2.py > /tmp/gripper_node.log 2>&1
'
```

The control console's gripper panel (Open / Close / slider) drives both
the sim bridge and this node from the same `/gripper_command` publish.

### Troubleshooting

- `OSError: [Errno 16] Device or resource busy` on startup → the
  Dynamixel Wizard or another script holds `/dev/ttyUSB0`. Disconnect it
  in the wizard before starting the node.
- `hw_error_status = 32` (Overload) → motor hit a mechanical hard-stop.
  Reboot the motor (`pkt.reboot(port, dxl_id)`), then retry; if it
  repeats on the same target, that target is past the real range.

---

## 3. m1013_gripper_bridge.py — sim animation speed match

The bridge previously called `set_joint_positions` with the raw target
opening every frame, so the sim gripper teleported to the new opening
within one physics step — visually instant, nothing like the real motor.

New behavior:

- `self.current_opening` ramps toward `self.target_opening` at a constant
  cap `GRIPPER_OPENING_SPEED` (m/s) each `LOOP_DT` (1/60 s).
- The bridge's `set_joint_positions` now uses `current_opening`, so the
  visible motion matches the hardware's traversal time.

`GRIPPER_OPENING_SPEED = 0.075 m/s` was settled by visual comparison
against the real motor. Direct timing of the motor itself (open ↔ close,
67 mm, profile_velocity=100) gives 1.458 s = 0.046 m/s, but a fraction
of that is end-of-motion settle the operator can't see, so 0.075 m/s
lines up better with what the eye perceives.

If you change `profile_velocity` on the motor, retune this constant.

---

## 4. hammer_demo.py — pin-array over-travel

The user's gripper is a **pin-array gripper** — closing past the
handle, the pins retract locally so the plates over-travel into the
handle. The demo previously stopped close motion at 20 mm to avoid
clashing the plates with the handle (treating the gripper as a rigid
two-plate device).

Now: `grip(0.0, "CLOSE + auto-grasp")`. With pin-array hardware the
plates close fully on the real device, and the bridge's gripper
collisions are already disabled (`그리퍼 collision 비활성: 3 prims`
in bridge boot log), so sim plates visibly pass through the handle —
which is the correct pin-array visual.

The bridge's `HAMMER_ATTACH_OPENING_M = 0.04` trigger still fires well
before `0`, so auto-grasp behavior is unchanged.

---

## Typical session (Spark side)

1. `docker/container.sh start`
2. `scripts/_ensure_stack.sh` (or manually: run emulator, virtual driver,
   bridge, telemetry, console — see `restart_isaac.sh`)
3. In Connection Settings: **Start Real Driver** → wait for `ready - STANDBY`
4. Start the gripper node (one-liner above) — once per session
5. Toggle target mode → **REAL** → Apply
6. Console Open / Close drives both sim and real gripper at matched speed
