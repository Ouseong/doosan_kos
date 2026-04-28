# M1013 Real-Robot Telemetry Catalog

What real-time data the M1013 driver exposes (topics + services + RT mode).
Verified live on 2026-04-28 against the real robot at `192.168.137.100`.

Driver runs in `/dsr01_real` namespace (real robot) and `/dsr01` (sim/emulator).
Replace `/dsr01_real` with `/dsr01` in any path below to query the sim instead.

---

## 1. Topics (passive subscribe, ~14 Hz)

| Topic | Type | Notes |
|---|---|---|
| `/dsr01_real/joint_states` | `sensor_msgs/JointState` | position, velocity. **effort = NaN** (no torque on this channel) |
| `/dsr01_real/dynamic_joint_states` | `control_msgs/DynamicJointState` | per-hardware-interface (position, velocity only) |
| `/dsr01_real/error` | `dsr_msgs2/RobotError` | event-based (publishes only on error) |
| `/dsr01_real/robot_disconnection` | `dsr_msgs2/RobotDisconnection` | event-based |
| `/dsr01_real/dsr_controller2/torque_rt_stream` | `dsr_msgs2/TorqueRtStream` | RT command channel (publish to send torques) |

For continuous torque/force monitoring, plain topics are insufficient — use services or RT mode.

---

## 2. Services — `aux_control` (most useful for monitoring)

All under `/dsr01_real/dsr_controller2/aux_control/...`

### Forces / Torques
| Service | Type | Returns |
|---|---|---|
| `get_joint_torque` | `dsr_msgs2/srv/GetJointTorque` | `jts: float64[6]` [N·m] |
| `get_external_torque` | `GetExternalTorque` | `ext_torque: float64[6]` (collision / external load estimate) |
| `get_tool_force` | `GetToolForce(ref)` | `tool_force: float64[6]` ([Fx, Fy, Fz, Tx, Ty, Tz] at TCP, [N, N·m]) |
| `get_orientation_error` | — | orientation error |
| `force/get_workpiece_weight` | — | currently configured workpiece weight |

### Position / Pose / Velocity
- `get_current_posj` → 6 joint angles [deg]
- `get_current_posx(ref)` → TCP pose [mm, deg] in BASE / TOOL / WORLD frame
- `get_current_velj`, `get_current_velx` → joint / TCP velocity
- `get_desired_posj/posx/velj/velx` → controller-commanded targets (slightly differ from actual)
- `get_current_tool_flange_posx` → flange (pre-TCP) pose
- `get_current_rotm` → rotation matrix
- `get_current_solution_space`, `get_solution_space` → IK solution space (0–7)
- `get_robot_link_info`

---

## 3. Services — `system`

- `get_robot_state` → robot_state int:
  - `1 = STANDBY`, `2 = MOVING`, `3 = SAFE_OFF`, `5 = SAFE_STOP`,
  - `6, 7 = EMERGENCY_STOP`, `8 = HOMING`, `9 = RECOVERY`, `15 = NOT_READY`, …
- `get_robot_mode` → AUTONOMOUS / MANUAL
- `get_robot_speed_mode`
- `get_robot_system`
- `get_last_alarm`
- `get_current_pose`
- `set_safety_mode`, `servo_off`, `change_collision_sensitivity`

---

## 4. Services — IO

- `io/get_ctrl_box_digital_input` (16 bits), `get_ctrl_box_digital_output` (16 bits)
- `io/get_ctrl_box_analog_input`, `set_ctrl_box_analog_output`
- `io/get_tool_digital_input` (6 bits), `set_tool_digital_output`
- `modbus/*` — modbus master/slave registers
- `plc/*` — PLC input/output registers (bit / int / float)

---

## 5. Services — TCP / Tool

- `tcp/get_current_tcp`, `set_current_tcp`, `config_create_tcp`, `config_delete_tcp`
- `tool/get_current_tool`, `set_current_tool`, `set_tool_shape`

---

## 6. Services — DRL / Force control

- DRL: `drl/drl_start`, `drl_stop`, `drl_pause`, `drl_resume`, `get_drl_state`
- Compliance / Force: `force/task_compliance_ctrl`, `release_compliance_ctrl`, `set_desired_force`, `release_force`, `set_stiffnessx`
- Conditions: `force/check_force_condition`, `check_orientation_condition1/2`, `check_position_condition`
- Axes: `force/parallel_axis1/2`, `align_axis1/2`
- User coords: `force/set_user_cart_coord1/2/3`, `get_user_cart_coord`, `overwrite_user_cart_coord`, `coord_transform`, `calc_coord`
- Bolt: `force/is_done_bolt_tightening`

---

## 7. RT Mode — RobotStateRt (best for high-frequency telemetry, up to ~1 kHz)

Start RT control then `read_data_rt` returns a `RobotStateRt` snapshot:

```
realtime/connect_rt_control
realtime/start_rt_control
realtime/read_data_rt          ← read RobotStateRt
realtime/stop_rt_control
realtime/disconnect_rt_control
realtime/get_rt_control_input_data_list
realtime/get_rt_control_output_data_list
realtime/set_velj_rt / set_velx_rt / set_accj_rt / set_accx_rt
realtime/set_rt_control_input  / set_rt_control_output
realtime/write_data_rt
```

### `RobotStateRt.msg` — the richest single message available

```
time_stamp                           float64
actual_joint_position           [6]  motor-side encoder [deg]
actual_joint_position_abs       [6]  link-side absolute encoder [deg]
actual_joint_velocity           [6]  motor-side [deg/s]
actual_joint_velocity_abs       [6]  link-side [deg/s]
actual_tcp_position             [6]  base-frame Euler-ZYZ [mm, deg]
actual_tcp_velocity             [6]  [mm/s, deg/s]
actual_flange_position          [6]
actual_flange_velocity          [6]
actual_motor_torque             [6]  gear_ratio × current2torque × motor current [N·m]
actual_joint_torque             [6]  controller-estimated joint torque [N·m]
raw_joint_torque                [6]  calibrated JTS sensor [N·m]
raw_force_torque                [6]  calibrated FT sensor at flange [N, N·m]
external_joint_torque           [6]  estimated external joint torque [N·m]
external_tcp_force              [6]  estimated TCP external force [N, N·m]
target_joint_position           [6]  [deg]
target_joint_velocity           [6]  [deg/s]
target_joint_acceleration       [6]  [deg/s²]
target_motor_torque             [6]  [N·m]
target_tcp_position             [6]  [mm, deg]
```

This is the message to subscribe / poll for high-fidelity monitoring
(force-control, teaching, anomaly detection, dataset recording).

---

## 8. RobotState.msg (lower-frequency, big snapshot)

Defined but not auto-published in this build — must be assembled from services.
Fields include:

- `dynamic_tor`, `actual_jts`, `actual_ejt`, `actual_ett`
- `actual_bk` — brake state
- `actual_mc` — motor current
- `actual_mt` — motor temperature
- `ctrlbox_digital_input/output[16]`
- `flange_digital_input/output[6]`
- `modbus_state[]`
- `rotation_matrix`

If a single high-frequency monitoring topic is wanted, this is the
canonical structure to publish.

---

## 9. Sample readings at HOME_POSE (idle, gripper attached)

```
joint_torque    [N·m]: [-1.02, 43.52, 42.36, -4.01, -0.37, 0.61]
                       J2/J3 carry gravity load; rest near zero

external_torque [N·m]: [ 0.98, -7.12, -6.16, -0.32,  0.36, -0.61]
                       (idle, near zero)

tool_force      [N, N·m]: [-1.85, -0.57, -11.73, 0.39, 0.14, 0.62]
                       Fz ≈ -11.7 N → gripper weight ≈ 1.2 kg

joint_velocity  [°/s]: ~0
robot_state          : 1 (STATE_STANDBY)
```

---

## 10. Calling from shell (quick check)

Always source `dsr_msgs2` first:

```bash
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash

ros2 service call /dsr01_real/dsr_controller2/aux_control/get_joint_torque \
  dsr_msgs2/srv/GetJointTorque "{}"

ros2 service call /dsr01_real/dsr_controller2/aux_control/get_tool_force \
  dsr_msgs2/srv/GetToolForce "{ref: 0}"

ros2 service call /dsr01_real/dsr_controller2/system/get_robot_state \
  dsr_msgs2/srv/GetRobotState "{}"
```

In `rclpy`, lazy-create a client per service exactly like
`scripts/control_console.py` does for `movej`.
