# ServojStream Investigation (unresolved)

Why the iLQR swing → servoj_stream path didn't work on 2026-05-14.
Documented for next-session pickup so we don't redo the same failed
experiments. The MoveSplineJoint path (current demo default) is fine —
this doc is only about the ServojStream alternative.

The intent was: take the iLQR-planned per-step joint angles (CSV, dt
= 10 ms) and publish them over `/dsr01_real/dsr_controller2/servoj_stream`
at 100 Hz, so the robot tracks the exact iLQR plan without DRCF spline
interpolation. This kept tripping driver alarms and pushing the robot
into `STATE_SAFE_OFF`.

---

## Attempts and why each failed

### 1. `rt_streamer` (C++ DRFL `servoj_rt`) — hung at `connect_rt_control`

`scripts/rt_streamer.cpp` already existed and uses DRFL's RT channel
(port 12347) directly. We tried it standalone:

```
[1] open_connection(.100:12345)            OK
[2] manage_access_control(FORCE_REQUEST)   OK
[3] CONTROL_SERVO_ON + wait STATE_STANDBY  OK
[4] set_safety_mode(AUTONOMOUS, EVENT_MOVE) OK
[5] connect_rt_control(.100:12347)         HUNG indefinitely
```

The streamer comment says `Pre-req: dsr_hw_interface2 / dsr_bringup2
NOT running (DRCF allows one client)`. We tried SIGSTOPing the ros2
driver while rt_streamer ran, but `connect_rt_control` still hung.
RT channel setup in this environment didn't complete — likely needs
more than the SIGSTOP trick (UDP route, separate cleanup, etc).

Abandoned in favour of the topic-based ServojStream approach, which
doesn't need to detach the ros2 driver.

### 2. ServojStream, first publish — gear grinding on the real robot

Initial publish loop in `move_swing_servoj_stream`:

```python
msg.vel = [0.0] * 6
msg.acc = [0.0] * 6
msg.time = total_time_s / (n - 1)   # = 15 ms
msg.mode = 0   # DR_SERVO_OVERRIDE
```

Three wrong assumptions:
- **`vel = [0]*6` is not "no limit"** — DRFL appears to read it as a
  literal zero velocity target. The robot accelerates toward each new
  position but the controller simultaneously forces the velocity back
  to 0 → mechanical conflict → audible gear grinding.
- **dt = 15 ms mismatches the CSV's 10 ms** — iLQR sized the per-step
  joint displacement against a 10 ms tick; stretching to 15 ms means
  the controller's planner expects a velocity profile that doesn't
  match what we asked for.
- **mode 0 (DR_SERVO_OVERRIDE)** — every 10 ms message replaces the
  previous target instantly. While accelerating from one target the
  next override arrives, jerking the joint, so the grinding stress
  compounds.

User hit E-stop. After fixing this we never reproduced grinding
audibly, but later attempts still failed in software.

### 3. ServojStream with sane parameters — J3 exceeded velocity limit

Replaced the assumptions:

```python
msg.vel = [120, 120, 180, 200, 200, 200]   # M1013 spec ceiling
msg.acc = [vᵢ * 3 for vᵢ in msg.vel]
msg.time = 0.01                             # CSV native
msg.mode = 1                                # DR_SERVO_QUEUE
```

Alarm immediately on first swing:
```
[OnLogAlarm] level=3, index=1908
desired velocity J3 = -346.762 deg/s, limit 180.000 deg/s
```

The **1-second iLQR swing fundamentally exceeds J3's velocity limit**
in its mid-swing portion. MoveSplineJoint papered over this because the
DRCF planner smooths corners and lowers peaks; ServojStream sends the
raw per-step targets, so the controller sees the actual velocity demand
the iLQR put in.

Confirmed numerically with `swing_traj.csv` peak velocities:
```
J1 0   J2 166   J3 163   J4 0   J5 282   J6 0
              ↑                    ↑
       under limit (180)    OVER limit (225)
```

J5 peak 282 deg/s > 225 deg/s spec.

### 4. Switched to a 5 s OCP CSV — swing shape was wrong

Regenerated the trajectory with `swing_optimal.py` at:
```
T = 200       (1 s -> 2 s)         actually became 5 s in another path
V_DES_DOWN = 1.5  (3.0 -> 1.5 m/s downward impact)
W_VEL_BARRIER = 1e6  (5e3 -> 1e6, closer to hard constraint)
JOINT_VEL_LIMIT_DEG = [90, 90, 140, 170, 170, 170]   (75 % of spec)
```

The new `swing_traj_ocp.csv` (501 rows × 10 ms = 5 s) put all joint
peaks well under spec. Run in sim — user judged the swing shape itself
to be wrong (not the familiar swing arc), so we deleted that CSV.

### 5. Stayed with 1 s shape, lowered impact (impact 1.5 m/s, 2 s swing) — J6 velocity blew up before swing started

Re-ran swing_optimal.py with the milder settings above. CSV came back
with every joint within spec (J3 peak 76 %, J5 peak 69 %). Started the
demo and got:
```
[OnLogAlarm] level=3, index=1908
desired velocity J6 = 3949.184 deg/s, limit 225 deg/s
param = 6                       # J6
```

J6 is **locked** in the iLQR cost (`W_LOCK_JOINT = 1e5`), so the CSV's
J6 column varies by 0.0001°. The 3949 deg/s spike was the **BACKSWING
movej**, not servoj_stream — a `movej(J6 = -92.76°)` call right before
the swing.

Hypothesis: APPROACH → AT_HAMMER → LIFT use Cartesian `movejx`, and
the IK behind that resolves J6 onto a multi-turn branch hundreds of
degrees away from -92.76°. The follow-up `movej` then plans a long-way
rotation, demanding the impossible velocity, alarm, SAFE_OFF.

Verified: after BACKSWING aborted, sim joint position read
`J1 = -134°, J6 = -134°` — the old PRE_ALIGN target, not the iLQR
backswing.

### 6. PRE_ALIGN aligned to iLQR `q₀`, J1/J6 column shifted — `acc = NaN`

Tried two patches together:
- Change PRE_ALIGN target so J1, J6 land on iLQR backswing values
  (`-148°`, `-92.76°`) before APPROACH starts.
- After BACKSWING, shift the CSV's J1 / J6 columns so the swing's
  first row matches the robot's actual J1 / J6, since both are locked
  in the iLQR plan anyway.

Demo result:
```
[OnLogAlarm] level=3, index=1908
desired acceleration = -nan deg/s²
param = 6
```

`NaN` acceleration usually means a planner internal divide-by-zero —
the shift made `backswing_target == robot_pose` for J1 / J6, which
combined with whatever delta remained on J2 / J3 / J5 produced a
degenerate motion profile.

User reported the robot "spinning wildly" during PRE_ALIGN itself with
this patch, presumably because the new PRE_ALIGN target (J1 = -148°)
was a much larger rotation from home than the old (J1 = -134°).

Reverted everything via `git restore` and went back to the spline demo
to keep the robot safe.

---

## Open questions for next session

Before retrying, we need to answer these. Most of them are blocked on
not having looked at `doosan-robot2` driver source yet.

1. **What do `msg.vel` and `msg.acc` mean to the driver?** Are they
   per-step velocity targets, per-step ceilings, or DRFL `servoj()`
   limits? `0` was wrong, but is `[225]*6` "no limit" or "go at 225
   deg/s every step"?
2. **`mode = 0` vs `mode = 1`** — what is the driver-internal queue
   depth, and what happens when the queue overflows? Does QUEUE block
   the publisher or drop?
3. **Why did the multi-turn J6 from `movejx` not get re-normalised?**
   Is there a driver flag or service that wraps joint targets into
   the nearest-2π branch?
4. **Is there a sim-only path that doesn't go through the emulator
   container?** Reproducing the BACKSWING wrap-around without the
   real robot would be much faster to iterate on.

## What to do first next session

1. **Patch `control_console.py` to redirect the hammer_demo subprocess
   stdout to `/tmp/hammer_demo.log`**. Right now console reads the
   subprocess pipe and only echoes a one-line status, so our debug
   prints inside `hammer_demo.py` are invisible. Without seeing what
   `capture_joints()` reports for J6 after LIFT, every fix above is a
   guess.
2. Read `doosan-robot2/src/.../servoj_stream` subscriber callback to
   pin down (1) and (2).
3. Then retry with eyes open.

## Files that the failed attempts touched

All reverted now. Listed for archaeology:
- `scripts/hammer_demo.py` — added `move_swing_servoj_stream`,
  `move_swing_movej`, PRE_ALIGN edit, J1/J6 shift, BACKSWING debug.
- `scripts/ilqr/swing_optimal.py` — path hardcode + tuning constants
  (T, V_DES_DOWN, W_VEL_BARRIER, JOINT_VEL_LIMIT_DEG).
- `output/swing_traj_ocp.csv` — created and deleted.
- `output/swing_traj.csv` — overwritten by the 201-row OCP rerun,
  then restored from `swing_traj_optimal.csv`.

## What we kept

- Installed `crocoddyl + pinocchio` into `~/.local/lib/python3.13/`
  (miniconda Python 3.13). Survives reboots; lets us re-run
  swing_optimal.py without setting up a venv next time.
- Deleted `scripts/ilqr/swing_cubic.py`,
  `output/swing_traj_cubic.csv`, `output/swing_traj_ocp.csv` per user
  request. `swing_optimal.py` still has its own internal
  `cubic_hermite_warm` used as the FDDP warm start, so removing the
  standalone cubic file is safe.
