# DRCF Timing: Emulator vs Real, and the servoj_rt Bypass

Date: 2026-05-06
Hardware tested: x86_64 server (3× A6000) + Doosan M1013 at 192.168.137.100

## TL;DR

- Same DRCF firmware code, different runtime → very different motion timing.
- `MoveSplineJoint` ignores `req.time` on **both** emulator and real robot.
- Emulator: **22 s** per 21-pt strike. Real: **7.77 s** per 21-pt strike. Both >> requested 1.5 s.
- `servoj_rt` streaming bypasses the firmware planner; **verified working** today with the right setup sequence.
- Critical pitfall: position deviation at end of streaming triggers firmware emergency stop.
- `dsr_emulator` is **amd64-only** — but the slowness is firmware logic, not architecture: Spark/QEMU is not the cause.

---

## Part 1: Why MoveSplineJoint is slow

### Measurements

Same iLQR swing (101 rows × dt=10 ms, sub-sampled to 21 waypoints), `req.time = 1.5 s`, `vel_lim = [120,120,180,200,200,200]°/s`:

| Backend | Plan stage | Exec stage | Total | Peak vel observed |
|---|---|---|---|---|
| dsr_emulator (amd64 Docker on x86 host, no QEMU) | ~10.3 s | ~11.7 s | **22.05 s** | unknown |
| Real M1013 (RTOS controller box) | < 1 s (est.) | 7.77 s | **7.77 s** | 21°/s |

`req.time` is **completely ignored** — emulator gives identical 22 s for `req.time` 1.5/3/5 s. Pacing is linear in waypoint count: ~0.55 s/pt on real, ~1.05 s/pt on emulator.

Async (`amovesj`, sync_type=1) on emulator returns in 10.3 s — that's the plan-stage time; rest is exec.

### Why the difference

DRCF firmware is the same C code. Runtime environment differs:

- **Emulator** runs DRCF on general Linux (Docker). Plan stage uses wallclock pacing — likely deliberate, for deterministic test reproducibility.
- **Real** robot runs DRCF on dedicated RTOS controller with 1 ms cycle. Plan completes near-instantly. But exec still has per-segment pacing (~0.37 s/pt), suggesting an internal trapezoidal-profile generator runs *per spline segment* regardless of `req.time`.

Peak velocity on real = 21°/s, only **17 %** of J2's 120°/s max — firmware pads each segment with a conservative profile.

### User-perceived behavior

Matches measurements:
- Emulator: long visible pause (≈ plan stage 10 s) + slow swing (≈ exec stage 12 s) → "wait time + slow swing"
- Real: no pause + slightly slower-than-natural swing → "no wait, but strike feels similar duration"

### Implication

Even on real M1013, `MoveSplineJoint` cannot deliver the iLQR-optimized 1.0 s strike with 1.5 m/s impact velocity. **The planner is the floor; we have to bypass it.**

---

## Part 2: servoj_rt — bypassing the planner

### Concept

```
ROS service MoveSplineJoint  →  DRCF planner (slow)  →  servo loop
                  vs
direct DRFL servoj_rt          →  servo loop (no planner)
```

We stream individual joint targets at high rate; firmware just runs servo, no per-segment planning.

### Required setup sequence (this is what failed on Spark before)

Skipping any of these makes the robot silently ignore commands. Verified working today:

```cpp
drfl.open_connection("192.168.137.100", 12345);
drfl.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

// poll get_robot_state() until STATE_STANDBY (else firmware rejects)
for (retry < 10):
    if state == STATE_STANDBY: break
    drfl.set_robot_control(CONTROL_SERVO_ON)
    sleep 800ms

drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);  // ⚠ MOST OFTEN MISSED
drfl.connect_rt_control("192.168.137.100", 12347);
drfl.set_rt_control_output("v1.0", 0.001, 4);
drfl.start_rt_control();
drfl.set_velj_rt(vel_lim);   // float[6] in deg/s
drfl.set_accj_rt(acc_lim);   // float[6] in deg/s²

// stream loop
for sample in trajectory:
    drfl.servoj_rt(sample_pos_deg, zero_vel, zero_acc, servo_time);
    // pace to match streaming rate

drfl.stop_rt_control();
drfl.disconnect_rt_control();
drfl.close_connection();
```

Notes:
- `pos` is in **degrees**, not radians (DRFL convention).
- The ROS driver `dsr_hardware2` already holds the DRCF connection — kill it first. Single client per DRCF.
- `set_safety_mode(AUTONOMOUS, EVENT_MOVE)` is what `dsr_hw_interface2.cpp` does on idle→commanding transition. Without it, firmware silently no-ops servoj_rt calls (return value still `true`).

### What works (verified today on this server)

- Setup sequence above runs without errors.
- `servoj_rt` returned `true` for all 250 samples (J6 ±10° sine over 2 s, 100 Hz).
- Robot physically moved — final J6 displacement +3.5 ° from start (visible on hardware).
- "Spark에서 실패" 추정 원인: `set_safety_mode(AUTONOMOUS, EVENT_MOVE)` 또는 `set_robot_control(CONTROL_SERVO_ON)` 호출 누락.

### Pitfalls discovered

1. **Streaming period vs `servo_time` mismatch**
   - Initial test: 100 Hz period (10 ms) + `servo_time = 0.1 s` → robot tracked only ~3.5° amplitude on a ±10° command.
   - When `period < servo_time`, each new command interrupts the smoothing window of the previous → output is heavily attenuated.
   - Fix: use `period ≥ servo_time`, e.g. `100 Hz / servo_time=0.01` or `50 Hz / servo_time=0.02`.

2. **End-of-stream position error → firmware emergency stop ("쾅")**
   - After streaming sine, robot ended at +7.1° while last commanded was +3.5° (3.6° tracking lag).
   - Firmware detected the deviation as a safety event and engaged motor brakes — audible bang.
   - Mitigations (TODO in code):
     - Hold final commanded pose for ≥1 s before `stop_rt_control`.
     - Ramp vel feedforward to zero in last few samples.
     - Read actual pose via `get_current_posj()` before stop, verify error within tolerance.

3. **Restarting `dsr_bringup2` right after `rt_streamer` is delicate**
   - `ros2_control` framework's command interface initializes to 0 — but actual init sequence has not been fully traced. Verify carefully before re-enabling driver after RT streaming.

### What's still TBD

- **Tracking accuracy**: only end-pose was logged, not continuous trajectory. Step-input test (50 samples at start, 50 at +10°, 50 at start) would give clean reachability data.
- **`servo_time` tuning**: try 0.005 / 0.01 with 100/200 Hz streaming.
- **iLQR CSV adapter**: `--swing` mode in `rt_streamer.cpp` reads `output/swing_traj.csv` and prepends a 3 s smoothstep warmup from current pose to backswing pose, then plays the 1 s swing. **Not yet executed** (test was rejected after the "쾅" incident; needs guards from pitfall #2 first).

---

## Architecture map

```
                     User
                       │
                       ▼
        control_console.py / hammer_demo.py
                       │   (ROS service: /dsr01_real/dsr_controller2/motion/...)
                       ▼
              dsr_controller2 (callback)
                       │
                       ▼
                  Drfl.movej / Drfl.movesj   ← MoveSplineJoint path (slow)
                       │
                       ▼
                     DRCF
                       │
                       ▼
                   robot motor

         ────────────────────────────────────

                     User
                       │
                       ▼
                 rt_streamer (C++)         ← bypass path (today's work)
                       │
                       ▼
                  Drfl.servoj_rt             at 100 Hz, planner skipped
                       │
                       ▼
                     DRCF
                       │
                       ▼
                   robot motor
```

---

## Files

- `scripts/rt_streamer.cpp` — direct DRFL streamer; modes `--test` (J6 wiggle) / `--swing <csv>` (iLQR CSV).
- `scripts/build_rt_streamer.sh` — compile via `dsr_common2`'s `libDRFL.a` (x86_64 + arm64 both shipped).
- `output/swing_traj.csv` — iLQR-optimized 1 s swing (101 rows × 6 joints, deg).
- `scripts/hammer_demo.py` — patched: `DSR_NS` env var (default `dsr01`, set to `dsr01_real` for real-robot mode).
- `scripts/control_console.py` — patched:
  - `DEFAULT_REAL_IP` corrected to `192.168.137.100`.
  - `start_real_driver()` made idempotent (attaches to external driver if `/dsr01_real` services already exist, instead of spawning a duplicate).
  - Hammer-demo button passes `DSR_NS` based on current `target_mode`.

---

## Next steps (Spark / home)

1. **Reproduce setup sequence on Spark** (aarch64 + JetPack 7). `libDRFL.a` arm64 build is in `dsr_common2/lib/jazzy/arm64/`. No QEMU needed for the streamer itself.
2. **Step-input tracking test** to characterize true tracking accuracy at various `(period, servo_time)` pairs.
3. **Add end-of-stream guards** to `rt_streamer.cpp`: hold final pose, verify deviation, smooth shutdown.
4. **iLQR CSV → 1 ms upsampled stream** for the actual 1 s impact swing. Stretch goal: match the iLQR-designed 1.5 m/s impact velocity.
5. **Compare with `doosanrobotics_cumotion_driver`** JointTrajectoryAction path. Hypothesis: cumotion's MoveIt → JointTrajectory → ros2_control → `servoj_rt` chain has the same advantage but uses standard ROS interfaces.
6. **`dsr_emulator` is amd64-only** — confirmed via Docker manifest inspect. On Spark this runs under QEMU. But this is **not** the dominant slowness factor; firmware planner pacing is.
