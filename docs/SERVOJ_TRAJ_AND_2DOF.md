# servoj-ready trajectory + 2-DOF hammering implementation

Date: 2026-05-17
Status: 시뮬 검증 완료 (Isaac Sim emulator), 내일 lab 서버에서 servoj_rt 시도 예정

## 오늘 한 일 요약

### 1. servoj-ready CSV 생성기 (3-DOF, J2/J3/J5 자유)
`scripts/ilqr/swing_servoj.py` — `swing_optimal.py` 기반, servoj_rt 스트리밍용 13열 CSV.

핵심 변경 vs `swing_optimal.py`:
- **13열 저장**: `t, J1..J6, J̇1..J̇6` — servoj_rt 의 (pos, vel feedforward) 인자에 직접 매핑
- **임팩트 속도 1.5 m/s** (V_DES_DOWN, 이전 3.0 → 절반). M1013 스펙 안에서 swing 가능
- **vel barrier 강화** (W_VEL_BARRIER 5e3 → 5e4)
- **자동 T 조정 루프**: vel peak 가 SPEC (J3=180, J5=225 deg/s) 안에 들어올 때까지 horizon 늘림
- **J5 backswing offset 뒤집힘** (+60° → −120°): 망치를 그리퍼 위로 들어올림 → 진짜 "위에서 내려치기"
  - 이전 `swing_optimal.py` 의 J5 offset +60° 는 망치를 그리퍼 아래로 늘어뜨려 "끌어오는" swing 이었음
  - −120° offset 으로 q₀.J5 = −67° → hammer z=1.01m (그리퍼 위 16cm) → 자연스러운 백스윙

산출: `output/swing_traj_servoj.csv` (101 row × 13 col, dt=10ms, total=1.0s)

검증 결과:
- 임팩트 위치 오차 0.6mm (못 머리에 정확히 닿음)
- 임팩트 속도 1.50 m/s (목표 정확히)
- vel peak: J2 102 (85%), J3 102 (57%), J5 191 (85% spec) — 모두 안전
- 운동량 0.75 kg·m/s

### 2. 2-DOF 변형 (J3+J5 만, J2 락)
`scripts/ilqr/swing_servoj_2dof.py` — J2 어깨 고정한 손목+팔꿈치 swing.

차이점:
- **J2 lock** (iLQR cost: `state_w_q[1] = W_LOCK_JOINT`)
- **J2 velocity soft lock** (W_VEL_LOCK_SOFT = 1e2, 1e5 는 발산)
  - 강하게 락하면 J3/J5 와 강하게 coupled 인 J2 의 reaction transient 가 솔버 발산시킴
  - soft lock 결과 J2 가 82.5 deg/s 까지 자연스럽게 진동 (스펙 120 안)
- **새 IK refinement 함수** `refine_ik_j2lock`: J2 강제 고정 후 J3/J5 만으로 못 위치 다시 매칭
- **J3 offset -40°** (이전 +20° 는 망치를 안쪽으로 끌어와 z 낮춤; 음수 = 팔꿈치 펴짐 = 망치 위로)
- **J5 offset -120°** (반대 분기)
- **CLI 인자 `--j2`** 로 J2 lock 값 임의 지정 가능

산출: `output/swing_traj_servoj_2dof_J2+26.csv` (101 row × 13 col)

검증 결과:
- 임팩트 오차 6.5mm, 속도 1.49 m/s, 운동량 0.75 kg·m/s
- vel peak: J3 153 (85%), J5 191 (85%), J2 82.5 (69%, transient)
- 백스윙 자세: J3=61.30°, J5=-67° → 망치 z=1.03m (못 위 93cm)

### 3. hammer_demo 2-DOF 변형
`scripts/hammer_demo_2dof.py` — `hammer_demo.py` 기반.

- **새 CSV 사용**: `output/swing_traj_servoj_2dof_J2+26.csv`
- **13열 CSV loader**: header 첫 컬럼 `t` 감지 → 컬럼 2-7 추출
- **DEBUG_DUMP_PATH** 별도 (`/tmp/hammer_demo_2dof_sent.jsonl`)

기존 hammer_demo 와 동일하게:
- PRE_ALIGN → APPROACH → AT_HAMMER → CLOSE+auto-grasp → LIFT → BACKSWING → SWING × 3
- swing 단계는 여전히 **MoveSplineJoint** (21pt 다운샘플, 1.5s) — servoj_rt 아님

### 4. control_console Hammer Demo 버튼 변경
`scripts/control_console.py:1695` — `hammer_demo.py` → `hammer_demo_2dof.py`

### 5. view_swing_2d 13-col 지원
`scripts/ilqr/view_swing_2d.py` — header 첫 컬럼 `t` 자동 감지 + `--csv` CLI 인자.

## Isaac Sim 검증

2026-05-17 22:04 / 23:10 두 번 데모 실행. control_console 버튼으로 호출.

각 run 마다 STRIKE 1/3, 2/3, 3/3 모두 21pt MoveSplineJoint 성공 (fallback 없음).

마지막 STRIKE 페이로드 검증:
- pos_cnt = 21
- vel = [120, 120, 180, 200, 200, 200] deg/s
- acc = [300, 300, 400, 500, 500, 500] deg/s²
- time = 1.5s, mode = 0
- q₀ (row 0): J1=-148.06, **J2=25.58 (락)**, J3=61.30, J5=-66.90, J6=-92.76 ✓
- q₁ (row 20): J1=-148.05, J2=26.41, J3=99.92, J5=54.67, J6=-92.76 ✓

J2 swing 내내 25.58° → 26.41° 변동 (락 의도대로). J3 38.6°, J5 121.6° 협응.

## 환경 셋업 (오늘 검증된 순서)

```bash
# /tmp/docker_xauth 함정 정리 (디렉토리 → file 변환)
sudo rm -rf /tmp/docker_xauth && sudo touch /tmp/docker_xauth && sudo chmod 666 /tmp/docker_xauth
xauth nlist $DISPLAY | sed 's/^..../ffff/' | xauth -f /tmp/docker_xauth nmerge -

# X server 접근 권한 (컨테이너 uid 1234 허용)
xhost +local:    # 또는 xhost + (덜 안전)

# 환경 시작
cd /home/kos/Desktop/Code/doosan_kos
bash docker/container.sh start
bash scripts/connect_virtual.sh      # emulator + ROS virtual driver
bash scripts/restart_isaac.sh        # Isaac Sim bridge

# bridge/console/telemetry 가 X 못 붙으면 DISPLAY 명시 재시작
docker exec doosan_kos pkill -9 -f m1013_gripper_bridge.py
docker exec -d -e "DISPLAY=:0" doosan_kos bash -lc '
  /isaac-sim/python.sh /kos_workspace/isaac/m1013_gripper_bridge.py > /tmp/bridge.log 2>&1
'
# (console, telemetry 도 -e DISPLAY=:0 추가 필요)
```

`restart_isaac.sh` 자체에 `DISPLAY` 전파 추가하면 좋겠음 (TODO).

## 내일 lab 서버에서 servoj_rt 시도 가이드

### 출발점

CSV 두 개 다 servoj_rt-ready:
- `output/swing_traj_servoj.csv` (3-DOF, J2 swing 포함, 큰 인간형 swing)
- `output/swing_traj_servoj_2dof_J2+26.csv` (2-DOF, J3+J5 만)

둘 다 13열 (t, q, q̇), vel SPEC 안, dt=10ms, 1.0s.

### 작업 우선순위

1. **`scripts/rt_streamer.cpp` 13열 CSV 지원 추가**
   - 현재 6열만 읽음 (`load_csv` 함수)
   - q̇ 컬럼을 `drfl.servoj_rt(pos, vel, acc, time)` 의 vel 인자에 전달
   - `set_velj_rt` / `set_accj_rt` 한계는 별도 (feedforward vs limit 분리)

2. **UDP 12347 `connect_rt_control` hang 디버그**
   - 2026-05-06 lab x86 에서는 작동, 2026-05-14 DGX Spark 환경에서는 hang
   - SIGSTOP ROS driver 만으로 부족
   - 후보: UDP route, container 네트워크, 이전 세션 stale 소켓
   - `tcpdump -i any port 12347` 으로 패킷 흐름 관찰

3. **iLQR 궤적 첫 시도 시 안전 가이드**
   - rt_streamer 의 `--test` 모드 (J6 ±10° sine) 먼저 검증
   - warmup 3s (smoothstep) + swing 1s + hold 0.5s (rt_streamer 가 이미 함)
   - end-of-stream 마지막 자세 1초 hold → emergency stop 회피

### 알려진 함정 (5/14 카탈로그)

- ❌ `msg.vel = [0]*6` literal target 으로 해석 → 기어 그라인딩
- ❌ `msg.time ≠ CSV dt` (15ms vs 10ms) → controller planner mismatch
- ❌ `movejx` 후 J6 multi-turn → 다음 `movej` 가 long-way 회전 → alarm
  - PRE_ALIGN 단계에서 J6 를 swing q₀ 값(-92.76°) 으로 사전 정렬 필요

### 메시지 페이로드 (servoj_rt 인자)

```cpp
drfl.servoj_rt(
    float pos[6],   // CSV 컬럼 2-7, deg (DRFL convention)
    float vel[6],   // CSV 컬럼 8-13, deg/s (feedforward — rt 버전은 0 OK)
    float acc[6],   // zero[6] 또는 q̈ (수치 미분)
    float time      // smoothing window, 보통 0.02 (=2× stream period 10ms)
);
```

세션당 1회만:
```cpp
drfl.set_velj_rt(vel_lim);  // 진짜 ceiling, [120, 120, 180, 225, 225, 225]
drfl.set_accj_rt(acc_lim);  // [300, 300, 400, 500, 500, 500]
```

## 관련 메모리 / 문서

- `docs/HAMMERING_2DOF_PLAN.md` — 2-DOF 설계 동기 (mechanical aligned posture)
- `docs/SERVOJ_STREAM_INVESTIGATION.md` — 5/14 실패 카탈로그
- `docs/RT_STREAMING_AND_TIMING.md` — lab 첫 시도 (5/6) + 타이밍 분석
- `scripts/rt_streamer.cpp` — C++ DRFL servoj_rt 클라이언트
- `scripts/build_rt_streamer.sh` — Spark/x86 빌드
