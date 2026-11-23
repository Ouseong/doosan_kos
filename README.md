# DGX Spark × Doosan M1013
### Isaac Sim 5.1 · ROS2 Jazzy · DRCF 에뮬레이터 통합

DRCF 에뮬레이터 ↔ ROS2 driver ↔ Isaac Sim 을 실시간으로 연결해서 가상 로봇의 관절을 3D 로 시각화합니다.
NVIDIA DGX Spark (aarch64) 와 x86_64 Ubuntu 24.04 모두 지원.

![DGX Spark](https://img.shields.io/badge/DGX_Spark-aarch64-76B900) ![Isaac Sim](https://img.shields.io/badge/Isaac_Sim-5.1.0-green) ![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)

---

## 아키텍처

```
┌──────────────────────────── Host ────────────────────────────┐
│                                                               │
│   DRCF emulator (host, QEMU)              doosan_kos 컨테이너 │
│   ┌────────────────────┐                  ┌────────────────┐ │
│   │ doosanrobot/       │                  │ Isaac Sim 5.1  │ │
│   │ dsr_emulator:3.4.1 │◄────TCP :12345──►│ (base.kit)     │ │
│   │ (x86_64 binary)    │                  │                │ │
│   │                    │                  │ + ROS2 Jazzy   │ │
│   │ aarch64 에서는      │                  │ + doosan-robot2│ │
│   │ qemu-user-static   │                  │                │ │
│   │ 로 실행됨           │                  │ + bundled      │ │
│   └────────────────────┘                  │   rclpy (py3.11)│ │
│            ▲                              └────────────────┘ │
│            │                                      │           │
│            │                                      │           │
│         ROS2 driver (container)                   ▼           │
│         ros2_control_node → /dsr01/joint_states              │
│                                      │                        │
│                                      └──► Isaac Sim 구독      │
│                                            SingleArticulation│
│                                            → 3D 로봇 동기화   │
└───────────────────────────────────────────────────────────────┘
```

---

## 디렉토리 구조

```
doosan_kos/
├── docker/
│   ├── Dockerfile              Isaac Sim 5.1.0 + ROS2 Jazzy + Doosan deps
│   ├── bootstrap_ws.sh         doosan-robot2 clone + colcon build 자동화
│   ├── entrypoint.sh           컨테이너 시작 시 ROS2 env 세팅
│   ├── container.sh            start/enter/stop/clean 관리 스크립트 (--gpu N 지원)
│   ├── run_emulator.sh         DRCF 에뮬레이터 실행
│   └── register_qemu.sh        aarch64 에서 x86_64 DRCF 돌리기 위한 binfmt 등록
│
├── isaac/
│   ├── m1013_ros2_bridge.py    M1013 단독 브리지 (ROS2 joint_states → Isaac Sim)
│   ├── m1013_gripper_bridge.py ★ 통합 브리지 (M1013 + 그리퍼 동시 제어)
│   ├── create_gripper_usd.py   ★ 그리퍼 물리 USD 생성 (PrismaticJoint 포함)
│   ├── gripper_isaac_bridge.py 그리퍼 단독 Isaac Sim 브리지
│   │
│   ├── urdf_to_usd.py          URDF → USD 변환 (v1, 기본 옵션)
│   ├── urdf_to_usd_v2.py       URDF → USD 변환 (v2, merge_fixed_joints=True)
│   ├── assemble_m1013.py       M1013 USD 수동 조립 (base + physics sublayer)
│   │
│   ├── flatten_usd.py          USD 평탄화 (디버깅)
│   ├── flatten_deep.py         Dependencies 분석 + 평탄화 (디버깅)
│   └── inspect_usd.py          USD prim 구조 출력 (디버깅)
│
├── usd/
│   ├── full/
│   │   ├── gripper_full.stl            전체 조립 메쉬 (원본)
│   │   ├── gripper_full_urdf.stl       플랜지 원점 기준 변환 메쉬 (URDF용)
│   │   ├── gripper_full.urdf           단일 링크 URDF
│   │   └── urdf_preview.png            3방향 뷰 프리뷰 이미지
│   │
│   └── parts/
│       ├── gripper_body.stl/urdf       몸체 파트
│       ├── gripper_left.stl/urdf       왼쪽 플레이트 파트
│       ├── gripper_right.stl/urdf      오른쪽 플레이트 파트
│       ├── gripper_assembly.urdf       ★ 조립 URDF (PrismaticJoint + mimic)
│       ├── gripper_assembly_physics.usd ★ Isaac Sim 물리 USD (ArticulationRoot 포함)
│       └── gripper_parts_preview.png   파트별 프리뷰 이미지
│
├── Dynamixel_Control/
│   ├── gripper_node.py         ★ ROS2 노드 (/gripper_command 수신 → 모터 제어)
│   ├── gripper_test.py         수동 조그 테스트 (pygame GUI)
│   ├── original.py             원본 제어 코드
│   └── return_to_zero.py       영점 복귀
│
├── scripts/
│   ├── control_console.py      메인 GUI (6모드 대시보드)
│   ├── telemetry.py            실시간 텔레메트리 뷰어
│   ├── run_jog.sh              Isaac Sim + Console + Telemetry 한 번에 실행
│   ├── run_telemetry.sh        Telemetry 단독 실행
│   ├── _ensure_stack.sh        스택 상태 보장 (idempotent)
│   ├── m1013_sim_bringup.launch.py   DRCF 에뮬레이터 모드
│   └── real_bringup.launch.py        실제 로봇 모드
│
├── .gitignore
└── README.md
```

★ 핵심 파일.

---

## 빠른 시작 (복붙 한 번이면 끝)

> 사전: NVIDIA GPU + Docker + nvidia-container-toolkit 가 깔린 Ubuntu 22.04/24.04. (자세한 요구사항은 아래 [사전 요구사항](#사전-요구사항) 참고.)

### A. 처음 한 번만 — 설치
머신에서 한 번만 실행하면 컨테이너 빌드까지 다 끝남. **첫 빌드 30-45분 소요**.

```bash
# 1) 클론 + 권한
git clone https://github.com/Ouseong/doosan_kos.git ~/doosan_kos
cd ~/doosan_kos
chmod +x docker/*.sh scripts/*.sh

# 2) aarch64 (DGX Spark / Jetson) 만 — x86_64 일반 PC/서버는 건너뛰기
if [ "$(uname -m)" = "aarch64" ]; then
    sudo apt install -y qemu-user-static binfmt-support
    sudo bash docker/register_qemu.sh
fi

# 3) 컨테이너 빌드 + USD 조립 (한 번만)
bash docker/container.sh start
docker exec doosan_kos bash -c "/isaac-sim/python.sh /kos_workspace/isaac/urdf_to_usd_v2.py"
docker exec doosan_kos bash -c "/isaac-sim/python.sh /kos_workspace/isaac/assemble_m1013.py"
```

### B. 매번 실행 — Isaac Sim + Console + Telemetry 한 번에
설치 끝났으면 이후엔 그냥 한 줄:

```bash
cd ~/doosan_kos && bash scripts/run_jog.sh
```

자동으로 켜는 것:
- ① doosan_kos 컨테이너 (안 떠있으면 시작)
- ② DRCF 에뮬레이터 (port 12345)
- ③ ROS2 driver (`dsr_bringup2`)
- ④ Isaac Sim 3D viewer (체크 무늬 바닥)
- ⑤ Telemetry 창 (실시간 위치/속도/토크)
- ⑥ Control Console (6 모드 조작 GUI)

이미 떠있는 컴포넌트는 건너뛰고 안 떠있는 것만 시작 (idempotent).

가볍게 쓰고 싶으면:
```bash
bash scripts/run_jog.sh --no-isaac      # Isaac Sim + Telemetry 생략
bash scripts/run_telemetry.sh           # Telemetry 만
```

### C. 끝낼 때
```bash
bash docker/container.sh stop           # 정지 (재시작 빠름)
docker rm -f emulator                   # 에뮬레이터도 정리
```

## 공유 서버 사용 시 주의

도커 컨테이너 이름이 고정 (`doosan_kos`, `emulator`) 이라 **한 머신에서 동시에 한 사용자만** 사용 가능. 여러 사람이 쓰는 서버라면:

- 다 끝나면 반드시 `bash docker/container.sh stop` + `docker rm -f emulator` 로 정리
- 다음 사람은 같은 컨테이너 재사용 (이미 빌드돼있어서 즉시 부팅)
- 누군가 망가뜨려서 처음부터 빌드하려면 `bash docker/container.sh clean-all` 후 다시 빠른시작 A 부터
- GPU/X11 디스플레이도 공유 자원이라 동시에 두 사람이 Isaac Sim 띄우면 충돌. 시간 나눠 쓰기 권장.
- 특정 GPU 만 사용하려면 `--gpu N` 플래그 사용 (N = nvidia-smi 의 GPU 인덱스):
  ```bash
  bash docker/container.sh start --gpu 1   # GPU 1번만 사용
  bash docker/container.sh start           # 기본: 모든 GPU
  ```

## 사전 요구사항

| 항목 | 요구 |
|------|------|
| OS | Ubuntu 24.04 (Noble) |
| GPU | NVIDIA (RTX Ampere 이상 또는 GB10) |
| Docker | nvidia-container-runtime 설치 |
| 디스크 | 30GB+ 여유공간 |
| RAM | 32GB+ 권장 |
| Python | 시스템은 3.12 (Jazzy), Isaac Sim 번들은 3.11 |

**확인**:
```bash
lsb_release -a                  # Ubuntu 24.04
nvidia-smi                      # 드라이버 580+ 권장
docker info | grep -i runtime   # nvidia runtime 존재
df -h ~                         # 30GB+
```

**aarch64 (DGX Spark) 추가**:
```bash
# DRCF (x86_64) 를 ARM 에서 돌리기 위한 QEMU
sudo apt install -y qemu-user-static binfmt-support
sudo bash docker/register_qemu.sh
```

---

## 설치

### 1. 클론
```bash
git clone https://github.com/Ouseong/doosan_kos.git ~/doosan_kos
cd ~/doosan_kos
chmod +x docker/*.sh
```

### 2. 컨테이너 빌드 + 시작 (첫 빌드 30~45분)
```bash
bash docker/container.sh start
```

- Isaac Sim 5.1.0 이미지 pull + ROS2 Jazzy 설치 + Doosan 워크스페이스 빌드 일괄 실행
- 이후 `container.sh start` 는 기존 컨테이너 재시작

### 3. DRCF 에뮬레이터 실행 (호스트, 별도 터미널)
```bash
# x86_64 / aarch64 공통 — 3.4.1 권장 (3.0.1 은 aarch64 에서 모션 엔진 stall)
docker run -d --rm --name emulator --network host \
  --entrypoint /bin/bash \
  -e ROBOT_MODEL=M1013 \
  doosanrobot/dsr_emulator:3.4.1 \
  -c "cd /home/dra/Application/Simulator && ./DRCF M1013"

# 확인: port 12345 LISTEN 인지
ss -tlnp | grep 12345
```

### 4. M1013 USD 조립 (최초 1회)

Isaac Sim 의 URDF 임포터가 M1013 URDF 를 변환할 때 sublayer USD 들의 참조 체인이 깨집니다. 직접 조립해서 self-contained USD 를 만듭니다.

> ⚠ 아래 두 스크립트는 커밋 `eb1f4a4` 에서 삭제되었다. 같은 내용을 정리한 `isaac/furnace/assets/build_m1013.py`
> (furnace-charging 브랜치, 결과 `usd/generated/furnace/m1013/`) 를 쓰거나 `git show eb1f4a4^:isaac/urdf_to_usd_v2.py` 로 복구한다.

```bash
# 컨테이너 안에서 실행
docker exec doosan_kos bash -c "/isaac-sim/python.sh /kos_workspace/isaac/urdf_to_usd_v2.py"
docker exec doosan_kos bash -c "/isaac-sim/python.sh /kos_workspace/isaac/assemble_m1013.py"

# 결과: /tmp/m1013_v2/m1013_full.usda (mesh 40개 + joint 6개 포함)
```

### 5. 그리퍼 물리 USD 준비 (최초 1회 — 이미 git 포함)

`usd/parts/gripper_assembly_physics.usd` 는 git 에 이미 포함되어 있어 별도 생성 불필요.

STL 이 변경되거나 처음부터 재생성하려면 로컬 PC 에서:
```bash
pip install usd-core open3d matplotlib

# STL 파일이 usd/parts/ 에 있어야 함:
#   gripper_body_urdf.stl / gripper_left_urdf.stl / gripper_right_urdf.stl
python isaac/create_gripper_usd.py

# 재생성 후 git 에 반영
git add usd/parts/gripper_assembly_physics.usd
git commit -m "update gripper USD"
git push
```

그리퍼 물리 파라미터 (`create_gripper_usd.py` 상단에서 수정):

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `MASS_BODY` | 0.8 kg | 그리퍼 몸체 질량 |
| `MASS_PLATE` | 0.3 kg | 좌/우 플레이트 각 질량 |
| `MAX_TRAVEL` | 0.0335 m | 플레이트 최대 이동 거리 (33.5 mm) |
| Stiffness | 1000 N/m | PrismaticJoint drive 강성 |
| Damping | 100 N·s/m | PrismaticJoint drive 감쇠 |
| Max Force | 50 N | PrismaticJoint drive 최대 힘 |

관절 규약: `position = 0` → 완전 열림 (67 mm), `position = 0.0335` → 완전 닫힘

### 6. Doosan ROS2 driver 실행 (컨테이너)
```bash
docker exec -d doosan_kos bash -c "
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  model:=m1013 mode:=virtual host:=127.0.0.1 port:=12345 gui:=false
"
```

### 7. Isaac Sim 브리지 실행 — M1013 단독
```bash
docker exec doosan_kos bash -c "
export PYTHONUNBUFFERED=1
export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:\$LD_LIBRARY_PATH
/isaac-sim/python.sh /kos_workspace/isaac/m1013_ros2_bridge.py
"
```

Isaac Sim 창이 뜨고 M1013 로봇이 viewport 에 보입니다.

### 8. ★ Isaac Sim 브리지 실행 — M1013 + 그리퍼 통합

M1013 끝(link6)에 그리퍼가 달린 채로 Isaac Sim 시각화 + ROS2 제어:

```bash
docker exec doosan_kos bash -c "
export PYTHONUNBUFFERED=1
export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:\$LD_LIBRARY_PATH
/isaac-sim/python.sh /kos_workspace/isaac/m1013_gripper_bridge.py
"
```

그리퍼 열기/닫기 (별도 터미널에서):
```bash
# 완전 열기 (67mm)
docker exec doosan_kos bash -c "
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /gripper_command control_msgs/msg/GripperCommand '{position: 0.067, max_effort: 50.0}'
"
# 완전 닫기
docker exec doosan_kos bash -c "
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /gripper_command control_msgs/msg/GripperCommand '{position: 0.0, max_effort: 50.0}'
"
# 반 열기 (33.5mm)
docker exec doosan_kos bash -c "
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /gripper_command control_msgs/msg/GripperCommand '{position: 0.0335, max_effort: 50.0}'
"
```

현재 그리퍼 관절 위치 확인:
```bash
docker exec doosan_kos bash -c "
source /opt/ros/jazzy/setup.bash
ros2 topic echo /gripper_state
"
```

> ⚠️ **장착 회전 주의**: `MOUNT_RPY = [π/2, 0, 0]` 은 추정값입니다. Isaac Sim 실행 후 그리퍼 방향이 틀렸으면 `m1013_gripper_bridge.py` 의 `MOUNT_RPY` 를 조정하세요.

### 9. ★ 한 줄 짜리 실행 — GUI 도구

처음 셋업이 끝났다면 이후엔 그냥:

```bash
bash scripts/run_jog.sh             # Joint Jog GUI (관절 슬라이더 컨트롤)
bash scripts/run_telemetry.sh       # Telemetry 창 (실시간 위치/속도/토크/힘)
```

각 스크립트는 idempotent — 컨테이너/에뮬레이터/driver/Isaac Sim 중 안 떠있는 것만 자동으로 띄우고, 다 떠있으면 GUI 만 띄움. 두 스크립트 동시 실행 가능 (스택 공유).

가볍게 쓰고 싶으면 `--no-isaac` 플래그로 Isaac Sim 단계 생략:

```bash
bash scripts/run_jog.sh --no-isaac
```

| 도구 | 무엇을 함 |
|------|--------|
| `control_console.py` | **메인 GUI.** 6 모드 dashboard: ① Joint Slider ② Task Space (MoveL/MoveJX) ③ Incremental Jog ④ Waypoint Recorder ⑤ Speed Control (deadman) ⑥ MoveIt2 launcher |
| `telemetry.py` | 3섹션 라이브 뷰: ① 직접 측정 (관절 위치, 토크) ② DRCF 계산값 (관절 속도, 외부 토크, TCP 위치/속도/힘) ③ ROS 측 계산값 (관절·TCP 가속도, EMA 평활) |

### 10. 수동 동작 테스트
```bash
docker exec doosan_kos bash -c "
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint \
  '{pos: [45.0, 0, 0, 0, 0, 0], vel: 30, acc: 30}'
"
```

Isaac Sim viewport 에서 joint_1 이 45도 회전하는 걸 확인할 수 있습니다.

---

## Dynamixel 그리퍼 노드 (실제 하드웨어)

`Dynamixel_Control/gripper_node.py` 는 실제 Dynamixel 모터와 ROS2 를 연결하는 노드.  
Isaac Sim 브리지(`/gripper_command`)와 동일한 토픽 인터페이스를 사용하므로, 시뮬레이션과 실제 하드웨어를 토픽 하나로 전환 가능.

### 사전 설치

```bash
pip install dynamixel_sdk
```

### 실행

```bash
# 기본 파라미터
ros2 run doosan_kos gripper_node

# 파라미터 지정
ros2 run doosan_kos gripper_node --ros-args \
  -p device:=/dev/ttyUSB0 \
  -p baudrate:=57600 \
  -p dxl_id:=3 \
  -p torque_threshold:=300 \
  -p close_velocity:=5 \
  -p publish_rate:=20.0
```

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `device` | `/dev/ttyUSB0` | Dynamixel 시리얼 포트 |
| `baudrate` | `57600` | 통신 속도 |
| `dxl_id` | `3` | Dynamixel 모터 ID |
| `torque_threshold` | `300` | 파지 감지 토크 임계값 (mNm) |
| `close_velocity` | `5` | 닫힘 속도 (RPM) |
| `publish_rate` | `20.0` | 상태 발행 주기 (Hz) |

### 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/gripper_command` | `control_msgs/GripperCommand` | 수신 | `position`: 열림 너비 (m), `max_effort`: 토크 임계값 (0이면 기본값 사용) |
| `/gripper_state` | `sensor_msgs/JointState` | 발행 | `name`: [left_joint, right_joint], `position`: 각 플레이트 변위 (m) |

### 동작 방식

1. `/gripper_command.position` 수신
2. `position > 현재` → 열기 (모터 반시계)
3. `position < 현재` → 천천히 닫기 (`close_velocity` RPM), 토크가 `torque_threshold` 초과하면 정지·유지
4. `/gripper_state` 주기적 발행

수동 조그 테스트는 `Dynamixel_Control/gripper_test.py` (pygame GUI) 사용.

---

## 컨테이너 관리

```bash
bash docker/container.sh start     # 시작 / 재시작 (이미지 없으면 빌드)
bash docker/container.sh enter     # bash 접속
bash docker/container.sh stop      # 중지
bash docker/container.sh clean     # 컨테이너 삭제 (이미지 유지)
bash docker/container.sh clean-all # 컨테이너 + 이미지 전부 삭제
bash docker/container.sh status    # 상태 확인
```

---

## 주요 해결 사항

이 환경 구축에서 돌파한 주요 기술 이슈들:

| 이슈 | 원인 | 해결 |
|------|------|------|
| ROS2 Humble 설치 실패 | Isaac Sim 5.1 = Ubuntu 24.04, Humble 은 22.04 전용 | **ROS2 Jazzy 로 전환** |
| OmniGraph 세그폴트 (aarch64) | `libomni.graph.image.core` aarch64 빌드 버그 | `isaacsim.exp.base.kit` 사용 + Python API 직접 호출 |
| `rclpy` import 실패 | Isaac Sim Py 3.11 ↔ ROS2 Jazzy Py 3.12 충돌 | Isaac Sim 번들 rclpy 경로 `sys.path` 주입 |
| `libament_index_cpp.so` 못 찾음 | Isaac Sim 번들 ROS2 라이브러리 경로 누락 | `LD_LIBRARY_PATH` 보강 |
| Isaac Sim 창 안 뜸 | 컨테이너 DISPLAY/Xauthority 미설정 | 컨테이너 재생성 + 올바른 Xauth 바인드 |
| Viewport 까만 화면 | base.kit 이 카메라/조명 자동 생성 안 함 | 스크립트에서 LookAt 카메라 + DistantLight + Dome + GroundPlane 명시 생성 |
| 로봇이 누워있음 | URDF zero-pose 가 캘리브레이션 자세 | `HOME_POSE = [0, -π/2, π/2, 0, π/2, 0]` 초기화 + DRCF 전체-0 덮어쓰기 필터 |
| M1013 메시 미렌더 | URDF 임포터가 sublayer 깨진 USD 생성 | `m1013_base.usd` + `m1013_physics.usd` 를 sublayer 로 수동 조립 |
| DRCF x86_64 on aarch64 | DRCF 바이너리가 x86_64 전용 | `qemu-user-static` + binfmt 등록 |
| 에뮬레이터 모션 stall (aarch64) | `dsr_emulator:3.0.1` 의 모션 엔진이 qemu-user 환경에서 `MTNFCESTATE_HOLD_IDLE` 자가루프, 결국 `FaultOccured` | 더 새로운 `dsr_emulator:3.4.1` 사용 (실로봇은 native DRCF 라 무관) |
| `NameError: SetSingularityHandlingForce` | 업스트림 `DSR_ROBOT2.py` 의 오타 (srv 이름은 `SetSingularHandlingForce`) | bootstrap 단계에서 sed 로 자동 패치 |
| 예제가 `Set Robot Mode Service is not available` 무한 대기 | 업스트림 `_srv_name_prefix=''` ↔ `dsr_bringup2` 가 모든 서비스를 `dsr_controller2/` 하위에 등록 | bootstrap 단계에서 prefix 를 `dsr_controller2/` 로 자동 패치 |
| `gui:=false` 인데 RViz 가 항상 켜짐 | `dsr_bringup2_rviz.launch.py` 의 `IfCondition(gui)` 와 변수 선언이 둘 다 주석 처리 | bootstrap 단계에서 두 줄의 주석 자동 제거 |

---

## 실제 로봇 연결

DRCF 에뮬레이터 대신 실제 M1013 로봇 IP 로:
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  model:=m1013 mode:=real host:=<ROBOT_IP> port:=12345 gui:=false
```

---

## Furnace charging 시뮬레이션 (furnace-charging 브랜치)

핀어레이 그리퍼로 금속판(metal_plate, 재질 미확정)을 집어 용해로에 투입하고, 핀 변위로 파지 자세를 추정해 release 를 보정하는 연구용 시뮬레이션. 기존 브리지·해머링 작업과 분리되어 있다 (`isaac/furnace/`). 계획·원칙은 [docs/furnace_charging/](docs/furnace_charging/) (PLAN / SPEC / VERSIONS / ASSETS / RESULTS).

- 장면·로봇·그리퍼 물리: Isaac Sim 5.1 (PhysX), 별도 컨테이너 `furnace_kos` (같은 이미지, 리포 worktree 마운트, ROS_DOMAIN_ID 77)
- 유체: **Newton 1.8.0 MPM 단독** (별도 venv `~/venvs/furnace_newton`). Isaac Lab 의 MPM 은 Isaac Sim 6.2 전용이라 5.1 에서 못 씀. Isaac 화면의 유체는 Newton 결과 **재생**이며 화면에 그렇게 표시한다
- 유체는 "용탕의 기계적 거동을 단순화한 등온 물 수준 유체" 이고 실제 용탕을 재현한다고 주장하지 않는다
- Mobile Base 는 판매 페이지 치수(850×600×825 mm)로 만든 **임시 모델** (장착 위치·캐스터 가정). 공장·용해로는 절차적 생성 (외부 자산 없음)
- 가정값 목록: PLAN §7, `isaac/furnace/config/default.yaml` (각 값에 source 표시)

```bash
# 준비 (한 번): docs/furnace_charging/VERSIONS.md "재현 명령" 참고 (worktree, furnace_kos 컨테이너, venv)
# A. M1013 USD (공식 doosan-robot2 URDF → usd/generated/furnace/m1013)
docker exec -e CUDA_VISIBLE_DEVICES=1 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/assets/build_m1013.py --screenshot

# 1) scene_preview — 공장·용해로·로봇 외형과 조명 (카메라 4 개 PNG)
docker exec -e CUDA_VISIBLE_DEVICES=1 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_scene_preview.py --headless
# 2) fluid_plate_test — 판과 유체 물리만 (SPEC §13 1–8)
CUDA_VISIBLE_DEVICES=1 ~/venvs/furnace_newton/bin/python isaac/furnace/run_fluid_plate_test.py --suite
# 3) robot_charge_demo — 로봇 파지·이송 (PhysX, 핀 50 개 스프링)
docker exec -e CUDA_VISIBLE_DEVICES=1 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_robot_charge_demo.py --headless --tag demo --frames_every 24
# 4) evaluate — 조건 A/B/C/D 비교 (Newton)
CUDA_VISIBLE_DEVICES=1 ~/venvs/furnace_newton/bin/python isaac/furnace/run_evaluate.py --conditions A,B,C,D --seeds 0-19
# 보조: 추정기 평가, Newton 결과 재생 렌더, 단위 테스트
~/venvs/furnace_newton/bin/python isaac/furnace/run_grasp_estimator_eval.py
docker exec -e CUDA_VISIBLE_DEVICES=1 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_playback_render.py --headless --npz /kos_workspace/output/furnace/fluid_plate_test/<run>/free_drop_trajectory.npz
~/venvs/furnace_newton/bin/python -m pytest isaac/furnace/tests
# H. 로봇 데모를 GUI 로 보기 (끝나도 창 유지). 창은 디스플레이 GPU(2) + xhost +local: 필요
DISPLAY=:1 xhost +local:
docker exec -e DISPLAY=:1 -e CUDA_VISIBLE_DEVICES=2 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_robot_charge_demo.py --pin_k 5000 --tag a4_gui --keep_open
# I. splash 1 건 계산 (Newton, dx 18 mm, 약 30 s) → 두 건을 용해로 두 개에 나란히 재생 (GUI 반복 / --headless --record 로 프레임 저장)
CUDA_VISIBLE_DEVICES=1 ~/venvs/furnace_newton/bin/python isaac/furnace/run_splash_case.py --tag low_vertical --pitch 0 --height 0.06
CUDA_VISIBLE_DEVICES=1 ~/venvs/furnace_newton/bin/python isaac/furnace/run_splash_case.py --tag high_flat --pitch 90 --height 0.5
docker exec -e DISPLAY=:1 -e CUDA_VISIBLE_DEVICES=2 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_splash_view.py --cases low_vertical high_flat
# J. 로봇이 판을 액면 위 0.06 m 까지 내려 놓고, 놓는 순간부터 Newton splash 재생 (2 단계)
#    1) headless 로 놓는 순간 판 상태 저장 → 2) Newton 계산 → 3) GUI 에서 로봇 + splash 재생
docker exec -e CUDA_VISIBLE_DEVICES=1 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_robot_charge_demo.py --headless --pin_k 5000 --tag charge_release --drop_height 0.06 --drop_offset 0.25 --release --release_json /kos_workspace/output/furnace/splash_cases/robot_release.json
CUDA_VISIBLE_DEVICES=1 ~/venvs/furnace_newton/bin/python isaac/furnace/run_splash_case.py --tag robot_charge --release_json output/furnace/splash_cases/robot_release.json
docker exec -e DISPLAY=:1 -e CUDA_VISIBLE_DEVICES=2 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_robot_charge_demo.py --pin_k 5000 --tag charge_gui --drop_height 0.06 --drop_offset 0.25 --release --splash_npz /kos_workspace/output/furnace/splash_cases/robot_charge_trajectory.npz --keep_open
```

---

## 라이선스 / 기여

- Doosan 저장소: [doosan-robotics/doosan-robot2](https://github.com/doosan-robotics/doosan-robot2)
- Isaac Sim: NVIDIA Omniverse
- 본 레포: MIT
