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
│   ├── container.sh            start/enter/stop/clean 관리 스크립트
│   ├── run_emulator.sh         DRCF 에뮬레이터 실행
│   └── register_qemu.sh        aarch64 에서 x86_64 DRCF 돌리기 위한 binfmt 등록
│
├── isaac/
│   ├── m1013_ros2_bridge.py    ★ 메인 브리지 (ROS2 joint_states → Isaac Sim 관절)
│   │
│   ├── urdf_to_usd.py          URDF → USD 변환 (v1, 기본 옵션)
│   ├── urdf_to_usd_v2.py       URDF → USD 변환 (v2, merge_fixed_joints=True)
│   ├── assemble_m1013.py       ★ M1013 USD 수동 조립 (base + physics sublayer)
│   │
│   ├── flatten_usd.py          USD 평탄화 (디버깅)
│   ├── flatten_deep.py         Dependencies 분석 + 평탄화 (디버깅)
│   └── inspect_usd.py          USD prim 구조 출력 (디버깅)
│
├── scripts/
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

```bash
# 컨테이너 안에서 실행
docker exec doosan_kos bash -c "/isaac-sim/python.sh /kos_workspace/isaac/urdf_to_usd_v2.py"
docker exec doosan_kos bash -c "/isaac-sim/python.sh /kos_workspace/isaac/assemble_m1013.py"

# 결과: /tmp/m1013_v2/m1013_full.usda (mesh 40개 + joint 6개 포함)
```

### 5. Doosan ROS2 driver 실행 (컨테이너)
```bash
docker exec -d doosan_kos bash -c "
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  model:=m1013 mode:=virtual host:=127.0.0.1 port:=12345 gui:=false
"
```

### 6. Isaac Sim 브리지 실행
```bash
docker exec doosan_kos bash -c "
export PYTHONUNBUFFERED=1
export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:\$LD_LIBRARY_PATH
/isaac-sim/python.sh /kos_workspace/isaac/m1013_ros2_bridge.py
"
```

Isaac Sim 창이 뜨고 M1013 로봇이 viewport 에 보입니다.

### 7. ★ 한 줄 짜리 실행 — GUI 도구

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

### 8. 수동 동작 테스트
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

## 라이선스 / 기여

- Doosan 저장소: [doosan-robotics/doosan-robot2](https://github.com/doosan-robotics/doosan-robot2)
- Isaac Sim: NVIDIA Omniverse
- 본 레포: MIT
