# doosan_kos

두산 협동로봇 M1013 + Isaac Sim 5.1.0 + ROS2 Humble Docker 환경
✅ x86_64 (서버컴) + aarch64 (DGX Spark) 모두 지원

## 아키텍처

```
┌─────────────────────────────────────────────┐
│              Host (서버컴 or Spark)           │
│                                             │
│  ┌──────────────────┐  ┌─────────────────┐ │
│  │  DRCF 에뮬레이터  │  │   doosan_kos    │ │
│  │  (두산 공식)      │  │   컨테이너      │ │
│  │                  │  │                 │ │
│  │  가짜 로봇       │◄─►│  Isaac Sim 5.1  │ │
│  │  컨트롤러        │  │  ROS2 Humble    │ │
│  │  :12345          │  │  doosan-robot2  │ │
│  └──────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────┘
```

## 폴더 구조

```
doosan_kos/
├── docker/
│   ├── Dockerfile              ← Isaac Sim 5.1.0 + ROS2 + doosan 이미지
│   ├── bootstrap_ws.sh         ← doosan-robot2 자동 빌드
│   ├── entrypoint.sh           ← 컨테이너 시작 시 자동 실행
│   ├── container.sh            ← start/enter/stop/clean 관리
│   └── run_emulator.sh         ← DRCF 에뮬레이터 실행
├── isaac/
│   └── m1013_ros2_bridge.py    ← Isaac Sim 5.1.0 ROS2 브리지
├── scripts/
│   ├── m1013_sim_bringup.launch.py  ← 가상 모드 (에뮬레이터)
│   └── real_bringup.launch.py       ← 실제 로봇 모드
├── .gitignore
└── README.md
```

## 사전 요구사항

```bash
lsb_release -a                  # Ubuntu 22.04 확인
nvidia-smi                      # GPU 드라이버 확인
docker info | grep -i runtime   # nvidia 가 보여야 함
df -h ~                         # 여유 공간 30GB 이상
```

## 설치 및 실행

### 1. 레포 클론
```bash
git clone https://github.com/Ouseong/doosan_kos.git ~/doosan_kos
cd ~/doosan_kos
chmod +x docker/*.sh
```

### 2. 컨테이너 시작 (처음 한 번만 30~45분 소요)
```bash
bash docker/container.sh start
```

### 3. 실행 순서 (터미널 5개)

**터미널 1 (호스트)** - DRCF 에뮬레이터
```bash
bash docker/run_emulator.sh
```

**터미널 2 (컨테이너)** - Isaac Sim
```bash
# GUI 모드 (모니터 연결 시)
/isaac-sim/python.sh /kos_workspace/isaac/m1013_ros2_bridge.py --topic dsr01/joint_states

# 헤드리스 모드 (원격 접속 시)
/isaac-sim/python.sh /kos_workspace/isaac/m1013_ros2_bridge.py --topic dsr01/joint_states --headless
```

**터미널 3 (컨테이너)** - ROS2 드라이버
```bash
ros2 launch /kos_workspace/scripts/m1013_sim_bringup.launch.py \
    mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
```

**터미널 4 (컨테이너)** - 관절 상태 모니터링
```bash
ros2 topic echo /isaac_joint_states
```

**터미널 5 (컨테이너)** - 로봇 동작 명령
```bash
ros2 service call /dsr01/motion/move_home dsr_msgs2/srv/MoveHome "{target: 0}"
```

## 실제 로봇으로 전환

터미널 1 에뮬레이터 없이, 터미널 3만 변경:
```bash
ros2 launch /kos_workspace/scripts/real_bringup.launch.py \
    host:=<ROBOT_IP> model:=m1013
```

## 컨테이너 관리

```bash
bash docker/container.sh start     # 시작
bash docker/container.sh enter     # 접속 (추가 터미널)
bash docker/container.sh stop      # 중지
bash docker/container.sh clean     # 삭제 (이미지 유지)
bash docker/container.sh clean-all # 전부 삭제
bash docker/container.sh status    # 상태 확인
```

## DGX Spark 주의사항

- Isaac Sim 5.1.0 컨테이너는 aarch64 (ARM) 지원 ✅
- Livestreaming은 Spark에서 미지원 → `--headless` 옵션 사용
- GUI는 Spark 모니터에 직접 연결하거나 X11 포워딩으로 사용
