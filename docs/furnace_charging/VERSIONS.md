# 사용 버전과 Phase 0 환경 조사

조사일: 2026-11-23. 서버(oem-WS-C621E-SAGE-Series)에서 실제로 실행·import 해서 확인한 값만 적는다.

## 호스트

| 항목 | 값 | 확인 방법 |
|---|---|---|
| 아키텍처 | **x86_64** (DGX Spark 아님) | `uname -m` |
| OS | Ubuntu 22.04.5 LTS | `lsb_release -ds` |
| GPU | NVIDIA RTX A6000 × 3 (48 GB). 화면 출력은 GPU 2. furnace 계산은 GPU 1 사용 | `nvidia-smi` |
| 드라이버 | 580.65.06 | `nvidia-smi` |
| 디스크 | **여유 약 54 GB (97 % 사용)**. 큰 다운로드(Isaac Lab 3 + torch 등)는 주의 | `df -h` |
| 호스트 Python | 3.13.9 (miniconda) | `python3 --version` |

## Isaac Sim 컨테이너

기존 `doosan_kos` 컨테이너는 건드리지 않는다. 같은 이미지(`doosan_kos:latest`, 이미지 변경 없음)로 **별도 컨테이너 `furnace_kos`** 를 만들어 furnace-charging worktree(`~/doosan_kos`)를 `/kos_workspace` 에 마운트했다. ROS_DOMAIN_ID=77 로 실로봇 ROS 그래프와 분리.

| 항목 | 값 | 확인 방법 |
|---|---|---|
| Isaac Sim | **5.1.0-rc.19+release.26219.9c81211b.gl** | `/isaac-sim/VERSION` |
| 번들 Python | 3.11.13 | `python.sh -c "import sys"` |
| Warp (번들) | 1.8.2 (`omni.warp.core-1.8.2`) | `import warp` |
| PhysX 확장 | `omni.physx-107.3.26` | `/isaac-sim/extscache` |
| Isaac Lab | **설치 안 됨** (컨테이너·이미지 모두) | find, pip list |
| Newton | **없음** (5.1 이미지에 newton 확장/패키지 없음) | ls exts/extscache, pip list |
| 기타 pip | numpy 1.26.0, scipy 1.15.3, torch 2.7.0+cu128, trimesh 4.5.1, PyYAML 6.0.2 | `python.sh -m pip list` |
| import 되는 API | `isaacsim.core.api`, `isaacsim.core.prims`, `isaacsim.core.utils.stage`, `isaacsim.asset.importer.urdf`(명령 `URDFParseAndImportFile` 등), `omni.isaac.core`(구 이름도 아직 됨). `omni.importer.urdf` 는 없음 | `isaac/furnace` 프로브 실행 |

참고: 호스트에 `nvcr.io/nvidia/isaac-sim:6.0.0` 이미지(6.0.0-rc.59)도 있다. 그 안에 `isaacsim.physics.newton`, `isaacsim.pip.newton`, `omni.usd.schema.newton`, Warp 1.13.0 확장이 들어 있는 것만 확인했다(`ls exts`). **실행은 안 해 봤다.** Isaac Sim 안에서 Newton 을 쓰는 후보로만 기록.

## doosan-robot2 (M1013 모델 출처)

| 항목 | 값 |
|---|---|
| 리포 | https://github.com/doosan-robotics/doosan-robot2 (Apache-2.0, LICENSE 확인) |
| 브랜치 / 커밋 | `jazzy` / `816ecb5d1c2599303eaf9540216afa03552f80ad` (2026-04-23, "chore: release 20260423") |
| 받은 방법 | `docker/bootstrap_ws.sh` 가 `git clone -b jazzy` (커밋 고정 없음). `doosan_kos` 컨테이너의 `/ros2_ws/src` 에서 복사 |
| 사용 파일 | `dsr_description2/urdf/m1013.urdf` (리포에 포함된 xacro 생성본), `meshes/m1013_white/*.dae`, `meshes/m1013_collision/*.dae` → `assets_external/doosan-robot2/` (gitignore) |
| 공식 USD | `dsr_description2/usd/` 에 a0509·a0912·e0509·h2017·h2515·m0609·m0617 만 있고 **m1013.usd 는 없음** → URDF 변환 |

## Newton (유체 백엔드) — 별도 venv

| 항목 | 값 |
|---|---|
| venv | `~/venvs/furnace_newton` (호스트 Python 3.13.9, 리포·기존 환경과 분리) |
| newton | **1.8.0** (PyPI, Apache-2.0, Requires-Python ≥3.10) |
| warp-lang | 1.19.0 |
| mujoco / mujoco-warp | 3.17.0 / 3.17.0 (`newton[sim]` extra. 양방향 결합 예제의 강체 solver 가 요구) |
| matplotlib | 결과 플롯용 |
| 설치 명령 | `python3 -m venv ~/venvs/furnace_newton && ~/venvs/furnace_newton/bin/pip install "newton[sim]==1.8.0" matplotlib` |
| aarch64 | PyPI 에 `warp_lang-1.19.0-...manylinux_2_34_aarch64.whl`, `mujoco-3.17.0-...aarch64.whl` 있음. newton·mujoco-warp 는 순수 파이썬 wheel. **aarch64 에서 실행은 안 해 봄** |

## 유체 백엔드 결정 (PLAN §4 Phase 0-4 순서)

1. **Isaac Lab MPM → 불채택.** MPM 문서(`concepts/using_mpm.html`)는 Isaac Lab main(3.0) 문서에만 있고, main 설치 문서는 "Isaac Sim 6.2" 를 요구하며 "Isaac Sim 5.1 and older are unsupported" 라고 적혀 있다. 5.1 세대인 v2.3.0 문서의 `using_mpm.html` 은 404, v2.3.2 색인에도 MPM 이 없다. 설치본(5.1)에서 쓸 수 있는 경로가 아님.
2. **Newton 단독 venv → 채택.** 설치본 `newton/examples/mpm/` 에 `example_mpm_water_dam_break.py`, `example_mpm_twoway_coupling.py` 등이 실제로 있고, 둘 다 `--viewer null --num-frames 100` 으로 실행 성공 (각 1 분 9 초, 2 분 30 초, GPU 1). twoway 예제는 처음엔 `mujoco` 미설치로 실패 → `newton[sim]` 설치 후 성공.
3. PhysX PBD → 2 가 되어서 시도 안 함.

Isaac Sim 과의 관계: Newton 은 Isaac Sim 5.1 밖에서 돈다. Isaac Sim 화면에 보이려면 Newton 결과(`trajectory.npz`)를 **녹화 재생**해야 하고, 그 화면은 재생이라고 표시한다 (Phase 1 이후).

## Phase 0 최소 예제 결과 (요약, 상세는 PLAN §9)

`isaac/furnace/sim/newton_plate_drop_min.py` — 사각 용기(내부 0.6×0.6×0.7 m, 정적 박스 5개) + 물 수준 MPM 액체 0.4 m (입자 254,016개, dx 25 mm) + metal_plate 500×300×10 mm (밀도 후보 2700 → 4.05 kg), 액면 위 0.25 m 에서 20° 기울여 놓음. 양방향 결합. 1.5 s, 계산 34 s.

| SPEC §13 | 결과 |
|---|---|
| 1 액체 유지 | △ 끝에 1.51 % 가 용기 밖. **no-impact(판 없음) 대조도 1.37 % 밖** → 대부분 수치 누수 (바닥·벽 모서리, 벽 두께 안으로 파고든 입자 포함). splash 판정 전에 Phase 1 에서 줄여야 함 |
| 2 판 관통 없음 | ○ 판 최저 z 0.223 m, 판 xy 최대 0.260 m (< 내벽 0.30) |
| 3 자유 운동 | ○ 회전 변화 최대 22°, 하강 0.41 m |
| 4 판이 액체를 밀어냄 | ○ 최고 입자 높이 0.399 → 0.564 m |
| 5 액체 반력이 판에 반영 | ○ 진입 후 5 프레임 평균 가속도 +6.8 m/s² (자유낙하 −9.81) |
| 6 입자 위치·질량·속도 읽기 | ○ `particle_q/qd/mass` |
| 7 reset | 미시험 |
| 8 해상도·dt sweep | 미시험. 판 두께/격자 = 0.4 (판이 격자보다 얇음) |

## 재현 명령

```bash
# furnace 전용 컨테이너 (기존 doosan_kos 이미지 그대로, 컨테이너만 별도)
git -C ~/Desktop/doosan_kos worktree add ~/doosan_kos furnace-charging
C=~/docker/isaac-sim
docker run -d --name furnace_kos --gpus all --network host --ipc host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e DISPLAY=:1 -e ROS_DOMAIN_ID=77 \
  -v ~/.Xauthority:/isaac-sim/.Xauthority:ro -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/doosan_kos:/kos_workspace \
  -v $C/cache/main:/isaac-sim/.cache -v $C/cache/computecache:/isaac-sim/.nv/ComputeCache \
  -v $C/logs:/isaac-sim/.nvidia-omniverse/logs -v $C/config:/isaac-sim/.nvidia-omniverse/config \
  -v $C/data:/isaac-sim/.local/share/ov/data -v $C/pkg:/isaac-sim/.local/share/ov/pkg \
  -u 1234:1234 --entrypoint bash doosan_kos:latest -c "sleep infinity"
mkdir -p ~/doosan_kos/usd/generated/furnace ~/doosan_kos/output/furnace && chmod -R a+rwX ~/doosan_kos/usd/generated ~/doosan_kos/output/furnace   # 컨테이너 uid 1234 가 쓰도록

# A. M1013 USD (+ 확인 JSON, 스크린샷)
docker exec -e CUDA_VISIBLE_DEVICES=1 furnace_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/assets/build_m1013.py --screenshot

# B. Newton 최소 판-액체 + no-impact 대조 + 플롯
cd ~/doosan_kos
CUDA_VISIBLE_DEVICES=1 ~/venvs/furnace_newton/bin/python isaac/furnace/sim/newton_plate_drop_min.py --out output/furnace/phase0/newton_min
CUDA_VISIBLE_DEVICES=1 ~/venvs/furnace_newton/bin/python isaac/furnace/sim/newton_plate_drop_min.py --no_plate --out output/furnace/phase0/newton_min_noimpact
~/venvs/furnace_newton/bin/python isaac/furnace/sim/plot_newton_min.py output/furnace/phase0/newton_min
```
