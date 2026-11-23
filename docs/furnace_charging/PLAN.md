# Furnace Charging — 구현 계획

Date: 2026-11-23 (계획 작성: Windows 노트북, 실행 환경 없음)
Status: **계획 단계. 코드 없음.** 서버(Isaac Sim 컨테이너가 도는 Linux 머신)에서 Phase 0 부터 진행
Branch: `furnace-charging`
원본 요구사항: [SPEC.md](SPEC.md) — 원칙은 SPEC 이 우선한다.

---

## 0. 서버에서 시작하는 법 (먼저 읽기)

```bash
cd ~/doosan_kos
git fetch origin && git switch furnace-charging && git pull
# 이 문서 → SPEC.md → §1 확인된 사실 → Phase 0 순서로 진행
```

작업 규칙:
- **Phase 단위로 커밋**하고 `furnace-charging` 에 push 한다. `main` 에 직접 커밋하지 않는다.
- 진행할 때마다 §9 **진행 로그**에 "확인한 사실 / 미확정 / 만든 결과 / 다음 검증" 을 짧게 추가한다.
- 문서의 수치 중 **[가정]** 표시는 실측·공식값이 아니다. 확인되면 §7 가정값 표를 고친다.
- 생성된 USD(`*.usd*` 는 이미 gitignore), 다운로드한 외부 자산(`assets_external/`), 영상·프레임은 커밋하지 않는다. CSV/JSON 결과와 대표 스크린샷 PNG 만 커밋한다.
- 기존 `isaac/m1013_gripper_bridge.py`, 해머링 작업, `docker/Dockerfile` 은 **건드리지 않는다**. 새 기능은 `isaac/furnace/` 아래에 만든다. 새 의존성(Newton/Warp 등)은 기존 이미지에 섞지 말고 별도 venv 또는 별도 컨테이너에 설치한다.

---

## 1. 계획 시점에 확인된 사실 (2026-11-23, 리포·웹 조사)

### 1.1 리포 / 실행 환경
| 항목 | 내용 | 근거 |
|---|---|---|
| Isaac Sim | 5.1.0 Docker (`nvcr.io/nvidia/isaac-sim:5.1.0`) + ROS2 Jazzy, 컨테이너명 `doosan_kos` | `docker/Dockerfile` |
| 마운트 | 리포 루트 → 컨테이너 `/kos_workspace` | `docker/container.sh:88` |
| 실행 | `docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/...` | 기존 브리지 docstring |
| 하드웨어 | **x86_64 서버 (RTX A6000 ×3)** 에서 진행 (Phase 0 `uname -m` 확인). aarch64 는 wheel 존재만 확인 | VERSIONS.md |
| 기존 API | 브리지는 `omni.isaac.core` (World, add_reference_to_stage) 사용. Isaac Sim 5.x 에서는 `isaacsim.core.api` 등으로 이름이 바뀌는 중 → **새 코드는 설치본에서 실제로 import 되는 쪽을 확인 후 사용** | `isaac/m1013_gripper_bridge.py:120-125` |
| Isaac Lab / Newton | 5.1 컨테이너에 둘 다 없음 (Phase 0 확인). Newton 은 별도 venv 1.8.0 사용 | [VERSIONS.md](VERSIONS.md) |

### 1.2 M1013 로봇 USD — ⚠ 생성 스크립트가 지워져 있음
- 브리지 기본 경로: `--robot_usd /tmp/m1013_v2/m1013_full.usda` (임시 경로, 컨테이너 재시작 시 사라질 수 있음).
- 이 파일을 만들던 `isaac/urdf_to_usd_v2.py`, `isaac/assemble_m1013.py` 는 커밋 `eb1f4a4` 에서 **삭제됨**. README 빠른 시작 A-3 은 아직 이 스크립트를 실행하라고 되어 있음(README 가 낡음).
- 복구: `git show eb1f4a4^:isaac/urdf_to_usd_v2.py`, `git show eb1f4a4^:isaac/assemble_m1013.py`
- URDF: `usd/m1013/m1013.urdf`, mesh 는 `package://dsr_description2/meshes/m1013_{white,collision}/*.dae` (컨테이너의 doosan-robot2 ws). 로컬 `usd/m1013/meshes/` 는 gitignore.
- URDF 값(공식 doosan-robot2 유래):

| joint | lower/upper [rad] | effort [N·m] | velocity [rad/s] |
|---|---|---|---|
| joint_1 | ±6.2832 | 346 | 2.0944 |
| joint_2 | ±6.2832 | 346 | 2.0944 |
| joint_3 | ±2.7925 | 163 | 3.1416 |
| joint_4 | ±6.2832 | 50 | 3.927 |
| joint_5 | ±6.2832 | 50 | 3.927 |
| joint_6 | ±6.2832 | 50 | 3.927 |

  링크 질량: base 4.12 / l1 7.8 / l2 10.83 / l3 3.68 / l4 3.82 / l5 2.82 / l6 1.16 kg.
- 제조사 사양: 가반하중 **10 kg**, 작업반경 **1300 mm**, 로봇 무게 약 32–33 kg.

### 1.3 핀어레이 그리퍼 (사용자 실물 CAD 있음)
| 항목 | 값 | 근거 |
|---|---|---|
| CAD | `usd/sim_gripper/*.stl` (body, plate_{p,n}_{house,pcb,pins,mag}) + `meta.json` | 리포 |
| 프레임 | Sim_Gripper 좌표: 몸체 윗면(장착면) Z=0, 손가락 −Z, 판은 ±X 로 열림 | `meta.json` |
| 판 이동 | 한쪽 0–30 mm (`plate_travel_mm`), 명령 0–0.067 m = 최대 열림 **약 67 mm** | `meta.json`, 브리지 `CMD_MAX_OPENING_M` |
| grasp center | (0, 15.5, −128.4) mm (Sim_Gripper 좌표) | `meta.json` |
| 장착 | link_6 플랜지 → 원기둥 스페이서 Ø80×35 mm → 그리퍼 (X축 180° 뒤집음) | 브리지 `_setup_sim_gripper` |
| 핀 배열 | jaw 당 **5×5**, 피치 **12.0 mm** (STL 실측) | `Hall_Sensor/Code/Sim_Gripper.py:138` |
| 핀 단면/길이 | 약 9.0 × 8.97 mm, 길이 28.5 mm (핀 덩어리 bbox) | `Sim_Gripper.py:137` |
| 핀 stroke | 스프링 행정 12.0 mm (논문 그림은 12.5 mm) | `Sim_Gripper.py:210`, `논문_ICRA2027/make_figures.py:28` |
| 스프링 강성·감쇠 | **없음** → [가정] | — |
| 물리 | 현재 브리지의 그리퍼는 **시각 전용**(충돌·관절 없음). 핀은 움직이지 않음 | 브리지 docstring |
| ⚠ 설계 버전 | 리포 STL 은 `Left_Plate_v2.stl`(v2) 에서 생성. `Sim_Gripper.py` 현재 기본은 `DESIGN="A4"` (2026-11-05 판) → **어느 판으로 시뮬레이션할지 사용자 확인** (§8) | `meta.json`, `Sim_Gripper.py:128` |

### 1.4 이동 플랫폼 = 두산로보틱스 "Mobile Base" (사용자가 SYSCON 이라 부르는 장비)
사용자 사진(흰 캐비닛, 검은 하부 스커트, 둥근 모서리 금속 상판, 옆 손잡이, 앞문·서랍, 내부 제어기 트레이, 상판 케이블 구멍)과 블로그(M1013 + 모바일 베이스 개봉기, **M8 볼트 4개로 로봇 고정**)로 확인.
**자율주행 AMR 이 아니라 손으로 미는 캐비닛.** → 주행/도킹/SLAM 불필요, 레벨링 풋 내린 정지 상태로 모델링.

| 항목 | 값 | 근거 |
|---|---|---|
| 크기 | **850(W) × 600(D) × 825(H) mm** | Unchained Robotics, Cross Co. 판매 페이지 (두 곳 일치) |
| 무게 | 166 kg (두산 페이지 검색 요약은 175 kg) | 동일 |
| 상판 | 스테인리스강 | 동일 |
| 이동·고정 | heavy duty locking casters + heavy duty leveling system (개수·치수 미공개) | Cross Co. |
| 호환 | M0609/M0617/M1013/M1509, A·H 시리즈 | 동일 |
| 로봇 장착 위치 / 볼트 패턴 | **미공개** → [가정] 상판 중앙 | — |
| CAD | 공개 CAD/도면 없음 | 조사 |
| "SYSCON" | 판매 페이지에 SYSCON 표기 없음. 시스콘(syscon.kr)은 별도 자율주행 물류로봇 회사. 실물에 SYSCON 명판이 있다면 OEM 일 가능성 — 미확정 | 조사 |

출처: https://unchainedrobotics.de/en/products/cobot-base/doosan-mobile-base , https://www.crossco.com/providers/doosan-robotics/mobile-base/

### 1.5 가반하중 점검 (재질 미확정 → 후보별로만 계산)
판 부피 = 0.5 × 0.3 × 0.01 = 1.5×10⁻³ m³

| 밀도 후보 [kg/m³] (예시일 뿐, 재질 확정 아님) | 판 질량 | 10 kg 대비 |
|---|---|---|
| 2,700 | 4.05 kg | 여유 있음 (그리퍼 질량 추가 필요) |
| 7,140 | 10.7 kg | **초과** |
| 7,870 | 11.8 kg | **초과** |

- 그리퍼+스페이서 질량 **미확인** → [가정] 값을 설정에 두고 실측 요청 (§8).
- 판 끝을 잡고 수평으로 들면 무게중심까지 레버 0.15 m(긴 변 파지) ~ 0.25 m(짧은 변 파지). 예: 4.05 kg × 9.81 × 0.25 ≈ **9.9 N·m** 가 그리퍼·손목에 걸림. J4–J6 effort 50 N·m 와 비교하고, 그리퍼 마찰 파지력도 점검해야 함 (판이 수직으로 매달릴 때 필요 법선력 ≈ m·g / (2μ); μ=0.3, 4.05 kg → jaw 당 약 66 N).
- 실험 조건에 **payload check 를 자동으로 넣어** 초과 조건은 "로봇 불가" 로 표시한다 (실험에서 빼지는 말고 플래그).

---

## 2. 파지 형상과 관측 가능성 (Phase 4–5 설계의 기준)

### 2.1 파지 방식
- 최대 열림 ≈ 67 mm 이므로 **300 mm / 500 mm 폭을 가로질러 잡는 것은 불가능**. 구현하지 않는다.
- 기본 파지 = **판 가장자리(edge) 를 두께 방향으로 끼워 잡기**: 두 jaw 가 판의 앞·뒷면을 10 mm 두께를 사이에 두고 누르고, 판 가장자리가 jaw 사이로 일정 깊이 들어온 상태.
  - SPEC 의 "얇은 옆면을 잡는다" 를 이렇게 해석했다. 다른 의미(예: 10 mm 옆면에 핀을 대는 방식)라면 사용자 확인 필요 (§8).
- 짧은 변(300) 쪽 파지 / 긴 변(500) 쪽 파지를 설정으로 둔다. 기본은 [가정] 긴 변 중앙.

### 2.2 그리퍼 좌표(G) 정의 (코드 전체에서 통일)
- `G.x` = jaw 가 닫히는 방향(= 판 법선), `G.z` = 손가락 방향(그리퍼 밖으로), `G.y` = jaw 폭 방향.
- 각 jaw 의 5×5 핀은 G 의 y–z 평면에 피치 12 mm 로 배치. 핀은 ±G.x 로 움직인다.

### 2.3 관측 가능 / 불가능 자유도 (판 = 균일한 직사각형 강체)
| 자유도 (G 기준) | 관측 | 단서 |
|---|---|---|
| 판 중심면 x 오프셋 (jaw 사이 치우침) | ○ 연속값 | 좌/우 jaw 핀 변위 차이 + jaw opening |
| 판 법선 기울기 (G.y·G.z 축 회전) | ○ (핀이 바닥나지 않았을 때) | 눌린 핀들의 변위 기울기(평면 피팅) |
| 삽입 깊이 (G.z 방향 가장자리 위치) | △ 피치 단위로 양자화 (≈ ±6 mm) | 눌린 핀/안 눌린 핀 경계 행 |
| 가장자리 각도 (G.x 축 회전) | △ 거칠게 (5×5 경계선 기울기) | 경계선 피팅. 양쪽 jaw 결과 결합 |
| 가장자리 따라 미끄러짐 (G.y 방향) | **✕ 관측 불가** | 가장자리가 배열(≈57 mm)보다 훨씬 길어 핀 패턴이 변하지 않음. 모서리(corner)가 배열 안에 들어올 때만 관측 가능 |
| 판 두께 | ○ (검증용) | opening − 양쪽 변위 |

- G.y 미끄러짐은 판 무게중심의 옆 방향 위치를 바꾸므로 release 자세에 영향을 준다. 추정기는 이 값을 **추정하지 않고** "unobservable" 플래그 + 사전값(파지 명령 위치, 즉 픽업 시 의도한 위치)을 쓰고, 불확실성을 그대로 결과에 남긴다. 카메라 보조가 필요한 이유로 문서화한다.
- 위 표는 가설이다. Phase 5 에서 합성 데이터로 실제 식별 가능성(야코비안 rank / 섭동 실험)을 확인해 표를 고친다.

---

## 3. 코드 구조 (제안)

```
isaac/furnace/
├── config/
│   ├── default.yaml          # 모든 치수·물성·해상도. 각 값에 source: confirmed|assumed|vendor 표기
│   ├── fluid_low.yaml        # 탐색용 저해상도
│   └── fluid_high.yaml       # 검증용 고해상도
├── assets/                   # USD 생성 스크립트 (결과 USD 는 usd/generated/furnace/ 에, gitignore)
│   ├── build_m1013.py        # eb1f4a4^ 의 urdf_to_usd_v2 + assemble 을 5.1 API 로 정리, /tmp 대신 usd/generated/
│   ├── build_mobile_base.py  # 850×600×825 캐비닛 (상판·스커트·문·손잡이·캐스터·풋·케이블 구멍), 별도 reference
│   ├── build_pin_gripper.py  # 실물 STL 시각 + 단순 충돌 + 핀 50개 prismatic(스프링/감쇠)
│   ├── build_plate.py        # metal_plate (치수·밀도 설정)
│   ├── build_furnace.py      # visual 상세 / collision 단순 분리. 벽은 여러 convex 조각
│   └── build_factory.py      # 주물공장 작업구역 (외부 자산 + 절차적 생성)
├── sim/
│   ├── fluid_backend.py      # Newton-MPM / (대안) PhysX PBD 를 같은 인터페이스로 감쌈
│   ├── splash_metrics.py     # 순수 numpy (Isaac 없이 테스트 가능)
│   ├── grasp_estimator.py    # 순수 numpy. 입력: 핀 변위·opening·관절. GT 입력 금지
│   ├── release_controller.py
│   ├── lighting.py           # 조명 프리셋 (dark_foundry, debug_bright)
│   └── cameras.py            # 4개 카메라 프리셋
├── run_scene_preview.py
├── run_fluid_plate_test.py
├── run_robot_charge_demo.py
├── run_evaluate.py
└── tests/                    # pytest: metrics, estimator (Isaac 불필요)
output/furnace/<mode>/<timestamp>/   # CSV/JSON/PNG (영상·프레임은 gitignore)
assets_external/                      # 다운로드 원본 (gitignore). 절차는 ASSETS.md
docs/furnace_charging/
├── SPEC.md  PLAN.md  ASSETS.md
├── VERSIONS.md               # Phase 0 에서 작성
└── RESULTS.md                # Phase 6 에서 작성
```

- 월드 좌표: Z up, meter, `metersPerUnit = 1.0`. 원점 = Mobile Base 바닥 투영 중심.
- 시각 품질(렌더러·텍스처·조명)과 물리 해상도(dx, dt, substep)는 서로 다른 설정 키로 분리.
- 배경 소품에는 물리를 걸지 않는다 (충돌이 필요한 것: 바닥, 용해로 벽, 판, 그리퍼, 로봇, 플랫폼만).

---

## 4. Phase 별 작업

각 Phase 는 **산출물**과 **완료 기준**이 충족돼야 끝. 막히면 막힌 이유를 §9 에 남기고 독립적인 다음 Phase 로 넘어간다 (예: 유체가 막혀도 Phase 3·4 는 진행).

### Phase 0 — 환경 조사 & 실행 가능성 (반나절 목표)
1. 호스트: `uname -m`, `nvidia-smi`, `docker images | grep -E "isaac|doosan"`, 디스크 여유.
2. 컨테이너: `cat /isaac-sim/VERSION`, `/isaac-sim/python.sh -c "import sys; print(sys.version)"`,
   `/isaac-sim/python.sh -m pip list 2>/dev/null | grep -iE "isaaclab|newton|warp|yaml|trimesh"`.
   `isaacsim.core.api` 와 `omni.isaac.core` 중 어느 쪽이 import 되는지 확인.
3. **A. 로봇·장면 로딩**: `build_m1013.py` 초안(`git show eb1f4a4^:...` 복구본 기반)으로 M1013 USD 생성 → 빈 stage 에 로드 → joint 6개·한계·link_6 확인. 공식 doosan-robot2 브랜치/커밋, URDF 생성 방법(xacro 여부) 기록.
4. **B. Newton 판–액체 실행 가능성** — 아래 순서로 시도하고 첫 번째로 되는 것을 채택:
   1. Isaac Lab 이 설치돼 있거나 설치 가능하고, **설치 버전 문서에** MPM/Newton 이 있으면 그 경로 (main 문서만 보고 판단 금지).
   2. Newton 단독 (별도 venv, `newton` + `warp`): 공식 리포의 MPM 예제를 **현재 버전 기준으로** 찾아 실행. 결과(입자·판 궤적)를 `.npz` 로 저장하고, Isaac Sim 에서는 `UsdGeom.Points` 로 **녹화된 물리 결과를 재생**해서 렌더 (재생임을 화면/문서에 명시).
   3. 대안: Isaac Sim 5.1 내장 PhysX particle(PBD) 유체 + 강체 양방향 결합. Newton 이 아님을 명시.
   - aarch64(DGX Spark) 에서 wheel/빌드 지원 여부를 반드시 확인.
5. 산출물: `docs/furnace_charging/VERSIONS.md` (OS, 드라이버, GPU, Isaac Sim, Isaac Lab, Newton, Warp, Python, doosan-robot2 커밋), 채택한 유체 백엔드와 이유.
6. 완료 기준: A 의 로봇 스크린샷 1장 + B 의 최소 예제 1개 실행 로그(또는 실패 원인).

### Phase 1 — `fluid_plate_test` (최우선 물리 검증)
- 장면: 원통 용기 + 액체 + 10 mm 강체판. 바닥·벽은 충돌 형상으로 실제 액체·판을 막아야 함.
- [가정] 용기: 내경 0.80 m, 깊이 0.70 m, 액 깊이 0.45 m (rim 아래 0.25 m). 액체 밀도·점도는 설정값 (기본: 물 수준 + 고밀도 변형). "용탕의 기계적 거동을 단순화한 등온 유체" 로만 설명.
- 판 투입 두 모드 분리:
  - `kinematic`: 판을 정해진 궤적으로 밀어 넣음 (one-way 여부 표시)
  - `free_drop`: 정지 상태에서 release → 자유 낙하·회전
- SPEC §13 검증 1–8 을 각각 자동 체크하고 JSON 으로 저장:
  1 액체 유지 / 2 판·벽 관통 없음 / 3 release 후 자유 운동 / 4 판이 액체를 밀어냄 / 5 액체 반력이 판 운동에 반영(two-way: 진입 후 판 감속 확인) / 6 입자 위치·질량·속도 읽기 / 7 reset 후 동일 초기상태 / 8 해상도·dt sweep
- 해상도: 판 두께 10 mm 대비 격자 크기. low(dx≈10 mm) / high(dx≈5 mm 이하) 를 비교하고, 판이 격자를 "빠져나가는지" 확인. 필요하면 용기 크기를 줄인 축소 시험을 먼저 한다 (축소 시험임을 표시).
- **no-impact 대조 실험**: 판 없이 같은 시간 동안 시뮬레이션 → 수치 누수량 = splash 판정의 바닥선.
- 산출물: `output/furnace/fluid_plate_test/...` (JSON, CSV, 측면 스크린샷), §9 기록.
- 완료 기준: 1–7 통과 + 8 의 sweep 표. 실패 항목은 실패라고 기록.

### Phase 2 — `splash_metrics.py` (Phase 1 과 병행 가능, Isaac 불필요)
- 입력: 시간별 입자 위치·속도·질량, 용기 기하 (원통: 내경 R, rim 높이 z_rim / 복잡 형상: inside 판정 함수).
- 지표:
  - `final_outside_mass`: 마지막 프레임에 용기 밖(r > R 또는 rim 밖으로 넘어가 아래로 떨어진 것)에 있는 질량
  - `cumulative_escaped_mass`: 한 번이라도 밖으로 나간 **고유 입자** 질량 (입자 ID 당 1회만 집계 — 중복 집계 안 함)
  - `over_rim_mass_peak`: z > z_rim 인 질량의 시간 최대값
  - `max_splash_height`: 액면 기준 최고 입자 높이 (상위 분위수도 같이 저장해 단일 입자 이상치 구분)
  - `outside_kinetic_energy`
  - `plate_inserted`, `collision_robot_or_gripper`, `task_time`
  - `leak_baseline`: 같은 설정 no-impact 결과. 보고값은 원값과 baseline 차감값을 둘 다 저장
- 시각 효과 입자는 질량 계산에 넣지 않는다.
- 완료 기준: 합성 데이터 pytest (입자 한 개가 경계를 여러 번 넘는 경우 포함) 통과.

### Phase 3 — `scene_preview` (유체와 독립, Phase 1 과 병행 가능)
1. 외부 자산 조사 (SPEC §5–6): **NVIDIA PhysicalAI_SceneAssembly.zip 을 먼저** 받아 라이선스, `metersPerUnit`, `upAxis`, reference/texture 경로 확인. 나머지 후보는 품질·라이선스 확인 후 필요한 것만. 각 자산을 [ASSETS.md](ASSETS.md) 에 기록. HDRI 는 조명 참고용으로만.
   - 다운로드는 `assets_external/<name>/` 에, 변환본은 `usd/generated/furnace/`. 재배포 불가면 절차만 문서화.
2. 용해로(`build_furnace.py`, 절차적 생성 기본): 내부 도가니 / 외부 하우징 분리, 내화재 상부 테두리, 철골 프레임·작업대, 전력 케이블·냉각수 배관 외형, 상부 집진 후드+덕트, 옆 제어반. 투입구는 판 치수·투입 경로에 맞게 (기본 내경 0.80 m 와 Phase 1 용기를 같은 치수로). 컷어웨이 금지.
   - collision: 벽을 N 개 convex 박스/조각으로 둘러 구성 (단일 convex hull 금지 — 내부가 막힘).
3. 공장 작업구역: 철골 기둥·보, 높은 천장, 주행 레일(크레인), 배관, 덕트, 제어반, 판 보관 랙, 내화벽돌 구역, 황색 안전선·난간, 사용감 있는 바닥. 공장 전체가 아니라 작업구역 우선.
4. 조명 `dark_foundry` 프리셋: 어두운 청회색 dome/ambient + 용탕 emissive 재질 + 도가니 안쪽 주황 rect/sphere light (용탕 빛이 판·그리퍼 하부·로봇·바닥에 실제로 비치게). **자동 노출 끄고 고정 노출** (RTX tonemapper 설정은 설치 버전 문서로 확인). 약한 보조광. 불꽃·연기 기본 off.
5. 카메라 4개: 전체 작업구역 / 로봇+용해로 3/4 view / 그리퍼–판 접촉 확대 / 액면 측면(또는 상부).
6. 산출물: 카메라별 실제 렌더 스크린샷 (`output/furnace/scene_preview/`), 대표 장면 1장 ("어두운 공장, Mobile Base 위 M1013 이 판을 잡고 붉은 용해로에 투입하려는 모습").
7. 완료 기준: 4개 시점 스크린샷 + ASSETS.md 채워짐.

### Phase 4 — 로봇·플랫폼·그리퍼 결합 (`robot_charge_demo` 의 기반)
1. `build_mobile_base.py`: §1.4 치수. 로봇 베이스는 [가정] 상판 중앙, 상판 위 z = 0.825 m. 별도 USD reference 로 분리하고 README·config 에 "치수는 판매 페이지 값, 장착 위치는 가정" 표시. 정적(고정) 바디.
2. M1013 (Phase 0 결과) 을 상판 위에 fixed joint 로 고정. 관절 drive(stiffness/damping)는 설정값 + 근거 기록.
3. `build_pin_gripper.py`:
   - 시각: 실물 STL (`usd/sim_gripper/`, 설계판은 §8 답변에 따름).
   - 물리: 몸체·jaw 단순 충돌 박스, jaw 2개 prismatic(0–30 mm), **핀 50개 각각 prismatic + 스프링(drive stiffness)/감쇠, 한계 0–stroke**. 핀 끝 충돌 = 작은 박스.
   - 핀 변위 = joint position 으로 읽기 (simulation ground truth). Hall 전압 모델은 후속 확장(`Estimate_5x5.py`/`Field_Model.py` 연동 가능성만 기록).
   - 성능 확인: 관절 50+개 articulation 이 안정적인지, 아니면 그리퍼를 별도 articulation 으로 두고 link_6 에 fixed joint 로 붙일지 비교.
4. `build_plate.py`: 500×300×10 mm, 밀도는 설정(후보 리스트). 이름은 `metal_plate`.
5. 파지: 마찰 기반 파지를 우선 시도. 미끄러짐·떨림으로 안 되면 "접촉 후 fixed joint 부착" 대안을 쓰되 **어느 방식인지 결과에 기록** (release 시 joint 제거 → 자유 낙하). 기존 해머링의 kinematic→dynamic 전환 패턴(`m1013_gripper_bridge.py:399-416`) 참고.
6. 도달성: 판 보관 랙 → 용해로 상부까지 IK 경로가 관절 한계·충돌 없이 가능한지 확인하고, 용해로·랙 배치를 거기에 맞춘다 (작업반경 1.3 m, 베이스 높이 0.825 m).
7. payload / 손목 토크 / 필요 파지력 자동 계산 (§1.5).
8. 완료 기준: 판을 집어 들어 용해로 위로 옮기는 영상(또는 프레임 시퀀스) + 핀 변위 로그 CSV.

### Phase 5 — `grasp_estimator.py`
- 입력: 핀 변위 50개, jaw opening, 관절 상태. **판 GT pose 는 입력 금지** (평가 코드에서만 사용).
- 출력: G 기준 판 자세 추정값 + 자유도별 `observable` 플래그 + 불확실성.
- 방법(해석/기하): 눌린 핀 → 평면 피팅(법선 기울기, x 오프셋), 눌림 경계선 피팅(삽입 깊이, 가장자리 각도). 관측 불가 자유도는 사전값 유지.
- 검증: 합성 데이터 pytest + Phase 4 시뮬 데이터로 GT 대비 오차 표. §2.3 표를 실험 결과로 갱신.
- AI/RL 은 쓰지 않는다.

### Phase 6 — release 제어 & `evaluate`
- `release_controller.py`: 목표 투입 자세(판 기준)를 추정값으로 보정해 그리퍼 목표 자세 계산 → 접근 → 감속 → jaw 열기(timing 설정).
- 비교 조건 (같은 seed, 같은 초기 파지 오차 분포):
  - A 고정 투입 (오차 무시)
  - B 핀 변위 추정 보정
  - C oracle (GT 사용, 상한 기준)
  - D 낮은 높이에서 조심스럽게 놓기 (단순 baseline)
- 탐색 변수: release height, plate pitch/roll/yaw, 수평 투입 위치, release 직전 선속도·각속도, jaw opening·해제 timing. 저해상도로 탐색 → 상위 후보만 고해상도 재검증.
- "추정 효과" 와 "낙하 높이를 낮춘 효과" 를 분리: 같은 release height 에서 A/B/C 비교 + height sweep 따로.
- 결론을 미리 정하지 않는다 (예: "기울일수록 splash 감소" 를 가정하지 않음).
- 산출물: `output/furnace/evaluate/<run>/results.csv` (조건, seed, 변수, 지표 전부), 요약 JSON, `docs/furnace_charging/RESULTS.md`.

### Phase 7 — 정리
- README 에 furnace 섹션 + 실행 명령 4개 모드.
- VERSIONS / ASSETS / 가정값 표 / 완료·미완료 / 알려진 한계 업데이트.
- 스크린샷·영상은 실제 시뮬레이터 결과만. 생성 이미지·컨셉 렌더 금지.

---

## 5. 실행 모드 (목표 명령 형태 — Phase 0 결과에 따라 조정)

```bash
docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_scene_preview.py --config isaac/furnace/config/default.yaml --headless
docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_fluid_plate_test.py --fluid low --mode free_drop
docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_robot_charge_demo.py
docker exec doosan_kos /isaac-sim/python.sh /kos_workspace/isaac/furnace/run_evaluate.py --conditions A,B,C,D --seeds 0-19
```
Newton 단독 경로를 쓰게 되면 `fluid_plate_test` / `evaluate` 의 유체 부분은 venv 에서 돌고, Isaac 쪽은 결과 재생·렌더만 담당한다. 이 경우 명령을 위와 별도로 적는다.

---

## 6. 리스크와 대응

| 리스크 | 대응 |
|---|---|
| Isaac Sim 5.1 + 설치된 Isaac Lab 에 Newton MPM 이 없음 / aarch64 미지원 | Newton 단독 venv → PhysX PBD 순서로 대안. 어떤 것을 썼는지 모든 결과에 표기 |
| 10 mm 판이 MPM 격자보다 얇아 결합이 약함 | dx sweep, 축소 용기 시험, 판 충돌 두께/SDF 방식 확인 |
| 고해상도 유체가 느림 | low/high 분리, 탐색은 low, 상위 후보만 high |
| 핀 50개 articulation 불안정 | 그리퍼 별도 articulation, substep 증가, 핀 질량·강성 조정. 최후엔 핀 변위를 접촉 깊이로 계산하는 대안(표시) |
| 마찰 파지 불안정 | fixed joint 부착 대안 (표시) |
| 재질에 따라 payload 초과 | 플래그로 표시, 실험에서 제외하지 않음 |
| M1013 USD 생성 스크립트 삭제됨 | `eb1f4a4^` 에서 복구, 5.1 API 로 정리 |
| 외부 공장 자산 라이선스 불명 | 쓰지 않고 절차적 생성으로 대체 |

---

## 7. 가정값 표 (설정 파일과 동기화할 것)

| 키 | 값 | 상태 |
|---|---|---|
| plate.size_m | [0.5, 0.3, 0.01] | 논의 중인 근삿값 (SPEC) |
| plate.density_candidates | [2700, 7140, 7870] | 예시 후보. 재질 미확정 |
| gripper.pins_per_jaw | 5×5 | 확인 (실물) |
| gripper.pin_pitch_m | 0.012 | 확인 (STL 실측) |
| gripper.pin_stroke_m | 0.012 | 확인 (스프링 행정; 논문 12.5 mm) |
| gripper.pin_spring_k / damping | — | **[가정] 미정** |
| gripper.max_opening_m | 0.067 | 확인 (명령 규약) |
| gripper.mass_kg | — | **[가정] 실측 요청** |
| gripper.mount_spacer | Ø0.080 × 0.035 m | 확인 (2026-10-26 실측) |
| base.size_m | [0.85, 0.60, 0.825] | 판매 페이지 |
| base.robot_mount_xy | 상판 중앙 | **[가정]** |
| furnace.crucible_inner_d_m | 0.80 | **[가정]** |
| furnace.crucible_depth_m | 0.70 | **[가정]** |
| furnace.melt_depth_m | 0.45 | **[가정]** |
| furnace.rim_height_from_floor_m | 1.0 | **[가정]** 도달성 확인 후 조정 |
| fluid.density / viscosity | 설정값 | **[가정]** 실제 용탕 아님 |
| grasp.edge | 긴 변 중앙 | **[가정]** |

---

## 8. 사용자에게 확인할 것 (없어도 가정값으로 진행)

1. 그리퍼 설계판: 시뮬레이션은 **v2** (리포 STL) 와 **A4** (최신, `Sim_Gripper.py` 기본) 중 어느 것? A4 면 `build_from_sim_gripper.py` 로 STL 재생성 필요.
2. 그리퍼+스페이서 총질량, 최대 파지력(모터 토크 기준), 핀 스프링 사양.
3. "얇은 옆면 파지" = §2.1 해석(판 가장자리를 두께 방향으로 끼워 잡기)이 맞는지.
4. Mobile Base 상판에서 로봇 베이스 중심 위치 (모서리로부터 거리).
5. 실제 용해로 투입구 크기·높이 (알면).

---

## 9. 진행 로그

| 날짜 | Phase | 확인한 사실 | 미확정 | 만든 결과 | 다음 검증 |
|---|---|---|---|---|---|
| 2026-11-23 | 계획 | §1 전부 | §7 [가정] 항목, §8 | PLAN.md, SPEC.md, ASSETS.md | Phase 0 |
| 2026-11-23 | 0 | x86_64·A6000·드라이버 580.65.06·디스크 여유 54 GB. Isaac Sim 5.1.0-rc.19, Isaac Lab·Newton 미설치. Isaac Lab MPM 은 main(3.0)=Isaac Sim 6.2 전용, 5.1 미지원(v2.3 문서 MPM 없음) → Newton 1.8.0 별도 venv 채택, 공식 MPM 예제 2개 실행 성공. M1013: 공식 m1013.usd 없음 → URDF(doosan-robot2 jazzy 816ecb5) 변환, joint 6·한계 URDF 와 일치·link_6 확인 (importer 가 drive stiffness 를 625 로 넣음, 1e7 설정 무시). 최소 판-액체(양방향): 판 진입 후 감속(+6.8 m/s²), 관통 없음, 액체 밀어냄 | 수치 누수 1.4 %(판 없음 대조에서도) → splash 판정 불가 상태. 판 두께/dx=0.4. reset·sweep 미시험. Isaac Sim 6.0 이미지의 Newton 확장 미실행. §8 답 없음 | `isaac/furnace/assets/build_m1013.py`, `isaac/furnace/sim/newton_plate_drop_min.py`, `plot_newton_min.py`, `output/furnace/phase0/` (m1013_check.json, m1013_home.png, newton_min*/result.json·timeseries.csv, side_view.png), VERSIONS.md | Phase 1: 누수 원인(바닥·벽 모서리, 벽 두께 vs dx) 제거, dx sweep(25/12.5 mm), reset 재현성, 원통 용기 |
| 2026-11-23 | 1 | 원통 도가니(내경 0.8 m, 벽=convex 박스 32 조각+바닥 박스)로 바꾸자 판 없는 대조의 탈출 0 % (Phase 0 누수 1.4 % 는 ground plane–박스 이음새 문제). Newton 기본 strain_basis P0 은 물이 1.5 s 에 36 % 압축됨 → `pic8` 로 1.6 % (3 s 에 1.8 %). dx 가 작을수록 미수렴 → 바닥 관통: dx 18 mm 는 150 회, 12.5 mm 는 400 회 반복 필요. solver 를 reset 때 새로 만들어야 reset 재현 (재사용 시 2.4 cm 차이). SPEC §13 1–7 통과: 탈출 0, 판 관통 0, 회전 25°, 유체 운동에너지 0.03→4.4 J(free)/24.7 J(kinematic), 진입 후 0.15 s 유체 충격량/무게 2.2, reset 차이 0.3 mm = 새 객체끼리 차이 | 판(밀도 2700)이 액면 근처에서 거의 가라앉지 않음 (1.5 s 에 3–8 cm, 단순 종단속도 추정은 ≈0.5 m/s) — 판이 격자보다 얇아(두께/dx 0.4–0.8) 생기는 수치 저항으로 보임. 해상도가 작을수록 더 가라앉음(최종 z 0.42→0.37 m) = 진입 후 운동은 해상도 수렴 안 됨. 투입 0.3 m 에서는 rim 넘침·탈출 0 | `isaac/furnace/sim/fluid_backend.py`, `run_fluid_plate_test.py`, `config/{default,fluid_low,fluid_high}.yaml`, `output/furnace/fluid_plate_test/phase1_suite/` (suite_result.json, sweep.csv, timeseries, side_view.png) | Phase 6 비교는 dx 25 mm·같은 설정 안에서 상대비교만. 진입 후 침강은 결론에 쓰지 않음 |
| 2026-11-23 | 2 | 영역: inside / penetrated(벽·바닥 두께 안, 수치 관통) / outside 분리. 고유 입자 1 회 집계, final vs cumulative 구분, baseline 차감은 원값과 함께 저장 | — | `isaac/furnace/sim/splash_metrics.py`, `tests/test_splash_metrics.py` (pytest 7 통과) | — |
