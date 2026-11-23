# 자산 출처·라이선스 기록

규칙 (SPEC §3): 무료 자산만. 자산마다 아래 항목을 모두 채운다. 확인하지 못한 칸은 "미확인" 으로 쓰고 사용하지 않는다.
다운로드 원본은 `assets_external/<name>/` (gitignore), 변환본은 `usd/generated/furnace/` (gitignore).
재배포가 허용되지 않는 자산은 아래 "재현 절차" 에 다운로드 방법만 적는다.

## 사용 중인 자산

| 이름 | 원본 URL | 제작자 | 라이선스 | 다운로드 날짜 | 원본 형식 | 변환·수정 | 실행 시 외부 의존 | 사용 위치 |
|---|---|---|---|---|---|---|---|---|
| M1013 URDF + mesh | https://github.com/doosan-robotics/doosan-robot2 (jazzy, 커밋 816ecb5) | Doosan Robotics | Apache-2.0 (리포 LICENSE 확인) | 2026-11-23 (doosan_kos 컨테이너 /ros2_ws/src 에서 복사) | URDF(`dsr_description2/urdf/m1013.urdf`), DAE (m1013_white, m1013_collision) | package:// → 절대경로로 바꿔 URDF importer 로 USD 변환 (`isaac/furnace/assets/build_m1013.py`) | `assets_external/doosan-robot2/` (gitignore, SOURCE.txt 에 커밋 기록) | 로봇 |
| 핀어레이 그리퍼 CAD | (사용자 자체 설계) | 사용자 | 자체 | — | STL (`usd/sim_gripper/`) | 단위 mm→m, 충돌·관절 추가 | 없음 | 그리퍼 |
| Mobile Base | (절차적 생성, 판매 페이지 치수 사용) | 이 프로젝트 | 자체 | — | Python/OpenUSD (`isaac/furnace/assets/build_mobile_base.py`) | 임시 모델: 장착 위치·캐스터·풋 가정 | 없음 | 플랫폼 |
| 용해로 | (절차적 생성) | 이 프로젝트 | 자체 | — | Python/OpenUSD (`build_furnace.py`) | 시각/충돌 분리, 충돌 = Newton 용기와 같은 치수 | 없음 | 용해로 |
| 주물공장 작업구역 | (절차적 생성) | 이 프로젝트 | 자체 | — | Python/OpenUSD (`build_factory.py`) | 기둥·크레인·벽·배관·덕트·내화벽돌·안전선·랙·소품 | 없음 | 배경 |
| 텍스처 (콘크리트·벽돌·내화재·금속 패널·녹·위험 줄무늬) | (절차적 생성, PIL) | 이 프로젝트 | 자체 | — | PNG (`usd/generated/furnace/textures/`, gitignore) | `usd_util.texture()` 가 노이즈로 생성 | 없음 | 재질 |

| Newton / Warp / MuJoCo (물리 엔진, 자산 아님) | https://pypi.org/project/newton/ | NVIDIA 외 (newton-physics) / Google DeepMind (mujoco) | Apache-2.0 (newton, warp-lang, mujoco 패키지 메타데이터) | 2026-11-23 | pip wheel | 없음 | `~/venvs/furnace_newton` | 유체 백엔드 |

## 검토한 후보 (사용 여부 판단 기록)

| 후보 | URL | 확인 결과 (라이선스·형식·품질) | 사용? |
|---|---|---|---|
| NVIDIA PhysicalAI_SceneAssembly (Factory.usd) | https://developer.nvidia.com/downloads/Omniverse/learning/Courses/PhysicalAI_SceneAssembly.zip | 2026-11-23 다운로드·압축목록 확인 (752 MB, `assets_external/nvidia_scene_assembly/`). ZIP 안에 LICENSE/README 없음. 강좌 페이지 2곳에도 자산 라이선스·재배포 조항 없음 (사이트 공통 footer 만). 내용 = 전자(PCB) 조립 라인 기계·공장 (주물공장 아님). 강좌는 Z-up·metersPerUnit 0.001(mm) 작업 설정을 안내 (파일 메타데이터는 미확인) | **아니오** — 라이선스 미확인 + 용도 불일치 |
| Abandoned Warehouse — Interior | https://www.fab.com/listings/17932eba-c510-4cd1-b6be-2b7145f36eea | 미확인 (Fab 로그인 필요, 자동 다운로드 시도 안 함) | 아니오 |
| Abandoned Factory / Mill | https://www.fab.com/listings/82955982-4098-4735-ac4a-4b6a19ff61a2 | 미확인 (확인 안 함 — 절차적 생성으로 대체) | 아니오 |
| Machine Shop and Factory Hall | https://3dassets.dev/assets/machine-shop-and-factory-hall-machine-sh-2df080ea-starter-scene | 미확인 (확인 안 함 — 절차적 생성으로 대체) | 아니오 |
| Factory and Production Line | https://blendswap.com/blend/18851 | 미확인 (확인 안 함 — 절차적 생성으로 대체) | 아니오 |
| Industrial Workshop Foundry HDRI | https://polyhaven.com/a/industrial_workshop_foundry | 미확인 (사용 안 함, 조명은 직접 구성) | 아니오 |
| Free Abandoned Factory Hall Sampler | https://skc-shop.itch.io/free-abandoned-factory-hall-sampler-mill-floor | 미확인 (확인 안 함 — 절차적 생성으로 대체) | 아니오 |
| Induction melting box furnace (GrabCAD) | https://grabcad.com/library/hydraulic-tilting-induction-melting-box-furnace-1 | 미확인 (확인 안 함 — 절차적 생성으로 대체) | 아니오 |
| Melting furnace scan (Sketchfab) | https://sketchfab.com/3d-models/melting-furnace-8fd3848129f34889877c7034be0295de | 미확인 (확인 안 함 — 절차적 생성으로 대체) | 아니오 |

## 재현 절차 (재배포 불가 자산)

- M1013 mesh: `doosan_kos` 컨테이너가 있으면 `docker cp doosan_kos:/ros2_ws/src/doosan-robot2/dsr_description2/{urdf/m1013.urdf,meshes/m1013_white,meshes/m1013_collision,package.xml} assets_external/doosan-robot2/dsr_description2/...`. 없으면 `git clone -b jazzy https://github.com/doosan-robotics/doosan-robot2.git && git checkout 816ecb5` 후 같은 파일 복사.
