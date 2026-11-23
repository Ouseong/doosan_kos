# 자산 출처·라이선스 기록

규칙 (SPEC §3): 무료 자산만. 자산마다 아래 항목을 모두 채운다. 확인하지 못한 칸은 "미확인" 으로 쓰고 사용하지 않는다.
다운로드 원본은 `assets_external/<name>/` (gitignore), 변환본은 `usd/generated/furnace/` (gitignore).
재배포가 허용되지 않는 자산은 아래 "재현 절차" 에 다운로드 방법만 적는다.

## 사용 중인 자산

| 이름 | 원본 URL | 제작자 | 라이선스 | 다운로드 날짜 | 원본 형식 | 변환·수정 | 실행 시 외부 의존 | 사용 위치 |
|---|---|---|---|---|---|---|---|---|
| M1013 URDF + mesh | https://github.com/DoosanRobotics/doosan-robot2 | Doosan Robotics | 미확인 (리포 LICENSE 확인) | 컨테이너 빌드 시 | URDF/xacro, DAE | USD 변환 (`isaac/furnace/assets/build_m1013.py`) | 컨테이너의 dsr_description2 | 로봇 |
| 핀어레이 그리퍼 CAD | (사용자 자체 설계) | 사용자 | 자체 | — | STL (`usd/sim_gripper/`) | 단위 mm→m, 충돌·관절 추가 | 없음 | 그리퍼 |
| Mobile Base | (절차적 생성, 판매 페이지 치수 사용) | 이 프로젝트 | 자체 | — | Python/OpenUSD | — | 없음 | 플랫폼 |
| 용해로 | (절차적 생성) | 이 프로젝트 | 자체 | — | Python/OpenUSD | — | 없음 | 용해로 |

## 검토한 후보 (사용 여부 판단 기록)

| 후보 | URL | 확인 결과 (라이선스·형식·품질) | 사용? |
|---|---|---|---|
| NVIDIA PhysicalAI_SceneAssembly (Factory.usd) | https://developer.nvidia.com/downloads/Omniverse/learning/Courses/PhysicalAI_SceneAssembly.zip | 미확인 | |
| Abandoned Warehouse — Interior | https://www.fab.com/listings/17932eba-c510-4cd1-b6be-2b7145f36eea | 미확인 | |
| Abandoned Factory / Mill | https://www.fab.com/listings/82955982-4098-4735-ac4a-4b6a19ff61a2 | 미확인 | |
| Machine Shop and Factory Hall | https://3dassets.dev/assets/machine-shop-and-factory-hall-machine-sh-2df080ea-starter-scene | 미확인 | |
| Factory and Production Line | https://blendswap.com/blend/18851 | 미확인 | |
| Industrial Workshop Foundry HDRI | https://polyhaven.com/a/industrial_workshop_foundry | 미확인 (조명 참고용만) | |
| Free Abandoned Factory Hall Sampler | https://skc-shop.itch.io/free-abandoned-factory-hall-sampler-mill-floor | 미확인 | |
| Induction melting box furnace (GrabCAD) | https://grabcad.com/library/hydraulic-tilting-induction-melting-box-furnace-1 | 미확인 | |
| Melting furnace scan (Sketchfab) | https://sketchfab.com/3d-models/melting-furnace-8fd3848129f34889877c7034be0295de | 미확인 | |

## 재현 절차 (재배포 불가 자산)

(해당 자산이 생기면 작성)
