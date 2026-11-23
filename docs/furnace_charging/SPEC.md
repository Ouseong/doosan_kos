# 원본 요구사항 (SPEC)

> 사용자가 2026-11-23 에 제공한 요구사항 원문이다. 구현 계획과 진행 상황은 [PLAN.md](PLAN.md) 를 본다.
> 이 문서와 PLAN.md 가 다르면 이 문서의 **원칙**(추측 금지, 무료 자산, GT 은닉, 애니메이션을 물리 결과로 꾸미지 않기)이 우선한다.
> 단, SYSCON 이동 플랫폼은 원문 작성 이후 "두산로보틱스 Mobile Base(수동 이동 캐비닛)" 로 확인되었다 (PLAN.md §1.4).

---

너는 로봇 시뮬레이션 및 OpenUSD 환경 구현 엔지니어다.
아래 연구 목적에 맞는 Isaac Sim / Isaac Lab 기반 프로젝트를 실제로 구현하라.

단순한 계획이나 설명으로 끝내지 말고, 현재 컴퓨터의 설치 환경과 프로젝트 파일을 조사한 후 실행 가능한 코드, 장면, 설정 파일을 작성하고 검증하라.

## 1. 연구 배경과 최종 목표

나는 핀어레이 그리퍼의 개별 핀 변위를 Hall sensor–magnet 방식으로 측정하는 연구를 하고 있다.

최종 application은 다음과 같다.

“SYSCON 이동 플랫폼에 장착한 두산로보틱스 M1013 로봇팔과 핀어레이 그리퍼로 납작한 금속 원재료를 집어 용해로에 투입한다. 핀 변위로 물체를 잡은 위치와 자세를 추정하고, 투입 자세 및 release 조건을 보정하여 용탕 splash를 최소화한다.”

작업 대상:
- 납작한 직사각형 금속판
- 임시 치수: 500 × 300 × 10 mm
- 이 치수는 현장 실측값이 아니라 현재 논의 중인 근삿값이다.
- 재질은 아직 미확정이다.
- 아연, 알루미늄, 철 등으로 임의 확정하지 말 것.
- 물체는 용해로에 투입되어 최종적으로 완전히 녹는 원료다.
- 도금처럼 담갔다가 꺼내는 작업이 아니다.
- 잉곳인지 판재 스크랩인지는 아직 미확정이므로 코드에서는 metal_plate 또는 plate_charge처럼 중립적으로 명명한다.

처음 구현할 범위:
- 금속판을 강체로 모델링한다.
- release 이후 낙하와 액면 진입, 그 직후의 splash를 평가한다.
- 실제 용융, 응고, 열전달, 산화막, 수분에 의한 폭발까지 초기 범위에 넣지 않는다.
- 초기 유체 모델이 실제 용탕을 정확히 재현한다고 주장하지 않는다.

## 2. 사용할 플랫폼과 기본 원칙

우선순위:
1) NVIDIA Isaac Sim
2) Isaac Lab
3) Newton 기반 MPM 유체·강체 상호작용

주의:
- Newton은 공장 배경 자산이 아니라 물리 엔진이다.
- 공장 USD와 Newton 물리를 별도로 설정해야 한다.
- Isaac Sim, Isaac Lab, Newton은 버전별 지원 범위가 달라질 수 있다.
- 현재 설치된 버전과 공식 문서를 먼저 확인한다.
- main 브랜치 문서에 있는 기능을 설치된 stable 버전에서도 된다고 가정하지 않는다.
- 인터넷 예제의 존재와 내 컴퓨터에서 실행되는 것을 구분한다.
- API 이름이나 설치 옵션을 추측해서 작성하지 말고 실제 코드와 문서로 확인한다.

기존 작업을 보존한다.
- 기존 저장소가 있다면 AGENTS.md와 README를 읽는다.
- 기존 패키지나 GPU 드라이버를 무단으로 대규모 변경하지 않는다.
- 새 의존성은 가급적 별도 가상환경에 설치한다.
- 비밀번호, 토큰, 라이선스 키를 출력하지 않는다.

## 3. 비용과 자산 사용 조건

무료 자료만 사용한다.

허용:
- NVIDIA 공식 무료 배포 자산
- CC0
- CC BY 등 용도에 맞게 사용할 수 있는 무료 자산
- MIT, Apache 등 오픈소스 코드
- 직접 생성하는 Python/OpenUSD/Blender 모델

금지:
- 유료 자산 구매
- 유료 자산의 비공식 다운로드
- 무료 샘플을 완성형 전체 환경이라고 소개
- HDRI를 이동·충돌 가능한 3D 공장이라고 소개
- 렌더 이미지밖에 없는 자료를 다운로드 가능한 CAD라고 소개

자산마다 기록:
- 원본 URL
- 제작자
- 라이선스
- 다운로드 날짜
- 원본 형식
- 변환·수정 내용
- 실행 시 외부 파일 의존성

가능하면 프로젝트를 로컬 자산만으로 재현할 수 있게 구성한다.
원본 자산 재배포가 허용되지 않으면 다운로드 절차와 경로를 문서화한다.

## 4. 원하는 공장 환경

일반적인 밝고 깨끗한 물류창고가 아니라,
실제 금속 용해 작업을 하는 어두운 주물공장 내부에 가까워야 한다.

시각적 목표:
- 높은 철골 천장
- 오래 사용한 금속 패널 또는 콘크리트 벽
- 철골 기둥과 보
- 천장 크레인 또는 주행 레일
- 산업용 배관
- 집진 후드와 대형 덕트
- 제어반
- 금속판 보관대
- 내화벽돌 또는 내화재 작업 구역
- 황색 안전선과 난간
- 건조하고 사용감 있는 바닥
- 적절한 설비 배치와 로봇 접근 공간

조명:
- 전체 주변광은 어두운 청회색 계열
- 용해로 내부는 붉은 주황색 계열
- 용탕 빛이 판, 그리퍼 하부, 로봇, 바닥에 반사되도록 한다.
- 장비가 안 보일 정도로 검게 만들지 않는다.
- 용탕을 단순한 빨간 플라스틱 표면처럼 보이게 만들지 않는다.
- 발광 재질과 실제 주변 조명 효과를 적절히 구성한다.
- 자동 노출 때문에 용해로 빛이 사라지거나 화면 전체가 과노출되지 않도록 한다.
- 필요하면 약한 보조광을 사용한다.
- 과장된 불꽃, 폭발, 비현실적인 연기는 기본 장면에서 제외한다.
- 연무나 열기 표현은 선택 기능으로 두고 계산 성능을 우선한다.

아래 유료 자산은 분위기 참고용이다.
구매하거나 복제하지 말고, 비슷한 조명·재질·공간 분위기를 무료 자산과 직접 제작으로 구현한다.

Steel Mill Environment Kit:
https://www.fab.com/listings/07b3c6fc-e3b3-45cf-873d-18ddf51642ad

공장 전체를 과도하게 거대하게 만들 필요는 없다.
로봇, 원료 보관대, 용해로, 주변 설비가 보이는 작업 구역을 우선 완성한다.

## 5. NVIDIA 공식 공장 자산

우선 조사할 자료:

- NVIDIA 공식 교육용 공장 장면: https://docs.nvidia.com/learning/physical-ai/assembling-digital-twins/latest/getting-started/course-application-access.html
- 공장 장면 구성 설명: https://docs.nvidia.com/learning/physical-ai/assembling-digital-twins/latest/optimization-data/adding-props-stage.html
- 자산 ZIP: https://developer.nvidia.com/downloads/Omniverse/learning/Courses/PhysicalAI_SceneAssembly.zip
- 미리보기: https://docs.nvidia.com/learning/physical-ai/assembling-digital-twins/latest/_images/image31.png
- 설비 배치 예시: https://docs.nvidia.com/learning/physical-ai/assembling-digital-twins/latest/_images/image11.png

이 자료의 Factory.usd는 NVIDIA 공식 교육용으로 별도 배포되는 장면이다.
Isaac Sim 기본 설치에 반드시 포함돼 있다고 가정하지 않는다.

- 공식 Isaac Sim 환경 목록: https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_environments.html
- SimReady: https://docs.omniverse.nvidia.com/simready/latest/simready-central.html

교육용 장면을 쓸 경우:
- 실제 다운로드 및 압축 해제 가능 여부 확인
- 라이선스 확인
- USD reference와 texture 경로 확인
- metersPerUnit 및 up axis 확인
- Isaac 프로젝트와 단위가 다르면 올바르게 변환
- 필요한 부분만 활용하고 조명과 재질을 용해공장 분위기로 수정

## 6. 추가 무료 공장 자산 후보

다음은 후보이며 품질과 라이선스를 직접 확인해서 선별하라.
전부 다운로드하거나 전부 섞을 필요는 없다.

A. Abandoned Warehouse — Interior Scene
https://www.fab.com/listings/17932eba-c510-4cd1-b6be-2b7145f36eea
- 무료로 표시됐던 공장·창고 실내 장면
- GLB, glTF, USDZ 제공 여부 확인
- 원본에 baked lighting이 있으면 새 조명과 충돌하는지 확인

B. Abandoned Factory / Mill
https://www.fab.com/listings/82955982-4098-4735-ac4a-4b6a19ff61a2
- 무료 설비 묶음과 결합 장면
- 배관, 탱크, 제어반 등 보조 소품 후보
- 세부 묘사가 단순할 수 있으므로 실제 미리보기로 판단

C. Machine Shop and Factory Hall
https://3dassets.dev/assets/machine-shop-and-factory-hall-machine-sh-2df080ea-starter-scene
- CC0 GLB로 안내된 공장 장면
- 일반 금속 가공공장이므로 용해로 구역은 별도 추가
- 지나치게 단순하거나 원하는 분위기와 맞지 않으면 사용하지 않아도 됨

D. Factory and Production Line
https://blendswap.com/blend/18851
- CC0로 검색됐던 오래된 Blender 장면
- 실제 다운로드와 라이선스 재확인
- 구버전 재질·노드 호환성 확인

E. Industrial Workshop Foundry HDRI
https://polyhaven.com/a/industrial_workshop_foundry
- CC0
- 배경·조명 참고용 HDRI
- 실제 3D geometry 대체물로 취급하지 말 것
- 원하는 어두운 분위기에 맞지 않으면 강도를 낮추거나 제외

F. 무료 공장 모듈 샘플
https://skc-shop.itch.io/free-abandoned-factory-hall-sampler-mill-floor
- 전체 공장이 아니라 일부 모듈만 무료인 샘플
- 호환 버전, 라이선스, 실제 파일 구성 확인

## 7. MIT 프로젝트에 대한 주의

- Open Industry Project: https://github.com/Open-Industry-Project/Open-Industry-Project
- 라이선스·자산 출처: https://github.com/Open-Industry-Project/Open-Industry-Project/blob/main/THIRD_PARTY_NOTICES.md

이 프로젝트의 MIT는 라이선스 이름이다.
MIT 대학에서 만든 모델이라는 뜻이 아니다.
NVIDIA Factory.usd를 바탕으로 만든 프로젝트라고 가정하지 않는다.

이것은 별도 공장 자동화 시뮬레이터이므로
Isaac Sim에서 바로 열리는 완성형 USD 공장이라고 소개하지 않는다.

이번 목표에 도움이 되는 모델이 있을 때만 참고한다.
브라우저 시뮬레이터 전체를 Isaac으로 이식하는 일에 시간을 쓰지 않는다.

## 8. 용해로

금속판을 상부 투입구로 넣을 수 있는 산업용 용해로를 구현한다.

초기 후보:
- 상부가 열린 도가니형 또는 유도 용해로
- 내부 도가니와 외부 하우징을 분리
- 내화재 상부 테두리
- 프레임 또는 작업대
- 배관·전력 연결부 외형
- 상부 집진 후드

중요:
- 물리 모델의 용기 벽은 실제로 액체와 판을 막아야 한다.
- 렌더용 상세 geometry와 계산용 collision geometry를 분리한다.
- 오목한 용기 전체를 하나의 convex hull로 만들어 내부가 막히지 않게 한다.
  > 주석(계획 작성자): 원문이 반대로 읽힐 수 있다. 의도는 "용기를 **단일 convex hull 로 만들면 내부가 막히므로, 그렇게 하지 말 것**" 이다. 벽을 여러 convex 조각(또는 SDF/triangle mesh 충돌)으로 구성한다.
- 외벽을 잘라서 액체가 옆으로 노출되는 컷어웨이는 기본 장면에서 사용하지 않는다.
- 용탕은 상부 개구부를 통해 보이게 한다.
- 판의 치수와 투입 경로에 맞는 개구부를 확보한다.

- 무료 모델 후보: https://grabcad.com/library/hydraulic-tilting-induction-melting-box-furnace-1
- 무료 스캔 후보: https://sketchfab.com/3d-models/melting-furnace-8fd3848129f34889877c7034be0295de

후보의 실제 파일이나 라이선스를 확인하지 못하면 사용 가능하다고 단정하지 않는다.
좋은 모델을 구하지 못하면 Python/OpenUSD 또는 Blender로 직접 제작한다.

단순한 원통 하나만 놓고 완성이라고 하지 말고,
실제 산업 설비처럼 보이도록 내화재, 프레임, 후드, 배관, 제어반을 구성한다.

참고로 아래는 유료 단품이므로 구매하지 않는다:
https://www.turbosquid.com/3d-models/3d-model-induction-furnace-1299197

## 9. 두산 M1013 로봇팔

로봇은 두산로보틱스 M1013이다.

- 공식 ROS 2 저장소: https://github.com/DoosanRobotics/doosan-robot2
- 공식 제품 페이지: https://www.doosanrobotics.com/kr/product-solutions/product/m-series/m1013/

가능하면 공식 URDF/Xacro와 mesh를 사용한다.

확인할 것:
- 올바른 M1013 모델인지
- ROS 배포판과 브랜치
- URDF/Xacro 생성 방법
- mesh 경로
- 관절 축과 관절 한계
- 질량·관성
- USD 변환
- articulation 및 drive 설정
- end-effector frame
- 로봇 베이스와 플랫폼 체결 관계

M1013을 닮은 임의의 팔 모델로 대체하고 정확한 M1013이라고 하지 않는다.

물리값이 없으면:
- 누락 항목을 기록
- 초기 가정값은 설정 파일에 명시
- 확인된 제조사 값과 가정값을 구분

로봇 가반하중을 확인하고,
금속판과 그리퍼 총질량을 비교한다.
판 재질 미확정 상태에서 하나의 중량으로 확정하지 않는다.

## 10. SYSCON 이동 플랫폼

사용자는 실제로 SYSCON 이동 장치와 M1013을 결합한 장비를 사용한다.
하지만 SYSCON의 정확한 제품 모델명과 CAD는 아직 제공되지 않았다.

따라서:
- SYSCON이라는 회사명만으로 임의 제품 모델을 확정하지 않는다.
- 로컬 파일에서 실제 플랫폼 CAD나 사진을 우선 찾는다.
- 정확한 CAD가 없으면 치수 조절 가능한 임시 이동 플랫폼을 제작한다.
- 임시 모델임을 README와 설정 파일에 표시한다.
- 나중에 실제 모델로 교체하기 쉽도록 별도 USD reference로 분리한다.

초기 구성:
- 낮고 안정적인 이동 베이스
- 바퀴
- 로봇 장착 플레이트 또는 pedestal
- 비상정지 버튼과 센서의 개념적 외형

초기 실험에서는 플랫폼을 정지시킨다.
주행·도킹·SLAM은 후속 단계로 분리한다.

## 11. 핀어레이 그리퍼

사용자 고유의 핀어레이 그리퍼를 장착한다.

로컬 CAD, URDF, STL, STEP 파일이 있으면 우선 활용한다.
파일이 없다면 교체 가능한 parametric prototype을 만든다.

초기 개념:
- 두 개의 마주보는 jaw
- 각 jaw에 5×5 핀 배열을 기본값으로 설정
- 핀마다 독립적인 직선 변위
- 핀 직경, 피치, stroke, 스프링 강성, 감쇠는 설정 가능
- 실제 값이 없으면 임시값으로 표시

파지 방향:
- 넓은 앞뒷면이 아니라 사용자가 의도한 얇은 옆면을 잡는 구성을 검토한다.
- 핀 진행 방향, 판 방향, jaw 배치를 명확히 정의한다.
- 전체 500mm 또는 300mm를 가로질러 잡는 구조가 실제 그리퍼 opening에 맞는지 확인한다.
- 실제 opening이 부족한데 그림처럼 큰 폭으로 잡는 구조를 억지로 구현하지 않는다.
- 실제 그리퍼 치수가 없으면 파지 방식과 jaw opening을 설정 가능한 후보로 두고 제약을 기록한다.
- 필요하면 넓은 면 파지와 좁은 옆면 파지를 비교 가능한 설정으로 분리하되 기본 목표는 사용자가 요청한 좁은 옆면 파지다.

핀 모델:
- simulation ground truth로 각 핀의 축방향 변위를 읽을 수 있어야 한다.
- 초기 센싱은 pin displacement 기반으로 구현한다.
- Hall 전압과 자기장 간섭의 정밀 재현은 별도 확장으로 분리한다.
- 실제 측정 데이터가 없으면 Hall 신호가 정확히 재현됐다고 주장하지 않는다.

## 12. 파지 위치·자세 추정

다음 값을 구분한다.

A. 실제 상태: 시뮬레이터가 알고 있는 판의 pose
B. 관측: 핀 변위, jaw opening, 로봇 관절 상태 (필요하면 이후 카메라를 추가할 수 있음)
C. 추정: 관측만으로 계산한 그리퍼 기준 판의 위치·자세

제어 정책에 판의 ground truth pose를 몰래 입력하지 않는다.
ground truth는 성능 평가용으로만 사용한다.

중요한 관측 가능성:
- 반듯하고 균일한 판은 접촉 정보만으로 전체 위치를 유일하게 알 수 없을 수 있다.
- 어떤 이동·회전을 핀 배열로 식별할 수 있는지 분석한다.
- 관측 불가능한 자유도를 임의로 정확하게 추정하는 코드를 만들지 않는다.
- 필요하면 알려진 판 치수, 양쪽 접촉 배열, jaw opening, 파지 이력 등의 제약을 사용한다.
- 카메라 보조가 필요한 경우 그 이유를 명시한다.

초기 구현은 해석적 또는 기하학적 estimator로 시작해도 좋다.
처음부터 불필요하게 AI나 RL을 도입하지 않는다.

## 13. Newton / MPM 액체 검증

- 공식 저장소: https://github.com/newton-physics/newton
- Isaac Lab MPM 문서: https://isaac-sim.github.io/IsaacLab/main/source/concepts/using_mpm.html
- 튜토리얼: https://isaac-sim.github.io/IsaacLab/main/source/how-to/mpm_tutorials.html

참고 예제: mpm_water_dam_break, mpm_twoway_coupling, surface_reconstruction.py
예제 이름과 실행 명령은 현재 버전에서 재확인한다.

먼저 아래 최소 장면을 실행한다.
- 용기
- 액체
- 떨어지는 얇은 강체판

검증:
1) 액체가 용기에 유지되는가?
2) 10mm 두께 판이 액체나 벽을 통과하지 않는가?
3) release 후 판이 자유롭게 이동·회전하는가?
4) 판이 액체를 밀어내는가?
5) 액체 반력이 판의 움직임에 반영되는가?
6) 유체 입자의 위치·질량·속도를 읽을 수 있는가?
7) reset 후 상태가 정상적으로 복구되는가?
8) 해상도와 timestep을 바꿨을 때 결과가 어떻게 달라지는가?

특히 10mm 판에 비해 격자나 충돌 해상도가 너무 크면
판이 유체에 제대로 작용하지 않을 수 있다.
실제 지원하는 충돌·경계 처리 방식으로 확인한다.

판을 강제로 정해진 궤적으로 움직이는 경우와,
release 후 물리적으로 낙하시키는 경우를 구분한다.

액체가 한쪽으로만 판의 영향을 받는 one-way 방식이면
그 사실을 표시하고 최종 동적 낙하 검증과 혼동하지 않는다.

실시간 성능이 부족하면:
- 학습/탐색용 저해상도 설정
- 검증용 고해상도 설정
을 분리한다.

실제 molten metal에 대한 열·상변화 모델이 없으면
“용탕의 기계적 거동을 단순화한 등온 유체 실험”으로 설명한다.

## 14. Splash 평가

시각적 이펙트 입자나 후처리 spray를 실제 유체 질량으로 계산하지 않는다.

주요 지표:
- 용기 외부로 유출된 유체 질량
- 용기 rim 위로 올라간 유체 질량
- 최고 비산 높이
- 필요하면 외부 입자의 운동에너지
- 판 투입 성공 여부
- 로봇 및 그리퍼 충돌 여부
- 작업 시간

구현 주의:
- 원통형 용기라면 radius와 rim height를 사용해 내부·외부를 판정한다.
- 복잡한 용기는 내부 영역과 외부 영역을 별도로 정의한다.
- 같은 입자가 반복해서 경계를 넘을 때 중복 집계할지 명확히 한다.
- 최종 외부 질량과 누적 탈출 질량을 구분한다.
- 수치적 누수를 splash로 오인하지 않는다.
- 판을 떨어뜨리지 않는 no-impact 조건에서도 누수가 발생하는지 확인한다.

## 15. 비교 실험

동일한 초기 파지 오차와 동일한 seed에서 다음을 비교한다.

A. 파지 오차를 무시한 고정 투입
B. 핀 변위 기반 추정으로 자세를 보정한 투입
C. ground truth pose를 쓰는 oracle 기준

가능하면 낮은 높이에서 조심스럽게 놓는 단순 baseline도 추가한다.

탐색 변수:
- release height
- plate pitch / roll / yaw
- 수평 투입 위치
- release 직전 선속도
- release 직전 각속도
- jaw opening 및 해제 timing

“기울일수록 무조건 splash가 작다” 같은 결론을 미리 넣지 않는다.
실험으로 확인한다.

파지 추정의 효과와 단순히 낙하 높이를 낮춘 효과를 구분한다.

## 16. 사용자 제공 참고 링크

- 현장 참고 링크: https://www.clien.net/service/board/use/12751088
  이 링크는 이전 조사에서 접근이 차단되어 본문을 확인하지 못했다.
  접근을 시도하되, 못 읽었으면 읽었다고 하지 않는다.
  본문을 확인하지 못해도 다른 구현 작업은 계속한다.
- 잉곳 장입 동작 참고 특허: https://patents.google.com/patent/KR20100136615A/ko
  - 도면 14는 잉곳이 아니라 잉곳 바스켓
  - 도면 15는 바스켓 내부 판스프링
  - 잉곳 자체는 200번
  - 이 특허가 실제 사용자 현장의 공정과 같다고 확정하지 않는다.
  - 안전한 장입 동작과 낙하에 따른 용탕 비산 문제를 이해하는 참고 자료다.
- 실제 잉곳 용해공정 참고: https://www.ronalgroup.com/en/production/
- 얇은 판형 주괴가 존재하는 사례: https://patents.google.com/patent/KR20060092271A/ko
  이 특허의 아연 전지용 주괴는 모양 참고일 뿐이다.
  사용자 현장이 배터리 공장이라거나 원료가 아연이라고 추론하지 않는다.

## 17. 장면과 코드 구조

프로젝트에는 최소한 다음 요소를 분리한다.

- factory environment
- furnace visual asset
- furnace collision geometry
- fluid configuration
- M1013 robot asset
- mobile platform asset
- pin-array gripper asset
- metal plate asset
- grasp estimator
- release controller
- splash metrics
- experiment runner
- camera / lighting presets

추천 실행 모드:
1) scene_preview — 공장·용해로·로봇 외형과 조명 확인
2) fluid_plate_test — 판과 유체의 물리만 검증
3) robot_charge_demo — 로봇의 파지·이송·release 통합
4) evaluate — baseline과 estimator 조건 비교

시각화 품질과 물리 해상도를 독립적으로 설정할 수 있게 한다.
배경의 모든 나사와 배관에 동적 물리를 걸지 않는다.

## 18. 카메라와 결과물

최소 카메라:
- 전체 공장 작업 구역
- 로봇과 용해로를 함께 보는 3/4 view
- 그리퍼와 판의 접촉부 확대
- 액면과 splash를 보는 측면 또는 상부 시점

원하는 대표 장면:
“어두운 공장 안에서 SYSCON 위 M1013이 얇은 판을 잡고, 붉게 빛나는 용해로에 투입하려는 모습”

결과물:
- 실제 실행 가능한 코드
- 재사용 가능한 USD 또는 USD 생성 스크립트
- 의존성과 실행 명령
- 자산 출처·라이선스 목록
- 사용한 버전 목록
- 가정한 치수·물성 목록
- 실제 시뮬레이터에서 저장한 스크린샷
- 가능하면 짧은 시연 영상
- splash 평가 CSV 또는 JSON
- 구현 완료/미완료 항목과 알려진 한계

생성 이미지나 컨셉 렌더를 실제 시뮬레이션 실행 결과처럼 제시하지 않는다.

## 19. 진행 순서와 완료 기준

먼저 현재 환경을 조사하고, 아래 두 작업의 실행 가능성을 빠르게 확인한다.

A. 공식 M1013 모델과 공장 USD 로딩
B. Newton에서 얇은 판–액체 상호작용 실행

그다음:
1) 실제 물리 동작이 가능한 최소 fluid test 완성
2) 공장·용해로 외형과 조명 완성
3) 이동 플랫폼·M1013·그리퍼 결합
4) 파지 추정과 release 제어 연결
5) baseline 비교 및 결과 저장

독립적으로 진행할 수 있는 부분은 막힌 기능 때문에 중단하지 않는다.
예를 들어 유체 기능에 버전 문제가 있으면 공장 장면과 로봇 구성은 계속 완성한다.

단, 지원되지 않는 기능을 지원된다고 꾸미거나
단순 애니메이션을 물리 기반 결과라고 하지 않는다.

진행 중에는:
- 실제 확인한 사실
- 아직 미확정인 부분
- 현재 만든 결과
- 다음 검증 항목
을 짧게 알려준다.

질문은 실제 CAD, 정확한 SYSCON 모델명처럼 결과에 필수인 정보만 묻는다.
답이 없어도 가정값을 명시한 임시 모델로 독립적인 작업을 계속한다.

최종적으로 “가능하다”는 설명이 아니라
내 컴퓨터에서 실행하고 확인할 수 있는 프로젝트를 남겨라.
