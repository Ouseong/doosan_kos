# 두산 M1013을 활용한 적응형 망치질 제어 연구 — 연구 제안서 (v3)

## 1. 한 줄 요약

**M1013 협동로봇으로 망치질 태스크를 수행함에 있어, 작업 평면을 2D로 환원하여 자유도를 단순화하고, 매 타격 사이클을 탐색-탐색-실행의 3단 구조로 구성하여 시스템 진동 응답·임팩트 위치·접촉 깊이를 사이클 내에서 적응적으로 측정하며, 비전 기반 사전 정보(못 위치 + 재료 카테고리)와 결합하여 사이클별 trajectory와 시간 의존 wrist stiffness profile을 자동 조정하는 self-adaptive impact-aware control framework.**

## 2. 연구의 동기

산업용 협동로봇이 못 박기, 두드림 작업 같은 충격 태스크를 안정적으로 수행하려면 다음 세 가지를 동시에 만족해야 한다.

- 충격으로 인한 로봇 손상 방지
- 못 박힘 깊이의 변동에 따른 적응
- 다양한 재료 위에서 일관된 성능

기존 연구는 이 중 일부만 다루며, 협동로봇(특히 두산 M1013) 위에서 통합한 사례는 거의 없다. 본 연구는 사람의 망치질 행동을 모방하여 이 세 요건을 동시에 만족하는 제어 프레임워크를 제안한다.

## 3. 핵심 Contribution 다섯 가지

### Contribution 1 — 2D 평면 환원으로 자유도 단순화

망치질은 본질적으로 위아래로 내려치는 평면 운동이다. M1013 baseline과 못이 만드는 수직 평면을 정의하고, 그 평면을 벗어나는 좌우 방향으로 작용하는 관절은 잠근다. 6자유도 로봇을 사실상 4자유도 평면 매니퓰레이터로 환원함으로써 제어 문제의 복잡도를 낮춘다. 못의 위치는 비전으로 자동 검출되며 (Contribution 5 참조), 이 위치로부터 평면이 자동 정의된다.

학술적 위치: **task-space redundancy resolution / task-constrained motion planning**의 응용. M1013은 엄밀한 redundant 로봇은 아니지만, 평면 구속을 통해 효과적으로 redundancy를 줄이는 task-specific simplification.

### Contribution 2 — "툭-툭-쾅" 3단 사이클 구조

매 타격 사이클이 "약-약-강"의 3단 구조를 가진다. 매번 툭, 툭, 쾅. 또 툭, 툭, 쾅.

- **툭1, 툭2 (탐색 타격)**: 가벼운 두 번의 탭. 세 가지 정보를 동시 수집한다.
  - 임팩트 위치 정밀 측정 (못이 어디 있는지)
  - 시스템 진동 응답 측정 (이 사이클의 강타 강도 결정용)
  - **접촉 깊이 측정** (forward kinematics 기반, Contribution 4 참조)
- **쾅 (적응적 강타)**: 위 정보를 바탕으로 trajectory와 강도가 매 사이클 다름

학술적 위치: **사이클 내 적응(intra-cycle adaptation)**. 기존 ILC가 사이클 간 학습이라면, 이 방식은 한 사이클 안에서 즉시 적응한다. **interactive perception / active sensing** 프레임워크의 망치질 적용.

### Contribution 3 — 시간 의존 Variable Wrist Impedance

각 타격 안에서 시간에 따라 wrist (joint 5) stiffness가 변조된다.

- 평소: stiff하게 잡아 정확도 확보
- 임팩트 직전: joint 5 stiffness를 낮춰 채찍 효과 + 충격 흡수
- 임팩트 직후: 점진적 stiffness 복원
- 다른 관절들: compliant mode로 충격 분산 흡수

이 stiffness profile은 탐색 탭과 강타에서 다르게 적용된다 — 탐색 탭은 깨끗한 신호를 위해 stiff하게, 강타는 충격 흡수를 위해 release를 크게.

학술적 위치: **variable impedance control**. Ti et al. (2024)이 자세 최적화까지 다뤘다면, 본 연구는 시간축 위의 stiffness 변조까지 확장한다.

### Contribution 4 — Multi-modal 기반 강성/깊이 추정

#### 4-1. Robot-Tool-Nail-Substrate 시스템의 진동 응답 기반 강성 추정

순수 재료 강성을 측정하는 것이 아니라, 시스템 전체(robot-tool-nail-substrate)의 dynamic 응답을 측정하여 다음 타격의 제어 파라미터로 직접 매핑한다. 시스템 종속적이지만 task에 대해서는 직접적인 측정값이다.

학술적 위치: **task-conditioned tactile probing**. 기존 강성 추정 연구(Krotkov 1996, At First Contact 2024 등)는 재료 분류가 목적이지만, 본 연구는 측정값을 바로 다음 행동으로 closed loop 피드백한다.

#### 4-2. Forward Kinematics 기반 Contact Depth Sensing

별도의 외부 깊이 센서 없이 로봇의 forward kinematics만으로 못 박힘 깊이를 자동 측정하는 contact-based depth sensing 메커니즘. 망치 머리가 못 머리에 접촉하는 순간의 기하학적 조건(망치 평면과 못 평면의 평행)을 이용하여, 그 순간의 joint encoder 값으로부터 못 머리의 절대 높이를 계산한다.

이 방식은 두 번의 탐색 탭을 통해 자연스럽게 달성되며, 탭이 (1) 위치 측정, (2) 진동 측정, (3) 깊이 측정의 세 기능을 동시에 수행하게 된다.

학술적 위치: **proprioceptive sensing**. 외부 센서 없이 로봇 자기 몸의 상태만으로 환경을 이해하는 접근. 비전 기반 깊이 측정 대비 calibration 부담이 없고, 인코더의 sub-mm 정밀도로 정확도가 우수하다. 산업 응용에서 비싼 센서를 줄이는 측면에서 매력적.

### Contribution 5 — Vision-based Prior Initialization

작업 시작 시 비전 시스템이 두 가지 사전 정보를 제공한다.

- **못 머리 위치 검출**: 못의 2D 픽셀 좌표 → 카메라 calibration을 통해 로봇 좌표로 변환 → baseline-못 평면 자동 정의 + joint 1 정렬
- **재료 카테고리 분류**: 작업 표면이 나무 / 금속 / 콘크리트 / 플라스틱 중 어느 거친 카테고리인가 판단 → 첫 사이클의 strike 강도와 stiffness profile에 prior로 입력

비전은 거친 카테고리만 다룬다. 같은 카테고리 안에서의 미세한 강성 차이(예: 소나무 vs 합판)는 진동 기반 추정(Contribution 4-1)이 담당. 두 채널이 명확한 hierarchy로 보완 관계를 이룬다.

학술적 위치: **vision-prior + tactile-refinement 구조**. 최근 manipulation 연구의 표준 패턴 중 하나. 본 연구는 이 구조를 망치질이라는 impact 태스크에 처음 적용.

## 4. 시스템 구성

### 하드웨어

- **로봇**: 두산 M1013 (6자유도 협동로봇)
- **End-effector**: 그리퍼 + 다양한 망치 종류 (일반 망치, 고무 망치, 작은/큰 망치)
- **센서 1 (관성/접촉)**: M1013 내장 토크 센서 + 그리퍼 부착 가속도계
  - 가속도계 1차: ADXL345 등 저가형 (5천원~1만5천원, 임팩트 시점 검출용)
  - 필요시 업그레이드: ADXL354 (4~10만원, 진동 분석 정밀도 향상)
- **센서 2 (비전)**: RGB 카메라
  - 옵션: 일반 USB 웹캠(2~5만원) 또는 산업용 RGB 카메라(10~20만원)
  - 위치: 작업 영역 위/옆 고정 거치 (그리퍼 부착 비추천 — 망치질 진동)
  - 깊이 정보 불필요 (forward kinematics로 처리)
- **계산 자원**: NVIDIA DGX Spark (GB10 Grace Blackwell, 128GB 통합 메모리)

### 소프트웨어

- **시뮬레이션**: NVIDIA Isaac Sim
- **실기 통신**: 두산 ROS 인터페이스 또는 자체 API
- **AI/제어 프레임워크**:
  - PyTorch (강성 추정 모델, vision 모델 fine-tuning)
  - YOLOv8 또는 v11 (못 머리 검출)
  - CLIP (재료 카테고리 zero-shot 분류)
  - scikit-optimize 또는 BoTorch (Bayesian optimization)
  - Crocoddyl 또는 자체 구현 (iLQR)

### 가속도계 위치

가속도계는 **그리퍼에 부착**한다. 다양한 도구를 바꿔가며 사용 가능하고, 측정 시스템이 도구에 독립적이다. 신호 품질 저하는 학습 모델이 보정.

### 카메라 위치 및 Calibration

카메라는 작업 영역을 안정적으로 보는 **고정 거치 (eye-to-hand)** 방식. 한 번 calibration 후 재사용. ChArUco 보드 같은 표준 방법으로 카메라 ↔ 로봇 좌표 변환 매트릭스를 측정.

## 5. 제어 사이클 단계별 설계

### 작업 전체 흐름

1. **작업 시작 시 1회 비전 처리**:
   - 못 머리 검출 → 평면 정의 → joint 1 회전 정렬
   - 재료 카테고리 분류 → strike 강도 prior 설정
2. **반복 사이클 (못이 충분히 박힐 때까지)**:
   - 툭1 → 툭2 → 쾅
   - 매 사이클 후 ILC가 trajectory 보정
   - 매 사이클 진동/깊이 측정으로 강타 파라미터 적응
3. **종료 조건**: 접촉 깊이 변화량이 임계치 이하 (못이 더 이상 안 박힘)

### 사이클 한 번의 흐름 (대략 1~2초)

| 단계 | 시간 (ms) | 동작 | Wrist Stiffness | 측정 |
|------|-----------|------|-----------------|------|
| 1. Wind-up | 0~300 | 망치 들기 | 높음 (stiff) | — |
| 2. 탭1 swing | 300~500 | 가볍게 내려치기 | 중간 | — |
| 3. 탭1 impact | 500 | 첫 번째 탭 | 중간 | 위치 + 진동 + **깊이** |
| 4. 탭1 recovery | 500~700 | 위치 복구 | 높음 | — |
| 5. 탭2 swing | 700~900 | 두 번째 가벼운 타격 | 중간 | — |
| 6. 탭2 impact | 900 | 두 번째 탭 | 중간 | 위치 + 진동 + **깊이** (검증) |
| 7. 탭2 recovery | 900~1100 | 위치 복구 | 높음 | — |
| 8. 강타 wind-up | 1100~1300 | 본격 타격 준비 | 높음 | — |
| 9. 강타 swing | 1300~1500 | 가속 내려치기 | 높음 | — |
| 10. Release | 1450~1500 | 임팩트 직전 | **낮음으로 전환** | — |
| 11. Impact | 1500 | 본격 타격 | 매우 낮음 | — |
| 12. Recovery | 1500~1700 | 점진적 복구 | 점진 증가 | — |

### 핵심 설계 변수 (Bayesian Optimization으로 튜닝)

- **Release timing**: 임팩트 몇 ms 전에 stiffness를 떨어뜨릴 것인가
- **Release stiffness**: 얼마나 낮출 것인가 (0%~100%)
- **Release transition shape**: 떨어지는 곡선의 모양 (선형/sigmoid/exponential)
- **Recovery timing**: 임팩트 후 얼마 만에 다시 잡을 것인가
- **탐색 탭 강도**: 본 강타의 몇 % 강도로 칠 것인가
- **탐색 탭 사이 간격**: 두 탭 사이 시간 간격

비전이 분류한 재료 카테고리에 따라 BO의 search space가 달라진다 (예: 금속이면 strike 강도 상한을 낮춤).

## 6. AI 및 제어 알고리즘 활용 계획

### 알고리즘 분류 — 무엇을 어디에 쓰는가

| 역할 | 알고리즘 | AI 분류 | 선택 근거 |
|------|----------|---------|----------|
| 못 머리 검출 (vision) | YOLOv8/v11 fine-tuning | **AI** (Computer Vision) | 매 작업 시 빠른 검출 필요, 라벨링 부담 적음 |
| 재료 카테고리 분류 (vision) | CLIP zero-shot | **AI** (Multi-modal CV) | 라벨링 불필요, 거친 분류만 필요 |
| Trajectory 큰 형태 결정 | iLQR (수치 최적화) | AI 아님 | 동역학과 cost가 명확하니 수치로 직접 풀면 효율적 |
| 핵심 timing/stiffness 파라미터 튜닝 | Bayesian Optimization (Gaussian Process) | **AI** (Probabilistic ML) | 평가 비용이 비싸므로 sample efficiency 중요 |
| 사이클 간 trajectory 보정 | ILC (Iterative Learning Control) | 회색 영역 | 같은 태스크 반복 + 측정 가능 = 전형적 적용 사례 |
| 진동 → 강성 추정 | 1D CNN 또는 MLP | **AI** (Signal processing ML) | 데이터에서 함수 학습이 핵심 |
| 임팩트 시점/접촉 검출 | Threshold + Forward Kinematics | AI 아님 | 단순 신호 처리 + 기하학 |

### Vision Model 1 — 못 머리 검출

- **입력**: 작업 영역 RGB 이미지
- **출력**: 못 머리의 2D 픽셀 좌표 → 로봇 좌표로 변환
- **모델**: YOLOv8 또는 v11을 못 머리 사진 50~100장으로 fine-tuning
- **활용**: 작업 시작 시 1회. 못 위치 → baseline-못 평면 정의 → joint 1 정렬
- **선택 근거**: 빠른 추론 (실시간 가능), 정확한 검출, 데이터 라벨링 부담 적음

### Vision Model 2 — 재료 카테고리 분류 (Zero-shot)

- **입력**: 작업 표면 근접 이미지
- **출력**: 카테고리 (나무 / 금속 / 콘크리트 / 플라스틱)
- **모델**: CLIP zero-shot. 텍스트 프롬프트("a photo of wooden surface" 등)와 이미지 임베딩의 유사도 비교
- **활용**: 작업 시작 시 1회. 카테고리 → BO search space 조정 + 첫 사이클 strike 강도 prior
- **선택 근거**: 라벨링 불필요, 거친 카테고리는 zero-shot으로 충분, 시스템 복잡도 최소화
- **한계**: 같은 카테고리 안의 미세 차이(소나무 vs 합판)는 구분 불가 → 진동 기반 추정이 보완

### Trajectory 학습 — 시뮬레이션 단계 (iLQR)

**시뮬에서 신뢰 가능한 것만 사용한다.**

시뮬에서 신뢰 가능한 것: 로봇 운동학(joint angle, end-effector 위치), 망치 머리 속도/가속도, 임팩트 timing, 충돌 감지, 운동량.

시뮬에서 신뢰 어려운 것: 정확한 임팩트 force 크기, 못 박힘 깊이, 진동 주파수 스펙트럼, 재료별 응답.

**시뮬의 역할**: 못 위치에서 망치 머리 속도(또는 운동량)를 최대화하는 reference trajectory를 iLQR로 풀어둔다. iLQR은 비선형 시스템을 매 순간 선형 근사하여 LQR을 반복 적용하는 알고리즘. 한 번 풀어둔 trajectory를 실기에서 baseline reference로 사용한다.

**왜 강화학습이 아닌가**: 망치질은 운동학이 명확하여 cost function을 직접 쓸 수 있다. trajectory optimization이 RL보다 빠르고 정확하며 해석 가능하다.

### 제어 파라미터 자동 튜닝 — Bayesian Optimization

**Bayesian Optimization (BO)** 으로 6개 핵심 파라미터를 학습한다.

**작동 원리**: 두 부분으로 구성된다.
- **Surrogate model**: Gaussian Process가 평가 데이터로부터 파라미터 → 결과 매핑을 확률적으로 모델링. 예측값과 불확실성을 동시에 추정.
- **Acquisition function**: Expected Improvement 등이 GP를 보고 다음 시도점을 결정 (exploration-exploitation 균형).

**왜 망치질에 적합한가**: 매 평가가 비싸다(못 갈고, 나무 갈고, 측정). BO는 30~100번이면 수렴. Random search 대비 100배 효율.

**비전 카테고리와의 통합**: BO의 search space가 재료 카테고리에 따라 다르게 정의된다. 금속 카테고리에서 학습한 BO 결과와 나무 카테고리에서의 결과는 별도 모델로 유지.

### 강성 추정 모델 — 신경망

**시뮬을 거치지 않고 실기에서만 학습한다.**

- 다양한 재료(소나무, 합판, MDF, 단단한 나무 등)에서 직접 데이터 수집
- 매 타격마다 진동 신호를 기록, **forward kinematics로 측정한 깊이 변화량**을 자동 라벨링
- 수백 개 sample로 작은 모델 학습

**모델 옵션**:
1. FFT 기반 features → 작은 MLP로 회귀
2. 1D CNN으로 raw 시계열 직접 처리 (성능 상한 더 높음)

### 사이클 간 보정 — ILC

**Iterative Learning Control**: $u_{k+1}(t) = u_k(t) + L \cdot e_k(t)$. 매 사이클의 임팩트 위치 에러와 못 박힘 깊이 변화를 측정하여 다음 사이클 trajectory를 보정. forward kinematics 기반 자동 측정으로 ILC가 fully automated.

### Forward Kinematics 기반 Contact Depth Sensing

탭 단계에서 망치가 못에 접촉하는 순간을 다음 신호 조합으로 검출:
- 그리퍼 가속도계의 급격한 감속 신호
- M1013 내장 토크 센서의 spike

접촉 시점의 모든 joint encoder 값을 수집 → forward kinematics로 망치 머리의 3D 위치 계산 → 그 z 좌표가 곧 현재 못 머리의 높이. 이전 사이클 대비 변화량이 못 박힘량.

**정확도**: M1013 인코더 분해능은 sub-mm 수준이므로 박힘량 측정 정확도가 매우 높음.

### DGX Spark의 역할

1. **Isaac Sim 환경 구동**: trajectory optimization 시뮬, 다양한 조건 병렬 탐색
2. **Vision 모델 학습 및 추론**: YOLO fine-tuning, CLIP 추론
3. **강성 추정 모델 학습 및 추론**
4. **BO 계산 및 ILC 업데이트**

### 제외한 방향

- **Pure 강화학습**: 안전성, sample efficiency, 학습 대상 대비 과도. Hybrid 접근으로 대체.
- **유튜브 영상 학습**: retargeting 어려움, 캘리브레이션 불가.
- **비전 기반 깊이 측정**: forward kinematics가 더 정확하고 단순.
- **세부 재료 분류 (소나무 vs 참나무 등)**: 비전으로는 부정확. 진동 채널이 담당.

## 7. 실험 설계

### 실험 1 — 2D 평면 환원의 효과

평면 구속 ON/OFF 비교. 평가지표: 수렴 속도, 안정성, joint torque 분포.

### 실험 2 — Wrist Stiffness Modulation의 효과

세 모드 비교: Stiff (고정 stiff) / Variable (제안 방식) / Fully released (임팩트 시 완전 free).
평가지표: 못 박힘 깊이, joint torque peak (각 관절별), 반복성.

### 실험 3 — 3단 사이클 vs 단순 반복

"툭-툭-쾅" 사이클 vs 매 사이클 동일한 강타만. 평가지표: 못이 정확히 박힌 비율, trajectory drift, 다양한 재료에서의 일관성.

### 실험 4 — 강성 추정의 효과

추정 ON/OFF 비교. 다양한 재료(연한~단단한)에서 못 박힘 효율과 robot 손상 위험 비교.

### 실험 5 — Bayesian Optimization 수렴성

BO의 sample efficiency 검증. Random search, grid search와 수렴 속도 비교.

### 실험 6 — Vision Prior의 효과

비전 prior(재료 카테고리) ON/OFF 비교. 첫 사이클부터 적절한 강도로 시작하는 효과 측정. 평가지표: 첫 사이클 over/under-shooting 비율, 수렴까지 사이클 수.

### 실험 7 — Forward Kinematics 깊이 측정 정확도

수동 측정(자/캘리퍼) 또는 비전 측정과의 정확도 비교 검증. 다양한 재료, 다양한 못 종류에서.

### 실험 8 — 다양한 재료 카테고리에서의 시스템 성능

나무, 금속(알루미늄), 플라스틱 등 다양한 재료에 못 박기 (재료별 적절한 못 사용). 시스템이 카테고리에 따라 어떻게 적응하는지 검증.

### 실험 9 — 다양한 도구 호환성

일반 망치, 고무 망치, 작은 망치, 큰 망치 등으로 도구를 바꿔가며 시스템 작동 검증.

## 8. 핵심 Reading List

### 직접 관련 (반드시 읽기)

- **Ti, Gao, Zhao, Calinon (2024)** — "An Optimal Control Formulation of Tool Affordance Applied to Impact Tasks" (arxiv 2402.05502).
- **Wang et al. (2020)** — "Multi-mode Trajectory Optimization for Impact-aware Manipulation" (arxiv 2006.13374).
- **Tokyo Robotics — Torobo** 사례.

### 강성 추정 관련

- **Pimpalkar, Slepyan, Thakor (2024)** — "At First Contact: Stiffness Estimation Using Vibrational Information" (arxiv 2411.18507).
- **Higashi et al.** — "Hardness Perception Based on Dynamic Stiffness in Tapping" (PMC6328787).
- **Krotkov (1996)**.

### Computer Vision 관련

- **Redmon et al.** — YOLO 시리즈. 가장 최신은 YOLOv11 (Ultralytics 문서).
- **Radford et al. (2021)** — "Learning Transferable Visual Models From Natural Language Supervision" (CLIP). zero-shot 분류의 표준.
- **Kirillov et al. (2023)** — "Segment Anything" (SAM). 필요 시 segmentation에 활용 가능.

### Bayesian Optimization 관련

- **Shahriari et al. (2016)** — "Taking the Human Out of the Loop: A Review of Bayesian Optimization".
- **BoTorch** 라이브러리 문서.

### Proprioceptive Sensing 관련

- **Wahlström et al.** — "Tactile Perception by Friction Inducing Vibrations". 접촉 진동 기반 인지.
- 두산 M1013 토크 센서 사양 문서.

### 인간 망치질 분석

- **Hammering Does Not Fit Fitts' Law** (PMC5447007).

### ILC 관련

- **Adaptive iterative learning control for robot manipulators** (ScienceDirect S0005109804000597).
- **Improving Needle Penetration via Precise Rotational Insertion Using ILC** (arxiv 2511.01256).

### Redundancy / Manipulability

- **Seraji et al.** — Configuration control 고전 논문.
- **Resolution of redundancy in robots and in a human arm** (ScienceDirect S0094114X17314374).

## 9. 앞으로 결정할 항목 체크리스트

### 단기 (실험 시작 전)

- [ ] 가속도계 모델 결정 (ADXL345로 시작 → 필요시 업그레이드)
- [ ] 그리퍼 모델 확인 (M1013 호환, 강성 충분한지)
- [ ] 망치 종류 확보 (실험에 사용할 도구 셋업)
- [ ] 못과 재료 확보 (소나무, 합판, MDF, 알루미늄 등)
- [ ] **카메라 결정 및 셋업** (USB 웹캠 vs 산업용 RGB)
- [ ] **카메라 calibration 수행** (ChArUco 보드)
- [ ] Isaac Sim 환경 구축 (M1013 URDF, 망치 모델, 못 모델)
- [ ] iLQR 라이브러리 결정 (Crocoddyl vs 자체 구현)
- [ ] BO 라이브러리 결정 (scikit-optimize vs BoTorch)
- [ ] **YOLO 학습용 못 머리 데이터셋 수집 (50~100장)**

### 중기 (시뮬 단계)

- [ ] iLQR로 reference trajectory 풀기
- [ ] Trajectory의 안정성 검증
- [ ] 임팩트 시점 검출 알고리즘 검증 (forward kinematics 깊이 측정 알고리즘 포함)
- [ ] BO 파라미터 공간 정의

### 중기 (실기 단계)

- [ ] M1013 토크 센서 신호 품질 확인
- [ ] 가속도계 부착 위치 미세 조정
- [ ] **YOLO fine-tuning 수행 및 정확도 검증**
- [ ] **CLIP zero-shot 분류 정확도 검증** (다양한 재료에서)
- [ ] **Forward kinematics 기반 깊이 측정 정확도 검증** (수동 측정과 비교)
- [ ] 강성 추정 모델 학습용 데이터셋 수집
- [ ] 안전 protocol 수립
- [ ] BO 초기 sample 수집 시작

### 장기 (논문 작성 단계)

- [ ] 평가지표 정량화 방식 확정
- [ ] Baseline 구현 (Stiff 모드, 단순 반복 모드, vision-off 모드 등)
- [ ] 통계 분석 방법 결정
- [ ] 타겟 학회/저널 결정 (IROS, ICRA, RA-L 등)

## 10. AI 콘텐츠 요약 (수혜 보고서용)

본 연구의 AI/머신러닝 핵심 요소는 다음 네 가지 영역에 걸친다.

1. **Computer Vision — Object Detection (YOLO 기반 못 머리 검출)**: 작업 영역 이미지에서 못 머리를 실시간 검출하여 평면 자동 정의. 50~100장 데이터로 fine-tuning한 YOLOv8/v11 모델 활용.

2. **Computer Vision — Multi-modal Zero-shot Classification (CLIP 기반 재료 분류)**: 작업 표면을 나무 / 금속 / 콘크리트 / 플라스틱 등 거친 카테고리로 분류. CLIP foundation model의 vision-language 임베딩을 활용한 zero-shot 분류로 라벨링 부담 없이 구현.

3. **Signal Processing ML — Deep learning 기반 강성 추정 (1D CNN)**: 진동 시계열 신호로부터 재료의 dynamic 응답을 추정하는 신경망 모델. 다양한 재료에서 수집한 raw sensor 데이터로 학습.

4. **Probabilistic ML — Bayesian Optimization (Gaussian Process)**: 6개 핵심 제어 파라미터를 sample-efficient하게 최적화. 매 평가가 비싼 실기 실험 환경에 적합.

추가로 ILC와 iLQR을 보조 제어 기법으로 활용하여 AI 기반 핵심 모듈과 통합한다. 본 연구의 contribution은 단순 알고리즘 개발이 아닌, 협동로봇 실기 환경에서 다양한 AI/제어 기법을 효과적으로 통합하여 산업 응용 가능한 시스템을 구축하는 데 있다.

특히 **vision (사전 정보) + tactile (실시간 측정)** 의 hierarchical sensor fusion 구조는 최근 manipulation 연구의 표준 패턴을 망치질이라는 impact 태스크에 적용한 사례로 의미가 있다.

## 11. Scope 관리에 대한 권고

5개 contribution을 한 논문에 다 담으면 분량과 실험량이 폭발할 수 있다. 상황에 따라 scope을 좁히는 옵션:

- **최소 scope (확실한 완성)**: Contribution 1 + 3 (2D 환원 + Variable wrist impedance). 가장 확실한 baseline 비교.
- **중간 scope**: Contribution 1 + 2 + 3 + 4. 비전 제외하고 핵심 제어에 집중.
- **현재 계획 (full scope)**: 모든 contribution 통합. 5개 영역 전체.
- **확장 scope (후속 연구)**: 위에 더해 다양한 도구 호환성, 다양한 임팩트 태스크로의 일반화.

처음에는 full scope으로 시작하되, 각 contribution을 모듈로 구현해서 필요시 빼거나 추가할 수 있게 설계할 것을 권장한다.

---

## 부록: 의사결정 기록

본 연구 설계 과정에서의 주요 결정과 그 근거.

| 결정 사항 | 선택 | 근거 |
|----------|------|------|
| Wrist 제어 방식 | Variable impedance (stiffness 변조) | 제어 가능성 + 정확도 + 해석 가능성 |
| 사이클 구조 | 매 사이클 툭-툭-쾅 반복 | 사이클 내 적응 + 사람 행동 모방 |
| 강성 측정 위치 | 시스템 응답 (재료 절대값 X) | Task에 직접적, 시스템 일관성 유지 |
| 시뮬 활용 범위 | Trajectory 학습만 | Isaac Sim contact dynamics의 한계 |
| 강성 추정 데이터 | 실기에서만 수집/학습 | Sim-to-real gap 회피 |
| 가속도계 위치 | 그리퍼 부착 | 다양한 도구 호환성 우선 |
| Trajectory 알고리즘 | iLQR (수치 최적화) | 운동학 명확, RL 대비 빠르고 해석 가능 |
| 파라미터 튜닝 알고리즘 | Bayesian Optimization | 평가 비용 비싸므로 sample efficiency 중요 |
| 강성 추정 모델 | 1D CNN 또는 MLP | 시계열 패턴 학습에 표준적 |
| 사이클 간 보정 | ILC | 같은 태스크 반복 + 측정 가능 |
| **깊이 측정 방식** | **Forward kinematics 기반 (vision X)** | **Sub-mm 정확도, calibration 부담 없음, 탭이 일석삼조 기능 수행** |
| **비전 활용 범위** | **못 위치 + 재료 카테고리 분류 (보조)** | **거친 분류만으로 prior 제공, 미세 분류는 진동이 담당** |
| **재료 분류 모델** | **CLIP zero-shot (vs fine-tuning)** | **라벨링 불필요, 거친 카테고리는 zero-shot으로 충분** |
| **못 검출 모델** | **YOLO fine-tuning (vs zero-shot)** | **빠른 추론 필요, 50장 라벨링은 부담 없음** |
| 카메라 종류 | RGB (RGB-D 불필요) | 깊이는 forward kinematics가 처리 |
| 카메라 위치 | 고정 거치 (eye-to-hand) | 망치질 진동에 흔들리지 않음 |
| AI 비중 | 4개 영역 (CV 2개 + ML 2개) | 자연스러운 통합, 수혜 조건 충분히 만족 |
| 제외한 방향 | Pure RL, 유튜브 학습, 비전 깊이 측정, 세부 재료 분류 | 각각 안전성/정확도/contribution 측면에서 부적합 |
