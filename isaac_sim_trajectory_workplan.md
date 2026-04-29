# Isaac Sim Trajectory 검증 단계 작업 계획

## 환경 정보

- **시뮬레이터**: Isaac Sim
- **연동 방식**: ROS2
- **로봇**: 두산 M1013 (URDF 이미 환경에 포함)
- **그리퍼**: 이미 환경에 포함
- **카메라**: 아직 미포함 (이 단계에서는 추가 안 함)
- **계산 자원**: NVIDIA DGX Spark
- **다음 작업 도구**: Claude Code

## 이 단계의 목표

**"Hand-crafted trajectory가 Isaac Sim에서 잘 작동하는가" 검증.**

충돌과 contact dynamics는 이 단계에서 다루지 않는다. 망치 머리가 의도한 위치에 의도한 속도로 도달하는지만 확인. 못은 시각적 마커, 망치는 그리퍼에 fixed로 부착하여 단순화.

이 단계가 끝나면:
- Trajectory 생성 코드가 작동
- 평면 구속(joint 1 정렬, 좌우 잠금)이 작동
- 툭-툭-쾅 사이클이 시간상으로 잘 흘러감
- 다양한 못 위치에 대해 일관되게 동작

다음 단계인 iLQR과 wrist stiffness modulation의 baseline이 됨.

## 작업 순서 (총 2~3주 예상)

### Step 0 — 환경 점검 (1일)

Claude Code 첫 세션에서 사용자의 현재 ROS2 + Isaac Sim 셋업을 살펴보고 다음을 확인:

- [ ] M1013 URDF 위치와 구조
- [ ] 그리퍼 부착 방식
- [ ] ROS2 토픽 구조 (joint state, joint command 등)
- [ ] Isaac Sim 실행 스크립트
- [ ] 사용 중인 controller 종류 (position, velocity, effort)

이 정보를 파악해야 다음 단계 코드를 정확히 작성 가능.

### Step 1 — 망치 모델 추가 (2~3일)

**목표**: 그리퍼가 잡고 있는 망치를 시뮬에 추가.

**구현 방법**:
- 망치를 단순 primitive로 모델링: 손잡이 (cylinder, 30cm × 2cm) + 머리 (box, 12cm × 4cm × 4cm)
- 그리퍼와 망치 사이를 fixed joint로 결합 (실기에서 그리퍼가 잡는 것을 시뮬에서 단순화)
- USD 파일로 export하거나, Python 스크립트로 런타임에 추가

**검증 항목**:
- 로봇이 움직일 때 망치가 그리퍼와 함께 움직이는가
- 망치의 무게가 적절한가 (실제 망치 ~500g)
- 시각적으로 잘 보이는가

**Claude Code에 요청할 것**:
"Isaac Sim 환경에 망치 모델을 추가하고 그리퍼에 부착하는 코드를 작성해줘. 망치는 손잡이(cylinder)와 머리(box)로 구성하고, 그리퍼 끝에 fixed joint로 부착. 무게는 500g."

### Step 2 — 못 모델 추가 (1일)

**목표**: 작업대 위에 못 마커 추가.

**구현 방법**:
- 작업대(box)를 로봇 앞에 배치
- 못을 작업대 위 cylinder로 표시 (지름 3mm, 높이 5cm)
- 못은 고정된 위치 (kinematic, 움직이지 않음)
- 충돌은 감지만 하고 물리 반응은 끄거나, 아예 collision 끄기

**검증 항목**:
- 못 위치가 코드에서 쉽게 변경 가능한가
- 시각적으로 식별 가능한가

**Claude Code에 요청할 것**:
"작업대와 못을 시뮬에 추가. 못 위치를 파라미터로 쉽게 변경할 수 있게. 충돌은 무시 (또는 detection만)."

### Step 3 — 평면 정의 및 Joint 1 정렬 (2~3일)

**목표**: 못 위치를 입력으로 받아 baseline-못 평면을 정의하고, joint 1을 그 평면에 맞춰 회전.

**구현 방법**:
- 로봇 baseline 위치와 못 위치로부터 평면 법선 계산
- Joint 1의 목표 각도 계산 (atan2 사용)
- 평면 외 방향 joint들 (M1013은 6-DOF니까 어떤 joint를 잠글지 결정 필요) 잠금
- Joint state command로 joint 1을 천천히 회전시켜 정렬

**검증 항목**:
- 다양한 못 위치 (좌우, 거리)에 대해 joint 1이 올바르게 회전하는가
- 회전 후 평면 안의 운동만으로 못에 도달 가능한가
- 잠긴 joint들이 정말로 안 움직이는가

**Claude Code에 요청할 것**:
"못 위치 (x, y, z)를 입력받아 (1) joint 1 목표 각도 계산, (2) 평면 외 joint 잠금, (3) joint 1을 부드럽게 회전시키는 함수 작성. M1013의 어떤 joint를 잠글지는 운동학 분석으로 결정."

### Step 4 — Minimum-jerk Trajectory 생성기 (3~4일)

**목표**: 시작점, 끝점, 시간을 받아 minimum-jerk trajectory를 생성하는 함수.

**핵심 수식**:
```
r(t) = 10·(t/T)³ - 15·(t/T)⁴ + 6·(t/T)⁵
position(t) = start + (end - start) · r(t)
velocity(t) = (end - start) · dr/dt
```

dr/dt도 미리 계산해두면 속도 검증에 유용.

**구현 방법**:
- Python 함수: `minimum_jerk(start, end, duration, num_steps)` → trajectory array 반환
- 작업 공간(end-effector position)에서 정의
- 매 step의 position을 IK로 풀어서 joint angle로 변환
- ROS2 토픽으로 joint command publish

**검증 항목**:
- 시작과 끝에서 속도가 0인가
- 중간 시점에서 속도 최대인가
- 부드러운가 (jerk가 작은가)

**Claude Code에 요청할 것**:
"Minimum-jerk trajectory 생성기를 작성. 입력: 시작 위치, 끝 위치, duration. 출력: 시간별 (position, velocity, acceleration). 그리고 이걸 IK로 joint angle로 변환해서 ROS2로 publish하는 함수."

### Step 5 — Keypoint 정의 및 사이클 구성 (3~4일)

**목표**: 망치질 사이클의 keypoint들을 정의하고 minimum-jerk으로 연결.

**Keypoint들**:
- `P_home`: 못 위 30cm, 망치 수직.
- `P_tap_windup`: 못 위 10cm, 약간 뒤로. (탭용 — 작은 swing)
- `P_strike_windup`: 못 위 35cm, 더 뒤로. (강타용 — 큰 swing)
- `P_impact`: 못 머리 위치 (못 위치와 동일).
- `P_recovery`: 못 위 25cm. (강타 후 들어올림)

**한 사이클의 흐름**:
```
[탭 1]: P_home → P_tap_windup → P_impact → P_home
[탭 2]: P_home → P_tap_windup → P_impact → P_home
[강타]:  P_home → P_strike_windup → P_impact → P_recovery → P_home
```

각 구간 200~300ms. 총 사이클 1.5~2초.

**검증 항목**:
- 사이클이 끝까지 잘 흐르는가
- 각 구간 사이의 transition이 부드러운가
- 망치 머리가 P_impact에 정확히 도달하는가
- 탭과 강타의 도달 속도가 차이나는가

**Claude Code에 요청할 것**:
"위에 정의한 keypoint들과 minimum-jerk trajectory 생성기를 조합해서 한 번의 망치질 사이클을 실행하는 코드. 데이터 로깅도 포함 (시간별 joint angle, end-effector position, velocity)."

### Step 6 — 검증 및 분석 (2~3일)

**목표**: 다양한 조건에서 trajectory가 일관되게 작동하는지 검증.

**테스트 시나리오**:
1. 못 위치 변화: 로봇 앞 30/50/70cm, 좌우 ±20cm
2. Trajectory 파라미터 변화: swing 시간, windup 높이
3. 사이클 반복: 같은 사이클 5번 반복 시 timing 일관성

**측정 지표**:
- P_impact 도달 정확도 (거리 오차 mm 단위)
- 도달 속도 (의도한 속도와의 오차)
- Joint angle이 한계 안에 있는가
- 사이클 간 표준편차

**Claude Code에 요청할 것**:
"검증 시나리오를 자동으로 돌리는 스크립트. 결과를 CSV/JSON으로 저장하고, matplotlib으로 시각화."

## 이 단계 후 다음 단계로

Step 6까지 완료되면 다음 중 하나로 진행:

- **iLQR 도입**: Hand-crafted를 baseline으로 두고 iLQR이 더 나은 trajectory를 찾는지 비교
- **Wrist stiffness modulation**: trajectory 위에 시간 의존 stiffness profile 얹기
- **카메라 추가**: 비전 기반 못 위치 자동 검출

## Claude Code 사용 팁

- 첫 세션에서 사용자 환경의 디렉토리 구조부터 보여주기. `tree` 명령이나 파일 탐색기로.
- 위 작업 계획서와 v3 제안서를 첨부하면 컨텍스트가 잘 전달됨.
- 작은 단위로 작업하고 자주 시뮬에서 확인. 한 번에 너무 큰 코드 변경은 디버깅이 어려움.
- 매 단계의 검증 항목을 명확히 두고 그것만 확인 후 다음 단계.

## 자주 막힐 수 있는 지점들 (미리 알아두기)

1. **그리퍼-망치 결합**: Fixed joint로 단순화하지만, 그리퍼 모델에 따라 attach 위치 잡기가 까다로울 수 있음.
2. **IK 수렴 실패**: 어떤 자세에서는 IK가 풀리지 않음. P_windup, P_impact 위치가 로봇 reachable workspace 안에 있는지 미리 확인.
3. **ROS2 latency**: Joint command publish와 실제 적용 사이 지연이 있을 수 있음. trajectory step 간격을 너무 짧게 잡으면 timing이 깨짐. 처음엔 50ms 간격 정도로 시작 권장.
4. **Joint angle 한계**: 평면 구속 후에도 특정 keypoint가 joint limit에 걸릴 수 있음. P_windup 높이를 조정하거나 자세 자체를 바꿔야 할 수 있음.
