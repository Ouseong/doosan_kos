# 2-DOF Hammering Plan (J3 + J5 only)

Date: 2026-05-10
Status: 제안 단계, 다음 세션(lab 서버) 에서 검증

## 동기

### 교수님 피드백
- 현재 iLQR 은 J2/J3/J5 (3 DOF) 사용. "왜 관절을 저렇게 많이 쓰나?"
- 망치질 같은 단순 작업은 더 적은 DOF 로 가능할 것

### 사용자의 mechanical 직관 (★ 핵심)
임팩트 시 reaction force 는 world Z 방향 (위로). 모든 joint 축이 이 force 와 axial 로 정렬되면:
- 굽힘 토크 → 축 압축으로 변환
- bearing/gear 부담 ↓
- resonance / chatter 회피
- 산업용 robotic hammering / riveting 의 표준 practice

→ "일렬 자세 (aligned posture) + 작은 DOF 만 swing" 이 mechanical 합리적.

## 제안 — 4-stage 흐름

### Stage 1: Pickup (현재 그대로)
```
HOME → APPROACH → AT_HAMMER → CLOSE grip → LIFT
```

### Stage 2: 일렬 자세로 이동 (★ 새 단계)
모든 link 가 못 향해 vertical plane 안에서 정렬:
```python
ALIGNED_DEG = [-145.0, -90.0, 0.0, 0.0, -90.0, -90.0]
# J1 = -145°  못 방향으로 어깨 회전
# J2 = -90°   어깨 들어올려 팔이 위 향함
# J3 = 0°     팔꿈치 펴짐
# J4 = 0°     wrist roll 유지
# J5 = -90°   손목 굽혀서 hammer 아래
# J6 = -90°   gripper 정렬
```
- 가정: 이 자세에서 hammer head 가 못 (-0.65, -0.45, 0.10) 의 위쪽 ~30cm 에 위치
- **검증 필요**: `verify_fk.py` 로 hammer_strike world pos 계산해서 못 위 정렬 확인

### Stage 3: Backswing (J3, J5 offset)
일렬 자세에서 J3, J5 만 더 굽혀 망치 위로 들어올림:
```python
BACKSWING_DEG = ALIGNED_DEG.copy()
BACKSWING_DEG[2] += 30.0   # J3 +30° 팔꿈치 굽힘
BACKSWING_DEG[4] += 60.0   # J5 +60° 손목 회전
```

### Stage 4: Strike (1 초 cubic)
Backswing → 일렬 + 약간 더 (망치가 못 닿는 정확한 자세):
```python
IMPACT_DEG = ALIGNED_DEG.copy()
IMPACT_DEG[2] += -5.0
IMPACT_DEG[4] += -5.0
# 또는 IK 로 정확히 못에 닿는 J3, J5 풀이
movej('STRIKE', IMPACT_DEG, vel=300, acc=600, time=1.0)
```

## 단순화 효과

| | 현재 (iLQR 3-DOF) | 제안 (J3+J5 cubic) |
|---|---|---|
| 변동 관절 | J2, J3, J5 | J3, J5 |
| 알고리즘 | iLQR (Crocoddyl) | Cubic Hermite (또는 단일 movej) |
| 의존성 | Pinocchio + Crocoddyl | 없음 (Doosan API 만) |
| 디버깅 | 3D plot, 분기 모호성 | 2D plot, 직관적 |
| 코드 | swing_optimal.py 12k 줄 | hammer_demo.py 분기 ~30 줄 추가 |
| 임팩트 momentum | 강 (어깨~손목 lever) | 약~중 (팔꿈치~손목 lever, ~50cm) |

## Trade-off — momentum 약화

J2 (어깨) 를 락하면 가장 큰 lever arm 손실. M1013 의 link 길이:
- J2~J3 (upper arm): ~45 cm
- J3~J5 (forearm): ~40 cm
- J5~hammer head: ~25 cm

J2 사용 시 어깨에서 hammer 까지 ~110 cm lever, J3 만 사용 시 ~65 cm. **약 40% lever 손실**.

→ 작은 못 (압정/finishing nail) 충분, 큰 못 어려움. 데모 시연용으로는 OK.

## 검증 plan (lab 서버에서)

### Step 1: 일렬 자세 reachability — 5 분
```bash
~/ilqr_venv/bin/python3 scripts/ilqr/verify_fk.py -145 -90 0 0 -90 -90
# → hammer_strike world pos 출력
# → 못 (-0.65, -0.45, 0.10) 직상부 (z>0.10) 인지 확인
```
필요 시 J2 값 조정 (-90 → -85 등) 으로 hammer head 높이 맞춤.

### Step 2: 정확한 IMPACT 자세 — 10 분
J3, J5 만 자유로 두고 IK 풀이:
```python
# solve_ik_2dof(locked={J1:-145, J2:-90, J4:0, J6:-90}, target=NAIL_HEAD)
# → q_impact[2], q_impact[4] 산출
```
또는 verify_fk 로 손튜닝 (offset 조정해가며 못 닿는 위치 찾기).

### Step 3: hammer_demo_2dof.py 작성 — 30 분
기존 hammer_demo.py copy 후:
- iLQR CSV 로드 제거
- Stage 2 (ALIGN movej) 추가
- Stage 3 (BACKSWING J3, J5 offset) 추가
- Stage 4 (단일 STRIKE movej, time=1.0) 변경

### Step 4: 시뮬 검증
- Isaac Sim 에서 망치 swing 시각 확인
- Joint 락 검증 (J1, J2, J4, J6 변동 < 1°)
- hammer_strike z 단조 감소 확인

### Step 5: 실 robot 검증 (옵션)
- 모터 한계 안에서 RC_ERROR 없이 실행
- 임팩트 vel 측정 (servo 토크/vel 로그)

## Open questions

1. **J2 fixed 값 최적값**: -90° (수직) vs -75° (살짝 기울임) — 못과의 거리/각도 trade-off
2. **Backswing offset 크기**: J3 +30° / J5 +60° 가 vs 작은 swing 벅 swing — 임팩트 vel 비교
3. **단일 movej vs cubic**: time=1.0 movej 의 trapezoidal 끝 vel = 0. cubic v_end 자유 시도?
4. **2-DOF iLQR 의 필요성**: cubic 으로 부족하면 작은 OCP (state 4D) 풀 수도

## Stretch goal

만약 2-DOF 로 임팩트 약하면:
- **Hybrid**: J3 + J5 + 약간의 J2 (3-DOF, but J2 변동 작게 제한) — momentum 키우면서 일렬 자세 유사하게
- **servoj_rt 로 1초 swing** (lab 의 rt_streamer 사용) — planner 우회로 의도한 vel 정확히 실행

## Related files / refs

- 기존 데모: `scripts/hammer_demo.py` (3-DOF iLQR)
- iLQR formulation: `scripts/ilqr/swing_optimal.py`
- FK 검증 도구: `scripts/ilqr/verify_fk.py`
- RT 우회 옵션: `docs/RT_STREAMING_AND_TIMING.md`, `scripts/rt_streamer.cpp`
- 망치질 mechanical insight: 본 문서 "사용자의 mechanical 직관" 섹션
