#!/bin/bash
# ================================================
# doosan-robot2 소스 클론 + 빌드
# 컨테이너 첫 실행 시 자동으로 실행됨
# ================================================

DOOSAN_REPO="https://github.com/doosan-robotics/doosan-robot2.git"
DOOSAN_BRANCH="jazzy"
DOOSAN_COMMIT=""
WS="/ros2_ws"

# 이미 빌드됐으면 스킵
if [ -f "$WS/install/.bootstrap_done" ]; then
    echo "[bootstrap] 이미 빌드됨 → 스킵"
    exit 0
fi

echo "[bootstrap] doosan-robot2 클론 중..."
cd "$WS/src" || { echo "[bootstrap] /ros2_ws/src 접근 실패"; exit 1; }

if [ ! -d "doosan-robot2" ]; then
    git clone -b "$DOOSAN_BRANCH" "$DOOSAN_REPO" || {
        echo "[bootstrap] 클론 실패! 네트워크 연결을 확인하세요."
        exit 1
    }
    if [ -n "$DOOSAN_COMMIT" ]; then
        cd doosan-robot2
        git checkout "$DOOSAN_COMMIT" || {
            echo "[bootstrap] 경고: 커밋 체크아웃 실패, 브랜치 최신 상태로 진행"
        }
        cd ..
    fi
fi

# ----- upstream 버그 패치 -----
# DSR_ROBOT2.py 에 두 가지 업스트림 버그가 있어 예제 실행이 막힘.
# 1) 'SetSingularityHandlingForce' 오타 (srv 이름은 SetSingularHandlingForce)
# 2) _srv_name_prefix='' 인데 dsr_bringup2 는 서비스를 dsr_controller2/ 하위에 등록
DSR_PY="$WS/src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py"
if [ -f "$DSR_PY" ]; then
    if grep -q 'SetSingularityHandlingForce' "$DSR_PY"; then
        echo "[bootstrap] DSR_ROBOT2.py 패치 (1/2): SetSingularityHandlingForce → SetSingularHandlingForce"
        sed -i 's/SetSingularityHandlingForce/SetSingularHandlingForce/g' "$DSR_PY"
    fi
    if grep -qE "^_srv_name_prefix\s*=\s*''" "$DSR_PY"; then
        echo "[bootstrap] DSR_ROBOT2.py 패치 (2/2): _srv_name_prefix → 'dsr_controller2/'"
        python3 -c "
import re
p = '$DSR_PY'
with open(p) as f: s = f.read()
s = re.sub(r\"^_srv_name_prefix\s*=\s*''.*\$\",
           \"_srv_name_prefix   = 'dsr_controller2/'  # patched: bringup namespaces under dsr_controller2\",
           s, count=1, flags=re.MULTILINE)
with open(p, 'w') as f: f.write(s)
"
    fi
fi
# ------------------------------

echo "[bootstrap] 의존성 설치 중..."
source /opt/ros/jazzy/setup.bash
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

echo "[bootstrap] colcon build 중... (시간이 걸려요)"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
BUILD_RESULT=$?

if [ $BUILD_RESULT -ne 0 ]; then
    echo "[bootstrap] 빌드 실패! 위 오류를 확인하세요."
    exit 1
fi

echo "[bootstrap] 완료!"
touch "$WS/install/.bootstrap_done"
