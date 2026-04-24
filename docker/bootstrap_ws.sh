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
