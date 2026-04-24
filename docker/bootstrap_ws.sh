#!/bin/bash
# ================================================
# doosan-robot2 소스 클론 + 빌드
# 컨테이너 첫 실행 시 자동으로 실행됨
# ================================================
set -e

DOOSAN_REPO="https://github.com/doosan-robotics/doosan-robot2.git"
DOOSAN_BRANCH="humble"
DOOSAN_COMMIT="ec9242546ec6202835900dbcd8498e2daabfa6a6"
WS="/ros2_ws"

# 이미 빌드됐으면 스킵
if [ -f "$WS/install/.bootstrap_done" ]; then
    echo "[bootstrap] 이미 빌드됨 → 스킵"
    exit 0
fi

echo "[bootstrap] doosan-robot2 클론 중..."
cd "$WS/src"
if [ ! -d "doosan-robot2" ]; then
    git clone -b "$DOOSAN_BRANCH" "$DOOSAN_REPO"
    cd doosan-robot2
    git checkout "$DOOSAN_COMMIT"
    cd ..
fi

echo "[bootstrap] 의존성 설치 중..."
source /opt/ros/humble/setup.bash
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y

echo "[bootstrap] colcon build 중... (시간이 걸려요)"
colcon build --symlink-install

echo "[bootstrap] 완료!"
touch "$WS/install/.bootstrap_done"
