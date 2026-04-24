# ================================================
# Doosan M1013 + Isaac Sim 5.1.0 + ROS2 Jazzy
# Base: nvcr.io/nvidia/isaac-sim:5.1.0 (Ubuntu 24.04 Noble 기반)
# ✅ x86_64 (서버컴) + aarch64 (DGX Spark) 모두 지원
# ================================================
FROM nvcr.io/nvidia/isaac-sim:5.1.0

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
# Isaac Sim 5.1.0 에서 user 1234 의 HOME 경로
ENV HOME=/isaac-sim

# ── root로 전환 (패키지 설치용) ──
USER root

# ── locale 설정 (ROS2 필수 요건) ──
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ── ROS2 Jazzy 설치 (Ubuntu 24.04 Noble 호환) ──
RUN apt-get update && apt-get install -y \
    curl gnupg lsb-release sudo \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
       ros-jazzy-ros-base \
       ros-jazzy-ros2-control \
       ros-jazzy-ros2-controllers \
       ros-jazzy-moveit \
       ros-jazzy-xacro \
       ros-jazzy-robot-state-publisher \
       ros-jazzy-joint-state-publisher \
       ros-jazzy-joint-state-publisher-gui \
       ros-jazzy-example-interfaces \
       python3-rosdep \
       python3-colcon-common-extensions \
       python3-argcomplete \
       python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── 기본 유틸리티 + Doosan 의존 라이브러리 설치 ──
RUN apt-get update && apt-get install -y \
    vim git tmux htop \
    libpoco-dev \
    libboost-all-dev \
    libeigen3-dev \
    libssl-dev \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# ── rosdep 초기화 (이미 초기화된 경우 무시) ──
RUN rosdep init || true \
    && rosdep update --rosdistro jazzy

# ── 인터랙티브 쉘에서도 ROS2 환경 자동 소싱 (docker exec 접속 시 대비) ──
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
    && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' \
       >> /etc/bash.bashrc

# ── ROS2 워크스페이스 + 로그 디렉토리 생성 (유저 1234 소유로 설정) ──
RUN mkdir -p /ros2_ws/src && chown -R 1234:1234 /ros2_ws \
    && mkdir -p /isaac-sim/.ros/log && chown -R 1234:1234 /isaac-sim/.ros

# ── bootstrap + entrypoint 스크립트 복사 ──
COPY bootstrap_ws.sh /usr/local/bin/bootstrap_ws.sh
COPY entrypoint.sh   /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/bootstrap_ws.sh /usr/local/bin/entrypoint.sh

# ── 작업 디렉토리 ──
RUN mkdir -p /kos_workspace && chown -R 1234:1234 /kos_workspace
WORKDIR /kos_workspace

# ── 5.1.0은 유저 1234로 실행됨 ──
USER 1234

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
