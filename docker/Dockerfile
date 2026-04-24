# ================================================
# Doosan M1013 + Isaac Sim 5.1.0 + ROS2 Humble
# Base: nvcr.io/nvidia/isaac-sim:5.1.0
# ✅ x86_64 (서버컴) + aarch64 (DGX Spark) 모두 지원
# ================================================
FROM nvcr.io/nvidia/isaac-sim:5.1.0

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# ── root로 전환 (패키지 설치용) ──
USER root

# ── ROS2 Humble 설치 ──
RUN apt-get update && apt-get install -y \
    curl gnupg lsb-release sudo \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
       ros-humble-ros-base \
       ros-humble-ros2-control \
       ros-humble-ros2-controllers \
       ros-humble-moveit \
       ros-humble-joint-state-publisher \
       python3-rosdep \
       python3-colcon-common-extensions \
       python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# ── 기본 유틸리티 설치 ──
RUN apt-get update && apt-get install -y \
    vim git tmux htop \
    && rm -rf /var/lib/apt/lists/*

# ── rosdep 초기화 ──
RUN rosdep init && rosdep update

# ── ROS2 워크스페이스 생성 (유저 1234가 쓸 수 있도록 권한 설정) ──
RUN mkdir -p /ros2_ws/src && chmod -R 777 /ros2_ws

# ── bootstrap + entrypoint 스크립트 복사 ──
COPY bootstrap_ws.sh /usr/local/bin/bootstrap_ws.sh
COPY entrypoint.sh   /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/bootstrap_ws.sh /usr/local/bin/entrypoint.sh

# ── 작업 디렉토리 ──
RUN mkdir -p /kos_workspace && chmod -R 777 /kos_workspace
WORKDIR /kos_workspace

# ── 5.1.0은 유저 1234로 실행됨 ──
USER 1234

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
