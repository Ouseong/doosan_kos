#!/bin/bash
# ================================================
# KOS 컨테이너 관리 스크립트
# ✅ Isaac Sim 5.1.0 기준 (x86_64 + aarch64 Spark)
# 사용법: bash container.sh [start|enter|stop|clean|clean-all|status]
# ================================================

CONTAINER_NAME="doosan_kos"
IMAGE_NAME="doosan_kos:latest"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# ── 5.1.0 캐시 폴더 경로 ──
CACHE_DIR="$HOME/docker/isaac-sim"

# ── 캐시 폴더 생성 함수 ──
setup_cache() {
    echo "[KOS] 캐시 폴더 설정 중..."
    mkdir -p "$CACHE_DIR/cache/main/ov"
    mkdir -p "$CACHE_DIR/cache/main/warp"
    mkdir -p "$CACHE_DIR/cache/computecache"
    mkdir -p "$CACHE_DIR/config"
    mkdir -p "$CACHE_DIR/data/documents"
    mkdir -p "$CACHE_DIR/data/Kit"
    mkdir -p "$CACHE_DIR/logs"
    mkdir -p "$CACHE_DIR/pkg"
    # 5.1.0은 유저 1234로 실행되므로 권한 설정 필요
    sudo chown -R 1234:1234 "$CACHE_DIR" 2>/dev/null || true
    echo "[KOS] 캐시 폴더 준비 완료"
}

case "$1" in
    start)
        # 이미지 없으면 빌드
        if ! docker images | grep -q "doosan_kos"; then
            echo "[KOS] 이미지 빌드 중... (30~45분 소요)"
            docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR"
        fi

        # 이미 실행 중이면 접속
        if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[KOS] 이미 실행 중 → 접속합니다"
            docker exec -it "$CONTAINER_NAME" bash
            exit 0
        fi

        # 종료된 컨테이너 재시작
        if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[KOS] 컨테이너 재시작..."
            docker start "$CONTAINER_NAME"
            docker exec -it "$CONTAINER_NAME" bash
            exit 0
        fi

        # 캐시 폴더 준비
        setup_cache

        # X11 설정 (DISPLAY 가 설정된 경우에만)
        if [ -n "$DISPLAY" ]; then
            xhost +local: 2>/dev/null || true
        fi

        # 새 컨테이너 생성
        echo "[KOS] 새 컨테이너 시작..."
        docker run -it \
            --name "$CONTAINER_NAME" \
            --gpus all \
            --network host \
            --ipc host \
            --privileged \
            -e ACCEPT_EULA=Y \
            -e PRIVACY_CONSENT=Y \
            -e DISPLAY="${DISPLAY:-:0}" \
            -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
            -v "$HOME/.Xauthority:/isaac-sim/.Xauthority:ro" \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v "$ROOT_DIR":/kos_workspace \
            -v "$CACHE_DIR/cache/main":/isaac-sim/.cache:rw \
            -v "$CACHE_DIR/cache/computecache":/isaac-sim/.nv/ComputeCache:rw \
            -v "$CACHE_DIR/logs":/isaac-sim/.nvidia-omniverse/logs:rw \
            -v "$CACHE_DIR/config":/isaac-sim/.nvidia-omniverse/config:rw \
            -v "$CACHE_DIR/data":/isaac-sim/.local/share/ov/data:rw \
            -v "$CACHE_DIR/pkg":/isaac-sim/.local/share/ov/pkg:rw \
            -u 1234:1234 \
            "$IMAGE_NAME" \
            bash
        ;;
    enter)
        if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
            echo "[KOS] 컨테이너가 실행 중이 아니에요. 먼저 start 해주세요."
            exit 1
        fi
        docker exec -it "$CONTAINER_NAME" bash
        ;;
    stop)
        echo "[KOS] 컨테이너 중지..."
        docker stop "$CONTAINER_NAME" 2>/dev/null && echo "중지 완료" || echo "실행 중인 컨테이너 없음"
        ;;
    clean)
        echo "[KOS] 컨테이너 삭제..."
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        docker rm "$CONTAINER_NAME" 2>/dev/null && echo "삭제 완료" || echo "컨테이너 없음"
        ;;
    clean-all)
        echo "[KOS] 컨테이너 + 이미지 전체 삭제..."
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        docker rm "$CONTAINER_NAME" 2>/dev/null || true
        docker rmi "$IMAGE_NAME" 2>/dev/null || true
        echo "삭제 완료"
        ;;
    status)
        echo "=== 컨테이너 상태 ==="
        docker ps -a | grep -E "CONTAINER|doosan" || echo "관련 컨테이너 없음"
        echo ""
        echo "=== 이미지 목록 ==="
        docker images | grep -E "REPOSITORY|doosan" || echo "관련 이미지 없음"
        ;;
    *)
        echo "사용법: bash container.sh [start|enter|stop|clean|clean-all|status]"
        echo ""
        echo "  start     - 컨테이너 시작 (없으면 새로 생성)"
        echo "  enter     - 실행 중인 컨테이너 접속"
        echo "  stop      - 컨테이너 중지"
        echo "  clean     - 컨테이너 삭제 (이미지 유지)"
        echo "  clean-all - 컨테이너 + 이미지 전부 삭제"
        echo "  status    - 현재 상태 확인"
        ;;
esac
