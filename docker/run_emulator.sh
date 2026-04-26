#!/bin/bash
# ================================================
# DRCF 에뮬레이터 실행 (호스트에서 실행)
# x86_64 / aarch64(QEMU) 공통
# ================================================

EMULATOR_IMAGE="doosanrobot/dsr_emulator:3.4.1"
MODEL="${1:-M1013}"
PORT="${2:-12345}"

# 이미 실행 중이면 종료
if docker ps --format '{{.Names}}' | grep -q "^emulator$"; then
    echo "[에뮬레이터] 이미 실행 중이에요!"
    echo "종료하려면: docker rm -f emulator"
    exit 0
fi

echo "[에뮬레이터] 시작... (이미지: $EMULATOR_IMAGE, 모델: $MODEL, 포트: $PORT)"
docker run -d --rm \
    --name emulator \
    --network host \
    --entrypoint /bin/bash \
    -e ROBOT_MODEL="$MODEL" \
    "$EMULATOR_IMAGE" \
    -c "cd /home/dra/Application/Simulator && ./DRCF $MODEL"

echo "[에뮬레이터] 포트 LISTEN 대기..."
for i in $(seq 1 15); do
    if ss -tln 2>/dev/null | grep -q ":$PORT"; then
        echo "[에뮬레이터] 정상 작동 중 (port $PORT)"
        exit 0
    fi
    sleep 2
done
echo "[에뮬레이터] $PORT 포트가 30초 안에 LISTEN 되지 않음. docker logs emulator 확인."
exit 1
