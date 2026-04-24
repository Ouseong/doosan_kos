#!/bin/bash
# ================================================
# DRCF 에뮬레이터 실행 (호스트에서 실행)
# ================================================

EMULATOR_IMAGE="doosanrobot/dsr_emulator:3.0.1"
MODEL="${1:-m1013}"
PORT="${2:-12345}"

# 이미 실행 중이면 종료
if docker ps --format '{{.Names}}' | grep -q "^emulator$"; then
    echo "[에뮬레이터] 이미 실행 중이에요!"
    echo "종료하려면: docker rm -f emulator"
    exit 0
fi

echo "[에뮬레이터] 시작... (모델: $MODEL, 포트: $PORT)"
docker run -d \
    --name emulator \
    --network host \
    "$EMULATOR_IMAGE" \
    "$MODEL" "$PORT"

echo "[에뮬레이터] 확인 중..."
sleep 2
ss -tlnp | grep "$PORT" && echo "[에뮬레이터] 정상 작동 중!" || echo "[에뮬레이터] 시작 실패"
