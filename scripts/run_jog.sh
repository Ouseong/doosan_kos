#!/bin/bash
# ================================================
# M1013 시뮬 스택 부팅 + Joint Jog GUI 실행
#
# Usage:
#   bash scripts/run_jog.sh             # Isaac Sim 포함
#   bash scripts/run_jog.sh --no-isaac  # 가볍게
#
# Telemetry 는 띄우지 않음.
# 일반 USB 웹캠이 연결돼 있으면 뷰어 자동 실행.
# ================================================

set -e
source "$(dirname "$0")/_ensure_stack.sh"

# Telemetry 없이 스택 부팅
ensure_stack --no-telemetry "$@"

# ── 일반 웹캠 탐색 및 뷰어 실행 ──────────────────────────────
proj_root="$(cd "$(dirname "$0")/.." && pwd)"
gui_display="${DISPLAY:-:2}"

echo ""
echo "[cam] 일반 웹캠 탐색 중..."

# RealSense 이외의 /dev/video* 기기를 찾음.
# v4l2-ctl이 있으면 장치 이름으로 구분; 없으면 /dev/video* 순서대로 시도.
find_webcam_dev() {
    if command -v v4l2-ctl > /dev/null 2>&1; then
        local in_rs=0
        while IFS= read -r line; do
            # 장치 이름 헤더 (탭 없이 시작)
            if [[ "$line" =~ ^[^[:space:]] ]]; then
                if echo "$line" | grep -qiE "RealSense|Intel.*Depth|Intel.*RealSense"; then
                    in_rs=1
                else
                    in_rs=0
                fi
            # 장치 경로 줄 (탭/스페이스로 시작)
            elif [ "$in_rs" = "0" ] && echo "$line" | grep -qE '/dev/video'; then
                echo "$line" | tr -d '[:space:]'
                return
            fi
        done < <(v4l2-ctl --list-devices 2>/dev/null)
    else
        # v4l2-ctl 없음 — /dev/video* 중 첫 번째를 반환
        ls /dev/video* 2>/dev/null | head -1
    fi
}

webcam_dev="$(find_webcam_dev)"

if [ -z "$webcam_dev" ] || [ ! -e "$webcam_dev" ]; then
    echo "  일반 웹캠 없음 (건너뜀)"
else
    echo "  웹캠 감지: $webcam_dev"

    # 컨테이너 안에서 device가 보이면 docker 내에서 실행, 아니면 호스트에서 실행
    if docker exec doosan_kos test -e "$webcam_dev" 2>/dev/null; then
        docker exec -d doosan_kos bash -lc "
            export DISPLAY=$gui_display
            python3 /kos_workspace/scripts/cam/webcam_view.py --dev $webcam_dev \
                > /tmp/webcam.log 2>&1
        "
        echo "  ✓ 컨테이너 내 뷰어 시작됨 (q 눌러 종료)"
    else
        DISPLAY="$gui_display" python3 "$proj_root/scripts/cam/webcam_view.py" \
            --dev "$webcam_dev" &
        echo "  ✓ 호스트 뷰어 시작됨 (q 눌러 종료)"
    fi
fi

# ── D435 뷰어 (Isaac Sim 켤 때만) ───────────────────────────────
with_isaac=1
for arg in "$@"; do
    [ "$arg" = "--no-isaac" ] && with_isaac=0
done

# 플래그 파일 초기화 (이전 실행 잔재 제거)
rm -f "$proj_root/.cam_open"

if [ "$with_isaac" = "1" ]; then
    echo ""
    echo "[d435] D435 S/N:043322073704 뷰어 시작..."

    # pyrealsense2 는 시스템 python3.10 에 설치돼 있음. 셸 기본 python3 가
    # miniconda(3.13 등)로 잡히면 import 가 실패하므로, pyrealsense2 가 실제로
    # 있는 인터프리터를 골라 감지·데몬 양쪽에 같은 걸 쓴다.
    cam_py=""
    for cand in python3.10 python3 python3.11 python3.12; do
        if command -v "$cand" > /dev/null 2>&1 \
           && "$cand" -c "import pyrealsense2" > /dev/null 2>&1; then
            cam_py="$cand"; break
        fi
    done

    if [ -z "$cam_py" ]; then
        echo "  pyrealsense2 있는 python 없음 (카메라 건너뜀)"
    else
        d435_found=$("$cam_py" -c "
import pyrealsense2 as rs
serials = [d.get_info(rs.camera_info.serial_number) for d in rs.context().devices]
print('yes' if '043322073704' in serials else 'no')
" 2>/dev/null || echo "no")

        if [ "$d435_found" = "yes" ]; then
            pkill -f cam_daemon.py 2>/dev/null; sleep 0.5
            nohup "$cam_py" "$proj_root/scripts/cam/cam_daemon.py" \
                > /tmp/cam_daemon.log 2>&1 &
            disown
            echo "  ✓ 카메라 데몬 시작됨 ($cam_py) — 콘솔 3번 Open Camera 로 제어"
        else
            echo "  D435 S/N:043322073704 연결 안 됨 (건너뜀)"
        fi
    fi
fi

# ── 카메라 데몬 watchdog (Open Camera 항상 동작하게 데몬 상시 유지) ──────────
pkill -f cam_watchdog.sh 2>/dev/null
nohup "$proj_root/scripts/cam/cam_watchdog.sh" > /tmp/cam_watchdog.log 2>&1 &
disown
echo "[cam] watchdog 시작 — Open Camera 상시 동작"

# ── Dynamixel 그리퍼 연결 (어댑터 ttyUSB# 자동대응 + 노드 기동) ──────────────
bash "$proj_root/scripts/ensure_gripper.sh" || echo "[gripper] 연결 건너뜀"

echo ""
echo "✓ 스택 + Control Console 모두 부팅됨."
echo "  홈에서 3개 모드 중 선택. 각 모드 화면에서 ← Home 으로 복귀."
