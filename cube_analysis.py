#!/usr/bin/env python3
"""
Eye-in-Hand 블루 큐브 분석 파이프라인
- RealSense D435 원본 프레임 캡처
- 파란 큐브 검출 + 중심 좌표
- 그리퍼 상태 판별 (Open/Closed)
- 테두리 선명도 수치화 (Laplacian Variance)
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# ── 설정 ──────────────────────────────────────────────────────────────────────
FRAME_W, FRAME_H = 640, 480

# 파란 큐브 HSV 범위 (조명 조건에 따라 조정)
BLUE_LOW  = np.array([ 90,  80,  50])
BLUE_HIGH = np.array([140, 255, 255])

# 그리퍼 판별용 좌우 ROI 너비 (픽셀)
GRIPPER_ROI_WIDTH = 120

# 그리퍼 닫힘 판단 임계값: ROI 내 비-흑색 픽셀 비율
GRIPPER_CLOSED_THRESHOLD = 0.15

OUTPUT_DIR = "output/cube_analysis"


# ── 1. 카메라 초기화 ──────────────────────────────────────────────────────────
def init_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, FRAME_W, FRAME_H, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, FRAME_W, FRAME_H, rs.format.z16,  30)
    pipeline.start(config)
    # 자동 노출 안정화 대기
    for _ in range(30):
        pipeline.wait_for_frames()
    print(f"[camera] 초기화 완료 ({FRAME_W}x{FRAME_H})")
    return pipeline


# ── 2. 원본 프레임 캡처 ───────────────────────────────────────────────────────
def capture_frame(pipeline):
    """D435 원본 해상도 그대로 캡처 — 스크린샷 왜곡 없음."""
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        raise RuntimeError("프레임 획득 실패")
    img = np.asanyarray(color_frame.get_data())
    assert img.shape[:2] == (FRAME_H, FRAME_W), f"해상도 불일치: {img.shape}"
    return img


# ── 3. 파란 큐브 검출 ─────────────────────────────────────────────────────────
def detect_blue_cube(img):
    """
    HSV 마스킹 → 외곽선 → 가장 큰 사각형 컨투어 반환.
    Returns: (bbox, mask, contour) or (None, mask, None)
      bbox = (x, y, w, h)
    """
    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)

    # 노이즈 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask, None

    # 가장 큰 컨투어 선택 (최소 면적 필터)
    best = max(contours, key=cv2.contourArea)
    if cv2.contourArea(best) < 500:
        return None, mask, None

    bbox = cv2.boundingRect(best)   # (x, y, w, h)
    return bbox, mask, best


# ── 4. 그리퍼 상태 판별 ───────────────────────────────────────────────────────
def detect_gripper_state(img):
    """
    좌우 ROI에서 비-흑색 픽셀 비율로 핀 어레이 존재 여부 판단.
    핀이 화면을 많이 가리면 → Closed, 가장자리로 물러나면 → Open.

    Returns: 'CLOSED' | 'OPEN', (left_ratio, right_ratio)
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    left_roi  = gray[:, :GRIPPER_ROI_WIDTH]
    right_roi = gray[:, FRAME_W - GRIPPER_ROI_WIDTH:]

    # 밝은 픽셀(핀 어레이 반사) 비율
    left_ratio  = float(np.sum(left_roi  > 40)) / left_roi.size
    right_ratio = float(np.sum(right_roi > 40)) / right_roi.size
    avg_ratio   = (left_ratio + right_ratio) / 2.0

    state = 'CLOSED' if avg_ratio > GRIPPER_CLOSED_THRESHOLD else 'OPEN'
    return state, (left_ratio, right_ratio)


# ── 5. 테두리 선명도 측정 ─────────────────────────────────────────────────────
def measure_edge_sharpness(img, bbox, contour):
    """
    큐브 외곽선 근방의 Laplacian Variance로 선명도 수치화.

    원리:
      Laplacian = 2차 미분 → 선명한 에지는 큰 분산, 흐릿한 에지는 작은 분산.
      그리퍼 OPEN(빛 번짐) → 낮은 값 (Soft Edge)
      그리퍼 CLOSED(핀홀 효과) → 높은 값 (Sharp Edge)

    Returns: {
        'laplacian_var': float,   # 주요 선명도 지표
        'gradient_mean': float,   # Sobel 그라디언트 평균
        'edge_mask_px':  int,     # 에지 픽셀 수
    }
    """
    x, y, w, h = bbox

    # 큐브 ROI (약간 패딩 추가하여 에지 영역 포함)
    pad = 10
    x1 = max(0, x - pad);  y1 = max(0, y - pad)
    x2 = min(FRAME_W, x + w + pad);  y2 = min(FRAME_H, y + h + pad)
    roi = cv2.cvtColor(img[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)

    # 컨투어 에지 마스크 (외곽선 5px 띠)
    edge_mask = np.zeros_like(roi)
    shifted_contour = contour - np.array([[x1, y1]])
    cv2.drawContours(edge_mask, [shifted_contour], -1, 255, thickness=5)

    # Laplacian Variance (선명도 핵심 지표)
    lap   = cv2.Laplacian(roi, cv2.CV_64F)
    lap_roi = lap[edge_mask > 0]
    lap_var = float(np.var(lap_roi)) if len(lap_roi) > 0 else 0.0

    # Sobel Gradient Magnitude
    gx = cv2.Sobel(roi, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(roi, cv2.CV_64F, 0, 1, ksize=3)
    grad_mag = np.sqrt(gx**2 + gy**2)
    grad_roi = grad_mag[edge_mask > 0]
    grad_mean = float(np.mean(grad_roi)) if len(grad_roi) > 0 else 0.0

    return {
        'laplacian_var': lap_var,
        'gradient_mean': grad_mean,
        'edge_mask_px':  int(np.sum(edge_mask > 0)),
    }


# ── 6. 픽셀 → 실제 크기 역산 (핀홀 카메라 기하학) ────────────────────────────
def compute_geometry(pixel_width, frame_width, z_mm,
                     fx_px=None, known_size_mm=50.0):
    """
    핀홀 모델: w_px / f_px = W_mm / Z_mm
      → W_mm = w_px * Z_mm / f_px  (실제 크기 추정)
      → Z_mm = W_mm * f_px / w_px  (깊이 역산, 실제 크기 알 때)

    fx_px: 초점거리(픽셀). None이면 센서 크기로 근사.
    known_size_mm: 큐브 실제 크기 (50mm).

    Returns: {
        'estimated_size_mm': float,   # 픽셀로 추정한 큐브 크기
        'estimated_z_mm':    float,   # 알려진 크기로 역산한 깊이
        'pixel_ratio':       float,   # w_px / frame_width (정규화)
    }
    """
    # D435 1280x720 기본 초점거리 근사
    if fx_px is None:
        fx_px = 0.85 * frame_width   # ≈ 1088 px (D435 수평 FOV 69° 기준)

    pixel_ratio       = pixel_width / frame_width
    estimated_size_mm = pixel_width * z_mm / fx_px
    estimated_z_mm    = known_size_mm * fx_px / pixel_width if pixel_width > 0 else 0.0

    return {
        'estimated_size_mm': estimated_size_mm,
        'estimated_z_mm':    estimated_z_mm,
        'pixel_ratio':       pixel_ratio,
    }


# ── 7. 결과 시각화 ────────────────────────────────────────────────────────────
def draw_results(img, bbox, contour, gripper_state, sharpness, geo, z_mm):
    vis = img.copy()

    if bbox is not None:
        x, y, w, h = bbox
        cx, cy = x + w // 2, y + h // 2

        # 바운딩박스 + 중심
        color = (0, 255, 0) if gripper_state == 'CLOSED' else (0, 165, 255)
        cv2.rectangle(vis, (x, y), (x+w, y+h), color, 2)
        cv2.drawContours(vis, [contour], -1, (255, 255, 0), 1)
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(vis, f"cx={cx} cy={cy}", (x, y-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # 그리퍼 ROI 선
    cv2.line(vis, (GRIPPER_ROI_WIDTH, 0),
             (GRIPPER_ROI_WIDTH, FRAME_H), (200, 200, 0), 1)
    cv2.line(vis, (FRAME_W - GRIPPER_ROI_WIDTH, 0),
             (FRAME_W - GRIPPER_ROI_WIDTH, FRAME_H), (200, 200, 0), 1)

    # 정보 오버레이
    lines = [
        f"Z = {z_mm} mm",
        f"Gripper: {gripper_state}",
        f"Laplacian Var: {sharpness['laplacian_var']:.1f}",
        f"Gradient Mean: {sharpness['gradient_mean']:.1f}",
    ]
    if bbox is not None:
        lines += [
            f"Cube px_w: {bbox[2]}  ratio: {geo['pixel_ratio']:.4f}",
            f"Est. size: {geo['estimated_size_mm']:.1f} mm",
            f"Est. Z:    {geo['estimated_z_mm']:.1f} mm",
        ]
    for i, txt in enumerate(lines):
        cv2.putText(vis, txt, (10, 25 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, txt, (10, 25 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    return vis


# ── 메인 ──────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    pipeline = init_camera()

    print("\n[큐브 분석 시작]  'c' = 캡처+분석 / 'q' = 종료\n")

    try:
        while True:
            img = capture_frame(pipeline)

            # 실시간 미리보기
            preview = cv2.resize(img, (854, 480))
            cv2.imshow("Preview (press 'c' to capture)", preview)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            if key == ord('c'):
                z_str = input("현재 로봇 Z축 높이 (mm): ").strip()
                try:
                    z_mm = float(z_str)
                except ValueError:
                    print("숫자를 입력해줘"); continue

                # ── 파이프라인 실행 ──
                bbox, mask, contour = detect_blue_cube(img)
                gripper_state, ratios = detect_gripper_state(img)

                empty_sharpness = {'laplacian_var': 0.0,
                                   'gradient_mean': 0.0, 'edge_mask_px': 0}
                empty_geo       = {'estimated_size_mm': 0.0,
                                   'estimated_z_mm': 0.0, 'pixel_ratio': 0.0}

                if bbox is not None:
                    sharpness = measure_edge_sharpness(img, bbox, contour)
                    geo       = compute_geometry(bbox[2], FRAME_W, z_mm)
                else:
                    sharpness, geo = empty_sharpness, empty_geo
                    print("  ⚠ 파란 큐브 미검출")

                # ── 출력 ──
                print(f"\n  Z = {z_mm} mm")
                print(f"  Gripper : {gripper_state}  "
                      f"(L={ratios[0]:.3f} R={ratios[1]:.3f})")
                if bbox:
                    x, y, w, h = bbox
                    print(f"  Cube    : bbox=({x},{y},{w},{h})  "
                          f"center=({x+w//2},{y+h//2})")
                    print(f"  Sharp   : Laplacian={sharpness['laplacian_var']:.1f}  "
                          f"Gradient={sharpness['gradient_mean']:.1f}")
                    print(f"  Geo     : ratio={geo['pixel_ratio']:.4f}  "
                          f"est_size={geo['estimated_size_mm']:.1f}mm  "
                          f"est_Z={geo['estimated_z_mm']:.1f}mm")

                # ── 저장 ──
                vis = draw_results(img, bbox, contour,
                                   gripper_state, sharpness, geo, z_mm)
                ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
                tag = f"Z{int(z_mm)}_{gripper_state}"
                cv2.imwrite(f"{OUTPUT_DIR}/{ts}_{tag}_raw.png", img)
                cv2.imwrite(f"{OUTPUT_DIR}/{ts}_{tag}_vis.png", vis)
                cv2.imwrite(f"{OUTPUT_DIR}/{ts}_{tag}_mask.png", mask)
                print(f"  저장 → {OUTPUT_DIR}/{ts}_{tag}_*.png\n")

                cv2.imshow("Analysis Result", cv2.resize(vis, (854, 480)))

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
