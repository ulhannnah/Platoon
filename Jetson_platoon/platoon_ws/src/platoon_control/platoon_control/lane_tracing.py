"""
lane_tracing.py
차선 검출 — 원본(친구가 짠 Flask 데모) 코드에서 알고리즘 부분만 순수 함수로 뽑아왔습니다.

원본 대비 수정한 것 (parameters.md에도 반영):
  1. 오프셋 부호 반전 — driving_control.py의 PID 기준("라인이 오른쪽에 있으면 +")과
     원본의 부호가 반대였습니다. 그대로 쓰면 조향이 반대로 돕니다.
  2. 양쪽 차선을 못 찾았을 때 offset=0(완벽히 중앙)으로 착각하던 문제 수정 —
     이제 detected=False로 명확히 보고합니다.
  3. 정규화 — 픽셀값이 아니라 -1.0~+1.0으로 변환해서 반환합니다 (해상도 바뀌어도
     PID 게인을 다시 안 잡아도 되게).
  4. 분기점 감지 — 원본에 나중에 추가된 기능을 가져왔습니다. 원본은 픽셀 상수
     (NORMAL_LANE_WIDTH=320, 640px 화면 전제)를 썼는데, 여기서는 위 3번과 같은
     이유로 화면 폭 대비 비율로 바꿨습니다.

PerceptionTracker는 카메라를 백그라운드 스레드에서 계속 돌리고, 메인 루프는
get_lane()으로 "가장 최근 결과"만 논블로킹으로 읽습니다. 이렇게 나눈 이유는
이 알고리즘(Canny + 슬라이딩 윈도우 9개 + 허프 변환 2회)이 라즈베리파이5에서
50Hz(20ms)를 못 맞출 가능성이 있어서입니다 — 카메라 처리 속도와 제어 루프
주기를 분리해두면, 영상처리가 느려져도 메인 루프(STM32 송신 등)는 멈추지 않습니다.

표지판 인식을 붙일 때는 sign_detection.py에 detect_signs(frame) 함수만 채우면
PerceptionTracker가 같은 프레임을 그쪽에도 자동으로 넘겨줍니다 (카메라를 새로 열
필요 없음).
"""

import threading
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from .camera import CameraCapture
from .driving_control import LaneTracingResult

try:
    from .sign_detection import detect_signs
except ImportError:
    detect_signs = None  # 표지판 모듈이 아직 없으면 그냥 안 돌림


# ── 튜닝 파라미터 (전부 임시값 — 하드웨어 확정 후 parameters.md 절차대로 재조정) ──
CANNY_LOW, CANNY_HIGH = 50, 150
ROI_TOP_RATIO = 0.5          # 화면 위쪽 몇 %를 마스킹할지
NWINDOWS = 9
WINDOW_MARGIN = 30
MIN_PIX = 20
MIN_FIT_POINTS = 50           # 이 개수 이상 에지 픽셀이 있어야 다항식 피팅 시도
SCAN_HEIGHT_STEP = 30
HISTOGRAM_MIN_PEAK = 100
SOLID_LEN_RATIO = 0.45
DASHED_LEN_RATIO = 0.10

# ── 분기점(갈림길) 감지 ────────────────────────────────────────────
# 판정 근거: 갈림길에 들어서면 (1) 좌우 차선 간격이 벌어지고 (2) 오른쪽 차선이
# 우측으로 급하게 눕는다. 두 조건이 동시에 성립할 때만 분기로 본다.
#
# 원본은 NORMAL_LANE_WIDTH=320(px)을 그대로 썼는데 640px 화면을 전제한 값이라,
# 해상도를 바꾸면 조용히 오작동한다. 화면 폭 대비 비율로 바꿔뒀다.
NORMAL_LANE_WIDTH_RATIO = 0.5   # 직진 시 정상 차선폭 ÷ 화면폭 (640px에서 320px)
JUNCTION_WIDTH_RATIO = 1.30     # 정상폭의 몇 배 이상 벌어지면 "확장"으로 볼지
JUNCTION_RIGHT_LEAN = 0.40      # 오른쪽 차선이 위로 갈수록 우측으로 눕는 정도(dx/dy)

JUNCTION_NONE = "NONE"
JUNCTION_FORK_RIGHT = "FORK_RIGHT"


@dataclass
class LaneDebugInfo:
    """제어에는 안 쓰지만 로깅/디버그에 유용한 부가 정보"""
    left_style: str = "Unknown"
    right_style: str = "Unknown"
    left_slope: float = 0.0     # dy/dx. 원본이 발행하던 값과 같은 정의 (거의 수직이면 큰 값)
    right_slope: float = 0.0
    # dx/dy에 부호를 뒤집은 값 = "위로 갈수록 바깥으로 눕는 정도". 수직이면 0.
    # 분기 판정에 쓰는 건 이쪽이다 — dy/dx는 수직선에서 발산해서 임계값을 못 잡는다.
    left_lean: float = 0.0
    right_lean: float = 0.0
    bases_found: bool = False
    junction: str = JUNCTION_NONE   # NONE / FORK_RIGHT
    lane_width_bottom: float = 0.0  # 하단 차선폭(px) — 임계값 튜닝할 때 보는 값
    lane_width_top: float = 0.0     # 상단 차선폭(px)


def _classify_line_style(edges_roi: np.ndarray, mask: np.ndarray, roi_h: int) -> str:
    masked = cv2.bitwise_and(edges_roi, mask)
    lines = cv2.HoughLinesP(masked, 1, np.pi / 180, threshold=20, minLineLength=10, maxLineGap=15)
    if lines is None:
        return "Unknown"
    max_len = max(
        np.hypot(x2 - x1, y2 - y1) for (x1, y1, x2, y2) in lines[:, 0]
    )
    if max_len > roi_h * SOLID_LEN_RATIO:
        return "Solid"
    if max_len > roi_h * DASHED_LEN_RATIO:
        return "Dashed"
    return "Unknown"


def detect_lane(frame: np.ndarray):
    """
    프레임 1장을 받아 차선 중앙 오프셋을 계산한다.

    반환값: (LaneTracingResult, debug_image, LaneDebugInfo)
    """
    H, W, _ = frame.shape
    roi_top = int(H * ROI_TOP_RATIO)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)
    edges[:roi_top, :] = 0
    debug_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

    roi_edges = edges[roi_top:, :]
    roi_h = roi_edges.shape[0]

    # ── 히스토그램 기반 시작점 검출 ──────────────────────────────
    scan_height = 40
    bases_found = False
    leftx_base, rightx_base = W // 4, (W // 4) * 3

    while scan_height <= roi_h:
        sample_zone = roi_edges[roi_h - scan_height:, :]
        histogram = np.sum(sample_zone, axis=0)
        midpoint = histogram.shape[0] // 2

        if np.max(histogram[:midpoint]) > HISTOGRAM_MIN_PEAK and np.max(histogram[midpoint:]) > HISTOGRAM_MIN_PEAK:
            leftx_base = int(np.argmax(histogram[:midpoint]))
            rightx_base = int(np.argmax(histogram[midpoint:]) + midpoint)
            bases_found = True
            break
        scan_height += SCAN_HEIGHT_STEP

    # ── 슬라이딩 윈도우 ───────────────────────────────────────────
    window_height = roi_h // NWINDOWS
    nonzero = roi_edges.nonzero()
    nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])

    leftx_current, rightx_current = leftx_base, rightx_base
    left_lane_inds, right_lane_inds = [], []
    left_centers, right_centers = [leftx_base], [rightx_base]

    for window in range(NWINDOWS):
        win_y_low = roi_h - (window + 1) * window_height
        win_y_high = roi_h - window * window_height

        if window >= 2:
            leftx_current = int(left_centers[-1] + (left_centers[-1] - left_centers[-2]) * 0.8)
            rightx_current = int(right_centers[-1] + (right_centers[-1] - right_centers[-2]) * 0.8)

        win_xleft_low, win_xleft_high = leftx_current - WINDOW_MARGIN, leftx_current + WINDOW_MARGIN
        win_xright_low, win_xright_high = rightx_current - WINDOW_MARGIN, rightx_current + WINDOW_MARGIN

        cv2.rectangle(debug_img, (win_xleft_low, win_y_low + roi_top), (win_xleft_high, win_y_high + roi_top), (0, 255, 0), 2)
        cv2.rectangle(debug_img, (win_xright_low, win_y_low + roi_top), (win_xright_high, win_y_high + roi_top), (0, 255, 0), 2)

        good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                     (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                      (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left)
        right_lane_inds.append(good_right)

        if len(good_left) > MIN_PIX:
            leftx_current = int(np.mean(nonzerox[good_left]))
        left_centers.append(leftx_current)

        if len(good_right) > MIN_PIX:
            rightx_current = int(np.mean(nonzerox[good_right]))
        right_centers.append(rightx_current)

    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        left_lane_inds = np.array([], dtype=int)
        right_lane_inds = np.array([], dtype=int)

    leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]
    rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]

    info = LaneDebugInfo(bases_found=bases_found)
    ploty = np.linspace(0, roi_h - 1, roi_h)

    if len(leftx) > MIN_FIT_POINTS:
        left_fit = np.polyfit(lefty, leftx, 2)
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        info.left_slope = float(np.polyfit(leftx, lefty, 1)[0])
        info.left_lean = -float(np.polyfit(lefty, leftx, 1)[0])
        pts_left = np.vstack((left_fitx, ploty + roi_top)).astype(np.int32).T
        cv2.polylines(debug_img, [pts_left], False, (255, 0, 0), 3)

        mask = np.zeros_like(roi_edges)
        for x_val, y_val in pts_left:
            y_val -= roi_top
            if 0 <= x_val < W and 0 <= y_val < roi_h:
                cv2.circle(mask, (x_val, y_val), 15, 255, -1)
        info.left_style = _classify_line_style(roi_edges, mask, roi_h)

    if len(rightx) > MIN_FIT_POINTS:
        right_fit = np.polyfit(righty, rightx, 2)
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        info.right_slope = float(np.polyfit(rightx, righty, 1)[0])
        info.right_lean = -float(np.polyfit(righty, rightx, 1)[0])
        pts_right = np.vstack((right_fitx, ploty + roi_top)).astype(np.int32).T
        cv2.polylines(debug_img, [pts_right], False, (0, 0, 255), 3)

        mask = np.zeros_like(roi_edges)
        for x_val, y_val in pts_right:
            y_val -= roi_top
            if 0 <= x_val < W and 0 <= y_val < roi_h:
                cv2.circle(mask, (x_val, y_val), 15, 255, -1)
        info.right_style = _classify_line_style(roi_edges, mask, roi_h)

    # ── 분기점(갈림길) 감지 ──────────────────────────────────────
    # 차선을 제대로 못 잡았을 때(bases_found=False)는 leftx_base/rightx_base가
    # 강제 기본값(W/4, 3W/4)이라 폭이 항상 화면의 절반으로 나온다. 그 값으로
    # 분기 판정을 하면 의미가 없으므로 검출에 성공했을 때만 본다.
    info.lane_width_bottom = float(rightx_base - leftx_base)
    info.lane_width_top = float(right_centers[-1] - left_centers[-1])

    if bases_found:
        width_threshold = W * NORMAL_LANE_WIDTH_RATIO * JUNCTION_WIDTH_RATIO
        width_expanded = (info.lane_width_bottom > width_threshold
                          or info.lane_width_top > width_threshold)
        right_curved = info.right_lean > JUNCTION_RIGHT_LEAN
        if width_expanded and right_curved:
            info.junction = JUNCTION_FORK_RIGHT

    # ── 오프셋 계산 (부호 수정 + 정규화) ─────────────────────────
    # 원본은 img_center - lane_center 였음 (부호 반대) → driving_control.py 기준으로 수정:
    #   라인이 화면 오른쪽에 있으면(lane_center > img_center) 양수가 되어야
    #   LANE_KP * offset 이 "오른쪽으로 조향"이 됨
    lane_center = (leftx_base + rightx_base) / 2
    img_center = W / 2
    offset_norm = (lane_center - img_center) / (W / 2)   # -1.0 ~ +1.0

    # bases_found가 False면 히스토그램이 아예 실패한 것 — 강제 기본값(W//4, 3W//4)으로
    # offset=0(완벽히 중앙)처럼 보고하면 안 되므로 여기서 확실히 detected=False 처리
    detected = bases_found and (len(leftx) > MIN_FIT_POINTS or len(rightx) > MIN_FIT_POINTS)

    cv2.putText(debug_img, f"L:{info.left_style} R:{info.right_style}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(debug_img, f"offset(norm): {offset_norm:+.3f}  detected={detected}", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(debug_img, f"width B/T: {info.lane_width_bottom:.0f}/{info.lane_width_top:.0f}",
                (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    if info.junction != JUNCTION_NONE:
        cv2.putText(debug_img, f"JUNCTION: {info.junction}", (10, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.line(debug_img, (int(img_center), roi_top), (int(img_center), roi_top + 20), (0, 0, 255), 3)
    cv2.line(debug_img, (int(lane_center), roi_top), (int(lane_center), roi_top + 20), (255, 0, 0), 3)

    result = LaneTracingResult(offset=offset_norm, detected=detected)
    return result, debug_img, info


class PerceptionTracker:
    """
    카메라 한 대를 백그라운드 스레드에서 계속 캡처하며, 붙어있는 인식기(들)를
    같은 프레임에 돌린다. 메인 루프는 get_lane()으로 최신 결과만 논블로킹으로 읽는다.

    표지판 인식을 붙일 때: sign_detection.py의 detect_signs(frame)만 구현하면
    이 클래스가 자동으로 같은 프레임을 넘겨준다 — 카메라를 새로 열 필요 없음.
    """

    def __init__(self):
        self._lane_result = LaneTracingResult(offset=0.0, detected=False)
        self._lane_info = LaneDebugInfo()
        self._sign_result = None
        self._debug_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.camera: Optional[CameraCapture] = None

        try:
            self.camera = CameraCapture()
        except Exception as e:
            print(f"[warn] 카메라 초기화 실패, 라인트레이싱 없이 진행: {e}")
            return

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    @property
    def available(self) -> bool:
        return self.camera is not None

    def _loop(self):
        while self._running:
            try:
                frame = self.camera.read()
                lane_result, debug_img, lane_info = detect_lane(frame)
                sign_result = detect_signs(frame) if detect_signs is not None else None
            except Exception as e:
                print(f"[warn] 프레임 처리 중 오류: {e}")
                continue

            with self._lock:
                self._lane_result = lane_result
                self._lane_info = lane_info
                self._sign_result = sign_result
                self._debug_frame = debug_img

    def get_lane(self) -> LaneTracingResult:
        with self._lock:
            return self._lane_result

    def get_lane_info(self) -> LaneDebugInfo:
        """
        차선 형태(실선/점선), 기울기, 분기점 감지 결과 등 부가 정보.
        제어에는 안 쓰이고 로깅·디버그용이다.

        TODO: junction("FORK_RIGHT")을 EgoState.checkpoint 갱신에 쓸지 검토.
              체크포인트 인식 방식(표지판 / 바닥마커 / 주행거리)이 아직 팀
              미정이라 지금은 FSM에 연결하지 않았다 (docs/jetson_rpi_todo.md §1.7).
        """
        with self._lock:
            return self._lane_info

    def get_sign(self):
        with self._lock:
            return self._sign_result

    def get_debug_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._debug_frame

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.camera is not None:
            self.camera.close()
