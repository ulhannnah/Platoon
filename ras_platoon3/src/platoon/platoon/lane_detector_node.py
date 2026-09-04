"""
비전 노드 (Lane Detector Node)
- Picamera2 영상 캡처 및 전처리 (ROI Remap, Canny)
- 슬라이딩 윈도우 및 다항식 피팅을 통한 차선 곡선 추출 (예측 알고리즘 적용)
- '동적 가상 오프셋' 로직을 적용하여 실시간 차선 폭 학습 및 /lane_info 발행
- 웹 모니터링을 위한 Flask 서버 구동
"""

import warnings
import os
import sys
import threading
import time
import numpy as np
import cv2

# 영상 토픽 발행용
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# numpy.polyfit 계산 시 발생하는 RankWarning 경고 메시지 출력 안 함
warnings.simplefilter('ignore', np.RankWarning)

# 로그 출력시 시간 안나옴
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

# libcamera 로그 차단
os.environ["LIBCAMERA_LOG_LEVELS"] = "3"

import rclpy
from rclpy.node import Node
from picamera2 import Picamera2
from flask import Flask, Response

# 커스텀 메세지 
from platoon_interfaces.msg import LaneInfo

# Flask 앱 생성 및 로그 최소화
app = Flask(__name__)
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')

        # 프로그램 실행 상태 플래그 (스레드 종료용)
        self.is_running = True

        # -------------------------------------------------------------
        # ROS 2 퍼블리셔 설정
        # -------------------------------------------------------------
        self.lane_info_pub = self.create_publisher(LaneInfo, 'lane_info', 10)
        self.image_pub = self.create_publisher(Image, 'processed_image', 10)
        self.bridge = CvBridge()

        # 웹 스트리밍용 인코딩 데이터 공유 변수 및 스레드 락
        self.latest_jpg_bytes = None
        self.frame_lock = threading.Lock()

        # -------------------------------------------------------------
        # ROS 2 파라미터 선언 (기본값 설정)
        # -------------------------------------------------------------
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hist_threshold', 100)
        self.declare_parameter('window_margin', 30)
        self.declare_parameter('window_minpix', 20)

        # 동적 오프셋을 위한 초기값 (학습 전 임시 사용)
        self.dynamic_virtual_offset = 160

        # -------------------------------------------------------------
        # 0. 캘리브레이션 데이터 로드 및 고속 remap 사전 연산
        # -------------------------------------------------------------
        PARAM_FILE = "camera_params.npz"
        if not os.path.exists(PARAM_FILE):
            self.get_logger().error(f"'{PARAM_FILE}' 파일을 찾을 수 없습니다.")
            sys.exit(1)
        
        calib_data = np.load(PARAM_FILE)
        mtx, dist = calib_data["mtx"], calib_data["dist"]
        full_map_x, full_map_y = cv2.initUndistortRectifyMap(mtx, dist, None, mtx, (640, 480), cv2.CV_16SC2)
        self.map_x_roi = full_map_x[240:, :]
        self.map_y_roi = full_map_y[240:, :]
        # 💡 [추가] 스트리밍용 전체 프레임(640x480) 리맵 맵 보존
        self.map_x_full = full_map_x
        self.map_y_full = full_map_y

        # -------------------------------------------------------------
        # 1. Picamera2 설정
        # -------------------------------------------------------------
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            sensor={"output_size": (1640, 1232)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)
        self.picam2.set_controls({"AwbMode": 0})

        # -------------------------------------------------------------
        # 2. Flask 웹 서버 데몬 스레드 구동
        # -------------------------------------------------------------
        self.setup_web_routes()
        self.web_thread = threading.Thread(target=self.run_web_server, daemon=True)
        self.web_thread.start()

        self.prev_left_fit = None
        self.prev_right_fit = None
        self.prev_leftx_base = None
        self.prev_rightx_base = None

        # 💡 [추가] 차선 연속 유실 프레임 카운터
        self.left_lost_count = 0
        self.right_lost_count = 0
        
        # ROS 2 타이머 (30FPS 타겟)
        self.timer = self.create_timer(0.033, self.timer_callback)

    # -------------------------------------------------------------
    # 웹 서버 관련 메서드
    # -------------------------------------------------------------
    def setup_web_routes(self):
        @app.route('/')
        def index():
            return '<!DOCTYPE html><html lang="ko"><head><meta charset="UTF-8"><style>body{margin:0;background:#222;display:flex;justify-content:center;align-items:center;height:100vh;}img{width:640px;height:480px;object-fit:contain;border-radius:8px;}</style></head><body><img src="/video_feed"></body></html>'
        
        @app.route('/video_feed')
        def video_feed():
            return Response(self.generate_web_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def generate_web_frames(self):
        while self.is_running:
            with self.frame_lock:
                jpg_bytes = self.latest_jpg_bytes
            if jpg_bytes is None:
                time.sleep(0.01)
                continue
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
            time.sleep(0.03)

    def run_web_server(self):
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)

    # -------------------------------------------------------------
    # 차선 인식 영상처리
    # -------------------------------------------------------------
    def process_frame(self, roi_img):
        p_canny_low = self.get_parameter('canny_low').value
        p_canny_high = self.get_parameter('canny_high').value
        p_hist_thresh = self.get_parameter('hist_threshold').value
        p_margin = self.get_parameter('window_margin').value
        p_minpix = self.get_parameter('window_minpix').value

        H, W = roi_img.shape[:2]

        src_pts = np.float32([
            [W * 0.35, H * 0.1],
            [W * 0.65, H * 0.1],
            [W * 0.10, H * 1.0],
            [W * 0.90, H * 1.0]
        ])

        dst_pts = np.float32([
            [W * 0.3, 0],
            [W * 0.7, 0],
            [W * 0.2, H],
            [W * 0.8, H]
        ])

        # 투시 변환 행렬 계산
        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

        # 💡 [추가] 역투시 변환 행렬 계산 (왼쪽 화면 합성을 위해 사용)
        Minv = cv2.getPerspectiveTransform(dst_pts, src_pts)

        # 💡 [수정됨] 전처리 및 Canny 엣지 검출을 '변환 전'에 먼저 수행합니다.
        gray_roi = cv2.cvtColor(roi_img, cv2.COLOR_RGB2GRAY)
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        edges_img = cv2.Canny(blurred_roi, p_canny_low, p_canny_high)

        # 💡 [수정됨] 엣지가 추출된 이미지를 버드아이뷰로 변환합니다. (가장자리 노이즈 해결)
        edges_all = cv2.warpPerspective(edges_img, matrix, (W, H), flags=cv2.INTER_LINEAR)
        display_img = cv2.cvtColor(edges_all, cv2.COLOR_GRAY2BGR)
        roi_H, roi_W = edges_all.shape

        midpoint = roi_W // 2
        nwindows = 8
        window_height = int(roi_H / nwindows)
        
        nonzero = edges_all.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])

        left_lane_inds, right_lane_inds = [], []
        left_valid_cnt, right_valid_cnt = 0, 0
        valid_left_centers, valid_left_y = [], []
        valid_right_centers, valid_right_y = [], []
        left_found, right_found = False, False
        
        # 하단 픽셀 탐색으로 시작점 찾기
        hist_bottom = np.sum(edges_all[roi_H - window_height:roi_H, :], axis=0)
        leftx_base = np.argmax(hist_bottom[:midpoint]) if np.max(hist_bottom[:midpoint]) > p_hist_thresh else roi_W // 4
        rightx_base = np.argmax(hist_bottom[midpoint:]) + midpoint if np.max(hist_bottom[midpoint:]) > p_hist_thresh else (roi_W // 4) * 3
        
        leftx_current, rightx_current = leftx_base, rightx_base

        # 슬라이딩 윈도우 진행
        for window in range(nwindows):
            win_y_low = roi_H - (window + 1) * window_height
            win_y_high = roi_H - window * window_height
            win_y_mid = (win_y_low + win_y_high) // 2
            next_win_y_mid = win_y_mid - window_height

            if not left_found:
                hist_left = np.sum(edges_all[win_y_low:win_y_high, :midpoint], axis=0)
                if np.max(hist_left) > p_hist_thresh:
                    left_candidates = np.where(hist_left > p_hist_thresh)[0]
                    leftx_current = left_candidates[np.argmax(left_candidates)]
                    leftx_base = leftx_current
                    self.prev_leftx_base = leftx_base
                    left_found = True
                elif getattr(self, 'prev_leftx_base', None) is not None:
                    leftx_current = self.prev_leftx_base
                    leftx_base = leftx_current
                    left_found = True

            if not right_found:
                hist_right = np.sum(edges_all[win_y_low:win_y_high, midpoint:], axis=0)
                if np.max(hist_right) > p_hist_thresh:
                    right_candidates = np.where(hist_right > p_hist_thresh)[0] + midpoint
                    rightx_current = right_candidates[np.argmin(abs(right_candidates - midpoint))]
                    rightx_base = rightx_current 
                    self.prev_rightx_base = rightx_base
                    right_found = True
                elif getattr(self, 'prev_rightx_base', None) is not None:
                    rightx_current = self.prev_rightx_base
                    rightx_base = rightx_current
                    right_found = True

            if left_found:
                leftx_current = max(0, min(roi_W - 1, leftx_current))
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                  (nonzerox >= leftx_current - p_margin) & (nonzerox < leftx_current + p_margin)).nonzero()[0]
                left_lane_inds.append(good_left_inds)
                leftx_draw = leftx_current
                if len(good_left_inds) > p_minpix:
                    leftx_draw = int(np.mean(nonzerox[good_left_inds]))
                    left_valid_cnt += 1
                    valid_left_centers.append(leftx_draw)
                    valid_left_y.append(win_y_mid)
                    leftx_current = leftx_draw
                    if left_valid_cnt >= 3:
                        left_fit_temp = np.polyfit(valid_left_y, valid_left_centers, 2)
                        leftx_current = int(left_fit_temp[0]*next_win_y_mid**2 + left_fit_temp[1]*next_win_y_mid + left_fit_temp[2])
                else:
                    if self.prev_left_fit is not None:
                        leftx_current = int(self.prev_left_fit[0]*next_win_y_mid**2 + self.prev_left_fit[1]*next_win_y_mid + self.prev_left_fit[2])
                cv2.rectangle(display_img, (leftx_draw - p_margin, win_y_low), (leftx_draw + p_margin, win_y_high), (0, 255, 0), 2)

            if right_found:
                rightx_current = max(0, min(roi_W - 1, rightx_current))
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                   (nonzerox >= rightx_current - p_margin) & (nonzerox < rightx_current + p_margin)).nonzero()[0]
                right_lane_inds.append(good_right_inds)
                rightx_draw = rightx_current
                if len(good_right_inds) > p_minpix:
                    rightx_draw = int(np.mean(nonzerox[good_right_inds]))
                    right_valid_cnt += 1
                    valid_right_centers.append(rightx_draw)
                    valid_right_y.append(win_y_mid)
                    rightx_current = rightx_draw
                    if right_valid_cnt >= 3:
                        right_fit_temp = np.polyfit(valid_right_y, valid_right_centers, 2)
                        rightx_current = int(right_fit_temp[0]*next_win_y_mid**2 + right_fit_temp[1]*next_win_y_mid + right_fit_temp[2])
                else:
                    if self.prev_right_fit is not None:
                        rightx_current = int(self.prev_right_fit[0]*next_win_y_mid**2 + self.prev_right_fit[1]*next_win_y_mid + self.prev_right_fit[2])
                cv2.rectangle(display_img, (rightx_draw - p_margin, win_y_low), (rightx_draw + p_margin, win_y_high), (0, 255, 0), 2)

        if len(left_lane_inds) > 0: left_lane_inds = np.concatenate(left_lane_inds)
        if len(right_lane_inds) > 0: right_lane_inds = np.concatenate(right_lane_inds)

        leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]
        rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]
        left_slope, right_slope = 0.0, 0.0
        
        left_style, right_style = LaneInfo.UNKNOWN, LaneInfo.UNKNOWN
        ploty = np.linspace(0, roi_H - 1, roi_H)

        left_detected = (left_valid_cnt >= 3)
        right_detected = (right_valid_cnt >= 3)
        lane_detected = left_detected or right_detected

        if left_detected:
            self.left_lost_count = 0
        else:
            self.left_lost_count += 1
            if self.left_lost_count > 5: 
                self.prev_left_fit = None
                self.prev_leftx_base = None

        if right_detected:
            self.right_lost_count = 0
        else:
            self.right_lost_count += 1
            if self.right_lost_count > 5:
                self.prev_right_fit = None
                self.prev_rightx_base = None
        
        left_bottom_x, right_bottom_x = leftx_base, rightx_base

        if left_detected:
            if left_valid_cnt >= 4:
                left_fit = np.polyfit(lefty, leftx, 2)
                self.prev_left_fit = left_fit 
                left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
                
                left_bottom_x = int(left_fitx[-1])
                
                dy = float(lefty[-1] - lefty[0]) if len(lefty) > 0 else 0.0
                dx = float(leftx[-1] - leftx[0]) if len(leftx) > 0 else 0.0
                left_slope = (dx / dy) if dy != 0 else 0.0
                pts_left = np.vstack((left_fitx, ploty)).astype(np.int32).T
                cv2.polylines(display_img, [pts_left], False, (255, 0, 0), 3)
            
            if len(lefty) > 0:
                sorted_y = np.sort(lefty)
                max_gap = np.max(np.diff(sorted_y)) if len(sorted_y) > 1 else 0
                y_span = sorted_y[-1] - sorted_y[0]
                left_style = LaneInfo.DASHED if (max_gap > 30 or y_span < (roi_H * 0.4)) else LaneInfo.SOLID

        if right_detected:
            if right_valid_cnt >= 4:
                right_fit = np.polyfit(righty, rightx, 2)
                self.prev_right_fit = right_fit 
                right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
                
                right_bottom_x = int(right_fitx[-1])
                
                dy = float(righty[-1] - righty[0]) if len(righty) > 0 else 0.0
                dx = float(rightx[-1] - rightx[0]) if len(rightx) > 0 else 0.0
                right_slope = (dx / dy) if dy != 0 else 0.0
                pts_right = np.vstack((right_fitx, ploty)).astype(np.int32).T
                cv2.polylines(display_img, [pts_right], False, (0, 0, 255), 3)

            if len(righty) > 0:
                sorted_y = np.sort(righty)
                max_gap = np.max(np.diff(sorted_y)) if len(sorted_y) > 1 else 0
                y_span = sorted_y[-1] - sorted_y[0]
                right_style = LaneInfo.DASHED if (max_gap > 30 or y_span < (roi_H * 0.4)) else LaneInfo.SOLID

        if left_detected and right_detected:
            lane_center = int((left_bottom_x + right_bottom_x) / 2)
            self.dynamic_virtual_offset = int((right_bottom_x - left_bottom_x) / 2)
        elif left_detected:
            lane_center = left_bottom_x + self.dynamic_virtual_offset
            
        elif right_detected:
            lane_center = right_bottom_x - self.dynamic_virtual_offset
            
        else:
            lane_center = roi_W // 2


        # 인식된 차선이 중앙너머로 가버릴 경우
        roi_center = roi_W // 2
        
        if right_detected and right_bottom_x < roi_center:
            # 우측 차선이 중앙을 넘어 왼쪽 영역으로 침범한 경우
            self.prev_right_fit = None
            self.prev_rightx_base = None
            self.right_lost_count = 999  # 강제로 유실 상태로 만들어 새 엣지를 탐색하게 유도

        if left_detected and left_bottom_x > roi_center:
            # 좌측 차선이 중앙을 넘어 오른쪽 영역으로 침범한 경우
            self.prev_left_fit = None
            self.prev_leftx_base = None
            self.left_lost_count = 999

        offset = (roi_W // 2) - lane_center
        cv2.line(display_img, (roi_W // 2, 0), (roi_W // 2, 20), (0, 0, 255), 3)
        cv2.line(display_img, (lane_center, 0), (lane_center, 20), (255, 0, 0), 3)

        msg = LaneInfo()
        msg.offset = int(offset)
        msg.lane_detected = bool(lane_detected)
        msg.left_style = int(left_style)
        msg.right_style = int(right_style)
        msg.left_slope = round(float(left_slope), 3)
        msg.right_slope = round(float(right_slope), 3)
        msg.dynamic_virtual_offset = float(self.dynamic_virtual_offset)
        

        # 색상 반전 방지를 위해 원본을 그대로 복사하여 사용합니다.
        roi_bgr = roi_img.copy()

        # -------------------------------------------------------------
        # 화면에 띄울 텍스트 설정 및 합성 (상수 직접 사용)
        # -------------------------------------------------------------
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (255, 255, 255)  # 흰색 텍스트
        thickness = 2

        # 💡 [수정됨] 숫자가 아닌 LaneInfo 상수를 직접 비교하도록 변경
        if msg.left_style == LaneInfo.DASHED:
            left_style_str = "Dashed"
        elif msg.left_style == LaneInfo.SOLID:
            left_style_str = "Solid"
        else:
            left_style_str = "None"

        if msg.right_style == LaneInfo.DASHED:
            right_style_str = "Dashed"
        elif msg.right_style == LaneInfo.SOLID:
            right_style_str = "Solid"
        else:
            right_style_str = "None"

        # display_img 좌측 상단에 텍스트 합성
        cv2.putText(display_img, f"Offset: {msg.offset:.1f}", (10, 30), font, font_scale, color, thickness)
        cv2.putText(display_img, f"L_Slope: {msg.left_slope:.2f} | R_Slope: {msg.right_slope:.2f}", (10, 60), font, font_scale, color, thickness)
        cv2.putText(display_img, f"L_Style: {left_style_str} | R_Style: {right_style_str}", (10, 90), font, font_scale, (0, 255, 255), thickness)

        return display_img, msg

    # -------------------------------------------------------------
    # ROS 2 타이머 콜백
    # -------------------------------------------------------------
    def timer_callback(self):
        if not self.is_running: return
        raw_frame = self.picam2.capture_array("main")
        undistorted_roi = cv2.remap(raw_frame, self.map_x_roi, self.map_y_roi, interpolation=cv2.INTER_LINEAR)
        
        processed_img, lane_info = self.process_frame(undistorted_roi)
        self.lane_info_pub.publish(lane_info)
        
        # 💡 [수정] 검출은 하단 240 ROI에서만 수행하되, 스트림은 전체 640x480 프레임으로 제공합니다.
        # 상단 240px = 원본(보정된) 영상, 하단 240px = 처리 결과(오버레이 포함)
        undistorted_full = cv2.remap(raw_frame, self.map_x_full, self.map_y_full, interpolation=cv2.INTER_LINEAR)
        display_full = undistorted_full.copy()
        display_full[240:480, :] = processed_img
        
        ret, buffer = cv2.imencode('.jpg', display_full, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ret:
            with self.frame_lock:
                self.latest_jpg_bytes = buffer.tobytes()

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'timer'): self.timer.destroy()
        if hasattr(self, 'picam2'):
            self.picam2.stop()
            self.picam2.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()