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
        self.declare_parameter('virtual_offset', 160)
        self.declare_parameter('drift_step',20)
        
        # -------------------------------------------------------------
        # 파라미터 로드 확인용 로그 출력
        # -------------------------------------------------------------
        self.get_logger().info('============= Lane Detector Node Parameters =============')
        self.get_logger().info(f" 1. 영상 처리 : canny_low={self.get_parameter('canny_low').value}, canny_high={self.get_parameter('canny_high').value}, hist_threshold={self.get_parameter('hist_threshold').value}")
        self.get_logger().info(f" 2. 윈도우 탐색 : window_margin={self.get_parameter('window_margin').value}, window_minpix={self.get_parameter('window_minpix').value}")
        self.get_logger().info(f" 3. 가상 오프셋 : virtual_offset={self.get_parameter('virtual_offset').value}")
        self.get_logger().info(f" 4. 슬라이딩 윈도우 : drift_step={self.get_parameter('drift_step').value}")
        self.get_logger().info('=========================================================')

        # -------------------------------------------------------------
        # 0. 캘리브레이션 데이터 로드 및 고속 remap 사전 연산
        # -------------------------------------------------------------
        PARAM_FILE = "camera_params.npz"

        if not os.path.exists(PARAM_FILE):
            self.get_logger().error(f"'{PARAM_FILE}' 파일을 찾을 수 없습니다.")
            sys.exit(1)

        try:
            calib_data = np.load(PARAM_FILE)
            mtx = calib_data["mtx"]
            dist = calib_data["dist"]

            self.map_x, self.map_y = cv2.initUndistortRectifyMap(
                mtx, dist, None, mtx, (640, 480), cv2.CV_16SC2
            )
        except Exception as e:
            self.get_logger().error(f"설정 실패: {e}")
            sys.exit(1)

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
        self.get_logger().info("웹 모니터링 서버가 5000번 포트에서 시작되었습니다.")

        # 타이머 설정 (약 30 FPS 타겟)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("차선 정보 전송 ROS 2 노드가 시작되었습니다.")

    # -------------------------------------------------------------
    # 웹 서버 관련 메서드
    # -------------------------------------------------------------
    def setup_web_routes(self):
        @app.route('/')
        def index():
            return """
            <!DOCTYPE html>
            <html lang="ko">
            <head>
                <meta charset="UTF-8">
                <title>IMX219 Canny Edge Stream</title>
                <style>
                body { margin: 0; background: #222; display: flex; justify-content: center; align-items: center; height: 100vh; }
                img { width: 640px; height: 480px; object-fit: contain; border-radius: 8px; }
                </style>
            </head>
            <body><img src="/video_feed"></body>
            </html>
            """

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

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n')
            
            time.sleep(0.03)

    def run_web_server(self):
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)

    # -------------------------------------------------------------
    # 차선 인식 영상처리
    # -------------------------------------------------------------
    def process_frame(self, img):
        # 파라미터 값 불러오기
        p_canny_low = self.get_parameter('canny_low').value
        p_canny_high = self.get_parameter('canny_high').value
        p_hist_thresh = self.get_parameter('hist_threshold').value
        p_margin = self.get_parameter('window_margin').value
        p_minpix = self.get_parameter('window_minpix').value
        p_virtual_offset = self.get_parameter('virtual_offset').value
        p_drift_step = self.get_parameter('drift_step').value

        H, W, _ = img.shape
        roi_top = H // 2

        # [단계 1] 관심 영역(ROI) 설정 및 Canny 엣지 검출
        roi_img = img[roi_top:, :]
        gray_roi = cv2.cvtColor(roi_img, cv2.COLOR_RGB2GRAY)
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        edges_all = cv2.Canny(blurred_roi, p_canny_low, p_canny_high)

        display_img = cv2.cvtColor(edges_all, cv2.COLOR_GRAY2BGR)
        roi_edges = edges_all
        roi_H = roi_edges.shape[0]

        leftx_base = W // 4
        rightx_base = (W // 4) * 3
        midpoint = W // 2


        # 초기 기본 기준점 (나중에 못 찾을 경우를 대비한 중앙 유지용)
        leftx_base = W // 4
        rightx_base = (W // 4) * 3
        midpoint = W // 2

        # [단계 2 & 3 통합] 층별 시작점 탐색 및 슬라이딩 윈도우 추적
        nwindows = 9
        window_height = int(roi_edges.shape[0] / nwindows)
        nonzero = roi_edges.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_inds, right_lane_inds = [], []
        left_valid_cnt, right_valid_cnt = 0, 0
        valid_left_centers, valid_left_y = [], []
        valid_right_centers, valid_right_y = [], []

        # 각 차선이 화면에서 처음 발견되었는지 여부를 추적하는 상태 변수
        left_found = False
        right_found = False
        
        leftx_current = leftx_base
        rightx_current = rightx_base

        for window in range(nwindows):
            win_y_low = roi_edges.shape[0] - (window + 1) * window_height
            win_y_high = roi_edges.shape[0] - window * window_height
            win_y_mid = (win_y_low + win_y_high) // 2
            next_win_y_mid = win_y_mid - window_height  # 다음 층의 중심 Y 좌표

            # --- 1. 차선 시작점 찾기 (해당 윈도우 층에서 가로 전체 스캔) ---
            if not left_found:
                window_band_left = roi_edges[win_y_low:win_y_high, :midpoint]
                hist_left = np.sum(window_band_left, axis=0)
                if np.max(hist_left) > p_hist_thresh:
                    # 현재 층에서 차선을 발견하면 기준점 갱신 및 탐색 시작
                    left_candidates = np.where(hist_left > p_hist_thresh)[0]
                    leftx_current = left_candidates[np.argmax(left_candidates)]
                    leftx_base = leftx_current  # [단계 6]의 가상 오프셋 계산을 위해 하단 기준점도 갱신
                    left_found = True

            if not right_found:
                window_band_right = roi_edges[win_y_low:win_y_high, midpoint:]
                hist_right = np.sum(window_band_right, axis=0)
                if np.max(hist_right) > p_hist_thresh:
                    # 현재 층에서 차선을 발견하면 기준점 갱신 및 탐색 시작
                    right_candidates = np.where(hist_right > p_hist_thresh)[0]
                    right_candidates += midpoint
                    rightx_current = right_candidates[np.argmin(abs(right_candidates - midpoint))]
                    rightx_base = rightx_current 
                    right_found = True


            # --- 2. 탐색 영역(Margin) 설정 및 픽셀 추출 ---
            # 좌우 p_margin 픽셀만큼
            if left_found:
                leftx_current = max(0, min(W - 1, leftx_current))
                win_xleft_low, win_xleft_high = leftx_current - p_margin, leftx_current + p_margin
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                  (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                left_lane_inds.append(good_left_inds)
                leftx_draw = leftx_current
            else:
                good_left_inds = []
                leftx_draw = W // 4  # 아직 못 찾았을 때 화면 표시 방지용

            if right_found:
                rightx_current = max(0, min(W - 1, rightx_current))
                win_xright_low, win_xright_high = rightx_current - p_margin, rightx_current + p_margin
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                   (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
                right_lane_inds.append(good_right_inds)
                rightx_draw = rightx_current
            else:
                good_right_inds = []
                rightx_draw = (W // 4) * 3

            # --- 3. [좌측 차선] 다음 윈도우 X좌표 갱신 ---
            if left_found:
                if len(good_left_inds) > p_minpix:
                    leftx_draw = int(np.mean(nonzerox[good_left_inds]))
                    left_valid_cnt += 1
                    valid_left_centers.append(leftx_draw)
                    valid_left_y.append(win_y_mid)
                    leftx_current = leftx_draw

                    if left_valid_cnt >= 4:
                        left_fit = np.polyfit(valid_left_y, valid_left_centers, 2)
                        leftx_current = int(left_fit[0] * next_win_y_mid**2 + left_fit[1] * next_win_y_mid + left_fit[2])
                else:
                    if left_valid_cnt >= 4:
                        left_fit = np.polyfit(valid_left_y, valid_left_centers, 2)
                        leftx_current = int(left_fit[0] * next_win_y_mid**2 + left_fit[1] * next_win_y_mid + left_fit[2])
                    else:
                        leftx_current += p_drift_step

            # --- 4. [우측 차선] 다음 윈도우 X좌표 갱신 ---
            if right_found:
                if len(good_right_inds) > p_minpix:
                    rightx_draw = int(np.mean(nonzerox[good_right_inds]))
                    right_valid_cnt += 1
                    valid_right_centers.append(rightx_draw)
                    valid_right_y.append(win_y_mid)
                    rightx_current = rightx_draw

                    if right_valid_cnt >= 4:
                        right_fit = np.polyfit(valid_right_y, valid_right_centers, 2)
                        rightx_current = int(right_fit[0] * next_win_y_mid**2 + right_fit[1] * next_win_y_mid + right_fit[2])
                else:
                    if right_valid_cnt >= 4:
                        right_fit = np.polyfit(valid_right_y, valid_right_centers, 2)
                        rightx_current = int(right_fit[0] * next_win_y_mid**2 + right_fit[1] * next_win_y_mid + right_fit[2])
                    else:
                        rightx_current -= p_drift_step

            # --- 5. 디스플레이용 사각형 그리기 (차선을 찾은 이후에만 그림) ---
            if left_found:
                cv2.rectangle(display_img, (leftx_draw - p_margin, win_y_low), (leftx_draw + p_margin, win_y_high), (0, 255, 0), 2)
            if right_found:
                cv2.rectangle(display_img, (rightx_draw - p_margin, win_y_low), (rightx_draw + p_margin, win_y_high), (0, 255, 0), 2)

        # 왼쪽 차선 병합 (리스트 안에 요소가 있을 때만)
        if len(left_lane_inds) > 0:
            left_lane_inds = np.concatenate(left_lane_inds)
        else:
            left_lane_inds = np.array([], dtype=np.int32)

        # 오른쪽 차선 병합 (리스트 안에 요소가 있을 때만)
        if len(right_lane_inds) > 0:
            right_lane_inds = np.concatenate(right_lane_inds)
        else:
            right_lane_inds = np.array([], dtype=np.int32)

        leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]
        rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]

        left_slope, right_slope = 0.0, 0.0
        left_style, right_style = LaneInfo.UNKNOWN, LaneInfo.UNKNOWN
        ploty = np.linspace(0, roi_edges.shape[0] - 1, roi_edges.shape[0])


        left_detected = (left_valid_cnt >= 4)
        right_detected = (right_valid_cnt >= 4)
        lane_detected = left_detected or right_detected
        
        # [단계 4] 다항식 피팅(Polyfit)을 통한 차선 곡선 추출 및 점선/실선 판별

        if left_detected:
            # 1. 3단계와 기준을 통일: 유효 윈도우가 4개 이상일 때만 2차 다항식 곡선 피팅
            if left_valid_cnt >= 4:
                left_fit = np.polyfit(lefty, leftx, 2)
                left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
            # else:
            #     # 유효 윈도우가 4개 미만일 때는 무리한 곡선 피팅을 포기하고, 
            #     # 안전하게 안쪽(오른쪽)으로 파고드는 1차 직선(기본 추세) 생성
            #     left_fit = [0.0, -p_drift_step / window_height, leftx_base + (roi_H * p_drift_step / window_height)]
            #     left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]

            # 기울기 계산 및 화면 그리기
            dy = float(lefty[-1] - lefty[0]) if len(lefty) > 0 else 0.0
            dx = float(leftx[-1] - leftx[0]) if len(leftx) > 0 else 0.0
            left_slope = (dx / dy) if dy != 0 else 0.0
            pts_left = np.vstack((left_fitx, ploty)).astype(np.int32).T
            cv2.polylines(display_img, [pts_left], False, (255, 0, 0), 3)

            # 2. 직관적인 점선/실선 판단 로직
            if left_valid_cnt >= 8:
                left_style = LaneInfo.SOLID     # 유효 윈도우가 7개 이상이면 실선
            elif left_valid_cnt >= 2:
                left_style = LaneInfo.DASHED    # 유효 윈도우가 2~6개면 점선
            else:
                left_style = LaneInfo.UNKNOWN   # 1개 이하는 판별 불가

        if right_detected:
            # 1. 3단계와 기준을 통일: 유효 윈도우가 4개 이상일 때만 2차 다항식 곡선 피팅
            if right_valid_cnt >= 4:
                right_fit = np.polyfit(righty, rightx, 2)
                right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
            # else:
            #     # 유효 윈도우가 4개 미만일 때는 무리한 곡선 피팅을 포기하고, 
            #     # 안전하게 안쪽(왼쪽)으로 파고드는 1차 직선(기본 추세) 생성
            #     right_fit = [0.0, p_drift_step / window_height, rightx_base - (roi_H * p_drift_step / window_height)]
            #     right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

            # 기울기 계산 및 화면 그리기
            dy = float(righty[-1] - righty[0]) if len(righty) > 0 else 0.0
            dx = float(rightx[-1] - rightx[0]) if len(rightx) > 0 else 0.0
            right_slope = (dx / dy) if dy != 0 else 0.0
            pts_right = np.vstack((right_fitx, ploty)).astype(np.int32).T
            cv2.polylines(display_img, [pts_right], False, (0, 0, 255), 3)

            # 2. 직관적인 점선/실선 판단 로직
            if right_valid_cnt >= 7:
                right_style = LaneInfo.SOLID
            elif right_valid_cnt >= 2:
                right_style = LaneInfo.DASHED
            else:
                right_style = LaneInfo.UNKNOWN

        # -------------------------------------------------------------------
        # [단계 5]  텍스트 출력
        # -------------------------------------------------------------------
        cv2.putText(display_img, f"Left Slope: {left_slope:.2f} ({left_style})", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(display_img, f"Right Slope: {right_slope:.2f} ({right_style})", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # -------------------------------------------------------------------
        # [단계 6] 가상 오프셋을 활용한 목표 중앙점(lane_center) 계산
        # -------------------------------------------------------------------
        if left_detected and right_detected:
            # 양쪽 차선이 모두 보일 때는 두 차선의 한가운데를 목표로 삼음
            lane_center = int((leftx_base + rightx_base) / 2)
        elif left_detected:
            # 왼쪽 차선만 보일 때는 왼쪽 차선 위치에서 '가상 오프셋'만큼 떨어진 곳을 중앙으로 삼음
            lane_center = int(leftx_base + p_virtual_offset)
            cv2.putText(display_img, "Using Virtual Offset (Left only)", (10, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        elif right_detected:
            # 오른쪽 차선만 보일 때는 오른쪽 차선 위치에서 '가상 오프셋'만큼 떨어진 곳을 중앙으로 삼음
            lane_center = int(rightx_base - p_virtual_offset)
            cv2.putText(display_img, "Using Virtual Offset (Right only)", (10, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            # 둘 다 안 보일 때는 차량의 현재 중앙을 유지
            lane_center = W // 2

        # 최종 제어용 오프셋 에러 계산 (카메라 중심점 - 목표 중앙점)
        offset = (W // 2) - lane_center

        cv2.line(display_img, (W // 2, 0), (W // 2, 20), (0, 0, 255), 3)
        cv2.line(display_img, (lane_center, 0), (lane_center, 20), (255, 0, 0), 3)
        cv2.putText(display_img, f"Lane Center: {lane_center}", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(display_img, f"Offset: {offset}", (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # [단계 7] 최종 계산된 정보를 ROS 2 메시지 객체로 묶기
        # 데이터 크기를 과하게 늘리지 않기 위해 슬로프는 3자리까지만 유지
        lane_info = LaneInfo()
        lane_info.offset = int(offset)
        lane_info.lane_detected = bool(lane_detected)
        lane_info.left_style = int(left_style)
        lane_info.right_style = int(right_style)
        lane_info.left_slope = round(float(left_slope), 3)
        lane_info.right_slope = round(float(right_slope), 3)

        return display_img, lane_info

    # -------------------------------------------------------------
    # ROS 2 타이머 콜백
    # -------------------------------------------------------------
    def timer_callback(self):
        if not self.is_running:
            return

        raw_frame = self.picam2.capture_array("main")
        undistorted_frame = cv2.remap(raw_frame, self.map_x, self.map_y, interpolation=cv2.INTER_LINEAR)

        # 1. 영상 처리 및 메시지 생성
        processed_img, lane_info = self.process_frame(undistorted_frame)

        # 2. 웹 디스플레이용 이미지 미리 인코딩 및 메모리 공유
        ret, buffer = cv2.imencode('.jpg', processed_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ret:
            with self.frame_lock:
                self.latest_jpg_bytes = buffer.tobytes()

        # 3. ROS 2 메시지 발행
        self.lane_info_pub.publish(lane_info)

        # 4. 이미지 토픽 발행 로직
        try:
            # processed_img(OpenCV 포맷)를 ROS 2 Image 메시지로 변환
            img_msg = self.bridge.cv2_to_imgmsg(processed_img, encoding="bgr8")
            
            # 토픽 발행
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'이미지 메시지 변환 실패: {e}')

    # -------------------------------------------------------------
    # 노드 종료 시 자원 해제 루틴
    # -------------------------------------------------------------
    def destroy_node(self):
        print("안전하게 종료 절차를 시작합니다...")
        self.is_running = False

        # 타이머 해제
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.destroy()

        # Picamera2 정리
        if hasattr(self, 'picam2') and self.picam2 is not None:
            try:
                self.picam2.stop()
                self.picam2.close()
                print("Picamera2 자원이 정상 해제되었습니다.")
            except Exception as e:
                print(f"Picamera2 해제 중 오류: {e}")

        # OpenCV 창 정리 (필요시)
        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[lane_detector_node]: 사용자에 의해 중단 요청(Ctrl+C)이 들어왔습니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()