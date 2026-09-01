import os
import math
import rclpy
from rclpy.node import Node
import serial
import threading
import time
import struct

from rcl_interfaces.msg import SetParametersResult
from platoon_interfaces.msg import VehicleCmd, Telemetry

# 로그 출력시 시간 안나옴
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

# ---- 프로토콜 상수 (STM32 usart.c와 일치하도록 수정) ----
RX_HEADER = 0xAA    # Jetson이 보낼 때의 헤더 (STM32 입장에선 RX)
RX_FOOTER = 0x55    # Jetson이 보낼 때의 푸터
TX_HEADER = 0xBB    # STM32가 보낼 때의 헤더 (STM32 입장에선 TX)
TX_FOOTER = 0x55    # STM32가 보낼 때의 푸터
CMD_FRAME_LEN = 4   # Jetson -> STM32 제어 명령 프레임 길이 (4바이트)
TELE_FRAME_LEN = 9  # STM32 -> Jetson 텔레메트리 프레임 길이 (기존 10에서 9로 수정)

class ControlNode(Node):
    """
    ROS 2 제어 노드 클래스 (PID 속도 제어 적용)
    - ROS 2 토픽(/vehicle_cmd)을 수신하여 PID 연산 후 STM32로 UART 제어 명령을 전송합니다.
    - 백그라운드 스레드에서 STM32의 센서/상태 데이터를 지속적으로 수신하여 /telemetry 토픽으로 발행합니다.
    """
    def __init__(self):
        super().__init__('control')

        # 1. 내부 고정변수
        self.baud = 115200
        self.steer_offset = 70
        self.stop_duty = 0

        # 1-1. 파라미터 선언
        # STM32 시리얼 포트 — /dev/ttyACM0 대신 by-id 고정 경로를 기본값으로 쓴다.
        # ttyACM 번호는 재부팅 때마다 USB 열거 순서에 따라 ESP32/STM32끼리
        # 뒤바뀔 수 있다(실제로 겪은 문제). 차량마다 꽂힌 보드가 다르므로
        # 실행할 때 --ros-args -p stm32_port:=/dev/serial/by-id/... 로 넣어줄 것.
        self.declare_parameter(
            'stm32_port',
            '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF485775495067181954-if02',
        )
        self.port = self.get_parameter('stm32_port').get_parameter_value().string_value
        self.declare_parameter('target_slow_speed', 15.0)
        self.declare_parameter('target_cruise_speed', 40.0) 
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        # V2X로 다른 차량에 실제 속도(m/s)를 알려주는 데만 쓴다 (PID 입력엔 안 씀).
        # TODO: 실측 필요
        self.declare_parameter('enc_cpr', 1560.0)       # 바퀴 1회전당 엔코더 펄스 수
        self.declare_parameter('wheel_dia_mm', 65.0)    # 바퀴 지름(mm)

        # 1-2. 파라미터 값 가져오기
        self.target_slow_speed = self.get_parameter('target_slow_speed').value
        self.target_cruise_speed = self.get_parameter('target_cruise_speed').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.enc_cpr = self.get_parameter('enc_cpr').value
        self.wheel_circ_m = (math.pi * self.get_parameter('wheel_dia_mm').value) / 1000.0

        # --- 파라미터 로드 확인용 로그 출력 ---
        self.get_logger().info('============= Control Node Parameters =============')
        self.get_logger().info(f' PID : Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')
        self.get_logger().info('===================================================')
        # -------------------------------------------------------------------

        # 2. PID 상태 저장용 변수 초기화
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = self.get_clock().now()
        self.current_speed = 0.0  # 텔레메트리에서 지속적으로 갱신됨 (PID 입력용, 엔코더 펄스 그대로)
        self.last_tele_time = time.time()  # speed_mps(m/s) 계산용 dt 기준

        # 3. 시리얼 스레드 락 및 통신 포트 초기화
        self.serial_lock = threading.Lock()
        self.ser = None
        self.open_serial()

        # 4. ROS 2 토픽 구독(Subscription) 및 발행(Publisher) 생성
        self.sub = self.create_subscription(
            VehicleCmd, '/vehicle_cmd', self.on_cmd, 10)
        self.tele_pub = self.create_publisher(Telemetry, '/telemetry', 10)

        # 5. 백그라운드 수신 스레드 생성 및 시작
        self.rx_running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # 6. 파라미터 실시간 변경을 감지하는 콜백
        self.add_on_set_parameters_callback(self.parameter_callback)

    # 터미널에서 값이 변경되면 자동으로 호출되는 함수
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
                self.get_logger().info(f'실시간 적용 완료: Kp = {self.kp}')
            elif param.name == 'ki':
                self.ki = param.value
                self.get_logger().info(f'실시간 적용 완료: Ki = {self.ki}')
            elif param.name == 'kd':
                self.kd = param.value
                self.get_logger().info(f'실시간 적용 완료: Kd = {self.kd}')
            elif param.name == 'target_cruise_speed':
                self.target_cruise_speed = param.value
                self.get_logger().info(f'실시간 적용 완료: target_cruise_speed = {self.target_cruise_speed}')
                
        return SetParametersResult(successful=True)
    
    def open_serial(self):
        """시리얼 포트 연결 시도 함수 (Thread-safe)"""
        with self.serial_lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info(f'시리얼 연결 완료: {self.port} @ {self.baud}')
                return True
            except serial.SerialException as e:
                self.get_logger().warn(f'시리얼 포트 연결 실패 ({self.port}): {e}')
                self.ser = None
                return False

    def calculate_pid(self, target_speed, current_speed):
        """목표 속도와 현재 속도를 바탕으로 모터 Duty 제어값을 계산합니다."""
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        if dt <= 0.0:
            dt = 0.01 

        error = target_speed - current_speed
        p_term = self.kp * error

        # 적분 누적 및 안티 와인드업(Anti-Windup) 처리 (과도한 누적 방지)
        self.integral_error += error * dt
        self.integral_error = max(min(self.integral_error, 100.0), -100.0) 
        i_term = self.ki * self.integral_error

        d_term = self.kd * (error - self.prev_error) / dt

        control_output = p_term + i_term + d_term

        self.prev_error = error
        self.prev_time = current_time

        # 최종 듀티값을 0~255 사이로 클램핑 (후진이 없다면 0이 최소)
        return max(0, min(255, int(control_output)))

    # ---- 명령 수신 콜백 → STM32로 UART 송신 ----
    def on_cmd(self, msg: VehicleCmd):
        """/vehicle_cmd 수신 시 PID를 계산하여 송신 패킷 전송"""
        
        # 1. speed_mode에 따른 타겟 속도 설정 및 PID 계산
        if msg.speed_mode == 0:
            duty_val = self.stop_duty
            # 정지 상태일 때는 적분 오차를 초기화하여 갑자기 튀어나가는 것을 방지합니다.
            self.integral_error = 0.0
            self.prev_error = 0.0
        elif msg.speed_mode == 1:
            duty_val = self.calculate_pid(self.target_slow_speed, self.current_speed)
        elif msg.speed_mode >= 2:
            duty_val = self.calculate_pid(self.target_cruise_speed, self.current_speed)
        else:
            duty_val = self.stop_duty

        duty = int(duty_val) & 0xFF
        
        # 2. 조향각 변환 (+70 오프셋 적용 및 0~140 범위 클램핑)
        steer_byte = int(msg.steering_deg) + self.steer_offset
        steer_byte = max(0, min(140, steer_byte)) & 0xFF

        # 3. 4바이트 패킷 생성
        frame = bytes([RX_HEADER, duty, steer_byte, RX_FOOTER])

        # 4. 시리얼 포트를 통해 STM32로 데이터 전송
        with self.serial_lock:
            if self.ser is None or not self.ser.is_open:
                return
            try:
                self.ser.write(frame)
            except serial.SerialException as e:
                self.get_logger().error(f'시리얼 쓰기 실패: {e}')
                if self.ser:
                    self.ser.close()
                self.ser = None

    # ---- STM32 텔레메트리 수신 루프 (별도 스레드에서 실행) ----
    def rx_loop(self):
        buf = bytearray()
        while self.rx_running:
            if self.ser is None or not self.ser.is_open:
                time.sleep(1.0)
                self.open_serial()
                continue

            try:
                with self.serial_lock:
                    if self.ser and self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting)
                    else:
                        data = None

                if data:
                    buf.extend(data)
                else:
                    time.sleep(0.005)
                    continue
            except serial.SerialException as e:
                self.get_logger().error(f'시리얼 읽기 오류: {e}')
                with self.serial_lock:
                    if self.ser:
                        self.ser.close()
                    self.ser = None
                continue

            while len(buf) >= TELE_FRAME_LEN:
                if buf[0] != TX_HEADER:
                    buf.pop(0)
                    continue
                if buf[TELE_FRAME_LEN - 1] != TX_FOOTER:
                    buf.pop(0)
                    continue

                frame = buf[:TELE_FRAME_LEN]
                self.publish_telemetry(frame)
                del buf[:TELE_FRAME_LEN]
                
    def publish_telemetry(self, frame: bytearray):
        """수신 패킷 파싱 및 현재 속도 업데이트"""
        msg = Telemetry()
        msg.steer_deg = frame[1] - self.steer_offset
        msg.left_delta = struct.unpack('>h', frame[2:4])[0]
        msg.right_delta = struct.unpack('>h', frame[4:6])[0]
        msg.dist_cm = struct.unpack('>H', frame[6:8])[0]

        # [핵심] 좌우 엔코더 변화량의 평균을 '현재 속도'로 저장하여 PID 계산에 활용
        self.current_speed = (msg.left_delta + msg.right_delta) / 2.0

        # speed_mps(실제 m/s)는 PID엔 안 쓰지만, V2X로 다른 차량에 내 속도를
        # 알려줄 때(self_status) 필요해서 별도로 환산해 Telemetry에 실어 보낸다.
        now = time.time()
        dt = now - self.last_tele_time
        self.last_tele_time = now
        if dt <= 0.0 or dt > 0.5:  # 패킷 누락/최초 수신 시 비정상 dt 방지
            dt = 0.02
        avg_delta = (msg.left_delta + msg.right_delta) / 2.0
        msg.speed_mps = (avg_delta / self.enc_cpr) * self.wheel_circ_m / dt

        self.tele_pub.publish(msg)

    def destroy_node(self):
        self.rx_running = False
        if hasattr(self, 'rx_thread') and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)

        with self.serial_lock:
            if self.ser is not None and self.ser.is_open:
                try:
                    stop_duty = int(self.stop_duty) & 0xFF
                    steer_offset = int(self.steer_offset) & 0xFF
                    self.ser.write(bytes([RX_HEADER, stop_duty, steer_offset, RX_FOOTER]))
                    self.ser.close()
                except Exception:
                    pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()