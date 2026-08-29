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
    ROS 2 제어 노드 클래스
    - ROS 2 토픽(/vehicle_cmd)을 수신하여 STM32로 UART 제어 명령을 전송합니다.
    - 백그라운드 스레드에서 STM32의 센서/상태 데이터를 지속적으로 수신하여 /telemetry 토픽으로 발행합니다.
    """
    def __init__(self):
        super().__init__('control')

        # 1. 내부 고정변수
        self.port = '/dev/ttyACM0'
        self.baud = 115200
        self.steer_offset = 70
        self.stop_duty = 0

        # 1-1. 파라미터 선언 및 초기값 할당
        self.declare_parameter('slow_duty', 80)
        self.declare_parameter('cruise_duty', 130)
        # PID 제거하면서 같이 없어졌던 값들. speed_mps 계산에만 다시 쓴다
        # (모터 제어엔 안 씀 — duty는 여전히 고정값). TODO: 실측 필요.
        self.declare_parameter('enc_cpr', 1560.0)       # 바퀴 1회전당 엔코더 펄스 수
        self.declare_parameter('wheel_dia_mm', 65.0)    # 바퀴 지름(mm)

        self.slow_duty = self.get_parameter('slow_duty').value
        self.cruise_duty = self.get_parameter('cruise_duty').value
        self.enc_cpr = self.get_parameter('enc_cpr').value
        self.wheel_circ_m = (math.pi * self.get_parameter('wheel_dia_mm').value) / 1000.0

        self.get_logger().info('=============== Decision Node Parameters ===============')
        self.get_logger().info(f'초기 파라미터 로드 - slow_duty: {self.slow_duty}, cruise_duty: {self.cruise_duty}')
        self.get_logger().info('========================================================')
        # 2. 상태 저장용 변수 초기화
        self.current_speed = 0.0  # 텔레메트리에서 지속적으로 갱신됨
        self.last_tele_time = time.time()  # speed_mps 계산용 dt 기준

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
        # PID 파라미터가 제거되어 파라미터 변경 시 특별 처리 없음
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

    # PID 제어 제거: 듀티는 메시지에서 직접 받거나 고정값을 사용합니다.

    # ---- 명령 수신 콜백 → STM32로 UART 송신 ----
    def on_cmd(self, msg: VehicleCmd):
        """/vehicle_cmd 수신 시 듀티/조향을 STM32로 전송 (PID 미사용)"""

        # 우선 메시지에 `duty` 필드가 있으면 그대로 사용 (0~255 클램프)
        if hasattr(msg, 'duty'):
            duty = int(getattr(msg, 'duty')) & 0xFF
        else:
            # 메시지에 듀티가 없을 경우 speed_mode 기준의 고정 듀티 사용
            if msg.speed_mode == 0:
                duty = int(self.stop_duty) & 0xFF
            elif msg.speed_mode == 1:
                duty = 100  # 기본 저속 듀티
            elif msg.speed_mode >= 2:
                duty = 200  # 기본 크루즈 듀티
            else:
                duty = int(self.stop_duty) & 0xFF
        
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

        # 좌우 엔코더 변화량의 평균 펄스 수 (PID 제거 이후 duty 제어엔 안 쓰지만,
        # 다른 노드가 "지금 실제 속도"를 알아야 할 때 필요해서 m/s로 환산해 내보낸다)
        now = time.time()
        dt = now - self.last_tele_time
        self.last_tele_time = now
        if dt <= 0.0 or dt > 0.5:  # 패킷 누락/최초 수신 시 비정상 dt 방지
            dt = 0.02

        avg_delta = (msg.left_delta + msg.right_delta) / 2.0
        self.current_speed = (avg_delta / self.enc_cpr) * self.wheel_circ_m / dt
        msg.speed_mps = self.current_speed

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