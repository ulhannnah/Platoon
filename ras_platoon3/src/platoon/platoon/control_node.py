"""
제어 노드 (Control Node)
- /vehicle_cmd 토픽을 수신하여 STM32로 전송할 4바이트 UART 패킷을 생성합니다.
- 백그라운드 스레드에서 STM32의 텔레메트리 데이터를 수신하여 /telemetry 토픽으로 발행합니다.
- 속도 PID 연산을 제외하고 명령 전달 및 통신에만 집중하도록 경량화되었습니다.
"""
import glob
import os
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

# ---- 프로토콜 상수 (STM32 usart.c와 일치) ----
RX_HEADER = 0xAA    # Jetson -> STM32 헤더
RX_FOOTER = 0x55    # Jetson -> STM32 푸터
TX_HEADER = 0xBB    # STM32 -> Jetson 헤더
TX_FOOTER = 0x55    # STM32 -> Jetson 푸터
CMD_FRAME_LEN = 4   # 제어 명령 프레임 길이 (4바이트)
TELE_FRAME_LEN = 9  # 텔레메트리 프레임 길이 (9바이트)

# 차량마다 STM32 보드의 정확한 시리얼번호(by-id)는 다르지만, 제조사 접두어는
# 항상 같다. 한 차량엔 STM32가 1개만 꽂혀있으므로 이 패턴으로 자동 탐색하면
# 차량마다 값을 안 바꿔도 된다 (여러 개 꽂혀있으면 첫 번째 것을 씀 — 그럴 땐
# stm32_port 파라미터로 직접 지정할 것).
STM32_BY_ID_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_*-if02"


def _autodetect_port(pattern: str, fallback: str) -> str:
    matches = glob.glob(pattern)
    return matches[0] if matches else fallback


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')

        # 1. 내부 고정 변수
        self.baud = 115200
        self.steer_offset = 70
        self.stop_duty = 0

        # 1-1. 파라미터 선언
        self.declare_parameter(
            'stm32_port',
            _autodetect_port(
                STM32_BY_ID_GLOB,
                '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF485775495067181954-if02',
            ),
        )
        self.port = self.get_parameter('stm32_port').get_parameter_value().string_value
        
        # 속도 모드별 고정 듀티값 설정 파라미터 (PID 대신 사용)
        self.declare_parameter('slow_duty', 30)
        self.declare_parameter('cruise_duty', 60)

        self.slow_duty = int(self.get_parameter('slow_duty').value)
        self.cruise_duty = int(self.get_parameter('cruise_duty').value)

        # --- 파라미터 로드 확인용 로그 출력 ---
        self.get_logger().info('============= Control Node Parameters =============')
        self.get_logger().info(f' Port: {self.port} @ {self.baud}')
        self.get_logger().info(f' Slow(모드1): {self.slow_duty}, Cruise(모드2+): {self.cruise_duty}')
        self.get_logger().info('===================================================')

        # 2. 시리얼 스레드 락 및 통신 포트 초기화
        self.serial_lock = threading.Lock()
        self.ser = None
        self.open_serial()

        # 3. ROS 2 토픽 구독 및 발행 설정
        self.sub = self.create_subscription(
            VehicleCmd, 'vehicle_cmd', self.on_cmd, 10)
        self.tele_pub = self.create_publisher(Telemetry, 'telemetry', 10)

        # 4. 백그라운드 수신 스레드 생성 및 시작
        self.rx_running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # 5. 파라미터 실시간 변경 콜백
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'slow_duty':
                self.slow_duty = int(param.value)
                self.get_logger().info(f'실시간 적용 완료: slow_duty = {self.slow_duty}')
            elif param.name == 'cruise_duty':
                self.cruise_duty = int(param.value)
                self.get_logger().info(f'실시간 적용 완료: cruise_duty = {self.cruise_duty}')
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

    # ---- 명령 수신 콜백 → STM32로 UART 송신 ----
    def on_cmd(self, msg: VehicleCmd):
        """vehicle_cmd 수신 시 speed_mode에 따른 듀티와 조향각을 패킷으로 묶어 송신"""
        
        # 1. speed_mode에 따른 모터 듀티 결정 (PID 제거 버전)
        if msg.speed_mode == 0:
            duty_val = self.stop_duty
        elif msg.speed_mode == 1:
            duty_val = self.slow_duty
        elif msg.speed_mode >= 2:
            duty_val = self.cruise_duty
        else:
            duty_val = self.stop_duty

        duty = int(duty_val) & 0xFF
        
        # 2. 조향각 변환 (+70 오프셋 적용 및 0~140 범위 클램핑)
        steer_byte = int(msg.steering_deg) + self.steer_offset
        steer_byte = max(0, min(140, steer_byte)) & 0xFF

        # 3. 4바이트 패킷 생성 [헤더, 듀티, 조향, 푸터]
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

    # ---- STM32 텔레메트리 수신 루프 (별도 스레드) ----
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
        """수신 패킷 파싱 후 /telemetry 토픽 발행"""
        msg = Telemetry()
        msg.steer_deg = frame[1] - self.steer_offset
        msg.left_delta = struct.unpack('>h', frame[2:4])[0]
        msg.right_delta = struct.unpack('>h', frame[4:6])[0]
        msg.dist_cm = struct.unpack('>H', frame[6:8])[0]
        msg.speed_mps = (left_delta + right_delta) / 2

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