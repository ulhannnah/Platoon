import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import subprocess
import serial
import threading
import math

class LidarIntegratedNode(Node):
    def __init__(self):
        super().__init__('lidar_integrated_node')
        
        # 1. 라이다 데이터 토픽 발행자 (Publisher)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # 2. 전원 제어 서비스 서버 (Service Server)
        self.power_srv = self.create_service(SetBool, 'control_lidar_power', self.power_callback)
        
        # 전원 상태 플래그
        self.is_powered_on = True 
        
        # 3. 백그라운드 데이터 수신 스레드 실행
        self.serial_thread = threading.Thread(target=self.read_and_publish_lidar, daemon=True)
        self.serial_thread.start()
        
        self.get_logger().info('통합 라이다 제어 노드가 시작되었습니다.')

    def power_callback(self, request, response):
        """외부에서 전원 ON/OFF 요청이 들어오면 uhubctl을 실행합니다."""
        power_state = "1" if request.data else "0"
        try:
            # USB 포트 전원 제어 (포트 번호 '1-1'은 환경에 맞게 수정)
            subprocess.run(["sudo", "uhubctl", "-l", "1-1", "-a", power_state], check=True)
            self.is_powered_on = request.data
            
            response.success = True
            response.message = f"라이다 전원을 {'켰' if request.data else '껐'}습니다."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"전원 제어 실패: {e}"
            self.get_logger().error(response.message)
            
        return response

    def read_and_publish_lidar(self):
        """전원이 켜져 있을 때만 시리얼 데이터를 읽고 ROS 2 토픽으로 발행합니다."""
        PORT_NAME = '/dev/ttyUSB0'
        BAUDRATE = 115200
        
        while rclpy.ok():
            if not self.is_powered_on:
                # 전원이 꺼져 있으면 대기합니다.
                rclpy.spin_once(self, timeout_sec=1.0)
                continue

            try:
                ser = serial.Serial(PORT_NAME, BAUDRATE, timeout=1)
                while self.is_powered_on and rclpy.ok():
                    if ser.read(1) == b'\x54':
                        packet = b'\x54' + ser.read(46)
                        if len(packet) == 47:
                            # ------------------------------------------------
                            # 이곳에 이전에 작성했던 parse_ld08_packet 로직과
                            # ROS 2 LaserScan 메시지로 변환하는 코드가 들어갑니다.
                            # ------------------------------------------------
                            
                            # 예시: 파싱이 끝난 데이터를 퍼블리시
                            # msg = LaserScan()
                            # ... 메시지 데이터 채우기 ...
                            # self.scan_pub.publish(msg)
                            pass
            except serial.SerialException:
                # 전원이 차단되어 장치가 사라졌을 때의 예외 처리
                self.get_logger().warn("라이다 장치를 찾을 수 없거나 연결이 끊어졌습니다. 재연결 대기 중...")
                rclpy.spin_once(self, timeout_sec=2.0)
            finally:
                if 'ser' in locals() and ser.is_open:
                    ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = LidarIntegratedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()