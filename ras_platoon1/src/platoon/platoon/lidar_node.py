import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import threading
import math
import time

class LidarIntegratedNode(Node):
    def __init__(self):
        super().__init__('lidar_integrated_node')
        
        # 1. 라이다 데이터 토픽 발행자 (Publisher)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # 2. 백그라운드 데이터 수신 스레드 실행
        self.serial_thread = threading.Thread(target=self.read_and_publish_lidar, daemon=True)
        self.serial_thread.start()
        
        self.get_logger().info('라이다 노드가 시작되었습니다. (데이터 수신 및 토픽 발행 전용)')

    def read_and_publish_lidar(self):
        PORT_NAME = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        BAUDRATE = 115200
        ranges = [float('inf')] * 360
        prev_angle = 0.0
        
        while rclpy.ok():
            try:
                ser = serial.Serial(PORT_NAME, BAUDRATE, timeout=1)
                self.get_logger().info(f'{PORT_NAME} 시리얼 포트 연결 성공.')
                
                while rclpy.ok():
                    if ser.read(1) == b'\x54':
                        packet = b'\x54' + ser.read(46)
                        if len(packet) == 47:
                            start_angle = (packet[5] << 8 | packet[4]) / 100.0
                            end_angle = (packet[43] << 8 | packet[42]) / 100.0
                            step = (end_angle - start_angle)
                            if step < 0:
                                step += 360.0
                            step /= 11.0
                            
                            for i in range(12):
                                angle = start_angle + step * i
                                if angle >= 360.0:
                                    angle -= 360.0
                                
                                # 한 바퀴(360도) 완료 감지 시 토픽 발행
                                if angle < prev_angle - 10.0:
                                    msg = LaserScan()
                                    msg.header.stamp = self.get_clock().now().to_msg()
                                    msg.header.frame_id = 'laser_frame'
                                    msg.angle_min = 0.0
                                    msg.angle_max = math.pi * 2
                                    msg.angle_increment = (math.pi * 2) / 360.0
                                    msg.range_min = 0.02
                                    msg.range_max = 12.0
                                    msg.ranges = ranges
                                    
                                    self.scan_pub.publish(msg)
                                    ranges = [float('inf')] * 360
                                
                                prev_angle = angle
                                dist_idx = 6 + i * 3
                                distance_mm = (packet[dist_idx + 1] << 8) | packet[dist_idx]
                                
                                if distance_mm > 0:
                                    idx = int(angle) % 360
                                    ranges[idx] = distance_mm / 1000.0
                                    
            except serial.SerialException as e:
                self.get_logger().warn(f"시리얼 연결 대기 중... ({e})")
                time.sleep(1.0)
            finally:
                if 'ser' in locals() and ser.is_open:
                    ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = LidarIntegratedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()