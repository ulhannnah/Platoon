"""
속도/조향각 판단 노드 (Decision Node)
- /lane_info 데이터를 수신하여 목표 조향각 및 속도 모드 결정
- P, D, FF 통합 제어로 오버슈트 방지 및 부드러운 코너링 구현
- 결과를 /vehicle_cmd 로 발행
"""
import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from platoon_interfaces.msg import LaneInfo, VehicleCmd

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

STEER_MAX = 100
STEER_MIN = -100

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision')
        
        self.cruise_speed = 2
        
        # ROS 2 동적 파라미터 등록
        self.declare_parameter('is_running', False) 
        self.declare_parameter('lost_stop', True)
        self.declare_parameter('kp_gain', 0.13)
        self.declare_parameter('kd_gain', 0.05)
        self.declare_parameter('ff_gain', 10.0)

        self.is_running = self.get_parameter('is_running').value
        self.lost_stop = self.get_parameter('lost_stop').value
        self.kp_gain = self.get_parameter('kp_gain').value
        self.kd_gain = self.get_parameter('kd_gain').value
        self.ff_gain = self.get_parameter('ff_gain').value


        # 시작 시 파라미터 설정 상태 로그 출력
        self.get_logger().info('=============== Decision Node Parameters ===============')
        self.get_logger().info(f' 1. 상태 : is_running={self.is_running}, lost_stop={self.lost_stop}')
        self.get_logger().info(f' 2. 조향 : kp_gain={self.kp_gain}, kd_gain={self.kd_gain}, ff_gain={self.ff_gain}')
        self.get_logger().info('========================================================')


        self.prev_offset = 0.0
        self.prev_time = self.get_clock().now()

        # 실시간 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self.on_param_change)
        
        self.sub = self.create_subscription(LaneInfo, '/lane_info', self.on_lane_info, 10)
        self.pub = self.create_publisher(VehicleCmd, '/vehicle_cmd', 10)
    def on_param_change(self, params):
        for param in params:
            if param.name == 'kp_gain': self.kp_gain = float(param.value)
            elif param.name == 'kd_gain': self.kd_gain = float(param.value)
            elif param.name == 'ff_gain': self.ff_gain = float(param.value)
            elif param.name == 'is_running': self.is_running = bool(param.value)
            elif param.name == 'lost_stop': self.lost_stop = bool(param.value)
        return SetParametersResult(successful=True)

    def on_lane_info(self, msg: LaneInfo):
        cmd = VehicleCmd()
        current_time = self.get_clock().now()

        target_steer = 0.0

        if not msg.lane_detected:
            self.prev_offset = 0.0
        else:
            # 1. 미분 제어용 dt 계산 (초 단위)
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt <= 0.0: 
                dt = 0.033 

            # 2. Feed-Forward 보상 (기울기 기반)
            left_exist = (msg.left_style != LaneInfo.UNKNOWN)
            right_exist = (msg.right_style != LaneInfo.UNKNOWN)
            avg_slope = 0.0
            
            if left_exist and right_exist: 
                avg_slope = (msg.left_slope + msg.right_slope) / 2.0
            elif left_exist: 
                avg_slope = msg.left_slope
            elif right_exist: 
                avg_slope = msg.right_slope
            
            ff_steer = avg_slope * self.ff_gain

            # 3. 비례-미분(PD) 제어
            p_steer = msg.offset * self.kp_gain
            d_steer = self.kd_gain * ((msg.offset - self.prev_offset) / dt)

            # 4. 최종 조향 연산
            target_steer = p_steer + d_steer + ff_steer
            self.prev_offset = msg.offset

        self.prev_time = current_time

        # 제한값 클램핑
        target_steer = max(STEER_MIN, min(STEER_MAX, target_steer))
        cmd.steering_deg = int(target_steer)

        # 주행 속도 모드 결정
        if not self.is_running or (not msg.lane_detected and self.lost_stop):
            cmd.speed_mode = 0  
        else:
            cmd.speed_mode = self.cruise_speed  

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()