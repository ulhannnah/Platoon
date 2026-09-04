import os
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from platoon_interfaces.msg import LaneInfo, VehicleCmd, Telemetry
from std_srvs.srv import Trigger

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

# 최대 조향각 제한
STEER_MAX = 100
STEER_MIN = -100


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision')
        
        # 기본변수들 설정
        self.cruise_speed = 2
        self.lost_stop = True
        
        # ROS 2 동적 파라미터 등록
        self.declare_parameter('is_running', False) 
        self.declare_parameter('kp_gain', 0.13)
        self.declare_parameter('kd_gain', 0.05)
        self.declare_parameter('ff_gain', 10.0)
        self.declare_parameter('min_change_distance', 0.3)
        self.declare_parameter('complete_change_distance', 1.8)
        

        self.is_running = self.get_parameter('is_running').value
        self.kp_gain = self.get_parameter('kp_gain').value
        self.kd_gain = self.get_parameter('kd_gain').value
        self.ff_gain = self.get_parameter('ff_gain').value
        self.min_change_distance = self.get_parameter('min_change_distance').value
        self.complete_change_distance = self.get_parameter('complete_change_distance').value


        # --- 내부 FSM 상태 관리 변수 ---
        self.current_state = 'STRAIGHT'  
        self.target_lane_offset = 0.0  
        
        # --- 누적 이동 거리 관리를 위한 변수 ---
        self.current_speed_mps = 0.0
        self.total_distance = 0.0          
        self.change_start_distance = 0.0   

        # 💡 [66mm 바퀴 규격 반영] 바퀴 둘레 계산 (지름 66mm = 0.066m)
        self.WHEEL_CONVERSION_FACTOR = 1.0 

        self.get_logger().info('=============== Decision Node Parameters ===============')
        self.get_logger().info(f' 1. 상태 : is_running={self.is_running}')
        self.get_logger().info(f' 2. PD제어 변수 : kp_gain : {self.kp_gain}, kd_gain = {self.kd_gain}')
        self.get_logger().info(f' 3. 차선변경 변수 : 언제부터 검사 : {self.min_change_distance}m, 언제까지 차선변경 = {self.complete_change_distance}m')
        self.get_logger().info('========================================================')

        self.prev_offset = 0.0
        self.prev_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.on_param_change)
        
        # 토픽 관리
        self.srv = self.create_service(Trigger, 'request_lane_change', self.lane_change_callback)
        self.sub = self.create_subscription(LaneInfo, 'lane_info', self.on_lane_info, 10)
        self.tele_sub = self.create_subscription(Telemetry, 'telemetry', self.on_telemetry, 10)
        self.pub = self.create_publisher(VehicleCmd, 'vehicle_cmd', 10)

    def on_param_change(self, params):
        for param in params:
            if param.name == 'kp_gain': self.kp_gain = float(param.value)
            elif param.name == 'kd_gain': self.kd_gain = float(param.value)
            elif param.name == 'ff_gain': self.ff_gain = float(param.value)
            elif param.name == 'is_running': self.is_running = bool(param.value)
        return SetParametersResult(successful=True)

    def lane_change_callback(self, request, response):
        if self.current_state == 'STRAIGHT':
            self.current_state = 'CHECKING_DASHED'
            response.success = True
            response.message = "차선 변경 모드 진입: 점선을 탐색합니다."
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"현재 {self.current_state} 상태이므로 명령을 무시합니다."
        return response

    # 💡 텔레메트리 콜백에서는 시간 연산 없이 속도 값만 가볍게 수신합니다.
    def on_telemetry(self, msg: Telemetry):
        self.current_speed_mps = msg.speed_mps * self.WHEEL_CONVERSION_FACTOR

    # FSM 데이터 바탕으로 조향각 결정
    def process_fsm(self, msg: LaneInfo):
        if self.current_state == 'CHECKING_DASHED':
            is_left_dashed = (msg.left_style == LaneInfo.DASHED)
            is_right_dashed = (msg.right_style == LaneInfo.DASHED)

            shift_offset = msg.dynamic_virtual_offset * 1.0 if msg.dynamic_virtual_offset > 0 else 100.0

            if is_left_dashed:
                self.target_lane_offset = -shift_offset 
                self.current_state = 'CHANGING_LEFT'
                self.change_start_distance = self.total_distance 
                self.get_logger().info(f'좌측 점선 확인! 동적 오프셋(-{shift_offset:.1f})으로 좌측 변경을 시작합니다.')
            
            elif is_right_dashed:
                self.target_lane_offset = shift_offset   
                self.current_state = 'CHANGING_RIGHT'
                self.change_start_distance = self.total_distance 
                self.get_logger().info(f'우측 점선 확인! 동적 오프셋({shift_offset:.1f})으로 우측 변경을 시작합니다.')
        
        elif self.current_state == 'CHANGING_LEFT':
            traveled = self.total_distance - self.change_start_distance
            
            if traveled < MIN_CHANGE_DISTANCE:
                return
            both_lanes_valid = (msg.left_style == LaneInfo.SOLID) and (msg.right_style == LaneInfo.DASHED)
            
            if both_lanes_valid and traveled > COMPLETE_CHANGE_DISTANCE:
                self.current_state = 'STRAIGHT'
                self.target_lane_offset = 0.0
                self.get_logger().info(f'새로운 차선 안착 완료 (이동거리: {traveled:.2f}m). 직진 모드로 복귀합니다.')

        elif self.current_state == 'CHANGING_RIGHT':
            traveled = self.total_distance - self.change_start_distance
            
            if traveled < MIN_CHANGE_DISTANCE:
                return
            both_lanes_valid = (msg.left_style == LaneInfo.DASHED) and (msg.right_style == LaneInfo.SOLID)
            
            if both_lanes_valid and traveled > COMPLETE_CHANGE_DISTANCE:
                self.current_state = 'STRAIGHT'
                self.target_lane_offset = 0.0
                self.get_logger().info(f'새로운 차선 안착 완료 (이동거리: {traveled:.2f}m). 직진 모드로 복귀합니다.')


    # LaneInfo msg 들어오면 자동으로 움직임
    def on_lane_info(self, msg: LaneInfo):
        cmd = VehicleCmd()
        current_time = self.get_clock().now()

        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0: dt = 0.033 

        # 💡 [최적화] Decision 노드의 주기로 dt를 활용해 이동 거리를 여기서 누적합니다.
        self.total_distance += (self.current_speed_mps * dt)

        # FSM 상태 갱신 및 목표 오프셋 결정
        self.process_fsm(msg)

        target_steer = 0.0

        if not msg.lane_detected:
            self.prev_offset = 0.0
        else:
            left_exist = (msg.left_style != LaneInfo.UNKNOWN)
            right_exist = (msg.right_style != LaneInfo.UNKNOWN)
            avg_slope = 0.0
            
            if left_exist and right_exist: 
                avg_slope = (msg.left_slope + msg.right_slope) / 2.0
            elif left_exist: avg_slope = msg.left_slope * 2.5
            elif right_exist: avg_slope = msg.right_slope * 2.5 
            
            ff_steer = avg_slope * self.ff_gain

            effective_offset = msg.offset - self.target_lane_offset

            p_steer = effective_offset * self.kp_gain
            d_steer = self.kd_gain * ((effective_offset - self.prev_offset) / dt)

            target_steer = p_steer + d_steer + ff_steer
            self.prev_offset = effective_offset

        self.prev_time = current_time

        target_steer = max(STEER_MIN, min(STEER_MAX, target_steer))
        cmd.steering_deg = int(target_steer)

        is_changing = self.current_state in ['CHANGING_LEFT', 'CHANGING_RIGHT']
        
        if not self.is_running:
            cmd.speed_mode = 0  
            cmd.steering_deg = 0
        elif not msg.lane_detected and self.lost_stop and not is_changing:
            cmd.speed_mode = 0  
            cmd.steering_deg = 0
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