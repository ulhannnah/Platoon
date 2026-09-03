"""
속도/조향각 판단 및 내부 FSM 테스트 노드 (Decision Node)
- 점선 인식 시 목표 오프셋을 조작하여 PD 제어기로 부드러운 차선 변경 수행
"""
import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from platoon_interfaces.msg import LaneInfo, VehicleCmd
from std_srvs.srv import Trigger

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

# 최대 조향각 제한
STEER_MAX = 100
STEER_MIN = -100

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision')
        
        # 기본주행 속도 설정
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

        # --- 내부 FSM 상태 관리 변수 ---
        self.current_state = 'STRAIGHT'  
        self.target_lane_offset = 0.0  # 💡 오프셋 저장 변수 추가
        self.change_start_time = None

        self.get_logger().info('=============== Decision Node Parameters ===============')
        self.get_logger().info(f' 1. 상태 : is_running={self.is_running}, lost_stop={self.lost_stop}')
        self.get_logger().info(f' 2. PD제어 변수 : kp_gain : {self.kp_gain}, kd_gain = {self.kd_gain}')
        self.get_logger().info('========================================================')

        self.prev_offset = 0.0
        self.prev_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.on_param_change)
        
        # 토픽 관리
        self.srv = self.create_service(Trigger, 'request_lane_change', self.lane_change_callback)
        self.sub = self.create_subscription(LaneInfo, 'lane_info', self.on_lane_info, 10)
        self.pub = self.create_publisher(VehicleCmd, 'vehicle_cmd', 10)

    def on_param_change(self, params):
        for param in params:
            if param.name == 'kp_gain': self.kp_gain = float(param.value)
            elif param.name == 'kd_gain': self.kd_gain = float(param.value)
            elif param.name == 'ff_gain': self.ff_gain = float(param.value)
            elif param.name == 'is_running': self.is_running = bool(param.value)
            elif param.name == 'lost_stop': self.lost_stop = bool(param.value)
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

    # FSM 데이터 바탕으로 조향각 결정
    def process_fsm(self, msg: LaneInfo, current_time):
        if self.current_state == 'CHECKING_DASHED':
            is_left_dashed = (msg.left_style == LaneInfo.DASHED)
            is_right_dashed = (msg.right_style == LaneInfo.DASHED)

            # 💡 동적으로 수신한 가상 오프셋을 이동 오프셋으로 설정합니다.
            shift_offset = msg.dynamic_virtual_offset * 1.0 if msg.dynamic_virtual_offset > 0 else 100.0

            if is_left_dashed:  
                self.target_lane_offset = -shift_offset 
                self.current_state = 'CHANGING_LEFT'
                self.change_start_time = current_time
                self.get_logger().info(f'좌측 점선 확인! 동적 오프셋(-{shift_offset:.1f})으로 좌측 변경을 시작합니다.')
            
            elif is_right_dashed:
                self.target_lane_offset = shift_offset   
                self.current_state = 'CHANGING_RIGHT'
                self.change_start_time = current_time
                self.get_logger().info(f'우측 점선 확인! 동적 오프셋({shift_offset:.1f})으로 우측 변경을 시작합니다.')
        
        # 왼쪽으로 갈 때
        elif self.current_state == 'CHANGING_LEFT':
            elapsed = (current_time - self.change_start_time).nanoseconds / 1e9
            
            if elapsed < 1.8:
                return
            both_lanes_valid = (msg.left_style == LaneInfo.SOLID) and (msg.right_style == LaneInfo.DASHED)
            
            if both_lanes_valid and elapsed > 3.0:
                self.current_state = 'STRAIGHT'
                self.target_lane_offset = 0.0
                self.get_logger().info('새로운 차선 안착 완료. 직진 모드로 복귀합니다.')

        # 오른쪽으로 갈 때
        elif self.current_state == 'CHANGING_RIGHT':
            elapsed = (current_time - self.change_start_time).nanoseconds / 1e9
            
            if elapsed < 1.8:
                return
            both_lanes_valid = (msg.left_style == LaneInfo.DASHED) and (msg.right_style == LaneInfo.SOLID)
            
            if both_lanes_valid and elapsed > 3.0:
                self.current_state = 'STRAIGHT'
                self.target_lane_offset = 0.0
                self.get_logger().info('새로운 차선 안착 완료. 직진 모드로 복귀합니다.')

    # LaneInfo msg 들어오면 자동으로 움직임
    def on_lane_info(self, msg: LaneInfo):
        cmd = VehicleCmd()
        current_time = self.get_clock().now()

        # FSM 상태 갱신 및 목표 오프셋 결정
        self.process_fsm(msg, current_time)

        target_steer = 0.0

        if not msg.lane_detected:
            self.prev_offset = 0.0
        else:
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt <= 0.0: dt = 0.033 

            # 차선 존재 여부 확인
            left_exist = (msg.left_style != LaneInfo.UNKNOWN)
            right_exist = (msg.right_style != LaneInfo.UNKNOWN)
            avg_slope = 0.0
            
            # 차선이 둘 다 존재시 반영할 곡선비율(직진이거나 곡선 초입부)
            if left_exist and right_exist: 
                avg_slope = (msg.left_slope + msg.right_slope) / 2.0
            # 한쪽 차선이 없을 경우 반영할 곡선 비율(곡선이니까 더 많이 꺾이게) 
            elif left_exist: avg_slope = msg.left_slope * 2.5
            elif right_exist: avg_slope = msg.right_slope * 2.5 
            
            ff_steer = avg_slope * self.ff_gain

            # 💡 [핵심] PD 제어기에 목표 오프셋을 반영하여 부드럽게 곡선 그리며 이동
            effective_offset = msg.offset - self.target_lane_offset

            p_steer = effective_offset * self.kp_gain
            d_steer = self.kd_gain * ((effective_offset - self.prev_offset) / dt)

            target_steer = p_steer + d_steer + ff_steer
            self.prev_offset = effective_offset

        self.prev_time = current_time

        # 최대 조향각 지정
        target_steer = max(STEER_MIN, min(STEER_MAX, target_steer))
        cmd.steering_deg = int(target_steer)

        # 주행 속도 모드 결정 (차선 변경 중 시야 유실에 의한 정지 방지)
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