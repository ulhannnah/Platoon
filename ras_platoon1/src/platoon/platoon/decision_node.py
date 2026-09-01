"""
속도/조향각 판단 노드 (Decision Node)
- 카메라 인식 결과(/lane_info)를 분석하여 차량의 목표 조향각 및 속도 모드를 결정합니다.
- 결정된 명령을 하위 제어기 전송용 토픽(/vehicle_cmd)으로 발행합니다.
- 실제 PID 모터 제어 및 서보 제어는 하위 제어기(Control Node / STM32)에서 수행합니다.
"""

import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from platoon_interfaces.msg import LaneInfo, VehicleCmd

# 로그 출력시 시간 안나옴
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

# STM32 서보모터 물리적 한계값 (최대 좌/우 조향각)
STEER_MAX = 100
STEER_MIN = -100


class DecisionNode(Node):
    """
    차선 인식 데이터를 바탕으로 주행 판단을 수행하는 ROS 2 노드
    """
    def __init__(self):
        super().__init__('decision')

        # 내부 고정 변수 선언
        self.cruise_speed = 2
        self.branch_speed = 1

        # 1. ROS 2 파라미터 선언 (기본값 설정)
        self.declare_parameter('lost_stop', True)            # 차선 완전 유실 시 정지 여부
        self.declare_parameter('offset_gain', 0.15)          # 카메라 offset -> 목표 조향각 변환 게인
        self.declare_parameter('target_recovery_deg', 25.0)  # 한쪽 차선 유실 시 복귀 조향 각도(deg)
        self.declare_parameter('is_running', False) 
        self.declare_parameter('backlash_comp', 5.0)  

        # 2. 파라미터 값 클래스 변수로 로드
        self.lost_stop = bool(self.get_parameter('lost_stop').value)
        self.offset_gain = float(self.get_parameter('offset_gain').value)
        self.target_recovery_deg = float(self.get_parameter('target_recovery_deg').value)
        self.is_running = bool(self.get_parameter('is_running').value)
        self.backlash_comp = float(self.get_parameter('backlash_comp').value)

        # --- 파라미터 로드 확인용 로그 출력 ---
        self.get_logger().info('=============== Decision Node Parameters ===============')
        self.get_logger().info(f' 1. 상태 : is_running={self.is_running}, lost_stop={self.lost_stop}')
        self.get_logger().info(f' 2. 조향 : offset_gain={self.offset_gain}, target_recovery_deg={self.target_recovery_deg}')
        self.get_logger().info(f' 3. 유격 : backlash_comp={self.backlash_comp}')
        self.get_logger().info('========================================================')
        # -------------------------------------------------------------------

        # 3. 실시간 파라미터 변경 콜백 함수 등록
        self.add_on_set_parameters_callback(self.on_param_change)

        # 4. ROS 2 토픽 구독(Subscription) 및 발행(Publisher) 설정
        # 4-1. 카메라/비전 노드로부터 차선 정보 수신
        self.sub = self.create_subscription(LaneInfo, '/lane_info', self.on_lane_info, 10)
        
        # 4-2. 하위 제어 노드(Control Node)로 차량 제어 명령 발행
        self.pub = self.create_publisher( VehicleCmd, '/vehicle_cmd', 10)

        # 5. 이전 조향 방향 상태 저장 (1: 좌측, -1: 우측, 0: 초기)
        self.last_steer_dir = 0


    def on_param_change(self, params):
        """
        노드 실행 중 파라미터가 동적으로 변경될 때 자동으로 호출되는 콜백 함수
        """
        for param in params:
            if param.name == 'lost_stop':
                self.lost_stop = bool(param.value)
                self.get_logger().info(f'실시간 적용 완료: lost_stop = {self.lost_stop}')
            elif param.name == 'offset_gain':
                self.offset_gain = float(param.value)
                self.get_logger().info(f'실시간 적용 완료: offset_gain = {self.offset_gain}')
            elif param.name == 'target_recovery_deg':
                self.target_recovery_deg = float(param.value)
                self.get_logger().info(f'실시간 적용 완료: target_recovery_deg = {self.target_recovery_deg}')
            elif param.name == 'is_running':
                self.is_running = bool(param.value)
                self.get_logger().info(f'실시간 적용 완료: is_running = {self.is_running}')
            elif param.name == 'backlash_comp':  # [추가]
                self.backlash_comp = float(param.value)
                self.get_logger().info(f'실시간 적용 완료: backlash_comp = {self.backlash_comp}')

        # 파라미터 변경 성공 결과 반환
        return SetParametersResult(successful=True)

    def on_lane_info(self, msg: LaneInfo):
        """
        /lane_info 토픽 수신 시 호출되는 핵심 주행 판단 콜백 함수
        """
        cmd = VehicleCmd()

        # 1. 수신된 차선 메시지 데이터 추출
        lane_detected = msg.lane_detected
        offset = float(msg.offset)

        left_style = msg.left_style
        right_style = msg.right_style

        # 2. 조향각(Steering Angle) 판단 로직
        is_recovering = False
        target_steer = 0.0

        # 2-1. 차선이 완전히 유실된 경우 -> 직진(0도) 유지
        if not lane_detected:
            target_steer = 0.0
        else:
            left_exist = (left_style != LaneInfo.UNKNOWN)
            right_exist = (right_style != LaneInfo.UNKNOWN)

            # 2-2. 오른쪽 차선만 유실된 경우 -> 우측으로 복귀 조향
            if left_exist and not right_exist:
                target_steer = -self.target_recovery_deg
                is_recovering = True
                self.get_logger().warn('오른쪽 차선 유실! 우측 복귀 목표 조향각 설정', throttle_duration_sec=1.0)

            # 2-3. 왼쪽 차선만 유실된 경우 -> 좌측으로 복귀 조향
            elif right_exist and not left_exist:
                target_steer = self.target_recovery_deg
                is_recovering = True
                self.get_logger().warn('왼쪽 차선 유실! 좌측 복귀 목표 조향각 설정', throttle_duration_sec=1.0)

            # 2-4. 양쪽 차선 모두 정상 인지된 경우 -> 비전 offset 기반 비례 제어 조향각 산출
            else:
                target_steer = offset * self.offset_gain

        # 3. 서보모터 하드웨어 보호를 위한 조향각 범위 제한 (Clamping)
        target_steer = max(STEER_MIN, min(STEER_MAX, target_steer))
        cmd.steering_deg = int(target_steer)

        # 4. 속도 모드 결정 (정지 조건 우선 처리)
        if not self.is_running:
            cmd.speed_mode = 0  # 주행 허용 스위치 OFF 시 정지
        elif not lane_detected and self.lost_stop:
            cmd.speed_mode = 0  # 차선 완전 유실 시 정지
        else:
            cmd.speed_mode = self.cruise_speed  # 일반 주행

        # 5. 최종 결정된 제어 명령 메시지 발행
        self.pub.publish(cmd)


def main(args=None):
    # 1. ROS 2 클라이언트 라이브러리 초기화
    rclpy.init(args=args)
    
    # 2. DecisionNode 인스턴스 생성
    node = DecisionNode()
    
    # 3. 노드 실행 및 콜백 대기 (Spin)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. 노드 파괴 및 ROS 2 셧다운
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()