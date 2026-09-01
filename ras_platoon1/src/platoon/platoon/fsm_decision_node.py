"""
FSM 기반 판단 노드
- platoon_fsm.py 모듈을 불러와 ROS 2 환경에서 가동
"""

import rclpy
from rclpy.node import Node

# 동일 패키지 내의 platoon_fsm 모듈 import
from .platoon_fsm import PlatoonFSM, EgoState, NearbyVehicle, DrivingCommand

from platoon_interfaces.msg import LaneInfo, Telemetry, VehicleCmd

from std_msgs.msg import String


class FsmDecisionNode(Node):
    def __init__(self):
        super().__init__('fsm_decision')

        # FSM 상태 모니터링용 토픽 퍼블리셔 추가
        self.pub_fsm_state = self.create_publisher(String, '/fsm_state_debug', 10)

        # 1. 파라미터 선언
        self.declare_parameter('vehicle_id', 1)
        self.declare_parameter('is_designated_leader', False)

        vehicle_id = self.get_parameter('vehicle_id').value
        is_designated_leader = self.get_parameter('is_designated_leader').value

        # 2. FSM 객체 생성 (친구분의 코드를 그대로 인스턴스화)
        self.fsm = PlatoonFSM(
            vehicle_id=vehicle_id,
            is_designated_leader=is_designated_leader
        )

        # 자차 상태(EgoState) 및 주변 차량 정보 저장용 변수
        self.ego_state = EgoState()
        self.nearby_vehicles = []  # 추후 ESP32 V2X 수신 시 채워질 리스트

        # 3. 구독 및 발행
        self.sub_lane = self.create_subscription(
            LaneInfo, '/lane_info', self.on_lane_info, 10
        )
        self.sub_tele = self.create_subscription(
            Telemetry, '/telemetry', self.on_telemetry, 10
        )
        self.pub_cmd = self.create_publisher(
            VehicleCmd, '/vehicle_cmd', 10
        )

        # 4. 20Hz (0.05초) 주기로 FSM update 실행
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'FSM 판단 노드 실행 시작 (ID: {vehicle_id}, Leader: {is_designated_leader})')

    def on_lane_info(self, msg: LaneInfo):
        """차선 인지 정보를 EgoState로 전달"""
        self.ego_state.lane_detected = msg.lane_detected
        # 픽셀 offset을 -1.0 ~ +1.0 범위로 정규화 (카메라 Width 640px 기준)
        self.ego_state.lane_offset = float(msg.offset) / 320.0
        self.ego_state.lane = 0

    def on_telemetry(self, msg: Telemetry):
        """STM32 텔레메트리 정보를 EgoState로 전달"""
        # distance_cm (cm) -> front_distance (m 단위 변환)
        if msg.distance_cm > 0:
            self.ego_state.front_distance = float(msg.distance_cm) / 100.0
        else:
            self.ego_state.front_distance = None

    def control_loop(self):
        """20Hz 메인 제어 주기로 FSM 실행 및 출력값 매핑"""
        # 1. FSM 연산 수행
        cmd: DrivingCommand = self.fsm.update(self.ego_state, self.nearby_vehicles)

        # 2. [추가] FSM 상태 및 모드 디버그 문자열 발행
        state_msg = String()
        # fsm.state (PlatoonState Enum) 와 cmd.mode 문자열 출력
        state_msg.data = f"State: {self.fsm.state.name} | MatchState: {self.fsm.match_state.name} | Mode: {cmd.mode} | TargetSpeed: {cmd.target_speed}"
        self.pub_fsm_state.publish(state_msg)

        
        # 2. DrivingCommand -> VehicleCmd 매핑
        vehicle_cmd = VehicleCmd()

        # [속도 판단]
        if cmd.emergency or (cmd.target_speed is not None and cmd.target_speed == 0.0):
            vehicle_cmd.speed_mode = 0  # 정지
        elif cmd.target_speed is not None and cmd.target_speed < 0.3:
            vehicle_cmd.speed_mode = 1  # 감속 / 저속
        else:
            vehicle_cmd.speed_mode = 2  # 크루즈 / 정상 속도

        # [조향각 판단]
        # FSM에서 구한 차선 오프셋을 조향각(-70 ~ +70)으로 P 제어 변환
        steer = int(self.ego_state.lane_offset * -70.0)
        vehicle_cmd.steering_deg = max(-70, min(70, steer))

        # 3. 하위 제어 노드로 전송
        self.pub_cmd.publish(vehicle_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FsmDecisionNode()
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