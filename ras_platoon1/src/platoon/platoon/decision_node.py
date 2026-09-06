"""
속도/조향각 판단 및 내부 FSM 테스트 노드 (Decision Node)
- 카메라 시야 유실에 대비하여 차선 변경 시 단방향 조향 후 핸들을 풀어 차선을 인식하도록 유도합니다.
- 직선 주행 시에는 PD 제어기를 통해 차선 유지
- 상태별 조향각 계산 로직을 독립된 함수로 분리함
"""
import os
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from platoon_interfaces.msg import LaneInfo, VehicleCmd, Telemetry
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

# 제어 노드의 물리적 한계를 반영한 최대/최소 조향각 (-60 ~ 60)
STEER_MAX = 60
STEER_MIN = -60

# 차선 변경 파라미터 (차량 스케일에 맞춘 세팅)
TURN_ANGLE = 35               # 1차 진입 시 강하게 꺾을 조향각
RELEASE_ANGLE = 0             # 2차 진입 시 핸들을 풀어줄 조향각 (0도 = 직진 상태로 대각선 진입)
PHASE1_DIST = 0.5            # 처음에 핸들을 꺾은 채로 이동할 거리 (m)
TARGET_CHANGE_DISTANCE = 0.8 # 조향 권한을 다시 카메라(PD 제어)로 넘길 총 이동 거리 (m)

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision')
        
        self.current_distance = 0.0

        # ROS 2 동적 파라미터 등록
        self.declare_parameter('is_running', False)
        self.declare_parameter('lost_stop', True)
        self.declare_parameter('kp_gain', 0.13)
        self.declare_parameter('kd_gain', 0.05)
        self.declare_parameter('ff_gain', 10.0)
        # 플래툰 개입 없을 때(platoon_speed_level=-1) 기본으로 내는 speed_mode.
        # 차량마다 다르게 주고 싶으면 launch.py에서 이 파라미터를 덮어쓰면 됨
        # (예: 리더를 계속 저속(1)으로 달리게 하고 싶을 때).
        self.declare_parameter('cruise_speed', 2)
        # fsm_decision_node가 플래툰 상태에 따라 속도를 강제할 때 씀.
        # -1 = 오버라이드 없음(기존처럼 cruise_speed 사용), 0/1/2 = speed_mode 강제 지정.
        self.declare_parameter('platoon_speed_level', -1)
        # fsm_decision_node가 request_lane_change 호출 직전에 세팅.
        # 'left'/'right' 지정 시 그 방향의 점선만 인정(반대쪽은 무시).
        # 빈 문자열이면 기존 동작(먼저 보이는 쪽) 그대로.
        self.declare_parameter('lane_change_dir', '')

        self.is_running = self.get_parameter('is_running').value
        self.lost_stop = self.get_parameter('lost_stop').value
        self.kp_gain = self.get_parameter('kp_gain').value
        self.kd_gain = self.get_parameter('kd_gain').value
        self.ff_gain = self.get_parameter('ff_gain').value
        self.cruise_speed = int(self.get_parameter('cruise_speed').value)
        self.platoon_speed_level = int(self.get_parameter('platoon_speed_level').value)
        self.lane_change_dir = self.get_parameter('lane_change_dir').value

        # --- 내부 FSM 상태 관리 변수 ---
        self.current_state = 'STRAIGHT'  
        self.target_lane_offset = 0.0  
        self.change_start_distance = 0.0 

        # --- 엔코더 변환 상수 설정 ---
        WHEEL_DIAMETER = 0.065  # 바퀴 지름 (예: 65mm -> 0.065m)
        MOTOR_PPR = 11.0        # 모터 1회전당 기본 펄스 수
        GEAR_RATIO = 30.0       # 감속 기어비
        
        # 4체배(TI1 and TI2) 모드가 적용된 총 CPR 계산
        ENCODER_CPR = MOTOR_PPR * 4.0 * GEAR_RATIO
        self.meters_per_tick = (math.pi * WHEEL_DIAMETER) / ENCODER_CPR

        self.prev_offset = 0.0
        self.prev_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.on_param_change)
        
        # 토픽 관리
        # 상대경로로 통일 — launch.py의 namespace=CAR_ID 안에서 control_node의
        # 'telemetry' 발행("/telemetry" 절대경로 아님)과 실제로 연결되려면 여기도
        # 상대경로여야 한다. (친구분이 '/car1/telemetry'로 하드코딩해서 급한 대로
        # car1에서는 연결되게 고쳤었는데, car2/car3에 이 파일 그대로 가져가면 다시
        # 안 붙는다 — request_lane_change 서비스도 같은 이유로 상대경로로 되돌림)
        self.srv = self.create_service(Trigger, 'request_lane_change', self.lane_change_callback)
        self.sub = self.create_subscription(LaneInfo, 'lane_info', self.on_lane_info, 10)
        self.pub = self.create_publisher(VehicleCmd, 'vehicle_cmd', 10)
        self.telemetry_sub = self.create_subscription(Telemetry, 'telemetry', self.telemetry_callback, 10)
        # 차선변경(CHANGING_LEFT/RIGHT -> STRAIGHT) 완료 시 1회 발행.
        # fsm_decision_node가 이걸 받아서 PLATOON_JOIN/EXIT 완료 판정에 씀.
        self.lane_change_done_pub = self.create_publisher(Bool, 'lane_change_done', 10)

    def on_param_change(self, params):
        for param in params:
            if param.name == 'kp_gain': self.kp_gain = float(param.value)
            elif param.name == 'kd_gain': self.kd_gain = float(param.value)
            elif param.name == 'ff_gain': self.ff_gain = float(param.value)
            elif param.name == 'is_running': self.is_running = bool(param.value)
            elif param.name == 'lost_stop': self.lost_stop = bool(param.value)
            elif param.name == 'cruise_speed': self.cruise_speed = int(param.value)
            elif param.name == 'platoon_speed_level': self.platoon_speed_level = int(param.value)
            elif param.name == 'lane_change_dir': self.lane_change_dir = param.value
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

    def update_state(self, msg: LaneInfo, current_distance):
        """FSM 상태 업데이트 전용 함수"""
        if self.current_state == 'CHECKING_DASHED':
            is_left_dashed = (msg.left_style == LaneInfo.DASHED)
            is_right_dashed = (msg.right_style == LaneInfo.DASHED)

            # lane_change_dir가 지정돼 있으면 그 방향의 점선만 인정한다.
            # (안 그러면 양쪽 다 점선일 때 원치 않는 방향으로 바뀔 수 있음 — 플래툰
            #  시나리오처럼 목표 차선이 명확히 정해져 있을 때 방향을 강제하기 위함)
            if self.lane_change_dir == 'left':
                is_right_dashed = False
            elif self.lane_change_dir == 'right':
                is_left_dashed = False

            if is_left_dashed:
                self.current_state = 'CHANGING_LEFT'
                self.change_start_distance = current_distance
                self.get_logger().info('좌측 점선 확인! 좌측 진입 기동을 시작합니다.')

            elif is_right_dashed:
                self.current_state = 'CHANGING_RIGHT'
                self.change_start_distance = current_distance
                self.get_logger().info('우측 점선 확인! 우측 진입 기동을 시작합니다.')

        elif self.current_state in ['CHANGING_LEFT', 'CHANGING_RIGHT']:
            # 음수 거리가 나오지 않도록 0.0으로 클램핑
            traveled_distance = max(0.0, current_distance - self.change_start_distance)

            # 좌측 조향 시 거리가 짧아지는 기구학적 특성을 보정 — 거리 기준점(change_start_distance)이
            # 아니라 목표거리 쪽에 비대칭을 줘야 이미 달려온 거리에 비례해 틀어지지 않는다.
            current_target = TARGET_CHANGE_DISTANCE
            if self.current_state == 'CHANGING_LEFT':
                current_target = TARGET_CHANGE_DISTANCE * 1.8

            self.get_logger().info(f'[FSM] 차선 변경 중... 이동한 거리: {traveled_distance:.3f}m / 목표: {current_target:.3f}m')

            # Phase 3: 보정된 목표 거리 이동 완료 시 직진(PD 제어) 모드 복귀
            if traveled_distance >= current_target:
                self.get_logger().info(f'목표 거리({current_target:.3f}m) 이동 완료. 카메라 인식을 통한 직진 모드로 전환합니다.')
                self.current_state = 'STRAIGHT'
                self.target_lane_offset = 0.0
                self.lane_change_done_pub.publish(Bool(data=True))

    # ---------------------------------------------------------
    # 조향 제어 독립 함수들
    # ---------------------------------------------------------
    def execute_straight(self, msg: LaneInfo, dt: float) -> float:
        """직진(차선 유지) 주행 시 PD 제어기를 통해 조향각을 계산합니다."""
        if not msg.lane_detected:
            self.prev_offset = 0.0
            return 0.0

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

        return target_steer

    def execute_change_left(self, traveled_distance: float) -> float:
        """좌측 차선 변경: 1차 진입 후 핸들을 풀어 사선으로 진입합니다."""
        if traveled_distance < PHASE1_DIST:
            return TURN_ANGLE
        else:
            return RELEASE_ANGLE

    def execute_change_right(self, traveled_distance: float) -> float:
        """우측 차선 변경: 1차 진입 후 핸들을 풀어 사선으로 진입합니다."""
        if traveled_distance < PHASE1_DIST:
            return -TURN_ANGLE
        else:
            return -RELEASE_ANGLE
    # ---------------------------------------------------------

    def on_lane_info(self, msg: LaneInfo):
        cmd = VehicleCmd()
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0: dt = 0.033 

        # 1. FSM 상태 갱신
        self.update_state(msg, self.current_distance)

        # 2. 상태에 따른 조향각 계산 함수 호출
        target_steer = 0.0
        traveled_distance = self.current_distance - self.change_start_distance
        
        if self.current_state == 'CHANGING_LEFT':
            target_steer = self.execute_change_left(traveled_distance)
        elif self.current_state == 'CHANGING_RIGHT':
            target_steer = self.execute_change_right(traveled_distance)
        else:
            target_steer = self.execute_straight(msg, dt)

        self.prev_time = current_time

        # 3. 물리적 한계치(-60 ~ 60)를 반영한 클램핑 적용
        target_steer = max(STEER_MIN, min(STEER_MAX, target_steer))
        cmd.steering_deg = int(target_steer)

        # 4. 주행 속도 모드 결정
        is_changing = self.current_state in ['CHANGING_LEFT', 'CHANGING_RIGHT']
        if not self.is_running:
            cmd.speed_mode = 0  
            cmd.steering_deg = 0
        elif not msg.lane_detected and self.lost_stop and not is_changing:
            cmd.speed_mode = 0
            cmd.steering_deg = 0
        elif self.platoon_speed_level >= 0:
            # fsm_decision_node가 플래툰 거리제어를 위해 강제한 속도단계
            cmd.speed_mode = self.platoon_speed_level
        else:
            cmd.speed_mode = self.cruise_speed

        self.pub.publish(cmd)

    def telemetry_callback(self, msg):
        # 좌우 엔코더 평균으로 이동거리 계산.
        tick_delta = (msg.left_delta + msg.right_delta) / 2.0
        delta_meters = tick_delta * self.meters_per_tick
        self.current_distance += delta_meters

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