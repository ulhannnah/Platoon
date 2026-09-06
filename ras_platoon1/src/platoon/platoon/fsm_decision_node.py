"""
통합 FSM 및 조향 제어 노드 (Merged FsmDecisionNode)

- ROS 2 서비스/파라미터 클라이언트를 통하지 않고 내부 변수로 즉시 상태를 전환합니다.
- Telemetry 콜백에서 전방 거리, 엔코더 이동 거리 및 현재 조향각(steer_deg)을 갱신합니다.
- 카메라(LaneInfo) 콜백에서 조향각과 속도를 계산하고 직접 VehicleCmd를 발행합니다.
"""

import math
import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, SetParametersResult
from std_msgs.msg import String

from platoon_interfaces.msg import LaneInfo, VehicleCmd, Telemetry, SelfStatus

# fsm_test.py 임포트
from .fsm_test import PlatoonFSM, EgoState, NearbyVehicle, DrivingCommand, PlatoonState

# ── 독립 상수 정의 ──
DRIVING_STATE_AUTO = 0
DRIVING_STATE_PLATOON = 1

PLATOON_STATE_SOLO = 0
PLATOON_STATE_JOIN = 1
PLATOON_STATE_KEEP = 2
PLATOON_STATE_EXIT = 3

PLATOON_ROLE_NONE = 0
PLATOON_ROLE_LEADER = 1
PLATOON_ROLE_FOLLOWER = 2

# fsm_test.py 상태 -> int 상태 매핑
_STATE_TO_WIRE = {
    "SOLO_DRIVE": PLATOON_STATE_SOLO,
    "PLATOON_JOIN": PLATOON_STATE_JOIN,
    "PLATOON_MAINTAIN": PLATOON_STATE_KEEP,
    "PLATOON_EXIT": PLATOON_STATE_EXIT,
    "CONVOY_EXIT": PLATOON_STATE_KEEP,
}

SPEED_CRUISE = 2

# 제어 노드의 물리적 한계를 반영한 최대/최소 조향각 (-60 ~ 60)
STEER_MAX = 60
STEER_MIN = -60

# 차선 변경 파라미터
TURN_ANGLE = 35
RELEASE_ANGLE = 0
PHASE1_DIST = 0.5
TARGET_CHANGE_DISTANCE = 0.7

class FsmDecisionNode(Node):
    def __init__(self):
        super().__init__('fsm_decision')

        # ── 1. 파라미터 선언 ──────────────────────────────────────────
        # FSM 및 군집 정보 관련
        self.declare_parameter('vehicle_id', 102)
        self.declare_parameter('platoon_id', 1)  # 군집 ID
        self.declare_parameter('is_designated_leader', False)
        self.declare_parameter('leader_id', 0)
        self.declare_parameter('initial_partner_id', 0)
        self.declare_parameter('expected_follower_ids', '')
        self.declare_parameter('default_join_lane', 1)
        self.declare_parameter('default_exit_lane', 2)
        self.declare_parameter('initial_lane', 1)
        
        # 조향 제어 관련
        self.declare_parameter('is_running', False)
        self.declare_parameter('lost_stop', True)
        self.declare_parameter('kp_gain', 0.13)
        self.declare_parameter('kd_gain', 0.05)
        self.declare_parameter('ff_gain', 10.0)

        # 파라미터 값 가져오기
        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.platoon_id = self.get_parameter('platoon_id').value
        is_designated_leader = self.get_parameter('is_designated_leader').value
        leader_id = self.get_parameter('leader_id').value or None
        initial_partner_id = self.get_parameter('initial_partner_id').value or None
        expected_followers_raw = self.get_parameter('expected_follower_ids').value
        expected_followers = [int(x) for x in expected_followers_raw.split(',') if x.strip()]
        
        self.default_join_lane = int(self.get_parameter('default_join_lane').value)
        self.default_exit_lane = int(self.get_parameter('default_exit_lane').value)
        initial_lane = int(self.get_parameter('initial_lane').value)

        self.is_running = self.get_parameter('is_running').value
        self.lost_stop = self.get_parameter('lost_stop').value
        self.kp_gain = self.get_parameter('kp_gain').value
        self.kd_gain = self.get_parameter('kd_gain').value
        self.ff_gain = self.get_parameter('ff_gain').value

        # ── 2. 상태 관리 변수 ─────────────────────────────────────────
        # FSM 상태
        self.fsm = PlatoonFSM(
            vehicle_id=self.vehicle_id,
            is_designated_leader=is_designated_leader,
            leader_id=leader_id,
            initial_partner_id=initial_partner_id,
            expected_follower_ids=expected_followers,
        )
        self.ego_state = EgoState(lane=initial_lane)
        
        self._pending_target_lane = None
        self._last_emergency = False
        
        # 조향 제어 변수
        self.steering_state = 'STRAIGHT'
        self.platoon_speed_level = -1
        self.lane_change_dir = ''
        self.cruise_speed = 2
        
        self.current_distance = 0.0
        self.change_start_distance = 0.0
        self.target_lane_offset = 0.0
        self.prev_offset = 0.0
        self.prev_time = self.get_clock().now()

        # Telemetry 모니터링 변수 (SelfStatus 반영용)
        self.latest_right_delta = 0
        self.latest_heading_deg = 0

        # 엔코더 변환 상수
        WHEEL_DIAMETER = 0.065
        MOTOR_PPR = 11.0
        GEAR_RATIO = 30.0
        ENCODER_CPR = MOTOR_PPR * 4.0 * GEAR_RATIO
        self.meters_per_tick = (math.pi * WHEEL_DIAMETER) / ENCODER_CPR

        # ── 3. ROS 클라이언트 및 통신 ──────────────────────────────────
        self.add_on_set_parameters_callback(self.on_param_change)
        
        self._control_param_client = self.create_client(
            SetParameters, 'control_node/set_parameters'
        )
        self._last_cruise_duty_sent = None

        # 구독
        self.create_subscription(Telemetry, 'telemetry', self.on_telemetry, 10)
        self.create_subscription(LaneInfo, 'lane_info', self.on_lane_info, 10)
        self.create_subscription(String, 'platoon_cmd', self.on_platoon_cmd, 10)
        
        # 발행
        self.pub_vehicle_cmd = self.create_publisher(VehicleCmd, 'vehicle_cmd', 10)
        self.pub_fsm_state = self.create_publisher(String, 'fsm_state_debug', 10)
        
        # SelfStatus 퍼블리셔 추가
        self.pub_self_status = self.create_publisher(SelfStatus, 'self_status', 10)

        # 20Hz 판단 루프
        self.create_timer(0.2, self.fsm_loop)

    def on_param_change(self, params):
        for param in params:
            if param.name == 'kp_gain': self.kp_gain = float(param.value)
            elif param.name == 'kd_gain': self.kd_gain = float(param.value)
            elif param.name == 'ff_gain': self.ff_gain = float(param.value)
            elif param.name == 'is_running': self.is_running = bool(param.value)
            elif param.name == 'lost_stop': self.lost_stop = bool(param.value)
        return SetParametersResult(successful=True)

    # ══════════════════════════════════════════════════════════════
    # 콜백 함수들
    # ══════════════════════════════════════════════════════════════
    def on_telemetry(self, msg: Telemetry):
        # 1. 초음파 거리 (cm -> m 변환)
        if msg.dist_cm > 0:
            self.ego_state.front_distance = float(msg.dist_cm) / 100.0
        else:
            self.ego_state.front_distance = None
            
        # 2. 엔코더 이동거리 갱신
        tick_delta = (msg.left_delta + msg.right_delta) / 2.0
        delta_meters = tick_delta * self.meters_per_tick
        self.current_distance += delta_meters
        
        # 3. SelfStatus에 반영할 물리 정보 저장 (int로 캐스팅)
        self.latest_right_delta = int(msg.right_delta)
        if hasattr(msg, 'steer_deg'):
            self.latest_heading_deg = int(msg.steer_deg)

    def on_platoon_cmd(self, msg: String):
        raw = msg.data.strip()
        if ':' in raw:
            cmd, lane_str = raw.split(':', 1)
            try: target_lane = int(lane_str)
            except ValueError: target_lane = None
        else:
            cmd = raw
            target_lane = None

        cmd = cmd.strip().upper()
        if target_lane is None:
            target_lane = self.default_join_lane if cmd == 'JOIN' else self.default_exit_lane

        self._pending_target_lane = target_lane
        self.fsm.handle_command(cmd, target_lane=target_lane, current_lane=self.ego_state.lane)

        if cmd == 'JOIN':
            self.is_running = True
            if self.fsm.state == PlatoonState.PLATOON_JOIN:

        self.get_logger().info(f'명령 수신: {cmd} (목표 lane={target_lane})')

    # ══════════════════════════════════════════════════════════════
    # 20Hz 판단 루프 (FSM 상태 업데이트 및 브로드캐스트)
    # ══════════════════════════════════════════════════════════════
    def fsm_loop(self):
        cmd: DrivingCommand = self.fsm.update(self.ego_state, [])
        self._last_emergency = cmd.emergency

        state_msg = String()
        state_msg.data = (
            f"State: {self.fsm.state.name} | Mode: {cmd.mode} | "
            f"speed_level: {cmd.speed_level} | lane: {self.ego_state.lane}"
        )
        self.pub_fsm_state.publish(state_msg)
        
        # SelfStatus 메시지 생성 및 발행
        self.pub_self_status.publish(self._build_self_status())

        if cmd.request_lane_change_dir is not None:
            self._trigger_lane_change(cmd.request_lane_change_dir)

        if cmd.cruise_duty is not None:
            self._apply_cruise_duty(cmd.cruise_duty)
        else:
            self.platoon_speed_level = -1 if cmd.speed_level is None else int(cmd.speed_level)

    def _build_self_status(self) -> SelfStatus:
        s = SelfStatus()
        s.vehicle_id = self.vehicle_id
        
        in_platoon = self.fsm.state.name != "SOLO_DRIVE"
        s.driving_state = DRIVING_STATE_PLATOON if in_platoon else DRIVING_STATE_AUTO
        s.platoon_state = _STATE_TO_WIRE.get(self.fsm.state.name, PLATOON_STATE_SOLO)
        
        s.right_delta = self.latest_right_delta
        s.heading_deg = self.latest_heading_deg
        
        s.platoon_enable = 1 if self.is_running and in_platoon else 0
        s.platoon_id = self.platoon_id
        
        s.platoon_role = (
            PLATOON_ROLE_LEADER if self.fsm.is_designated_leader
            else (PLATOON_ROLE_FOLLOWER if in_platoon else PLATOON_ROLE_NONE)
        )
        
        s.platoon_index = 0 if self.fsm.is_designated_leader else (1 if in_platoon else 0)
        
        s.leader_vehicle_id = self.fsm.leader_id or 0
        s.front_vehicle_id = self.fsm.partner_id or 0
        
        s.emergency = 1 if self._last_emergency else 0
        
        return s

    def _trigger_lane_change(self, direction: str):
        if self.steering_state == 'STRAIGHT':
            self.steering_state = 'CHECKING_DASHED'
            self.lane_change_dir = direction
            self.get_logger().info(f"내부 차선변경 모드 진입 (방향={direction}): 점선을 탐색합니다.")

    def _apply_cruise_duty(self, duty) -> None:
        if duty is None: return
        self.platoon_speed_level = SPEED_CRUISE
        duty = int(duty)
        if duty == self._last_cruise_duty_sent: return
        self._last_cruise_duty_sent = duty
        
        pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=duty)
        req = SetParameters.Request(parameters=[Parameter(name='cruise_duty', value=pv)])
        if self._control_param_client.service_is_ready():
            self._control_param_client.call_async(req)

    # ══════════════════════════════════════════════════════════════
    # 주행 제어 루프
    # ══════════════════════════════════════════════════════════════
    def on_lane_info(self, msg: LaneInfo):
        cmd = VehicleCmd()
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0: dt = 0.033 

        self._update_steering_state(msg)

        target_steer = 0.0
        traveled_distance = self.current_distance - self.change_start_distance
        
        if self.steering_state == 'CHANGING_LEFT':
            target_steer = self._execute_change_left(traveled_distance)
        elif self.steering_state == 'CHANGING_RIGHT':
            target_steer = self._execute_change_right(traveled_distance)
        else:
            target_steer = self._execute_straight(msg, dt)

        self.prev_time = current_time

        target_steer = max(STEER_MIN, min(STEER_MAX, target_steer))
        cmd.steering_deg = int(target_steer)

        is_changing = self.steering_state in ['CHANGING_LEFT', 'CHANGING_RIGHT']
        if not self.is_running:
            cmd.speed_mode = 0  
            cmd.steering_deg = 0
        elif not msg.lane_detected and self.lost_stop and not is_changing:
            cmd.speed_mode = 0
            cmd.steering_deg = 0
        elif self.platoon_speed_level >= 0:
            cmd.speed_mode = self.platoon_speed_level
        else:
            cmd.speed_mode = self.cruise_speed

        self.pub_vehicle_cmd.publish(cmd)

    def _update_steering_state(self, msg: LaneInfo):
        if self.steering_state == 'CHECKING_DASHED':
            is_left_dashed = (msg.left_style == LaneInfo.DASHED)
            is_right_dashed = (msg.right_style == LaneInfo.DASHED)

            if self.lane_change_dir == 'left': is_right_dashed = False
            elif self.lane_change_dir == 'right': is_left_dashed = False

            if is_left_dashed:
                self.steering_state = 'CHANGING_LEFT'
                self.change_start_distance = self.current_distance
            elif is_right_dashed:
                self.steering_state = 'CHANGING_RIGHT'
                self.change_start_distance = self.current_distance

        elif self.steering_state in ['CHANGING_LEFT', 'CHANGING_RIGHT']:
            traveled_distance = max(0.0, self.current_distance - self.change_start_distance)
            
            current_target = TARGET_CHANGE_DISTANCE
            if self.steering_state == 'CHANGING_LEFT':
                current_target = TARGET_CHANGE_DISTANCE * 1.8

            if traveled_distance >= current_target:
                self.get_logger().info(f'차선 변경 완료. 직진 모드로 복귀합니다.')
                self.steering_state = 'STRAIGHT'
                self.target_lane_offset = 0.0
                
                self.fsm.notify_lane_change_done()
                if self._pending_target_lane is not None:
                    self.ego_state.lane = self._pending_target_lane
                    self._pending_target_lane = None

    def _execute_straight(self, msg: LaneInfo, dt: float) -> float:
        if not msg.lane_detected:
            self.prev_offset = 0.0
            return 0.0

        left_exist = (msg.left_style != LaneInfo.UNKNOWN)
        right_exist = (msg.right_style != LaneInfo.UNKNOWN)
        avg_slope = 0.0
        
        if left_exist and right_exist: avg_slope = (msg.left_slope + msg.right_slope) / 2.0
        elif left_exist: avg_slope = msg.left_slope * 2.5
        elif right_exist: avg_slope = msg.right_slope * 2.5 
        
        ff_steer = avg_slope * self.ff_gain
        effective_offset = msg.offset - self.target_lane_offset
        p_steer = effective_offset * self.kp_gain
        d_steer = self.kd_gain * ((effective_offset - self.prev_offset) / dt)

        target_steer = p_steer + d_steer + ff_steer
        self.prev_offset = effective_offset
        return target_steer

    def _execute_change_left(self, traveled_distance: float) -> float:
        return TURN_ANGLE if traveled_distance < PHASE1_DIST else RELEASE_ANGLE

    def _execute_change_right(self, traveled_distance: float) -> float:
        return -TURN_ANGLE if traveled_distance < PHASE1_DIST else -RELEASE_ANGLE


def main(args=None):
    rclpy.init(args=args)
    node = FsmDecisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()