"""
FSM 기반 판단 노드
- platoon_fsm.py 모듈을 불러와 ROS 2 환경에서 가동

V2X 연동 (v2x_node.py와 토픽으로만 연결, 서로 import 안 함):
    구독 /v2x/targets       (V2xTargets) → NearbyVehicle 리스트로 변환해 FSM에 공급
    발행 /v2x/self_status   (SelfStatus) → v2x_node.py가 그대로 ESP32로 중계
"""

import math

import rclpy
from rclpy.node import Node

# 동일 패키지 내의 platoon_fsm 모듈 import
from .platoon_fsm import PlatoonFSM, EgoState, NearbyVehicle, DrivingCommand
from .v2x_node import (
    DRIVING_STATE_AUTO, DRIVING_STATE_PLATOON,
    PLATOON_STATE_SOLO, PLATOON_STATE_JOIN, PLATOON_STATE_KEEP, PLATOON_STATE_EXIT,
    PLATOON_ROLE_NONE, PLATOON_ROLE_LEADER, PLATOON_ROLE_FOLLOWER,
)

from platoon_interfaces.msg import LaneInfo, Telemetry, VehicleCmd, V2xTargets, SelfStatus

from std_msgs.msg import String

_PLATOON_STATE_MAP = {
    "SOLO_DRIVE": PLATOON_STATE_SOLO,
    "PLATOON_JOIN": PLATOON_STATE_JOIN,
    "PLATOON_MAINTAIN": PLATOON_STATE_KEEP,
    "PLATOON_EXIT": PLATOON_STATE_EXIT,
}
_ROLE_MAP = {None: PLATOON_ROLE_NONE, "LEADER": PLATOON_ROLE_LEADER, "FOLLOWER": PLATOON_ROLE_FOLLOWER}


class FsmDecisionNode(Node):
    def __init__(self):
        super().__init__('fsm_decision')

        # FSM 상태 모니터링용 토픽 퍼블리셔 추가
        self.pub_fsm_state = self.create_publisher(String, '/fsm_state_debug', 10)

        # 1. 파라미터 선언
        # 기본값은 이 차량(리더)을 기준으로 잡아둔다. 팔로워 차량은 실행할 때
        # --ros-args -p vehicle_id:=102 -p is_designated_leader:=false ... 로
        # 덮어써서 쓴다 (docs/250901_차량별_설정값_정리.md 참고).
        self.declare_parameter('vehicle_id', 101)
        self.declare_parameter('is_designated_leader', True)
        self.declare_parameter('uwb_id', 0)          # self_status 송신용 — ESP32와 맞는 값으로 지정 필요
        self.declare_parameter('destination_id', 0)  # 목적지 인식 방법 미정이라 당분간 고정값

        vehicle_id = self.get_parameter('vehicle_id').value
        is_designated_leader = self.get_parameter('is_designated_leader').value
        self.vehicle_id = vehicle_id
        self.uwb_id = self.get_parameter('uwb_id').value
        self.destination_id = self.get_parameter('destination_id').value

        # 2. FSM 객체 생성
        self.fsm = PlatoonFSM(
            vehicle_id=vehicle_id,
            is_designated_leader=is_designated_leader
        )

        # 자차 상태(EgoState) 및 주변 차량 정보 저장용 변수
        self.ego_state = EgoState()
        self.nearby_vehicles = []  # /v2x/targets 수신 시 on_v2x_targets()가 채움

        # 3. 구독 및 발행
        self.sub_lane = self.create_subscription(
            LaneInfo, '/lane_info', self.on_lane_info, 10
        )
        self.sub_tele = self.create_subscription(
            Telemetry, '/telemetry', self.on_telemetry, 10
        )
        self.sub_v2x = self.create_subscription(
            V2xTargets, '/v2x/targets', self.on_v2x_targets, 10
        )
        self.pub_cmd = self.create_publisher(
            VehicleCmd, '/vehicle_cmd', 10
        )
        self.pub_self_status = self.create_publisher(
            SelfStatus, '/v2x/self_status', 10
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
        """
        STM32 텔레메트리 정보를 EgoState로 전달.

        ego_state.speed는 여기서 채워야 self_status.speed_mps로 다른 차량에
        실제 속도가 나간다. control_node.py가 엔코더로 계산해서 채워주는
        msg.speed_mps를 그대로 받는다.
        """
        self.ego_state.speed = msg.speed_mps

        # dist_cm (cm) -> front_distance (m 단위 변환)
        # Telemetry.msg의 실제 필드명은 dist_cm이다 (distance_cm 아님 — 예전에
        # distance_cm으로 읽어서 매 호출마다 AttributeError 나던 버그가 있었음).
        if msg.dist_cm > 0:
            self.ego_state.front_distance = float(msg.dist_cm) / 100.0
        else:
            self.ego_state.front_distance = None

    def on_v2x_targets(self, msg: V2xTargets):
        """
        v2x_node.py가 ESP32에서 받은 주변 차량 목록을 NearbyVehicle 리스트로 변환.
        """
        self.nearby_vehicles = [
            NearbyVehicle(
                vehicle_id=t.vehicle_id,
                speed=t.speed_mps,
                heading=math.radians(t.heading_deg),
                platoon_allow=bool(t.platoon_enable),
                platoon_id=(t.platoon_id if t.platoon_id else None),
                leader_id=(t.leader_vehicle_id if t.leader_vehicle_id != -1 else None),
                uwb_distance=t.distance_m,
                uwb_angle=math.radians(t.angle_deg),
                timestamp=msg.timestamp_ms / 1000.0,
            )
            for t in msg.targets
        ]

    def _build_self_status(self, cmd: DrivingCommand) -> SelfStatus:
        """§5 self_status로 내보낼 값을 FSM 현재 상태에서 뽑아 채운다."""
        s = SelfStatus()
        s.vehicle_id = self.vehicle_id
        s.uwb_id = self.uwb_id
        s.destination_id = self.destination_id

        in_platoon = self.fsm.state.name != "SOLO_DRIVE"
        s.driving_state = DRIVING_STATE_PLATOON if in_platoon else DRIVING_STATE_AUTO
        s.platoon_state = _PLATOON_STATE_MAP.get(self.fsm.state.name, PLATOON_STATE_SOLO)

        s.speed_mps = self.ego_state.speed
        s.heading_deg = math.degrees(self.ego_state.heading)

        s.platoon_enable = int(self.ego_state.platoon_allow)
        s.platoon_id = self.fsm.platoon_id or 0
        s.platoon_role = _ROLE_MAP.get(self.fsm.role, PLATOON_ROLE_NONE)

        # TODO: 대열 내 절대 순번은 이 차량 혼자서는 알 수 없다 — 각자 앞/뒤차
        # ID만 안다(platoon_fsm.py의 partner_id/successor_id). 리더는 0으로
        # 확정할 수 있지만 팔로워 순번은 근거가 없어 1로 잠정 처리한다.
        s.platoon_index = 0 if s.platoon_role == PLATOON_ROLE_LEADER else (
            1 if s.platoon_role == PLATOON_ROLE_FOLLOWER else 0
        )

        s.leader_vehicle_id = self.fsm.leader_id if self.fsm.leader_id is not None else 0
        s.front_vehicle_id = (
            self.fsm.partner_id if (self.fsm.role == "FOLLOWER" and self.fsm.partner_id) else 0
        )

        s.target_speed_mps = cmd.target_speed if cmd.target_speed is not None else 0.0
        s.target_gap_m = cmd.target_distance if cmd.target_distance is not None else 0.0
        return s

    def control_loop(self):
        """20Hz 메인 제어 주기로 FSM 실행 및 출력값 매핑"""
        # 1. FSM 연산 수행
        cmd: DrivingCommand = self.fsm.update(self.ego_state, self.nearby_vehicles)

        # 2. FSM 상태 및 모드 디버그 문자열 발행
        state_msg = String()
        state_msg.data = f"State: {self.fsm.state.name} | MatchState: {self.fsm.match_state.name} | Mode: {cmd.mode} | TargetSpeed: {cmd.target_speed}"
        self.pub_fsm_state.publish(state_msg)

        # V2X로 내 상태 보고 (ESP32 -> 주변 차량에 브로드캐스트됨)
        self.pub_self_status.publish(self._build_self_status(cmd))

        # 3. DrivingCommand -> VehicleCmd 매핑
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

        # 4. 하위 제어 노드로 전송
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
