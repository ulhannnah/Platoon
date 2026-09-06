"""
FSM 판단 노드 (fsm_test.py 기반 — 촬영 시나리오 전용 경량 버전)

역할 분담:
    decision_node   : 조향(카메라 PD) + STM32로 나갈 VehicleCmd 실제 발행 (유일한 발행자)
    fsm_test.py     : SOLO_DRIVE/JOIN/MAINTAIN/EXIT 상태만 판단하는 순수 로직
    fsm_decision_node(이 파일) : 센서·V2X를 fsm_test.py에 먹이고, 결과를
                       decision_node "조작"으로 바꿔주는 배선판
                       (vehicle_cmd에는 절대 직접 발행하지 않음 — 조향 소유권 충돌 방지)

decision_node 조작 방법:
    - 차선변경: lane_change_dir 파라미터 세팅 후 request_lane_change 서비스 호출
    - 속도(가감속): platoon_speed_level 파라미터 세팅 (-1=오버라이드 해제)
    - 차선변경 완료 통보: decision_node가 발행하는 lane_change_done(Bool) 구독

외부 명령(JOIN/EXIT/EXIT_TOGETHER)은 'platoon_cmd'(std_msgs/String) 토픽으로 받는다.
    예) ros2 topic pub --once /car2/platoon_cmd std_msgs/String "{data: 'JOIN'}"
        ros2 topic pub --once /car2/platoon_cmd std_msgs/String "{data: 'EXIT:2'}"
        ros2 topic pub --once /car1/platoon_cmd std_msgs/String "{data: 'EXIT_TOGETHER:2'}"
"""

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

from .fsm_test import PlatoonFSM, EgoState, NearbyVehicle, DrivingCommand
from .v2x_node import (
    PLATOON_STATE_SOLO, PLATOON_STATE_JOIN, PLATOON_STATE_KEEP, PLATOON_STATE_EXIT,
    PLATOON_ROLE_NONE, PLATOON_ROLE_LEADER, PLATOON_ROLE_FOLLOWER,
    DRIVING_STATE_AUTO, DRIVING_STATE_PLATOON,
)

from platoon_interfaces.msg import Telemetry, V2xTargets, SelfStatus, VehicleCmd

# ESP32 wire 값(int) -> fsm_test.py의 문자열 상태 매핑
_WIRE_TO_STR = {
    PLATOON_STATE_SOLO: "SOLO",
    PLATOON_STATE_JOIN: "JOIN",
    PLATOON_STATE_KEEP: "MAINTAIN",
    PLATOON_STATE_EXIT: "EXIT",
}
# fsm_test.py 상태 -> ESP32 wire 값. CONVOY_EXIT은 "플래툰 유지" 취지라 KEEP으로 보고한다.
_STATE_TO_WIRE = {
    "SOLO_DRIVE": PLATOON_STATE_SOLO,
    "PLATOON_JOIN": PLATOON_STATE_JOIN,
    "PLATOON_MAINTAIN": PLATOON_STATE_KEEP,
    "PLATOON_EXIT": PLATOON_STATE_EXIT,
    "CONVOY_EXIT": PLATOON_STATE_KEEP,
}


class FsmDecisionNode(Node):
    def __init__(self):
        super().__init__('fsm_decision')

        # ── 파라미터 ────────────────────────────────────────────────
        self.declare_parameter('vehicle_id', 101)
        self.declare_parameter('is_designated_leader', True)
        # 팔로워만 의미 있음 — 전체 리더 ID, 그리고 JOIN 시 바로 붙을 내 앞차 ID
        # (3대 이상 체인의 맨 뒤 차량만 leader_id와 다르게 지정)
        self.declare_parameter('leader_id', 0)
        self.declare_parameter('initial_partner_id', 0)
        # 리더만 의미 있음 — 뒤에 붙을 것으로 예정된 차량 ID들, 콤마로 구분 (예: "102,103")
        self.declare_parameter('expected_follower_ids', '')
        # 명령에 lane 번호가 없을 때 쓸 기본 목표차선
        self.declare_parameter('default_join_lane', 1)
        self.declare_parameter('default_exit_lane', 2)
        # 이 차량이 처음 출발하는 차선 (Platoon2/3처럼 진입로에서 대기하는
        # 차량은 리더와 다른 차선에서 시작하므로 반드시 맞게 지정해야 함 —
        # 안 그러면 JOIN 목표차선과 같아 보여 차선변경이 트리거 안 됨)
        self.declare_parameter('initial_lane', 1)

        vehicle_id = self.get_parameter('vehicle_id').value
        is_designated_leader = self.get_parameter('is_designated_leader').value
        leader_id = self.get_parameter('leader_id').value or None
        initial_partner_id = self.get_parameter('initial_partner_id').value or None
        expected_followers_raw = self.get_parameter('expected_follower_ids').value
        expected_followers = [int(x) for x in expected_followers_raw.split(',') if x.strip()]
        self.default_join_lane = int(self.get_parameter('default_join_lane').value)
        self.default_exit_lane = int(self.get_parameter('default_exit_lane').value)

        self.vehicle_id = vehicle_id

        # ── FSM ────────────────────────────────────────────────────
        self.fsm = PlatoonFSM(
            vehicle_id=vehicle_id,
            is_designated_leader=is_designated_leader,
            leader_id=leader_id,
            initial_partner_id=initial_partner_id,
            expected_follower_ids=expected_followers,
        )

        self.ego_state = EgoState(lane=int(self.get_parameter('initial_lane').value))
        self.nearby_vehicles = []
        self._pending_target_lane = None  # lane_change_done 왔을 때 ego_state.lane에 반영할 값
        self._last_speed_level_sent = None  # decision_node 파라미터 스팸 방지용 캐시
        self._last_lane_dir_sent = None
        self._last_emergency = False  # 매 주기 fsm.update() 결과로 갱신, self_status에 반영
        # decision_node가 "지금 실제로" 내는 speed_mode. vehicle_cmd를 읽기 전용으로
        # 구독해서 얻음 (decision_node 수정 없이 CACC 피드포워드용으로 방송하기 위함)
        self._own_speed_level = 2

        # ── decision_node 조작용 클라이언트 ──────────────────────────
        self._decision_param_client = self.create_client(
            SetParameters, 'decision_node/set_parameters'
        )
        self._lane_change_client = self.create_client(
            Trigger, 'request_lane_change'
        )

        # ── 구독/발행 ────────────────────────────────────────────────
        self.create_subscription(Telemetry, 'telemetry', self.on_telemetry, 10)
        self.create_subscription(VehicleCmd, 'vehicle_cmd', self.on_vehicle_cmd, 10)
        self.create_subscription(V2xTargets, 'v2x/targets', self.on_v2x_targets, 10)
        self.create_subscription(Bool, 'lane_change_done', self.on_lane_change_done, 10)
        self.create_subscription(String, 'platoon_cmd', self.on_platoon_cmd, 10)
        self.pub_self_status = self.create_publisher(SelfStatus, 'v2x/self_status', 10)
        self.pub_fsm_state = self.create_publisher(String, 'fsm_state_debug', 10)

        self.create_timer(0.1, self.control_loop)  # 10Hz

        self.get_logger().info(
            f'fsm_decision_node(fsm_test 기반) 시작 (ID: {vehicle_id}, Leader: {is_designated_leader}, '
            f'leader_id={leader_id}, initial_partner_id={initial_partner_id}, followers={expected_followers})'
        )

    # ══════════════════════════════════════════════════════════════
    # 센서/V2X 입력
    # ══════════════════════════════════════════════════════════════
    def on_vehicle_cmd(self, msg: VehicleCmd):
        # decision_node가 실제로 내고 있는 speed_mode를 그대로 관찰 (읽기 전용,
        # 여기서 vehicle_cmd에 발행은 절대 안 함 — 조향 소유권 충돌 방지 원칙 유지)
        self._own_speed_level = int(msg.speed_mode)

    def on_telemetry(self, msg: Telemetry):
        if msg.dist_cm > 0:
            self.ego_state.front_distance = float(msg.dist_cm) / 100.0
        else:
            self.ego_state.front_distance = None

    def on_v2x_targets(self, msg: V2xTargets):
        self.nearby_vehicles = [
            NearbyVehicle(
                vehicle_id=t.vehicle_id,
                platoon_state=_WIRE_TO_STR.get(t.platoon_state, "SOLO"),
                emergency=bool(t.emergency),
                speed_level=int(t.speed_level),
                timestamp=msg.timestamp_ms / 1000.0,
            )
            for t in msg.targets
        ]

    def on_lane_change_done(self, msg: Bool):
        self.fsm.notify_lane_change_done()
        if self._pending_target_lane is not None:
            self.ego_state.lane = self._pending_target_lane
            self._pending_target_lane = None
        self.get_logger().info(f'차선변경 완료 통보 수신 (현재 lane={self.ego_state.lane})')

    def on_platoon_cmd(self, msg: String):
        """예: 'JOIN', 'EXIT:2', 'EXIT_TOGETHER:2'"""
        raw = msg.data.strip()
        if ':' in raw:
            cmd, lane_str = raw.split(':', 1)
            try:
                target_lane = int(lane_str)
            except ValueError:
                target_lane = None
        else:
            cmd = raw
            target_lane = None

        cmd = cmd.strip().upper()
        if target_lane is None:
            target_lane = self.default_join_lane if cmd == 'JOIN' else self.default_exit_lane

        self._pending_target_lane = target_lane
        self.fsm.handle_command(cmd, target_lane=target_lane, current_lane=self.ego_state.lane)

        if cmd == 'JOIN':
            # 정지 대기 중(is_running=False)이던 차량도 JOIN 명령 하나로 바로
            # 출발하도록. decision_node는 is_running=False면 차선변경 상태여도
            # speed_mode/steering을 무조건 0으로 깔아버리므로 이게 없으면 안 움직임.
            self._set_decision_param('is_running', True)

        self.get_logger().info(f'명령 수신: {cmd} (목표 lane={target_lane})')

    # ══════════════════════════════════════════════════════════════
    # decision_node 조작
    # ══════════════════════════════════════════════════════════════
    def _set_decision_param(self, name: str, value) -> None:
        if isinstance(value, bool):
            pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        elif isinstance(value, int):
            pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
        else:
            pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))

        req = SetParameters.Request(parameters=[Parameter(name=name, value=pv)])
        if not self._decision_param_client.service_is_ready():
            self.get_logger().warn(f'decision_node/set_parameters 서비스 준비 안 됨 (param={name})')
            return  
        self._decision_param_client.call_async(req)

    def _request_lane_change(self, direction: str) -> None:
        # 방향 파라미터를 먼저 세팅한 뒤 서비스 호출 (decision_node가 그 방향만 인정)
        self._set_decision_param('lane_change_dir', direction)
        if not self._lane_change_client.service_is_ready():
            self.get_logger().warn('request_lane_change 서비스 준비 안 됨')
            return
        self._lane_change_client.call_async(Trigger.Request())
        self.get_logger().info(f'차선변경 요청 전송 (방향={direction})')

    def _apply_speed_level(self, level) -> None:
        wire_level = -1 if level is None else int(level)
        if wire_level == self._last_speed_level_sent:
            return  # 값 안 바뀌었으면 파라미터 서비스 스팸 방지
        self._last_speed_level_sent = wire_level
        self._set_decision_param('platoon_speed_level', wire_level)

    # ══════════════════════════════════════════════════════════════
    def _build_self_status(self) -> SelfStatus:
        s = SelfStatus()
        s.vehicle_id = self.vehicle_id
        in_platoon = self.fsm.state.name != "SOLO_DRIVE"
        s.driving_state = DRIVING_STATE_PLATOON if in_platoon else DRIVING_STATE_AUTO
        s.platoon_state = _STATE_TO_WIRE.get(self.fsm.state.name, PLATOON_STATE_SOLO)
        s.platoon_role = (
            PLATOON_ROLE_LEADER if self.fsm.is_designated_leader
            else (PLATOON_ROLE_FOLLOWER if in_platoon else PLATOON_ROLE_NONE)
        )
        s.leader_vehicle_id = self.fsm.leader_id or 0
        s.front_vehicle_id = self.fsm.partner_id or 0
        s.emergency = 1 if self._last_emergency else 0
        s.speed_level = self._own_speed_level
        return s

    def control_loop(self):
        cmd: DrivingCommand = self.fsm.update(self.ego_state, self.nearby_vehicles)
        self._last_emergency = cmd.emergency

        state_msg = String()
        state_msg.data = (
            f"State: {self.fsm.state.name} | Mode: {cmd.mode} | "
            f"speed_level: {cmd.speed_level} | lane: {self.ego_state.lane}"
        )
        self.pub_fsm_state.publish(state_msg)

        self.pub_self_status.publish(self._build_self_status())

        if cmd.request_lane_change_dir is not None:
            self._request_lane_change(cmd.request_lane_change_dir)

        self._apply_speed_level(cmd.speed_level)


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
