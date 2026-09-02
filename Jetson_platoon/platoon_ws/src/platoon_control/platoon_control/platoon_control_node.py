"""
platoon_control_node.py
main.py를 ROS2 노드로 재작성한 버전.

바뀐 것 1: while문 + time.sleep() → create_timer(콜백 방식)
바뀐 것 2: ESP32(V2X) 통신을 v2x_node.py로 분리했다. 이 노드는 이제 ESP32와
          직접 얘기하지 않고 /v2x/esp32_data 구독 + RosPlatoonComm(ros_v2x_comm.py)
          으로만 주변 차량 정보와 패킷을 주고받는다. read_esp32_packet()이 있던
          자리는 이제 _on_esp32_data() 구독 콜백이 최신값을 저장해두는 방식으로
          바뀌었을 뿐, merge_link_into_nearby()/find_partner() 등 나머지 로직은
          전혀 안 바뀌었다 (esp32_data의 dict 구조가 동일하므로).
바뀐 것 3: V2X 토픽이 std_msgs/String(JSON) 대신 platoon_interfaces의 타입 있는
          메시지(Esp32Data 등)를 쓴다. _on_esp32_data()가 msg_to_esp32_data()로
          역직렬화하는 부분만 바뀌었고, 그 이후(merge_link_into_nearby 등)는 동일.
바뀐 것 4: 카메라(라인트레이싱)를 lane_detector_node.py로 분리했다. 이제 이 노드는
          카메라를 직접 안 열고 /lane_info(platoon_interfaces/LaneInfo, 준호님
          레포와 동일 메시지 설계) 구독으로만 차선 정보를 받는다.
          LaneInfo.offset은 픽셀 정수라서 _on_lane_info()에서 정규화(-1~1)로
          변환한다 — lane_image_width 파라미터가 lane_detector_node 쪽 캡처
          해상도와 일치해야 한다(기본 640, 둘 다 CameraCapture 기본값과 같음).
안 바뀐 것: fsm.update(), compute_control() 등 판단/제어 로직은 전부 그대로
           (platoon_fsm.py는 v2x_node.py/lane_detector_node.py의 존재 자체를
           모른다 — PlatoonComm 인터페이스와 LaneTracingResult 모양만 알면
           되므로 이번 노드 분리로 1줄도 안 고쳤다)
           WASD 수동조종(manual_control.py)도 카메라와 무관하게 STM32를 직접
           잡는 경로라 이번 분리로 전혀 안 바뀌었다.

실행:
    ros2 launch platoon_control platoon_control_launch.py is_leader:=true
    (v2x_node, lane_detector_node를 platoon_control_node와 함께 띄워준다)

    개별 실행(디버깅용, 다른 노드들을 따로 띄워야 함):
    ros2 run platoon_control v2x_node
    ros2 run platoon_control lane_detector_node
    ros2 run platoon_control platoon_control_node --ros-args -p mode:=manual

실행 중 Q로 자동/수동 전환 가능 (이전과 동일).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from platoon_interfaces.msg import Esp32Data, LaneInfo

from .platoon_fsm import PlatoonFSM, EgoState, V_MAX
from .driving_control import compute_control, LaneFollower, MAX_STEERING_RAD, LaneTracingResult
from .stm32_interface import STM32Interface
from .manual_control import ManualController
from .ros_v2x_comm import RosPlatoonComm, msg_to_esp32_data


def open_stm32_port():
    """
    STM32 UART 포트를 연다 (젯슨 기준). 프로토콜은 stm32_protocol.md 참고.
    보드/캐리어에 따라 UART 장치명이 다르므로 후보를 순서대로 시도한다.
    """
    import serial
    candidates = ["/dev/ttyTHS1", "/dev/ttyUSB0", "/dev/ttyACM0"]
    for path in candidates:
        try:
            port = serial.Serial(path, 115200, timeout=0)
            print(f"[stm32] {path} 로 연결됨")
            return port
        except Exception:
            continue
    print(f"[warn] STM32 포트를 못 열었습니다 (시도: {candidates}). 송수신 없이 진행합니다.")
    return None


def merge_link_into_nearby(esp32_data: dict, partner_id) -> list:
    nearby = esp32_data["nearby"]
    link_id = esp32_data["link_partner_id"]
    if link_id is None or link_id != partner_id:
        return nearby
    for v in nearby:
        if v.vehicle_id == link_id:
            if esp32_data["link_distance"] is not None:
                v.uwb_distance = esp32_data["link_distance"]
            if esp32_data["link_angle"] is not None:
                v.uwb_angle = esp32_data["link_angle"]
            if esp32_data["link_speed"] is not None:
                v.speed = esp32_data["link_speed"]
            break
    return nearby


def find_partner(nearby: list, partner_id):
    if partner_id is None:
        return None
    for v in nearby:
        if v.vehicle_id == partner_id:
            return v
    return None


class PlatoonControlNode(Node):
    def __init__(self):
        super().__init__("platoon_control_node")

        self.declare_parameter("mode", "auto")
        start_mode = self.get_parameter("mode").get_parameter_value().string_value

        # 실차 시나리오 고정값: 젯슨 차량만 --ros-args -p is_leader:=true 로 띄운다.
        # 라즈베리파이 팔로워 차량들은 기본값(false) 그대로 실행하면 된다.
        self.declare_parameter("is_leader", False)
        is_leader = self.get_parameter("is_leader").get_parameter_value().bool_value

        # comm=RosPlatoonComm(self) — v2x_node.py와 /v2x/outgoing, /v2x/incoming
        # 토픽으로 패킷을 주고받는다. FSM 입장에서는 LoopbackComm(시뮬)이나
        # NullComm(기본값)과 다를 게 없다 — send()/poll() 뒤에 뭐가 있는지 모른다.
        self.fsm = PlatoonFSM(vehicle_id=1, comm=RosPlatoonComm(self),
                              is_designated_leader=is_leader)  # TODO: 실제 차량 고유 ID로 교체
        self.get_logger().info(f"역할 고정: {'리더(젯슨)' if is_leader else '팔로워(라즈베리파이)'}")
        self.ego = EgoState()

        # v2x_node.py가 주기적으로 올려주는 값. read_esp32_packet()이 있던 자리 —
        # 이제 함수 호출이 아니라 최신 수신값을 저장해두는 방식이라 초기값이 필요하다
        # (v2x_node가 아직 첫 메시지를 안 보냈을 때 = 스텁이었을 때와 동일한 빈 상태).
        self._esp32_data = {
            "nearby": [], "link_partner_id": None,
            "link_distance": None, "link_angle": None, "link_speed": None,
        }
        self.create_subscription(Esp32Data, "/v2x/esp32_data", self._on_esp32_data, 10)
        self.stm32 = STM32Interface(open_stm32_port())
        self.lane_follower = LaneFollower()

        # lane_detector_node.py가 올려주는 값. LaneInfo.offset은 픽셀 정수라서
        # 정규화(-1~1)로 변환해서 저장한다 — driving_control.py의 게인(LANE_KP 등)이
        # 정규화 오프셋 기준으로 튜닝돼 있으므로.
        self.declare_parameter("lane_image_width", 640)
        self._lane_image_width = self.get_parameter("lane_image_width").value
        self._lane = LaneTracingResult(offset=0.0, detected=False)
        self.create_subscription(LaneInfo, "/lane_info", self._on_lane_info, 10)

        self.manual = ManualController(max_steering_rad=MAX_STEERING_RAD, max_speed=V_MAX)
        self.manual_mode = (start_mode == "manual")
        if self.manual.available:
            self.get_logger().info(
                f"{'수동(WASD)' if self.manual_mode else '자동'} 모드로 시작. Q로 전환 가능."
            )
        else:
            self.manual_mode = False
            self.get_logger().warn("키보드 리스너를 못 띄워서 자동 주행만 가능합니다.")

        self.loop_dt = 0.02  # 약 50Hz

        # 상태 모니터링용 토픽 (선택사항 — 관제/디버깅에 사용)
        self.status_pub = self.create_publisher(String, "/platoon_control/status", 10)

        self.timer = self.create_timer(self.loop_dt, self._tick)

    def _on_esp32_data(self, msg: Esp32Data) -> None:
        self._esp32_data = msg_to_esp32_data(msg)

    def _on_lane_info(self, msg: LaneInfo) -> None:
        offset_norm = msg.offset / (self._lane_image_width / 2)
        self._lane = LaneTracingResult(offset=offset_norm, detected=bool(msg.lane_detected))

    # ── 매 주기 호출되는 콜백 — main.py의 while문 안 내용과 동일 ──────
    def _tick(self):
        if self.manual.available and self.manual.should_quit:
            self.manual._quit = False
            self.manual_mode = not self.manual_mode
            self.manual.state.speed = 0.0
            self.manual.state.steering = 0.0
            self.get_logger().info(f"{'수동' if self.manual_mode else '자동'} 모드로 전환")

        if self.manual_mode:
            steering, speed = self.manual.update(self.loop_dt)
            self.stm32.send_command(mode="SOLO_DRIVE", target_speed=speed, target_steer=steering)
            self._publish_status(f"MANUAL steer={steering:+.3f} speed={speed:+.2f}")
            return

        esp32_data = self._esp32_data

        feedback = self.stm32.poll()
        if feedback is not None:
            current_speed = feedback.current_speed
            self.ego.obstacle = feedback.obstacle
            self.ego.front_distance = feedback.front_distance
            self.ego.stm32_failsafe = feedback.failsafe
        else:
            current_speed = self.ego.speed

        nearby = merge_link_into_nearby(esp32_data, self.fsm.partner_id)

        lane = self._lane  # lane_detector_node가 /lane_info로 올려준 최신값

        self.ego.speed = current_speed
        self.ego.lane_offset = lane.offset
        self.ego.lane_detected = lane.detected
        # TODO: checkpoint / route / lane / heading 채우기

        cmd = self.fsm.update(self.ego, nearby)
        role = self.fsm.role

        partner = find_partner(nearby, self.fsm.partner_id)

        control = compute_control(
            cmd, role, self.lane_follower,
            lane=lane,
            dt=self.loop_dt,
            uwb_distance=partner.uwb_distance if partner else None,
            uwb_angle=partner.uwb_angle if partner else None,
            current_speed=current_speed,
        )

        lane_stopping = control.lane_lost and not control.uwb_fallback
        stm32_mode = "EMERGENCY_STOP" if (cmd.emergency or lane_stopping) else cmd.mode
        self.stm32.send_command(
            mode=stm32_mode,
            target_speed=control.speed,
            target_steer=control.steering_rad,
        )

        self._publish_status(
            f"AUTO state={self.fsm.state.name} role={role} "
            f"steer={control.steering_rad:+.3f} speed={control.speed:+.2f}"
        )

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def destroy_node(self):
        # Ctrl+C 등으로 종료될 때 키보드 리스너를 깔끔히 정리 (카메라는 lane_detector_node 소관)
        self.manual.stop()
        self.stm32.send_command(mode="EMERGENCY_STOP", target_speed=0.0, target_steer=0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlatoonControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()