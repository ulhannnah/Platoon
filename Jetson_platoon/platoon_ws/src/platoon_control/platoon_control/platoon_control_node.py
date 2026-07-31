"""
platoon_control_node.py
main.py를 ROS2 노드로 재작성한 버전.

바뀐 것: while문 + time.sleep() → create_timer(콜백 방식)
안 바뀐 것: fsm.update(), compute_control() 등 판단/제어 로직은 전부 그대로
           (platoon_fsm.py, driving_control.py 등 원본 그대로 가져다 씀)

지금은 노드 1개로 시작합니다 (라인트레이싱/ESP32가 아직 이 노드 안에 통합되어
있어서). 나중에 라인트레이싱이 정말 별도 프로세스/노드로 분리되면, 그때는
perception.get_lane() 호출을 /lane_tracing 토픽 구독으로 바꾸면 됩니다 —
fsm.update() 이후 로직은 안 바뀝니다.

실행:
    ros2 run platoon_control platoon_control_node --ros-args -p mode:=manual
    ros2 run platoon_control platoon_control_node --ros-args -p mode:=auto
    (mode 파라미터 생략 시 auto)

실행 중 Q로 자동/수동 전환 가능 (이전과 동일).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .platoon_fsm import PlatoonFSM, EgoState, V_MAX
from .driving_control import compute_control, LaneFollower, MAX_STEERING_RAD
from .stm32_interface import STM32Interface
from .manual_control import ManualController
from .lane_tracing import PerceptionTracker


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


def read_esp32_packet():
    """
    ESP32-S3가 올려주는 값. TODO: RPi/Jetson<->ESP32 시리얼 프로토콜 확정되면
    실제 구독/파싱으로 교체. 지금은 main.py 때와 동일한 스텁.
    """
    return {
        "nearby": [],
        "link_partner_id": None,
        "link_distance": None,
        "link_angle": None,
        "link_speed": None,
    }


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

        self.fsm = PlatoonFSM(vehicle_id=1)  # TODO: 실제 차량 고유 ID로 교체
        self.ego = EgoState()
        self.stm32 = STM32Interface(open_stm32_port())
        self.lane_follower = LaneFollower()
        self.perception = PerceptionTracker()
        if not self.perception.available:
            self.get_logger().warn("카메라를 못 열어서 라인트레이싱 없이 진행합니다.")

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

        esp32_data = read_esp32_packet()

        feedback = self.stm32.poll()
        if feedback is not None:
            current_speed = feedback.current_speed
            self.ego.obstacle = feedback.obstacle
            self.ego.front_distance = feedback.front_distance
            self.ego.stm32_failsafe = feedback.failsafe
        else:
            current_speed = self.ego.speed

        nearby = merge_link_into_nearby(esp32_data, self.fsm.partner_id)

        lane = self.perception.get_lane()  # 논블로킹

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
        # Ctrl+C 등으로 종료될 때 카메라/키보드 리스너를 깔끔히 정리
        self.perception.stop()
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
