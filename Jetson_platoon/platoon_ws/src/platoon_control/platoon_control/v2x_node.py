"""
v2x_node.py
V2X(ESP32-S3) 통신 전용 ROS2 노드.

FSM 노드(platoon_control_node)에서 분리했다 — 이제 ESP32 실제 시리얼 프로토콜이
확정되면 이 파일만 고치면 되고, platoon_fsm.py와 platoon_control_node.py는
그대로 둔다. 이 노드가 하는 일은 platoon_control_node.py에 있던
read_esp32_packet() 스텁을 그대로 옮겨와서 토픽으로 내보내는 것뿐이다.

토픽: ros_v2x_comm.py 참고.

실행:
    ros2 run platoon_control v2x_node
    (보통은 platoon_control_launch.py가 platoon_control_node와 함께 띄운다)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .ros_v2x_comm import encode_esp32_data, decode_packet


def read_esp32_packet():
    """
    ESP32-S3가 올려주는 값. TODO: RPi/Jetson<->ESP32 시리얼 프로토콜 확정되면
    실제 구독/파싱으로 교체. platoon_control_node.py에 있던 것과 동일한 스텁을
    그대로 옮겨왔다.

    ★ 프로토콜 확정 시 nearby 항목에 상대 차량의 platoon_state / platoon_id /
      leader_id 를 반드시 포함해야 한다 (NearbyVehicle의 동명 필드).
      3대 이상 체인에서 신규 차량은 자기가 붙는 맨 뒤 차량이 아는 "진짜 리더 ID와
      플래툰 ID"를 이 필드로 물려받는다(platoon_fsm._setup_platoon). 비어 있으면
      3번째 차량이 2번째 차량을 리더로 착각하고 별도 플래툰을 새로 만든다
      — tools/sim_two_vehicles.py 시나리오 B-2가 이 경우를 검증한다.
    """
    return {
        "nearby": [],
        "link_partner_id": None,
        "link_distance": None,
        "link_angle": None,
        "link_speed": None,
    }


class V2XNode(Node):
    def __init__(self):
        super().__init__("v2x_node")

        self.declare_parameter("publish_rate_hz", 50.0)
        rate = self.get_parameter("publish_rate_hz").value

        self._data_pub = self.create_publisher(String, "/v2x/esp32_data", 10)
        self._incoming_pub = self.create_publisher(String, "/v2x/incoming", 10)
        self.create_subscription(String, "/v2x/outgoing", self._on_outgoing, 10)

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("V2X 노드 시작 (ESP32 미연결 — 스텁으로 진행)")

    def _tick(self):
        esp32_data = read_esp32_packet()
        msg = String()
        msg.data = encode_esp32_data(esp32_data)
        self._data_pub.publish(msg)

    def _on_outgoing(self, msg: String) -> None:
        """
        이 차량이 보낸 패킷 — 실제로는 ESP-NOW로 다른 차량에 중계해야 한다.

        TODO: ESP32 시리얼이 붙으면 여기서 JSON을 바이트 프레임으로 바꿔 송신하고,
              ESP32가 수신한 패킷은 별도 콜백에서 파싱해 /v2x/incoming으로 발행한다.
              지금은 시리얼이 없어 로그만 남긴다 — 실차 2대 이상 동시 테스트 전까지는
              tools/sim_two_vehicles.py의 LoopbackBus가 이 역할을 대신한다.
        """
        packet = decode_packet(msg.data)
        self.get_logger().debug(f"[v2x] 송신 요청(중계 미구현): {packet}")


def main(args=None):
    rclpy.init(args=args)
    node = V2XNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
