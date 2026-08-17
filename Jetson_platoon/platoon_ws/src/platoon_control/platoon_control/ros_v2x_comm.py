"""
ros_v2x_comm.py
V2X 노드와 FSM 노드를 ROS2 토픽으로 잇는 어댑터.

platoon_fsm.py는 이 파일의 존재를 전혀 모른다 — PlatoonComm 인터페이스(send/poll)만
알면 되므로 노드를 쪼개도 FSM 로직은 손대지 않는다 (v2x_protocol.py의 NullComm,
LoopbackComm과 같은 자리에 끼워 넣는 구현체 하나가 늘어난 것뿐).

패킷(JOIN_REQUEST 등)은 dataclass 그대로 JSON으로 직렬화해서 std_msgs/String에
싣는다. ESP32 프로토콜(바이트 포맷)이 아직 미정이라 커스텀 .msg 타입을 여러 개
만들어봤자 프로토콜이 바뀌면 다시 만들어야 한다 — 그래서 지금은 이 JSON 봉투가
"V2X 노드 ↔ FSM 노드" 사이의 임시 계약이다. 나중에 실제 ESP32 시리얼이 붙으면
v2x_node.py 안에서 JSON ↔ 실제 바이트 변환만 추가하면 되고, 이 파일과
platoon_fsm.py는 그대로 둔다.

토픽 (한 차량 내부에서만 쓰는 로컬 통신 — 다른 차량과의 무선 통신은 V2X 노드가
ESP-NOW로 담당):
    /v2x/outgoing    FSM → V2X : 내가 보낼 패킷 (JOIN_REQUEST 등)
    /v2x/incoming    V2X → FSM : 다른 차량에게서 받은 패킷
    /v2x/esp32_data  V2X → FSM : 주변 차량 목록 + UWB 링크 정보 (read_esp32_packet() 반환값과 동일 구조)
"""

import json
import queue
from dataclasses import asdict

from std_msgs.msg import String

from . import v2x_protocol as v2x
from .v2x_protocol import PlatoonComm
from .platoon_fsm import NearbyVehicle

# msg_type 문자열 -> dataclass 매핑. v2x_protocol.py에 새 패킷을 추가하면 여기도 추가해야 한다.
_PACKET_TYPES = {
    cls().msg_type: cls
    for cls in (
        v2x.PlatoonJoinRequest, v2x.PlatoonJoinAccept, v2x.PlatoonJoinReject,
        v2x.PlatoonSetup, v2x.MergeCommand, v2x.MergeStatus, v2x.JoinComplete,
        v2x.PlatoonControl, v2x.EmergencyMessage, v2x.LeaveMessage,
    )
}


def encode_packet(packet) -> str:
    return json.dumps(asdict(packet))


def decode_packet(text: str):
    """알 수 없는 msg_type이면 None — 호출부는 조용히 무시한다 (프로토콜 버전 불일치 대비)."""
    data = json.loads(text)
    cls = _PACKET_TYPES.get(data.get("msg_type"))
    if cls is None:
        return None
    return cls(**data)


def encode_esp32_data(data: dict) -> str:
    """read_esp32_packet()의 반환값(dict, nearby는 NearbyVehicle 리스트)을 그대로 직렬화."""
    d = dict(data)
    d["nearby"] = [asdict(v) for v in d["nearby"]]
    return json.dumps(d)


def decode_esp32_data(text: str) -> dict:
    d = json.loads(text)
    d["nearby"] = [NearbyVehicle(**v) for v in d["nearby"]]
    return d


class RosPlatoonComm(PlatoonComm):
    """
    PlatoonFSM(comm=...)에 주입하는 어댑터. FSM 입장에서는 LoopbackComm과
    구분되지 않는다 — send()/poll() 뒤에서 ROS2 토픽을 쓸 뿐이다.

    poll()이 논블로킹이어야 하므로(FSM.update()가 매 주기 즉시 반환해야 함),
    구독 콜백은 큐에 쌓기만 하고 poll()이 그걸 비운다.
    """

    def __init__(self, node, outgoing_topic="/v2x/outgoing", incoming_topic="/v2x/incoming"):
        self._pub = node.create_publisher(String, outgoing_topic, 10)
        self._queue: "queue.Queue" = queue.Queue()
        node.create_subscription(String, incoming_topic, self._on_incoming, 10)

    def _on_incoming(self, msg: String) -> None:
        packet = decode_packet(msg.data)
        if packet is not None:
            self._queue.put(packet)

    def send(self, packet) -> None:
        out = String()
        out.data = encode_packet(packet)
        self._pub.publish(out)

    def poll(self) -> list:
        packets = []
        while True:
            try:
                packets.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return packets
