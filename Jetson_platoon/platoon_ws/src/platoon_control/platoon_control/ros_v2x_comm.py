"""
ros_v2x_comm.py
V2X 노드와 FSM 노드를 ROS2 토픽으로 잇는 어댑터.

platoon_fsm.py는 이 파일의 존재를 전혀 모른다 — PlatoonComm 인터페이스(send/poll)만
알면 되므로 노드를 쪼개도 FSM 로직은 손대지 않는다 (platoon_fsm.py의 NullComm,
LoopbackComm과 같은 자리에 끼워 넣는 구현체 하나가 늘어난 것뿐).

패킷 클래스(PlatoonJoinRequest 등)는 platoon_fsm.py 안에 정의돼 있다 (원래
v2x_protocol.py였는데, platoon_fsm.py를 다른 ROS2 노드에서 독립 파일로 그대로
가져다 쓸 수 있어야 해서 그 파일로 합쳤다). 여기서 새로 만들면 안 된다 —
이름이 같아도 다른 클래스가 되어 isinstance() 비교가 깨진다.

타입 있는 메시지(platoon_interfaces 패키지)를 쓴다. 예전엔 dataclass를 JSON으로
직렬화해서 std_msgs/String에 실었는데, `ros2 topic echo`로 안 읽히고 타입 안전성도
없어서, platoon_interfaces/msg/*.msg 로 바꿨다 — 필드 이름을 platoon_fsm.py의
dataclass와 1:1로 맞춰서 이름 매핑 없이 그대로 복사/복원한다 (아래 packet_to_msg/
msg_to_packet 참고).

ROS2 메시지는 Optional(None)을 표현 못 하므로 아래 규칙으로 센티널을 쓴다:
    Optional[int]   없으면 -1
    Optional[float] 없으면 NaN

토픽 (한 차량 내부에서만 쓰는 로컬 통신 — 다른 차량과의 무선 통신은 V2X 노드가
ESP-NOW로 담당):
    /v2x/outgoing    FSM → V2X : 내가 보낼 패킷 (JOIN_REQUEST 등), 타입 V2xPacket
    /v2x/incoming    V2X → FSM : 다른 차량에게서 받은 패킷, 타입 V2xPacket
    /v2x/esp32_data  V2X → FSM : 주변 차량 목록 + UWB 링크 정보, 타입 Esp32Data
                     (read_esp32_packet() 반환값과 동일 구조)
"""

import math
import queue
from dataclasses import fields as dataclass_fields
from typing import Optional

from platoon_interfaces.msg import V2xPacket, NearbyVehicleMsg, Esp32Data

from . import platoon_fsm as fsm
from .platoon_fsm import PlatoonComm, NearbyVehicle

# msg_type 문자열 -> dataclass 매핑. platoon_fsm.py에 새 패킷을 추가하면 여기와
# V2xPacket.msg 양쪽에 필드를 추가해야 한다.
_PACKET_TYPES = {
    cls().msg_type: cls
    for cls in (
        fsm.PlatoonJoinRequest, fsm.PlatoonJoinAccept, fsm.PlatoonJoinReject,
        fsm.PlatoonSetup, fsm.MergeCommand, fsm.MergeStatus, fsm.JoinComplete,
        fsm.PlatoonControl, fsm.EmergencyMessage, fsm.LeaveMessage,
    )
}


# ── Optional 값 <-> ROS2 센티널 변환 ────────────────────────────────
def _opt_int_to_wire(v: Optional[int]) -> int:
    return -1 if v is None else int(v)


def _wire_to_opt_int(v: int) -> Optional[int]:
    return None if v == -1 else int(v)


def _opt_float_to_wire(v: Optional[float]) -> float:
    return float("nan") if v is None else float(v)


def _wire_to_opt_float(v: float) -> Optional[float]:
    return None if math.isnan(v) else float(v)


# ── V2X 패킷(10종 dataclass) <-> V2xPacket 메시지 ───────────────────
def packet_to_msg(packet) -> V2xPacket:
    """
    dataclasses.fields()로 그 패킷이 실제로 가진 필드만 훑어서 복사한다.
    V2xPacket.msg가 10종 전체 필드의 합집합이라, 어떤 패킷을 넣어도 이름이
    같은 필드에만 값이 채워지고 나머지는 메시지 기본값(0/""/false)으로 남는다.
    """
    msg = V2xPacket()
    for f in dataclass_fields(packet):
        setattr(msg, f.name, getattr(packet, f.name))
    return msg


def msg_to_packet(msg: V2xPacket):
    """알 수 없는 msg_type이면 None — 호출부는 조용히 무시한다 (프로토콜 버전 불일치 대비)."""
    cls = _PACKET_TYPES.get(msg.msg_type)
    if cls is None:
        return None
    kwargs = {f.name: getattr(msg, f.name) for f in dataclass_fields(cls)}
    return cls(**kwargs)


# ── NearbyVehicle <-> NearbyVehicleMsg ──────────────────────────────
def nearby_to_msg(v: NearbyVehicle) -> NearbyVehicleMsg:
    msg = NearbyVehicleMsg()
    msg.vehicle_id = v.vehicle_id
    msg.checkpoint = v.checkpoint
    msg.next_checkpoint = v.next_checkpoint
    msg.route = list(v.route)
    msg.destination = v.destination
    msg.lane = v.lane
    msg.speed = v.speed
    msg.accel = v.accel
    msg.heading = v.heading
    msg.platoon_allow = v.platoon_allow
    msg.platoon_state = v.platoon_state
    msg.platoon_id = _opt_int_to_wire(v.platoon_id)
    msg.leader_id = _opt_int_to_wire(v.leader_id)
    msg.pdr = v.pdr
    msg.rssi = v.rssi
    msg.emergency = v.emergency
    msg.uwb_distance = _opt_float_to_wire(v.uwb_distance)
    msg.uwb_angle = _opt_float_to_wire(v.uwb_angle)
    msg.timestamp = v.timestamp
    return msg


def msg_to_nearby(msg: NearbyVehicleMsg) -> NearbyVehicle:
    return NearbyVehicle(
        vehicle_id=msg.vehicle_id,
        checkpoint=msg.checkpoint,
        next_checkpoint=msg.next_checkpoint,
        route=list(msg.route),
        destination=msg.destination,
        lane=msg.lane,
        speed=msg.speed,
        accel=msg.accel,
        heading=msg.heading,
        platoon_allow=msg.platoon_allow,
        platoon_state=msg.platoon_state,
        platoon_id=_wire_to_opt_int(msg.platoon_id),
        leader_id=_wire_to_opt_int(msg.leader_id),
        pdr=msg.pdr,
        rssi=msg.rssi,
        emergency=msg.emergency,
        uwb_distance=_wire_to_opt_float(msg.uwb_distance),
        uwb_angle=_wire_to_opt_float(msg.uwb_angle),
        timestamp=msg.timestamp,
    )


# ── esp32_data(dict) <-> Esp32Data 메시지 ───────────────────────────
def esp32_data_to_msg(data: dict) -> Esp32Data:
    """read_esp32_packet()의 반환값(dict, nearby는 NearbyVehicle 리스트)을 그대로 직렬화."""
    msg = Esp32Data()
    msg.nearby = [nearby_to_msg(v) for v in data["nearby"]]
    msg.link_partner_id = _opt_int_to_wire(data["link_partner_id"])
    msg.link_distance = _opt_float_to_wire(data["link_distance"])
    msg.link_angle = _opt_float_to_wire(data["link_angle"])
    msg.link_speed = _opt_float_to_wire(data["link_speed"])
    return msg


def msg_to_esp32_data(msg: Esp32Data) -> dict:
    return {
        "nearby": [msg_to_nearby(v) for v in msg.nearby],
        "link_partner_id": _wire_to_opt_int(msg.link_partner_id),
        "link_distance": _wire_to_opt_float(msg.link_distance),
        "link_angle": _wire_to_opt_float(msg.link_angle),
        "link_speed": _wire_to_opt_float(msg.link_speed),
    }


class RosPlatoonComm(PlatoonComm):
    """
    PlatoonFSM(comm=...)에 주입하는 어댑터. FSM 입장에서는 LoopbackComm과
    구분되지 않는다 — send()/poll() 뒤에서 ROS2 토픽을 쓸 뿐이다.

    poll()이 논블로킹이어야 하므로(FSM.update()가 매 주기 즉시 반환해야 함),
    구독 콜백은 큐에 쌓기만 하고 poll()이 그걸 비운다.
    """

    def __init__(self, node, outgoing_topic="/v2x/outgoing", incoming_topic="/v2x/incoming"):
        self._pub = node.create_publisher(V2xPacket, outgoing_topic, 10)
        self._queue: "queue.Queue" = queue.Queue()
        node.create_subscription(V2xPacket, incoming_topic, self._on_incoming, 10)

    def _on_incoming(self, msg: V2xPacket) -> None:
        packet = msg_to_packet(msg)
        if packet is not None:
            self._queue.put(packet)

    def send(self, packet) -> None:
        self._pub.publish(packet_to_msg(packet))

    def poll(self) -> list:
        packets = []
        while True:
            try:
                packets.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return packets
