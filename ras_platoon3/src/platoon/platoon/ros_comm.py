"""ROS 2 topic-based PlatoonComm — 2차량 LAN 데모용.

ESP32가 JOIN 핸드셰이크 패킷을 릴레이하지 않는 구조에서,
v2x_protocol.py의 dataclass 패킷을 ROS 토픽(/v2x/handshake, std_msgs/String,
JSON 페이로드)으로 왕복시켜 두 차량이 서로 JOIN/MAINTAIN 핸드셰이크를
완수할 수 있게 한다.

사용법:
    from .ros_comm import RosPlatoonComm
    ...
    self.fsm = PlatoonFSM(
        vehicle_id=..., is_designated_leader=...,
        comm=RosPlatoonComm(self),   # self = rclpy Node
    )

토픽: /v2x/handshake  (std_msgs/String, JSON)
QoS:   reliable (핸드셰이크는 손실되면 안 됨)
"""
from __future__ import annotations

import dataclasses
import json
import threading
from typing import Any, Dict, List, Optional

from rclpy.node import Node
from std_msgs.msg import String

from .v2x_protocol import (
    PlatoonComm,
    PlatoonJoinRequest,
    PlatoonJoinAccept,
    PlatoonJoinReject,
    PlatoonSetup,
    MergeCommand,
    MergeStatus,
    JoinComplete,
    PlatoonControl,
    EmergencyMessage,
    LeaveMessage,
)

# msg_type 문자열 → dataclass 매핑 (v2x_protocol.py의 클래스 레벨 default 사용)
_MSG_TYPE_TO_CLASS = {
    "JOIN_REQUEST":    PlatoonJoinRequest,
    "JOIN_ACCEPT":     PlatoonJoinAccept,
    "JOIN_REJECT":     PlatoonJoinReject,
    "PLATOON_SETUP":   PlatoonSetup,
    "MERGE_COMMAND":   MergeCommand,
    "MERGE_STATUS":    MergeStatus,
    "JOIN_COMPLETE":   JoinComplete,
    "PLATOON_CONTROL": PlatoonControl,
    "EMERGENCY":       EmergencyMessage,
    "LEAVE":           LeaveMessage,
}

_DEFAULT_TOPIC = "/v2x/handshake"


def packet_to_json(packet: Any) -> str:
    """dataclass 패킷 → JSON 문자열 (msg_type 필드 포함)."""
    d = dataclasses.asdict(packet)
    # dataclass 클래스 레벨 default가 asdict에 반영되므로 msg_type이 이미 있음.
    # 없으면(예: 상속) type 기반 fallback.
    d.setdefault("msg_type", getattr(type(packet), "msg_type", type(packet).__name__))
    return json.dumps(d, ensure_ascii=False)


def json_to_packet(s: str) -> Optional[Any]:
    """JSON 문자열 → dataclass 패킷 (알 수 없는 msg_type이면 None)."""
    try:
        d = json.loads(s)
    except (json.JSONDecodeError, TypeError):
        return None
    cls = _MSG_TYPE_TO_CLASS.get(d.get("msg_type"))
    if cls is None:
        return None
    try:
        fields = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        return cls(**fields)
    except (TypeError, ValueError):
        return None


class RosPlatoonComm(PlatoonComm):
    """ROS 토픽을 매개로 하는 PlatoonComm 구현체.

    - send(packet):  dataclass → JSON → /v2x/handshake 발행
    - poll():        수신 큐를 비워 dataclass 리스트로 반환

    스레드 안전: rclpy subscription 콜백은 executor 스레드,
    FSM update()는 main 스레드. threading.Lock으로 보호.
    """

    def __init__(self, node: Node, topic: str = _DEFAULT_TOPIC):
        self._node = node
        self._topic = topic
        self._pub = node.create_publisher(String, topic, 10)
        self._rx: List[Any] = []
        self._lock = threading.Lock()
        self._sub = node.create_subscription(String, topic, self._on_rx, 10)
        node.get_logger().info(
            f"[RosPlatoonComm] handshake 브리지 시작 (토픽: {topic})"
        )

    # ── 수신 ──────────────────────────────────────────────────────
    def _on_rx(self, msg: String) -> None:
        packet = json_to_packet(msg.data)
        if packet is None:
            self._node.get_logger().debug(
                f"[RosPlatoonComm] 무시: {msg.data[:120]!r}"
            )
            return
        with self._lock:
            self._rx.append(packet)

    # ── PlatoonComm 인터페이스 ─────────────────────────────────────
    def send(self, packet: Any) -> None:
        try:
            payload = packet_to_json(packet)
        except Exception as e:
            self._node.get_logger().error(
                f"[RosPlatoonComm] serialize 실패: {e}"
            )
            return
        self._pub.publish(String(data=payload))

    def poll(self) -> list:
        with self._lock:
            packets, self._rx = list(self._rx), []
        return packets

    # ── 정리 ──────────────────────────────────────────────────────
    def shutdown(self) -> None:
        try:
            self._pub.destroy_publisher()
        except Exception:
            pass
        try:
            self._sub.destroy_subscription()
        except Exception:
            pass
