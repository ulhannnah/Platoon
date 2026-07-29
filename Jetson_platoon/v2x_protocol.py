"""
v2x_protocol.py
플래툰 결합 통신 절차(§10)에서 주고받는 메시지(패킷) 정의.

기준 문서: 260714_platoon_algorithm_design.md
- §9.2  : 플래툰 전용 통신에서 주고받는 데이터 항목
- §10   : 플래툰 결합 통신 절차 (요청→응답→설정→병합→완료→주기교환, 7단계)
- §10.1 : JOIN_REQUEST C 구조체 (문서에 이미 정의돼 있던 유일한 구조체 — 필드 순서 그대로 이식)
- §15.4 : "통신 패킷 구체화"가 미완료 항목으로 명시돼 있었음
          → JOIN_ACCEPT/REJECT, PLATOON_SETUP, MERGE_COMMAND/STATUS,
            JOIN_COMPLETE, PLATOON_CONTROL, LEAVE는 이번에 새로 정의함

여기 정의된 건 RPi 프로세스 내부(FSM ↔ 통신 계층)에서 쓰는 파이썬 표현입니다.
ESP32와 실제로 주고받을 바이트 포맷(struct.pack 등)은 RPi<->ESP32 시리얼
프로토콜이 확정되면 별도로 정합니다. ESP-NOW 페이로드 한도(약 250byte)를
넘지 않도록 나중에 필드를 다시 점검하세요.
"""

from dataclasses import dataclass


@dataclass
class PlatoonJoinRequest:
    """§10.1 — 요청 차량이 상대 차량에게 결합을 요청"""
    msg_type: str = "JOIN_REQUEST"
    requester_id: int = 0
    target_id: int = 0
    suitability: int = 0          # 적합도 0~100 (문서 원본: uint8_t suitability)
    checkpoint_id: int = 0
    destination_id: int = 0
    speed: float = 0.0
    distance: float = 0.0
    timestamp: float = 0.0


@dataclass
class PlatoonJoinAccept:
    """§10.2 — 상대 차량이 재검사 후 승인"""
    msg_type: str = "JOIN_ACCEPT"
    responder_id: int = 0
    requester_id: int = 0
    suitability: int = 0          # 상대 차량이 자체 계산한 적합도 (0~100)
    timestamp: float = 0.0


@dataclass
class PlatoonJoinReject:
    """§10.2 — 상대 차량이 재검사 후 거절"""
    msg_type: str = "JOIN_REJECT"
    responder_id: int = 0
    requester_id: int = 0
    reason_code: int = 0          # TODO: 사유 코드 체계 확정 (조건 미달/이미 결합중 등)
    timestamp: float = 0.0


@dataclass
class PlatoonSetup:
    """§10.3 — 승인 후 플래툰 ID와 역할 공유"""
    msg_type: str = "PLATOON_SETUP"
    sender_id: int = 0
    platoon_id: int = 0            # Leader ID + Platoon Sequence 조합 (§10.3)
    leader_id: int = 0
    follower_id: int = 0
    timestamp: float = 0.0


@dataclass
class MergeCommand:
    """§11.2 — 리더가 팔로워에게 목표 속도·거리 전달"""
    msg_type: str = "MERGE_COMMAND"
    leader_id: int = 0
    follower_id: int = 0
    target_speed: float = 0.0
    target_distance: float = 0.0
    accel_request: str = "HOLD"    # ACCELERATE / HOLD / DECELERATE
    timestamp: float = 0.0


@dataclass
class MergeStatus:
    """§10 5단계 — 팔로워가 현재 거리와 결합 진행 상태 보고"""
    msg_type: str = "MERGE_STATUS"
    follower_id: int = 0
    leader_id: int = 0
    current_distance: float = 0.0
    current_speed: float = 0.0
    progress_state: str = "JOIN_LANE_ALIGN"   # JoinSubState 이름 재사용
    timestamp: float = 0.0


@dataclass
class JoinComplete:
    """§11.3 — 결합 완료 조건(거리·속도오차, 1초 유지) 충족 시 양쪽이 교환"""
    msg_type: str = "JOIN_COMPLETE"
    sender_id: int = 0
    platoon_id: int = 0
    timestamp: float = 0.0


@dataclass
class PlatoonControl:
    """§9.2 — PLATOON_MAINTAIN 중 주기적으로 교환하는 전용 통신 패킷"""
    msg_type: str = "PLATOON_CONTROL"
    platoon_id: int = 0
    leader_id: int = 0
    follower_id: int = 0
    current_speed: float = 0.0
    target_speed: float = 0.0
    accel_state: str = "HOLD"
    current_distance: float = 0.0
    target_distance: float = 0.0
    progress_state: str = "PLATOON_MAINTAIN"
    emergency: bool = False
    seq_num: int = 0
    timestamp: float = 0.0


@dataclass
class EmergencyMessage:
    """플래툰 내 긴급 상황 즉시 전파 — 급정지·장애물·강제 해제"""
    msg_type: str = "EMERGENCY"
    sender_id: int = 0
    platoon_id: int = 0
    emergency_type: str = "STOP"   # STOP / OBSTACLE / FORCE_RELEASE
    timestamp: float = 0.0


@dataclass
class LeaveMessage:
    """PLATOON_EXIT 시작 시 플래툰 해제를 통지"""
    msg_type: str = "LEAVE"
    sender_id: int = 0
    platoon_id: int = 0
    timestamp: float = 0.0


# ── 통신 계층 인터페이스 — FSM은 이 인터페이스만 알면 됨 ───────────
class PlatoonComm:
    """
    ESP32-S3로 실제 패킷을 보내고 받는 인터페이스.
    TODO: RPi<->ESP32 시리얼 프로토콜이 정해지면 진짜 구현(예: SerialPlatoonComm)으로 교체.
    """
    def send(self, packet) -> None:
        raise NotImplementedError

    def poll(self) -> list:
        """새로 수신된 패킷 리스트를 반환한다 (없으면 빈 리스트)."""
        raise NotImplementedError


class NullComm(PlatoonComm):
    """실제 통신이 붙기 전 기본값 — 아무것도 보내지 않고 항상 빈 리스트만 반환."""
    def send(self, packet) -> None:
        pass

    def poll(self) -> list:
        return []