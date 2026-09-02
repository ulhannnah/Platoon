"""
platoon_fsm.py
플래툰(군집주행) 알고리즘 FSM 골격 코드 — Raspberry Pi 5 측에서 실행

기준 문서: 260714_platoon_algorithm_design.md
- §5   : 주변 차량 1차 선별
- §6   : 플래툰 절대조건 (C_join)
- §7   : 플래툰 적합도 계산 (S_platoon)
- §8   : 최종 결합 판단 (T_join, 5회 중 4회 안정성)
- §12  : 매칭 파이프라인 상태(V2X_SEARCH → EVALUATE → JOIN_REQUEST → PLATOON_SETUP)
- §17  : 결합 이후 최상위 상태 (SOLO_DRIVE → PLATOON_JOIN → PLATOON_MAINTAIN → PLATOON_EXIT)
- §19.1: PLATOON_JOIN 내부 세부 단계

§5~§8(1차 선별/절대조건/적합도/안정성 판단)은 이번에 실제 로직으로 채웠습니다.
그 외(통신, JOIN 이후 단계 등)는 아직 뼈대만 있는 상태이며, 각 TODO는 설계 문서에
정의됐지만 구체값이 아직 안 정해진 부분(§15.5 "파라미터 실험 및 튜닝" 대상)입니다.

입력  : EgoState(자차 주행정보, RPi 자체 주행 알고리즘에서),
        list[NearbyVehicle](ESP32-S3가 올려주는 V2X+UWB 데이터)
출력  : DrivingCommand → 주행 알고리즘(→ STM32)으로 전달

역할 경계:
- UWB 거리(m)·각도(rad)는 ESP32-S3가 연산을 끝낸 값을 그대로 소비한다.
  (원시 UWB 신호 처리, ESP-NOW 송수신은 ESP32 담당)
- 이 파일은 "판단"만 한다. 조향각·PWM·모터 PID는 일절 다루지 않으며,
  목표 차선/목표 속도/목표 차간거리 같은 목표값만 DrivingCommand로 내보낸다.

독립 파일 정책: 이 파일은 표준 라이브러리 외에 아무것도 import하지 않는다
(v2x_protocol.py를 여기로 합침). ROS2 노드 여러 개에서 각자 이 파일 하나만
떼어다 쓸 수 있어야 하기 때문 — 상대 임포트(from . import ...)가 하나라도
있으면 패키지 밖에서 단독으로 못 쓴다. 패킷 구조체가 여러 곳에서 각자
정의되면 isinstance() 비교가 깨지므로(이름은 같아도 다른 클래스가 됨),
반드시 이 파일의 정의를 그대로 가져다 써야 한다 — 복사해서 새로 만들지 말 것.
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


# ══════════════════════════════════════════════════════════════════
# V2X 패킷 정의 + 통신 계층 인터페이스 (원래 v2x_protocol.py, 이 파일로 합침)
#
# 기준 문서: 260714_platoon_algorithm_design.md
# - §9.2  : 플래툰 전용 통신에서 주고받는 데이터 항목
# - §10   : 플래툰 결합 통신 절차 (요청→응답→설정→병합→완료→주기교환, 7단계)
# - §10.1 : JOIN_REQUEST C 구조체 (문서에 이미 정의돼 있던 유일한 구조체)
# - §15.4 : "통신 패킷 구체화"가 미완료 항목으로 명시돼 있었음
#           → JOIN_ACCEPT/REJECT, PLATOON_SETUP, MERGE_COMMAND/STATUS,
#             JOIN_COMPLETE, PLATOON_CONTROL, LEAVE는 새로 정의함
#
# ESP32와 실제로 주고받을 바이트 포맷(struct.pack 등)은 시리얼 프로토콜이
# 확정되면 별도로 정한다. ESP-NOW 페이로드 한도(약 250byte)를 넘지 않도록
# 나중에 필드를 다시 점검할 것.
# ══════════════════════════════════════════════════════════════════

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


class LoopbackBus:
    """
    메모리상의 가짜 V2X 버스. 한 차량이 보낸 패킷을 나머지 모든 차량이 받는다
    (ESP-NOW 브로드캐스트와 같은 동작). 실제 통신 코드가 붙기 전에
    결합 로직을 검증하는 용도 (tools/sim_two_vehicles.py에서 사용).
    """

    def __init__(self):
        self._ports: dict = {}   # vehicle_id -> 수신 대기 큐

    def create_port(self, vehicle_id: int) -> "LoopbackComm":
        self._ports.setdefault(vehicle_id, [])
        return LoopbackComm(self, vehicle_id)

    def broadcast(self, sender_id: int, packet) -> None:
        for vid, queue in self._ports.items():
            if vid != sender_id:
                queue.append(packet)

    def receive(self, vehicle_id: int) -> list:
        queue = self._ports.get(vehicle_id, [])
        packets, queue[:] = list(queue), []
        return packets


class LoopbackComm(PlatoonComm):
    """LoopbackBus에 연결된 한 대의 통신 포트."""

    def __init__(self, bus: LoopbackBus, vehicle_id: int):
        self.bus = bus
        self.vehicle_id = vehicle_id

    def send(self, packet) -> None:
        self.bus.broadcast(self.vehicle_id, packet)

    def poll(self) -> list:
        return self.bus.receive(self.vehicle_id)


# ── 최상위 플래툰 상태 (§17) ──────────────────────────────────────
class PlatoonState(Enum):
    SOLO_DRIVE = auto()
    PLATOON_JOIN = auto()
    PLATOON_MAINTAIN = auto()
    PLATOON_EXIT = auto()


# ── SOLO_DRIVE 동안의 매칭 파이프라인 서브상태 (§12) ───────────────
class MatchState(Enum):
    IDLE = auto()
    V2X_SEARCH = auto()
    EVALUATE = auto()
    JOIN_REQUEST = auto()    # 요청 보내고 응답 대기 (요청 측)
    AWAIT_SETUP = auto()     # 승인 보내고 PLATOON_SETUP 대기 (수신 측)
    PLATOON_SETUP = auto()


# ── PLATOON_JOIN 내부 세부 단계 (§19.1) ────────────────────────────
class JoinSubState(Enum):
    JOIN_REQUESTED = auto()
    JOIN_LANE_ALIGN = auto()
    JOIN_GAP_CONTROL = auto()
    JOIN_STABILIZE = auto()


# ── 적합도 계산 파라미터 (§6 "초기 기준값 예시", §7 "초기 가중치") ──
# 문서에 구체 수치가 없는 항목(D_SAFE_M, MAX_SPEED_DIFF, RSSI 범위,
# CHECKPOINT_ADJACENT_RANGE)은 §15.5 "파라미터 실험 및 튜닝" 대상이라
# 임시값으로 채워뒀습니다 — 실차 시험 전에 반드시 교체하세요.
MIN_ROUTE_OVERLAP = 2                      # §6 최소 경로 중첩 (연속 체크포인트 개수)
MAX_HEADING_DIFF_RAD = math.radians(30)    # §6 최대 진행 방향 차이
MAX_JOIN_DISTANCE_M = 5.0                  # §6 최대 후보 거리 (= §7.1 d_max)
D_SAFE_M = 0.5                             # TODO: 실측값으로 교체 (§7.1 d_safe, 문서에 예시 없음)
MAX_SPEED_DIFF = 1.0                       # TODO: 실제 최대속도 기준으로 교체 (§7.3 Δv_max)
RSSI_MIN = -90.0                           # TODO: 실제 ESP-NOW RSSI 범위로 교체 (§7.4)
RSSI_MAX = -30.0
PDR_WEIGHT = 0.7                           # §7.4 alpha
CHECKPOINT_ADJACENT_RANGE = 1              # TODO: "인접" 체크포인트 판정 기준 확정 필요 (§5)

W_DISTANCE = 0.25   # S_d
W_ROUTE = 0.35       # S_r
W_SPEED = 0.20       # S_v
W_COMM = 0.10        # S_q
W_ANGLE = 0.10       # S_a

T_JOIN = 0.70                # §8 결합 기준점수
STABILITY_WINDOW = 5         # §8 최근 5회 평가
STABILITY_PASS_REQUIRED = 4  # §8 그 중 4회 이상 통과

# 응답이 오지 않을 때 무한 대기하지 않도록 (문서에 명시 없음, 실측 후 조정)
JOIN_RESPONSE_TIMEOUT_S = 2.0
SETUP_TIMEOUT_S = 2.0

# §15.3 예외 처리 — 플래툰 중 상대 차량 정보가 이 시간 이상 끊기면 안전 이탈
PARTNER_LOST_TIMEOUT_S = 1.0

# ── 전방 장애물 대응 (초음파 기반) ─────────────────────────────────
# 초음파는 "앞에 뭐가 있는지"를 보고, UWB는 "리더가 어디 있는지"를 본다.
# 플래툰 중에는 초음파에 리더가 항상 잡히므로 그것만으로는 장애물 판정이 불가능하다.
# 초음파 거리가 UWB 거리보다 유의미하게 가까우면 = 리더와 나 사이에 뭔가 끼어든 것.
OBSTACLE_STOP_DISTANCE_M = 0.30   # TODO: 제동거리 실측 후 조정 — 즉시 정지
OBSTACLE_SLOW_DISTANCE_M = 0.60   # TODO: 감속 시작 거리
CUTIN_MARGIN_M = 0.15             # TODO: UWB/초음파 측정 오차보다 크게 잡아야 오탐 방지
OBSTACLE_SLOW_FACTOR = 0.5        # 감속 시 목표속도 배율

# §10.2 거절 사유 코드
REJECT_NOT_ALLOWED = 1      # 플래툰 참여 비허용 / 긴급·고장 상태
REJECT_BUSY = 2             # 이미 다른 결합 절차 진행 중
REJECT_CONDITION_FAIL = 3   # 절대조건 미달 (경로·방향·거리)
REJECT_LOW_SCORE = 4        # 자체 적합도가 기준점수 미달
REJECT_UNKNOWN_VEHICLE = 5  # 요청 차량을 주변 목록에서 못 찾음


# ── JOIN / MAINTAIN / EXIT 파라미터 (§11, §19, §20, §21) ───────────
TARGET_DISTANCE_M = 0.8        # §11.1, §20.3 초기 목표 차간거리 (고정값으로 시작)
DISTANCE_TOLERANCE_M = 0.1     # §19.7 |현재거리 - 목표거리| 허용오차
SPEED_TOLERANCE = 0.1          # §19.7 |리더속도 - 팔로워속도| 허용오차
STABLE_TIME_S = 1.0            # §19.7, §21.4 안정 상태 유지시간
APPROACH_ZONE_M = 0.3          # §19.5 목표거리 근접구간 (목표거리 + 0.3m)
MAX_CLOSING_SPEED = 0.3        # §19.5 최대 접근속도
MAX_SPEED_STEP = 0.05          # §19.5 한 주기 속도 변화량 제한

D_RELEASE_M = 2.0              # §21.4 플래툰 해제 완료 거리
EXIT_REMAINING_CHECKPOINTS = 1 # §21.1 공통경로 마지막보다 1개 앞선 체크포인트에서 EXIT 시작

# 제어 이득 — 문서에 "실제 적용 전 결정해야 한다"고만 되어 있어 임시값 (§20.2)
K_D = 0.4     # TODO: 거리 제어 이득, 실차 튜닝 필요
K_V = 0.6     # TODO: 상대속도 제어 이득, 실차 튜닝 필요
K_L_JOIN = 0.3  # TODO: JOIN 시 리더 감속 이득 (§19.5)
K_F_JOIN = 0.5  # TODO: JOIN 시 팔로워 가속 이득 (§19.5)
K_EL_EXIT = 0.3  # TODO: EXIT 시 리더 가속 이득 (§21.3)
K_EF_EXIT = 0.3  # TODO: EXIT 시 팔로워 감속 이득 (§21.3)

V_MIN = 0.0   # TODO: 실제 차량 최소속도로 교체
V_MAX = 1.0   # TODO: 실제 차량 최대속도로 교체

# §19.4 차선 정렬 허용 오차 — 카메라 기반 차선 중앙 오프셋 (설계문서 원안)
# 라인트레이싱을 계속 수행하므로 문서대로 카메라 값을 쓴다.
# 값은 정규화된 오프셋(-1.0 ~ +1.0) 기준.
LANE_ALIGN_OFFSET_TOLERANCE = 0.15  # TODO: 실측 튜닝 필요


@dataclass
class EgoState:
    """자차 주행 정보 — 주행 알고리즘(라인트레이싱 모듈)에서 받아옴"""
    checkpoint: int = 0
    next_checkpoint: int = 0
    route: list = field(default_factory=list)   # 현재 위치 이후 예정 체크포인트 리스트
    destination: int = 0
    speed: float = 0.0
    accel: float = 0.0
    heading: float = 0.0
    lane: int = 0
    lane_offset: float = 0.0      # 카메라 기반 차선 중앙 오프셋 (-1.0 ~ +1.0)
    lane_detected: bool = False   # 이번 프레임에서 라인을 찾았는지
    obstacle: bool = False        # STM32가 올려준 근접 감지 플래그
    front_distance: Optional[float] = None  # 초음파 전방거리(m), 미측정이면 None
    stm32_failsafe: bool = False  # STM32가 RPi 무응답으로 자체 정지 중인지
    platoon_allow: bool = True
    emergency: bool = False


@dataclass
class NearbyVehicle:
    """V2X(ESP32-S3)로부터 올라온 주변 차량 정보"""
    vehicle_id: int
    checkpoint: int = 0
    next_checkpoint: int = 0
    route: list = field(default_factory=list)
    destination: int = 0
    lane: int = 0
    speed: float = 0.0
    accel: float = 0.0
    heading: float = 0.0
    platoon_allow: bool = True
    platoon_state: str = "SOLO"
    platoon_id: Optional[int] = None
    leader_id: Optional[int] = None
    pdr: float = 1.0          # 0~1, S_PDR로 그대로 사용
    rssi: float = 0.0
    emergency: bool = False
    uwb_distance: Optional[float] = None
    uwb_angle: Optional[float] = None
    timestamp: float = 0.0


@dataclass
class DrivingCommand:
    """주행 알고리즘(→STM32)으로 내려줄 출력"""
    mode: str = "SOLO_DRIVE"
    target_lane: Optional[int] = None
    target_speed: Optional[float] = None
    target_distance: Optional[float] = None
    leader_id: Optional[int] = None
    accel_request: Optional[str] = None  # ACCELERATE / HOLD / DECELERATE
    emergency: bool = False              # 비상정지 필요 (STM32에 EMERGENCY_STOP 전송)


class PlatoonFSM:
    def __init__(self, vehicle_id: int = 0, comm: Optional[PlatoonComm] = None,
                 is_designated_leader: bool = False):
        """
        vehicle_id : 자차 고유 ID (V2X 패킷에 requester_id 등으로 실림)
                     TODO: 차량마다 실제 고유 ID 부여 방식 확정되면 교체
        comm       : ESP32와의 통신 인터페이스. 아직 시리얼 프로토콜이 없어서
                     기본값은 NullComm(아무것도 안 보내고 항상 빈 리스트 반환).
        is_designated_leader : 실차 시나리오 고정값 — 젯슨 차량은 True, 라즈베리파이
                     차량은 False. 역할이 UWB 각도 등으로 동적으로 정해지지 않고
                     보드 종류로 미리 고정된다.
        """
        self.vehicle_id = vehicle_id
        self.comm: PlatoonComm = comm or NullComm()
        self.is_designated_leader = is_designated_leader

        self.state = PlatoonState.SOLO_DRIVE
        self.match_state = MatchState.IDLE
        self.join_sub_state: Optional[JoinSubState] = None

        self.platoon_id: Optional[int] = None
        self.role: Optional[str] = None       # "LEADER" / "FOLLOWER"
        self.leader_id: Optional[int] = None
        self.partner_id: Optional[int] = None   # 내 바로 앞차(예정 상대 포함) — 거리유지/JOIN 대상
        self.successor_id: Optional[int] = None  # 내 바로 뒤에 결합한 차량. 있으면 나는 "맨 뒤"가 아님
        self.candidate_id: Optional[int] = None
        self._pending_candidate: Optional[NearbyVehicle] = None  # JOIN_REQUEST 보낸 상대 캐시
        self._platoon_seq = 1  # TODO: 여러 대 동시 결합 시 전역 고유성 보장 필요 (지금은 단순 카운터)

        # §20.4~20.5 플래툰 공통 경로 / EXIT 시작점
        self.platoon_route: list = []
        self.exit_start_checkpoint: Optional[int] = None

        # 조건이 "일정 시간 유지"되었는지 재는 타이머 (§19.4, §19.7, §21.4)
        self._stable_since: Optional[float] = None
        self._last_target_speed: Optional[float] = None  # 한 주기 속도 변화량 제한용 (§19.5)

        # 수신 처리 결과를 담아두는 플래그
        self._join_response: Optional[bool] = None   # True=승인 / False=거절 / None=무응답
        self._wait_since: Optional[float] = None     # 응답·SETUP 대기 시작 시각
        self._peer_join_complete = False             # 상대가 JOIN_COMPLETE를 보냈는지
        self._partner_last_seen: Optional[float] = None  # 상대 차량 정보를 마지막으로 본 시각
        self.partner_lost = False                    # 통신 두절로 안전 이탈 중인지
        self._hazard_slow = False                    # 전방 위험으로 감속 중인지
        self.emergency_reason: Optional[str] = None  # 마지막 비상정지 사유 (로깅용)

        # vehicle_id -> 최근 평가 결과(bool) 리스트, 최대 STABILITY_WINDOW개 (§8)
        self._score_history: dict = {}

    # ── 메인 루프에서 매 주기 호출 ──────────────────────────────────
    def update(self, ego: EgoState, nearby: list) -> DrivingCommand:
        # 수신 패킷은 매 주기 한 번만 뽑아서 여기서 전부 처리한다.
        # (여러 곳에서 poll하면 패킷이 서로 잡아먹혀 유실된다)
        self._handle_incoming(ego, nearby)
        self._expire_open_slot()

        # 상태 로직보다 먼저 — 어떤 상태에 있든 안전이 우선한다
        emergency_cmd = self._check_safety(ego, nearby)
        if emergency_cmd is not None:
            return emergency_cmd

        if self.state == PlatoonState.SOLO_DRIVE:
            cmd = self._run_solo_drive(ego, nearby)
        elif self.state == PlatoonState.PLATOON_JOIN:
            cmd = self._run_platoon_join(ego, nearby)
        elif self.state == PlatoonState.PLATOON_MAINTAIN:
            cmd = self._run_platoon_maintain(ego, nearby)
        elif self.state == PlatoonState.PLATOON_EXIT:
            cmd = self._run_platoon_exit(ego, nearby)
        else:
            raise RuntimeError(f"unknown state: {self.state}")

        # 전방 위험 감속 — 정지까지는 아니지만 속도를 낮춰야 하는 구간
        if self._hazard_slow and cmd.target_speed is not None:
            cmd.target_speed *= OBSTACLE_SLOW_FACTOR

        return cmd

    # ══════════════════════════════════════════════════════════════
    # 안전 검사 — 장애물 / STM32 failsafe
    # ══════════════════════════════════════════════════════════════
    def _check_safety(self, ego: EgoState, nearby: list) -> Optional[DrivingCommand]:
        """
        비상정지가 필요하면 DrivingCommand를 반환하고, 아니면 None.
        플래툰 중이면 EMERGENCY 패킷을 전파해서 뒤차도 함께 대응하게 한다.
        """
        # STM32가 자체 정지 중 — FSM이 계속 주행 중인 줄 알면 안 된다
        if ego.stm32_failsafe:
            return self._emergency_stop(ego, "STM32 failsafe (RPi 무응답 감지)")

        hazard = self._front_hazard(ego, nearby)
        if hazard == "STOP":
            return self._emergency_stop(ego, "전방 장애물 근접")

        self._hazard_slow = (hazard == "SLOW")
        return None

    def _front_hazard(self, ego: EgoState, nearby: list) -> Optional[str]:
        """
        초음파 기반 전방 위험 판정. 반환값: "STOP" / "SLOW" / None

        플래툰 팔로워는 초음파에 앞차가 잡히는 게 정상이므로, 그것만으로 장애물이라
        판정하면 안 된다. UWB로 잰 앞차 거리보다 초음파가 유의미하게 가까울 때만
        "사이에 끼어든 것"으로 본다.

        단, 이 교차판정은 **상대가 내 앞에 있을 때만** 성립한다. 리더는 partner_id가
        뒤차(successor)라서 전방 초음파에 절대 잡히지 않는데, 그 뒤차 거리와 비교하면
        진짜 전방 장애물을 "앞차 자신"으로 오판해 비상정지를 놓친다.
        (예: 뒤차 0.4m, 전방 장애물 0.35m → 0.35 ≥ 0.4-0.15 이라 정지 안 함)
        """
        d = ego.front_distance
        if d is None:
            return None   # 초음파 값 없음 (미측정 또는 STM32 미연결)

        partner = self._find_partner(nearby) if self.partner_id else None
        partner_ahead = (self.state != PlatoonState.SOLO_DRIVE
                         and self.role == "FOLLOWER"
                         and partner is not None
                         and partner.uwb_distance is not None)

        if partner_ahead:
            # 앞차보다 가까운 무언가가 있는지
            if d >= partner.uwb_distance - CUTIN_MARGIN_M:
                return None   # 초음파가 본 게 앞차 자신
        # 단독주행·리더이거나, 끼어든 물체가 확인된 경우 → 순수 거리로 판정
        if d <= OBSTACLE_STOP_DISTANCE_M:
            return "STOP"
        if d <= OBSTACLE_SLOW_DISTANCE_M:
            return "SLOW"
        return None

    def _emergency_stop(self, ego: EgoState, reason: str) -> DrivingCommand:
        """비상정지. 플래툰 중이었다면 EMERGENCY를 전파하고 플래툰을 해제한다."""
        if self.state != PlatoonState.SOLO_DRIVE:
            self.comm.send(EmergencyMessage(
                sender_id=self.vehicle_id,
                platoon_id=self.platoon_id or 0,
                emergency_type="STOP",
                timestamp=time.time(),
            ))
            self._release_platoon()

        self.emergency_reason = reason
        return DrivingCommand(mode="SOLO_DRIVE", target_speed=0.0, emergency=True)

    # ── SOLO_DRIVE + 매칭 파이프라인 (§4~§13) ───────────────────────
    def _run_solo_drive(self, ego: EgoState, nearby: list) -> DrivingCommand:
        if self.is_designated_leader:
            # 젯슨(리더)은 능동적으로 결합 상대를 찾지 않는다 — 항상 리더이므로,
            # 결합은 팔로워 쪽에서 보내는 JOIN_REQUEST를 받는 것으로만 시작된다
            # (_on_join_request에서 처리). 리더는 그저 계속 SOLO_DRIVE로 주행하며 대기한다.
            return DrivingCommand(mode="SOLO_DRIVE")

        if self.match_state == MatchState.IDLE:
            self.match_state = MatchState.V2X_SEARCH

        if self.match_state == MatchState.V2X_SEARCH:
            candidates = self._prefilter(ego, nearby)   # §5 1차 선별
            if candidates:
                self.match_state = MatchState.EVALUATE
            # else: SOLO_DRIVE 유지, 계속 탐색

        elif self.match_state == MatchState.EVALUATE:
            candidates = self._prefilter(ego, nearby)
            best, score = self._evaluate_candidates(ego, candidates)  # §6~§8
            if best is not None and self._is_score_stable(best.vehicle_id):
                self.candidate_id = best.vehicle_id
                self._send_join_request(ego, best, score)
                self.match_state = MatchState.JOIN_REQUEST
            else:
                self.match_state = MatchState.V2X_SEARCH

        elif self.match_state == MatchState.JOIN_REQUEST:
            # 승인/거절 확인 (실제 수신은 _handle_incoming에서 처리됨)
            accepted = self._check_join_response()
            if accepted is True:
                self.match_state = MatchState.PLATOON_SETUP
            elif accepted is False:
                self._reset_matching()

        elif self.match_state == MatchState.AWAIT_SETUP:
            # 승인을 보낸 쪽 — 상대가 보내줄 PLATOON_SETUP 대기.
            # 실제 전이는 _on_platoon_setup()에서 일어나고, 여기서는 타임아웃만 본다.
            if self._wait_since is not None and (time.time() - self._wait_since) > SETUP_TIMEOUT_S:
                self._reset_matching()

        elif self.match_state == MatchState.PLATOON_SETUP:
            # §10.3 리더/팔로워 역할 결정 + 플래툰 ID 생성/공유
            # TODO: §18 ESP32에 PLATOON_HIGH_RATE 통신모드 전환 명령
            self._setup_platoon()
            self.match_state = MatchState.IDLE
            self.state = PlatoonState.PLATOON_JOIN
            self.join_sub_state = JoinSubState.JOIN_REQUESTED
            self._stable_since = None

        return DrivingCommand(mode="SOLO_DRIVE")

    # ── PLATOON_JOIN 세부 단계 (§19) ────────────────────────────────
    def _run_platoon_join(self, ego: EgoState, nearby: list) -> DrivingCommand:
        partner = self._find_partner(nearby)

        # §15.3 상대 차량 소실 — 결합 중단하고 안전 이탈
        if self._check_partner_lost():
            return self._abort_to_exit("JOIN 중 상대 차량 정보 두절")

        if self.join_sub_state == JoinSubState.JOIN_REQUESTED:
            # 상대 차량 정보가 실제로 잡히면 정렬 단계로 진행
            if partner is not None:
                self.join_sub_state = JoinSubState.JOIN_LANE_ALIGN
                self._stable_since = None

        elif self.join_sub_state == JoinSubState.JOIN_LANE_ALIGN:
            if self._lane_align_complete(ego, partner):      # §19.4
                self.join_sub_state = JoinSubState.JOIN_GAP_CONTROL
                self._stable_since = None

        elif self.join_sub_state == JoinSubState.JOIN_GAP_CONTROL:
            if self._gap_control_complete(partner):          # §19.5
                self.join_sub_state = JoinSubState.JOIN_STABILIZE
                self._stable_since = None

        elif self.join_sub_state == JoinSubState.JOIN_STABILIZE:
            if self._join_stabilized(ego, partner):          # §19.7
                # §19.7 양쪽이 JOIN_COMPLETE를 교환하고 PLATOON_MAINTAIN으로 전환
                self.comm.send(JoinComplete(
                    sender_id=self.vehicle_id,
                    platoon_id=self.platoon_id or 0,
                    timestamp=time.time(),
                ))
                self.join_sub_state = None
                self._stable_since = None
                self.state = PlatoonState.PLATOON_MAINTAIN
                self._remember_platoon_route(ego, partner)   # §20.4

        target_speed, accel_request = self._join_target_speed(ego, partner)

        return DrivingCommand(
            mode="PLATOON_JOIN",
            leader_id=self.leader_id,
            target_lane=partner.lane if (partner and self.role == "FOLLOWER") else ego.lane,
            target_speed=target_speed,
            target_distance=TARGET_DISTANCE_M,
            accel_request=accel_request,
        )

    # ── PLATOON_MAINTAIN (§20) ──────────────────────────────────────
    def _run_platoon_maintain(self, ego: EgoState, nearby: list) -> DrivingCommand:
        partner = self._find_partner(nearby)

        # §15.3 상대 차량 소실 — 간격 제어 근거가 사라졌으므로 안전 이탈
        if self._check_partner_lost():
            return self._abort_to_exit("MAINTAIN 중 상대 차량 정보 두절")

        # TODO: §20.6 신규 차량 지속 탐색 및 맨 뒤 합류 처리 (2대 결합 안정화 후 확장)
        if self._exit_condition_met(ego):                    # §21.1
            self.comm.send(LeaveMessage(
                sender_id=self.vehicle_id,
                platoon_id=self.platoon_id or 0,
                timestamp=time.time(),
            ))
            self.state = PlatoonState.PLATOON_EXIT
            self._stable_since = None

        if self.role == "FOLLOWER" and partner is not None:
            # §20.2  v_target = v_L + Kd(d - d_target) + Kv(v_L - v_F)
            distance = self._partner_distance(partner)
            raw = (partner.speed
                   + K_D * ((distance - TARGET_DISTANCE_M) if distance is not None else 0.0)
                   + K_V * (partner.speed - ego.speed))
            target_speed = self._limit_speed(raw)
        else:
            target_speed = self._limit_speed(ego.speed)      # 리더는 자기 기준속도 유지

        return DrivingCommand(
            mode="PLATOON_MAINTAIN",
            leader_id=self.leader_id,
            target_lane=ego.lane,
            target_speed=target_speed,
            target_distance=TARGET_DISTANCE_M,
        )

    def _abort_to_exit(self, reason: str) -> DrivingCommand:
        """
        §15.3 예외 상황에서 플래툰을 중단한다.

        상대 위치를 모르는 상태이므로 EXIT의 간격 확대 제어(거리 피드백)를
        쓸 수 없다. 팔로워는 UWB 추종도 불가능하므로 즉시 단독주행으로
        되돌려 카메라 차선추종을 다시 쓰게 하고, 감속은 주행 알고리즘에 맡긴다.
        """
        self.partner_lost = True
        self.comm.send(LeaveMessage(
            sender_id=self.vehicle_id,
            platoon_id=self.platoon_id or 0,
            timestamp=time.time(),
        ))
        self._release_platoon()
        # TODO: 소실 직후 감속 정도를 어떻게 할지 결정 필요.
        #       지금은 목표속도를 지정하지 않아 주행 알고리즘의 기본값을 따른다.
        return DrivingCommand(mode="SOLO_DRIVE")

    def _release_platoon(self) -> None:
        """플래툰 관련 상태를 전부 초기화하고 단독주행으로 되돌린다."""
        self.state = PlatoonState.SOLO_DRIVE
        self.match_state = MatchState.IDLE
        self.join_sub_state = None
        self.platoon_id = None
        self.role = None
        self.leader_id = None
        self.partner_id = None
        self.successor_id = None
        self.candidate_id = None
        self.platoon_route = []
        self.exit_start_checkpoint = None
        self._pending_candidate = None
        self._stable_since = None
        self._join_response = None
        self._wait_since = None
        self._peer_join_complete = False
        self._partner_last_seen = None
        self._last_target_speed = None
        self._score_history.clear()

    # ── PLATOON_EXIT (§21) ──────────────────────────────────────────
    def _run_platoon_exit(self, ego: EgoState, nearby: list) -> DrivingCommand:
        partner = self._find_partner(nearby)

        # §15.3 해제 중 상대 소실 — 이미 분리된 것으로 보고 단독주행 복귀
        if self._check_partner_lost():
            return self._abort_to_exit("EXIT 중 상대 차량 정보 두절")

        distance = self._partner_distance(partner)

        # §21.3 간격 확대: 리더는 제한범위 내 가속, 팔로워는 제한범위 내 감속
        e_exit = D_RELEASE_M - distance if distance is not None else 0.0
        if self.role == "LEADER":
            target_speed = self._limit_speed(ego.speed + K_EL_EXIT * e_exit)
        else:
            target_speed = self._limit_speed(ego.speed - K_EF_EXIT * e_exit)

        if self._exit_complete(distance):                    # §21.4
            # TODO: §21.2 EXIT_REQUEST/ACK/START, PLATOON_EXIT_COMPLETE 패킷 교환 미구현
            self._release_platoon()
            return DrivingCommand(mode="SOLO_DRIVE")

        return DrivingCommand(
            mode="PLATOON_EXIT",
            leader_id=self.leader_id,
            target_speed=target_speed,
            target_distance=D_RELEASE_M,
        )

    # ══════════════════════════════════════════════════════════════
    # §5 주변 차량 1차 선별
    # ══════════════════════════════════════════════════════════════
    def _prefilter(self, ego: EgoState, nearby: list) -> list:
        candidates = []
        for v in nearby:
            checkpoint_close = abs(v.checkpoint - ego.checkpoint) <= CHECKPOINT_ADJACENT_RANGE
            same_direction = (v.next_checkpoint == ego.next_checkpoint)
            lane_ok = True  # TODO: 차선 인접 규칙 아직 미정 (도로 차선 구조 확정되면 채우기)
            within_range = v.uwb_distance is not None and v.uwb_distance <= MAX_JOIN_DISTANCE_M
            has_bearing = v.uwb_angle is not None  # 앞/뒤 판단 가능 여부

            if checkpoint_close and same_direction and lane_ok and within_range and has_bearing:
                candidates.append(v)
        return candidates

    # ══════════════════════════════════════════════════════════════
    # §6 절대조건 (C_join)
    # ══════════════════════════════════════════════════════════════
    def _check_absolute_conditions(self, ego: EgoState, v: "NearbyVehicle") -> bool:
        c_p = bool(ego.platoon_allow and v.platoon_allow)
        c_r = self._route_overlap_count(ego, v) >= MIN_ROUTE_OVERLAP
        c_h = abs(ego.heading - v.heading) <= MAX_HEADING_DIFF_RAD
        c_d = v.uwb_distance is not None and v.uwb_distance <= MAX_JOIN_DISTANCE_M
        c_s = (not ego.emergency) and (not v.emergency)
        return c_p and c_r and c_h and c_d and c_s

    def _route_overlap_count(self, ego: EgoState, v: "NearbyVehicle") -> int:
        """
        §7.2: 현재 위치 이후 동일한 순서로 연속해서 겹치는 체크포인트 개수(N_common).
        ego.route / v.route는 "현재 위치 이후의 예정 체크포인트 리스트"로 가정.
        """
        common = 0
        for a, b in zip(ego.route, v.route):
            if a != b:
                break
            common += 1
        return common

    # ══════════════════════════════════════════════════════════════
    # §7 플래툰 적합도 계산 (S_platoon)
    # ══════════════════════════════════════════════════════════════
    def _distance_score(self, d: Optional[float]) -> float:
        if d is None or d < D_SAFE_M or d > MAX_JOIN_DISTANCE_M:
            return 0.0
        return 1 - (d - D_SAFE_M) / (MAX_JOIN_DISTANCE_M - D_SAFE_M)

    def _route_score(self, ego: EgoState, v: "NearbyVehicle") -> float:
        n_common = self._route_overlap_count(ego, v)
        n_i = len(ego.route) or 1
        n_j = len(v.route) or 1
        return n_common / min(n_i, n_j)

    def _speed_score(self, ego: EgoState, v: "NearbyVehicle") -> float:
        return max(0.0, 1 - abs(ego.speed - v.speed) / MAX_SPEED_DIFF)

    def _normalize_rssi(self, rssi: float) -> float:
        span = RSSI_MAX - RSSI_MIN
        if span <= 0:
            return 0.0
        return max(0.0, min(1.0, (rssi - RSSI_MIN) / span))

    def _comm_score(self, v: "NearbyVehicle") -> float:
        s_pdr = max(0.0, min(1.0, v.pdr))
        s_rssi = self._normalize_rssi(v.rssi)
        return PDR_WEIGHT * s_pdr + (1 - PDR_WEIGHT) * s_rssi

    def _angle_score(self, ego: EgoState, v: "NearbyVehicle") -> float:
        return max(0.0, 1 - abs(ego.heading - v.heading) / MAX_HEADING_DIFF_RAD)

    def _score_platoon(self, ego: EgoState, v: "NearbyVehicle") -> float:
        s_d = self._distance_score(v.uwb_distance)
        s_r = self._route_score(ego, v)
        s_v = self._speed_score(ego, v)
        s_q = self._comm_score(v)
        s_a = self._angle_score(ego, v)
        return (W_DISTANCE * s_d + W_ROUTE * s_r + W_SPEED * s_v
                + W_COMM * s_q + W_ANGLE * s_a)

    def _evaluate_candidates(self, ego: EgoState, candidates: list):
        """
        절대조건을 통과한 후보 중 최고 적합도 차량을 고른다 (§8 arg max, 동점 시 §8 우선순위 일부 반영).
        반환값: (best: NearbyVehicle | None, score: float)
        """
        best = None
        best_score = -1.0
        for v in candidates:
            if not self._check_absolute_conditions(ego, v):
                continue
            score = self._score_platoon(ego, v)
            if score > best_score:
                best, best_score = v, score
            elif score == best_score and best is not None:
                # 동점 시 우선순위: 경로 중첩 > 속도차 작음 > 통신품질 > ID 작음
                if self._route_overlap_count(ego, v) > self._route_overlap_count(ego, best):
                    best, best_score = v, score
                elif v.vehicle_id < best.vehicle_id:
                    best, best_score = v, score

        if best is not None:
            self._record_score(best.vehicle_id, best_score >= T_JOIN)

        return best, best_score

    # ══════════════════════════════════════════════════════════════
    # §8 최종 결합 판단 — 5회 중 4회 이상 통과해야 결합 요청 (순간값 방지)
    # ══════════════════════════════════════════════════════════════
    def _record_score(self, vehicle_id: int, passed: bool) -> None:
        history = self._score_history.setdefault(vehicle_id, [])
        history.append(passed)
        if len(history) > STABILITY_WINDOW:
            history.pop(0)

    def _is_score_stable(self, vehicle_id: int) -> bool:
        history = self._score_history.get(vehicle_id, [])
        return len(history) >= STABILITY_WINDOW and sum(history) >= STABILITY_PASS_REQUIRED

    # ══════════════════════════════════════════════════════════════
    # 수신 패킷 처리 — 매 주기 update() 맨 앞에서 한 번만 실행
    # ══════════════════════════════════════════════════════════════
    def _handle_incoming(self, ego: EgoState, nearby: list) -> None:
        for packet in self.comm.poll():
            if isinstance(packet, PlatoonJoinRequest):
                self._on_join_request(packet, ego, nearby)
            elif isinstance(packet, PlatoonJoinAccept):
                self._on_join_accept(packet)
            elif isinstance(packet, PlatoonJoinReject):
                self._on_join_reject(packet)
            elif isinstance(packet, PlatoonSetup):
                self._on_platoon_setup(packet)
            elif isinstance(packet, JoinComplete):
                self._on_join_complete(packet)
            elif isinstance(packet, LeaveMessage):
                self._on_leave(packet)
            elif isinstance(packet, EmergencyMessage):
                self._on_emergency(packet)
            # 알 수 없는 패킷은 무시

    # ── §10.2 결합 요청 수신 → 자체 재검사 후 승인/거절 ──────────────
    def _on_join_request(self, req: PlatoonJoinRequest, ego: EgoState, nearby: list) -> None:
        # 1. 대상 차량 ID 확인 — 나에게 온 요청이 아니면 무시
        if req.target_id != self.vehicle_id:
            return

        def reject(code: int) -> None:
            self.comm.send(PlatoonJoinReject(
                responder_id=self.vehicle_id,
                requester_id=req.requester_id,
                reason_code=code,
                timestamp=time.time(),
            ))

        # 2. 자차의 플래툰 참여 허용 여부
        if not ego.platoon_allow or ego.emergency:
            reject(REJECT_NOT_ALLOWED)
            return

        # 2-1. 실차 시나리오상 리더는 젯슨 1대로 고정이다. 아직 플래툰에 속하지 않은
        #      팔로워 차량(라즈베리파이)이 요청을 받아주면, 요청자가 SETUP에서 나를
        #      leader_id로 지정하기 때문에 "라즈베리파이가 리더인 플래툰"이 만들어진다.
        #      팔로워는 이미 플래툰에 들어가 있을 때(= 맨 뒤 자리를 내주는 경우)만 받는다.
        if not self.is_designated_leader and self.state == PlatoonState.SOLO_DRIVE:
            reject(REJECT_NOT_ALLOWED)
            return

        # 3. "빈 자리(맨 뒤)"인지 확인 — 3대 이상 체인 지원을 위해 §20.7 방식으로:
        #    - 아직 아무와도 안 붙었고, 다른 상대와 협상 중도 아니거나 (리더의 첫 결합)
        #    - 이미 MAINTAIN 중이지만 내 뒤에는 아직 아무도 없는 경우(현재 맨 뒤 팔로워)
        #    둘 중 하나여야 새 요청을 받을 수 있다. 예전엔 "이미 결합돼 있으면 무조건 거절"
        #    이었는데, 그러면 2번째 차가 결합하는 순간부터 3번째 차는 영원히 거절당한다.
        #    AWAIT_SETUP 제외는 두 대가 같은 맨 뒤 자리에 동시에 승인받는 것을 막는다.
        is_open_slot = (
            (self.state == PlatoonState.SOLO_DRIVE
             and self.match_state not in (MatchState.JOIN_REQUEST,
                                          MatchState.AWAIT_SETUP,
                                          MatchState.PLATOON_SETUP))
            or (self.state == PlatoonState.PLATOON_MAINTAIN
                and self.successor_id is None
                and self.match_state != MatchState.AWAIT_SETUP)
        )
        if not is_open_slot:
            reject(REJECT_BUSY)
            return

        # 4. 요청 차량을 주변 목록에서 찾아 경로·방향·거리 재확인
        requester = None
        for v in nearby:
            if v.vehicle_id == req.requester_id:
                requester = v
                break
        if requester is None:
            reject(REJECT_UNKNOWN_VEHICLE)
            return

        if not self._check_absolute_conditions(ego, requester):
            reject(REJECT_CONDITION_FAIL)
            return

        # 5. 자체 적합도 계산 — C_agreement = J_ij ∧ J_ji (양쪽 다 통과해야 결합)
        score = self._score_platoon(ego, requester)
        if score < T_JOIN:
            reject(REJECT_LOW_SCORE)
            return

        # 6. 승인. 플래툰 ID는 요청 차량이 생성해서 PLATOON_SETUP으로 내려주므로 대기한다.
        self.candidate_id = req.requester_id
        self._pending_candidate = requester
        self.match_state = MatchState.AWAIT_SETUP
        self._wait_since = time.time()
        self.comm.send(PlatoonJoinAccept(
            responder_id=self.vehicle_id,
            requester_id=req.requester_id,
            suitability=int(round(max(0.0, min(1.0, score)) * 100)),
            timestamp=time.time(),
        ))

    def _on_join_accept(self, pkt: PlatoonJoinAccept) -> None:
        if pkt.requester_id == self.vehicle_id and pkt.responder_id == self.candidate_id:
            self._join_response = True

    def _on_join_reject(self, pkt: PlatoonJoinReject) -> None:
        if pkt.requester_id == self.vehicle_id and pkt.responder_id == self.candidate_id:
            self._join_response = False
            # 거절당한 상대에게 매 주기 재요청하지 않도록 안정성 이력을 비운다.
            # 이력이 남아 있으면 _is_score_stable()이 계속 True라서 거절→재요청을
            # 제어주기마다 반복한다. 비워두면 §8대로 5회 재평가 후에나 다시 시도한다.
            self._score_history.pop(pkt.responder_id, None)

    # ── §10.3 PLATOON_SETUP 수신 → 역할·플래툰 ID 채택 ──────────────
    def _on_platoon_setup(self, pkt: PlatoonSetup) -> None:
        """
        §10.3 SETUP 수신 처리.

        예전엔 "내가 pkt.leader_id나 pkt.follower_id 중 하나여야 한다"는 조건이었는데,
        3대 이상 체인에서는 기존 팔로워(맨 뒤, 응답자)가 pkt.leader_id(진짜 리더)도
        pkt.follower_id(신규 결합자)도 아니게 되어 이 조건에 걸려 자기가 방금 승인한
        결합 결과를 스스로 무시하는 문제가 있었다. 그래서 "내가 방금 승인 보내고
        기다리던 바로 그 상대가 보낸 SETUP인가"로 판정 기준을 바꿨다.
        """
        if not (self.match_state == MatchState.AWAIT_SETUP and pkt.sender_id == self.candidate_id):
            return
        if pkt.sender_id == self.vehicle_id:
            return

        if self.state == PlatoonState.SOLO_DRIVE:
            # 첫 결합 — 리더가 첫 팔로워를 받아들이는 경우 (또는 그 반대)
            self.platoon_id = pkt.platoon_id
            self.leader_id = pkt.leader_id
            self.role = "LEADER" if pkt.leader_id == self.vehicle_id else "FOLLOWER"
            self.partner_id = pkt.sender_id     # 내 JOIN 상대 = 방금 결합한 신규 차량
            self.successor_id = pkt.sender_id   # 이제 내 뒤에 이 차량이 붙음

            self.state = PlatoonState.PLATOON_JOIN
            self.join_sub_state = JoinSubState.JOIN_REQUESTED
        else:
            # 이미 플래툰 중(맨 뒤 팔로워)이었고, 내 뒤에 새 차량이 결합한 경우.
            # 내 리더/역할/앞차(partner_id)는 그대로 둔다 — 나는 여전히 내 앞차를
            # 그대로 따라가면 되고, 실제 정렬·간격 조절은 신규 차량 쪽에서 진행한다.
            self.successor_id = pkt.sender_id
            # successor가 나중에 EXIT/소실되면 _on_leave가 successor_id를 다시
            # None으로 돌려서 "맨 뒤" 자리를 재개방한다.

        # AWAIT_SETUP 해제는 두 경우 공통이다. 예전엔 SOLO 분기에서만 IDLE로
        # 돌려서, MAINTAIN 중 뒤차를 받아준 차량은 match_state가 AWAIT_SETUP에
        # 영원히 남았다 — 그러면 그 뒤차가 빠진 뒤에도 맨 뒤 자리가 다시 열리지 않는다.
        self.match_state = MatchState.IDLE
        self.candidate_id = None
        self._pending_candidate = None
        self._wait_since = None
        self._stable_since = None

    def _expire_open_slot(self) -> None:
        """
        맨 뒤 자리를 승인해줬는데 신규 차량이 PLATOON_SETUP을 보내지 않는 경우
        (요청자가 꺼졌거나 패킷 유실) 자리를 잠근 채 방치되지 않도록 대기를 푼다.

        SOLO_DRIVE 상태의 AWAIT_SETUP은 _run_solo_drive()가 같은 타임아웃으로
        이미 처리하므로 여기서는 플래툰 주행 중인 경우만 본다.
        """
        if self.state == PlatoonState.SOLO_DRIVE:
            return
        if self.match_state != MatchState.AWAIT_SETUP:
            return
        if self._wait_since is None or (time.time() - self._wait_since) <= SETUP_TIMEOUT_S:
            return
        self.match_state = MatchState.IDLE
        self.candidate_id = None
        self._pending_candidate = None
        self._wait_since = None

    def _on_join_complete(self, pkt: JoinComplete) -> None:
        if pkt.platoon_id == self.platoon_id and pkt.sender_id == self.partner_id:
            self._peer_join_complete = True

    def _on_leave(self, pkt: LeaveMessage) -> None:
        """
        상대가 플래툰을 떠난다는 통지.

        예전엔 "누가 하나 나가면 나도 무조건 EXIT"이었는데, 3대 이상에서는 안
        맞는다 — 맨 뒤(3번) 차가 빠지는데 리더나 중간 차까지 끌려 나가면 안 된다.
        그렇다고 successor_id만 비우면 반대로 "내 앞차가 빠졌는데 아무 반응도
        안 하는" 구멍이 생긴다. 그래서 앞/뒤를 나눠서 처리한다.

            내 뒤차(successor)가 나감 → 맨 뒤 자리만 다시 열고 계속 주행.
                                       단 리더는 partner_id == successor_id 라서
                                       뒤가 비면 플래툰 자체가 성립하지 않는다 → EXIT.
            내 앞차(partner)가 나감   → 간격 제어 기준이 사라지므로 나도 EXIT.
            그 외 차량               → 무관하므로 무시.

        TODO: 여기서 전이한 EXIT는 내 LEAVE를 다시 보내지 않는다. 내 뒤차에게는
              _check_partner_lost() 타임아웃(1초)을 통해 뒤늦게 전파되므로,
              §21.4의 "마지막 팔로워부터 순차 해제"를 정확히 맞추려면 EXIT_REQUEST/
              ACK 패킷(§21.2) 구현이 필요하다.
        """
        if pkt.platoon_id != self.platoon_id:
            return

        if pkt.sender_id == self.successor_id:
            self.successor_id = None
            if self.role != "LEADER":
                return   # 중간 차량은 앞차를 그대로 따라가면 된다
        elif pkt.sender_id != self.partner_id:
            return

        if self.state in (PlatoonState.PLATOON_JOIN, PlatoonState.PLATOON_MAINTAIN):
            self.state = PlatoonState.PLATOON_EXIT
            self.join_sub_state = None
            self._stable_since = None

    def _on_emergency(self, pkt: EmergencyMessage) -> None:
        """
        긴급 상황 수신 — 즉시 해제 절차로 전환한다.
        TODO: 급정지(STOP)는 EXIT의 완만한 간격확대가 아니라 즉시 정지가 맞다.
              주행 알고리즘에 EMERGENCY_STOP을 내리는 경로를 추가해야 함.
        """
        if pkt.platoon_id != self.platoon_id:
            return
        if self.state in (PlatoonState.PLATOON_JOIN, PlatoonState.PLATOON_MAINTAIN):
            self.state = PlatoonState.PLATOON_EXIT
            self.join_sub_state = None
            self._stable_since = None

    def _reset_matching(self) -> None:
        """매칭 절차를 처음부터 다시 — 거절·타임아웃 시 호출"""
        self.candidate_id = None
        self._pending_candidate = None
        self._join_response = None
        self._wait_since = None
        self.match_state = MatchState.V2X_SEARCH

    # ══════════════════════════════════════════════════════════════
    # §10 플래툰 결합 통신 절차 (JOIN_REQUEST → ACCEPT/REJECT → SETUP)
    # ══════════════════════════════════════════════════════════════
    def _send_join_request(self, ego: EgoState, candidate: "NearbyVehicle", score: float) -> None:
        """§10.1 — 결합 요청 전송. 응답 대기 중 candidate 정보를 캐시해둔다."""
        self._pending_candidate = candidate
        self._join_response = None
        self._wait_since = time.time()
        packet = PlatoonJoinRequest(
            requester_id=self.vehicle_id,
            target_id=candidate.vehicle_id,
            suitability=int(round(max(0.0, min(1.0, score)) * 100)),
            checkpoint_id=ego.checkpoint,
            destination_id=ego.destination,
            speed=ego.speed,
            distance=candidate.uwb_distance or 0.0,
            timestamp=time.time(),
        )
        self.comm.send(packet)

    def _check_join_response(self):
        """
        §10.2 — 승인/거절 결과 확인. 실제 패킷 수신은 _handle_incoming()에서 처리하고
        여기서는 그 결과 플래그만 읽는다.
        반환값: True(승인) / False(거절) / None(아직 응답 없음)
        """
        if self._join_response is not None:
            return self._join_response
        # 무응답 타임아웃 — 상대가 꺼졌거나 패킷이 유실된 경우
        if self._wait_since is not None and (time.time() - self._wait_since) > JOIN_RESPONSE_TIMEOUT_S:
            return False
        return None

    def _setup_platoon(self) -> None:
        """
        §10.3 — 리더/팔로워 역할 결정 + 플래툰 ID 생성/공유.

        역할은 더 이상 UWB 각도로 추측하지 않는다 — 실차 시나리오가 "젯슨=항상
        리더, 라즈베리파이=항상 팔로워"로 고정되어 있으므로 self.is_designated_leader
        하나로 결정된다.

        3대 이상 확장 대응: 이미 플래툰에 속한 차량(맨 뒤 팔로워)에 합류하는 경우,
        그 차량이 물려준 진짜 리더 ID·플래툰 ID를 그대로 받는다. 그렇지 않으면
        "내가 결합한 상대가 곧 리더"로 착각해서, 3번째 차가 2번째 차를 리더로
        오인하는 문제가 생긴다.

        TODO: §18 ESP32에 PLATOON_HIGH_RATE 통신모드 전환 명령은 아직 미구현
              (전용 통신 채널 자체가 아직 없음)
        """
        candidate = self._pending_candidate
        if candidate is None:
            return

        if self.is_designated_leader:
            self.role = "LEADER"
            self.leader_id = self.vehicle_id
        else:
            self.role = "FOLLOWER"
            # candidate가 이미 플래툰에 속해 있으면(맨 뒤 팔로워) 그 차량이 아는
            # 진짜 리더 ID를 물려받는다. 아니면(candidate가 리더 자신) candidate가 곧 리더.
            self.leader_id = candidate.leader_id if candidate.leader_id is not None else candidate.vehicle_id

        follower_id = self.vehicle_id if self.role == "FOLLOWER" else candidate.vehicle_id
        self.partner_id = candidate.vehicle_id  # 거리 유지 대상 = 방금 결합한 바로 앞차

        if candidate.platoon_id is not None:
            # 이미 결성된 플래툰 뒤에 합류 — 기존 플래툰 ID를 그대로 받는다
            self.platoon_id = candidate.platoon_id
        else:
            # 새 플래툰 시작 (리더와 첫 팔로워가 처음 만나는 순간)
            self.platoon_id = self.leader_id * 1000 + self._platoon_seq  # "Leader ID + Sequence" (§10.3)
            self._platoon_seq += 1

        self.comm.send(PlatoonSetup(
            sender_id=self.vehicle_id,
            platoon_id=self.platoon_id,
            leader_id=self.leader_id,
            follower_id=follower_id,
            timestamp=time.time(),
        ))

        self._pending_candidate = None

    # ══════════════════════════════════════════════════════════════
    # §19~§21 결합 이후 단계 판정
    # ══════════════════════════════════════════════════════════════
    def _find_partner(self, nearby: list) -> Optional["NearbyVehicle"]:
        """
        플래툰 상대 차량(리더 또는 팔로워)을 nearby 목록에서 찾는다.
        찾을 때마다 마지막 목격 시각을 갱신해 통신 두절 판정에 쓴다 (§15.3).
        """
        if self.partner_id is None:
            return None
        for v in nearby:
            if v.vehicle_id == self.partner_id:
                self._partner_last_seen = time.time()
                return v
        return None

    def _check_partner_lost(self) -> bool:
        """
        §15.3 상대 차량 정보가 일정 시간 이상 끊겼는지.

        팔로워에게는 안전 조건이다 — 앞차 정보 없이는 간격 제어를 할 수 없다.
        리더에게는 안전 문제가 아니라 상태 정리 문제다. 뒤차가 조용히 사라져도
        (전원 차단·통신 범위 이탈처럼 LEAVE조차 못 보내는 경우) 리더 자신의 주행에는
        지장이 없지만, 아무도 없는 플래툰의 MAINTAIN에 갇힌 채 사라진 차를
        partner_id로 붙들고 있게 된다. 그러면 새 차량이 붙어도 리더는 여전히
        옛 차량 기준으로 간격을 계산한다.

        리더의 이탈 처리는 감속을 동반하지 않는다(_abort_to_exit는 목표속도를
        지정하지 않고 SOLO_DRIVE로만 되돌린다). 또 PARTNER_LOST_TIMEOUT_S(1초)는
        전용 통신 주기(20~50ms, §18) 기준으로 20~50회 연속 유실이라 순간적인
        패킷 누락으로는 발동하지 않는다.
        """
        if self._partner_last_seen is None:
            self._partner_last_seen = time.time()
            return False
        return (time.time() - self._partner_last_seen) > PARTNER_LOST_TIMEOUT_S

    def _partner_distance(self, partner: Optional["NearbyVehicle"]) -> Optional[float]:
        """
        상대 차량과의 현재 거리.
        §19.6 권장 순서상 MAINTAIN 단계에서는 초음파가 주가 되어야 하지만,
        아직 초음파 값을 FSM으로 올려주는 경로가 없어 UWB 값만 사용한다.
        TODO: 초음파 센서 값 연동 후 근접구간에서 UWB와 교차검증
        """
        return partner.uwb_distance if partner else None

    def _hold(self, condition: bool, duration: float = STABLE_TIME_S) -> bool:
        """조건이 duration초 이상 연속 유지되었는지 판정 (§19.4, §19.7, §21.4)"""
        if not condition:
            self._stable_since = None
            return False
        now = time.time()
        if self._stable_since is None:
            self._stable_since = now
            return False
        return (now - self._stable_since) >= duration

    def _limit_speed(self, speed: float) -> float:
        """§19.5 목표속도 상·하한 + 한 주기 속도 변화량 제한"""
        speed = max(V_MIN, min(V_MAX, speed))
        if self._last_target_speed is not None:
            delta = speed - self._last_target_speed
            if abs(delta) > MAX_SPEED_STEP:
                speed = self._last_target_speed + math.copysign(MAX_SPEED_STEP, delta)
        self._last_target_speed = speed
        return speed

    def _join_target_speed(self, ego: EgoState, partner: Optional["NearbyVehicle"]):
        """
        §19.5 차간거리 조정.
        리더는 v_base - K_L·e_d, 팔로워는 v_base + K_F·e_d.
        접근속도(v_F - v_L)는 MAX_CLOSING_SPEED로 제한한다.
        """
        distance = self._partner_distance(partner)
        if partner is None or distance is None:
            return self._limit_speed(ego.speed), "HOLD"

        e_d = distance - TARGET_DISTANCE_M

        if self.role == "LEADER":
            raw = ego.speed - K_L_JOIN * e_d
        else:
            raw = ego.speed + K_F_JOIN * e_d
            # 팔로워 접근속도 제한
            max_speed = partner.speed + MAX_CLOSING_SPEED
            raw = min(raw, max_speed)

        target = self._limit_speed(raw)

        if abs(e_d) <= DISTANCE_TOLERANCE_M:
            accel_request = "HOLD"
        elif target > ego.speed:
            accel_request = "ACCELERATE"
        elif target < ego.speed:
            accel_request = "DECELERATE"
        else:
            accel_request = "HOLD"

        return target, accel_request

    def _lane_align_complete(self, ego: EgoState, partner: Optional["NearbyVehicle"]) -> bool:
        """
        §19.4 차선 정렬 완료 조건 (설계문서 원안 그대로).

            같은 차선 ID  and  |카메라 차선 오프셋| ≤ 허용치  and  1초 이상 유지

        라인트레이싱을 계속 수행하므로 카메라 오프셋을 그대로 쓸 수 있다.
        리더·팔로워가 각자 자기 차선 중앙에 정렬돼 있고 차선 ID가 같으면
        서로 정렬된 것이므로, 앞/뒤 위치와 무관하게 같은 기준이 적용된다.
        """
        if partner is None:
            return self._hold(False)
        same_lane = (partner.lane == ego.lane)
        centered = ego.lane_detected and abs(ego.lane_offset) <= LANE_ALIGN_OFFSET_TOLERANCE
        return self._hold(same_lane and centered)

    def _gap_control_complete(self, partner: Optional["NearbyVehicle"]) -> bool:
        """§19.5 목표거리 근접구간(목표거리 + 0.3m) 안으로 들어왔는지"""
        distance = self._partner_distance(partner)
        if distance is None:
            return False
        return distance <= TARGET_DISTANCE_M + APPROACH_ZONE_M

    def _join_stabilized(self, ego: EgoState, partner: Optional["NearbyVehicle"]) -> bool:
        """§19.7 JOIN 완료 조건 (동일차선 + 거리오차 + 속도오차, 1초 이상 유지)"""
        distance = self._partner_distance(partner)
        if partner is None or distance is None:
            return self._hold(False)
        # TODO: "플래툰 전용 통신 = 정상" 조건은 전용 통신 채널 구현 후 추가
        ok = (partner.lane == ego.lane
              and abs(distance - TARGET_DISTANCE_M) <= DISTANCE_TOLERANCE_M
              and abs(partner.speed - ego.speed) <= SPEED_TOLERANCE)
        return self._hold(ok)

    def _remember_platoon_route(self, ego: EgoState, partner: Optional["NearbyVehicle"]) -> None:
        """§20.4 공통 경로와 EXIT 시작 체크포인트를 결합 시점에 저장"""
        if partner is None:
            return
        common = []
        for a, b in zip(ego.route, partner.route):
            if a != b:
                break
            common.append(a)
        self.platoon_route = common
        if len(common) >= 2:
            self.exit_start_checkpoint = common[-2]  # 마지막보다 하나 앞선 체크포인트 (§21.1)
        elif common:
            self.exit_start_checkpoint = common[-1]

    def _exit_condition_met(self, ego: EgoState) -> bool:
        """§21.1 EXIT 진입 조건 — 남은 공통 체크포인트 수 기준"""
        if not self.platoon_route:
            return False
        if ego.checkpoint not in self.platoon_route:
            return False
        remaining = len(self.platoon_route) - 1 - self.platoon_route.index(ego.checkpoint)
        return remaining <= EXIT_REMAINING_CHECKPOINTS

    def _exit_complete(self, distance: Optional[float]) -> bool:
        """§21.4 EXIT 완료 조건 — 해제거리 이상이 1초 이상 유지"""
        if distance is None:
            return self._hold(False)
        # TODO: "차선 변경 가능 상태 = TRUE" 조건은 주행 알고리즘 연동 후 추가
        return self._hold(distance >= D_RELEASE_M)


if __name__ == "__main__":
    # 메인 루프 사용 예시 (실제로는 라인트레이싱 루프 안에서 매 주기 호출)
    class PrintComm(NullComm):
        """실제 시리얼 대신 전송되는 패킷을 콘솔에 찍어주는 데모용 스텁"""
        def send(self, packet) -> None:
            print(f"  [SEND] {packet}")

    fsm = PlatoonFSM(vehicle_id=1, comm=PrintComm())
    ego = EgoState(checkpoint=1, next_checkpoint=2, route=[2, 3, 4], speed=0.5, heading=0.0)
    candidate = NearbyVehicle(
        vehicle_id=2, checkpoint=1, next_checkpoint=2, route=[2, 3, 4],
        speed=0.5, heading=0.02, pdr=0.95, rssi=-50.0,
        uwb_distance=1.2, uwb_angle=0.1,
    )

    for i in range(12):
        cmd = fsm.update(ego, [candidate])
        print(f"cycle {i}: {fsm.state} / {fsm.match_state} / history={fsm._score_history} / {cmd}")