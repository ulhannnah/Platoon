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
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from .v2x_protocol import (
    PlatoonComm,
    NullComm,
    PlatoonJoinRequest,
    PlatoonJoinAccept,
    PlatoonJoinReject,
    PlatoonSetup,
    JoinComplete,
    EmergencyMessage,
    LeaveMessage,
)


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
    def __init__(self, vehicle_id: int = 0, comm: Optional[PlatoonComm] = None):
        """
        vehicle_id : 자차 고유 ID (V2X 패킷에 requester_id 등으로 실림)
                     TODO: 차량마다 실제 고유 ID 부여 방식 확정되면 교체
        comm       : ESP32와의 통신 인터페이스. 아직 시리얼 프로토콜이 없어서
                     기본값은 NullComm(아무것도 안 보내고 항상 빈 리스트 반환).
        """
        self.vehicle_id = vehicle_id
        self.comm: PlatoonComm = comm or NullComm()

        self.state = PlatoonState.SOLO_DRIVE
        self.match_state = MatchState.IDLE
        self.join_sub_state: Optional[JoinSubState] = None

        self.platoon_id: Optional[int] = None
        self.role: Optional[str] = None       # "LEADER" / "FOLLOWER"
        self.leader_id: Optional[int] = None
        self.partner_id: Optional[int] = None   # 플래툰 상대 차량(리더 또는 팔로워) ID
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

        플래툰 중에는 초음파에 리더가 잡히는 게 정상이므로, 그것만으로 장애물이라
        판정하면 안 된다. UWB로 잰 리더 거리보다 초음파가 유의미하게 가까울 때만
        "사이에 끼어든 것"으로 본다.
        """
        d = ego.front_distance
        if d is None:
            return None   # 초음파 값 없음 (미측정 또는 STM32 미연결)

        partner = self._find_partner(nearby) if self.partner_id else None
        in_platoon = (self.state != PlatoonState.SOLO_DRIVE
                      and partner is not None
                      and partner.uwb_distance is not None)

        if in_platoon:
            # 리더보다 가까운 무언가가 있는지
            if d >= partner.uwb_distance - CUTIN_MARGIN_M:
                return None   # 초음파가 본 게 리더 자신
        # 단독주행이거나, 끼어든 물체가 확인된 경우 → 순수 거리로 판정
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

        # 3. 다른 결합 절차가 진행 중인지 확인
        busy = (self.state != PlatoonState.SOLO_DRIVE
                or self.match_state in (MatchState.JOIN_REQUEST,
                                        MatchState.AWAIT_SETUP,
                                        MatchState.PLATOON_SETUP))
        if busy:
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

    # ── §10.3 PLATOON_SETUP 수신 → 역할·플래툰 ID 채택 ──────────────
    def _on_platoon_setup(self, pkt: PlatoonSetup) -> None:
        # 내가 참여 대상이 아니면 무시
        if self.vehicle_id not in (pkt.leader_id, pkt.follower_id):
            return
        # 이미 결합 완료된 상태면 무시 (중복 SETUP 방지)
        if self.state != PlatoonState.SOLO_DRIVE:
            return
        # 내가 보낸 SETUP이 되돌아온 경우는 무시
        if pkt.sender_id == self.vehicle_id:
            return

        self.platoon_id = pkt.platoon_id
        self.leader_id = pkt.leader_id
        if pkt.leader_id == self.vehicle_id:
            self.role = "LEADER"
            self.partner_id = pkt.follower_id
        else:
            self.role = "FOLLOWER"
            self.partner_id = pkt.leader_id

        self.match_state = MatchState.IDLE
        self.state = PlatoonState.PLATOON_JOIN
        self.join_sub_state = JoinSubState.JOIN_REQUESTED
        self._pending_candidate = None
        self._wait_since = None
        self._stable_since = None

    def _on_join_complete(self, pkt: JoinComplete) -> None:
        if pkt.platoon_id == self.platoon_id and pkt.sender_id == self.partner_id:
            self._peer_join_complete = True

    def _on_leave(self, pkt: LeaveMessage) -> None:
        """상대가 플래툰을 떠나면 자차도 해제 절차로 들어간다."""
        if pkt.platoon_id != self.platoon_id:
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
        앞/뒤 판단은 UWB 각도(전방 90도 이내면 상대가 앞쪽) 기준. 각도 정보가
        없으면 문서 우선순위 4번(차량 ID가 작은 쪽)으로 임시 대체한다.

        TODO: §18 ESP32에 PLATOON_HIGH_RATE 통신모드 전환 명령은 아직 미구현
              (전용 통신 채널 자체가 아직 없음)
        """
        candidate = self._pending_candidate
        if candidate is None:
            return

        if candidate.uwb_angle is not None:
            candidate_ahead = abs(candidate.uwb_angle) < (math.pi / 2)
        else:
            candidate_ahead = candidate.vehicle_id < self.vehicle_id  # TODO: 임시 대체 기준

        if candidate_ahead:
            self.role = "FOLLOWER"
            self.leader_id = candidate.vehicle_id
            follower_id = self.vehicle_id
        else:
            self.role = "LEADER"
            self.leader_id = self.vehicle_id
            follower_id = candidate.vehicle_id

        self.partner_id = candidate.vehicle_id
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
        플래툰 중에는 상대 위치를 모른 채로 간격 제어를 계속하면 안 된다.
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
