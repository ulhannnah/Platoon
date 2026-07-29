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
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from v2x_protocol import (
    PlatoonComm,
    NullComm,
    PlatoonJoinRequest,
    PlatoonJoinAccept,
    PlatoonJoinReject,
    PlatoonSetup,
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
    JOIN_REQUEST = auto()
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
    obstacle: bool = False
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
        self.candidate_id: Optional[int] = None
        self._pending_candidate: Optional[NearbyVehicle] = None  # JOIN_REQUEST 보낸 상대 캐시
        self._platoon_seq = 1  # TODO: 여러 대 동시 결합 시 전역 고유성 보장 필요 (지금은 단순 카운터)

        # vehicle_id -> 최근 평가 결과(bool) 리스트, 최대 STABILITY_WINDOW개 (§8)
        self._score_history: dict = {}

    # ── 메인 루프에서 매 주기 호출 ──────────────────────────────────
    def update(self, ego: EgoState, nearby: list) -> DrivingCommand:
        if self.state == PlatoonState.SOLO_DRIVE:
            return self._run_solo_drive(ego, nearby)
        elif self.state == PlatoonState.PLATOON_JOIN:
            return self._run_platoon_join(ego, nearby)
        elif self.state == PlatoonState.PLATOON_MAINTAIN:
            return self._run_platoon_maintain(ego, nearby)
        elif self.state == PlatoonState.PLATOON_EXIT:
            return self._run_platoon_exit(ego, nearby)
        raise RuntimeError(f"unknown state: {self.state}")

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
            # JOIN_ACCEPT / JOIN_REJECT 수신 대기 (실제 시리얼 연결 전까지는 NullComm이라 항상 None)
            accepted = self._check_join_response()
            if accepted is True:
                self.match_state = MatchState.PLATOON_SETUP
            elif accepted is False:
                self.candidate_id = None
                self.match_state = MatchState.V2X_SEARCH

        elif self.match_state == MatchState.PLATOON_SETUP:
            # TODO: §10.3 리더/팔로워 역할 결정 + 플래툰 ID 생성/공유
            #       ESP32에 PLATOON_HIGH_RATE 모드 전환 명령 (§18)
            self._setup_platoon()
            self.match_state = MatchState.IDLE
            self.state = PlatoonState.PLATOON_JOIN
            self.join_sub_state = JoinSubState.JOIN_REQUESTED

        return DrivingCommand(mode="SOLO_DRIVE")

    # ── PLATOON_JOIN 세부 단계 (§19) ────────────────────────────────
    def _run_platoon_join(self, ego: EgoState, nearby: list) -> DrivingCommand:
        if self.join_sub_state == JoinSubState.JOIN_REQUESTED:
            # TODO: 모든 차량 설정 확인되면 다음 단계
            self.join_sub_state = JoinSubState.JOIN_LANE_ALIGN

        elif self.join_sub_state == JoinSubState.JOIN_LANE_ALIGN:
            # TODO: §19.4 차선 정렬 완료조건 확인
            if self._lane_align_complete(ego):
                self.join_sub_state = JoinSubState.JOIN_GAP_CONTROL

        elif self.join_sub_state == JoinSubState.JOIN_GAP_CONTROL:
            # TODO: §19.5 거리 오차 기반 속도 제어
            if self._gap_control_complete(ego):
                self.join_sub_state = JoinSubState.JOIN_STABILIZE

        elif self.join_sub_state == JoinSubState.JOIN_STABILIZE:
            # TODO: §11.3 결합 완료조건 (거리오차/속도오차/유지시간)
            if self._join_stabilized(ego):
                self.join_sub_state = None
                self.state = PlatoonState.PLATOON_MAINTAIN

        return DrivingCommand(
            mode="PLATOON_JOIN",
            leader_id=self.leader_id,
            # TODO: target_lane / target_speed / target_distance 채우기
        )

    # ── PLATOON_MAINTAIN (§20) ──────────────────────────────────────
    def _run_platoon_maintain(self, ego: EgoState, nearby: list) -> DrivingCommand:
        # TODO: §20.2 목표속도 계산 v_target = v_L + Kd(d-d_target) + Kv(vL-vF)
        # TODO: §20.4~20.6 EXIT 조건 체크, 신규 차량 합류 처리
        if self._exit_condition_met(ego):
            self.state = PlatoonState.PLATOON_EXIT

        return DrivingCommand(
            mode="PLATOON_MAINTAIN",
            leader_id=self.leader_id,
        )

    # ── PLATOON_EXIT (§21) ──────────────────────────────────────────
    def _run_platoon_exit(self, ego: EgoState, nearby: list) -> DrivingCommand:
        # TODO: §21.3 간격 확대 제어, §21.4 완료조건
        if self._exit_complete(ego):
            self.state = PlatoonState.SOLO_DRIVE
            self.platoon_id = None
            self.role = None
            self.leader_id = None

        return DrivingCommand(mode="PLATOON_EXIT")

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
    # §10 플래툰 결합 통신 절차 (JOIN_REQUEST → ACCEPT/REJECT → SETUP)
    # ══════════════════════════════════════════════════════════════
    def _send_join_request(self, ego: EgoState, candidate: "NearbyVehicle", score: float) -> None:
        """§10.1 — 결합 요청 전송. 응답 대기 중 candidate 정보를 캐시해둔다."""
        self._pending_candidate = candidate
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
        §10.2 — JOIN_ACCEPT / JOIN_REJECT 수신 확인.
        반환값: True(승인) / False(거절) / None(아직 응답 없음)
        """
        if self._pending_candidate is None:
            return None

        for packet in self.comm.poll():
            if isinstance(packet, PlatoonJoinAccept):
                if packet.requester_id == self.vehicle_id and packet.responder_id == self.candidate_id:
                    return True
            elif isinstance(packet, PlatoonJoinReject):
                if packet.requester_id == self.vehicle_id and packet.responder_id == self.candidate_id:
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

    def _lane_align_complete(self, ego):
        return False

    def _gap_control_complete(self, ego):
        return False

    def _join_stabilized(self, ego):
        return False

    def _exit_condition_met(self, ego):
        return False

    def _exit_complete(self, ego):
        return False


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