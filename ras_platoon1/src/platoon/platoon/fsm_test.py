"""
fsm_test.py
촬영용 스크립트 시나리오 전용 경량 플래툰 FSM.

platoon_fsm.py(적합도 판단·후보 탐색·핸드셰이크 패킷 왕복)를 전부 걷어내고,
- 리더/파트너 관계는 launch 파라미터로 미리 고정 (탐색 없음)
- 상태 전이는 외부 명령(JOIN/EXIT/EXIT_TOGETHER)으로 트리거
- 차선변경은 decision_node의 기존 request_lane_change 서비스를 "요청"만 함
  (조향각 계산은 절대 여기서 하지 않음 — decision_node가 전담)
- 거리 제어는 UWB 없이 초음파(ego.front_distance)만 사용, 연속값 대신
  decision_node의 기존 speed_mode 3단계(0/1/2)를 그대로 재사용

로 완전히 다시 짠 버전. ROS/시리얼 의존성 없는 순수 로직 (fsm_decision_node.py가
센서·통신과 이어준다).
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class PlatoonState(Enum):
    SOLO_DRIVE = auto()
    PLATOON_JOIN = auto()
    PLATOON_MAINTAIN = auto()
    PLATOON_EXIT = auto()
    CONVOY_EXIT = auto()   # §13 — 대형(플래툰) 유지한 채 차선변경 후 도로 이탈


# ── 파라미터 (실측 전 임시값 — docs/parameters.md 참고) ───────────────
TARGET_DISTANCE_M = 0.4          # 목표 차간거리
GAP_HYSTERESIS_M = 0.08          # 이 폭 안에서는 이전 속도단계 유지 (떨림 방지)
OBSTACLE_STOP_DISTANCE_M = 0.15  # 전방 초근접 시 무조건 비상정지

# decision_node.py의 speed_mode와 동일한 의미 (0: 정지, 1: 저속, 2: 크루즈)
SPEED_STOP = 0
SPEED_SLOW = 1
SPEED_CRUISE = 2


@dataclass
class EgoState:
    lane: int = 1                              # 현재 차선 (lane_change_done 이벤트로만 갱신)
    front_distance: Optional[float] = None     # 초음파 전방거리(m)
    emergency: bool = False


@dataclass
class NearbyVehicle:
    vehicle_id: int
    platoon_state: str = "SOLO"    # "SOLO" / "JOIN" / "MAINTAIN" / "EXIT"
    emergency: bool = False
    speed_level: int = SPEED_CRUISE  # 이 차량이 decision_node에서 지금 실제로 내고 있는 speed_mode(0/1/2)
    timestamp: float = 0.0


@dataclass
class DrivingCommand:
    mode: str = "SOLO_DRIVE"
    # "left"/"right" — 이번 주기에 1회 차선변경 요청 (아니면 None)
    request_lane_change_dir: Optional[str] = None
    # decision_node의 speed_mode를 이 값으로 덮어씀. None이면 decision_node
    # 자체 기본값(cruise) 사용 — 리더는 항상 None (§7, 그냥 SOLO_DRIVE처럼 주행)
    speed_level: Optional[int] = None
    emergency: bool = False


class PlatoonFSM:
    def __init__(self, vehicle_id: int, is_designated_leader: bool,
                 leader_id: Optional[int] = None,
                 initial_partner_id: Optional[int] = None,
                 expected_follower_ids: Optional[list] = None):
        """
        leader_id           : 팔로워만 사용. 전체 리더 ID (§12 재연결 시 폴백 대상).
        initial_partner_id  : 팔로워만 사용. JOIN 시 바로 붙을 "내 앞차" ID.
                               지정 안 하면 leader_id와 동일(=리더 바로 뒤).
                               3대 이상 체인의 맨 뒤 차량은 바로 앞 팔로워 ID를 줘야 함.
        expected_follower_ids: 리더만 사용. 뒤에 붙을 것으로 예정된 차량 ID 목록.
                               순서대로 하나씩 소진하며 자동으로 successor로 인식.
        """
        self.vehicle_id = vehicle_id
        self.is_designated_leader = is_designated_leader
        self.leader_id = leader_id
        self._expected_followers = list(expected_follower_ids or [])

        self.state = PlatoonState.SOLO_DRIVE
        self.partner_id: Optional[int] = initial_partner_id if initial_partner_id is not None else leader_id
        self.successor_id: Optional[int] = None  # 리더 전용 — 참고/디버그용

        self._pending_lane_dir: Optional[str] = None
        self._lane_change_pending = False   # True인 동안은 아직 목표차선 도달 전
        self._last_speed_level: int = SPEED_CRUISE

    # ══════════════════════════════════════════════════════════════
    # 외부 명령 — fsm_decision_node가 명령 토픽 수신 시 호출
    # ══════════════════════════════════════════════════════════════
    def handle_command(self, cmd: str, target_lane: int, current_lane: int) -> None:
        cmd = cmd.upper()

        if cmd == "JOIN" and self.state == PlatoonState.SOLO_DRIVE and not self.is_designated_leader:
            self.state = PlatoonState.PLATOON_JOIN
            self._start_lane_change(target_lane, current_lane)

        elif cmd == "EXIT" and self.state == PlatoonState.PLATOON_MAINTAIN:
            self.state = PlatoonState.PLATOON_EXIT
            self._start_lane_change(target_lane, current_lane)

        elif cmd == "EXIT_TOGETHER" and self.state == PlatoonState.PLATOON_MAINTAIN:
            # §13 — 플래툰(파트너 연결) 유지한 채로 차선변경만 수행
            self.state = PlatoonState.CONVOY_EXIT
            self._start_lane_change(target_lane, current_lane)

    def _start_lane_change(self, target_lane: int, current_lane: int) -> None:
        if target_lane == current_lane:
            return  # 이미 목표 차선
        # 차선번호 규약: 1=안쪽(왼쪽), 2=바깥쪽(오른쪽). 실제 도로 배치가 반대면
        # 이 한 줄만 뒤집으면 된다.
        self._pending_lane_dir = "left" if target_lane < current_lane else "right"
        self._lane_change_pending = True

    def notify_lane_change_done(self) -> None:
        """decision_node의 lane_change_done 이벤트 수신 시 fsm_decision_node가 호출."""
        self._lane_change_pending = False

    # ══════════════════════════════════════════════════════════════
    # 리더 전용 — 팔로워가 JOIN 방송 중이면 자동으로 받아들여 상태만 맞춰줌
    # (조향/속도에는 영향 없음 — §7 리더는 항상 그냥 주행)
    # ══════════════════════════════════════════════════════════════
    def _leader_track_follower(self, nearby: list) -> None:
        if self.successor_id is None:
            for v in nearby:
                if v.vehicle_id in self._expected_followers and v.platoon_state == "JOIN":
                    self.successor_id = v.vehicle_id
                    self._expected_followers.remove(v.vehicle_id)
                    if self.state == PlatoonState.SOLO_DRIVE:
                        self.state = PlatoonState.PLATOON_JOIN
                    break

        if self.state == PlatoonState.PLATOON_JOIN and self.successor_id is not None:
            succ = self._find(nearby, self.successor_id)
            if succ is not None and succ.platoon_state == "MAINTAIN":
                self.state = PlatoonState.PLATOON_MAINTAIN  # §6

    # ══════════════════════════════════════════════════════════════
    def update(self, ego: EgoState, nearby: list) -> DrivingCommand:
        # 1. 안전 최우선.
        # §플래툰 전체 즉시 전파 — 초음파로 내 앞차가 서는 걸 감지할 때까지
        # 기다리지 않고, 누구든(리더 포함) V2X로 emergency를 방송하는 순간
        # 나도 같이 선다. 안 그러면 맨 뒤차까지 정지가 순차적으로(체인 지연)
        # 전달되어 플래툰으로 비상정지하는 이점이 사라진다.
        peer_emergency = any(v.emergency for v in nearby)
        if ego.emergency or peer_emergency or (
                ego.front_distance is not None
                and ego.front_distance <= OBSTACLE_STOP_DISTANCE_M):
            return DrivingCommand(mode="EMERGENCY", speed_level=SPEED_STOP, emergency=True)

        if self.is_designated_leader:
            self._leader_track_follower(nearby)

        # 2. §12 — 내 앞차(partner)가 리더가 아닌데 사라지거나 이탈했으면
        #    리더에게 직접 재연결 (중간 차량 이탈 대응)
        if (not self.is_designated_leader
                and self.state in (PlatoonState.PLATOON_MAINTAIN, PlatoonState.CONVOY_EXIT)
                and self.partner_id is not None and self.partner_id != self.leader_id):
            partner = self._find(nearby, self.partner_id)
            if partner is None or partner.platoon_state in ("SOLO", "EXIT"):
                self.partner_id = self.leader_id

        # 3. 차선변경 요청은 딱 한 주기만 내보낸다
        lane_dir = self._pending_lane_dir
        self._pending_lane_dir = None

        # 4. 상태별 처리
        if self.state == PlatoonState.SOLO_DRIVE:
            cmd = DrivingCommand(mode="SOLO_DRIVE")

        elif self.state == PlatoonState.PLATOON_JOIN:
            if not self._lane_change_pending:
                self.state = PlatoonState.PLATOON_MAINTAIN  # §5 차선변경 완료 → MAINTAIN
            cmd = DrivingCommand(mode="PLATOON_JOIN")

        elif self.state == PlatoonState.PLATOON_MAINTAIN:
            cmd = self._run_maintain(ego, nearby, mode="PLATOON_MAINTAIN")

        elif self.state == PlatoonState.PLATOON_EXIT:
            if not self._lane_change_pending:
                self._reset()
                cmd = DrivingCommand(mode="SOLO_DRIVE")
            else:
                cmd = DrivingCommand(mode="PLATOON_EXIT")

        elif self.state == PlatoonState.CONVOY_EXIT:
            cmd = self._run_maintain(ego, nearby, mode="CONVOY_EXIT")
            if not self._lane_change_pending:
                self.state = PlatoonState.PLATOON_MAINTAIN  # 차선변경만 끝나면 MAINTAIN 복귀(연결 유지)

        else:
            cmd = DrivingCommand(mode="SOLO_DRIVE")

        cmd.request_lane_change_dir = lane_dir
        return cmd

    def _run_maintain(self, ego: EgoState, nearby: list, mode: str) -> DrivingCommand:
        if self.is_designated_leader:
            return DrivingCommand(mode=mode, speed_level=None)  # §7

        # §8 — CACC를 3단계로 흉내: 앞차(partner)가 "지금 실제로 내고 있는"
        # speed_mode를 피드포워드 기준값으로 삼고, 초음파 거리오차로 ±1단계만
        # 보정한다. 순수 거리 히스테리시스보다 앞차 속도 변화에 더 빨리 반응함
        # (거리가 실제로 벌어지길 기다리지 않아도 됨).
        partner = self._find(nearby, self.partner_id)
        base = partner.speed_level if partner is not None else self._last_speed_level

        d = ego.front_distance
        if d is None:
            level = base
        elif d < TARGET_DISTANCE_M - GAP_HYSTERESIS_M:
            level = max(SPEED_STOP, base - 1)      # 너무 가까움 — 앞차보다 한 단계 감속
        elif d > TARGET_DISTANCE_M + GAP_HYSTERESIS_M:
            level = min(SPEED_CRUISE, base + 1)    # 너무 멂 — 한 단계 더 가속해서 따라잡기
        else:
            level = base                           # 적당함 — 앞차와 같은 속도단계 유지

        self._last_speed_level = level
        return DrivingCommand(mode=mode, speed_level=level)

    def _find(self, nearby: list, vid: Optional[int]) -> Optional[NearbyVehicle]:
        if vid is None:
            return None
        return next((v for v in nearby if v.vehicle_id == vid), None)

    def _reset(self) -> None:
        self.state = PlatoonState.SOLO_DRIVE
        self.partner_id = self.leader_id
        self._pending_lane_dir = None
        self._lane_change_pending = False
