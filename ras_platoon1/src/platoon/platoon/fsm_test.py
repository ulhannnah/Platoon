"""
platoon_fsm.py
UWB를 제거하고 초음파(거리) + V2X(상태/차선/속도) + 카메라(조향) 기반으로 
완전히 새로 작성된 경량화 플래툰 FSM 골격 코드.
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

# 기존 V2X 프로토콜 임포트 (형식 유지)
from .v2x_protocol import (
    PlatoonComm, NullComm, PlatoonJoinRequest, PlatoonJoinAccept,
    PlatoonJoinReject, PlatoonSetup, JoinComplete, EmergencyMessage, LeaveMessage
)

# ── 최상위 플래툰 상태 ───────────────────────────────────────────
class PlatoonState(Enum):
    SOLO_DRIVE = auto()
    PLATOON_JOIN = auto()
    PLATOON_MAINTAIN = auto()
    PLATOON_EXIT = auto()

# ── 파라미터 설정 ────────────────────────────────────────────────
TARGET_DISTANCE_M = 0.4        # 목표 차간거리 (예: 40cm)
DISTANCE_TOLERANCE_M = 0.05    # 거리 허용 오차 (±5cm)
SPEED_TOLERANCE = 0.1          # 속도 허용 오차
STABLE_TIME_S = 1.0            # 안정화 판단 시간 (1초)

OBSTACLE_STOP_DISTANCE_M = 0.15 # 긴급 정지 거리 (15cm 이내면 무조건 정지)

K_D = 0.8  # 거리 오차 제어 게인 (초음파 기반이므로 응답성을 높임)
K_V = 0.5  # 상대 속도 제어 게인

V_MIN, V_MAX = 0.0, 1.0


@dataclass
class EgoState:
    lane: int = 1
    speed: float = 0.0
    front_distance: Optional[float] = None  # 초음파 거리 센서 값 (핵심)
    checkpoint: int = 0
    destination: int = 0
    emergency: bool = False


@dataclass
class NearbyVehicle:
    vehicle_id: int
    lane: int = 1
    speed: float = 0.0
    platoon_state: str = "SOLO"
    platoon_id: Optional[int] = None
    leader_id: Optional[int] = None
    emergency: bool = False
    timestamp: float = 0.0


@dataclass
class DrivingCommand:
    mode: str = "SOLO_DRIVE"
    target_lane: Optional[int] = None
    target_speed: Optional[float] = None
    emergency: bool = False


class PlatoonFSM:
    def __init__(self, vehicle_id: int, is_designated_leader: bool, comm: PlatoonComm):
        self.vehicle_id = vehicle_id
        self.is_designated_leader = is_designated_leader  # True: 젯슨, False: 라즈베리파이
        self.comm = comm

        self.state = PlatoonState.SOLO_DRIVE
        
        self.platoon_id: Optional[int] = None
        self.leader_id: Optional[int] = None
        self.partner_id: Optional[int] = None

        self._stable_since: Optional[float] = None
        self._last_target_speed: float = 0.0

    def update(self, ego: EgoState, nearby: list) -> DrivingCommand:
        self._handle_incoming()

        # 1. 안전 최우선 검사 (초음파 기반)
        if ego.emergency or (ego.front_distance is not None and ego.front_distance <= OBSTACLE_STOP_DISTANCE_M):
            return self._trigger_emergency("전방 초근접 위험 또는 시스템 긴급 정지")

        # 2. 상태별 주행 명령 산출
        if self.state == PlatoonState.SOLO_DRIVE:
            cmd = self._run_solo_drive(ego, nearby)
        elif self.state == PlatoonState.PLATOON_JOIN:
            cmd = self._run_platoon_join(ego, nearby)
        elif self.state == PlatoonState.PLATOON_MAINTAIN:
            cmd = self._run_platoon_maintain(ego, nearby)
        elif self.state == PlatoonState.PLATOON_EXIT:
            cmd = self._run_platoon_exit(ego)
        else:
            cmd = DrivingCommand(mode="SOLO_DRIVE")

        return cmd

    # ── [1] 안전 제어 ────────────────────────────────────────────────
    def _trigger_emergency(self, reason: str) -> DrivingCommand:
        if self.state != PlatoonState.SOLO_DRIVE:
            self.comm.send(EmergencyMessage(
                sender_id=self.vehicle_id, platoon_id=self.platoon_id or 0, timestamp=time.time()
            ))
            self._reset_platoon()
        return DrivingCommand(mode="SOLO_DRIVE", target_speed=0.0, emergency=True)

    # ── [2] 단독 주행 및 결합 시작 (SOLO_DRIVE) ──────────────────────
    def _run_solo_drive(self, ego: EgoState, nearby: list) -> DrivingCommand:
        # V2X 패킷(SETUP 등)을 받으면 상태가 JOIN으로 넘어가도록 통신 콜백에서 처리됩니다.
        # 여기서는 V2X 목록을 보고 조건이 맞으면 통신 모듈을 통해 결합 요청(JoinRequest)을 보냅니다.
        
        if not self.is_designated_leader:
            leader = next((v for v in nearby if v.platoon_state in ["SOLO", "MAINTAIN"] and not v.emergency), None)
            if leader:
                # TODO: V2X 핸드셰이크 트리거 (요청 전송)
                pass 

        return DrivingCommand(mode="SOLO_DRIVE")

    # ── [3] 군집 결합 제어 (PLATOON_JOIN) ────────────────────────────
    def _run_platoon_join(self, ego: EgoState, nearby: list) -> DrivingCommand:
        partner = self._find_partner(nearby)
        if not partner:
            return DrivingCommand(mode="SOLO_DRIVE")

        target_lane = partner.lane if not self.is_designated_leader else ego.lane
        target_speed = ego.speed  # 기본값

        # 같은 차선에 진입했고, 초음파 거리가 측정되고 있다면 거리 조절 시작
        if ego.lane == partner.lane and ego.front_distance is not None:
            distance_error = ego.front_distance - TARGET_DISTANCE_M
            
            # P 제어로 앞차와의 거리를 좁히거나 벌림
            if not self.is_designated_leader:
                target_speed = partner.speed + (K_D * distance_error)
            
            # 안정화 조건: 거리가 목표치에 근접하고 속도 차이가 적을 때
            is_stable = (abs(distance_error) <= DISTANCE_TOLERANCE_M and 
                         abs(partner.speed - ego.speed) <= SPEED_TOLERANCE)
                         
            if self._hold_condition(is_stable, STABLE_TIME_S):
                self.state = PlatoonState.PLATOON_MAINTAIN
                self._stable_since = None
                self.comm.send(JoinComplete(sender_id=self.vehicle_id, platoon_id=self.platoon_id, timestamp=time.time()))

        return DrivingCommand(
            mode="PLATOON_JOIN",
            target_lane=target_lane,
            target_speed=self._limit_speed(target_speed)
        )

    # ── [4] 군집 주행 유지 (PLATOON_MAINTAIN) ────────────────────────
    def _run_platoon_maintain(self, ego: EgoState, nearby: list) -> DrivingCommand:
        partner = self._find_partner(nearby)
        
        # 앞차 소실 또는 V2X 정보 두절 시 이탈
        if not partner or ego.front_distance is None:
            self.state = PlatoonState.SOLO_DRIVE
            self._reset_platoon()
            return DrivingCommand(mode="SOLO_DRIVE")

        # 목적지 도달 시 해제
        if ego.checkpoint >= ego.destination:
            self.state = PlatoonState.PLATOON_EXIT
            return self._run_platoon_exit(ego)

        target_lane = partner.lane if not self.is_designated_leader else ego.lane
        target_speed = None  # 리더는 None으로 두어 하위 크루즈 속도 사용
        
        # CACC (협력 적응형 크루즈 컨트롤) 수식 적용
        if not self.is_designated_leader:
            distance_error = ego.front_distance - TARGET_DISTANCE_M
            target_speed = partner.speed + (K_D * distance_error) + (K_V * (partner.speed - ego.speed))
            target_speed = self._limit_speed(target_speed)

        return DrivingCommand(
            mode="PLATOON_MAINTAIN",
            target_lane=target_lane,
            target_speed=target_speed
        )

    # ── [5] 군집 해제 (PLATOON_EXIT) ─────────────────────────────────
    def _run_platoon_exit(self, ego: EgoState) -> DrivingCommand:
        self.comm.send(LeaveMessage(sender_id=self.vehicle_id, platoon_id=self.platoon_id, timestamp=time.time()))
        self._reset_platoon()
        return DrivingCommand(mode="SOLO_DRIVE")

    # ── 유틸리티 함수 ────────────────────────────────────────────────
    def _find_partner(self, nearby: list) -> Optional[NearbyVehicle]:
        return next((v for v in nearby if v.vehicle_id == self.partner_id), None)

    def _hold_condition(self, condition: bool, duration: float) -> bool:
        if not condition:
            self._stable_since = None
            return False
        now = time.time()
        if self._stable_since is None:
            self._stable_since = now
            return False
        return (now - self._stable_since) >= duration

    def _limit_speed(self, speed: float) -> float:
        return max(V_MIN, min(V_MAX, speed))
        
    def _reset_platoon(self):
        self.state = PlatoonState.SOLO_DRIVE
        self.platoon_id = None
        self.leader_id = None
        self.partner_id = None
        self._stable_since = None

    def _handle_incoming(self):
        # TODO: self.comm.poll() 을 통해 들어온 V2X 패킷(Request, Accept, Setup)을 
        # 확인하고 self.state, self.partner_id 등을 설정하는 로직
        pass