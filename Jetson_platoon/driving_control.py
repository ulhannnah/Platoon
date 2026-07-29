"""
driving_control.py
자율주행 제어 모듈 — "어떻게 움직일지" 담당 (Raspberry Pi 5, 빠른 루프)

FSM(platoon_fsm.py)이 결정한 DrivingCommand(목표값)를 받아서
실제 조향각/속도를 계산해 STM32로 보낼 값을 만든다.

역할 분담:
- SOLO_DRIVE, 또는 리더 역할            → 카메라 기반 차선추종 (기존 라인트레이싱 코드 연동 지점)
- PLATOON_JOIN/PLATOON_MAINTAIN + 팔로워 → UWB 상대위치 기반 pure pursuit
  (요구사항: 팔로워는 카메라를 쓰지 않고 ESP32가 UWB로 계산해준 값만으로 주행)

이 파일도 골격입니다. 카메라 라인트레이싱 부분은 이미 있는 팀 코드를 연결하는
자리(TODO)로 남겨뒀고, pure pursuit/팔로워 속도식은 앞서 정한 내용으로 채워져 있습니다.
"""

import math
from dataclasses import dataclass
from typing import Optional

from platoon_fsm import DrivingCommand


# ── 차량 제원 (실측값으로 교체) ────────────────────────────────────
WHEELBASE_M = 0.20                    # TODO: 실제 축간거리(m)로 교체
MAX_STEERING_RAD = math.radians(30)   # TODO: 실제 최대 조향각으로 교체


@dataclass
class ControlOutput:
    """STM32로 최종 전송할 값"""
    steering_rad: float = 0.0
    speed: float = 0.0


# ── 리더 / 단독주행: 카메라 기반 차선추종 ─────────────────────────
def camera_lane_tracing(frame) -> float:
    """
    기존 라인트레이싱 코드 연동 지점.
    카메라 프레임을 받아 차선 중앙 오프셋 기반 조향각(rad)을 반환한다고 가정.

    TODO: 이미 작성된 라인트레이싱 코드의 실제 함수 호출로 교체
    """
    return 0.0


# ── 팔로워: UWB 상대위치 기반 pure pursuit ────────────────────────
def pure_pursuit_steering(distance_to_leader: Optional[float],
                           angle_to_leader: Optional[float],
                           wheelbase: float = WHEELBASE_M) -> float:
    """
    distance_to_leader : ESP32가 UWB로 계산해서 넘겨주는 리더까지의 거리 (m)
    angle_to_leader    : ESP32가 UWB로 계산해서 넘겨주는 상대각 (rad, 팔로워 헤딩 기준)
    반환값              : 조향각 (rad)

    δ = atan2(2·L·sin(α), Ld)
    """
    if not distance_to_leader or distance_to_leader <= 0 or angle_to_leader is None:
        return 0.0
    steering = math.atan2(2 * wheelbase * math.sin(angle_to_leader), distance_to_leader)
    return max(-MAX_STEERING_RAD, min(MAX_STEERING_RAD, steering))


def follower_target_speed(distance_to_leader: Optional[float],
                           target_distance: Optional[float],
                           leader_speed: Optional[float],
                           current_speed: float,
                           kd: float = 0.4,
                           kv: float = 0.6) -> float:
    """
    팔로워 목표속도 계산 (design doc §20.2 개념 반영)
    거리오차 + 리더와의 속도차를 함께 보정
    """
    if distance_to_leader is None or target_distance is None or leader_speed is None:
        return current_speed
    distance_error = distance_to_leader - target_distance
    speed = leader_speed + kd * distance_error + kv * (leader_speed - current_speed)
    return max(0.0, speed)


# ── FSM 출력 + 센서값 → 최종 제어값 ────────────────────────────────
def compute_control(cmd: DrivingCommand,
                     role: Optional[str],
                     *,
                     camera_frame=None,
                     uwb_distance: Optional[float] = None,
                     uwb_angle: Optional[float] = None,
                     leader_speed: Optional[float] = None,
                     current_speed: float = 0.0) -> ControlOutput:
    """
    role: "LEADER" / "FOLLOWER" / None(SOLO)

    팔로워가 실제로 플래툰 추종 중일 때만 UWB pure pursuit를 쓰고,
    그 외(단독주행, 리더, 아직 매칭 전)는 전부 카메라를 씀.
    """
    is_following = (role == "FOLLOWER" and cmd.mode in ("PLATOON_JOIN", "PLATOON_MAINTAIN"))

    if is_following:
        steering = pure_pursuit_steering(uwb_distance, uwb_angle)
        speed = follower_target_speed(
            distance_to_leader=uwb_distance,
            target_distance=cmd.target_distance,
            leader_speed=leader_speed,
            current_speed=current_speed,
        )
    else:
        steering = camera_lane_tracing(camera_frame)
        speed = cmd.target_speed if cmd.target_speed is not None else current_speed

    return ControlOutput(steering_rad=steering, speed=speed)