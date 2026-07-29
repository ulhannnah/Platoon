"""
driving_control.py
자율주행 제어 모듈 — "어떻게 움직일지" 담당 (Raspberry Pi 5, 빠른 루프)

FSM(platoon_fsm.py)이 결정한 DrivingCommand(목표값)를 받아서
STM32로 보낼 목표 조향각/목표 속도를 계산한다.

역할 경계:
- 입력  UWB 거리(m)·각도(rad)는 ESP32-S3가 이미 연산을 끝낸 값을 그대로 받는다.
        (원시 UWB 신호 처리는 ESP32 담당, 여기서 하지 않음)
- 출력  목표 조향각(rad)과 목표 속도(m/s)까지만 계산한다.
        PWM 듀티·모터 PID·아커만 기하 보정은 STM32 담당, 여기서 하지 않음.

주행 방식 분기:
- SOLO_DRIVE, 또는 리더 역할            → 카메라 기반 차선추종 (기존 라인트레이싱 코드 연동 지점)
- PLATOON_JOIN/PLATOON_MAINTAIN + 팔로워 → UWB 상대위치 기반 pure pursuit
  (요구사항: 팔로워는 카메라를 쓰지 않고 리더로부터의 V2V 정보만으로 주행)

이 파일도 골격입니다. 카메라 라인트레이싱 부분은 이미 있는 팀 코드를 연결하는
자리(TODO)로 남겨뒀고, pure pursuit/팔로워 속도식은 앞서 정한 내용으로 채워져 있습니다.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

from platoon_fsm import DrivingCommand


# ── 차량 제원 (실측값으로 교체) ────────────────────────────────────
WHEELBASE_M = 0.20                    # TODO: 실제 축간거리(m)로 교체
MAX_STEERING_RAD = math.radians(30)   # TODO: 실제 최대 조향각으로 교체

# ── 차선추종 PID 게인 (실차 튜닝 필요) ─────────────────────────────
# 주의: D항은 1/dt로 증폭된다. 50Hz(dt=0.02)면 오차 변화가 50배로 곱해지므로
#       KD는 KP보다 훨씬 작게 잡아야 조향이 포화되지 않는다.
LANE_KP = 0.5     # TODO: 실차 튜닝 — 반응이 굼뜨면 올리기
LANE_KI = 0.0     # TODO: 정상상태 오차가 남으면 조금씩 올리기
LANE_KD = 0.0     # TODO: P만으로 시작. 코너 반응이 굼뜨면 0.005부터 조금씩 올리기
                  #       (오프셋 노이즈가 1/dt로 증폭되므로 크게 잡으면 조향이 떨림)
LANE_I_LIMIT = 0.5           # 적분 와인드업 방지
LANE_LOST_TIMEOUT_S = 0.3    # 라인 유실 후 이 시간 넘으면 정지


@dataclass
class LaneTracingResult:
    """
    라인트레이싱 모듈에서 매 프레임 받아오는 결과.

    offset   : 정규화된 차선 중앙 오프셋 (-1.0 ~ +1.0)
               = (라인중심_x - 이미지중심_x) / (이미지폭 / 2)
               + 값이면 라인이 오른쪽에 있음 → 오른쪽으로 조향해야 함
               정규화해서 받아야 해상도가 바뀌어도 게인을 다시 안 잡음
    detected : 이번 프레임에서 라인을 찾았는지.
               False가 이어지면 마지막 조향각을 계속 쓰지 않고 정지한다.
    """
    offset: float = 0.0
    detected: bool = False


@dataclass
class ControlOutput:
    """
    STM32로 전송할 목표값 — "얼만큼 갈지"까지만 담는다.

    steering_rad : 목표 조향각 (rad). STM32가 서보 PWM으로 변환.
    speed        : 목표 속도 (m/s).   STM32가 모터 PID로 추종.

    PWM 듀티, 모터 PID 게인, 아커만 기하 보정은 STM32 담당이므로
    이 값에 포함하지 않는다.
    """
    steering_rad: float = 0.0
    speed: float = 0.0
    lane_lost: bool = False   # 라인 유실로 정지 중인지 (상위 판단/로깅용)


# ── 리더 / 단독주행: 카메라 기반 차선추종 ─────────────────────────
class LaneFollower:
    """
    라인트레이싱 모듈이 준 오프셋을 조향각(rad)으로 바꾸는 PID.

    라인 검출(카메라 → 오프셋)은 팀원의 라인트레이싱 코드 담당이고,
    오프셋 → 조향각 변환은 제어 계층인 여기 담당이다.
    """

    def __init__(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._lost_since: Optional[float] = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._lost_since = None

    def update(self, lane: Optional[LaneTracingResult], dt: float):
        """
        반환값: (steering_rad, lane_lost)
        lane_lost가 True면 호출부는 속도를 0으로 낮춰야 한다.
        """
        now = time.time()

        # 라인을 놓친 경우: 짧게는 직전 조향을 유지하되, 일정 시간 넘으면 정지.
        # 여기서 _lost_since를 지우면 안 된다 — 지우면 타이머가 매번 다시 시작돼서
        # "정지 → 옛날 조향각으로 재출발 → 정지"를 반복한다.
        if lane is None or not lane.detected:
            if self._lost_since is None:
                self._lost_since = now
            if (now - self._lost_since) >= LANE_LOST_TIMEOUT_S:
                self._integral = 0.0
                self._prev_error = 0.0
                self._steering = 0.0
                return 0.0, True
            return self._last_steering(), False

        # 라인을 다시 찾았을 때만 유실 상태 해제
        if self._lost_since is not None:
            self._lost_since = None
            self._integral = 0.0
            self._prev_error = lane.offset   # 재진입 시 D항 튐 방지

        error = lane.offset
        self._integral = max(-LANE_I_LIMIT,
                             min(LANE_I_LIMIT, self._integral + error * dt))
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        steering = LANE_KP * error + LANE_KI * self._integral + LANE_KD * derivative
        steering = max(-MAX_STEERING_RAD, min(MAX_STEERING_RAD, steering))
        self._steering = steering
        return steering, False

    def _last_steering(self) -> float:
        return getattr(self, "_steering", 0.0)


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


# 팔로워 목표속도 계산은 platoon_fsm.py가 담당한다.
# (JOIN §19.5 _join_target_speed / MAINTAIN §20.2 / EXIT §21.3)
# 같은 이득(K_D, K_V)을 두 파일에 두면 한쪽만 튜닝했을 때
# FSM 판단과 실제 제어가 어긋나므로 여기서는 계산하지 않는다.


# ── FSM 출력 + 센서값 → 최종 제어값 ────────────────────────────────
# 팔로워가 UWB 추종을 쓰는 구간. EXIT까지 포함하는 이유:
#   해제 중에 조향 기준을 UWB→카메라로 바꾸면, 거리를 벌리는 도중에
#   라인을 아직 못 잡은 상태로 전환될 수 있다(뒤차가 있으면 위험).
#   완전히 분리되어 SOLO_DRIVE로 돌아갈 때 한 번에 전환한다.
FOLLOWER_UWB_MODES = ("PLATOON_JOIN", "PLATOON_MAINTAIN", "PLATOON_EXIT")


def compute_control(cmd: DrivingCommand,
                     role: Optional[str],
                     lane_follower: "LaneFollower",
                     *,
                     lane: Optional[LaneTracingResult] = None,
                     dt: float = 0.02,
                     uwb_distance: Optional[float] = None,
                     uwb_angle: Optional[float] = None,
                     current_speed: float = 0.0) -> ControlOutput:
    """
    role: "LEADER" / "FOLLOWER" / None(SOLO)

    목표 속도는 FSM이 이미 계산해서 cmd.target_speed로 내려준다
    (JOIN §19.5 / MAINTAIN §20.2 / EXIT §21.3). 여기서 다시 계산하지 않는다.
    이 파일이 정하는 것은 조향각을 어느 센서로 만들 것인가뿐이다.
    """
    target_speed = cmd.target_speed if cmd.target_speed is not None else current_speed

    if role == "FOLLOWER" and cmd.mode in FOLLOWER_UWB_MODES:
        # 팔로워는 카메라를 쓰지 않고 V2V/UWB만으로 주행
        steering = pure_pursuit_steering(uwb_distance, uwb_angle)
        return ControlOutput(steering_rad=steering, speed=target_speed)

    steering, lane_lost = lane_follower.update(lane, dt)
    if lane_lost:
        # 라인을 놓친 채로 계속 달리면 이탈한다 — 정지
        return ControlOutput(steering_rad=0.0, speed=0.0, lane_lost=True)

    return ControlOutput(steering_rad=steering, speed=target_speed)