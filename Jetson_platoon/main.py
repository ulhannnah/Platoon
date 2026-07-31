"""
main.py
전체 메인 루프 — 유일한 while문이 여기 있습니다. FSM 자체는 루프를 안 돌고,
이 while문이 매 사이클마다 fsm.update()를 "한 번씩" 호출하는 구조입니다.

═══════════════════════════════════════════════════════════════════
3계층 역할 경계 (이 경계를 넘는 코드는 여기 들어오면 안 됨)
═══════════════════════════════════════════════════════════════════

[ESP32-S3 + UWB]  ← 이 프로젝트 코드 아님
  - UWB 원시 신호(ToF/위상차) → 거리(m), 각도(rad) 연산까지 ESP32가 완료
  - ESP-NOW 송수신, RSSI/PDR 집계
  - RPi로는 "이미 계산이 끝난 물리값"만 올려보냄
        ↓ 거리(m) / 각도(rad) / 주변차량 정보

[Raspberry Pi 5]  ← 이 프로젝트가 담당 (판단 + 목표값 계산)
  - platoon_fsm.py   : 적합도 계산, FSM 상태 전이, 결합/해제 판단
  - driving_control.py: 조향각(rad)·목표속도(m/s) 계산 (pure pursuit / 차선추종)
  - 모터 PWM, 듀티비, 엔코더 틱 같은 하드웨어 레벨 값은 여기서 다루지 않음
        ↓ 목표 조향각(rad) / 목표 속도(m/s)  ← "얼만큼 갈지"만 전달

[STM32]  ← 이 프로젝트 코드 아님
  - 받은 목표 조향각 → 서보 PWM 변환
  - 받은 목표 속도  → 모터 PID 제어로 추종
  - 아커만 조향 기하 기반 실제 구동

═══════════════════════════════════════════════════════════════════

매 사이클:
  1) ESP32/센서값 + 라인트레이싱 읽기 (플래툰 중에도 라인트레이싱은 계속)
  2) EgoState 갱신 + UWB 값 단일화(merge)
  3) fsm.update() 호출 → DrivingCommand(목표값) 받음
  4) compute_control()로 목표 조향각/속도 계산
  5) STM32로 목표값 전송 (프로토콜은 stm32_protocol.md 참고)
"""

import time

from platoon_fsm import PlatoonFSM, EgoState
from driving_control import compute_control, LaneFollower, LaneTracingResult
from stm32_interface import STM32Interface


# ══════════════════════════════════════════════════════════════════
# ESP32 연동 지점 — 연산은 ESP32가 끝냈고, 여기서는 값만 읽는다
# ══════════════════════════════════════════════════════════════════
def read_esp32_packet():
    """
    ESP32-S3가 올려주는 값을 읽어온다. (UWB 연산은 ESP32에서 이미 완료된 상태)

      nearby      : NearbyVehicle 리스트 — 일반 V2X 비콘 기반, 느린 주기 (§9.1)
                    각 항목의 uwb_distance/uwb_angle은 ESP32가 계산해 채워줌
      link_*      : 플래툰 전용 통신 기반 리더 링크값, 빠른 주기 (§9.2)
                    결합 이후 제어에 쓰이므로 nearby보다 최신일 수 있음

    TODO: RPi<->ESP32 시리얼 프로토콜 확정되면 파싱 로직으로 교체.
    """
    return {
        "nearby": [],
        "link_partner_id": None,
        "link_distance": None,   # m,   ESP32 연산 완료값
        "link_angle": None,      # rad, ESP32 연산 완료값
        "link_speed": None,      # m/s, 상대 차량이 V2X로 보낸 자기 속도
    }


def merge_link_into_nearby(esp32_data: dict, partner_id) -> list:
    """
    같은 물리량(리더까지의 거리·각도)이 두 경로로 들어오면 FSM 판단과 실제
    조향이 서로 다른 값을 보게 되므로, 빠른 링크값을 nearby 항목에 덮어써서
    단일 소스로 만든다. FSM도 제어도 이후로는 nearby 값만 본다.
    """
    nearby = esp32_data["nearby"]
    link_id = esp32_data["link_partner_id"]
    if link_id is None or link_id != partner_id:
        return nearby

    for v in nearby:
        if v.vehicle_id == link_id:
            if esp32_data["link_distance"] is not None:
                v.uwb_distance = esp32_data["link_distance"]
            if esp32_data["link_angle"] is not None:
                v.uwb_angle = esp32_data["link_angle"]
            if esp32_data["link_speed"] is not None:
                v.speed = esp32_data["link_speed"]
            break
    return nearby


def find_partner(nearby: list, partner_id):
    if partner_id is None:
        return None
    for v in nearby:
        if v.vehicle_id == partner_id:
            return v
    return None


# ══════════════════════════════════════════════════════════════════
# 기존 팀 코드 연동 지점
# ══════════════════════════════════════════════════════════════════
def read_lane_tracing() -> LaneTracingResult:
    """
    팀원의 라인트레이싱 코드에서 결과를 받아온다.

    필요한 값 두 가지:
      offset   : 정규화된 차선 중앙 오프셋 (-1.0 ~ +1.0)
                 = (라인중심_x - 이미지중심_x) / (이미지폭 / 2)
      detected : 이번 프레임에서 라인을 찾았는지

    픽셀값 그대로가 아니라 정규화해서 받아야 해상도가 바뀌어도
    PID 게인을 다시 잡지 않는다.

    TODO: 실제 라인트레이싱 함수 호출로 교체
    """
    return LaneTracingResult(offset=0.0, detected=False)


def open_stm32_port():
    """
    STM32 UART 포트를 연다. 프로토콜은 stm32_protocol.md 참고.
    TODO: 실제 포트 경로 확인 (/dev/ttyAMA0, /dev/ttyUSB0 등)
    """
    try:
        import serial
        return serial.Serial("/dev/ttyAMA0", 115200, timeout=0)
    except Exception as e:
        print(f"[warn] STM32 포트 열기 실패, 송수신 없이 진행: {e}")
        return None


def main():
    fsm = PlatoonFSM(vehicle_id=1)  # TODO: 실제 차량 고유 ID로 교체
    ego = EgoState()
    stm32 = STM32Interface(open_stm32_port())
    lane_follower = LaneFollower()
    loop_dt = 0.02  # 약 50Hz

    while True:
        esp32_data = read_esp32_packet()

        # STM32 피드백 = 엔코더 기반 실제 속도. 없으면 이전 값 유지.
        feedback = stm32.poll()
        if feedback is not None:
            current_speed = feedback.current_speed
            ego.obstacle = feedback.obstacle
            ego.front_distance = feedback.front_distance   # 초음파 실측 거리(m)
            ego.stm32_failsafe = feedback.failsafe          # STM32 자체 정지 중인지
        else:
            current_speed = ego.speed

        # UWB 값 단일화 — FSM과 제어가 같은 값을 보도록
        nearby = merge_link_into_nearby(esp32_data, fsm.partner_id)

        # 라인트레이싱은 플래툰 여부와 무관하게 항상 수행한다
        # (조향은 언제나 카메라 기반, V2V는 속도·거리 담당)
        lane = read_lane_tracing()

        # FSM 입력값(EgoState) 최신화
        ego.speed = current_speed
        ego.lane_offset = lane.offset
        ego.lane_detected = lane.detected
        # TODO: checkpoint / route / lane / heading 도 주행 알고리즘에서 받아 채우기

        # FSM은 매 루프 "한 번" 호출될 뿐, 자체 while 없음
        cmd = fsm.update(ego, nearby)
        role = fsm.role  # "LEADER" / "FOLLOWER" / None(SOLO)

        partner = find_partner(nearby, fsm.partner_id)

        control = compute_control(
            cmd, role, lane_follower,
            lane=lane,
            dt=loop_dt,
            uwb_distance=partner.uwb_distance if partner else None,
            uwb_angle=partner.uwb_angle if partner else None,
            current_speed=current_speed,
        )

        # STM32로는 "얼만큼 갈지"만 전달 — PID/PWM은 STM32가 처리
        # 비상정지 조건:
        #   - FSM 판정 (전방 장애물 / STM32 failsafe)
        #   - 라인 유실로 실제 정지 (플래툰 중 UWB로 버티는 경우는 제외)
        lane_stopping = control.lane_lost and not control.uwb_fallback
        stm32_mode = "EMERGENCY_STOP" if (cmd.emergency or lane_stopping) else cmd.mode
        stm32.send_command(
            mode=stm32_mode,
            target_speed=control.speed,
            target_steer=control.steering_rad,
        )

        time.sleep(loop_dt)


if __name__ == "__main__":
    main()