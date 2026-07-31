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

import argparse
import time

from platoon_fsm import PlatoonFSM, EgoState, V_MAX
from driving_control import compute_control, LaneFollower, MAX_STEERING_RAD
from stm32_interface import STM32Interface
from manual_control import ManualController
from lane_tracing import PerceptionTracker


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
def open_stm32_port():
    """
    STM32 UART 포트를 연다 (젯슨 기준). 프로토콜은 stm32_protocol.md 참고.

    젯슨은 보드/캐리어에 따라 UART 장치명이 다릅니다:
      - Jetson Nano 개발자 키트: 보통 /dev/ttyTHS1
      - USB-시리얼 변환기 경유: /dev/ttyUSB0
    어떤 걸 쓰는지 확정되면 아래 후보 목록 순서를 바꾸거나 하나로 고정하세요.
    """
    import serial
    candidates = ["/dev/ttyTHS1", "/dev/ttyUSB0", "/dev/ttyACM0"]
    for path in candidates:
        try:
            port = serial.Serial(path, 115200, timeout=0)
            print(f"[stm32] {path} 로 연결됨")
            return port
        except Exception:
            continue
    print(f"[warn] STM32 포트를 못 열었습니다 (시도: {candidates}). 송수신 없이 진행합니다.")
    return None


def main():
    parser = argparse.ArgumentParser(description="플래툰 주행 메인 루프")
    parser.add_argument(
        "--mode", choices=["auto", "manual"], default="auto",
        help="시작 모드 (기본: auto). 실행 중에는 Q로 언제든 전환 가능.",
    )
    args = parser.parse_args()

    fsm = PlatoonFSM(vehicle_id=1)  # TODO: 실제 차량 고유 ID로 교체
    ego = EgoState()
    stm32 = STM32Interface(open_stm32_port())
    lane_follower = LaneFollower()
    perception = PerceptionTracker()   # 카메라 캡처+차선인식이 백그라운드 스레드에서 계속 돔
    if not perception.available:
        print("[warn] 카메라를 못 열어서 라인트레이싱 없이 진행합니다 (계속 lane_lost 상태).")
    loop_dt = 0.02  # 약 50Hz

    # 수동 조종 모드 — Q로 진입/이탈 토글 (같은 리스너를 자동 모드에서도 계속 띄워둠)
    manual = ManualController(max_steering_rad=MAX_STEERING_RAD, max_speed=V_MAX)
    manual_mode = (args.mode == "manual")
    if manual.available:
        print(f"{'수동(WASD)' if manual_mode else '자동'} 모드로 시작합니다. Q를 누르면 전환됩니다.")
    else:
        manual_mode = False
        print("[warn] 수동 모드 키보드 리스너를 못 띄워서 자동 주행만 가능합니다.")

    try:
        while True:
            # Q로 자동/수동 모드 전환. should_quit이 "모드 전환 신호"를 겸한다.
            if manual.available and manual.should_quit:
                manual._quit = False
                manual_mode = not manual_mode
                manual.state.speed = 0.0   # 모드 전환 시 항상 정지 상태로 시작 (안전)
                manual.state.steering = 0.0
                print(f"\n{'수동' if manual_mode else '자동'} 모드로 전환")

            if manual_mode:
                steering, speed = manual.update(loop_dt)
                stm32.send_command(mode="SOLO_DRIVE", target_speed=speed, target_steer=steering)
                print(f"\r[수동] steer={steering:+.3f}rad speed={speed:+.2f}m/s   ", end="", flush=True)
                time.sleep(loop_dt)
                continue

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
            # get_lane()은 논블로킹 — 백그라운드 스레드가 처리 중이면 직전 결과를 그대로 씀
            lane = perception.get_lane()

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
    finally:
        # Ctrl+C 등으로 종료될 때 카메라/키보드 리스너를 깔끔히 정리
        perception.stop()
        manual.stop()
        stm32.send_command(mode="EMERGENCY_STOP", target_speed=0.0, target_steer=0.0)
        print("\n종료 — 정지 명령 전송")


if __name__ == "__main__":
    main()