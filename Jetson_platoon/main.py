"""
main.py
전체 메인 루프 — 유일한 while문이 여기 있습니다. FSM 자체는 루프를 안 돌고,
이 while문이 매 사이클마다 fsm.update()를 "한 번씩" 호출하는 구조입니다.

매 사이클:
  1) 센서/통신값 읽기
     - ESP32가 UWB로 이미 계산해서 준 값(리더까지의 거리·각도, 주변차량 리스트)
     - 팔로워로 추종 중이 아닐 때만 카메라 프레임
  2) EgoState 갱신
  3) fsm.update() 호출 → DrivingCommand(목표값) 받음
  4) driving_control.compute_control()로 실제 조향각/속도 계산
     - 팔로워 추종 중 → pure pursuit (카메라 안 씀)
     - 그 외 → 카메라 차선추종
  5) STM32로 최종 값 전송

ESP32 쪽 연산(원시 UWB 신호 → 거리/각도 계산)은 ESP32 펌웨어가 담당하므로
이 파일/프로젝트에서는 다루지 않습니다. 여기서는 이미 계산된 값을
"어떻게 시리얼로 읽어오는지"만 자리를 잡아뒀습니다(TODO).
"""

import time

from platoon_fsm import PlatoonFSM, EgoState
from driving_control import compute_control


# ── ESP32 연동 지점 (연산은 ESP32가 함, 여기선 값만 읽음) ──────────
def read_esp32_packet():
    """
    ESP32-S3가 UWB 기반으로 이미 계산해서 시리얼로 올려주는 값을 읽어온다.
      - distance_to_leader / angle_to_leader : 팔로워 조향(pure pursuit)용
      - leader_speed                         : 팔로워 목표속도 계산용
      - nearby                               : NearbyVehicle 리스트 (FSM 매칭/평가용)

    TODO: 실제 시리얼 프로토콜(포맷) 정해지면 파싱 로직으로 교체.
          지금은 아직 없으므로 빈/기본값 반환.
    """
    return {
        "distance_to_leader": None,
        "angle_to_leader": None,
        "leader_speed": None,
        "nearby": [],
    }


def read_camera_frame():
    """TODO: 기존 라인트레이싱 코드의 카메라 캡처 함수로 교체"""
    return None


def read_current_speed() -> float:
    """TODO: 실제 속도 센서/엔코더 값으로 교체"""
    return 0.0


def send_to_stm32(steering_rad: float, speed: float):
    """TODO: 기존 STM32 UART 송신 코드로 교체"""
    pass


def main():
    fsm = PlatoonFSM(vehicle_id=1)  # TODO: 실제 차량 고유 ID로 교체
    ego = EgoState()

    while True:
        esp32_data = read_esp32_packet()
        current_speed = read_current_speed()

        # FSM 입력값(EgoState) 최신화
        ego.speed = current_speed
        nearby = esp32_data["nearby"]

        # FSM은 매 루프 "한 번" 호출될 뿐, 자체 while 없음
        cmd = fsm.update(ego, nearby)
        role = fsm.role  # "LEADER" / "FOLLOWER" / None(SOLO)

        # 팔로워로 실제 추종 중일 때만 카메라를 건너뜀
        is_following = (role == "FOLLOWER" and cmd.mode in ("PLATOON_JOIN", "PLATOON_MAINTAIN"))
        frame = None if is_following else read_camera_frame()

        control = compute_control(
            cmd, role,
            camera_frame=frame,
            uwb_distance=esp32_data["distance_to_leader"],
            uwb_angle=esp32_data["angle_to_leader"],
            leader_speed=esp32_data["leader_speed"],
            current_speed=current_speed,
        )

        send_to_stm32(control.steering_rad, control.speed)

        time.sleep(0.02)  # 약 50Hz


if __name__ == "__main__":
    main()