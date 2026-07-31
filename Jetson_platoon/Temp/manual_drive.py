"""
manual_drive.py
WASD로 직접 조종 — FSM/라인트레이싱을 전혀 거치지 않고 키보드 -> STM32로 바로 나간다.
배선과 STM32 파이프라인이 살아있는지 가장 먼저 확인하는 용도.

실행:
    sudo python3 manual_drive.py

sudo가 필요할 수 있는 이유: pynput이 /dev/input에 접근해야 키 이벤트를 받을 수 있다.

조작: W/S 속도, A/D 조향, SPACE 브레이크, Q 종료
종료 시 자동으로 정지 명령을 보낸다.
"""

import math
import time

from manual_control import ManualController
from stm32_interface import STM32Interface
from platoon_fsm import V_MAX
from driving_control import MAX_STEERING_RAD

LOOP_DT = 0.02  # 50Hz — main.py와 동일한 주기


def open_stm32_port():
    """젯슨 UART 후보 경로를 순서대로 시도한다."""
    import serial
    candidates = ["/dev/ttyTHS1", "/dev/ttyUSB0", "/dev/ttyACM0"]
    for path in candidates:
        try:
            port = serial.Serial(path, 115200, timeout=0)
            print(f"[stm32] {path} 로 연결됨")
            return port
        except Exception:
            continue
    print(f"[warn] STM32 포트를 못 열었습니다 (시도: {candidates}). 값만 콘솔에 출력하며 진행합니다.")
    return None


def main():
    stm32 = STM32Interface(open_stm32_port())
    controller = ManualController(max_steering_rad=MAX_STEERING_RAD, max_speed=V_MAX)

    if not controller.available:
        print("키보드 리스너를 못 띄웠습니다. sudo로 다시 실행해보세요.")
        return

    print("W/S: 속도   A/D: 조향   SPACE: 브레이크   Q: 종료")
    print(f"(최대 속도 {V_MAX:.2f} m/s, 최대 조향 {math.degrees(MAX_STEERING_RAD):.0f}도)")

    try:
        while not controller.should_quit:
            steering, speed = controller.update(LOOP_DT)
            stm32.send_command(mode="SOLO_DRIVE", target_speed=speed, target_steer=steering)
            print(f"\rsteer={math.degrees(steering):+6.1f}deg  speed={speed:+.2f}m/s   ", end="", flush=True)
            time.sleep(LOOP_DT)
    finally:
        controller.stop()
        stm32.send_command(mode="EMERGENCY_STOP", target_speed=0.0, target_steer=0.0)
        print("\n종료 — 정지 명령 전송")


if __name__ == "__main__":
    main()