"""
manual_control.py
WASD 수동 조종 모드 — 실차 배선/제어 파이프라인 확인용, 이후 상시 수동 모드로도 사용.

조작:
  W / S : 전진/후진 속도 증가 — 누르고 있는 동안 계속 증가, 떼면 그 값 유지
  A / D : 조향 증가 (D=오른쪽, A=왼쪽) — 누르고 있는 동안 계속 증가, 떼면 그 값 유지
  SPACE : 즉시 정지 (속도만 0, 조향은 유지)
  Q     : 수동 모드 종료

"떼면 유지"는 실제 keydown/keyup 이벤트가 있어야 구현 가능하다. 터미널에서 흔히
쓰는 한 글자씩 읽는 방식은 "떼었다"는 신호 자체가 없어서, 여기서는 pynput으로
OS 레벨 키 이벤트를 직접 받는다.

이 파일은 두 부분으로 나뉜다:
  - ManualControlState : 순수 계산 로직 (pynput 없이도 단독 테스트 가능)
  - ManualController    : 위 로직에 pynput 키보드 리스너를 연결하는 어댑터

리눅스에서 헤드리스로 돌리면 /dev/input 접근 권한이 필요해 보통 sudo로 실행해야 한다.
"""

import math
import time
from threading import Lock
from typing import Optional, Set

STEERING_RATE_RAD_S = math.radians(45)   # 조향 변화 속도 (초당)
SPEED_RATE_MPS_S = 0.6                    # 속도 변화 속도 (초당)


class ManualControlState:
    """
    순수 계산 로직. 눌려있는 키 집합(held)만 주어지면 매 tick 조향/속도를 갱신한다.
    pynput 등 실제 입력 수단과 분리되어 있어 단독으로 테스트할 수 있다.
    """

    def __init__(self, max_steering_rad: float, max_speed: float):
        self.max_steering = max_steering_rad
        self.max_speed = max_speed
        self.steering = 0.0
        self.speed = 0.0

    def tick(self, held: Set[str], braked: bool, dt: float) -> tuple:
        """
        held   : 이번 tick에 눌려있는 것으로 확인된 키 이름 집합 ({"w","a","s","d"} 중 일부)
        braked : SPACE가 눌렸는지 (1회성 이벤트로 소비되어야 함, 호출부 책임)
        dt     : 경과 시간(초)
        반환값 : (steering_rad, speed)
        """
        if braked:
            self.speed = 0.0

        if "w" in held and "s" in held:
            pass  # 동시 입력은 무시 — 마지막 값 유지
        elif "w" in held:
            self.speed = min(self.max_speed, self.speed + SPEED_RATE_MPS_S * dt)
        elif "s" in held:
            self.speed = max(-self.max_speed, self.speed - SPEED_RATE_MPS_S * dt)
        # 둘 다 안 눌려있으면 마지막 값 유지 (자동 감속 없음 — 요구사항)

        if "a" in held and "d" in held:
            pass
        elif "d" in held:
            self.steering = min(self.max_steering, self.steering + STEERING_RATE_RAD_S * dt)
        elif "a" in held:
            self.steering = max(-self.max_steering, self.steering - STEERING_RATE_RAD_S * dt)

        return self.steering, self.speed


class ManualController:
    """ManualControlState에 pynput 키보드 리스너를 연결하는 어댑터."""

    def __init__(self, max_steering_rad: float, max_speed: float):
        self.state = ManualControlState(max_steering_rad, max_speed)
        self._held: Set[str] = set()
        self._lock = Lock()
        self._braked = False
        self._quit = False
        self._listener = None

        try:
            from pynput import keyboard
            self._listener = keyboard.Listener(
                on_press=self._on_press, on_release=self._on_release
            )
            self._listener.start()
        except Exception as e:
            # 디스플레이/입력장치 접근 불가 등 — 수동모드 없이 나머지는 정상 동작해야 하므로
            # 여기서 죽지 않고 경고만 남긴다. 호출부에서 self.available로 확인.
            print(f"[warn] 키보드 리스너 시작 실패 (sudo로 실행했는지 확인): {e}")

    @property
    def available(self) -> bool:
        return self._listener is not None

    @staticmethod
    def _key_name(key) -> Optional[str]:
        try:
            return key.char.lower()
        except AttributeError:
            from pynput import keyboard
            if key == keyboard.Key.space:
                return "space"
            return None

    def _on_press(self, key):
        name = self._key_name(key)
        if name is None:
            return
        with self._lock:
            if name == "space":
                self._braked = True
            elif name == "q":
                self._quit = True
            else:
                self._held.add(name)

    def _on_release(self, key):
        name = self._key_name(key)
        if name is None:
            return
        with self._lock:
            self._held.discard(name)

    @property
    def should_quit(self) -> bool:
        with self._lock:
            return self._quit

    def update(self, dt: float) -> tuple:
        with self._lock:
            held = set(self._held)
            braked = self._braked
            self._braked = False   # 1회성 이벤트 소비
        return self.state.tick(held, braked, dt)

    def stop(self):
        if self._listener is not None:
            self._listener.stop()
