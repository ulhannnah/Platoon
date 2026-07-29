"""
stm32_interface.py
RPi5 ↔ STM32 UART 송수신 (Raspberry Pi 5 측)

프로토콜 명세: stm32_protocol.md — STM32 쪽 C 구조체와 1:1로 대응합니다.
한쪽을 고치면 반드시 다른 쪽도 같이 고쳐야 합니다.

역할 경계:
- 이 파일은 "목표 조향각(rad) / 목표 속도(m/s)"를 고정소수점으로 바꿔서
  프레이밍해 보내는 것까지만 한다.
- PWM 듀티, 모터 PID, 서보 각도 변환은 전부 STM32 담당.

프레임 구조:
    [0xAA][0x55][LEN][PAYLOAD...][CHECKSUM]
    CHECKSUM = LEN 및 PAYLOAD 전체의 XOR (헤더 제외)
"""

import struct
from dataclasses import dataclass
from typing import Optional

# ── 프로토콜 상수 (stm32_protocol.md §5.1과 동일해야 함) ────────────
HEADER_1 = 0xAA
HEADER_2 = 0x55
MAX_PAYLOAD = 32

MSG_DRIVE_COMMAND = 0x01    # RPi  -> STM32
MSG_DRIVE_FEEDBACK = 0x81   # STM32 -> RPi

MODE_SOLO_DRIVE = 0
MODE_PLATOON_JOIN = 1
MODE_PLATOON_MAINTAIN = 2
MODE_PLATOON_EXIT = 3
MODE_EMERGENCY_STOP = 0xFF

STATUS_MOTOR_ENABLED = 1 << 0
STATUS_FAILSAFE = 1 << 1
STATUS_OBSTACLE = 1 << 2

DISTANCE_INVALID = 0xFFFF   # 초음파 미측정

# DrivingCommand.mode(문자열) → 프로토콜 mode(정수)
MODE_MAP = {
    "SOLO_DRIVE": MODE_SOLO_DRIVE,
    "PLATOON_JOIN": MODE_PLATOON_JOIN,
    "PLATOON_MAINTAIN": MODE_PLATOON_MAINTAIN,
    "PLATOON_EXIT": MODE_PLATOON_EXIT,
    "EMERGENCY_STOP": MODE_EMERGENCY_STOP,
}

# 구조체 포맷 ('<' = little-endian, C쪽 __attribute__((packed))와 대응)
#   DriveCommand  : msg_type, mode, target_speed_mmps, target_steer_mrad, seq
FMT_DRIVE_COMMAND = "<BBhhB"       # 7 bytes
#   DriveFeedback : msg_type, status, speed, steer, front_distance, seq
FMT_DRIVE_FEEDBACK = "<BBhhHB"     # 9 bytes

INT16_MIN, INT16_MAX = -32768, 32767


def _clamp_int16(value: float) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(round(value))))


def calc_checksum(payload: bytes) -> int:
    """LEN 바이트 + PAYLOAD 전체의 XOR (C쪽 calc_checksum과 동일)"""
    cs = len(payload)
    for b in payload:
        cs ^= b
    return cs & 0xFF


def build_frame(payload: bytes) -> bytes:
    if not 0 < len(payload) <= MAX_PAYLOAD:
        raise ValueError(f"payload 길이 오류: {len(payload)}")
    return bytes([HEADER_1, HEADER_2, len(payload)]) + payload + bytes([calc_checksum(payload)])


@dataclass
class DriveFeedback:
    """STM32가 올려주는 상태 — EgoState.speed 등을 채우는 데 사용"""
    status: int = 0
    current_speed: float = 0.0          # m/s   (수신값 mm/s를 변환)
    current_steer: float = 0.0          # rad   (수신값 밀리라디안을 변환)
    front_distance: Optional[float] = None  # m, 미측정이면 None
    seq: int = 0

    @property
    def failsafe(self) -> bool:
        return bool(self.status & STATUS_FAILSAFE)

    @property
    def obstacle(self) -> bool:
        return bool(self.status & STATUS_OBSTACLE)


class PacketParser:
    """
    바이트가 한 개씩 들어와도 동작하는 상태머신 파서.
    STM32 쪽 parser_feed()와 같은 로직 — 헤더를 찾을 때까지 버리고,
    체크섬이 틀리면 패킷을 통째로 폐기한다.
    """

    WAIT_HEADER_1, WAIT_HEADER_2, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM = range(5)

    def __init__(self):
        self.state = self.WAIT_HEADER_1
        self.length = 0
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list:
        """수신 바이트를 넣고, 완성된 payload 리스트를 반환한다."""
        packets = []
        for byte in data:
            packet = self._feed_byte(byte)
            if packet is not None:
                packets.append(packet)
        return packets

    def _feed_byte(self, byte: int) -> Optional[bytes]:
        if self.state == self.WAIT_HEADER_1:
            if byte == HEADER_1:
                self.state = self.WAIT_HEADER_2

        elif self.state == self.WAIT_HEADER_2:
            if byte == HEADER_2:
                self.state = self.WAIT_LEN
            elif byte == HEADER_1:
                pass  # 0xAA 0xAA 0x55 대비, 상태 유지
            else:
                self.state = self.WAIT_HEADER_1

        elif self.state == self.WAIT_LEN:
            if byte == 0 or byte > MAX_PAYLOAD:
                self.state = self.WAIT_HEADER_1   # 비정상 길이, 폐기
            else:
                self.length = byte
                self.buffer.clear()
                self.state = self.WAIT_PAYLOAD

        elif self.state == self.WAIT_PAYLOAD:
            self.buffer.append(byte)
            if len(self.buffer) >= self.length:
                self.state = self.WAIT_CHECKSUM

        elif self.state == self.WAIT_CHECKSUM:
            payload = bytes(self.buffer)
            self.state = self.WAIT_HEADER_1
            if byte == calc_checksum(payload):
                return payload
            # 체크섬 불일치 → 폐기 (부분 반영 금지)

        return None


class STM32Interface:
    """
    STM32와의 UART 송수신.

    serial 포트는 생성자에서 주입받는다(테스트 시 가짜 객체 주입 가능).
    실제 사용 예:
        import serial
        port = serial.Serial("/dev/ttyAMA0", 115200, timeout=0)
        stm = STM32Interface(port)
    """

    def __init__(self, port=None):
        self.port = port
        self.parser = PacketParser()
        self._tx_seq = 0
        self.last_feedback: Optional[DriveFeedback] = None

    # ── 송신: 목표값을 STM32로 ────────────────────────────────────
    def send_command(self, mode: str, target_speed: float, target_steer: float) -> bytes:
        """
        target_speed : 목표 속도 (m/s)
        target_steer : 목표 조향각 (rad), + = 좌회전
        반환값       : 실제로 보낸 프레임 (디버깅/테스트용)
        """
        payload = struct.pack(
            FMT_DRIVE_COMMAND,
            MSG_DRIVE_COMMAND,
            MODE_MAP.get(mode, MODE_SOLO_DRIVE),
            _clamp_int16(target_speed * 1000),   # m/s  → mm/s
            _clamp_int16(target_steer * 1000),   # rad  → 밀리라디안
            self._tx_seq,
        )
        self._tx_seq = (self._tx_seq + 1) & 0xFF

        frame = build_frame(payload)
        if self.port is not None:
            self.port.write(frame)
        return frame

    def send_emergency_stop(self) -> bytes:
        return self.send_command("EMERGENCY_STOP", 0.0, 0.0)

    # ── 수신: STM32 상태 읽기 ─────────────────────────────────────
    def poll(self) -> Optional[DriveFeedback]:
        """
        수신 버퍼를 읽어 가장 최신 피드백을 반환한다 (없으면 None).
        논블로킹 — 메인 루프를 막지 않는다 (serial timeout=0 권장).
        """
        if self.port is None:
            return None

        waiting = getattr(self.port, "in_waiting", 0)
        if not waiting:
            return None

        for payload in self.parser.feed(self.port.read(waiting)):
            feedback = self._parse_feedback(payload)
            if feedback is not None:
                self.last_feedback = feedback

        return self.last_feedback

    @staticmethod
    def _parse_feedback(payload: bytes) -> Optional[DriveFeedback]:
        if len(payload) != struct.calcsize(FMT_DRIVE_FEEDBACK):
            return None
        if payload[0] != MSG_DRIVE_FEEDBACK:
            return None

        _, status, speed_mmps, steer_mrad, front_mm, seq = struct.unpack(FMT_DRIVE_FEEDBACK, payload)

        return DriveFeedback(
            status=status,
            current_speed=speed_mmps / 1000.0,
            current_steer=steer_mrad / 1000.0,
            front_distance=None if front_mm == DISTANCE_INVALID else front_mm / 1000.0,
            seq=seq,
        )


if __name__ == "__main__":
    # 루프백 자체검증 — 만든 프레임을 자기 파서로 되읽어본다
    stm = STM32Interface()

    frame = stm.send_command("PLATOON_MAINTAIN", target_speed=0.5, target_steer=0.12)
    print("TX frame:", frame.hex(" "))

    # STM32가 보낼 피드백을 흉내내서 파싱 검증
    fb_payload = struct.pack(FMT_DRIVE_FEEDBACK, MSG_DRIVE_FEEDBACK,
                             STATUS_MOTOR_ENABLED, 480, 115, 820, 7)
    fb_frame = build_frame(fb_payload)
    print("RX frame:", fb_frame.hex(" "))

    parser = PacketParser()
    # 앞에 쓰레기 바이트를 섞어도 헤더를 찾아 복구하는지 확인
    for payload in parser.feed(b"\x12\x34" + fb_frame):
        print("parsed  :", STM32Interface._parse_feedback(payload))

    # 체크섬이 틀린 프레임은 폐기되어야 함
    broken = bytearray(fb_frame)
    broken[-1] ^= 0xFF
    print("broken  :", parser.feed(bytes(broken)), "(빈 리스트여야 정상)")