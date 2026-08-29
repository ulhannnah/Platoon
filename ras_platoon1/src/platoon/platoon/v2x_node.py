#!/usr/bin/env python3
"""
v2x_node.py
ESP32-S3 V2X 통신 전용 ROS2 노드.

"Raspberry Pi 플래툰 알고리즘 연동용 ESP32-S3 V2X 인터페이스 정리.md" 문서 기준으로
구현. USB CDC Serial + JSON Line 프로토콜(메시지 끝 '\n')로 ESP32-S3와 주고받는다.

    Pi  -> ESP32-S3 : self_status  (§5) — 주기적으로 송신
    ESP32-S3 -> Pi  : v2x_targets  (§7) — 수신하는 대로 파싱

이 노드는 FSM을 전혀 모른다 — SelfStatus 메시지를 그대로 JSON으로 바꿔 보내고,
받은 v2x_targets를 그대로 V2xTargets 메시지로 바꿔 발행하기만 한다. FSM 지식은
fsm_decision_node.py 쪽에만 있다 (이 노드가 fsm_decision_node.py를 import하지
않는 것도 그래서다 — 두 노드는 토픽으로만 연결된다).

★ 포트 충돌 주의: 문서는 ESP32도 /dev/ttyACM0을 예상한다고 되어 있지만,
  control_node.py가 이미 STM32용으로 /dev/ttyACM0을 쓰고 있다. 같은 포트를 두
  USB 장치가 동시에 쓸 수 없으므로, 실제로는 다른 경로(예: /dev/ttyACM1)로
  잡힐 가능성이 높다. 실행 전 `ls /dev/ttyACM* /dev/ttyUSB*`로 확인하고
  serial_port 파라미터로 지정할 것 — 기본값은 확정이 아니라 추정값이다.

토픽:
    발행 /v2x/targets      V2xTargets  (ESP32가 준 주변 차량 목록)
    구독 /v2x/self_status  SelfStatus  (fsm_decision_node.py가 주는 내 상태)

실행:
    ros2 run platoon v2x_node --ros-args -p serial_port:=/dev/ttyACM1
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from platoon_interfaces.msg import V2xTarget, V2xTargets, SelfStatus

# §9 상태값 정의 (문서 그대로) — fsm_decision_node.py도 이 값과 맞춰야 한다.
DRIVING_STATE_STOP = 0
DRIVING_STATE_MANUAL = 1
DRIVING_STATE_AUTO = 2
DRIVING_STATE_PLATOON = 3

PLATOON_STATE_WAIT = 0
PLATOON_STATE_SOLO = 1
PLATOON_STATE_JOIN = 2
PLATOON_STATE_KEEP = 3
PLATOON_STATE_EXIT = 4
PLATOON_STATE_PARKING = 5

PLATOON_ROLE_NONE = 0
PLATOON_ROLE_LEADER = 1
PLATOON_ROLE_FOLLOWER = 2


def _default_self_status() -> SelfStatus:
    """fsm_decision_node.py가 아직 한 번도 발행 안 했을 때 보낼 안전한 기본값 (§6.1 SOLO 예시)."""
    s = SelfStatus()
    s.driving_state = DRIVING_STATE_AUTO
    s.platoon_state = PLATOON_STATE_SOLO
    s.platoon_enable = 0
    return s


class V2XNode(Node):
    def __init__(self):
        super().__init__("v2x_node")

        self.declare_parameter("serial_port", "/dev/ttyACM1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("self_status_rate_hz", 10.0)  # 문서 §10: 주행 통합 단계 50~100ms

        self.port_path = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").value
        rate = self.get_parameter("self_status_rate_hz").value

        self._targets_pub = self.create_publisher(V2xTargets, "/v2x/targets", 10)
        self.create_subscription(SelfStatus, "/v2x/self_status", self._on_self_status, 10)

        self._last_self_status = _default_self_status()
        self._tx_seq = 0
        self._serial_lock = threading.Lock()
        self.ser = None
        self._open_serial()

        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.create_timer(1.0 / rate, self._send_self_status)
        self.get_logger().info(f"V2X 노드 시작 (포트: {self.port_path} @ {self.baud})")

    # ── 시리얼 열기/닫기 ─────────────────────────────────────────────
    def _open_serial(self) -> bool:
        with self._serial_lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(self.port_path, self.baud, timeout=0.5)
                self.get_logger().info(f"ESP32-S3 시리얼 연결 성공: {self.port_path}")
                return True
            except serial.SerialException as e:
                self.get_logger().warn(f"ESP32-S3 시리얼 연결 실패({self.port_path}): {e}")
                self.ser = None
                return False

    # ── 수신: ESP32 -> Pi (v2x_targets) ─────────────────────────────
    def _rx_loop(self):
        """
        JSON Line은 텍스트라 바이트 프로토콜(control_node.py의 헤더/체크섬 파서)과
        달리 개행 단위로 그냥 readline()하면 된다 — pyserial이 버퍼링을 해준다.
        """
        buf = b""
        while self._rx_running:
            if self.ser is None or not self.ser.is_open:
                time.sleep(1.0)
                self._open_serial()
                continue
            try:
                with self._serial_lock:
                    chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                self.get_logger().error(f"시리얼 수신 오류: {e}")
                with self._serial_lock:
                    if self.ser:
                        self.ser.close()
                    self.ser = None
                continue

            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_line(line)

    def _handle_line(self, raw: bytes) -> None:
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f"JSON 파싱 실패, 무시: {line[:100]!r}", throttle_duration_sec=2.0)
            return

        msg_type = data.get("type")
        if msg_type == "v2x_targets":
            self._targets_pub.publish(self._parse_v2x_targets(data))
        elif msg_type == "esp_status":
            pass  # TODO: §13.1 — 필요해지면 /v2x/esp_status로 발행
        # 그 외 msg_type은 조용히 무시 (프로토콜 확장 대비)

    def _parse_v2x_targets(self, data: dict) -> V2xTargets:
        """
        §7.2 v2x_targets JSON -> V2xTargets 메시지.

        leader_vehicle_id 보강: 원본 패킷엔 타겟별 리더 ID가 없다. 같은 패킷 안에
        platoon_role==LEADER인 차량이 있으면 그 vehicle_id를, 같은 platoon_id를
        쓰는 FOLLOWER 타겟들의 leader_vehicle_id로 채워준다. 리더가 이번 패킷에
        안 잡히면(멀어서 UWB/ESP-NOW 둘 다 안 보이면) -1로 남는다 — 이 경우
        platoon_fsm.py의 3대 이상 체인 결합 판단이 부정확해질 수 있다(§ 아래 TODO).
        """
        raw_targets = data.get("targets", [])

        leader_by_platoon = {
            t.get("platoon_id"): t.get("vehicle_id")
            for t in raw_targets
            if t.get("platoon_role") == PLATOON_ROLE_LEADER and t.get("platoon_id")
        }

        msg = V2xTargets()
        msg.seq = int(data.get("seq", 0))
        msg.timestamp_ms = int(data.get("timestamp_ms", 0))
        msg.self_vehicle_id = int(data.get("self_vehicle_id", 0))

        targets = []
        for t in raw_targets:
            v = V2xTarget()
            v.vehicle_id = int(t.get("vehicle_id", 0))
            v.uwb_id = int(t.get("uwb_id", 0))
            v.distance_m = float(t.get("distance_m", 0.0))
            v.angle_deg = float(t.get("angle_deg", 0.0))
            v.rel_x_m = float(t.get("rel_x_m", 0.0))
            v.rel_y_m = float(t.get("rel_y_m", 0.0))
            v.speed_mps = float(t.get("speed_mps", 0.0))
            v.heading_deg = float(t.get("heading_deg", 0.0))
            v.driving_state = int(t.get("driving_state", DRIVING_STATE_AUTO))
            v.platoon_state = int(t.get("platoon_state", PLATOON_STATE_SOLO))
            v.platoon_id = int(t.get("platoon_id", 0))
            v.platoon_enable = int(t.get("platoon_enable", 0))
            v.platoon_role = int(t.get("platoon_role", PLATOON_ROLE_NONE))
            v.platoon_index = int(t.get("platoon_index", 0))
            v.uwb_valid = int(t.get("uwb_valid", 0))
            v.espnow_valid = int(t.get("espnow_valid", 0))
            v.confidence = float(t.get("confidence", 0.0))

            if v.platoon_role == PLATOON_ROLE_LEADER:
                v.leader_vehicle_id = v.vehicle_id
            else:
                v.leader_vehicle_id = leader_by_platoon.get(t.get("platoon_id"), -1)

            targets.append(v)

        msg.targets = targets
        return msg

    # ── 송신: Pi -> ESP32 (self_status) ─────────────────────────────
    def _on_self_status(self, msg: SelfStatus) -> None:
        self._last_self_status = msg

    def _send_self_status(self) -> None:
        s = self._last_self_status
        self._tx_seq += 1
        payload = {
            "type": "self_status",
            "seq": self._tx_seq,
            "timestamp_ms": int(time.time() * 1000),
            "vehicle_id": s.vehicle_id,
            "uwb_id": s.uwb_id,
            "destination_id": s.destination_id,
            "driving_state": s.driving_state,
            "platoon_state": s.platoon_state,
            "speed_mps": s.speed_mps,
            "heading_deg": s.heading_deg,
            "platoon_enable": s.platoon_enable,
            "platoon_id": s.platoon_id,
            "platoon_role": s.platoon_role,
            "platoon_index": s.platoon_index,
            "leader_vehicle_id": s.leader_vehicle_id,
            "front_vehicle_id": s.front_vehicle_id,
            "target_speed_mps": s.target_speed_mps,
            "target_gap_m": s.target_gap_m,
        }
        line = (json.dumps(payload) + "\n").encode("utf-8")

        with self._serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(line)
                except serial.SerialException as e:
                    self.get_logger().error(f"ESP32 송신 실패: {e}")
                    self.ser.close()
                    self.ser = None

    def destroy_node(self):
        self._rx_running = False
        if hasattr(self, "_rx_thread") and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        with self._serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = V2XNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
