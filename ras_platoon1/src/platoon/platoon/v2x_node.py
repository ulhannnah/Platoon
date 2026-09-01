"""
v2x_node.py
ESP32-S3 V2X 통신 전용 ROS2 노드.

"Raspberry Pi 플래툰 알고리즘 연동용 ESP32-S3 V2X 인터페이스 정리.md" 문서 기준으로
구현. USB CDC Serial + JSON Line 프로토콜(메시지 끝 '\n')로 ESP32-S3와 주고받는다.

    Pi  -> ESP32-S3 : self_status  (§5) — 주기적으로 송신
    ESP32-S3 -> Pi  : v2x_targets  (§7) — 수신하는 대로 파싱

★ 포트 번호(ttyACM0/1)는 재부팅 때마다 USB 열거 순서에 따라 ESP32/STM32끼리
  뒤바뀔 수 있다(실측으로 확인됨). 그래서 기본값을 /dev/serial/by-id/의 고정
  경로로 잡는다 — 이 경로는 이 ESP32 보드의 시리얼번호에 매여 있어 재부팅해도
  안 바뀐다. 보드를 교체하면 `ls /dev/serial/by-id/`로 새 이름 확인 후 이
  기본값 또는 serial_port 파라미터를 갱신할 것.

토픽:
    발행 /v2x/targets      V2xTargets  (ESP32가 준 주변 차량 목록)
    구독 /v2x/self_status  SelfStatus  (fsm_decision_node.py가 주는 내 상태)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from platoon_interfaces.msg import V2xTarget, V2xTargets, SelfStatus

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 상태 정상 정의 (fsm_decision_node.py도 이 값과 맞춰야 함)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# 주행 상태 (자동주행 준비 상태)
DRIVING_STATE_STOP = 0       # 정지 상태
DRIVING_STATE_MANUAL = 1     # 수동 운전
DRIVING_STATE_AUTO = 2       # 자동 주행 (수동 시점)
DRIVING_STATE_PLATOON = 3    # 플래툰 자동 주행

# 플래툰 상태 (플래툰 편대 상태)
PLATOON_STATE_WAIT = 0       # 대기 중
PLATOON_STATE_SOLO = 1       # 단독 주행 (플래툰 없음)
PLATOON_STATE_JOIN = 2       # 플래툰 입장 시도 중
PLATOON_STATE_KEEP = 3       # 플래툰 유지 중
PLATOON_STATE_EXIT = 4       # 플래툰 이탈 중
PLATOON_STATE_PARKING = 5    # 주차 상태

# 플래툰 역할
PLATOON_ROLE_NONE = 0        # 역할 없음
PLATOON_ROLE_LEADER = 1      # 리더 (편대장)
PLATOON_ROLE_FOLLOWER = 2    # 팔로워 (추종 차량)


# fsm_decision_node.py가 아직 한 번도 발행하지 않았을 때 사용할 안전한 기본값.
def _default_self_status() -> SelfStatus:
    s = SelfStatus()
    s.driving_state = DRIVING_STATE_AUTO   # 자동 주행 모드
    s.platoon_state = PLATOON_STATE_SOLO   # 단독 주행
    s.platoon_enable = 0                    # 플래툰 비활성화
    return s


class V2XNode(Node):
    
    def __init__(self):
        super().__init__("v2x_node")
        # ESP32-S3 시리얼 포트 경로
        # by-id 경로 사용: 재부팅 후에도 USB 열거 순서 변경 대비
        self.declare_parameter(
            "serial_port",
            "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:C1:26:8C-if00",
        )
        self.declare_parameter("baud", 115200)
        # self_status 송신 주기 (Hz 단위, 기본 10Hz = 100ms)
        self.declare_parameter("self_status_rate_hz", 10.0)

        # 파라미터값 읽기
        self.port_path = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").value
        rate = self.get_parameter("self_status_rate_hz").value

        # 토픽 발행 및 구독
        self._targets_pub = self.create_publisher(V2xTargets, "/v2x/targets", 10)
        self.create_subscription(SelfStatus, "/v2x/self_status", self._on_self_status, 10)

        # 마지막으로 받은 self_status (서브스크라이브 콜백이 갱신)
        # fsm_decision_node가 아직 발행 안 했으면 기본값으로 시작
        self._last_self_status = _default_self_status()
        
        # 송신 시퀀스 번호 (매 송신마다 +1, 수신 확인용)
        self._tx_seq = 0
        
        # 시리얼 포트 접근 보호 (메인 스레드 + 수신 스레드 동시 접근 방지)
        self._serial_lock = threading.Lock()
        
        # 시리얼 포트 객체 (None이면 미연결 상태)
        self.ser = None

        # ESP32-S3과 시리얼 연결 시도
        self._open_serial()

        # 수신 루프 플래그 (종료 시 False로 설정해 스레드 정지)
        self._rx_running = True
        
        # 수신 전담 스레드 생성
        # JSON Line 프로토콜로 들어오는 메시지들을 블로킹 방식으로 대기/처리
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # ROS2 타이머: rate(Hz)에 따라 주기적으로 _send_self_status() 호출
        # 기본 10Hz = 100ms마다 현재 상태를 ESP32-S3로 송신
        self.create_timer(1.0 / rate, self._send_self_status)
        
        # 노드 시작 로그
        self.get_logger().info(f"V2X 노드 시작 (포트: {self.port_path} @ {self.baud})")

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 시리얼 포트 연결 관리
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _open_serial(self) -> bool:
        with self._serial_lock:
            try:
                # 기존 연결 닫기
                if self.ser and self.ser.is_open:
                    self.ser.close()
                
                # 새 시리얼 포트 열기
                # timeout=0.5: 수신 시 0.5초 대기 후 타임아웃 (블로킹 방지)
                self.ser = serial.Serial(self.port_path, self.baud, timeout=0.5)
                self.get_logger().info(f"ESP32-S3 시리얼 연결 성공: {self.port_path}")
                return True
            except serial.SerialException as e:
                self.get_logger().warn(f"ESP32-S3 시리얼 연결 실패({self.port_path}): {e}")
                self.ser = None
                return False

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 수신: ESP32-S3 -> Raspberry Pi (v2x_targets)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    def _rx_loop(self):
        buf = b""  # 누적 버퍼 (완성된 라인 분리할 때까지 보관)
        while self._rx_running:
            # 시리얼 포트 미연결 상태 처리
            if self.ser is None or not self.ser.is_open:
                time.sleep(1.0)  # 1초 대기 후 재연결 시도
                self._open_serial()
                continue
            
            try:
                # 시리얼 포트에서 사용 가능한 바이트 읽기
                # in_waiting: 읽을 수 있는 바이트 개수
                # in_waiting이 0이면 1바이트 대기 (블로킹)
                with self._serial_lock:
                    chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                # 시리얼 통신 오류 (예: USB 케이블 뽑음)
                self.get_logger().error(f"시리얼 수신 오류: {e}")
                with self._serial_lock:
                    if self.ser:
                        self.ser.close()
                    self.ser = None
                continue

            if not chunk:
                continue
            
            # 버퍼에 수신 데이터 누적
            buf += chunk
            
            # 버퍼에 완성된 라인('\n' 포함)이 있으면 하나씩 처리
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)  # 첫 '\n'을 기준으로 분리
                self._handle_line(line)

    def _handle_line(self, raw: bytes) -> None:
        # UTF-8 디코딩 (오류 있으면 '?'로 대체)
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return  # 빈 라인 무시
        
        # JSON 파싱
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            # 파싱 실패 → 로그만 기록하고 무시
            # throttle_duration_sec: 같은 메시지는 2초 이내에 1번만 출력 (로그 폭발 방지)
            self.get_logger().warn(f"JSON 파싱 실패, 무시: {line[:100]!r}", throttle_duration_sec=2.0)
            return

        # 메시지 타입별 처리
        msg_type = data.get("type")
        if msg_type == "v2x_targets":
            # ESP32-S3가 감지한 주변 차량 목록
            self._targets_pub.publish(self._parse_v2x_targets(data))
        elif msg_type == "esp_status":
            # TODO: §13.1 — ESP32 상태 정보 (필요하면 /v2x/esp_status 토픽으로 발행)
            pass
        # 그 외 msg_type은 조용히 무시 (프로토콜 향후 확장 대비)

    def _parse_v2x_targets(self, data: dict) -> V2xTargets:
        # 원본 targets 배열 추출
        raw_targets = data.get("targets", [])

        # 플래툰 ID별 리더 차량 ID 매핑
        # 같은 platoon_id를 쓰는 LEADER가 있으면 그 vehicle_id를 기록
        leader_by_platoon = {
            t.get("platoon_id"): t.get("vehicle_id")
            for t in raw_targets
            if t.get("platoon_role") == PLATOON_ROLE_LEADER and t.get("platoon_id")
        }

        # ROS2 메시지 객체 생성 및 메타데이터 채우기
        msg = V2xTargets()
        msg.seq = int(data.get("seq", 0))
        msg.timestamp_ms = int(data.get("timestamp_ms", 0))
        msg.self_vehicle_id = int(data.get("self_vehicle_id", 0))

        # 각 타겟을 V2xTarget 메시지로 변환
        targets = []
        for t in raw_targets:
            v = V2xTarget()
            # 신원 정보
            v.vehicle_id = int(t.get("vehicle_id", 0))
            v.uwb_id = int(t.get("uwb_id", 0))
            
            # 거리/각도 정보 (극좌표)
            v.distance_m = float(t.get("distance_m", 0.0))
            v.angle_deg = float(t.get("angle_deg", 0.0))
            
            # 상대 위치 (직각좌표)
            v.rel_x_m = float(t.get("rel_x_m", 0.0))
            v.rel_y_m = float(t.get("rel_y_m", 0.0))
            
            # 운동 상태
            v.speed_mps = float(t.get("speed_mps", 0.0))
            v.heading_deg = float(t.get("heading_deg", 0.0))
            
            # 주행/플래툰 상태
            v.driving_state = int(t.get("driving_state", DRIVING_STATE_AUTO))
            v.platoon_state = int(t.get("platoon_state", PLATOON_STATE_SOLO))
            v.platoon_id = int(t.get("platoon_id", 0))
            v.platoon_enable = int(t.get("platoon_enable", 0))
            v.platoon_role = int(t.get("platoon_role", PLATOON_ROLE_NONE))
            v.platoon_index = int(t.get("platoon_index", 0))
            
            # 센서 유효성
            v.uwb_valid = int(t.get("uwb_valid", 0))
            v.espnow_valid = int(t.get("espnow_valid", 0))
            v.confidence = float(t.get("confidence", 0.0))

            # leader_vehicle_id 채우기 (리더는 자신의 ID, 팔로워는 리더의 ID 또는 -1)
            if v.platoon_role == PLATOON_ROLE_LEADER:
                v.leader_vehicle_id = v.vehicle_id
            else:
                v.leader_vehicle_id = leader_by_platoon.get(t.get("platoon_id"), -1)

            targets.append(v)

        msg.targets = targets
        return msg

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 송신: Raspberry Pi -> ESP32-S3 (self_status)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    def _on_self_status(self, msg: SelfStatus) -> None:
        self._last_self_status = msg

    def _send_self_status(self) -> None:
        s = self._last_self_status
        self._tx_seq += 1  # 송신 시퀀스 번호 증가
        
        # JSON 페이로드 구성
        payload = {
            "type": "self_status",
            "seq": self._tx_seq,
            "timestamp_ms": int(time.time() * 1000),  # 현재 시간(ms)
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
        
        # JSON을 문자열로 변환 후 개행 추가 (JSON Line 프로토콜)
        line = (json.dumps(payload) + "\n").encode("utf-8")

        # 스레드 안전하게 시리얼 포트로 송신
        with self._serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(line)
                except serial.SerialException as e:
                    self.get_logger().error(f"ESP32 송신 실패: {e}")
                    self.ser.close()
                    self.ser = None


    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 노드 종료 처리
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    def destroy_node(self):
        # 수신 스레드에 종료 신호
        self._rx_running = False
        
        # 수신 스레드가 실행 중이면 종료 대기 (최대 1초)
        if hasattr(self, "_rx_thread") and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        
        # 시리얼 포트 종료
        with self._serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        
        # ROS2 노드 정리
        super().destroy_node()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 노드 실행 진입점
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    # 노드 생성
    node = V2XNode()
    
    try:
        # ROS2 이벤트 루프 (토픽 수신, 타이머 콜백 등 처리)
        # Ctrl+C 입력 시 rclpy.spin() 탈출
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 입력 시 정상적으로 처리
        pass
    finally:
        # 항상 정리 작업 수행
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
