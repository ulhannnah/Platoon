#!/usr/bin/env python3
"""
fake_esp32.py
실제 ESP32-S3 없이 v2x_node.py를 테스트하기 위한 가짜 ESP32 스크립트.

ROS2가 필요 없다 (pyserial + json만 씀) — v2x_node.py 입장에서는 진짜 ESP32와
구분이 안 되는 시리얼 상대 역할을 한다. socat으로 만든 가상 시리얼 페어의
반대쪽 끝에서 실행한다.

셋업 (라즈베리파이에서):
    1) 가상 시리얼 케이블 생성
       sudo apt install -y socat   # 없으면
       socat -d -d pty,raw,echo=0,link=/tmp/ttyESP_pi pty,raw,echo=0,link=/tmp/ttyESP_fake &

    2) 이 스크립트를 fake 쪽에서 실행
       python3 fake_esp32.py --port /tmp/ttyESP_fake --scenario approaching

    3) v2x_node를 pi 쪽 포트로 실행
       ros2 run platoon v2x_node --ros-args -p serial_port:=/tmp/ttyESP_pi

    4) 확인
       ros2 topic echo /v2x/targets      # 이 스크립트가 보낸 값이 그대로 보이는지
       (이 스크립트 터미널에도 self_status가 수신되는 대로 출력됨 — v2x_node가
        /v2x/self_status를 잘 중계하는지 왕복 확인)

주의: 이 테스트는 "시리얼 프로토콜이 맞게 도는지"만 확인한다. platoon_fsm.py가
실제로 결합(JOIN)까지 가는지는 별도 문제다 — v2x_targets엔 체크포인트/경로
정보가 없어서 §6 절대조건(C_R)이 항상 실패하고, 그건 이 테스트로 해결 안 된다
(esp32_v2x_보강요청.md 3번 항목, 표지판/마커 결정 이후 사안).
"""

import argparse
import json
import threading
import time

import serial


def build_targets(seq: int, scenario: str) -> dict:
    """§7.2 형식의 v2x_targets 페이로드를 시나리오별로 생성."""
    if scenario == "empty":
        targets = []

    elif scenario == "static":
        # 고정된 거리의 팔로워 후보 차량 1대 — 가장 단순한 "뭔가 잡히는지" 확인용
        targets = [{
            "vehicle_id": 102, "uwb_id": 40962,
            "distance_m": 1.5, "angle_deg": -5.0,
            "rel_x_m": 1.49, "rel_y_m": -0.13,
            "speed_mps": 0.5, "heading_deg": 0.0,
            "driving_state": 2, "platoon_state": 1,
            "platoon_id": 0, "platoon_enable": 1,
            "platoon_role": 0, "platoon_index": 0,
            "uwb_valid": 1, "espnow_valid": 1, "confidence": 0.9,
        }]

    elif scenario == "approaching":
        # 5m -> 1m로 서서히 가까워지는 차량. seq가 커질수록 거리 감소.
        distance = max(1.0, 5.0 - seq * 0.1)
        targets = [{
            "vehicle_id": 102, "uwb_id": 40962,
            "distance_m": round(distance, 2), "angle_deg": 0.0,
            "rel_x_m": round(distance, 2), "rel_y_m": 0.0,
            "speed_mps": 0.5, "heading_deg": 0.0,
            "driving_state": 2, "platoon_state": 1,
            "platoon_id": 0, "platoon_enable": 1,
            "platoon_role": 0, "platoon_index": 0,
            "uwb_valid": 1, "espnow_valid": 1, "confidence": 0.9,
        }]

    elif scenario == "three_car":
        # 이미 결합된 리더(101)+팔로워1(102) 뒤에 내가 붙는 상황.
        # leader_vehicle_id 보강 로직(v2x_node._parse_v2x_targets) 테스트용 —
        # 101이 LEADER로 같은 패킷에 잡히면 102(FOLLOWER)의 leader_vehicle_id가
        # v2x_node에서 101로 채워져야 정상이다.
        targets = [
            {"vehicle_id": 101, "uwb_id": 40961, "distance_m": 2.3, "angle_deg": 0.0,
             "rel_x_m": 2.3, "rel_y_m": 0.0, "speed_mps": 0.5, "heading_deg": 0.0,
             "driving_state": 3, "platoon_state": 3, "platoon_id": 7, "platoon_enable": 1,
             "platoon_role": 1, "platoon_index": 0, "uwb_valid": 1, "espnow_valid": 1, "confidence": 0.9},
            {"vehicle_id": 102, "uwb_id": 40962, "distance_m": 1.5, "angle_deg": 0.0,
             "rel_x_m": 1.5, "rel_y_m": 0.0, "speed_mps": 0.5, "heading_deg": 0.0,
             "driving_state": 3, "platoon_state": 3, "platoon_id": 7, "platoon_enable": 1,
             "platoon_role": 2, "platoon_index": 1, "uwb_valid": 1, "espnow_valid": 1, "confidence": 0.9},
        ]

    else:
        raise ValueError(f"알 수 없는 시나리오: {scenario}")

    return {
        "type": "v2x_targets",
        "seq": seq,
        "timestamp_ms": int(time.time() * 1000),
        "self_vehicle_id": 999,  # 가짜 ESP32 자신은 신경 안 써도 됨 (Pi가 안 씀)
        "targets": targets,
    }


def reader_thread(ser: serial.Serial) -> None:
    """Pi가 보내는 self_status를 그대로 받아서 화면에 찍는다 (왕복 확인용)."""
    buf = b""
    while True:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            try:
                data = json.loads(line.decode("utf-8", errors="replace"))
            except json.JSONDecodeError:
                print(f"[fake_esp32] 수신 파싱 실패: {line[:120]!r}")
                continue
            if data.get("type") == "self_status":
                print(f"[fake_esp32] <- self_status 수신: "
                      f"speed_mps={data.get('speed_mps')} "
                      f"platoon_role={data.get('platoon_role')} "
                      f"platoon_state={data.get('platoon_state')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="가상 시리얼 포트 (예: /tmp/ttyESP_fake)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate-hz", type=float, default=5.0, help="v2x_targets 송신 주기")
    ap.add_argument("--scenario", default="static",
                    choices=["empty", "static", "approaching", "three_car"])
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    print(f"[fake_esp32] {args.port} 열림, 시나리오={args.scenario}, {args.rate_hz}Hz로 송신 시작")

    threading.Thread(target=reader_thread, args=(ser,), daemon=True).start()

    seq = 0
    try:
        while True:
            seq += 1
            payload = build_targets(seq, args.scenario)
            line = (json.dumps(payload) + "\n").encode("utf-8")
            ser.write(line)
            print(f"[fake_esp32] -> v2x_targets #{seq} 송신 "
                  f"(targets={len(payload['targets'])}개)")
            time.sleep(1.0 / args.rate_hz)
    except KeyboardInterrupt:
        print("\n[fake_esp32] 종료")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
