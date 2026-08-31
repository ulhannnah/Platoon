import json
import sys
import time

import serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM11"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    seq = 1
    with serial.Serial(port, baud, timeout=1) as ser:
        print(f"Sending dummy vehicle self_status to {port} @ {baud}")
        while True:
            msg = {
                "type": "self_status",
                "seq": seq,
                "timestamp_ms": int(time.time() * 1000) & 0xFFFFFFFF,
                "vehicle_id": 202,
                "uwb_id": 40962,
                "destination_id": 3,
                "driving_state": 3,
                "platoon_state": 3,
                "speed_mps": 0.45 + 0.01 * (seq % 10),
                "heading_deg": float((seq * 5) % 360),
                "platoon_enable": 1,
                "platoon_id": 7,
                "platoon_role": 2,
                "platoon_index": 1,
                "leader_vehicle_id": 101,
                "front_vehicle_id": 101,
                "target_speed_mps": 0.50,
                "target_gap_m": 1.50,
            }
            line = json.dumps(msg, separators=(",", ":")) + "\n"
            ser.write(line.encode("utf-8"))
            ser.flush()
            print(line.strip())

            # Read whatever the dummy ESP printed back.
            deadline = time.time() + 0.1
            while time.time() < deadline:
                rx = ser.readline().decode("utf-8", errors="replace").strip()
                if rx:
                    print("ESP:", rx)

            seq += 1
            time.sleep(0.5)


if __name__ == "__main__":
    main()
