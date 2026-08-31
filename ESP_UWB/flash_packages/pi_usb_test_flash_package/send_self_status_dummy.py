import json
import sys
import time
import serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    seq = 1
    with serial.Serial(port, baud, timeout=1) as ser:
        print(f"Sending dummy self_status to {port} @ {baud}...")
        while True:
            msg = {
                "type": "self_status",
                "seq": seq,
                "timestamp_ms": int(time.time() * 1000) & 0xFFFFFFFF,
                "vehicle_id": 101,
                "uwb_id": 40961,
                "destination_id": 3,
                "driving_state": 3,
                "platoon_state": 3,
                "speed_mps": 0.42,
                "heading_deg": 0.0,
                "platoon_enable": 1,
                "platoon_id": 7,
                "platoon_role": 1,
                "platoon_index": 0,
                "leader_vehicle_id": 101,
                "front_vehicle_id": 0,
                "target_speed_mps": 0.50,
                "target_gap_m": 1.50,
            }
            line = json.dumps(msg, separators=(",", ":")) + "\n"
            ser.write(line.encode("utf-8"))
            ser.flush()
            print(line.strip())
            seq += 1
            time.sleep(0.5)


if __name__ == "__main__":
    main()
