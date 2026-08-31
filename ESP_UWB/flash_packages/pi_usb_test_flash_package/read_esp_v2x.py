import json
import sys
import serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    with serial.Serial(port, baud, timeout=2) as ser:
        print(f"Reading ESP JSON Lines from {port} @ {baud}...")
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if not line.startswith("{"):
                print(line)
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                print("BAD JSON:", line)
                continue
            print(json.dumps(msg, ensure_ascii=False))


if __name__ == "__main__":
    main()
