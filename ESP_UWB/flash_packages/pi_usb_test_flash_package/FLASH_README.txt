ESP32-S3 Pi USB communication test firmware

Purpose:
- ESP32-S3 <-> Raspberry Pi USB Serial/JTAG JSON Line communication test
- ESP -> Pi: v2x_targets JSON Line every 200 ms
- Pi -> ESP: self_status JSON Line input
- JSON-only serial output mode is enabled.
- ESP-NOW/UWB debug logs are suppressed so they do not corrupt JSON lines.
- v2x_targets is built as one complete line in RAM and sent with usb_serial_jtag_write_bytes() until all bytes are written.
- This avoids 64-byte USB Serial/JTAG console truncation/partial-write issues.

Expected ESP -> Pi output:
{"type":"v2x_targets","seq":0,"timestamp_ms":12345,"self_vehicle_id":101,"targets":[]}

Flash command for esptool v5:

python -m esptool --chip esp32s3 -p COM11 -b 460800 --before default-reset --after hard-reset write-flash 0x0 .\bootloader\bootloader.bin 0x8000 .\partition_table\partition-table.bin 0x10000 .\esp32s3_uwb_v2v_gateway.bin

If your port is different, replace COM11.

Examples:

python -m esptool --chip esp32s3 -p COM3 -b 460800 --before default-reset --after hard-reset write-flash 0x0 .\bootloader\bootloader.bin 0x8000 .\partition_table\partition-table.bin 0x10000 .\esp32s3_uwb_v2v_gateway.bin

python -m esptool --chip esp32s3 -p COM5 -b 460800 --before default-reset --after hard-reset write-flash 0x0 .\bootloader\bootloader.bin 0x8000 .\partition_table\partition-table.bin 0x10000 .\esp32s3_uwb_v2v_gateway.bin

Raspberry Pi receive test:

ls /dev/ttyACM*
python3 read_esp_v2x.py /dev/ttyACM0

Raspberry Pi send dummy self_status:

python3 send_self_status_dummy.py /dev/ttyACM0

Windows PC dummy self_status test:

python send_self_status_dummy.py COM11

Notes:
- Line ending matters. Each JSON message must end with '\n'.
- It is normal for targets to be [] when no other vehicle is detected.
- This firmware uses USB Serial/JTAG console for stdin/stdout.
