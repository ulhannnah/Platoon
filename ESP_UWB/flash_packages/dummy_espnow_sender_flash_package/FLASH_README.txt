ESP32-S3 dummy ESP-NOW sender firmware

Purpose:
- Use one extra ESP32-S3 connected to a PC as a dummy vehicle.
- PC sends self_status JSON Line to this ESP over USB.
- This ESP broadcasts the same vehicle status through ESP-NOW.
- The Pi-side ESP gateway receives it and forwards it to Raspberry Pi as v2x_targets.

Default dummy vehicle if PC sends nothing:
- vehicle_id: 202
- uwb_id: 40962
- driving_state: 3
- platoon_state: 3
- speed_mps: 0.45
- heading_deg: 0.0
- platoon_id: 7
- platoon_role: 2
- platoon_index: 1

Flash command for esptool v5:

python -m esptool --chip esp32s3 -p COM11 -b 460800 --before default-reset --after hard-reset write-flash 0x0 .\bootloader\bootloader.bin 0x8000 .\partition_table\partition-table.bin 0x10000 .\dummy_espnow_sender.bin

If your PC-side ESP port is different, replace COM11.

After flashing, open a serial terminal or use the included Python script:

python send_dummy_status_from_pc.py COM11

The ESP will print JSON ack/tx lines:

{"type":"dummy_ready","role":"pc_espnow_sender"}
{"type":"dummy_tx","vehicle_id":202,"uwb_id":40962,"speed_mps":0.45,"heading_deg":0.0,"seq":0}

Expected final result on Raspberry Pi:
- Pi-side v2x_node should receive v2x_targets.
- targets[] should contain vehicle_id 202 / uwb_id 40962.

Important:
- ESPNOW channel is 1. It must match the Pi-side ESP gateway.
- The binary packet layout matches the current ESP_UWB gateway firmware.
