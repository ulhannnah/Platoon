# ESP32-S3 UWB / Wi-Fi V2V Gateway

This ESP-IDF project follows the architecture in `../docs/260524_esp32s3_uwb_v2v_gateway_architecture.md`.

Current implementation:

- ESP-NOW vehicle status broadcast and receive
- Vehicle table keyed by `vehicle_id` and `uwb_id`
- UWB manager interface prepared for DW3000 integration
- Automatic fallback to Wi-Fi/ESP-NOW-only mode when UWB is unavailable
- Raspberry Pi output as line-delimited JSON over USB serial / UART console

Important files:

- `main/app_config.h`: vehicle ID, UWB ID, platoon info, periods, and UWB mock settings
- `main/espnow_manager.c`: Wi-Fi STA + ESP-NOW setup
- `main/uwb_manager.c`: DW3000 placeholder and optional mock mode
- `main/pi_serial_bridge.c`: JSON output for Raspberry Pi

To enable fake UWB data before connecting DW3000, set this in `main/app_config.h`:

```c
#define APP_ENABLE_UWB_MOCK        1
```

Typical build flow after ESP-IDF is installed and exported:

```powershell
cd D:\WorkSpace\Platoon\ESP_UWB\src
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```
