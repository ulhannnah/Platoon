ESP32-S3 UWB V2V Gateway flash package

Target:
- ESP32-S3
- Flash size: 4MB
- Flash mode: DIO
- Flash freq: 80MHz

Files:
- bootloader/bootloader.bin
- partition_table/partition-table.bin
- esp32s3_uwb_v2v_gateway.bin
- flash_args

Flash with ESP-IDF:
1. Copy this package to the PC connected to the ESP32-S3 board.
2. Open ESP-IDF PowerShell.
3. Move to this folder.
4. Run:

   esptool.py --chip esp32s3 -p COM_PORT -b 460800 --before default_reset --after hard_reset write_flash @flash_args

Example:

   esptool.py --chip esp32s3 -p COM3 -b 460800 --before default_reset --after hard_reset write_flash @flash_args

If esptool.py is not found, use:

   python -m esptool --chip esp32s3 -p COM3 -b 460800 --before default_reset --after hard_reset write_flash @flash_args

Monitor after flashing:

   idf.py -p COM3 monitor

Exit monitor:

   Ctrl + ]
