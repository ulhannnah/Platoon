idf_component_register(
    SRCS
        "main.c"
        "vehicle_table.c"
        "uwb_manager.c"
        "espnow_manager.c"
        "tdma_scheduler.c"
        "id_matcher.c"
        "pi_serial_bridge.c"
        "system_monitor.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event nvs_flash esp_timer esp_driver_gpio esp_driver_spi esp_driver_usb_serial_jtag json decadriver libdeca
)
