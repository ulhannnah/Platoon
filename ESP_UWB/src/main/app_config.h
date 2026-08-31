#pragma once

#include <stdint.h>

/* 0 means the board derives a unique ID from its Wi-Fi STA MAC address. */
#define APP_SELF_VEHICLE_ID        0u
#define APP_SELF_UWB_ID            0u
#define APP_PLATOON_ID             7u
#define APP_PLATOON_ENABLE         1u
#define APP_PLATOON_ROLE           1u
#define APP_PLATOON_INDEX          1u

#define APP_MAX_VEHICLES           16
#define APP_ESPNOW_CHANNEL         1

#define APP_TDMA_CYCLE_MS          100u
#define APP_TDMA_SLOT_MS           25u
#define APP_TDMA_GUARD_MS          5u

#define APP_ESPNOW_TX_PERIOD_MS    500u
#define APP_PI_TX_PERIOD_MS        200u
#define APP_MONITOR_PERIOD_MS      500u
#define APP_VEHICLE_TIMEOUT_MS     3000u

#define APP_ENABLE_UWB             1
#define APP_ENABLE_UWB_MOCK        0
#define APP_ENABLE_UWB_PDOA_TEST   1
#define APP_ENABLE_PI_BRIDGE       1
#define APP_ENABLE_DEBUG_SERIAL_LOGS 0

#define APP_UWB_SPI_HOST           SPI2_HOST
#define APP_UWB_PIN_SPI_CLK        10
#define APP_UWB_PIN_SPI_MISO       11
#define APP_UWB_PIN_SPI_MOSI       12
#define APP_UWB_PIN_SPI_CS         13
#define APP_UWB_PIN_IRQ            5
#define APP_UWB_PIN_RST            4
#define APP_UWB_SPI_FREQ_HZ        500000
