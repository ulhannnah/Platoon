#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t espnow_manager_init(void);
esp_err_t espnow_manager_send_self_status(void);
uint32_t espnow_manager_get_self_vehicle_id(void);
uint32_t espnow_manager_get_self_uwb_id(void);
