#pragma once

#include "esp_err.h"

esp_err_t espnow_manager_init(void);
esp_err_t espnow_manager_send_self_status(void);
