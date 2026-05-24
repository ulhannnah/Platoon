#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "packet_defs.h"

esp_err_t uwb_manager_init(void);
esp_err_t uwb_manager_poll(uwb_result_t *out_result);
bool uwb_manager_is_available(void);
