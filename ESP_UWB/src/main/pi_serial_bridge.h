#pragma once

#include "esp_err.h"

esp_err_t pi_serial_bridge_init(void);
void pi_serial_bridge_send_snapshot(void);
void pi_serial_bridge_rx_loop(void);
