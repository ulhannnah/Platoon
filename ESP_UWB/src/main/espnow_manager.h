#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint32_t seq;
    uint32_t timestamp_ms;
    uint32_t vehicle_id;
    uint32_t uwb_id;
    uint8_t destination_id;
    uint8_t driving_state;
    uint8_t platoon_state;
    float speed_mps;
    float heading_deg;
    uint8_t platoon_enable;
    uint32_t platoon_id;
    uint8_t platoon_role;
    uint8_t platoon_index;
    uint32_t leader_vehicle_id;
    uint32_t front_vehicle_id;
    float target_speed_mps;
    float target_gap_m;
} pi_self_status_t;

esp_err_t espnow_manager_init(void);
esp_err_t espnow_manager_send_self_status(void);
void espnow_manager_update_self_status_from_pi(const pi_self_status_t *status);
uint32_t espnow_manager_get_self_vehicle_id(void);
uint32_t espnow_manager_get_self_uwb_id(void);
