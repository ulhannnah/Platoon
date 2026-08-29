#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "packet_defs.h"

typedef struct {
    uint32_t vehicle_id;
    uint32_t uwb_id;

    float distance_m;
    float angle_deg;
    float rel_x_m;
    float rel_y_m;

    float speed_mps;
    float heading_deg;

    uint8_t driving_state;
    uint8_t platoon_state;

    uint32_t platoon_id;
    uint8_t platoon_enable;
    uint8_t platoon_role;
    uint8_t platoon_index;

    uint32_t last_uwb_ms;
    uint32_t last_espnow_ms;

    float confidence;
    uint8_t valid;
} tracked_vehicle_t;

void vehicle_table_init(void);
void vehicle_table_update_status(const vehicle_status_packet_t *packet, uint32_t now_ms);
bool vehicle_table_update_uwb(const uwb_result_t *result, uint32_t now_ms);
size_t vehicle_table_snapshot(tracked_vehicle_t *out, size_t max_count);
void vehicle_table_remove_stale(uint32_t now_ms, uint32_t timeout_ms);
