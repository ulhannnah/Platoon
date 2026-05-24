#pragma once

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint32_t vehicle_id;
    uint32_t uwb_id;

    uint32_t platoon_id;
    uint8_t platoon_enable;
    uint8_t platoon_role;
    uint8_t platoon_index;

    float speed_mps;
    float heading_deg;

    uint32_t timestamp_ms;
    uint16_t seq;
} vehicle_status_packet_t;

typedef struct {
    uint32_t target_uwb_id;

    float distance_m;
    float angle_deg;

    float fp_power;
    float rx_power;
    int16_t rssi;

    uint32_t timestamp_ms;
    uint16_t seq;

    uint8_t valid;
} uwb_result_t;
