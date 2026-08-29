#pragma once

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t state;
    uint8_t destination_id;
    uint8_t platoon_state;

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

#define V2X_MSG_BASIC_STATUS       0x01u

#define V2X_STATE_WAIT             0u
#define V2X_STATE_SOLO             1u
#define V2X_STATE_JOIN             2u
#define V2X_STATE_KEEP             3u
#define V2X_STATE_EXIT             4u
#define V2X_STATE_PARKING          5u

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
