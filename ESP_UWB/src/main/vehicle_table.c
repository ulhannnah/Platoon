#include "vehicle_table.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static tracked_vehicle_t s_table[APP_MAX_VEHICLES];
static SemaphoreHandle_t s_lock;

static tracked_vehicle_t *find_by_vehicle_id(uint32_t vehicle_id)
{
    for (size_t i = 0; i < APP_MAX_VEHICLES; ++i) {
        if (s_table[i].valid && s_table[i].vehicle_id == vehicle_id) {
            return &s_table[i];
        }
    }
    return NULL;
}

static tracked_vehicle_t *find_by_uwb_id(uint32_t uwb_id)
{
    for (size_t i = 0; i < APP_MAX_VEHICLES; ++i) {
        if (s_table[i].valid && s_table[i].uwb_id == uwb_id) {
            return &s_table[i];
        }
    }
    return NULL;
}

static tracked_vehicle_t *alloc_slot(void)
{
    for (size_t i = 0; i < APP_MAX_VEHICLES; ++i) {
        if (!s_table[i].valid) {
            memset(&s_table[i], 0, sizeof(s_table[i]));
            s_table[i].valid = 1;
            return &s_table[i];
        }
    }
    return NULL;
}

void vehicle_table_init(void)
{
    memset(s_table, 0, sizeof(s_table));
    s_lock = xSemaphoreCreateMutex();
}

void vehicle_table_update_status(const vehicle_status_packet_t *packet, uint32_t now_ms)
{
    if (packet == NULL || packet->vehicle_id == APP_SELF_VEHICLE_ID) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    tracked_vehicle_t *item = find_by_vehicle_id(packet->vehicle_id);
    if (item == NULL) {
        item = alloc_slot();
    }

    if (item != NULL) {
        item->vehicle_id = packet->vehicle_id;
        item->uwb_id = packet->uwb_id;
        item->speed_mps = packet->speed_mps;
        item->heading_deg = packet->heading_deg;
        item->platoon_id = packet->platoon_id;
        item->platoon_enable = packet->platoon_enable;
        item->platoon_role = packet->platoon_role;
        item->platoon_index = packet->platoon_index;
        item->last_espnow_ms = now_ms;
        item->confidence = item->last_uwb_ms ? 0.9f : 0.5f;
    }
    xSemaphoreGive(s_lock);
}

bool vehicle_table_update_uwb(const uwb_result_t *result, uint32_t now_ms)
{
    if (result == NULL || !result->valid) {
        return false;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    tracked_vehicle_t *item = find_by_uwb_id(result->target_uwb_id);
    if (item == NULL) {
        item = alloc_slot();
        if (item != NULL) {
            item->uwb_id = result->target_uwb_id;
        }
    }

    bool updated = false;
    if (item != NULL) {
        const float angle_rad = result->angle_deg * (float)M_PI / 180.0f;
        item->distance_m = result->distance_m;
        item->angle_deg = result->angle_deg;
        item->rel_x_m = result->distance_m * cosf(angle_rad);
        item->rel_y_m = result->distance_m * sinf(angle_rad);
        item->last_uwb_ms = now_ms;
        item->confidence = item->last_espnow_ms ? 0.9f : 0.6f;
        updated = true;
    }
    xSemaphoreGive(s_lock);
    return updated;
}

size_t vehicle_table_snapshot(tracked_vehicle_t *out, size_t max_count)
{
    if (out == NULL || max_count == 0) {
        return 0;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t count = 0;
    for (size_t i = 0; i < APP_MAX_VEHICLES && count < max_count; ++i) {
        if (s_table[i].valid) {
            out[count++] = s_table[i];
        }
    }
    xSemaphoreGive(s_lock);
    return count;
}

void vehicle_table_remove_stale(uint32_t now_ms, uint32_t timeout_ms)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (size_t i = 0; i < APP_MAX_VEHICLES; ++i) {
        if (!s_table[i].valid) {
            continue;
        }

        const uint32_t last_seen = s_table[i].last_espnow_ms > s_table[i].last_uwb_ms
            ? s_table[i].last_espnow_ms
            : s_table[i].last_uwb_ms;
        if ((now_ms - last_seen) > timeout_ms) {
            memset(&s_table[i], 0, sizeof(s_table[i]));
        }
    }
    xSemaphoreGive(s_lock);
}
