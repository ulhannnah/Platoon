#include "pi_serial_bridge.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "cJSON.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "espnow_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "packet_defs.h"
#include "vehicle_table.h"

static const char *TAG = "pi_serial_bridge";
static uint32_t s_pi_tx_seq;
static char s_tx_line[4096];

esp_err_t pi_serial_bridge_init(void)
{
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = {
            .tx_buffer_size = 4096,
            .rx_buffer_size = 1024,
        };
        esp_err_t err = usb_serial_jtag_driver_install(&cfg);
        if (err != ESP_OK) {
            return err;
        }
    }

    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_use_driver();
    return ESP_OK;
}

static void pi_serial_write_all(const char *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        const int n = usb_serial_jtag_write_bytes(data + sent, len - sent, pdMS_TO_TICKS(100));
        if (n > 0) {
            sent += (size_t)n;
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
}

static void appendf(char *buf, size_t buf_size, size_t *pos, const char *fmt, ...)
{
    if (*pos >= buf_size) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(buf + *pos, buf_size - *pos, fmt, args);
    va_end(args);

    if (n < 0) {
        return;
    }

    const size_t available = buf_size - *pos;
    if ((size_t)n >= available) {
        *pos = buf_size - 1;
    } else {
        *pos += (size_t)n;
    }
}

static uint32_t json_u32(const cJSON *root, const char *name, uint32_t fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    return cJSON_IsNumber(item) ? (uint32_t)item->valuedouble : fallback;
}

static uint8_t json_u8(const cJSON *root, const char *name, uint8_t fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    return cJSON_IsNumber(item) ? (uint8_t)item->valuedouble : fallback;
}

static float json_float(const cJSON *root, const char *name, float fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
    return cJSON_IsNumber(item) ? (float)item->valuedouble : fallback;
}

static void handle_self_status(const cJSON *root)
{
    pi_self_status_t status = {
        .seq = json_u32(root, "seq", 0),
        .timestamp_ms = json_u32(root, "timestamp_ms", 0),
        .vehicle_id = json_u32(root, "vehicle_id", espnow_manager_get_self_vehicle_id()),
        .uwb_id = json_u32(root, "uwb_id", espnow_manager_get_self_uwb_id()),
        .destination_id = json_u8(root, "destination_id", 0),
        .driving_state = json_u8(root, "driving_state", V2X_STATE_SOLO),
        .platoon_state = json_u8(root, "platoon_state", V2X_STATE_SOLO),
        .speed_mps = json_float(root, "speed_mps", 0.0f),
        .heading_deg = json_float(root, "heading_deg", 0.0f),
        .platoon_enable = json_u8(root, "platoon_enable", 0),
        .platoon_id = json_u32(root, "platoon_id", 0),
        .platoon_role = json_u8(root, "platoon_role", 0),
        .platoon_index = json_u8(root, "platoon_index", 0),
        .leader_vehicle_id = json_u32(root, "leader_vehicle_id", 0),
        .front_vehicle_id = json_u32(root, "front_vehicle_id", 0),
        .target_speed_mps = json_float(root, "target_speed_mps", 0.0f),
        .target_gap_m = json_float(root, "target_gap_m", 0.0f),
    };

    espnow_manager_update_self_status_from_pi(&status);
}

void pi_serial_bridge_send_snapshot(void)
{
    tracked_vehicle_t snapshot[APP_MAX_VEHICLES];
    const size_t count = vehicle_table_snapshot(snapshot, APP_MAX_VEHICLES);
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const uint32_t self_vehicle_id = espnow_manager_get_self_vehicle_id();
    size_t pos = 0;

    appendf(s_tx_line, sizeof(s_tx_line), &pos,
            "{\"type\":\"v2x_targets\",\"seq\":%" PRIu32 ",\"timestamp_ms\":%" PRIu32 ",\"self_vehicle_id\":%" PRIu32 ",\"targets\":[",
            s_pi_tx_seq++,
            now_ms,
            self_vehicle_id);

    size_t written = 0;
    for (size_t i = 0; i < count; ++i) {
        const tracked_vehicle_t *v = &snapshot[i];
        if (v->vehicle_id == self_vehicle_id) {
            continue;
        }

        const uint8_t uwb_valid = v->last_uwb_ms != 0;
        const uint8_t espnow_valid = v->last_espnow_ms != 0;
        appendf(s_tx_line, sizeof(s_tx_line), &pos,
                "%s{\"vehicle_id\":%" PRIu32 ",\"uwb_id\":%" PRIu32 ",\"distance_m\":%.2f,\"angle_deg\":%.2f,"
                "\"rel_x_m\":%.2f,\"rel_y_m\":%.2f,\"speed_mps\":%.2f,\"heading_deg\":%.2f,"
                "\"driving_state\":%" PRIu8 ",\"platoon_state\":%" PRIu8 ",\"platoon_id\":%" PRIu32 ","
                "\"platoon_enable\":%" PRIu8 ",\"platoon_role\":%" PRIu8 ",\"platoon_index\":%" PRIu8 ","
                "\"uwb_valid\":%" PRIu8 ",\"espnow_valid\":%" PRIu8 ",\"confidence\":%.2f}",
                written == 0 ? "" : ",",
                (uint32_t)v->vehicle_id,
                (uint32_t)v->uwb_id,
                v->distance_m,
                v->angle_deg,
                v->rel_x_m,
                v->rel_y_m,
                v->speed_mps,
                v->heading_deg,
                v->driving_state,
                v->platoon_state,
                (uint32_t)v->platoon_id,
                v->platoon_enable,
                v->platoon_role,
                v->platoon_index,
                uwb_valid,
                espnow_valid,
                v->confidence);
        ++written;
    }

    appendf(s_tx_line, sizeof(s_tx_line), &pos, "]}\n");
    pi_serial_write_all(s_tx_line, pos);
}

void pi_serial_bridge_rx_loop(void)
{
    char line[512];

    while (fgets(line, sizeof(line), stdin) != NULL) {
        if (line[0] != '{') {
            continue;
        }

        cJSON *root = cJSON_Parse(line);
        if (root == NULL) {
            ESP_LOGW(TAG, "invalid JSON from Pi");
            continue;
        }

        const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
        if (cJSON_IsString(type) && type->valuestring != NULL && strcmp(type->valuestring, "self_status") == 0) {
            handle_self_status(root);
        } else {
            ESP_LOGW(TAG, "unsupported Pi JSON type");
        }

        cJSON_Delete(root);
    }
}
