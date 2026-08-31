#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define ESPNOW_CHANNEL 1
#define TX_PERIOD_MS 500

#define V2X_MSG_BASIC_STATUS 0x01u
#define V2X_STATE_SOLO 1u
#define V2X_STATE_KEEP 3u

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

typedef struct {
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
} dummy_status_t;

static const char *TAG = "dummy_espnow";
static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static SemaphoreHandle_t s_lock;
static dummy_status_t s_status = {
    .vehicle_id = 202,
    .uwb_id = 40962,
    .destination_id = 3,
    .driving_state = V2X_STATE_KEEP,
    .platoon_state = V2X_STATE_KEEP,
    .speed_mps = 0.45f,
    .heading_deg = 0.0f,
    .platoon_enable = 1,
    .platoon_id = 7,
    .platoon_role = 2,
    .platoon_index = 1,
};
static uint16_t s_seq;

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

static void print_json_line(const char *line)
{
    size_t len = strlen(line);
    size_t sent = 0;
    while (sent < len) {
        int n = usb_serial_jtag_write_bytes(line + sent, len - sent, pdMS_TO_TICKS(100));
        if (n > 0) {
            sent += (size_t)n;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    usb_serial_jtag_write_bytes("\n", 1, pdMS_TO_TICKS(100));
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
}

static void usb_serial_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_use_driver();
}

static void apply_json_status(const char *line)
{
    cJSON *root = cJSON_Parse(line);
    if (root == NULL) {
        print_json_line("{\"type\":\"dummy_error\",\"reason\":\"invalid_json\"}");
        return;
    }

    const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (!cJSON_IsString(type) || strcmp(type->valuestring, "self_status") != 0) {
        cJSON_Delete(root);
        print_json_line("{\"type\":\"dummy_error\",\"reason\":\"unsupported_type\"}");
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.vehicle_id = json_u32(root, "vehicle_id", s_status.vehicle_id);
    s_status.uwb_id = json_u32(root, "uwb_id", s_status.uwb_id);
    s_status.destination_id = json_u8(root, "destination_id", s_status.destination_id);
    s_status.driving_state = json_u8(root, "driving_state", s_status.driving_state);
    s_status.platoon_state = json_u8(root, "platoon_state", s_status.platoon_state);
    s_status.speed_mps = json_float(root, "speed_mps", s_status.speed_mps);
    s_status.heading_deg = json_float(root, "heading_deg", s_status.heading_deg);
    s_status.platoon_enable = json_u8(root, "platoon_enable", s_status.platoon_enable);
    s_status.platoon_id = json_u32(root, "platoon_id", s_status.platoon_id);
    s_status.platoon_role = json_u8(root, "platoon_role", s_status.platoon_role);
    s_status.platoon_index = json_u8(root, "platoon_index", s_status.platoon_index);
    xSemaphoreGive(s_lock);

    cJSON_Delete(root);
    print_json_line("{\"type\":\"dummy_ack\",\"status\":\"self_status_applied\"}");
}

static esp_err_t wifi_espnow_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE), TAG, "wifi channel failed");

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "espnow init failed");

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        return err;
    }

    return ESP_OK;
}

static void task_rx_json(void *arg)
{
    (void)arg;
    char line[512];
    while (fgets(line, sizeof(line), stdin) != NULL) {
        if (line[0] == '{') {
            apply_json_status(line);
        }
    }
    vTaskDelete(NULL);
}

static void task_tx_espnow(void *arg)
{
    (void)arg;
    while (true) {
        dummy_status_t status;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        status = s_status;
        xSemaphoreGive(s_lock);

        vehicle_status_packet_t packet = {
            .msg_type = V2X_MSG_BASIC_STATUS,
            .state = status.driving_state,
            .destination_id = status.destination_id,
            .platoon_state = status.platoon_state,
            .vehicle_id = status.vehicle_id,
            .uwb_id = status.uwb_id,
            .platoon_id = status.platoon_id,
            .platoon_enable = status.platoon_enable,
            .platoon_role = status.platoon_role,
            .platoon_index = status.platoon_index,
            .speed_mps = status.speed_mps,
            .heading_deg = status.heading_deg,
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
            .seq = s_seq++,
        };

        esp_now_send(BROADCAST_MAC, (const uint8_t *)&packet, sizeof(packet));

        char ack[256];
        snprintf(ack, sizeof(ack),
                 "{\"type\":\"dummy_tx\",\"vehicle_id\":%" PRIu32 ",\"uwb_id\":%" PRIu32 ",\"speed_mps\":%.2f,\"heading_deg\":%.1f,\"seq\":%" PRIu16 "}",
                 packet.vehicle_id,
                 packet.uwb_id,
                 packet.speed_mps,
                 packet.heading_deg,
                 packet.seq);
        print_json_line(ack);

        vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_MS));
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    usb_serial_init();
    s_lock = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(wifi_espnow_init());

    print_json_line("{\"type\":\"dummy_ready\",\"role\":\"pc_espnow_sender\"}");
    xTaskCreate(task_rx_json, "rx_json", 4096, NULL, 4, NULL);
    xTaskCreate(task_tx_espnow, "tx_espnow", 4096, NULL, 5, NULL);
}
