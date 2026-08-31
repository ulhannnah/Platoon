#include "espnow_manager.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "packet_defs.h"
#include "vehicle_table.h"

static const char *TAG = "espnow_manager";
static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static uint16_t s_seq;
static uint32_t s_self_vehicle_id;
static uint32_t s_self_uwb_id;
static pi_self_status_t s_pi_status;
static bool s_has_pi_status;
static SemaphoreHandle_t s_status_lock;

static void print_mac(const uint8_t *mac)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static uint32_t id_from_mac(const uint8_t *mac)
{
    uint32_t id = ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    return id == 0 ? 1 : id;
}

static void on_send(const uint8_t *mac_addr, esp_now_send_status_t status)
{
#if APP_ENABLE_DEBUG_SERIAL_LOGS
    printf("[ESP-NOW TX_CB] dst=");
    print_mac(mac_addr);
    printf(" status=%s\n", status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
#else
    (void)mac_addr;
    (void)status;
#endif
}

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (info == NULL || data == NULL || len != sizeof(vehicle_status_packet_t)) {
        ESP_LOGW(TAG, "invalid RX packet length=%d expected=%d", len, (int)sizeof(vehicle_status_packet_t));
        return;
    }

    vehicle_status_packet_t packet;
    memcpy(&packet, data, sizeof(packet));
    if (packet.msg_type != V2X_MSG_BASIC_STATUS || packet.vehicle_id == s_self_vehicle_id) {
        return;
    }

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    vehicle_table_update_status(&packet, now_ms);

#if APP_ENABLE_DEBUG_SERIAL_LOGS
    printf("\n========== ESP-NOW RX ==========\n");
    printf("from MAC       : ");
    print_mac(info->src_addr);
    printf("\n");
    printf("msg_type       : %" PRIu8 "\n", packet.msg_type);
    printf("vehicle_id     : %" PRIu32 "\n", packet.vehicle_id);
    printf("uwb_id         : %" PRIu32 "\n", packet.uwb_id);
    printf("driving_state  : %" PRIu8 "\n", packet.state);
    printf("platoon_state  : %" PRIu8 "\n", packet.platoon_state);
    printf("destination_id : %" PRIu8 "\n", packet.destination_id);
    printf("platoon_id     : %" PRIu32 "\n", packet.platoon_id);
    printf("platoon_enable : %" PRIu8 "\n", packet.platoon_enable);
    printf("platoon_role   : %" PRIu8 "\n", packet.platoon_role);
    printf("platoon_index  : %" PRIu8 "\n", packet.platoon_index);
    printf("speed_mps      : %.3f\n", packet.speed_mps);
    printf("heading_deg    : %.1f\n", packet.heading_deg);
    printf("seq            : %" PRIu16 "\n", packet.seq);
    printf("timestamp_ms   : %" PRIu32 "\n", packet.timestamp_ms);
    printf("================================\n\n");
#endif
}

static esp_err_t wifi_init_for_espnow(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init failed");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(APP_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE), TAG, "wifi channel failed");

    uint8_t mac[ESP_NOW_ETH_ALEN];
    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, mac), TAG, "get STA MAC failed");
    s_self_vehicle_id = APP_SELF_VEHICLE_ID == 0 ? id_from_mac(mac) : APP_SELF_VEHICLE_ID;
    s_self_uwb_id = APP_SELF_UWB_ID == 0 ? s_self_vehicle_id : APP_SELF_UWB_ID;

#if APP_ENABLE_DEBUG_SERIAL_LOGS
    printf("================================\n");
    printf("ESP32-S3 ESP-NOW BASIC TEST\n");
    printf("STA MAC        : ");
    print_mac(mac);
    printf("\n");
    printf("vehicle_id     : %" PRIu32 "\n", s_self_vehicle_id);
    printf("uwb_id         : %" PRIu32 "\n", s_self_uwb_id);
    printf("Wi-Fi channel  : %u\n", APP_ESPNOW_CHANNEL);
    printf("================================\n");
#else
    (void)print_mac;
#endif

    return ESP_OK;
}

esp_err_t espnow_manager_init(void)
{
    s_status_lock = xSemaphoreCreateMutex();

    ESP_RETURN_ON_ERROR(wifi_init_for_espnow(), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "esp-now init failed");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(on_send), TAG, "esp-now send callback failed");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_recv), TAG, "esp-now recv callback failed");

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    peer.channel = APP_ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_RETURN_ON_ERROR(err, TAG, "add broadcast peer failed");
    }

    ESP_LOGI(TAG, "ESP-NOW ready on channel %d", APP_ESPNOW_CHANNEL);
    return ESP_OK;
}

esp_err_t espnow_manager_send_self_status(void)
{
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const uint16_t seq = s_seq++;

    pi_self_status_t pi_status = {0};
    bool has_pi_status = false;
    if (s_status_lock != NULL) {
        xSemaphoreTake(s_status_lock, portMAX_DELAY);
        pi_status = s_pi_status;
        has_pi_status = s_has_pi_status;
        xSemaphoreGive(s_status_lock);
    }

    const uint8_t driving_state = has_pi_status ? pi_status.driving_state
                                                : ((seq % 10u) < 5u ? V2X_STATE_SOLO : V2X_STATE_WAIT);
    const uint8_t platoon_state = has_pi_status ? pi_status.platoon_state : driving_state;
    const uint8_t destination_id = has_pi_status ? pi_status.destination_id : (uint8_t)((seq % 4u) + 1u);
    const float speed_mps = has_pi_status ? pi_status.speed_mps : 0.30f + 0.01f * (float)(seq % 20u);
    const float heading_deg = has_pi_status ? pi_status.heading_deg : (float)((seq * 15u) % 360u);

    vehicle_status_packet_t packet = {
        .msg_type = V2X_MSG_BASIC_STATUS,
        .state = driving_state,
        .destination_id = destination_id,
        .platoon_state = platoon_state,
        .vehicle_id = has_pi_status && pi_status.vehicle_id != 0 ? pi_status.vehicle_id : s_self_vehicle_id,
        .uwb_id = has_pi_status && pi_status.uwb_id != 0 ? pi_status.uwb_id : s_self_uwb_id,
        .platoon_id = has_pi_status ? pi_status.platoon_id : APP_PLATOON_ID,
        .platoon_enable = has_pi_status ? pi_status.platoon_enable : APP_PLATOON_ENABLE,
        .platoon_role = has_pi_status ? pi_status.platoon_role : APP_PLATOON_ROLE,
        .platoon_index = has_pi_status ? pi_status.platoon_index : APP_PLATOON_INDEX,
        .speed_mps = speed_mps,
        .heading_deg = heading_deg,
        .timestamp_ms = now_ms,
        .seq = seq,
    };

    esp_err_t err = esp_now_send(BROADCAST_MAC, (const uint8_t *)&packet, sizeof(packet));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[TX] vehicle_id=%" PRIu32 " driving=%" PRIu8 " platoon=%" PRIu8 " dest=%" PRIu8
                 " speed=%.3f heading=%.1f seq=%" PRIu16,
                 packet.vehicle_id,
                 packet.state,
                 packet.platoon_state,
                 packet.destination_id,
                 packet.speed_mps,
                 packet.heading_deg,
                 packet.seq);
    }
    return err;
}

void espnow_manager_update_self_status_from_pi(const pi_self_status_t *status)
{
    if (status == NULL) {
        return;
    }

    if (s_status_lock != NULL) {
        xSemaphoreTake(s_status_lock, portMAX_DELAY);
        s_pi_status = *status;
        s_has_pi_status = true;
        if (s_pi_status.vehicle_id != 0) {
            s_self_vehicle_id = s_pi_status.vehicle_id;
        }
        if (s_pi_status.uwb_id != 0) {
            s_self_uwb_id = s_pi_status.uwb_id;
        }
        xSemaphoreGive(s_status_lock);
    }

    ESP_LOGI(TAG, "Pi self_status applied vehicle_id=%" PRIu32 " uwb_id=%" PRIu32
             " driving=%" PRIu8 " platoon=%" PRIu8 " speed=%.3f heading=%.1f",
             status->vehicle_id,
             status->uwb_id,
             status->driving_state,
             status->platoon_state,
             status->speed_mps,
             status->heading_deg);
}

uint32_t espnow_manager_get_self_vehicle_id(void)
{
    return s_self_vehicle_id;
}

uint32_t espnow_manager_get_self_uwb_id(void)
{
    return s_self_uwb_id;
}
