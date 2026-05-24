#include "espnow_manager.h"

#include <string.h>

#include "app_config.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "packet_defs.h"
#include "vehicle_table.h"

static const char *TAG = "espnow_manager";
static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static uint16_t s_seq;

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;
    if (data == NULL || len != sizeof(vehicle_status_packet_t)) {
        return;
    }

    vehicle_status_packet_t packet;
    memcpy(&packet, data, sizeof(packet));
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    vehicle_table_update_status(&packet, now_ms);
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

    return ESP_OK;
}

esp_err_t espnow_manager_init(void)
{
    ESP_RETURN_ON_ERROR(wifi_init_for_espnow(), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "esp-now init failed");
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
    vehicle_status_packet_t packet = {
        .vehicle_id = APP_SELF_VEHICLE_ID,
        .uwb_id = APP_SELF_UWB_ID,
        .platoon_id = APP_PLATOON_ID,
        .platoon_enable = APP_PLATOON_ENABLE,
        .platoon_role = APP_PLATOON_ROLE,
        .platoon_index = APP_PLATOON_INDEX,
        .speed_mps = 0.0f,
        .heading_deg = 0.0f,
        .timestamp_ms = now_ms,
        .seq = s_seq++,
    };

    return esp_now_send(BROADCAST_MAC, (const uint8_t *)&packet, sizeof(packet));
}
