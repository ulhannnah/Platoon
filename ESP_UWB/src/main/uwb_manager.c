#include "uwb_manager.h"

#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "uwb_manager";
static bool s_available;
#if APP_ENABLE_UWB_MOCK
static uint16_t s_mock_seq;
#endif

esp_err_t uwb_manager_init(void)
{
#if APP_ENABLE_UWB
#if APP_ENABLE_UWB_MOCK
    s_available = true;
    ESP_LOGW(TAG, "UWB mock mode enabled");
    return ESP_OK;
#else
    s_available = false;
    ESP_LOGW(TAG, "DW3000 driver is not linked yet; running Wi-Fi/ESP-NOW only");
    return ESP_ERR_NOT_SUPPORTED;
#endif
#else
    s_available = false;
    ESP_LOGI(TAG, "UWB disabled by app_config.h");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t uwb_manager_poll(uwb_result_t *out_result)
{
    if (out_result == NULL || !s_available) {
        return ESP_ERR_INVALID_STATE;
    }

#if APP_ENABLE_UWB_MOCK
    const int64_t now_ms = esp_timer_get_time() / 1000;
    out_result->target_uwb_id = 0xA002u;
    out_result->distance_m = 2.0f + (float)((now_ms / 500) % 5) * 0.25f;
    out_result->angle_deg = -12.0f + (float)((now_ms / 300) % 8);
    out_result->fp_power = -82.0f;
    out_result->rx_power = -78.0f;
    out_result->rssi = -70;
    out_result->timestamp_ms = (uint32_t)now_ms;
    out_result->seq = s_mock_seq++;
    out_result->valid = 1;
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

bool uwb_manager_is_available(void)
{
    return s_available;
}
