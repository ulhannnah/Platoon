#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "espnow_manager.h"
#include "id_matcher.h"
#include "pi_serial_bridge.h"
#include "system_monitor.h"
#include "tdma_scheduler.h"
#if APP_ENABLE_UWB
#include "uwb_manager.h"
#endif
#include "vehicle_table.h"

static const char *TAG = "app_main";

static void task_espnow_tx(void *arg)
{
    (void)arg;
    while (true) {
        esp_err_t err = espnow_manager_send_self_status();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ESP-NOW send failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(APP_ESPNOW_TX_PERIOD_MS));
    }
}

#if APP_ENABLE_UWB
static void task_uwb(void *arg)
{
    (void)arg;
    while (true) {
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (uwb_manager_is_available() && tdma_scheduler_is_my_uwb_slot(now_ms)) {
            uwb_result_t result = {0};
            if (uwb_manager_poll(&result) == ESP_OK) {
                id_matcher_apply_uwb_result(&result);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

#if APP_ENABLE_PI_BRIDGE
static void task_pi_tx(void *arg)
{
    (void)arg;
    while (true) {
        pi_serial_bridge_send_snapshot();
        vTaskDelay(pdMS_TO_TICKS(APP_PI_TX_PERIOD_MS));
    }
}
#endif

static void task_monitor(void *arg)
{
    (void)arg;
    while (true) {
        system_monitor_tick();
        vTaskDelay(pdMS_TO_TICKS(APP_MONITOR_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 V2V gateway boot");

    vehicle_table_init();
    tdma_scheduler_init(APP_PLATOON_INDEX);

#if APP_ENABLE_UWB
    esp_err_t uwb_err = uwb_manager_init();
    if (uwb_err != ESP_OK) {
        ESP_LOGW(TAG, "UWB unavailable, continuing with Wi-Fi/ESP-NOW only");
    }
#else
    ESP_LOGI(TAG, "UWB disabled, running ESP-NOW only");
#endif

    ESP_ERROR_CHECK(espnow_manager_init());

    xTaskCreate(task_espnow_tx, "espnow_tx", 4096, NULL, 5, NULL);
#if APP_ENABLE_UWB
    xTaskCreate(task_uwb, "uwb", 4096, NULL, 5, NULL);
#endif
#if APP_ENABLE_PI_BRIDGE
    xTaskCreate(task_pi_tx, "pi_tx", 4096, NULL, 4, NULL);
#endif
    xTaskCreate(task_monitor, "monitor", 3072, NULL, 3, NULL);
}
