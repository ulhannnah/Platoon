#include "system_monitor.h"

#include "app_config.h"
#include "esp_timer.h"
#include "vehicle_table.h"

void system_monitor_tick(void)
{
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    vehicle_table_remove_stale(now_ms, APP_VEHICLE_TIMEOUT_MS);
}
