#include "id_matcher.h"

#include "esp_timer.h"
#include "vehicle_table.h"

bool id_matcher_apply_uwb_result(const uwb_result_t *result)
{
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return vehicle_table_update_uwb(result, now_ms);
}
