#include "pi_serial_bridge.h"

#include <inttypes.h>
#include <stdio.h>

#include "app_config.h"
#include "esp_timer.h"
#include "vehicle_table.h"

void pi_serial_bridge_send_snapshot(void)
{
    tracked_vehicle_t snapshot[APP_MAX_VEHICLES];
    const size_t count = vehicle_table_snapshot(snapshot, APP_MAX_VEHICLES);
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    printf("{\"self_vehicle_id\":%" PRIu32 ",\"timestamp_ms\":%" PRIu32 ",\"targets\":[",
           (uint32_t)APP_SELF_VEHICLE_ID,
           now_ms);

    for (size_t i = 0; i < count; ++i) {
        const tracked_vehicle_t *v = &snapshot[i];
        printf("%s{\"vehicle_id\":%" PRIu32 ",\"uwb_id\":%" PRIu32 ",\"distance_m\":%.2f,\"angle_deg\":%.2f,"
               "\"rel_x_m\":%.2f,\"rel_y_m\":%.2f,\"speed_mps\":%.2f,\"heading_deg\":%.2f,"
               "\"platoon_id\":%" PRIu32 ",\"confidence\":%.2f}",
               i == 0 ? "" : ",",
               (uint32_t)v->vehicle_id,
               (uint32_t)v->uwb_id,
               v->distance_m,
               v->angle_deg,
               v->rel_x_m,
               v->rel_y_m,
               v->speed_mps,
               v->heading_deg,
               (uint32_t)v->platoon_id,
               v->confidence);
    }

    printf("]}\n");
}
