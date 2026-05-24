#include "tdma_scheduler.h"

#include "app_config.h"

static uint8_t s_my_slot;

void tdma_scheduler_init(uint8_t my_slot)
{
    s_my_slot = my_slot;
}

bool tdma_scheduler_is_my_uwb_slot(uint32_t now_ms)
{
    const uint32_t t = now_ms % APP_TDMA_CYCLE_MS;
    const uint8_t current_slot = (uint8_t)(t / APP_TDMA_SLOT_MS);
    const uint32_t slot_elapsed = t % APP_TDMA_SLOT_MS;

    return current_slot == s_my_slot && slot_elapsed < (APP_TDMA_SLOT_MS - APP_TDMA_GUARD_MS);
}
