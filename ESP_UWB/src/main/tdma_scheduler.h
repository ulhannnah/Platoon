#pragma once

#include <stdbool.h>
#include <stdint.h>

void tdma_scheduler_init(uint8_t my_slot);
bool tdma_scheduler_is_my_uwb_slot(uint32_t now_ms);
