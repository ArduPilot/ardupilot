#pragma once

#include <stdint.h>

#if defined(RP2350)
void rp2350_board_early_init(void);
void rp2350_board_pre_hal_init(void);
void rp2350_board_post_hal_init(void);
void rp2350_board_init(void);
void rp2350_xip_cache_stats(uint32_t *hit, uint32_t *acc);
#endif

