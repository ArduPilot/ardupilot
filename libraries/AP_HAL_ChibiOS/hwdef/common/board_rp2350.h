#pragma once

#if defined(RP2350)
void rp2350_board_early_init(void);
void rp2350_board_pre_hal_init(void);
void rp2350_board_post_hal_init(void);
void rp2350_board_init(void);
#endif

#if defined(RP2350) && defined(RP_CORE1_START) && RP_CORE1_START == TRUE
void c1_startup_verify(void);
#endif
