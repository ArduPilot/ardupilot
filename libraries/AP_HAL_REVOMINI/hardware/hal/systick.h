#ifndef _SYSTICK_H
#define _SYSTICK_H

#include <stm32f4xx.h>

#ifdef __cplusplus
  extern "C" {
#endif

/** System elapsed time, in milliseconds */
extern volatile uint64_t systick_uptime_millis;

extern volatile uint32_t uart1_lic_millis;
extern volatile uint32_t uart2_lic_millis;
extern volatile uint32_t uart3_lic_millis;
extern volatile uint32_t uart4_lic_millis;
extern volatile uint32_t uart5_lic_millis;
extern volatile uint32_t uart6_lic_millis;

/**
 * @brief Initialize and enable SysTick.
 *
 * Clocks the system timer with the core clock, turns it on, and
 * enables interrupts.
 *
 * @param reload_val Appropriate reload counter to tick every 1 ms.
 */
void systick_init(uint32_t reload_val);

/**
 * Clock the system timer with the core clock and turn it on;
 * interrupt every 1 ms, for systick_timer_millis.
 */
void systick_enable();

/**
 * Clock the system timer with the core clock, but don't turn it
 * on or enable interrupt.
 */
void systick_disable();

/**
 * @brief Returns the system uptime, in milliseconds.
 */
static inline uint64_t systick_uptime(void) {
    return systick_uptime_millis;
}
/**
 * @brief Returns the current value of the SysTick counter.
 */
static inline uint32_t systick_get_count(void) {
    return SysTick->VAL;
}

/**
 * @brief Check for underflow.
 *
 * This function returns 1 if the SysTick timer has counted to 0 since
 * the last time it was called.  However, any reads of any part of the
 * SysTick Control and Status Register SYSTICK_BASE->CSR will
 * interfere with this functionality.  See the ARM Cortex M3 Technical
 * Reference Manual for more details (e.g. Table 8-3 in revision r1p1).
 */
static inline uint32_t systick_check_underflow(void) {
    return SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;
}

/**
 * @brief Attach a callback to be called from the SysTick exception handler.
 *
 * To detach a callback, call this function again with a null argument.
 */
void systick_attach_callback(void (*callback)(void));

#ifdef __cplusplus
  }
#endif

#endif
