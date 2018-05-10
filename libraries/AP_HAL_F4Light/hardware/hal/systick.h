#ifndef _SYSTICK_H
#define _SYSTICK_H

#include <stm32f4xx.h>
#include <hal_types.h>
#include <hal.h>


#ifdef __cplusplus
  extern "C" {
#endif

/** System elapsed time, in milliseconds */
extern volatile uint64_t systick_uptime_millis;
extern voidFuncPtr boardEmergencyHandler;
extern void emerg_delay(uint32_t);


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
static inline void systick_enable() {   SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; }

/**
 * Clock the system timer with the core clock, but don't turn it
 * on or enable interrupt.
 */
static inline void systick_disable() {  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk; }

/**
 * @brief Returns the system uptime, in milliseconds.
 */
static inline uint64_t systick_uptime(void) {  return systick_uptime_millis; }
/**
 * @brief Returns the current value of the SysTick counter.
 */
static inline uint32_t systick_get_count(void) {  return SysTick->VAL; }

/**
 * @brief Check for underflow.
 *
 * This function returns 1 if the SysTick timer has counted to 0 since
 * the last time it was called.  However, any reads of any part of the
 * SysTick Control and Status Register SYSTICK_BASE->CSR will
 * interfere with this functionality.  See the ARM Cortex M3 Technical
 * Reference Manual for more details (e.g. Table 8-3 in revision r1p1).
 */
static inline uint32_t systick_check_underflow(void) { return SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk; }

/**
 * @brief Attach a callback to be called from the SysTick exception handler.
 *
 * To detach a callback, call this function again with a null argument.
 */
void systick_attach_callback(Handler callback);
void systick_detach_callback(Handler callback);

uint32_t systick_micros(void);

void SysTick_Handler(void);


/*
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
*/
void __attribute__((noreturn)) __error(uint32_t pc, uint32_t num, uint32_t lr, uint32_t flag);
void __attribute__((noreturn)) error_throb(uint32_t num);


#ifdef __cplusplus
  }
#endif

#endif
