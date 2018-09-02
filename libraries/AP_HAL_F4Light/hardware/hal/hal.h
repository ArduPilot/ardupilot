#ifndef _HAL_H_
#define _HAL_H_

#include <errno.h>

#include "hal_types.h"
#include "stm32.h"
#include "stopwatch.h"
#include "util.h"
#include "gpio_hal.h"
#include "delay.h"
#include "adc.h"
#include "systick.h"
#include "nvic.h"
#include "mpu.h"
#include <boards.h>


#ifdef __cplusplus
  extern "C" {
#endif

extern void clock_gettime(uint32_t a1, void *a2); // to mimics Posix

extern void revo_call_handler(Handler h, uint32_t arg);
extern void hal_yield(uint16_t ttw);
extern void hal_delay(uint16_t t);
extern void hal_delay_microseconds(uint16_t t);
extern void hal_isr_time(uint32_t t);
extern uint32_t hal_micros();
extern int printf(const char *msg, ...);

#ifdef __cplusplus
  }
#endif

#ifdef ISR_PROF
    extern uint64_t isr_time;
#endif


#endif

