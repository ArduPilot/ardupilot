/**
 * @file delay.h
 * @brief Delay implementation
 */
#ifndef _DELAY_H_
#define _DELAY_H_

#include "hal_types.h"
#include "stm32.h"

#ifdef __cplusplus
  extern "C" {
#endif
 
#if 0

/**
 * @brief Delay the given number of microseconds.
 *
 * @param us Number of microseconds to delay.
 */
static inline void delay_us(uint32_t us) {
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    //us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r" (us)
                 : "r0");
}
#endif

static inline void delay_ns100(uint32_t us) {
    us *= STM32_DELAY_US_MULT;
    us /= 10;

    /* fudge for function call overhead  */
    //us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r" (us)
                 : "r0");
}

#ifdef __cplusplus
  }
#endif
 

#endif

