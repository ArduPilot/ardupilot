#ifndef _EXTI_H_
#define _EXTI_H_

#include "gpio_hal.h"
#include "hal_types.h"

#ifdef __cplusplus
  extern "C" {
#endif
 
/**
 * @brief Register a handler to run upon external interrupt.
 *
 * This function assumes that the interrupt request corresponding to
 * the given external interrupt is masked.
 *
 * @param num     External interrupt line number.
 * @param port    Port to use as source input for external interrupt.
 * @param handler Function handler to execute when interrupt is triggered.
 * @param mode    Type of transition to trigger on, one of:
 *                EXTI_RISING, EXTI_FALLING, EXTI_RISING_FALLING.
 * @see afio_exti_num
 * @see afio_exti_port
 * @see voidFuncPtr
 * @see exti_trigger_mode
 */
void exti_attach_interrupt(afio_exti_num num,
                           afio_exti_port port,
                           voidFuncPtr handler,
                           exti_trigger_mode mode);

/**
 * @brief Unregister an external interrupt handler
 * @param num Number of the external interrupt line to disable.
 * @see afio_exti_num
 */
void exti_detach_interrupt(afio_exti_num num);
                       
#ifdef __cplusplus
  }
#endif
 

#endif
