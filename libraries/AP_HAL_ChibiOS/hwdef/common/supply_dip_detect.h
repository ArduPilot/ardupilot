#pragma once

#include "hal.h"

/*
  detect dips in the MCU supply (VDD) using the STM32 Programmable
  Voltage Detector (PVD), a comparator against a selectable threshold.
  This only records that VDD fell below the threshold; it never resets
  the MCU
 */
#ifndef AP_MCU_SUPPLY_DIP_DETECT_ENABLED
#if (defined(STM32F1) || defined(STM32F4) || defined(STM32F7) || defined(STM32H7)) && \
    !defined(HAL_BOOTLOADER_BUILD)
#define AP_MCU_SUPPLY_DIP_DETECT_ENABLED 1
#else
#define AP_MCU_SUPPLY_DIP_DETECT_ENABLED 0
#endif // STM32F1, F4, F7 or H7 and not the bootloader
#endif // AP_MCU_SUPPLY_DIP_DETECT_ENABLED

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#if AP_MCU_SUPPLY_DIP_DETECT_ENABLED
/*
  start the comparator at a falling threshold of about 2.8V
 */
void stm32_supply_dip_detect_init(void);

/*
  return true if VDD has fallen below the threshold since the last
  call. The EXTI pending bit latches the event, so a dip shorter than
  the polling interval is still seen. Call from one thread only; the
  first call arms the latch and returns false
 */
bool stm32_supply_dip_detect_poll(void);
#endif // AP_MCU_SUPPLY_DIP_DETECT_ENABLED

#ifdef __cplusplus
}
#endif // __cplusplus
