/*
  supply voltage monitoring with the programmable voltage detector

  The PVD output is on EXTI line 16. The line is unmasked in the EXTI
  so the pending bit latches a dip, but its NVIC vector is left
  disabled so no interrupt is taken; stm32_supply_dip_detect_poll()
  reads and clears the pending bit instead. The NVIC also marks the
  PVD interrupt pending and that is never cleared, which is harmless
  while the vector stays disabled. Nothing here resets the MCU.
 */

#include "hal.h"
#include "supply_dip_detect.h"

#if AP_MCU_SUPPLY_DIP_DETECT_ENABLED

#define DIP_EXTI_LINE (1U<<16)

/*
  PLS level for a falling threshold of about 2.8V (typical):
  F1 7:2.78V, F4/F7 5:2.84V, H7 6:2.76V
 */
#ifndef HAL_SUPPLY_DIP_DETECT_PLS
#if defined(STM32F1)
#define HAL_SUPPLY_DIP_DETECT_PLS 7
#elif defined(STM32F4) || defined(STM32F7)
#define HAL_SUPPLY_DIP_DETECT_PLS 5
#elif defined(STM32H7)
#define HAL_SUPPLY_DIP_DETECT_PLS 6
#endif // defined(STM32F1)
#endif // HAL_SUPPLY_DIP_DETECT_PLS

#if HAL_SUPPLY_DIP_DETECT_PLS < 0 || HAL_SUPPLY_DIP_DETECT_PLS > 7
#error "HAL_SUPPLY_DIP_DETECT_PLS must be 0 to 7"
#endif // HAL_SUPPLY_DIP_DETECT_PLS < 0 || HAL_SUPPLY_DIP_DETECT_PLS > 7
#if defined(STM32H7) && HAL_SUPPLY_DIP_DETECT_PLS == 7
#error "HAL_SUPPLY_DIP_DETECT_PLS 7 selects the PVD_IN pin on H7"
#endif // defined(STM32H7) && HAL_SUPPLY_DIP_DETECT_PLS == 7

static bool exti_armed;

void stm32_supply_dip_detect_init(void)
{
    // PWR and EXTI registers are also changed by other code
    chSysLock();
#if defined(STM32H7)
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_PLS) | (HAL_SUPPLY_DIP_DETECT_PLS << PWR_CR1_PLS_Pos) | PWR_CR1_PVDEN;
#else
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
#if defined(STM32F7)
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_PLS) | (HAL_SUPPLY_DIP_DETECT_PLS << PWR_CR1_PLS_Pos) | PWR_CR1_PVDE;
#else
    PWR->CR = (PWR->CR & ~PWR_CR_PLS) | (HAL_SUPPLY_DIP_DETECT_PLS << PWR_CR_PLS_Pos) | PWR_CR_PVDE;
#endif // defined(STM32F7)
#endif // defined(STM32H7)
    chSysUnlock();
}

/*
  route the PVD output to the EXTI pending bit. This is done on the
  first poll rather than in stm32_supply_dip_detect_init() so the comparator has
  settled and its startup can't latch a false dip
 */
static void dip_exti_arm(void)
{
    chSysLock();
    // the PVD output rises when VDD falls below the threshold
#if defined(STM32H7)
    EXTI->RTSR1 |= DIP_EXTI_LINE;
    EXTI_D1->PR1 = DIP_EXTI_LINE;
    EXTI_D1->IMR1 |= DIP_EXTI_LINE;
#else
    EXTI->RTSR |= DIP_EXTI_LINE;
    EXTI->PR = DIP_EXTI_LINE;
    EXTI->IMR |= DIP_EXTI_LINE;
#endif // defined(STM32H7)
    chSysUnlock();
}

bool stm32_supply_dip_detect_poll(void)
{
    if (!exti_armed) {
        dip_exti_arm();
        exti_armed = true;
        return false;
    }
#if defined(STM32H7)
    if ((EXTI_D1->PR1 & DIP_EXTI_LINE) == 0) {
        return false;
    }
    EXTI_D1->PR1 = DIP_EXTI_LINE;
#else
    if ((EXTI->PR & DIP_EXTI_LINE) == 0) {
        return false;
    }
    EXTI->PR = DIP_EXTI_LINE;
#endif // defined(STM32H7)
    return true;
}

#endif // AP_MCU_SUPPLY_DIP_DETECT_ENABLED
