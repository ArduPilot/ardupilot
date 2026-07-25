/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    STM32MP1xx/hal_lld.c
 * @brief   STM32MP1xx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_stm32g4xx.h.
 */
uint32_t SystemCoreClock = 0; //STM32_HCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Configures the PWR unit.
 * @note    CR1, CR2 and CR5 are not initialized inside this function.
 */
__STATIC_INLINE void hal_lld_set_static_pwr(void) {

  /* Static PWR configurations.*/
  PWR->MCUCR        = STM32_PWR_MCUCR;
  PWR->MCUWKUPENR   = STM32_PWR_MCUWKUPENR;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* DMA subsystems initialization.*/
#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* NVIC initialization.*/
  nvicInit();

  /* IRQ subsystem initialization.*/
  irqInit();
}

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   STM32L4xx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {

#if !STM32_NO_INIT
#endif /* STM32_NO_INIT */
}

#else /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */
void stm32_clock_init(void) {

#if !STM32_NO_INIT
  /* Trying to allocate SYSCFG.*/
  rccEnableAPB3(RCC_MC_APB3ENSETR_SYSCFGEN, false);

  /* Static PWR configurations.*/
  hal_lld_set_static_pwr();

  /* Clocks setup, this is only necessary when no trusted environment
     is started on the MPU side (device in engineering mode).*/
//  lse_init();
#if !STM32_TZEN_ENABLED
  osalDbgAssert((RCC->TZCR & RCC_TZCR_TZEN) == 0U, "TZEN enabled");
  lsi_init();
#endif
#if !STM32_TZEN_ENABLED || !STM32_MCKPROT_ENABLED
  osalDbgAssert(((RCC->TZCR & RCC_TZCR_TZEN) == 0U) ||
                ((RCC->TZCR & RCC_TZCR_MCKPROT) == 0U), "MCKPROT enabled");
  hsi_init();
  csi_init();
  hse_init();
#endif

  /* PLLs activation, if required.*/
#if !STM32_TZEN_ENABLED || !STM32_MCKPROT_ENABLED
  pll3_init();
#endif

#endif /* STM32_NO_INIT */
}
#endif /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Switches to a different clock configuration
 *
 * @param[in] ccp       pointer to clock a @p halclkcfg_t structure
 * @return              The clock switch result.
 * @retval false        if the clock switch succeeded
 * @retval true         if the clock switch failed
 *
 * @notapi
 */
bool hal_lld_clock_switch_mode(const halclkcfg_t *ccp) {

  (void)cpp;

  return false;
}

/**
 * @brief   Returns the frequency of a clock point in Hz.
 *
 * @param[in] clkpt     clock point to be returned
 * @return              The clock point frequency in Hz or zero if the
 *                      frequency is unknown.
 *
 * @notapi
 */
halfreq_t hal_lld_get_clock_point(halclkpt_t clkpt) {

  osalDbgAssert(clkpt < CLK_ARRAY_SIZE, "invalid clock point");

  return clock_points[clkpt];
}
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/** @} */
