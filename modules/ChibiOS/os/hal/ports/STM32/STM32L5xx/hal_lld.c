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
 * @file    STM32L5xx/hal_lld.c
 * @brief   STM32L5xx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if STM32_SECURE_MODE
#define IS_PERMITTED(r, m) (((r) & (m)) != 0U)
#else
#define IS_PERMITTED(r, m) (((r) & (m)) == 0U)
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_stm32l5xx.h.
 */
uint32_t SystemCoreClock = STM32_HCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#include "stm32_bd.inc"

__STATIC_INLINE void flash_ws_init(uint32_t bits) {

  FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | bits;
  while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != (bits & FLASH_ACR_LATENCY_Msk)) {
  }
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

  /* Security initialization.*/
  secure_init();
}

/**
 * @brief   STM32L4xx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {

#if !STM32_NO_INIT
  uint32_t secflags;

  /* Reset of all peripherals.
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB1(~0);
  rccResetAHB2(~STM32_GPIO_EN_MASK);
  rccResetAHB3(~0);
  rccResetAPB1R1(~RCC_APB1RSTR1_PWRRST);
  rccResetAPB1R2(~0);
  rccResetAPB2(~0);

  /* RCC-related security settings.*/
#if STM32_SECURE_MODE
  RCC->SECCFGR = STM32_RCC_SECCFGR;
  secflags = STM32_RCC_SECCFGR;
#else
  secflags = RCC->SECSR;
#endif

  /* SYSCFG clock enabled here because it is a multi-functional unit shared
     among multiple drivers.*/
  rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, false);

  /* PWR clock enable.*/
#if defined(HAL_USE_RTC) && defined(RCC_APB1ENR1_RTCAPBEN)
  RCC->APB1ENR1 = RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
#else
  RCC->APB1ENR1 = RCC_APB1ENR1_PWREN;
#endif

  /* Core voltage setup, backup domain made accessible.*/
  PWR->CR1 = STM32_VOS | PWR_CR1_DBP;
  while ((PWR->SR2 & PWR_SR2_VOSF) != 0) {  /* Wait until voltage is stable.*/
  }

  /* Additional PWR configurations.*/
  PWR->CR2 = STM32_PWR_CR2;
  PWR->CR3 = STM32_PWR_CR3;
  PWR->CR4 = STM32_PWR_CR4;

  /* Backup domain reset.*/
  bd_reset();

  /* Setting the wait states required by MSI clock.*/
  flash_ws_init(STM32_MSI_FLASHBITS);

  /* Clocks setup, if permitted in current context.*/
  if (IS_PERMITTED(secflags, RCC_SECSR_LSESECF)) {
    lse_init();                             /* LSE first because MSIPLL.    */
  }
  if (IS_PERMITTED(secflags, RCC_SECSR_MSISECF)) {
    msi_init();
  }
  if (IS_PERMITTED(secflags, RCC_SECSR_LSISECF)) {
    lsi_init();
  }
  if (IS_PERMITTED(secflags, RCC_SECSR_HSISECF)) {
    hsi16_init();
  }
  if (IS_PERMITTED(secflags, RCC_SECSR_HSI48SECF)) {
    hsi48_init();
  }
  if (IS_PERMITTED(secflags, RCC_SECSR_HSESECF)) {
    hse_init();
  }

  /* Backup domain initializations.*/
  bd_init();

  /* PLLs activation, if permitted in current context.*/
  if (IS_PERMITTED(secflags, RCC_SECSR_PLLSECF)) {
    pll_init();
  }
  if (IS_PERMITTED(secflags, RCC_SECSR_PLLSAI1SECF)) {
    pllsai1_init();
  }
  if (IS_PERMITTED(secflags, RCC_SECSR_PLLSAI2SECF)) {
    pllsai2_init();
  }

  /* Other clock-related settings (dividers, MCO etc). No security check
     because some fields could be permitted, others not.*/
  RCC->CFGR = STM32_MCOPRE | STM32_MCOSEL | STM32_STOPWUCK |
              STM32_PPRE2  | STM32_PPRE1  | STM32_HPRE;

  /* CCIPR register initialization.*/
  {
    uint32_t ccipr1 = STM32_ADCSEL     | STM32_CLK48SEL  | STM32_FDCANSEL   |
                      STM32_LPTIM3SEL  | STM32_LPTIM2SEL | STM32_LPTIM1SEL  |
                      STM32_I2C3SEL    | STM32_I2C2SEL   | STM32_I2C1SEL    |
                      STM32_LPUART1SEL | STM32_UART5SEL  | STM32_UART4SEL   |
                      STM32_USART3SEL  | STM32_USART2SEL | STM32_USART1SEL;
    RCC->CCIPR1 = ccipr1;
  }

  /* CCIPR2 register initialization, note, must take care of the _OFF
     pseudo settings.*/
  {
    uint32_t ccipr2 = STM32_OSPISEL    | STM32_SDMMCSEL  | STM32_ADFSDMSEL  |
                      STM32_DFSDMSEL   | STM32_I2C4SEL;
#if STM32_SAI2SEL != STM32_SAI2SEL_OFF
    ccipr |= STM32_SAI2SEL;
#endif
#if STM32_SAI1SEL != STM32_SAI1SEL_OFF
    ccipr |= STM32_SAI1SEL;
#endif
    RCC->CCIPR2 = ccipr2;
  }

  /* Clock switching, if permitted in current context.*/
  if (IS_PERMITTED(secflags, RCC_SECSR_SYSCLKSECF)) {
    /* Wait states if SYSCLK requires more wait states than MSICLK.*/
    if (STM32_FLASHBITS > STM32_MSI_FLASHBITS) {
      flash_ws_init(STM32_FLASHBITS);
    }

    /* Switching to the configured SYSCLK source if it is different from MSI.*/
#if STM32_SW != STM32_SW_MSI
    RCC->CFGR |= STM32_SW;        /* Switches on the selected clock source.   */
    /* Wait until SYSCLK is stable.*/
    while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
      ;
#endif

    /* Wait states if SYSCLK requires less wait states than MSICLK.*/
    if (STM32_FLASHBITS < STM32_MSI_FLASHBITS) {
      flash_ws_init(STM32_FLASHBITS);
    }
  }

  /* Cache enable.*/
  icache_init();

#endif /* STM32_NO_INIT */
}

/** @} */
