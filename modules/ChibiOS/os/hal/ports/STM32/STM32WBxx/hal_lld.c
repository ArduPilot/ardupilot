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
/*
    Concepts and parts of this file have been contributed by Ilya Kharin.
*/

/**
 * @file    STM32WBxx/hal_lld.c
 * @brief   STM32WBxx HAL subsystem low level driver source.
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
 */
uint32_t SystemCoreClock = STM32_HCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Safe setting of flash ACR register.
 *
 * @param[in] acr       value for the ACR register
 */
__STATIC_INLINE void flash_set_acr(uint32_t acr) {

  FLASH->ACR = acr;
  while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != (acr & FLASH_ACR_LATENCY_Msk)) {
    /* Waiting for flash wait states setup.*/
  }
}

/**
 * @brief   Configures the PWR unit.
 * @note    CR1, CR2 and CR5 are not initialized inside this function.
 */
__STATIC_INLINE void hal_lld_set_static_pwr(void) {
  /* Static PWR configurations.*/
  PWR->CR3   = STM32_PWR_CR3;
  PWR->CR4   = STM32_PWR_CR4;
  PWR->PUCRA = STM32_PWR_PUCRA;
  PWR->PDCRA = STM32_PWR_PDCRA;
  PWR->PUCRB = STM32_PWR_PUCRB;
  PWR->PDCRB = STM32_PWR_PDCRB;
  PWR->PUCRC = STM32_PWR_PUCRC;
  PWR->PDCRC = STM32_PWR_PDCRC;
  PWR->PUCRD = STM32_PWR_PUCRD;
  PWR->PDCRD = STM32_PWR_PDCRD;
  PWR->PUCRE = STM32_PWR_PUCRE;
  PWR->PDCRE = STM32_PWR_PDCRE;
  PWR->PUCRH = STM32_PWR_PUCRH;
  PWR->PDCRH = STM32_PWR_PDCRH;
}

/**
 * @brief   Initializes static muxes and dividers.
 */
__STATIC_INLINE void hal_lld_set_static_clocks(void) {
  uint32_t ccipr;

  /* Clock-related settings (dividers, MCO etc).*/
  RCC->CFGR = STM32_MCOPRE | STM32_MCOSEL | STM32_STOPWUCK |
              STM32_PPRE2  | STM32_PPRE1  | STM32_HPRE;

  /* Waiting for PPRE2, PPRE1 and HPRE applied. */
  while ((RCC->CFGR & (RCC_CFGR_PPRE2F_Msk | RCC_CFGR_PPRE1F_Msk |
                       RCC_CFGR_HPREF_Msk)) !=
         (RCC_CFGR_PPRE2F | RCC_CFGR_PPRE1F | RCC_CFGR_HPREF))
    ;

  /* CCIPR2 register initialization, note, must take care of the _OFF
     pseudo settings.*/
  ccipr = STM32_RNGSEL    | STM32_ADCSEL    | STM32_CLK48SEL  |
          STM32_LPTIM2SEL | STM32_LPTIM1SEL | STM32_I2C1SEL   |
          STM32_USART1SEL | STM32_LPUART1SEL;
#if STM32_SAI1SEL != STM32_SAI1SEL_OFF
    ccipr |= STM32_SAI1SEL;
#endif
    RCC->CCIPR = ccipr;
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

  /* Programmable voltage detector enable.*/
#if STM32_PVD_ENABLE
  PWR->CR2 = PWR_CR2_PVDE | (STM32_PLS & STM32_PLS_MASK);
#else
  PWR->CR2 = 0;
#endif /* STM32_PVD_ENABLE */

  /* Enabling independent VDDUSB.*/
#if HAL_USE_USB
  PWR->CR2 |= PWR_CR2_USV;
#endif /* HAL_USE_USB */
}

/**
 * @brief   STM32WBxx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {

#if !STM32_NO_INIT
  /* Reset of all peripherals.
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB1(~0);
  rccResetAHB2(~STM32_GPIO_EN_MASK);
  rccResetAHB3(~0);
  rccResetAPB1R1(~0);
  rccResetAPB1R2(~0);
  rccResetAPB2(~0);

  /* Flash setup for selected MSI speed setting.*/
  flash_set_acr(FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN |
                STM32_MSI_FLASHBITS);

  /* Static PWR configurations.*/
  hal_lld_set_static_pwr();

  /* Core voltage setup, backup domain access enabled and left open.*/
  PWR->CR1 = STM32_VOS | PWR_CR1_DBP;

  /* Additional PWR configurations.*/
  PWR->CR2  = STM32_PWR_CR2;

  /* Wait until regulator is stable. */
  while ((PWR->SR2 & PWR_SR2_VOSF) != 0)
    ;

  /* MSI clock reset.*/
  msi_reset();

  /* Backup domain reset.*/
  bd_reset();

  /* Clocks setup.*/
  lse_init();
  lsi_init();
  msi_init();
  hsi16_init();
  hsi48_init();
  hse32_init();

  /* Backup domain initializations.*/
  bd_init();

  /* Static clocks setup.*/
  hal_lld_set_static_clocks();

  /* PLLs activation, if required.*/
  pll_init();
  pllsai1_init();

  /* Extended clock recovery register (HCLK2, HCLK4, HCLK5). */
  RCC->EXTCFGR = STM32_RFCSSSEL | STM32_C2HPRE | STM32_SHDHPRE;

  /* Waiting for C2HPRE and SHDHPRE. */
  while ((RCC->EXTCFGR & (RCC_EXTCFGR_C2HPREF_Msk |
                          RCC_EXTCFGR_SHDHPREF_Msk)) !=
         (RCC_EXTCFGR_C2HPREF | RCC_EXTCFGR_SHDHPREF))
    ;

  /* Set flash WS's for SYSCLK source */
  if (STM32_FLASHBITS > STM32_MSI_FLASHBITS) {
    flash_set_acr((FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | STM32_FLASHBITS);
  }

  /* Switching to the configured SYSCLK source if it is different from MSI.*/
#if (STM32_SW != STM32_SW_MSI)
  RCC->CFGR |= STM32_SW;        /* Switches on the selected clock source.   */
  /* Wait until SYSCLK is stable.*/
  while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
    ;
#endif

  /* Reduce the flash WS's for SYSCLK source if they are less than MSI WSs */
  if (STM32_FLASHBITS < STM32_MSI_FLASHBITS) {
    flash_set_acr((FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | STM32_FLASHBITS);
  }

#endif /* STM32_NO_INIT */
}

/** @} */
