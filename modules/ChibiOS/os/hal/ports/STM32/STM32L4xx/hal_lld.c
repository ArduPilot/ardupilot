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
 * @file    STM32L4xx/hal_lld.c
 * @brief   STM32L4xx HAL subsystem low level driver source.
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
 * @note    It is declared in system_stm32l4xx.h.
 */
uint32_t SystemCoreClock = STM32_HCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#include "stm32_bd.inc"

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

/**
 * @brief   STM32L4xx clocks and PLL initialization.
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
  FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN |
               STM32_MSI_FLASHBITS;

  /* SYSCFG clock enabled here because it is a multi-functional unit shared
     among multiple drivers.*/
  rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, false);

  /* PWR clock enabled.*/
  rccEnablePWRInterface(false);

  /* RTC APB clock enable.*/
#if (HAL_USE_RTC == TRUE) && defined(RCC_APB1ENR1_RTCAPBEN)
  rccEnableAPB1R1(RCC_APB1ENR1_RTCAPBEN, true);
#endif

  /* Core voltage setup, backup domain access enabled and left open.*/
  PWR->CR1 = STM32_VOS | PWR_CR1_DBP;
  while ((PWR->SR2 & PWR_SR2_VOSF) != 0)    /* Wait until regulator is      */
    ;                                       /* stable.                      */

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

  /* Enabling independent VDDIO2 required by GPIOG.*/
#if STM32_HAS_GPIOG
  PWR->CR2 |= PWR_CR2_IOSV;
#endif /* STM32_HAS_GPIOG */

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
  hse_init();

  /* Backup domain initializations.*/
  bd_init();

  /* PLLs activation, if required.*/
  pll_init();
  pllsai1_init();
  pllsai2_init();

  /* Other clock-related settings (dividers, MCO etc).*/
  RCC->CFGR = STM32_MCOPRE | STM32_MCOSEL | STM32_STOPWUCK |
              STM32_PPRE2  | STM32_PPRE1  | STM32_HPRE;

  /* CCIPR register initialization, note, must take care of the _OFF
     pseudo settings.*/
  {
    uint32_t ccipr = STM32_DFSDMSEL  | STM32_SWPMI1SEL | STM32_ADCSEL    |
                     STM32_CLK48SEL  | STM32_LPTIM2SEL | STM32_LPTIM1SEL |
                     STM32_I2C3SEL   | STM32_I2C2SEL   | STM32_I2C1SEL   |
                     STM32_UART5SEL  | STM32_UART4SEL  | STM32_USART3SEL |
                     STM32_USART2SEL | STM32_USART1SEL | STM32_LPUART1SEL;
#if STM32_SAI2SEL != STM32_SAI2SEL_OFF
    ccipr |= STM32_SAI2SEL;
#endif
#if STM32_SAI1SEL != STM32_SAI1SEL_OFF
    ccipr |= STM32_SAI1SEL;
#endif
    RCC->CCIPR = ccipr;
  }

#if STM32_HAS_I2C4
  /* CCIPR2 register initialization.*/
  {
    uint32_t ccipr2 = STM32_I2C4SEL;
    RCC->CCIPR2 = ccipr2;
  }
#endif

  /* Set flash WS's for SYSCLK source */
  if (STM32_FLASHBITS > STM32_MSI_FLASHBITS) {
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | STM32_FLASHBITS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) !=
           (STM32_FLASHBITS & FLASH_ACR_LATENCY_Msk)) {
    }
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
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | STM32_FLASHBITS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) !=
           (STM32_FLASHBITS & FLASH_ACR_LATENCY_Msk)) {
    }
  }
#endif /* STM32_NO_INIT */
}

/** @} */
