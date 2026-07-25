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
 * @file    STM32F7xx/hal_lld.c
 * @brief   STM32F7xx HAL subsystem low level driver source.
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
 * @note    It is declared in system_stm32f7xx.h.
 */
uint32_t SystemCoreClock = STM32_HCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Initializes the backup domain.
 * @note    WARNING! Changing clock source impossible without resetting
 *          of the whole BKP domain.
 */
static void hal_lld_backup_domain_init(void) {

  /* Backup domain access enabled and left open.*/
  PWR->CR1 |= PWR_CR1_DBP;

  /* Reset BKP domain if different clock source selected.*/
  if ((RCC->BDCR & STM32_RTCSEL_MASK) != STM32_RTCSEL) {
    /* Backup domain reset.*/
    RCC->BDCR = RCC_BDCR_BDRST;
    RCC->BDCR = 0;
  }

#if STM32_LSE_ENABLED
#if defined(STM32_LSE_BYPASS)
  /* LSE Bypass.*/
  RCC->BDCR |= STM32_LSEDRV | RCC_BDCR_LSEON | RCC_BDCR_LSEBYP;
#else
  /* No LSE Bypass.*/
  RCC->BDCR |= STM32_LSEDRV | RCC_BDCR_LSEON;
#endif
  while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0)
    ;                                       /* Waits until LSE is stable.   */
#endif

#if HAL_USE_RTC
  /* If the backup domain hasn't been initialized yet then proceed with
     initialization.*/
  if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0) {
    /* Selects clock source.*/
    RCC->BDCR |= STM32_RTCSEL;

    /* RTC clock enabled.*/
    RCC->BDCR |= RCC_BDCR_RTCEN;
  }
#endif /* HAL_USE_RTC */

#if STM32_BKPRAM_ENABLE
  rccEnableBKPSRAM(true);

  PWR->CSR1 |= PWR_CSR1_BRE;
  while ((PWR->CSR1 & PWR_CSR1_BRR) == 0)
    ;                                /* Waits until the regulator is stable */
#else
  PWR->CSR1 &= ~PWR_CSR1_BRE;
#endif /* STM32_BKPRAM_ENABLE */
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

  /* Reset of all peripherals. AHB3 is not reseted because it could have
     been initialized in the board initialization file (board.c).
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB1(~STM32_GPIO_EN_MASK);
  rccResetAHB2(~0);
  rccResetAPB1(~RCC_APB1RSTR_PWRRST);
  rccResetAPB2(~0);

  /* Initializes the backup domain.*/
  hal_lld_backup_domain_init();

  /* DMA subsystems initialization.*/
#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* IRQ subsystem initialization.*/
  irqInit();

  /* MPU initialization if required.*/
#if STM32_NOCACHE_ENABLE == TRUE
  {
    mpuConfigureRegion(STM32_NOCACHE_MPU_REGION,
                       STM32_NOCACHE_RBAR,
                       MPU_RASR_ATTR_AP_RW_RW |
                       MPU_RASR_ATTR_NON_CACHEABLE |
                       MPU_RASR_ATTR_S |
                       STM32_NOCACHE_RASR |
                       MPU_RASR_ENABLE);
    mpuEnable(MPU_CTRL_PRIVDEFENA);

    /* Invalidating data cache to make sure that the MPU settings are taken
       immediately.*/
    SCB_CleanInvalidateDCache();
  }
#endif

  /* Programmable voltage detector enable.*/
#if STM32_PVD_ENABLE
  PWR->CR1 |= PWR_CR1_PVDE | (STM32_PLS & STM32_PLS_MASK);
#endif /* STM32_PVD_ENABLE */
}

/**
 * @brief   STM32F2xx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {

#if !STM32_NO_INIT
  /* PWR clock enabled.*/
#if defined(HAL_USE_RTC) && defined(RCC_APB1ENR_RTCEN)
  RCC->APB1ENR = RCC_APB1ENR_PWREN | RCC_APB1ENR_RTCEN;
#else
  RCC->APB1ENR = RCC_APB1ENR_PWREN;
#endif

  /* PWR initialization.*/
  PWR->CR1 = STM32_VOS;

  /* HSI setup, it enforces the reset situation in order to handle possible
     problems with JTAG probes and re-initializations.*/
  RCC->CR |= RCC_CR_HSION;                  /* Make sure HSI is ON.         */
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;                                       /* Wait until HSI is stable.    */

  /* HSI is selected as new source without touching the other fields in
     CFGR. Clearing the register has to be postponed after HSI is the
     new source.*/
  RCC->CFGR &= ~RCC_CFGR_SW;                /* Reset SW, selecting HSI.     */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    ;                                       /* Wait until HSI is selected.  */

  /* Registers finally cleared to reset values.*/
  RCC->CR &= RCC_CR_HSITRIM | RCC_CR_HSION; /* CR Reset value.              */
  RCC->CFGR = 0;                            /* CFGR reset value.            */

#if STM32_HSE_ENABLED
  /* HSE activation.*/
#if defined(STM32_HSE_BYPASS)
  /* HSE Bypass.*/
  RCC->CR |= RCC_CR_HSEON | RCC_CR_HSEBYP;
#else
  /* No HSE Bypass.*/
  RCC->CR |= RCC_CR_HSEON;
#endif
  while ((RCC->CR & RCC_CR_HSERDY) == 0)
    ;                           /* Waits until HSE is stable.               */
#endif

#if STM32_LSI_ENABLED
  /* LSI activation.*/
  RCC->CSR |= RCC_CSR_LSION;
  while ((RCC->CSR & RCC_CSR_LSIRDY) == 0)
    ;                           /* Waits until LSI is stable.               */
#endif

#if STM32_ACTIVATE_PLL
  /* PLL activation.*/
  RCC->PLLCFGR = STM32_PLLQ | STM32_PLLSRC | STM32_PLLP | STM32_PLLN |
                 STM32_PLLM;
  RCC->CR |= RCC_CR_PLLON;

  /* Synchronization with voltage regulator stabilization.*/
  while ((PWR->CSR1 & PWR_CSR1_VOSRDY) == 0)
    ;                           /* Waits until power regulator is stable.   */

#if STM32_OVERDRIVE_REQUIRED
  /* Overdrive activation performed after activating the PLL in order to save
     time as recommended in RM in "Entering Over-drive mode" paragraph.*/
  PWR->CR1 |= PWR_CR1_ODEN;
  while (!(PWR->CSR1 & PWR_CSR1_ODRDY))
      ;
  PWR->CR1 |= PWR_CR1_ODSWEN;
  while (!(PWR->CSR1 & PWR_CSR1_ODSWRDY))
      ;
#endif /* STM32_OVERDRIVE_REQUIRED */

  /* Waiting for PLL lock.*/
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;
#endif /* STM32_OVERDRIVE_REQUIRED */

#if STM32_ACTIVATE_PLLI2S
  /* PLLI2S activation.*/
  RCC->PLLI2SCFGR = STM32_PLLI2SR | STM32_PLLI2SQ | STM32_PLLI2SP |
                    STM32_PLLI2SN;
  RCC->CR |= RCC_CR_PLLI2SON;

  /* Waiting for PLL lock.*/
  while (!(RCC->CR & RCC_CR_PLLI2SRDY))
    ;
#endif

#if STM32_ACTIVATE_PLLSAI
  /* PLLSAI activation.*/
  RCC->PLLSAICFGR = STM32_PLLSAIR | STM32_PLLSAIQ | STM32_PLLSAIP |
                    STM32_PLLSAIN;
  RCC->CR |= RCC_CR_PLLSAION;

  /* Waiting for PLL lock.*/
  while (!(RCC->CR & RCC_CR_PLLSAIRDY))
    ;
#endif

  /* Other clock-related settings (dividers, MCO etc).*/
  RCC->CFGR = STM32_MCO2SEL | STM32_MCO2PRE | STM32_MCO1PRE | STM32_I2SSRC |
              STM32_MCO1SEL | STM32_RTCPRE  | STM32_PPRE2   | STM32_PPRE1  |
              STM32_HPRE;

  /* DCKCFGR1 register initialization, note, must take care of the _OFF
     pseudo settings.*/
  {
    uint32_t dckcfgr1 = STM32_PLLI2SDIVQ | STM32_PLLSAIDIVQ | STM32_PLLSAIDIVR;
#if STM32_SAI2SEL != STM32_SAI2SEL_OFF
    dckcfgr1 |= STM32_SAI2SEL;
#endif
#if STM32_SAI1SEL != STM32_SAI1SEL_OFF
    dckcfgr1 |= STM32_SAI1SEL;
#endif
#if STM32_TIMPRE_ENABLE == TRUE
    dckcfgr1 |= RCC_DCKCFGR1_TIMPRE;
#endif
    RCC->DCKCFGR1 = dckcfgr1;
  }

  /* Peripheral clock sources.*/
  RCC->DCKCFGR2 = STM32_SDMMC2SEL | STM32_SDMMC1SEL | STM32_CK48MSEL  |
                  STM32_CECSEL    | STM32_LPTIM1SEL | STM32_I2C4SEL   |
                  STM32_I2C3SEL   | STM32_I2C2SEL   | STM32_I2C1SEL   |
                  STM32_UART8SEL  | STM32_UART7SEL  | STM32_USART6SEL |
                  STM32_UART5SEL  | STM32_UART4SEL  | STM32_USART3SEL |
                  STM32_USART2SEL | STM32_USART1SEL;

  /* Flash setup.*/
  FLASH->ACR = FLASH_ACR_ARTEN | FLASH_ACR_PRFTEN | STM32_FLASHBITS;
  while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) !=
         (STM32_FLASHBITS & FLASH_ACR_LATENCY_Msk)) {
  }

  /* Switching to the configured clock source if it is different from HSI.*/
#if (STM32_SW != STM32_SW_HSI)
  RCC->CFGR |= STM32_SW;        /* Switches on the selected clock source.   */
  while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
    ;
#endif
#endif /* STM32_NO_INIT */

  /* SYSCFG clock enabled here because it is a multi-functional unit shared
     among multiple drivers.*/
  rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, true);
}

/** @} */
