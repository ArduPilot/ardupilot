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
 * @file    STM32F0xx/hal_lld.c
 * @brief   STM32F0xx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define STM32_PLLXTPRE_OFFSET   17          /**< PLLXTPRE offset             */
#define STM32_PLLXTPRE_MASK     0x01        /**< PLLXTPRE mask               */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_stm32f0xx.h.
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
  PWR->CR |= PWR_CR_DBP;

  /* Reset BKP domain if different clock source selected.*/
  if ((RCC->BDCR & STM32_RTCSEL_MASK) != STM32_RTCSEL) {
    /* Backup domain reset.*/
    RCC->BDCR = RCC_BDCR_BDRST;
    RCC->BDCR = 0;
  }

  /* If enabled then the LSE is started.*/
#if STM32_LSE_ENABLED
#if defined(STM32_LSE_BYPASS)
  /* LSE Bypass.*/
  RCC->BDCR |= STM32_LSEDRV | RCC_BDCR_LSEON | RCC_BDCR_LSEBYP;
#else
  /* No LSE Bypass.*/
  RCC->BDCR |= STM32_LSEDRV | RCC_BDCR_LSEON;
#endif
  while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0)
    ;                                     /* Waits until LSE is stable.   */
#endif

#if STM32_RTCSEL != STM32_RTCSEL_NOCLOCK
  /* If the backup domain hasn't been initialized yet then proceed with
     initialization.*/
  if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0) {
    /* Selects clock source.*/
    RCC->BDCR |= STM32_RTCSEL;

    /* RTC clock enabled.*/
    RCC->BDCR |= RCC_BDCR_RTCEN;
  }
#endif /* STM32_RTCSEL != STM32_RTCSEL_NOCLOCK */
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_DMA_REQUIRED) || defined(__DOXYGEN__)
#if defined(STM32_DMA1_CH23_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 streams 2 and 3 shared ISR.
 * @note    It is declared here because this device has a non-standard
 *          DMA shared IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH23_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 2.*/
  dmaServeInterrupt(STM32_DMA1_STREAM2);

  /* Check on channel 3.*/
  dmaServeInterrupt(STM32_DMA1_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(STM32_DMA1_CH23_HANDLER) */

#if defined(STM32_DMA1_CH4567_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 streams 4, 5, 6 and 7 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH4567_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 4.*/
  dmaServeInterrupt(STM32_DMA1_STREAM4);

  /* Check on channel 5.*/
  dmaServeInterrupt(STM32_DMA1_STREAM5);

#if STM32_DMA1_NUM_CHANNELS > 5
  /* Check on channel 6.*/
  dmaServeInterrupt(STM32_DMA1_STREAM6);
#endif

#if STM32_DMA1_NUM_CHANNELS > 6
  /* Check on channel 7.*/
  dmaServeInterrupt(STM32_DMA1_STREAM7);
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(STM32_DMA1_CH4567_HANDLER) */

#if defined(STM32_DMA12_CH23_CH12_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 streams 2 and 3, DMA2 streams 1 and 1 shared ISR.
 * @note    It is declared here because this device has a non-standard
 *          DMA shared IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA12_CH23_CH12_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 2 of DMA1.*/
  dmaServeInterrupt(STM32_DMA1_STREAM2);

  /* Check on channel 3 of DMA1.*/
  dmaServeInterrupt(STM32_DMA1_STREAM3);

  /* Check on channel 1 of DMA2.*/
  dmaServeInterrupt(STM32_DMA2_STREAM1);

  /* Check on channel 2 of DMA2.*/
  dmaServeInterrupt(STM32_DMA2_STREAM2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(STM32_DMA12_CH23_CH12_HANDLER) */

#if defined(STM32_DMA12_CH4567_CH345_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 streams 4, 5, 6 and 7, DMA2 streams 3, 4 and 5 shared ISR.
 * @note    It is declared here because this device has a non-standard
 *          DMA shared IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA12_CH4567_CH345_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 4 of DMA1.*/
  dmaServeInterrupt(STM32_DMA1_STREAM4);

  /* Check on channel 5 of DMA1.*/
  dmaServeInterrupt(STM32_DMA1_STREAM5);

  /* Check on channel 6 of DMA1.*/
  dmaServeInterrupt(STM32_DMA1_STREAM6);

  /* Check on channel 7 of DMA1.*/
  dmaServeInterrupt(STM32_DMA1_STREAM7);

  /* Check on channel 3 of DMA2.*/
  dmaServeInterrupt(STM32_DMA2_STREAM3);

  /* Check on channel 4 of DMA2.*/
  dmaServeInterrupt(STM32_DMA2_STREAM4);

  /* Check on channel 5 of DMA2.*/
  dmaServeInterrupt(STM32_DMA2_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(STM32_DMA12_CH4567_CH345_HANDLER) */
#endif /* defined(STM32_DMA_REQUIRED) */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* Reset of all peripherals.
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB(~STM32_GPIO_EN_MASK);
  rccResetAPB1(0xFFFFFFFF);
  rccResetAPB2(~RCC_APB2RSTR_DBGMCURST);

  /* PWR clock enabled.*/
  rccEnablePWRInterface(true);

  /* Initializes the backup domain.*/
  hal_lld_backup_domain_init();

  /* DMA subsystems initialization.*/
#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* IRQ subsystem initialization.*/
  irqInit();

  /* Programmable voltage detector enable.*/
#if STM32_PVD_ENABLE
  PWR->CR |= PWR_CR_PVDE | (STM32_PLS & STM32_PLS_MASK);
#endif /* STM32_PVD_ENABLE */
}

/**
 * @brief   STM32 clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {

#if !STM32_NO_INIT
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
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;                                       /* Waits until HSE is stable.   */
#endif

#if STM32_HSI14_ENABLED
  /* HSI14 activation.*/
  RCC->CR2 |= RCC_CR2_HSI14ON;
  while (!(RCC->CR2 & RCC_CR2_HSI14RDY))
    ;                                       /* Waits until HSI14 is stable. */
#endif

#if STM32_HSI48_ENABLED
  /* HSI48 activation.*/
  RCC->CR2 |= RCC_CR2_HSI48ON;
  while (!(RCC->CR2 & RCC_CR2_HSI48RDY))
    ;                                       /* Waits until HSI48 is stable. */
#endif

#if STM32_LSI_ENABLED
  /* LSI activation.*/
  RCC->CSR |= RCC_CSR_LSION;
  while ((RCC->CSR & RCC_CSR_LSIRDY) == 0)
    ;                                       /* Waits until LSI is stable.   */
#endif

  /* Clock settings.*/
  /* CFGR2 must be configured first since CFGR value could change CFGR2 */
  RCC->CFGR2 = STM32_PREDIV;
  RCC->CFGR  = STM32_PLLNODIV | STM32_MCOPRE | STM32_MCOSEL | STM32_PLLMUL |
               STM32_PLLSRC   | STM32_PPRE   | STM32_HPRE |
               ((STM32_PREDIV & STM32_PLLXTPRE_MASK) << STM32_PLLXTPRE_OFFSET);
#if STM32_CECSW == STM32_CECSW_OFF
  RCC->CFGR3 = STM32_USBSW  | STM32_I2C1SW | STM32_USART1SW;
#else
  RCC->CFGR3 = STM32_USBSW  | STM32_CECSW  | STM32_I2C1SW | STM32_USART1SW;
#endif

#if STM32_ACTIVATE_PLL
  /* PLL activation.*/
  RCC->CR   |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;                                       /* Waits until PLL is stable.   */
#endif

  /* Flash setup and final clock selection.   */
  FLASH->ACR = STM32_FLASHBITS;
  while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) !=
         (STM32_FLASHBITS & FLASH_ACR_LATENCY_Msk)) {
  }

  /* Switching to the configured clock source if it is different from HSI.*/
#if (STM32_SW != STM32_SW_HSI)
  /* Switches clock source.*/
  RCC->CFGR |= STM32_SW;
  while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
    ;                                       /* Waits selection complete.    */
#endif

  /* SYSCFG clock enabled here because it is a multi-functional unit shared
     among multiple drivers.*/
  rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, true);
#endif /* !STM32_NO_INIT */
}

/** @} */
