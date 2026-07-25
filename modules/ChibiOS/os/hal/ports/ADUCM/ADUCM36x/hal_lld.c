/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/hal_lld.c
 * @brief   ADUCM36x HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Disable watchdog at startup.
 */
#define HAL_CFG_DISABLE_WDG                 TRUE

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_ADuCM36x.h.
 */
uint32_t SystemCoreClock = ADUCM_HCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/** @todo fix this. */
#if defined(ADUCM_DMA_REQUIRED) || defined(__DOXYGEN__)
#if defined(ADUCM_DMA1_CH23_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 streams 2 and 3 shared ISR.
 * @note    It is declared here because this device has a non-standard
 *          DMA shared IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ADUCM_DMA1_CH23_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 2.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM2);

  /* Check on channel 3.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(ADUCM_DMA1_CH23_HANDLER) */

#if defined(ADUCM_DMA1_CH4567_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 streams 4, 5, 6 and 7 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ADUCM_DMA1_CH4567_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 4.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM4);

  /* Check on channel 5.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM5);

#if ADUCM_DMA1_NUM_CHANNELS > 5
  /* Check on channel 6.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM6);
#endif

#if ADUCM_DMA1_NUM_CHANNELS > 6
  /* Check on channel 7.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM7);
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(ADUCM_DMA1_CH4567_HANDLER) */

#if defined(ADUCM_DMA12_CH23_CH12_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 streams 2 and 3, DMA2 streams 1 and 1 shared ISR.
 * @note    It is declared here because this device has a non-standard
 *          DMA shared IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ADUCM_DMA12_CH23_CH12_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 2 of DMA1.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM2);

  /* Check on channel 3 of DMA1.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM3);

  /* Check on channel 1 of DMA2.*/
  dmaServeInterrupt(ADUCM_DMA2_STREAM1);

  /* Check on channel 2 of DMA2.*/
  dmaServeInterrupt(ADUCM_DMA2_STREAM2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(ADUCM_DMA12_CH23_CH12_HANDLER) */

#if defined(ADUCM_DMA12_CH4567_CH345_HANDLER) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 streams 4, 5, 6 and 7, DMA2 streams 3, 4 and 5 shared ISR.
 * @note    It is declared here because this device has a non-standard
 *          DMA shared IRQ handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ADUCM_DMA12_CH4567_CH345_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 4 of DMA1.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM4);

  /* Check on channel 5 of DMA1.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM5);

  /* Check on channel 6 of DMA1.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM6);

  /* Check on channel 7 of DMA1.*/
  dmaServeInterrupt(ADUCM_DMA1_STREAM7);

  /* Check on channel 3 of DMA2.*/
  dmaServeInterrupt(ADUCM_DMA2_STREAM3);

  /* Check on channel 4 of DMA2.*/
  dmaServeInterrupt(ADUCM_DMA2_STREAM4);

  /* Check on channel 5 of DMA2.*/
  dmaServeInterrupt(ADUCM_DMA2_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif /* defined(ADUCM_DMA12_CH4567_CH345_HANDLER) */
#endif /* defined(ADUCM_DMA_REQUIRED) */

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
#if defined(ADUCM_DMA_REQUIRED)
  dmaInit();
#endif

  /* IRQ subsystem initialization.*/
  irqInit();

  /* Disabling Watchdog timer which is enabled by default. */
#if HAL_CFG_DISABLE_WDG
  pADI_WDT->T3CON = 0;
#endif
}

/**
 * @brief   ADUCM clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void aducm_clock_init(void) {

#if !ADUCM_NO_INIT

#if ADUCM_XOSC_REQUIRED
  /* Configuring the external oscillator. */
  pADI_CLKCTL->XOSCCON = ADUCM_XOSC_ENABLE | ADUCM_XOSC_PREDIV;
#else
  /* Enforcing the external oscillator off condition. */
  pADI_CLKCTL->XOSCCON = ADUCM_XOSC_DISABLE;
#endif

#if ADUCM_CLKMUX == ADUCM_CLKMUX_HFOSC
  /* Changing HFOSC clock divider. */
  pADI_CLKCTL->CLKSYSDIV &= ~CLKSYSDIV_DIV2EN_MSK;
  pADI_CLKCTL->CLKSYSDIV = ADUCM_HFOSC_PREDIV;
#endif

  /* Configuring clock source and divider. */
  pADI_CLKCTL->CLKCON0 = ADUCM_CD_DIV | ADUCM_CLKMUX;

  /* Managing peripheral clock dividers. */
  pADI_CLKCTL->CLKCON1 = ADUCM_SPI0CD_DIV | ADUCM_SPI1CD_DIV |
                         ADUCM_I2CCD_DIV | ADUCM_UARTCD_DIV |
                         ADUCM_PWMCD_DIV;
#endif /* !ADUCM_NO_INIT */
}
/** @} */
