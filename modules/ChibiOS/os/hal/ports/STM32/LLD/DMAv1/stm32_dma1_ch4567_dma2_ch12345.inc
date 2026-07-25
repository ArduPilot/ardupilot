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
 * @file    DMAv1/stm32_dma1_ch4567_dma2_ch12345.inc
 * @brief   Shared DMA1 Channels 4, 5, 6, 7 and DMA2 Channels 1, 2,
 *          3, 4, 5 handler.
 *
 * @addtogroup STM32_DMA1_CH4567_DMA2_CH12345_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Other checks.*/
#if !defined(STM32_DMA1_CH4567_DMA2_CH12345_HANDLER)
#error "STM32_DMA1_CH4567_DMA2_CH12345_HANDLER not defined in stm32_isr.h"
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if defined(STM32_DMA_REQUIRED) || defined(__DOXYGEN__)
/**
 * @brief   DMA1 (4567) and DMA2 (12345)shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH4567_DMA2_CH12345_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 4.*/
  dmaServeInterrupt(STM32_DMA1_STREAM4);

#if STM32_DMA1_NUM_CHANNELS >= 5
  /* Check on channel 5.*/
  dmaServeInterrupt(STM32_DMA1_STREAM5);
#endif

#if STM32_DMA1_NUM_CHANNELS >= 6
  /* Check on channel 6.*/
  dmaServeInterrupt(STM32_DMA1_STREAM6);
#endif

#if STM32_DMA1_NUM_CHANNELS >= 7
  /* Check on channel 7.*/
  dmaServeInterrupt(STM32_DMA1_STREAM7);
#endif

  /* Check on channel 1.*/
  dmaServeInterrupt(STM32_DMA2_STREAM1);

  /* Check on channel 2.*/
  dmaServeInterrupt(STM32_DMA2_STREAM2);

  /* Check on channel 3.*/
  dmaServeInterrupt(STM32_DMA2_STREAM3);

  /* Check on channel 4.*/
  dmaServeInterrupt(STM32_DMA2_STREAM4);

  /* Check on channel 5.*/
  dmaServeInterrupt(STM32_DMA2_STREAM5);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
