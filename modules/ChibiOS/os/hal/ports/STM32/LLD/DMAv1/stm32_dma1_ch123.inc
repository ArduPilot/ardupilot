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
 * @file    DMAv1/stm32_dma1_ch123.inc
 * @brief   Shared DMA1 Channels 1, 2 and 3 handler.
 *
 * @addtogroup STM32_DMA1_CH123_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Other checks.*/
#if !defined(STM32_DMA1_CH123_HANDLER)
#error "STM32_DMA1_CH123_HANDLER not defined in stm32_isr.h"
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
 * @brief   DMA1 streams 1, 2 and 3 shared ISR.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_DMA1_CH123_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  /* Check on channel 1.*/
  dmaServeInterrupt(STM32_DMA1_STREAM1);

  /* Check on channel 2.*/
  dmaServeInterrupt(STM32_DMA1_STREAM2);

  /* Check on channel 3.*/
  dmaServeInterrupt(STM32_DMA1_STREAM3);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
