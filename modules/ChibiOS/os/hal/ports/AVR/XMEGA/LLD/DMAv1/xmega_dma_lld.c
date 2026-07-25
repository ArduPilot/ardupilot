/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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
 * @file    DMAv1/xmega_dma_lld.c
 * @brief   AVR XMEGA DMA low level driver source file.
 *
 * @addtogroup DMA
 * @{
 */

#include "hal.h"

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

/**
 * @brief   ISR for DMA channel 0.
 *
 * @param[in] DMA_CH0_vect  DMA controller channel0 interrupt vector.
 */
OSAL_IRQ_HANDLER(DMA_CH0_vect) {

  OSAL_IRQ_PROLOGUE();
  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   ISR for DMA channel 1.
 *
 * @param[in] DMA_CH1_vect  DMA controller channel1 interrupt vector.
 */
OSAL_IRQ_HANDLER(DMA_CH1_vect) {

  OSAL_IRQ_PROLOGUE();
  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   ISR for DMA channel 2.
 *
 * @param[in] DMA_CH2_vect  DMA controller channel2 interrupt vector.
 */
OSAL_IRQ_HANDLER(DMA_CH2_vect) {

  OSAL_IRQ_PROLOGUE();
  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   ISR for DMA channel 3.
 *
 * @param[in] DMA_CH3_vect  DMA controller channel3 interrupt vector.
 */
OSAL_IRQ_HANDLER(DMA_CH3_vect) {

  OSAL_IRQ_PROLOGUE();
  OSAL_IRQ_EPILOGUE();
}

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Enable DMA controller.
 *
 * @param[in] dmacp   pointer to the dma controller
 */
void dmaControllerEnable(DMA_t *dmacp) {

  dmacp->CTRL |= DMA_ENABLE_bm;
}

/**
 * @brief   Disable DMA controller.
 * @note    On going transfers will be aborted.
 *
 * @param[in] dmacp   pointer to the dma controller
 */
void dmaControllerDisable(DMA_t *dmacp) {

  dmacp->CTRL &= ~(DMA_ENABLE_bm);
}

/**
 * @brief   Reset DMA controller.
 * @pre     The DMA controller must be disabled before to process to a reset.
 *
 * @param[in] dmacp   pointer to the dma controller
 */
void damReset(void) {

  DMA.CTRL &= ~DMA_ENABLE_bm;     /* Disable the DMA before a reset.      */
  DMA.CTRL |= DMA_RESET_bm;       /* Perform the reset of the DMA module. */
  while(DMA.CTRL & DMA_RESET_bm); /* Wait until reset is complated.       */
}

/**
 * @brief   Enable a DMA channel.
 *
 * @param[in] dmacp   pointer to the dma channel
 */
void dmaChannelEnable(DMA_CH_t *dmacp) {

  dmacp->CTRLA |= DMA_CH_ENABLE_bm;
}

/**
 * @brief   Disable a DMA channel.
 *
 * @param[in] dmacp   pointer to the dma channel
 */
void dmaChannelDisable(DMA_CH_t *dmacp) {

  dmacp->CTRLA &= ~DMA_CH_ENABLE_bm;
}

/**
 * @brief   Disable a DMA channel.
 * @note    This can only be done if the DMA channel is disable.
 *
 * @param[in] dmacp   pointer to the dma channel
 */
void dmaChannelReset(DMA_CH_t *dmacp) {

}

void dmaEnableSingleShot(DMA_CH_t * dmacp) {

  dmacp->CTRLA |= DMA_CH_SINGLE_bm;
}

void dmaDisableSingleShot(DMA_CH_t * dmacp) {

  dmacp->CTRLA &= ~DMA_CH_SINGLE_bm;
}

void dmaSetTriggerSource(DMA_CH_t * dmacp, uint8_t trigger) {

  dmacp->TRIGSRC = trigger;
}

void dmaStartTransfer(DMA_CH_t * dmacp) {

  dmacp->CTRLA |= DMA_CH_TRFREQ_bm;
}

/** @} */

