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
 * @file    QUADSPIv1//hal_wspi_lld.c
 * @brief   STM32 WSPI subsystem low level driver source.
 *
 * @addtogroup WSPI
 * @{
 */

#include "hal.h"

#if (HAL_USE_WSPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define QUADSPI1_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_WSPI_QUADSPI1_DMA_STREAM,                      \
                       STM32_QUADSPI1_DMA_CHN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief QUADSPI1 driver identifier.*/
#if STM32_WSPI_USE_QUADSPI1 || defined(__DOXYGEN__)
WSPIDriver WSPID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Waits for completion of previous operation.
 */
static inline void wspi_lld_sync(WSPIDriver *wspip) {

  while ((wspip->qspi->SR & QUADSPI_SR_BUSY) != 0U) {
  }
}

/**
 * @brief   Shared service routine.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void wspi_lld_serve_dma_interrupt(WSPIDriver *wspip, uint32_t flags) {

  (void)wspip;
  (void)flags;

  /* DMA errors handling.*/
#if defined(STM32_WSPI_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_WSPI_DMA_ERROR_HOOK(wspip);
  }
#endif
}

/**
 * @brief   Shared service routine.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 */
static void wspi_lld_serve_interrupt(WSPIDriver *wspip) {

  /* Portable WSPI ISR code defined in the high level driver, note, it is
     a macro.*/
  _wspi_isr_code(wspip);

  /* Stop everything, we need to give DMA enough time to complete the ongoing
     operation. Race condition hidden here.*/
  while (dmaStreamGetTransactionSize(wspip->dma) > 0U)
    ;

  /* Clearing DMA interrupts here because the DMA ISR is not called on
     transfer complete.*/
  dmaStreamClearInterrupt(wspip->dma);
  dmaStreamDisable(wspip->dma);

#if defined(STM32L471xx) || defined(STM32L475xx) ||                         \
    defined(STM32L476xx) || defined(STM32L486xx)
  /* Handling of errata: Extra data written in the FIFO at the end of a
     read transfer.*/
  if (wspip->state == WSPI_RECEIVE) {
    while ((wspip->qspi->SR & QUADSPI_SR_BUSY) != 0U) {
      (void) wspip->qspi->DR;
    }
  }
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_WSPI_USE_QUADSPI1 || defined(__DOXYGEN__)
#if !defined(STM32_QUADSPI1_SUPPRESS_ISR)
#if !defined(STM32_QUADSPI1_HANDLER)
#error "STM32_QUADSPI1_HANDLER not defined"
#endif
/**
 * @brief   STM32_QUADSPI1_HANDLER interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_QUADSPI1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  QUADSPI->FCR = QUADSPI_FCR_CTEF | QUADSPI_FCR_CTCF |
                 QUADSPI_FCR_CSMF | QUADSPI_FCR_CTOF;

  wspi_lld_serve_interrupt(&WSPID1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_QUADSPI1_SUPPRESS_ISR) */
#endif /* STM32_WSPI_USE_QUADSPI1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level WSPI driver initialization.
 *
 * @notapi
 */
void wspi_lld_init(void) {

#if STM32_WSPI_USE_QUADSPI1
  wspiObjectInit(&WSPID1);
  WSPID1.qspi       = QUADSPI;
  WSPID1.dma        = NULL;
  WSPID1.dmamode    = STM32_DMA_CR_CHSEL(QUADSPI1_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_WSPI_QUADSPI1_DMA_PRIORITY) |
                      STM32_DMA_CR_PSIZE_BYTE |
                      STM32_DMA_CR_MSIZE_BYTE |
                      STM32_DMA_CR_MINC |
                      STM32_DMA_CR_DMEIE |
                      STM32_DMA_CR_TEIE;
  nvicEnableVector(STM32_QUADSPI1_NUMBER, STM32_WSPI_QUADSPI1_IRQ_PRIORITY);
#endif
}

/**
 * @brief   Configures and activates the WSPI peripheral.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
void wspi_lld_start(WSPIDriver *wspip) {

  /* If in stopped state then full initialization.*/
  if (wspip->state == WSPI_STOP) {
#if STM32_WSPI_USE_QUADSPI1
    if (&WSPID1 == wspip) {
      wspip->dma = dmaStreamAllocI(STM32_WSPI_QUADSPI1_DMA_STREAM,
                                   STM32_WSPI_QUADSPI1_DMA_IRQ_PRIORITY,
                                   (stm32_dmaisr_t)wspi_lld_serve_dma_interrupt,
                                   (void *)wspip);
      osalDbgAssert(wspip->dma != NULL, "unable to allocate stream");
      rccEnableQUADSPI1(true);
#if STM32_DMA_SUPPORTS_DMAMUX
      dmaSetRequestSource(wspip->dma, STM32_DMAMUX1_QUADSPI);
#endif
    }
#endif

    /* Common initializations.*/
    dmaStreamSetPeripheral(wspip->dma, &wspip->qspi->DR);
  }

  /* WSPI setup and enable.*/
  wspip->qspi->DCR = wspip->config->dcr;
  wspip->qspi->CR  = ((STM32_WSPI_QUADSPI1_PRESCALER_VALUE - 1U) << 24U) |
                      QUADSPI_CR_TCIE | QUADSPI_CR_DMAEN | QUADSPI_CR_EN;
  wspip->qspi->FCR = QUADSPI_FCR_CTEF | QUADSPI_FCR_CTCF |
                     QUADSPI_FCR_CSMF | QUADSPI_FCR_CTOF;
}

/**
 * @brief   Deactivates the WSPI peripheral.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
void wspi_lld_stop(WSPIDriver *wspip) {

  /* If in ready state then disables the QUADSPI clock.*/
  if (wspip->state == WSPI_READY) {

    /* WSPI disable.*/
    wspip->qspi->CR = 0U;

    /* Releasing the DMA.*/
    dmaStreamFreeI(wspip->dma);
    wspip->dma = NULL;

    /* Stopping involved clocks.*/
#if STM32_WSPI_USE_QUADSPI1
    if (&WSPID1 == wspip) {
      rccDisableQUADSPI1();
    }
#endif
  }
}

/**
 * @brief   Sends a command without data phase.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 *
 * @notapi
 */
void wspi_lld_command(WSPIDriver *wspip, const wspi_command_t *cmdp) {

#if STM32_USE_STM32_D1_WORKAROUND == TRUE
  /* If it is a command without address and alternate phases then the command
     is sent as an alternate byte, the command phase is suppressed.*/
  if ((cmdp->cfg & (WSPI_CFG_ADDR_MODE_MASK | WSPI_CFG_ALT_MODE_MASK)) == 0U) {
    /* The command mode field is copied in the alternate mode field. All
       other fields are not used in this scenario.*/
    wspip->qspi->DLR = 0U;
    wspip->qspi->ABR = cmdp->cmd;
    wspip->qspi->CCR = (cmdp->cfg  & WSPI_CFG_CMD_MODE_MASK) << 6U;
    return;
  }
#endif
  wspip->qspi->DLR = 0U;
  wspip->qspi->ABR = cmdp->alt;
  wspip->qspi->CCR = cmdp->cmd | cmdp->cfg;
  if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE) {
    wspip->qspi->AR  = cmdp->addr;
  }
}

/**
 * @brief   Sends a command with data over the WSPI bus.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[in] n         number of bytes to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void wspi_lld_send(WSPIDriver *wspip, const wspi_command_t *cmdp,
                   size_t n, const uint8_t *txbuf) {

  dmaStreamSetMemory0(wspip->dma, txbuf);
  dmaStreamSetTransactionSize(wspip->dma, n);
  dmaStreamSetMode(wspip->dma, wspip->dmamode | STM32_DMA_CR_DIR_M2P);

  wspip->qspi->DLR = n - 1;
  wspip->qspi->ABR = cmdp->alt;
  wspip->qspi->CCR = cmdp->cmd | cmdp->cfg;
  if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE) {
    wspip->qspi->AR  = cmdp->addr;
  }

  dmaStreamEnable(wspip->dma);
}

/**
 * @brief   Sends a command then receives data over the WSPI bus.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[in] n         number of bytes to send
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void wspi_lld_receive(WSPIDriver *wspip, const wspi_command_t *cmdp,
                      size_t n, uint8_t *rxbuf) {

  dmaStreamSetMemory0(wspip->dma, rxbuf);
  dmaStreamSetTransactionSize(wspip->dma, n);
  dmaStreamSetMode(wspip->dma, wspip->dmamode | STM32_DMA_CR_DIR_P2M);

  wspip->qspi->DLR = n - 1;
  wspip->qspi->ABR = cmdp->alt;
  wspip->qspi->CCR = cmdp->cmd | cmdp->cfg |
                     QUADSPI_CCR_DUMMY_CYCLES(cmdp->dummy) |
                     QUADSPI_CCR_FMODE_0;
  if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE) {
    wspip->qspi->AR  = cmdp->addr;
  }

  dmaStreamEnable(wspip->dma);
}

#if (WSPI_SUPPORTS_MEMMAP == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Maps in memory space a WSPI flash device.
 * @pre     The memory flash device must be initialized appropriately
 *          before mapping it in memory space.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 * @param[in] cmdp      pointer to the command descriptor
 * @param[out] addrp    pointer to the memory start address of the mapped
 *                      flash or @p NULL
 *
 * @notapi
 */
void wspi_lld_map_flash(WSPIDriver *wspip,
                        const wspi_command_t *cmdp,
                        uint8_t **addrp) {

  /* Disabling the DMA request while in memory mapped mode.*/
  wspip->qspi->CR &= ~QUADSPI_CR_DMAEN;

  /* Starting memory mapped mode using the passed parameters.*/
  wspip->qspi->DLR = 0;
  wspip->qspi->ABR = 0;
  wspip->qspi->CCR = cmdp->cmd | cmdp->cfg |
                     QUADSPI_CCR_DUMMY_CYCLES(cmdp->dummy) |
                     QUADSPI_CCR_FMODE_1 | QUADSPI_CCR_FMODE_0;

  /* Mapped flash absolute base address.*/
  if (addrp != NULL) {
    *addrp = (uint8_t *)0x90000000;
  }
}

/**
 * @brief   Unmaps from memory space a WSPI flash device.
 * @post    The memory flash device must be re-initialized for normal
 *          commands exchange.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
void wspi_lld_unmap_flash(WSPIDriver *wspip) {

  /* Aborting memory mapped mode.*/
  wspip->qspi->CR |= QUADSPI_CR_ABORT;
  while ((wspip->qspi->CR & QUADSPI_CR_ABORT) != 0U) {
  }

  /* Re-enabling DMA request, we are going back to indirect mode.*/
  wspip->qspi->CR |= QUADSPI_CR_DMAEN;
}
#endif /* WSPI_SUPPORTS_MEMMAP == TRUE */

#endif /* HAL_USE_WSPI */

/** @} */
