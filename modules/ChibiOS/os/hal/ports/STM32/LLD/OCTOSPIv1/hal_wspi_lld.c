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
 * @file    OCTOSPIv1/hal_wspi_lld.c
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

/* Workarounds for bugs in ST headers.*/
#if !defined(OCTOSPI_FCR_CTOF) && defined(OCTOSPI_FCR_TOF)
#define OCTOSPI_FCR_CTOF OCTOSPI_FCR_TOF
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief OCTOSPI1 driver identifier.*/
#if STM32_WSPI_USE_OCTOSPI1 || defined(__DOXYGEN__)
WSPIDriver WSPID1;
#endif

/** @brief OCTOSPI2 driver identifier.*/
#if STM32_WSPI_USE_OCTOSPI2 || defined(__DOXYGEN__)
WSPIDriver WSPID2;
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

  while ((wspip->ospi->SR & OCTOSPI_SR_BUSY) != 0U) {
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
  dmaStreamDisable(wspip->dma);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_WSPI_USE_OCTOSPI1 || defined(__DOXYGEN__)
#if !defined(STM32_OCTOSPI1_SUPPRESS_ISR)
#if !defined(STM32_OCTOSPI1_HANDLER)
#error "STM32_OCTOSPI1_HANDLER not defined"
#endif
/**
 * @brief   STM32_OCTOSPI1_HANDLER interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_OCTOSPI1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  OCTOSPI1->FCR = OCTOSPI_FCR_CTEF | OCTOSPI_FCR_CTCF |
                  OCTOSPI_FCR_CSMF | OCTOSPI_FCR_CTOF;

  wspi_lld_serve_interrupt(&WSPID1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_OCTOSPI1_SUPPRESS_ISR) */
#endif /* STM32_WSPI_USE_OCTOSPI1 */

#if STM32_WSPI_USE_OCTOSPI2 || defined(__DOXYGEN__)
#if !defined(STM32_OCTOSPI2_SUPPRESS_ISR)
#if !defined(STM32_OCTOSPI2_HANDLER)
#error "STM32_OCTOSPI2_HANDLER not defined"
#endif
/**
 * @brief   STM32_OCTOSPI2_HANDLER interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_OCTOSPI2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  OCTOSPI2->FCR = OCTOSPI_FCR_CTEF | OCTOSPI_FCR_CTCF |
                  OCTOSPI_FCR_CSMF | OCTOSPI_FCR_CTOF;

  wspi_lld_serve_interrupt(&WSPID2);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(STM32_OCTOSPI2_SUPPRESS_ISR) */
#endif /* STM32_WSPI_USE_OCTOSPI2 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level WSPI driver initialization.
 *
 * @notapi
 */
void wspi_lld_init(void) {

#if STM32_WSPI_USE_OCTOSPI1
  wspiObjectInit(&WSPID1);
  WSPID1.extra_tcr  = 0U
#if STM32_WSPI_OCTOSPI1_SSHIFT
                    | OCTOSPI_TCR_SSHIFT
#endif
#if STM32_WSPI_OCTOSPI1_DHQC
                    | OCTOSPI_TCR_DHQC
#endif
                    ;
  WSPID1.ospi       = OCTOSPI1;
  WSPID1.dma        = NULL;
  WSPID1.dmamode    = STM32_DMA_CR_CHSEL(OCTOSPI1_DMA_STREAM) |
                      STM32_DMA_CR_PL(STM32_WSPI_OCTOSPI1_DMA_PRIORITY) |
                      STM32_DMA_CR_PSIZE_BYTE |
                      STM32_DMA_CR_MSIZE_BYTE |
                      STM32_DMA_CR_MINC |
                      STM32_DMA_CR_DMEIE |
                      STM32_DMA_CR_TEIE;
  nvicEnableVector(STM32_OCTOSPI1_NUMBER, STM32_WSPI_OCTOSPI1_IRQ_PRIORITY);
#endif

#if STM32_WSPI_USE_OCTOSPI2
  wspiObjectInit(&WSPID2);
  WSPID2.extra_tcr  = 0U
#if STM32_WSPI_OCTOSPI2_SSHIFT
                    | OCTOSPI_TCR_SSHIFT
#endif
#if STM32_WSPI_OCTOSPI1_DHQC
                    | OCTOSPI_TCR_DHQC
#endif
                    ;
  WSPID2.ospi       = OCTOSPI2;
  WSPID2.dma        = NULL;
  WSPID2.dmamode    = STM32_DMA_CR_CHSEL(OCTOSPI2_DMA_STREAM) |
                      STM32_DMA_CR_PL(STM32_WSPI_OCTOSPI2_DMA_PRIORITY) |
                      STM32_DMA_CR_PSIZE_BYTE |
                      STM32_DMA_CR_MSIZE_BYTE |
                      STM32_DMA_CR_MINC |
                      STM32_DMA_CR_DMEIE |
                      STM32_DMA_CR_TEIE;
  nvicEnableVector(STM32_OCTOSPI2_NUMBER, STM32_WSPI_OCTOSPI2_IRQ_PRIORITY);
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
  uint32_t dcr2;

  /* If in stopped state then full initialization.*/
  if (wspip->state == WSPI_STOP) {
#if STM32_WSPI_USE_OCTOSPI1
    if (&WSPID1 == wspip) {
      wspip->dma = dmaStreamAllocI(STM32_WSPI_OCTOSPI1_DMA_STREAM,
                                   STM32_WSPI_OCTOSPI1_DMA_IRQ_PRIORITY,
                                   (stm32_dmaisr_t)wspi_lld_serve_dma_interrupt,
                                   (void *)wspip);
      osalDbgAssert(wspip->dma != NULL, "unable to allocate stream");
      rccEnableOCTOSPI1(true);
      dmaSetRequestSource(wspip->dma, STM32_DMAMUX1_OCTOSPI1);
      dcr2 = STM32_DCR2_PRESCALER(STM32_WSPI_OCTOSPI1_PRESCALER_VALUE - 1U);
   }
#endif

#if STM32_WSPI_USE_OCTOSPI2
    if (&WSPID2 == wspip) {
      wspip->dma = dmaStreamAllocI(STM32_WSPI_OCTOSPI2_DMA_STREAM,
                                   STM32_WSPI_OCTOSPI2_DMA_IRQ_PRIORITY,
                                   (stm32_dmaisr_t)wspi_lld_serve_dma_interrupt,
                                   (void *)wspip);
      osalDbgAssert(wspip->dma != NULL, "unable to allocate stream");
      rccEnableOCTOSPI2(true);
      dmaSetRequestSource(wspip->dma, STM32_DMAMUX1_OCTOSPI2);
      dcr2 = STM32_DCR2_PRESCALER(STM32_WSPI_OCTOSPI2_PRESCALER_VALUE - 1U);
    }
#endif

    /* Common initializations.*/
    dmaStreamSetPeripheral(wspip->dma, &wspip->ospi->DR);
  }

  /* WSPI setup and enable.*/
  wspip->ospi->DCR1 = wspip->config->dcr1;
  wspip->ospi->DCR2 = wspip->config->dcr2 | dcr2;
  wspip->ospi->DCR3 = wspip->config->dcr3;
  wspip->ospi->CR   = OCTOSPI_CR_TCIE  | OCTOSPI_CR_DMAEN | OCTOSPI_CR_EN;
  wspip->ospi->FCR  = OCTOSPI_FCR_CTEF | OCTOSPI_FCR_CTCF |
                      OCTOSPI_FCR_CSMF | OCTOSPI_FCR_CTOF;
}

/**
 * @brief   Deactivates the WSPI peripheral.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 *
 * @notapi
 */
void wspi_lld_stop(WSPIDriver *wspip) {

  /* Waiting for the previous operation to complete, if any.*/
  wspi_lld_sync(wspip);

  /* If in ready state then disables the OCTOSPI clock.*/
  if (wspip->state == WSPI_READY) {

    /* WSPI disable.*/
    wspip->ospi->CR = 0U;

    /* Releasing the DMA.*/
    dmaStreamFreeI(wspip->dma);
    wspip->dma = NULL;

    /* Stopping involved clocks.*/
#if STM32_WSPI_USE_OCTOSPI1
    if (&WSPID1 == wspip) {
      rccDisableOCTOSPI1();
    }
#endif

#if STM32_WSPI_USE_OCTOSPI2
    if (&WSPID2 == wspip) {
      rccDisableOCTOSPI2();
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

#if 0 //STM32_USE_STM32_D1_WORKAROUND == TRUE
  /* If it is a command without address and alternate phases then the command
     is sent as an alternate byte, the command phase is suppressed.*/
  if ((cmdp->cfg & (WSPI_CFG_ADDR_MODE_MASK | WSPI_CFG_ALT_MODE_MASK)) == 0U) {
    /* The command mode field is copied in the alternate mode field. All
       other fields are not used in this scenario.*/
    wspip->ospi->DLR = 0U;
    wspip->ospi->ABR = cmdp->cmd;
    wspip->ospi->CCR = (cmdp->cfg  & WSPI_CFG_CMD_MODE_MASK) << 6U;
    return;
  }
#endif
  wspip->ospi->CR &= ~OCTOSPI_CR_FMODE;
  wspip->ospi->DLR = 0U;
  wspip->ospi->TCR = cmdp->dummy | wspip->extra_tcr;
  wspip->ospi->CCR = cmdp->cfg;
  wspip->ospi->ABR = cmdp->alt;
  wspip->ospi->IR  = cmdp->cmd;
  if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE) {
    wspip->ospi->AR  = cmdp->addr;
  }
}

/**
 * @brief   Sends a command with data over the WSPI bus.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    If using DTR in 8 lines mode then the following restrictions
 *          apply:
 *          - Command size must be 0, 2 or 4 bytes.
 *          - Address must be even.
 *          - Alternate bytes size must be 0, 2 or 4 bytes.
 *          - Data size must be a multiple of two.
 *          .
 *          There is no check on the above conditions in order to keep the
 *          code efficient.
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

  wspip->ospi->CR &= ~OCTOSPI_CR_FMODE;
  wspip->ospi->DLR = n - 1U;
  wspip->ospi->TCR = cmdp->dummy | wspip->extra_tcr;
  wspip->ospi->CCR = cmdp->cfg;
  wspip->ospi->ABR = cmdp->alt;
  wspip->ospi->IR  = cmdp->cmd;
  if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE) {
    wspip->ospi->AR  = cmdp->addr;
  }

  dmaStreamEnable(wspip->dma);
}

/**
 * @brief   Sends a command then receives data over the WSPI bus.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    If using DTR in 8 lines mode then the following restrictions
 *          apply:
 *          - Command size must be 0, 2 or 4 bytes.
 *          - Address must be even.
 *          - Alternate bytes size must be 0, 2 or 4 bytes.
 *          - Data size must be a multiple of two.
 *          .
 *          There is no check on the above conditions in order to keep the
 *          code efficient.
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

  wspip->ospi->CR  = (wspip->ospi->CR & ~OCTOSPI_CR_FMODE) | OCTOSPI_CR_FMODE_0;
  wspip->ospi->DLR = n - 1U;
  wspip->ospi->TCR = cmdp->dummy | wspip->extra_tcr;
  wspip->ospi->CCR = cmdp->cfg;
  wspip->ospi->ABR = cmdp->alt;
  wspip->ospi->IR  = cmdp->cmd;
  if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE) {
    wspip->ospi->AR  = cmdp->addr;
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

  /* Starting memory mapped mode using the passed parameters.*/
  wspip->ospi->CR   = OCTOSPI_CR_FMODE_1 | OCTOSPI_CR_FMODE_0 | OCTOSPI_CR_EN;
  wspip->ospi->TCR  = cmdp->dummy | wspip->extra_tcr;
  wspip->ospi->CCR  = cmdp->cfg;
  wspip->ospi->IR   = cmdp->cmd;
  wspip->ospi->ABR  = 0U;
  wspip->ospi->AR   = 0U;
  wspip->ospi->WTCR = 0U;
  wspip->ospi->WCCR = 0U;
  wspip->ospi->WIR  = 0U;
  wspip->ospi->WABR = 0U;

  /* Mapped flash absolute base address.*/
#if STM32_WSPI_USE_OCTOSPI1
  if (&WSPID1 == wspip) {
    if (addrp != NULL) {
      *addrp = (uint8_t *)0x90000000U;
    }
  }
#endif
#if STM32_WSPI_USE_OCTOSPI2
  if (&WSPID2 == wspip) {
    if (addrp != NULL) {
      *addrp = (uint8_t *)0x70000000U;
    }
  }
#endif
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
  wspip->ospi->CR |= OCTOSPI_CR_ABORT;
  while ((wspip->ospi->CR & OCTOSPI_CR_ABORT) != 0U) {
  }

  /* Disabling memory mapped mode and re-enabling DMA and IRQs.*/
  wspip->ospi->CR = OCTOSPI_CR_TCIE | OCTOSPI_CR_DMAEN | OCTOSPI_CR_EN;
}
#endif /* WSPI_SUPPORTS_MEMMAP == TRUE */

#endif /* HAL_USE_WSPI */

/** @} */
