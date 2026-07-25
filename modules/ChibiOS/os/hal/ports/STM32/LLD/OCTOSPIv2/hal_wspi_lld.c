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

/* @brief MDMA HW request is QSPI FIFO threshold Flag */
#define MDMA_REQUEST_OCTOSPI1_FIFO_TH     ((uint32_t)0x00000016U)

/* @brief MDMA HW request is QSPI Transfer complete Flag */
#define MDMA_REQUEST_OCTOSPI1_TC          ((uint32_t)0x00000017U)

/* @brief MDMA HW request is QSPI FIFO threshold Flag */
#define MDMA_REQUEST_OCTOSPI2_FIFO_TH     ((uint32_t)0x00000020U)

/* @brief MDMA HW request is QSPI Transfer complete Flag */
#define MDMA_REQUEST_OCTOSPI2_TC          ((uint32_t)0x00000021U)

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
static void wspi_lld_serve_mdma_interrupt(WSPIDriver *wspip, uint32_t flags) {

  (void)wspip;
  (void)flags;

  /* DMA errors handling.*/
#if defined(STM32_WSPI_MDMA_ERROR_HOOK)
  if ((flags & STM32_MDMA_CISR_TEIF) != 0) {
    STM32_WSPI_MDMA_ERROR_HOOK(wspip);
  }
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

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
  WSPID1.mdma       = NULL;
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
  WSPID2.mdma       = NULL;
#endif

  /* Shared unit, enabling it here.*/
  rccEnableOCTOSPIM(false);
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
      wspip->mdma = mdmaChannelAllocI(STM32_WSPI_OCTOSPI1_MDMA_CHANNEL,
                                       (stm32_mdmaisr_t)wspi_lld_serve_mdma_interrupt,
                                       (void *)wspip);
      osalDbgAssert(wspip->mdma != NULL, "unable to allocate MDMA channel");
      rccEnableOCTOSPI1(true);
      mdmaChannelSetTrigModeX(wspip->mdma, MDMA_REQUEST_OCTOSPI1_FIFO_TH);
      dcr2 = STM32_DCR2_PRESCALER(STM32_WSPI_OCTOSPI1_PRESCALER_VALUE - 1U);
    }
#endif

#if STM32_WSPI_USE_OCTOSPI2
    if (&WSPID2 == wspip) {
      wspip->mdma = mdmaChannelAllocI(STM32_WSPI_OCTOSPI2_MDMA_CHANNEL,
                                       (stm32_mdmaisr_t)wspi_lld_serve_mdma_interrupt,
                                       (void *)wspip);
      osalDbgAssert(wspip->mdma != NULL, "unable to allocate MDMA channel");
      rccEnableOCTOSPI2(true);
      mdmaChannelSetTrigModeX(wspip->mdma, MDMA_REQUEST_OCTOSPI2_FIFO_TH);
      dcr2 = STM32_DCR2_PRESCALER(STM32_WSPI_OCTOSPI2_PRESCALER_VALUE - 1U);
    }
#endif
  }

  /* WSPI setup and enable.*/
  wspip->ospi->DCR1 = wspip->config->dcr1;
  wspip->ospi->DCR2 = wspip->config->dcr2 | dcr2;
  wspip->ospi->DCR3 = wspip->config->dcr3;
  wspip->ospi->DCR4 = wspip->config->dcr4;
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
    mdmaChannelFreeI(wspip->mdma);
    wspip->mdma = NULL;

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
  uint32_t ctcr = STM32_MDMA_CTCR_BWM_NON_BUFF  |   /* Dest. non-cacheable. */
                  STM32_MDMA_CTCR_TRGM_BUFFER   |   /* Trigger on buffer.   */
                  STM32_MDMA_CTCR_TLEN(0U)      |   /* One byte buffer.     */
                  STM32_MDMA_CTCR_DBURST_1      |   /* Assuming AXI bus.    */
                  STM32_MDMA_CTCR_SBURST_1      |   /* Assuming AXI bus.    */
                  STM32_MDMA_CTCR_DINCOS_BYTE   |   /* Byte increment.      */
                  STM32_MDMA_CTCR_SINCOS_BYTE   |   /* Byte increment.      */
                  STM32_MDMA_CTCR_DSIZE_BYTE    |   /* Destination size.    */
                  STM32_MDMA_CTCR_SSIZE_BYTE    |   /* Source size.         */
                  STM32_MDMA_CTCR_DINC_FIXED    |   /* Destination fixed.   */
                  STM32_MDMA_CTCR_SINC_INC;         /* Source incremented.  */
  uint32_t ccr  = STM32_MDMA_CCR_PL(STM32_WSPI_OCTOSPI1_MDMA_PRIORITY) |
                  STM32_MDMA_CCR_TEIE;              /* On transfer error.   */

  /* MDMA initializations.*/
  mdmaChannelSetSourceX(wspip->mdma, txbuf);
  mdmaChannelSetDestinationX(wspip->mdma, &wspip->ospi->DR);
  mdmaChannelSetTransactionSizeX(wspip->mdma, n, 0, 0);
  mdmaChannelSetModeX(wspip->mdma, ctcr, ccr);

  wspip->ospi->CR &= ~OCTOSPI_CR_FMODE;
  wspip->ospi->DLR = n - 1U;
  wspip->ospi->TCR = cmdp->dummy | wspip->extra_tcr;
  wspip->ospi->CCR = cmdp->cfg;
  wspip->ospi->ABR = cmdp->alt;
  wspip->ospi->IR  = cmdp->cmd;
  if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE) {
    wspip->ospi->AR  = cmdp->addr;
  }

  mdmaChannelEnableX(wspip->mdma);
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
  uint32_t ctcr = STM32_MDMA_CTCR_BWM_NON_BUFF  |   /* Dest. non-cacheable. */
                  STM32_MDMA_CTCR_TRGM_BUFFER   |   /* Trigger on buffer.   */
                  STM32_MDMA_CTCR_TLEN(0)       |   /* One byte buffer.     */
                  STM32_MDMA_CTCR_DBURST_1      |   /* Assuming AXI bus.    */
                  STM32_MDMA_CTCR_SBURST_1      |   /* Assuming AXI bus.    */
                  STM32_MDMA_CTCR_DINCOS_BYTE   |   /* Byte increment.      */
                  STM32_MDMA_CTCR_SINCOS_BYTE   |   /* Byte increment.      */
                  STM32_MDMA_CTCR_DSIZE_BYTE    |   /* Destination size.    */
                  STM32_MDMA_CTCR_SSIZE_BYTE    |   /* Source size.         */
                  STM32_MDMA_CTCR_DINC_INC      |   /* Destination incr.    */
                  STM32_MDMA_CTCR_SINC_FIXED;       /* Source fixed.        */
  uint32_t ccr  = STM32_MDMA_CCR_PL(STM32_WSPI_OCTOSPI1_MDMA_PRIORITY) |
                  STM32_MDMA_CCR_TEIE;              /* On transfer error.   */

  /* MDMA initializations.*/
  mdmaChannelSetSourceX(wspip->mdma, &wspip->ospi->DR);
  mdmaChannelSetDestinationX(wspip->mdma, rxbuf);
  mdmaChannelSetTransactionSizeX(wspip->mdma, n, 0, 0);
  mdmaChannelSetModeX(wspip->mdma, ctcr, ccr);

  wspip->ospi->CR  = (wspip->ospi->CR & ~OCTOSPI_CR_FMODE) | OCTOSPI_CR_FMODE_0;
  wspip->ospi->DLR = n - 1U;
  wspip->ospi->TCR = cmdp->dummy | wspip->extra_tcr;
  wspip->ospi->CCR = cmdp->cfg;
  wspip->ospi->ABR = cmdp->alt;
  wspip->ospi->IR  = cmdp->cmd;
  if ((cmdp->cfg & WSPI_CFG_ADDR_MODE_MASK) != WSPI_CFG_ADDR_MODE_NONE) {
    wspip->ospi->AR  = cmdp->addr;
  }

  mdmaChannelEnableX(wspip->mdma);
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

/**
 * @brief   Shared service routine.
 *
 * @param[in] wspip     pointer to the @p WSPIDriver object
 */
void wspi_lld_serve_interrupt(WSPIDriver *wspip) {

  wspip->ospi->FCR = OCTOSPI_FCR_CTEF | OCTOSPI_FCR_CTCF |
                     OCTOSPI_FCR_CSMF | OCTOSPI_FCR_CTOF;

  /* Portable WSPI ISR code defined in the high level driver, note, it is
     a macro.*/
  _wspi_isr_code(wspip);

  /* Stop everything, we need to give DMA enough time to complete the ongoing
     operation. Race condition hidden here.*/
  while (mdmaChannelIsEnabled(wspip->mdma)) {
    /* Waiting for MDMA transaction completion.*/
  }
}

#endif /* HAL_USE_WSPI */

/** @} */
