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
 * @file    SDIOv1/hal_sdc_lld.c
 * @brief   STM32 SDC subsystem low level driver source.
 *
 * @addtogroup SDC
 * @{
 */

#include <string.h>

#include "hal.h"

#if HAL_USE_SDC || defined(__DOXYGEN__)

#include "bouncebuffer.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*
 * The following definitions are missing from some implementations, fixing
 * as zeroed masks.
 */
#if !defined(SDIO_STA_STBITERR)
#define SDIO_STA_STBITERR       0
#endif

#if !defined(SDIO_ICR_STBITERRC)
#define SDIO_ICR_STBITERRC      0
#endif

#if !defined(SDIO_ICR_CEATAENDC)
#define SDIO_ICR_CEATAENDC      0
#endif

#if !defined(SDIO_MASK_STBITERRIE)
#define SDIO_MASK_STBITERRIE    0
#endif

#define SDIO_ICR_ALL_FLAGS     0xFFFFFFFFU

#define SDIO_STA_ERROR_MASK                                                 \
  (SDIO_STA_CCRCFAIL | SDIO_STA_DCRCFAIL |                                  \
   SDIO_STA_CTIMEOUT | SDIO_STA_DTIMEOUT |                                  \
   SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR)

#define SDIO_DMA_CHANNEL                                                    \
  STM32_DMA_GETCHANNEL(STM32_SDC_SDIO_DMA_STREAM,                           \
                       STM32_SDC_SDIO_DMA_CHN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SDCD1 driver identifier.*/
SDCDriver SDCD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   SDIO default configuration.
 */
static const SDCConfig sdc_default_cfg = {
  SDC_MODE_4BIT,
  0U
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Calculates a clock divider for the specified frequency.
 * @note    The divider is calculated to not exceed the required frequency
 *          in case of non-integer division.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] f         required frequency
 * @return              The CLKCR value.
 */
static uint32_t sdc_lld_clkdiv(SDCDriver *sdcp, uint32_t f) {
  uint32_t div;

#if defined(STM32_SDC_MAX_CLOCK)
  /* Optional enforcement of an arbitrary frequency limit.*/
  if (f > STM32_SDC_MAX_CLOCK) {
    f = STM32_SDC_MAX_CLOCK;
  }
#endif

  div = sdcp->config->slowdown + ((48000000U + f - 1U) / f);
  if (div == 1U) {
    return SDIO_CLKCR_BYPASS;
  }

  return  div - 2U;
}

/**
 * @brief   Calculates the value to be put in DTIMER for timeout.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] ms        timeout in milliseconds
 * @return              The DTIMER value.
 */
__STATIC_FORCEINLINE uint32_t sdc_lld_get_timeout(SDCDriver *sdcp,
                                                  uint32_t ms) {
  uint32_t div, clkcr;

  clkcr = sdcp->sdio->CLKCR;
  if ((clkcr & SDIO_CLKCR_BYPASS) != 0U) {
    div = 1U;
  }
  else {
    div = (clkcr & SDIO_CLKCR_CLKDIV_Msk) + 2U;
  }
  return (((48000000U / (div * 2U)) / 1000U) * ms);
}

/**
 * @brief   Prepares to handle read transaction.
 * @details Designed for read special registers from card.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[out] buf      pointer to the read buffer
 * @param[in] bytes     number of bytes to read
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool sdc_lld_prepare_read_bytes(SDCDriver *sdcp,
                                       uint8_t *buf, uint32_t bytes) {
  osalDbgCheck(bytes < 0x1000000U);

  sdcp->sdio->DTIMER = sdc_lld_get_timeout(sdcp, STM32_SDC_READ_TIMEOUT_MS);

  /* Checks for errors and waits for the card to be ready for reading.*/
  if (_sdc_wait_for_transfer_state(sdcp))
    return HAL_FAILED;

  /* Prepares the DMA channel for writing.*/
  dmaStreamSetMemory0(sdcp->dma, buf);
  dmaStreamSetTransactionSize(sdcp->dma, bytes / sizeof (uint32_t));
  dmaStreamSetMode(sdcp->dma, sdcp->dmamode | STM32_DMA_CR_DIR_P2M);
  dmaStreamEnable(sdcp->dma);

  /* Setting up data transfer.*/
  sdcp->sdio->ICR   = SDIO_ICR_ALL_FLAGS;
  sdcp->sdio->MASK  = SDIO_MASK_DCRCFAILIE |
                      SDIO_MASK_DTIMEOUTIE |
                      SDIO_MASK_STBITERRIE |
                      SDIO_MASK_RXOVERRIE |
                      SDIO_MASK_DATAENDIE;
  sdcp->sdio->DLEN  = bytes;

  /* Transaction starts just after DTEN bit setting.*/
  sdcp->sdio->DCTRL = SDIO_DCTRL_DTDIR |
                      SDIO_DCTRL_DTMODE |   /* Multibyte data transfer.*/
                      SDIO_DCTRL_DMAEN |
                      SDIO_DCTRL_DTEN;

  return HAL_SUCCESS;
}

/**
 * @brief   Prepares card to handle read transaction.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  first block to read
 * @param[in] n         number of blocks to read
 * @param[in] resp      pointer to the response buffer
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool sdc_lld_prepare_read(SDCDriver *sdcp, uint32_t startblk,
                                 uint32_t n, uint32_t *resp) {

  /* Driver handles data in 512 bytes blocks (just like HC cards). But if we
     have not HC card than we must convert address from blocks to bytes.*/
  if (!(sdcp->cardmode & SDC_MODE_HIGH_CAPACITY))
    startblk *= MMCSD_BLOCK_SIZE;

  if (n > 1U) {
    /* Send read multiple blocks command to card.*/
    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_READ_MULTIPLE_BLOCK,
                                   startblk, resp) || MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }
  }
  else {
    /* Send read single block command.*/
    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_READ_SINGLE_BLOCK,
                                   startblk, resp) || MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Prepares card to handle write transaction.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  first block to read
 * @param[in] n         number of blocks to write
 * @param[in] resp      pointer to the response buffer
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool sdc_lld_prepare_write(SDCDriver *sdcp, uint32_t startblk,
                                  uint32_t n, uint32_t *resp) {

  /* Driver handles data in 512 bytes blocks (just like HC cards). But if we
     have not HC card than we must convert address from blocks to bytes.*/
  if (!(sdcp->cardmode & SDC_MODE_HIGH_CAPACITY))
    startblk *= MMCSD_BLOCK_SIZE;

  if (n > 1U) {
    /* Write multiple blocks command.*/
    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_WRITE_MULTIPLE_BLOCK,
                                   startblk, resp) || MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }
  }
  else {
    /* Write single block command.*/
    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_WRITE_BLOCK,
                                   startblk, resp) || MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Wait end of data transaction and performs finalizations.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] n         number of blocks in transaction
 * @param[in] resp      pointer to the response buffer
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 */
static bool sdc_lld_wait_transaction_end(SDCDriver *sdcp, uint32_t n,
                                         uint32_t *resp) {

  /* Note the mask is checked before going to sleep because the interrupt
     may have occurred before reaching the critical zone.*/
  osalSysLock();

  if (sdcp->sdio->MASK != 0U) {
    osalThreadSuspendS(&sdcp->thread);
  }

  /* Mask has now been set to zero by interrupt handler. */
  osalSysUnlock();

  /* Data transfer not complete, let error cleanup stop DMA.*/
  if ((sdcp->sdio->STA & SDIO_STA_DATAEND) == 0U) {
    return HAL_FAILED;
  }

  /* Waiting for transfer completion at DMA level, then the stream is disabled
     and cleared.*/
  dmaWaitCompletion(sdcp->dma);
  sdcp->sdio->DCTRL = 0U;

  /* Clearing status.*/
  sdcp->sdio->ICR = SDIO_ICR_ALL_FLAGS;

  /* Finalize transaction.*/
  if (n > 1U)
    return sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_STOP_TRANSMISSION, 0, resp);

  return HAL_SUCCESS;
}

/**
 * @brief   Gets SDC errors.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] sta       value of the STA register
 *
 * @notapi
 */
static void sdc_lld_collect_errors(SDCDriver *sdcp, uint32_t sta) {
  uint32_t errors = SDC_NO_ERROR;

  if (sta & SDIO_STA_CCRCFAIL)
    errors |= SDC_CMD_CRC_ERROR;
  if (sta & SDIO_STA_DCRCFAIL)
    errors |= SDC_DATA_CRC_ERROR;
  if (sta & SDIO_STA_CTIMEOUT)
    errors |= SDC_COMMAND_TIMEOUT;
  if (sta & SDIO_STA_DTIMEOUT)
    errors |= SDC_DATA_TIMEOUT;
  if (sta & SDIO_STA_TXUNDERR)
    errors |= SDC_TX_UNDERRUN;
  if (sta & SDIO_STA_RXOVERR)
    errors |= SDC_RX_OVERRUN;
  if (sta & SDIO_STA_STBITERR)
    errors |= SDC_STARTBIT_ERROR;

  sdcp->errors |= errors;
}

/**
 * @brief   Performs clean transaction stopping in case of errors.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] n         number of blocks in transaction
 * @param[in] resp      pointer to the response buffer
 *
 * @notapi
 */
static void sdc_lld_error_cleanup(SDCDriver *sdcp,
                                  uint32_t n,
                                  uint32_t *resp) {
  uint32_t sta;

  dmaStreamDisable(sdcp->dma);

  /* Clearing status.*/
  sta                = sdcp->sdio->STA;
  sdcp->sdio->ICR   = sta;
  sdcp->sdio->DCTRL = 0U;
  sdc_lld_collect_errors(sdcp, sta);

  if (n > 1U) {
    sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_STOP_TRANSMISSION, 0, resp);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if !defined(STM32_SDIO_HANDLER)
#error "STM32_SDIO_HANDLER not defined"
#endif
/**
 * @brief   SDIO IRQ handler.
 * @details It just wakes transaction thread, errors handling is performed in
 *          there.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_SDIO_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();

  /* Disables the source but the status flags are not reset because the
     read/write functions needs to check them.*/
  SDIO->MASK = 0U;

  osalThreadResumeI(&SDCD1.thread, MSG_OK);

  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SDC driver initialization.
 *
 * @notapi
 */
void sdc_lld_init(void) {

  sdcObjectInit(&SDCD1);
  SDCD1.thread = NULL;
  SDCD1.dma    = NULL;
  SDCD1.sdio   = SDIO;
  nvicEnableVector(STM32_SDIO_NUMBER, STM32_SDC_SDIO_IRQ_PRIORITY);
}

/**
 * @brief   Configures and activates the SDC peripheral.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @notapi
 */
void sdc_lld_start(SDCDriver *sdcp) {

  /* Checking configuration, using a default if NULL has been passed.*/
  if (sdcp->config == NULL) {
    sdcp->config = &sdc_default_cfg;
  }

  sdcp->dmamode = STM32_DMA_CR_CHSEL(SDIO_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_SDC_SDIO_DMA_PRIORITY) |
                  STM32_DMA_CR_PSIZE_WORD |
                  STM32_DMA_CR_MSIZE_WORD |
                  STM32_DMA_CR_MINC;

#if (defined(STM32F4XX) || defined(STM32F2XX))
  sdcp->dmamode |= STM32_DMA_CR_PFCTRL |
                   STM32_DMA_CR_PBURST_INCR4 |
                   STM32_DMA_CR_MBURST_INCR4;
#endif

  /* If in stopped state then clocks are enabled and DMA initialized.*/
  if (sdcp->state == BLK_STOP) {
    sdcp->dma = dmaStreamAllocI(STM32_SDC_SDIO_DMA_STREAM,
                                STM32_SDC_SDIO_IRQ_PRIORITY,
                                NULL,
                                NULL);
    osalDbgAssert(sdcp->dma != NULL, "unable to allocate stream");

    dmaStreamSetPeripheral(sdcp->dma, &sdcp->sdio->FIFO);
#if (defined(STM32F4XX) || defined(STM32F2XX))
    dmaStreamSetFIFO(sdcp->dma, STM32_DMA_FCR_DMDIS | STM32_DMA_FCR_FTH_FULL);
#endif
    rccEnableSDIO(true);
  }

  /* Configuration, card clock is initially stopped.*/
  sdcp->sdio->POWER  = 0U;
  sdcp->sdio->CLKCR  = 0U;
  sdcp->sdio->DCTRL  = 0U;
  sdcp->sdio->DTIMER = 0U;
  sdcp->sdio->ICR    = SDIO_ICR_ALL_FLAGS;
}

/**
 * @brief   Deactivates the SDC peripheral.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @notapi
 */
void sdc_lld_stop(SDCDriver *sdcp) {

  if (sdcp->state != BLK_STOP) {

    /* SDIO deactivation.*/
    sdcp->sdio->POWER  = 0U;
    sdcp->sdio->CLKCR  = 0U;
    sdcp->sdio->DCTRL  = 0U;
    sdcp->sdio->DTIMER = 0U;

    /* DMA stream released.*/
    dmaStreamFreeI(sdcp->dma);
    sdcp->dma = NULL;

    /* Clock deactivation.*/
    rccDisableSDIO();
  }
}

/**
 * @brief   Starts the SDIO clock and sets it to init mode (400kHz or less).
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @notapi
 */
void sdc_lld_start_clk(SDCDriver *sdcp) {

  /* Initial clock setting: 400kHz, 1bit mode.*/
  sdcp->sdio->CLKCR  = sdc_lld_clkdiv(sdcp, 400000U);
  sdcp->sdio->POWER |= SDIO_POWER_PWRCTRL_0 | SDIO_POWER_PWRCTRL_1;
  sdcp->sdio->CLKCR |= SDIO_CLKCR_CLKEN;

  /* Clock activation delay.*/
  osalThreadSleep(OSAL_MS2I(STM32_SDC_CLOCK_ACTIVATION_DELAY));
}

/**
 * @brief   Sets the SDIO clock to data mode (25/50 MHz or less).
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] clk       the clock mode
 *
 * @notapi
 */
void sdc_lld_set_data_clk(SDCDriver *sdcp, sdcbusclk_t clk) {

  if (SDC_CLK_50MHz == clk) {
    sdcp->sdio->CLKCR = (sdcp->sdio->CLKCR & ~(SDIO_CLKCR_BYPASS_Msk |
                                               SDIO_CLKCR_PWRSAV_Msk |
                                               SDIO_CLKCR_CLKDIV_Msk)) |
#if STM32_SDC_SDIO_PWRSAV
                         sdc_lld_clkdiv(sdcp, 50000000U) | SDIO_CLKCR_PWRSAV;
#else
                         sdc_lld_clkdiv(sdcp, 50000000U);
#endif
  }
  else {
    sdcp->sdio->CLKCR = (sdcp->sdio->CLKCR & ~(SDIO_CLKCR_BYPASS_Msk |
                                               SDIO_CLKCR_PWRSAV_Msk |
                                               SDIO_CLKCR_CLKDIV_Msk)) |
#if STM32_SDC_SDIO_PWRSAV
                         sdc_lld_clkdiv(sdcp, 25000000U) | SDIO_CLKCR_PWRSAV;
#else
                         sdc_lld_clkdiv(sdcp, 25000000U);
#endif
  }
}

/**
 * @brief   Stops the SDIO clock.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @notapi
 */
void sdc_lld_stop_clk(SDCDriver *sdcp) {

  sdcp->sdio->CLKCR = 0U;
  sdcp->sdio->POWER = 0U;
}

/**
 * @brief   Switches the bus to 1, 4 or 8 bits mode.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] mode      bus mode
 *
 * @notapi
 */
void sdc_lld_set_bus_mode(SDCDriver *sdcp, sdcbusmode_t mode) {
  uint32_t clk = sdcp->sdio->CLKCR & ~SDIO_CLKCR_WIDBUS;

  switch (mode) {
  case SDC_MODE_1BIT:
    sdcp->sdio->CLKCR = clk;
    break;
  case SDC_MODE_4BIT:
    sdcp->sdio->CLKCR = clk | SDIO_CLKCR_WIDBUS_0;
    break;
  case SDC_MODE_8BIT:
    sdcp->sdio->CLKCR = clk | SDIO_CLKCR_WIDBUS_1;
    break;
  }
}

/**
 * @brief   Sends an SDIO command with no response expected.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] cmd       card command
 * @param[in] arg       command argument
 *
 * @notapi
 */
void sdc_lld_send_cmd_none(SDCDriver *sdcp, uint8_t cmd, uint32_t arg) {

  sdcp->sdio->ARG = arg;
  sdcp->sdio->CMD = (uint32_t)cmd | SDIO_CMD_CPSMEN;
  while ((sdcp->sdio->STA & SDIO_STA_CMDSENT) == 0U) {
  }
  sdcp->sdio->ICR = SDIO_ICR_CMDSENTC;
}

/**
 * @brief   Sends an SDIO command with a short response expected.
 * @note    The CRC is not verified.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] cmd       card command
 * @param[in] arg       command argument
 * @param[out] resp     pointer to the response buffer (one word)
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool sdc_lld_send_cmd_short(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                            uint32_t *resp) {
  uint32_t sta;

  sdcp->sdio->ARG = arg;
  sdcp->sdio->CMD = (uint32_t)cmd | SDIO_CMD_WAITRESP_0 | SDIO_CMD_CPSMEN;
  while (((sta = sdcp->sdio->STA) & (SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT |
                                     SDIO_STA_CCRCFAIL)) == 0U)
    ;
  sdcp->sdio->ICR = sta & (SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT |
                           SDIO_STA_CCRCFAIL);
  if ((sta & (SDIO_STA_CTIMEOUT)) != 0U) {
    sdc_lld_collect_errors(sdcp, sta);
    return HAL_FAILED;
  }
  *resp = sdcp->sdio->RESP1;
  return HAL_SUCCESS;
}

/**
 * @brief   Sends an SDIO command with a short response expected and CRC.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] cmd       card command
 * @param[in] arg       command argument
 * @param[out] resp     pointer to the response buffer (one word)
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool sdc_lld_send_cmd_short_crc(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                                uint32_t *resp) {
  uint32_t sta;

  sdcp->sdio->ARG = arg;
  sdcp->sdio->CMD = (uint32_t)cmd | SDIO_CMD_WAITRESP_0 | SDIO_CMD_CPSMEN;
  while (((sta = sdcp->sdio->STA) & (SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT |
                                     SDIO_STA_CCRCFAIL)) == 0U)
    ;
  sdcp->sdio->ICR = sta & (SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL);
  if ((sta & (SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL)) != 0U) {
    sdc_lld_collect_errors(sdcp, sta);
    return HAL_FAILED;
  }
  *resp = sdcp->sdio->RESP1;
  return HAL_SUCCESS;
}

/**
 * @brief   Sends an SDIO command with a long response expected and CRC.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] cmd       card command
 * @param[in] arg       command argument
 * @param[out] resp     pointer to the response buffer (four words)
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool sdc_lld_send_cmd_long_crc(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                               uint32_t *resp) {
  uint32_t sta;

  (void)sdcp;

  sdcp->sdio->ARG = arg;
  sdcp->sdio->CMD = (uint32_t)cmd | SDIO_CMD_WAITRESP_0 | SDIO_CMD_WAITRESP_1 |
                                    SDIO_CMD_CPSMEN;
  while (((sta = sdcp->sdio->STA) & (SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT |
                                     SDIO_STA_CCRCFAIL)) == 0U)
    ;
  sdcp->sdio->ICR = sta & (SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT |
                           SDIO_STA_CCRCFAIL);
  if ((sta & (SDIO_STA_ERROR_MASK)) != 0U) {
    sdc_lld_collect_errors(sdcp, sta);
    return HAL_FAILED;
  }
  /* Save bytes in reverse order because MSB in response comes first.*/
  *resp++ = sdcp->sdio->RESP4;
  *resp++ = sdcp->sdio->RESP3;
  *resp++ = sdcp->sdio->RESP2;
  *resp   = sdcp->sdio->RESP1;
  return HAL_SUCCESS;
}

/**
 * @brief   Reads special registers using data bus.
 * @details Needs only during card detection procedure.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[out] buf      pointer to the read buffer
 * @param[in] bytes     number of bytes to read
 * @param[in] cmd       card command
 * @param[in] arg       argument for command
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool sdc_lld_read_special(SDCDriver *sdcp, uint8_t *buf, size_t bytes,
                          uint8_t cmd, uint32_t arg) {
  uint32_t resp[1];

  if (!bouncebuffer_setup_read(sdcp->bouncebuffer, &buf, bytes)) {
      return HAL_FAILED;
  }

  if (sdc_lld_prepare_read_bytes(sdcp, buf, bytes))
    goto error;

  if (sdc_lld_send_cmd_short_crc(sdcp, cmd, arg, resp)
                                 || MMCSD_R1_ERROR(resp[0]))
    goto error;

  if (sdc_lld_wait_transaction_end(sdcp, 1, resp))
    goto error;

  bouncebuffer_finish_read(sdcp->bouncebuffer, buf, bytes);

  return HAL_SUCCESS;

error:
  sdc_lld_error_cleanup(sdcp, 1, resp);
  bouncebuffer_finish_read(sdcp->bouncebuffer, buf, bytes);
  return HAL_FAILED;
}

/**
 * @brief   Reads one or more blocks.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  first block to read
 * @param[out] buf      pointer to the read buffer
 * @param[in] blocks    number of blocks to read
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool sdc_lld_read_aligned(SDCDriver *sdcp, uint32_t startblk,
                          uint8_t *buf, uint32_t blocks) {
  uint32_t resp[1];

  osalDbgCheck(blocks < 0x1000000 / MMCSD_BLOCK_SIZE);

  sdcp->sdio->DTIMER = sdc_lld_get_timeout(sdcp, STM32_SDC_READ_TIMEOUT_MS);

  /* Checks for errors and waits for the card to be ready for reading.*/
  if (_sdc_wait_for_transfer_state(sdcp))
    return HAL_FAILED;


  if (!bouncebuffer_setup_read(sdcp->bouncebuffer, &buf, blocks * MMCSD_BLOCK_SIZE)) {
      return HAL_FAILED;
  }

  /* Prepares the DMA channel for reading.*/
  dmaStreamSetMemory0(sdcp->dma, buf);
  dmaStreamSetTransactionSize(sdcp->dma,
                              (blocks * MMCSD_BLOCK_SIZE) / sizeof (uint32_t));
  dmaStreamSetMode(sdcp->dma, sdcp->dmamode | STM32_DMA_CR_DIR_P2M);
  dmaStreamEnable(sdcp->dma);

  /* Setting up data transfer.*/
  sdcp->sdio->ICR   = SDIO_ICR_ALL_FLAGS;
  sdcp->sdio->MASK  = SDIO_MASK_DCRCFAILIE |
                      SDIO_MASK_DTIMEOUTIE |
                      SDIO_MASK_STBITERRIE |
                      SDIO_MASK_RXOVERRIE |
                      SDIO_MASK_DATAENDIE;
  sdcp->sdio->DLEN  = blocks * MMCSD_BLOCK_SIZE;

  if (sdc_lld_prepare_read(sdcp, startblk, blocks, resp) == true)
    goto error;

  /* Transaction starts just after DTEN bit setting.*/
  sdcp->sdio->DCTRL = SDIO_DCTRL_DTDIR |
                      SDIO_DCTRL_DBLOCKSIZE_3 |
                      SDIO_DCTRL_DBLOCKSIZE_0 |
                      SDIO_DCTRL_DMAEN |
                      SDIO_DCTRL_DTEN;

  if (sdc_lld_wait_transaction_end(sdcp, blocks, resp) == true)
    goto error;

  bouncebuffer_finish_read(sdcp->bouncebuffer, buf, blocks * MMCSD_BLOCK_SIZE);

  return HAL_SUCCESS;

error:
  sdc_lld_error_cleanup(sdcp, blocks, resp);
  bouncebuffer_finish_read(sdcp->bouncebuffer, buf, blocks * MMCSD_BLOCK_SIZE);
  return HAL_FAILED;
}

/**
 * @brief   Writes one or more blocks.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  first block to write
 * @param[out] buf      pointer to the write buffer
 * @param[in] n         number of blocks to write
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool sdc_lld_write_aligned(SDCDriver *sdcp, uint32_t startblk,
                           const uint8_t *buf, uint32_t blocks) {
  uint32_t resp[1];

  osalDbgCheck(blocks < 0x1000000 / MMCSD_BLOCK_SIZE);

  sdcp->sdio->DTIMER = sdc_lld_get_timeout(sdcp, STM32_SDC_WRITE_TIMEOUT_MS);

  /* Checks for errors and waits for the card to be ready for writing.*/
  if (_sdc_wait_for_transfer_state(sdcp))
    return HAL_FAILED;

  if (!bouncebuffer_setup_write(sdcp->bouncebuffer, &buf, blocks * MMCSD_BLOCK_SIZE)) {
      return HAL_FAILED;
  }
  
  /* Prepares the DMA channel for writing.*/
  dmaStreamSetMemory0(sdcp->dma, buf);
  dmaStreamSetTransactionSize(sdcp->dma,
                             (blocks * MMCSD_BLOCK_SIZE) / sizeof (uint32_t));
  dmaStreamSetMode(sdcp->dma, sdcp->dmamode | STM32_DMA_CR_DIR_M2P);
  dmaStreamEnable(sdcp->dma);

  /* Setting up data transfer.*/
  sdcp->sdio->ICR   = SDIO_ICR_ALL_FLAGS;
  sdcp->sdio->MASK  = SDIO_MASK_DCRCFAILIE |
                      SDIO_MASK_DTIMEOUTIE |
                      SDIO_MASK_STBITERRIE |
                      SDIO_MASK_TXUNDERRIE |
                      SDIO_MASK_DATAENDIE;
  sdcp->sdio->DLEN  = blocks * MMCSD_BLOCK_SIZE;

  /* Talk to card what we want from it.*/
  if (sdc_lld_prepare_write(sdcp, startblk, blocks, resp) == true)
    goto error;

  /* Transaction starts just after DTEN bit setting.*/
  sdcp->sdio->DCTRL = SDIO_DCTRL_DBLOCKSIZE_3 |
                      SDIO_DCTRL_DBLOCKSIZE_0 |
                      SDIO_DCTRL_DMAEN |
                      SDIO_DCTRL_DTEN;

  if (sdc_lld_wait_transaction_end(sdcp, blocks, resp) == true)
    goto error;

  bouncebuffer_finish_write(sdcp->bouncebuffer, buf);

  return HAL_SUCCESS;

error:
  sdc_lld_error_cleanup(sdcp, blocks, resp);
  bouncebuffer_finish_write(sdcp->bouncebuffer, buf);
  return HAL_FAILED;
}

/**
 * @brief   Reads one or more blocks.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  first block to read
 * @param[out] buf      pointer to the read buffer
 * @param[in] blocks    number of blocks to read
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool sdc_lld_read(SDCDriver *sdcp, uint32_t startblk,
                  uint8_t *buf, uint32_t blocks) {

#if STM32_SDC_SDIO_UNALIGNED_SUPPORT
  if (((unsigned)buf & 3U) != 0U) {
    uint32_t i;
    for (i = 0U; i < blocks; i++) {
      if (sdc_lld_read_aligned(sdcp, startblk, sdcp->buf, 1)) {
        return HAL_FAILED;
      }
      memcpy(buf, sdcp->buf, MMCSD_BLOCK_SIZE);
      buf += MMCSD_BLOCK_SIZE;
      startblk++;
    }
    return HAL_SUCCESS;
  }
#else /* !STM32_SDC_SDIO_UNALIGNED_SUPPORT */
  osalDbgAssert((((unsigned)buf & 3U) == 0U), "unaligned buffer");
#endif /* !STM32_SDC_SDIO_UNALIGNED_SUPPORT */
  return sdc_lld_read_aligned(sdcp, startblk, buf, blocks);
}

/**
 * @brief   Writes one or more blocks.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  first block to write
 * @param[out] buf      pointer to the write buffer
 * @param[in] blocks    number of blocks to write
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool sdc_lld_write(SDCDriver *sdcp, uint32_t startblk,
                   const uint8_t *buf, uint32_t blocks) {

#if STM32_SDC_SDIO_UNALIGNED_SUPPORT
  if (((unsigned)buf & 3U) != 0U) {
    uint32_t i;
    for (i = 0U; i < blocks; i++) {
      memcpy(sdcp->buf, buf, MMCSD_BLOCK_SIZE);
      buf += MMCSD_BLOCK_SIZE;
      if (sdc_lld_write_aligned(sdcp, startblk, sdcp->buf, 1))
        return HAL_FAILED;
      startblk++;
    }
    return HAL_SUCCESS;
  }
#else /* !STM32_SDC_SDIO_UNALIGNED_SUPPORT */
  osalDbgAssert((((unsigned)buf & 3U) == 0U), "unaligned buffer");
#endif /* !STM32_SDC_SDIO_UNALIGNED_SUPPORT */
  return sdc_lld_write_aligned(sdcp, startblk, buf, blocks);
}

/**
 * @brief   Waits for card idle condition.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  the operation succeeded.
 * @retval HAL_FAILED   the operation failed.
 *
 * @api
 */
bool sdc_lld_sync(SDCDriver *sdcp) {

  /* CHTODO: Implement.*/
  (void)sdcp;
  return HAL_SUCCESS;
}

#endif /* HAL_USE_SDC */

/** @} */
