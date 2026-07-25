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
/*
   Parts of this file have been contributed by Matthias Blaicher.
 */

/**
 * @file    hal_mmc_spi.c
 * @brief   MMC over SPI driver code.
 *
 * @addtogroup MMC_SPI
 * @{
 */

#include <string.h>

#include "hal.h"

#if (HAL_USE_MMC_SPI == TRUE) || defined(__DOXYGEN__)

#ifdef HAL_SDCARD_SPI_HOOK
#include "spi_hook.h"
#define spiStart spiStartHook
#define spiStop spiStopHook
#define spiAcquireBus spiAcquireBusHook
#define spiReleaseBus spiReleaseBusHook
#define spiSelect spiSelectHook
#define spiUnselect spiUnselectHook
#define spiIgnore spiIgnoreHook
#define spiSend spiSendHook
#define spiReceive spiReceiveHook
#endif

#define MMC_READ_WAIT_USEC       250
#define MMC_TIMEOUT_READ         (MMC_IDLE_TIMEOUT_MS * 1000/MMC_READ_WAIT_USEC)
#define MMC_WRITE_WAIT_USEC      500
#define MMC_TIMEOUT_WRITE        (MMC_IDLE_TIMEOUT_MS * 1000/MMC_WRITE_WAIT_USEC)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/* Forward declarations required by mmc_vmt.*/
static bool mmc_is_card_inserted(void *instance);
static bool mmc_is_write_protected(void *instance);
static bool mmc_connect(void *instance);
static bool mmc_disconnect(void *instance);
static bool mmc_read(void *instance, uint32_t startblk,
                       uint8_t *buffer, uint32_t n);
static bool mmc_write(void *instance, uint32_t startblk,
                        const uint8_t *buffer, uint32_t n);
static bool mmc_sync(void *instance);
static bool mmc_get_info(void *instance, BlockDeviceInfo *bdip);

/**
 * @brief   Virtual methods table.
 */
static const struct mmc_spi_driver_vmt mmc_vmt = {
  (size_t)0,
  mmc_is_card_inserted,
  mmc_is_write_protected,
  mmc_connect,
  mmc_disconnect,
  mmc_read,
  mmc_write,
  mmc_sync,
  mmc_get_info
};

/**
 * @brief   Lookup table for CRC-7 ( based on polynomial x^7 + x^3 + 1).
 */
static const uint8_t mmc_crc7_lookup_table[256] = {
  0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f, 0x48, 0x41, 0x5a, 0x53,
  0x6c, 0x65, 0x7e, 0x77, 0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
  0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e, 0x32, 0x3b, 0x20, 0x29,
  0x16, 0x1f, 0x04, 0x0d, 0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
  0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14, 0x63, 0x6a, 0x71, 0x78,
  0x47, 0x4e, 0x55, 0x5c, 0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
  0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13, 0x7d, 0x74, 0x6f, 0x66,
  0x59, 0x50, 0x4b, 0x42, 0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
  0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69, 0x1e, 0x17, 0x0c, 0x05,
  0x3a, 0x33, 0x28, 0x21, 0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
  0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38, 0x41, 0x48, 0x53, 0x5a,
  0x65, 0x6c, 0x77, 0x7e, 0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
  0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67, 0x10, 0x19, 0x02, 0x0b,
  0x34, 0x3d, 0x26, 0x2f, 0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
  0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04, 0x6a, 0x63, 0x78, 0x71,
  0x4e, 0x47, 0x5c, 0x55, 0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
  0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a, 0x6d, 0x64, 0x7f, 0x76,
  0x49, 0x40, 0x5b, 0x52, 0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
  0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b, 0x17, 0x1e, 0x05, 0x0c,
  0x33, 0x3a, 0x21, 0x28, 0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
  0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31, 0x46, 0x4f, 0x54, 0x5d,
  0x62, 0x6b, 0x70, 0x79
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static bool mmc_is_card_inserted(void *instance) {
  MMCDriver *mmcp = (MMCDriver *)instance;
  bool err;

  err = mmcIsCardInserted(mmcp);

  return err;
}

static bool mmc_is_write_protected(void *instance) {
  MMCDriver *mmcp = (MMCDriver *)instance;
  bool err;

  err = mmcIsWriteProtected(mmcp);

  return err;
}

static bool mmc_connect(void *instance) {
  MMCDriver *mmcp = (MMCDriver *)instance;
  bool err;

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiAcquireBus(mmcp->config->spip);
#endif

  err = mmcConnect(mmcp);

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiReleaseBus(mmcp->config->spip);
#endif

  return err;
}

static bool mmc_disconnect(void *instance) {
  MMCDriver *mmcp = (MMCDriver *)instance;
  bool err;

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiAcquireBus(mmcp->config->spip);
#endif

  err = mmcDisconnect(mmcp);

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiReleaseBus(mmcp->config->spip);
#endif

  return err;
}

static bool mmc_read(void *instance, uint32_t startblk,
                uint8_t *buffer, uint32_t n) {
  MMCDriver *mmcp = (MMCDriver *)instance;
  bool err = HAL_FAILED;

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiAcquireBus(mmcp->config->spip);
#endif

  do {
    if (mmcStartSequentialRead(mmcp, startblk)) {
      break;
    }

    while (n > 0U) {
      if (mmcSequentialRead(mmcp, buffer)) {
        break;
      }
      buffer += MMCSD_BLOCK_SIZE;
      n--;
    }

    if (mmcStopSequentialRead(mmcp)) {
      break;
    }

    err = HAL_SUCCESS;
  } while (false);

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiReleaseBus(mmcp->config->spip);
#endif

  return err;
}

static bool mmc_write(void *instance, uint32_t startblk,
                 const uint8_t *buffer, uint32_t n) {
  MMCDriver *mmcp = (MMCDriver *)instance;
  bool err = HAL_FAILED;

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiAcquireBus(mmcp->config->spip);
#endif

  do {
    if (mmcStartSequentialWrite(mmcp, startblk)) {
      break;
    }

    while (n > 0U) {
      if (mmcSequentialWrite(mmcp, buffer)) {
        break;
      }
      buffer += MMCSD_BLOCK_SIZE;
      n--;
    }

    if (mmcStopSequentialWrite(mmcp)) {
      break;
    }

    err = HAL_SUCCESS;
  } while (false);

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiReleaseBus(mmcp->config->spip);
#endif

  return err;
}

static bool mmc_sync(void *instance) {
  MMCDriver *mmcp = (MMCDriver *)instance;
  bool err;

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiAcquireBus(mmcp->config->spip);
#endif

  err = mmcSync(mmcp);

#if MMC_USE_MUTUAL_EXCLUSION == TRUE
  spiReleaseBus(mmcp->config->spip);
#endif

  return err;
}

static bool mmc_get_info(void *instance, BlockDeviceInfo *bdip) {
  MMCDriver *mmcp = (MMCDriver *)instance;
  bool err;

  err = mmcGetInfo(mmcp, bdip);

  return err;
}

/**
 * @brief Calculate the MMC standard CRC-7 based on a lookup table.
 *
 * @param[in] crc       start value for CRC
 * @param[in] buffer    pointer to data buffer
 * @param[in] len       length of data
 * @return              Calculated CRC
 */
static uint8_t mmc_crc7(uint8_t crc, const uint8_t *buffer, size_t len) {

  while (len > 0U) {
    crc = mmc_crc7_lookup_table[(crc << 1) ^ (*buffer++)];
    len--;
  }
  return crc;
}

/**
 * @brief   Waits an idle condition.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @notapi
 */
static bool mmc_wait_idle(MMCDriver *mmcp) {
  unsigned i;

  for (i = 0U; i < 16U; i++) {
    (void) spiReceive(mmcp->config->spip, 1U, mmcp->buffer);
    if (mmcp->buffer[0] == 0xFFU) {
      return HAL_SUCCESS;
    }
  }

  /* Looks like it is a long wait.*/
  i = 0U;
  do {
    (void) spiReceive(mmcp->config->spip, 1U, mmcp->buffer);
    if (mmcp->buffer[0] == 0xFFU) {
      return HAL_SUCCESS;
    }

    /* Trying to be nice with the other threads.*/
    osalThreadSleepMicroseconds(MMC_WRITE_WAIT_USEC);
  } while (++i < MMC_TIMEOUT_WRITE);

  return HAL_FAILED;
}

/**
 * @brief   Waits for a response from read operation.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @return first non-idle byte or 0xFF if timeout
 *
 * @notapi
 */
static uint8_t mmc_wait_nonidle(MMCDriver *mmcp) {
  unsigned i;

  for (i = 0U; i < 16U; i++) {
    spiReceive(mmcp->config->spip, 1U, mmcp->buffer);
    if (mmcp->buffer[0] != 0xFFU) {
      return mmcp->buffer[0];
    }
  }

  /* Looks like it is a long wait.*/
  i = 0U;
  do {
    spiReceive(mmcp->config->spip, 1U, mmcp->buffer);
    if (mmcp->buffer[0] != 0xFFU) {
      return mmcp->buffer[0];
    }

    /* Trying to be nice with the other threads.*/
    osalThreadSleepMicroseconds(MMC_READ_WAIT_USEC);
  } while (++i < MMC_TIMEOUT_READ);

  return 0xFF;
}

/**
 * @brief   Sends a command header.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[in] cmd       the command id
 * @param[in] arg       the command argument
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @notapi
 */
static bool mmc_send_hdr(MMCDriver *mmcp, uint8_t cmd, uint32_t arg) {

  /* Wait for the bus to become idle if a write operation was in progress.*/
  if (cmd != MMCSD_CMD_GO_IDLE_STATE && mmc_wait_idle(mmcp) == HAL_FAILED) {
    return HAL_FAILED;
  }

  mmcp->buffer[0] = (uint8_t)0x40U | cmd;
  mmcp->buffer[1] = (uint8_t)(arg >> 24U);
  mmcp->buffer[2] = (uint8_t)(arg >> 16U);
  mmcp->buffer[3] = (uint8_t)(arg >> 8U);
  mmcp->buffer[4] = (uint8_t)arg;
  /* Calculate CRC for command header, shift to right position, add stop bit.*/
  mmcp->buffer[5] = ((mmc_crc7(0, mmcp->buffer, 5U) & 0x7FU) << 1U) | 0x01U;

  (void) spiSend(mmcp->config->spip, 6, mmcp->buffer);

  return HAL_SUCCESS;
}

/**
 * @brief   Receives a single byte response.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[out] r1p      pointer to buffer for R1 answer
 * @return              Operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @notapi
 */
static bool mmc_recvr1(MMCDriver *mmcp, uint8_t *r1p) {
  int i;

  for (i = 0; i < 9; i++) {
    (void) spiReceive(mmcp->config->spip, 1, mmcp->buffer);
    *r1p = mmcp->buffer[0];
    if (mmcp->buffer[0] != 0xFFU) {
      return HAL_SUCCESS;
    }
  }
  return HAL_FAILED;
}

/**
 * @brief   Receives a three byte response.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[out] r1p      pointer to buffer for R1 answer
 * @return              Operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @notapi
 */
static bool mmc_recvr3(MMCDriver *mmcp, uint8_t *r1p) {
  bool ret;

  ret = mmc_recvr1(mmcp, r1p);
  (void) spiReceive(mmcp->config->spip, 4, mmcp->buffer);

  return ret;
}

/**
 * @brief   Sends a command an returns a single byte response.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[in] cmd       the command id
 * @param[in] arg       the command argument
 * @param[out] r1p      pointer to buffer for R1 answer
 * @return              Operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @notapi
 */
static bool mmc_send_command_R1(MMCDriver *mmcp, uint8_t cmd,
                                uint32_t arg, uint8_t *r1p) {
  bool result;

  spiSelect(mmcp->config->spip);

  result = mmc_send_hdr(mmcp, cmd, arg);
  result = mmc_recvr1(mmcp, r1p) || result;

  spiUnselect(mmcp->config->spip);

  return result;
}

/**
 * @brief   Sends a command which returns a five bytes response (R3).
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[in] cmd       the command id
 * @param[in] arg       the command argument
 * @param[out] r1p      pointer to buffer for R1 answer
 * @return              The first byte of the response (R1) as an @p
 *                      uint8_t value.
 * @retval 0xFF         timed out.
 *
 * @notapi
 */
static bool mmc_send_command_R3(MMCDriver *mmcp, uint8_t cmd, uint32_t arg,
                                uint8_t *r1p) {
  bool result;

  spiSelect(mmcp->config->spip);

  result = mmc_send_hdr(mmcp, cmd, arg);
  result = mmc_recvr3(mmcp, r1p) || result;

  spiUnselect(mmcp->config->spip);

  return result;
}

/**
 * @brief   Reads the CSD.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[out] cmd      command
 * @param[out] cxd      pointer to the CSD/CID buffer
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @notapi
 */
static bool mmc_read_CxD(MMCDriver *mmcp, uint8_t cmd, uint32_t cxd[4]) {
  uint8_t r1;
  uint8_t *bp;
  bool result;

  spiSelect(mmcp->config->spip);

  result = mmc_send_hdr(mmcp, cmd, 0);
  result = mmc_recvr1(mmcp, &r1) || result;
  if (result || (r1 != 0x00U)) {

    spiUnselect(mmcp->config->spip);
    return HAL_FAILED;
  }

  /* Wait for data availability.*/
  if (mmc_wait_nonidle(mmcp) == 0xFEU) {
      uint32_t *wp;

      (void) spiReceive(mmcp->config->spip, 16, mmcp->buffer);
      bp = mmcp->buffer;
      for (wp = &cxd[3]; wp >= cxd; wp--) {
        *wp = ((uint32_t)bp[0] << 24U) | ((uint32_t)bp[1] << 16U) |
              ((uint32_t)bp[2] << 8U)  | (uint32_t)bp[3];
        bp += 4;
      }

      /* CRC ignored then end of transaction. */
      (void) spiIgnore(mmcp->config->spip, 2);
      spiUnselect(mmcp->config->spip);

      return HAL_SUCCESS;
    }

  spiUnselect(mmcp->config->spip);

  return HAL_FAILED;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   MMC over SPI driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void mmcInit(void) {

}

/**
 * @brief   Initializes an instance.
 *
 * @param[out] mmcp         pointer to the @p MMCDriver object
 * @param[in] buffer        operations buffer
 *
 * @init
 */
void mmcObjectInit(MMCDriver *mmcp, uint8_t *buffer) {

  mmcp->vmt             = &mmc_vmt;
  mmcp->state           = BLK_STOP;
  mmcp->config          = NULL;
  mmcp->block_addresses = false;
  mmcp->buffer          = buffer;
}

/**
 * @brief   Configures and activates the MMC peripheral.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[in] config    pointer to the @p MMCConfig object.
 * @return              The operation status.
 *
 * @api
 */
msg_t mmcStart(MMCDriver *mmcp, const MMCConfig *config) {

  osalDbgCheck((mmcp != NULL) && (config != NULL));
  osalDbgAssert((mmcp->state == BLK_STOP) || (mmcp->state == BLK_ACTIVE),
                "invalid state");

  mmcp->config = config;
  mmcp->state = BLK_ACTIVE;

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Disables the MMC peripheral.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 *
 * @api
 */
void mmcStop(MMCDriver *mmcp) {

  osalDbgCheck(mmcp != NULL);
  osalDbgAssert((mmcp->state == BLK_STOP) || (mmcp->state == BLK_ACTIVE),
                "invalid state");

  if (mmcp->config != NULL) {
    spiStop(mmcp->config->spip);
  }

  mmcp->config = NULL;
  mmcp->state  = BLK_STOP;
}

/**
 * @brief   Performs the initialization procedure on the inserted card.
 * @details This function should be invoked when a card is inserted and
 *          brings the driver in the @p MMC_READY state where it is possible
 *          to perform read and write operations.
 * @note    It is possible to invoke this function from the insertion event
 *          handler.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcConnect(MMCDriver *mmcp) {
  unsigned i;
  uint8_t r1;

  osalDbgCheck(mmcp != NULL);

  osalDbgAssert((mmcp->state == BLK_ACTIVE) || (mmcp->state == BLK_READY),
                "invalid state");

  /* Connection procedure in progress.*/
  mmcp->state = BLK_CONNECTING;
  mmcp->block_addresses = false;

  /* Slow clock mode and 128 clock pulses.*/
  (void) spiStart(mmcp->config->spip, mmcp->config->lscfg);
  (void) spiIgnore(mmcp->config->spip, 16);

  /* SPI mode selection.*/
  i = 0U;
  while (true) {
    if ((mmc_send_command_R1(mmcp, MMCSD_CMD_GO_IDLE_STATE, 0, &r1) == HAL_SUCCESS) &&
        (r1 == 0x01U)) {
      break;
    }

    if (++i >= MMC_CMD0_RETRY) {
      goto failed;
    }

    osalThreadSleepMilliseconds(10);
  }

  /* Try to detect if this is a high capacity card and switch to block
     addresses if possible.
     This method is based on "How to support SDC Ver2 and high capacity cards"
     by ElmChan.*/
  if (mmc_send_command_R3(mmcp, MMCSD_CMD_SEND_IF_COND,
                          MMCSD_CMD8_PATTERN, &r1) == HAL_FAILED) {
    goto failed;
  }
  if (r1 != 0x05U) {

    /* Switch to SDHC mode.*/
    i = 0;
    while (true) {
      if (mmc_send_command_R1(mmcp, MMCSD_CMD_APP_CMD, 0, &r1) == HAL_SUCCESS) {
        if (r1 <= 0x01U) {
          if (mmc_send_command_R3(mmcp, MMCSD_CMD_APP_OP_COND, 0x400001AAU, &r1) == HAL_SUCCESS) {
            if (r1 == 0x00U) {
              break;
            }
          }
        }
      }

      if (++i >= MMC_ACMD41_RETRY) {
        goto failed;
      }
      osalThreadSleepMilliseconds(10);
    }

    /* Execute dedicated read on OCR register */
    if (mmc_send_command_R3(mmcp, MMCSD_CMD_READ_OCR, 0, &r1) == HAL_FAILED) {
      goto failed;
    }

    /* Check if CCS is set in response. Card operates in block mode if set.*/
    if ((mmcp->buffer[0] & 0x40U) != 0U) {
      mmcp->block_addresses = true;
    }
  }

  /* Initialization.*/
  i = 0;
  while (true) {
    (void) mmc_send_command_R1(mmcp, MMCSD_CMD_INIT, 0, &r1);
    if (r1 == 0x00U) {
      break;
    }
    if (r1 != 0x01U) {
      goto failed;
    }
    if (++i >= MMC_CMD1_RETRY) {
      goto failed;
    }
    osalThreadSleepMilliseconds(10);
  }

  /* Initialization complete, full speed.*/
  (void) spiStart(mmcp->config->spip, mmcp->config->hscfg);

  /* Setting block size.*/
  if (mmc_send_command_R1(mmcp, MMCSD_CMD_SET_BLOCKLEN,
                          MMCSD_BLOCK_SIZE, &r1) == HAL_FAILED) {
    goto failed;
  }
  if (r1 != 0x00U) {
    goto failed;
  }

  /* Determine capacity.*/
  if (mmc_read_CxD(mmcp, MMCSD_CMD_SEND_CSD, mmcp->csd) == HAL_FAILED) {
    goto failed;
  }

  mmcp->capacity = _mmcsd_get_capacity(mmcp->csd);
  if (mmcp->capacity == 0U) {
    goto failed;
  }

  if (mmc_read_CxD(mmcp, MMCSD_CMD_SEND_CID, mmcp->cid) == HAL_FAILED) {
    goto failed;
  }

  mmcp->state = BLK_READY;

  return HAL_SUCCESS;

  /* Connection failed, state reset to BLK_ACTIVE.*/
failed:
  spiStop(mmcp->config->spip);

  mmcp->state = BLK_ACTIVE;

  return HAL_FAILED;
}

/**
 * @brief   Brings the driver in a state safe for card removal.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @return              The operation status.
 *
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcDisconnect(MMCDriver *mmcp) {
  bool result;

  osalDbgCheck(mmcp != NULL);

  osalSysLock();

  osalDbgAssert((mmcp->state == BLK_ACTIVE) || (mmcp->state == BLK_READY),
                "invalid state");

  if (mmcp->state == BLK_ACTIVE) {
    osalSysUnlock();
    return HAL_SUCCESS;
  }

  mmcp->state = BLK_DISCONNECTING;

  osalSysUnlock();

  /* Wait for the pending write operations to complete.*/
  (void) spiStart(mmcp->config->spip, mmcp->config->hscfg);
  spiSelect(mmcp->config->spip);

  result = mmc_wait_idle(mmcp);

  spiUnselect(mmcp->config->spip);
  spiStop(mmcp->config->spip);

  mmcp->state = BLK_ACTIVE;

  return result;
}

/**
 * @brief   Starts a sequential read.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[in] startblk  first block to read
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcStartSequentialRead(MMCDriver *mmcp, uint32_t startblk) {
  uint8_t r1;
  bool error;

  osalDbgCheck(mmcp != NULL);
  osalDbgAssert(mmcp->state == BLK_READY, "invalid state");

  /* Read operation in progress.*/
  mmcp->state = BLK_READING;

  /* (Re)starting the SPI in case it has been reprogrammed externally, it can
     happen if the SPI bus is shared among multiple peripherals.*/
  (void) spiStart(mmcp->config->spip, mmcp->config->hscfg);
  spiSelect(mmcp->config->spip);

  if (mmcp->block_addresses) {
    error = mmc_send_hdr(mmcp, MMCSD_CMD_READ_MULTIPLE_BLOCK, startblk);
  }
  else {
    error = mmc_send_hdr(mmcp, MMCSD_CMD_READ_MULTIPLE_BLOCK,
                         startblk * MMCSD_BLOCK_SIZE);
  }
  if (error) {
    goto failed;
  }

  if ((mmc_recvr1(mmcp, &r1)) ||
      (r1 != 0x00U)) {
    goto failed;
  }

  return HAL_SUCCESS;

failed:
  spiUnselect(mmcp->config->spip);

  mmcp->state = BLK_READY;

  return HAL_FAILED;
}

/**
 * @brief   Reads a block within a sequential read operation.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[out] buffer   pointer to the read buffer
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcSequentialRead(MMCDriver *mmcp, uint8_t *buffer) {

  osalDbgCheck((mmcp != NULL) && (buffer != NULL));

  if (mmcp->state != BLK_READING) {
    return HAL_FAILED;
  }

  if (mmc_wait_nonidle(mmcp) == 0xFEU) {
      spiReceive(mmcp->config->spip, MMCSD_BLOCK_SIZE, buffer);
      /* CRC ignored. */
      (void) spiIgnore(mmcp->config->spip, 2);
      return HAL_SUCCESS;
    }

  /* Timeout.*/
  spiUnselect(mmcp->config->spip);

  mmcp->state = BLK_READY;

  return HAL_FAILED;
}

/**
 * @brief   Stops a sequential read gracefully.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcStopSequentialRead(MMCDriver *mmcp) {
  uint8_t r1;
  static const uint8_t stopcmd[] = {
    (uint8_t)(0x40U | MMCSD_CMD_STOP_TRANSMISSION), 0, 0, 0, 0, 1, 0xFF
  };

  osalDbgCheck(mmcp != NULL);

  if (mmcp->state != BLK_READING) {
    return HAL_FAILED;
  }

  (void) spiSend(mmcp->config->spip, sizeof(stopcmd), stopcmd);

  /* TODO Ignoring R1 answer from the command. There is no action we could
     do on error.*/
  (void) mmc_recvr1(mmcp, &r1);

  /* Read operation finished.*/
  spiUnselect(mmcp->config->spip);

  mmcp->state = BLK_READY;

  return HAL_SUCCESS;
}

/**
 * @brief   Starts a sequential write.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[in] startblk  first block to write
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcStartSequentialWrite(MMCDriver *mmcp, uint32_t startblk) {
  uint8_t r1;
  bool error;

  osalDbgCheck(mmcp != NULL);
  osalDbgAssert(mmcp->state == BLK_READY, "invalid state");

  /* Write operation in progress.*/
  mmcp->state = BLK_WRITING;

  (void) spiStart(mmcp->config->spip, mmcp->config->hscfg);
  spiSelect(mmcp->config->spip);
  if (mmcp->block_addresses) {
    error = mmc_send_hdr(mmcp, MMCSD_CMD_WRITE_MULTIPLE_BLOCK, startblk);
  }
  else {
    error = mmc_send_hdr(mmcp, MMCSD_CMD_WRITE_MULTIPLE_BLOCK,
                         startblk * MMCSD_BLOCK_SIZE);
  }
  if (error) {
    goto failed;
  }

  if (mmc_recvr1(mmcp, &r1) || (r1 != 0x00U)) {
    goto failed;
  }

  return HAL_SUCCESS;

failed:
  spiUnselect(mmcp->config->spip);

  mmcp->state = BLK_READY;

  return HAL_FAILED;
}

/**
 * @brief   Writes a block within a sequential write operation.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[out] buffer   pointer to the write buffer
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcSequentialWrite(MMCDriver *mmcp, const uint8_t *buffer) {
  static const uint8_t start[] = {0xFF, 0xFC};

  osalDbgCheck((mmcp != NULL) && (buffer != NULL));

  if (mmcp->state != BLK_WRITING) {
    return HAL_FAILED;
  }

  (void) spiSend(mmcp->config->spip, sizeof(start), start);    /* Data prologue.   */
  (void) spiSend(mmcp->config->spip, MMCSD_BLOCK_SIZE, buffer);/* Data.            */
  (void) spiIgnore(mmcp->config->spip, 2);                     /* CRC ignored.     */
  (void) spiReceive(mmcp->config->spip, 1, mmcp->buffer);
  if ((mmcp->buffer[0] & 0x1FU) == 0x05U) {
    return mmc_wait_idle(mmcp);
  }

  /* Error.*/
  spiUnselect(mmcp->config->spip);

  mmcp->state = BLK_READY;

  return HAL_FAILED;
}

/**
 * @brief   Stops a sequential write gracefully.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcStopSequentialWrite(MMCDriver *mmcp) {
  static const uint8_t stop[] = {0xFD, 0xFF};

  osalDbgCheck(mmcp != NULL);

  if (mmcp->state != BLK_WRITING) {
    return HAL_FAILED;
  }

  (void) spiSend(mmcp->config->spip, sizeof(stop), stop);
  spiUnselect(mmcp->config->spip);

  /* Write operation finished.*/
  mmcp->state = BLK_READY;

  return HAL_SUCCESS;
}

/**
 * @brief   Waits for card idle condition.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcSync(MMCDriver *mmcp) {
  bool result;

  osalDbgCheck(mmcp != NULL);

  if (mmcp->state != BLK_READY) {
    return HAL_FAILED;
  }

  /* Synchronization operation in progress.*/
  mmcp->state = BLK_SYNCING;

  (void) spiStart(mmcp->config->spip, mmcp->config->hscfg);
  spiSelect(mmcp->config->spip);

  result = mmc_wait_idle(mmcp);

  spiUnselect(mmcp->config->spip);

  /* Synchronization operation finished.*/
  mmcp->state = BLK_READY;

  return result;
}

/**
 * @brief   Returns the media info.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[out] bdip     pointer to a @p BlockDeviceInfo structure
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcGetInfo(MMCDriver *mmcp, BlockDeviceInfo *bdip) {

  osalDbgCheck((mmcp != NULL) && (bdip != NULL));

  if (mmcp->state != BLK_READY) {
    return HAL_FAILED;
  }

  bdip->blk_num  = mmcp->capacity;
  bdip->blk_size = MMCSD_BLOCK_SIZE;

  return HAL_SUCCESS;
}

/**
 * @brief   Erases blocks.
 *
 * @param[in] mmcp      pointer to the @p MMCDriver object
 * @param[in] startblk  starting block number
 * @param[in] endblk    ending block number
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  if the operation succeeded.
 * @retval HAL_FAILED   if the operation failed.
 *
 * @api
 */
bool mmcErase(MMCDriver *mmcp, uint32_t startblk, uint32_t endblk) {
  uint8_t r1;

  osalDbgCheck((mmcp != NULL));

  /* Erase operation in progress.*/
  mmcp->state = BLK_WRITING;

  /* Handling command differences between HC and normal cards.*/
  if (!mmcp->block_addresses) {
    startblk *= MMCSD_BLOCK_SIZE;
    endblk *= MMCSD_BLOCK_SIZE;
  }

  if ((mmc_send_command_R1(mmcp, MMCSD_CMD_ERASE_RW_BLK_START,
                          startblk, &r1) == HAL_FAILED) ||
      (r1 != 0x00U)) {
    goto failed;
  }

  if ((mmc_send_command_R1(mmcp, MMCSD_CMD_ERASE_RW_BLK_END,
                          endblk, &r1) == HAL_FAILED) ||
      (r1 != 0x00U)) {
    goto failed;
  }

  if ((mmc_send_command_R1(mmcp, MMCSD_CMD_ERASE, 0, &r1) == HAL_FAILED) ||
      (r1 != 0x00U)) {
    goto failed;
  }

  mmcp->state = BLK_READY;

  return HAL_SUCCESS;

  /* Command failed, state reset to BLK_READY.*/
failed:
  mmcp->state = BLK_READY;

  return HAL_FAILED;
}

#endif /* HAL_USE_MMC_SPI == TRUE */

/** @} */
