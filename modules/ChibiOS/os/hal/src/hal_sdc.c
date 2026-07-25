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
 * @file    hal_sdc.c
 * @brief   SDC Driver code.
 *
 * @addtogroup SDC
 * @{
 */

#include <string.h>

#include "hal.h"

#if (HAL_USE_SDC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   MMC switch mode.
 */
typedef enum {
  MMC_SWITCH_COMMAND_SET = 0,
  MMC_SWITCH_SET_BITS    = 1,
  MMC_SWITCH_CLEAR_BITS  = 2,
  MMC_SWITCH_WRITE_BYTE  = 3
} mmc_switch_t;

/**
 * @brief   SDC switch mode.
 */
typedef enum {
  SD_SWITCH_CHECK = 0,
  SD_SWITCH_SET   = 1
} sd_switch_t;

/**
 * @brief   SDC switch function.
 */
typedef enum {
  SD_SWITCH_FUNCTION_SPEED = 0,
  SD_SWITCH_FUNCTION_CMD_SYSTEM = 1,
  SD_SWITCH_FUNCTION_DRIVER_STRENGTH = 2,
  SD_SWITCH_FUNCTION_CURRENT_LIMIT = 3
} sd_switch_function_t;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Virtual methods table.
 */
static const struct SDCDriverVMT sdc_vmt = {
  (size_t)0,
  (bool (*)(void *))sdc_lld_is_card_inserted,
  (bool (*)(void *))sdc_lld_is_write_protected,
  (bool (*)(void *))sdcConnect,
  (bool (*)(void *))sdcDisconnect,
  (bool (*)(void *, uint32_t, uint8_t *, uint32_t))sdcRead,
  (bool (*)(void *, uint32_t, const uint8_t *, uint32_t))sdcWrite,
  (bool (*)(void *))sdcSync,
  (bool (*)(void *, BlockDeviceInfo *))sdcGetInfo
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
/**
 * @brief   Detects card mode.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool mode_detect(SDCDriver *sdcp) {
  uint32_t resp[1];

  /* V2.0 cards detection.*/
  if (!sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_SEND_IF_COND,
                                  MMCSD_CMD8_PATTERN, resp)) {
    sdcp->cardmode = SDC_MODE_CARDTYPE_SDV20;
    /* Voltage verification.*/
    if (((resp[0] >> 8U) & 0xFU) != 1U) {
      return HAL_FAILED;
    }
    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_APP_CMD, 0, resp) ||
        MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }
  }
  else {
    /* MMC or SD V1.1 detection.*/
    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_APP_CMD, 0, resp) ||
        MMCSD_R1_ERROR(resp[0])) {
      sdcp->cardmode = SDC_MODE_CARDTYPE_MMC;
    }
    else {
      sdcp->cardmode = SDC_MODE_CARDTYPE_SDV11;

      /* Reset error flag illegal command.*/
      sdc_lld_send_cmd_none(sdcp, MMCSD_CMD_GO_IDLE_STATE, 0);
    }
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Init procedure for MMC.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool mmc_init(SDCDriver *sdcp) {
  uint32_t ocr;
  unsigned i;
  uint32_t resp[1];
#if defined(STM32_VDD) && STM32_VDD <= 180
  ocr = 0xC0000080U;
#else
  ocr = 0xC0FF8000U;
#endif
  i = 0;
  while (true) {
    osalThreadSleepMilliseconds(10);
    if (sdc_lld_send_cmd_short(sdcp, MMCSD_CMD_INIT, ocr, resp)) {
      return HAL_FAILED;
    }
    if ((resp[0] & 0x80000000U) != 0U) {
      if ((resp[0] & 0x40000000U) != 0U) {
        sdcp->cardmode |= SDC_MODE_HIGH_CAPACITY;
      }
      break;
    }
    if (++i >= (unsigned)SDC_INIT_RETRY) {
      return HAL_FAILED;
    }
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Init procedure for SDC.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool sdc_init(SDCDriver *sdcp) {
  unsigned i;
  uint32_t ocr;
  uint32_t resp[1];

  if ((sdcp->cardmode &  SDC_MODE_CARDTYPE_MASK) == SDC_MODE_CARDTYPE_SDV20) {
    ocr = SDC_INIT_OCR_V20;
  }
  else {
    ocr = SDC_INIT_OCR;
  }

  i = 0;
  while (true) {
    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_APP_CMD, 0, resp) ||
        MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }
    if (sdc_lld_send_cmd_short(sdcp, MMCSD_CMD_APP_OP_COND, ocr, resp)) {
      return HAL_FAILED;
    }
    if ((resp[0] & 0x80000000U) != 0U) {
      if ((resp[0] & 0x40000000U) != 0U) {
        sdcp->cardmode |= SDC_MODE_HIGH_CAPACITY;
      }
      break;
    }
    if (++i >= (unsigned)SDC_INIT_RETRY) {
      return HAL_FAILED;
    }
    osalThreadSleepMilliseconds(10);
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Constructs CMD6 argument for MMC.
 *
 * @param[in] access    EXT_CSD access mode
 * @param[in] idx       EXT_CSD byte number
 * @param[in] value     value to be written in target field
 * @param[in] cmd_set   switch current command set
 *
 * @return              CMD6 argument.
 *
 * @notapi
 */
static uint32_t mmc_cmd6_construct(mmc_switch_t access, uint32_t idx,
                                   uint32_t value, uint32_t cmd_set) {

  osalDbgAssert(idx <= 191U, "This field is not writable");
  osalDbgAssert(cmd_set < 8U, "This field has only 3 bits");

  return ((uint32_t)access << 24U) | (idx << 16U) | (value << 8U) | cmd_set;
}

/**
 * @brief   Constructs CMD6 argument for SDC.
 *
 * @param[in] mode      switch/test mode
 * @param[in] function  function number to be switched
 * @param[in] value     value to be written in target function
 *
 * @return              CMD6 argument.
 *
 * @notapi
 */
static uint32_t sdc_cmd6_construct(sd_switch_t mode,
                                   sd_switch_function_t function,
                                   uint32_t value) {
  uint32_t ret = 0xFFFFFF;

  osalDbgAssert((value < 16U), "This field has only 4 bits");

  ret &= ~((uint32_t)0xFU << ((uint32_t)function * 4U));
  ret |= value << ((uint32_t)function * 4U);
  return ret | ((uint32_t)mode << 31U);
}

/**
 * @brief   Extracts information from CMD6 answer.
 *
 * @param[in] function  function number to be switched
 * @param[in] buf       buffer with answer
 *
 * @return              extracted answer.
 *
 * @notapi
 */
static uint16_t sdc_cmd6_extract_info(sd_switch_function_t function,
                                      const uint8_t *buf) {

  unsigned start = 12U - ((unsigned)function * 2U);

  return ((uint16_t)buf[start] << 8U) | (uint16_t)buf[start + 1U];
}

/**
 * @brief   Checks status after switching using CMD6.
 *
 * @param[in] function  function number to be switched
 * @param[in] buf       buffer with answer
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool sdc_cmd6_check_status(sd_switch_function_t function,
                                 const uint8_t *buf) {

  uint32_t tmp;
  uint32_t status;

  tmp = ((uint32_t)buf[14] << 16U) |
        ((uint32_t)buf[15] << 8U) |
        (uint32_t)buf[16];
  status = (tmp >> ((uint32_t)function * 4U)) & 0xFU;
  if (0xFU != status) {
    return HAL_SUCCESS;
  }
  return HAL_FAILED;
}

/**
 * @brief   Reads supported bus clock and switch SDC to appropriate mode.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[out] clk      pointer to clock enum
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool sdc_detect_bus_clk(SDCDriver *sdcp, sdcbusclk_t *clk) {
  uint32_t cmdarg;
  const size_t N = 64;
  uint8_t *tmp = sdcp->buf;

  /* Safe default.*/
  *clk = SDC_CLK_25MHz;

  /* Looks like only "high capacity" cards produce meaningful results during
     this clock detection procedure.*/
  if (0U == _mmcsd_get_slice(sdcp->csd, MMCSD_CSD_10_CSD_STRUCTURE_SLICE)) {
    *clk = SDC_CLK_25MHz;
    return HAL_SUCCESS;
  }

  /* Read switch functions' register.*/
  if (sdc_lld_read_special(sdcp, tmp, N, MMCSD_CMD_SWITCH, 0)) {
    *clk = SDC_CLK_25MHz; // always return something
    return HAL_SUCCESS;
  }

  /* Check card capabilities parsing acquired data.*/
  if ((sdc_cmd6_extract_info(SD_SWITCH_FUNCTION_SPEED, tmp) & 2U) == 2U) {
    /* Construct command to set the bus speed.*/
    cmdarg = sdc_cmd6_construct(SD_SWITCH_SET, SD_SWITCH_FUNCTION_SPEED, 1);

    /* Write constructed command and read operation status in single call.*/
    if (sdc_lld_read_special(sdcp, tmp, N, MMCSD_CMD_SWITCH, cmdarg)) {
      *clk = SDC_CLK_25MHz; // always return something
      return HAL_SUCCESS;
    }

    /* Check card answer for success status bits.*/
    if (HAL_SUCCESS == sdc_cmd6_check_status(SD_SWITCH_FUNCTION_SPEED, tmp)) {
      *clk = SDC_CLK_50MHz;
    }
    else {
      *clk = SDC_CLK_25MHz;
    }
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Reads supported bus clock and switch MMC to appropriate mode.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[out] clk      pointer to clock enum
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool mmc_detect_bus_clk(SDCDriver *sdcp, sdcbusclk_t *clk) {
  uint32_t cmdarg;
  uint32_t resp[1];

  /* Safe default.*/
  *clk = SDC_CLK_25MHz;

  cmdarg = mmc_cmd6_construct(MMC_SWITCH_WRITE_BYTE, 185, 1, 0);
  if (!(sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_SWITCH, cmdarg, resp) ||
                                   MMCSD_R1_ERROR(resp[0]))) {
    *clk = SDC_CLK_50MHz;
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Reads supported bus clock and switch card to appropriate mode.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[out] clk      pointer to clock enum
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool detect_bus_clk(SDCDriver *sdcp, sdcbusclk_t *clk) {

  if (SDC_MODE_CARDTYPE_MMC == (sdcp->cardmode & SDC_MODE_CARDTYPE_MASK)) {
    return mmc_detect_bus_clk(sdcp, clk);
  }
  return sdc_detect_bus_clk(sdcp, clk);
}

/**
 * @brief   Sets bus width for SDC.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool sdc_set_bus_width(SDCDriver *sdcp) {
  uint32_t resp[1];

  if (SDC_MODE_1BIT == sdcp->config->bus_width) {
    /* Nothing to do. Bus is already in 1bit mode.*/
    return HAL_SUCCESS;
  }
  else if (SDC_MODE_4BIT == sdcp->config->bus_width) {
    sdc_lld_set_bus_mode(sdcp, SDC_MODE_4BIT);
    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_APP_CMD, sdcp->rca, resp) ||
        MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }

    if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_SET_BUS_WIDTH, 2, resp) ||
        MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }
  }
  else {
    /* SD card does not support 8bit bus.*/
    return HAL_FAILED;
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Sets bus width for MMC.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool mmc_set_bus_width(SDCDriver *sdcp) {
  uint32_t resp[1];
  uint32_t cmdarg = mmc_cmd6_construct(MMC_SWITCH_WRITE_BYTE, 183, 0, 0);

  switch (sdcp->config->bus_width) {
  case SDC_MODE_1BIT:
    /* Nothing to do. Bus is already in 1bit mode.*/
    return HAL_SUCCESS;
  case SDC_MODE_4BIT:
    cmdarg = mmc_cmd6_construct(MMC_SWITCH_WRITE_BYTE, 183, 1, 0);
    break;
  case SDC_MODE_8BIT:
    cmdarg = mmc_cmd6_construct(MMC_SWITCH_WRITE_BYTE, 183, 2, 0);
    break;
  default:
    osalDbgAssert(false, "unexpected case");
    break;
  }

  sdc_lld_set_bus_mode(sdcp, sdcp->config->bus_width);
  if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_SWITCH, cmdarg, resp) ||
      MMCSD_R1_ERROR(resp[0])) {
    return HAL_FAILED;
  }

  return HAL_SUCCESS;
}

/**
 * @brief   Wait for the card to complete pending operations.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] crc_check CRC check flag

 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
static bool _sdc_wait_for_transfer_state_internal(SDCDriver *sdcp,
                                                  bool crc_check) {
  uint32_t resp[1];
  bool cmd_fail;

  while (true) {
    if (crc_check) {
      cmd_fail = sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_SEND_STATUS, sdcp->rca, resp);
    }
    else {
      cmd_fail = sdc_lld_send_cmd_short(sdcp, MMCSD_CMD_SEND_STATUS, sdcp->rca, resp);
    }

    if (cmd_fail || MMCSD_R1_ERROR(resp[0])) {
      return HAL_FAILED;
    }

    switch (MMCSD_R1_STS(resp[0])) {
    case MMCSD_STS_TRAN:
      return HAL_SUCCESS;
    case MMCSD_STS_DATA:
    case MMCSD_STS_RCV:
    case MMCSD_STS_PRG:
#if SDC_NICE_WAITING == TRUE
      osalThreadSleepMilliseconds(1);
#endif
      continue;
    default:
      /* The card should have been initialized so any other state is not
         valid and is reported as an error.*/
      return HAL_FAILED;
    }
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Wait for the card to complete pending operations with CRC check.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool _sdc_wait_for_transfer_state(SDCDriver *sdcp) {

  return _sdc_wait_for_transfer_state_internal(sdcp, true);
}

/**
 * @brief   Wait for the card to complete pending operations without CRC check.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @notapi
 */
bool _sdc_wait_for_transfer_state_nocrc(SDCDriver *sdcp) {

  return _sdc_wait_for_transfer_state_internal(sdcp, false);
}

/**
 * @brief   SDC Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void sdcInit(void) {

  sdc_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p SDCDriver structure.
 *
 * @param[out] sdcp     pointer to the @p SDCDriver object
 *
 * @init
 */
void sdcObjectInit(SDCDriver *sdcp) {

  sdcp->vmt      = &sdc_vmt;
  sdcp->state    = BLK_STOP;
  sdcp->errors   = SDC_NO_ERROR;
  sdcp->config   = NULL;
  sdcp->capacity = 0;
}

/**
 * @brief   Configures and activates the SDC peripheral.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] config    pointer to the @p SDCConfig object, can be @p NULL if
 *                      the driver supports a default configuration or
 *                      requires no configuration
 * @return              The operation status.
 *
 * @api
 */
msg_t sdcStart(SDCDriver *sdcp, const SDCConfig *config) {
  msg_t msg;

  osalDbgCheck(sdcp != NULL);

  osalSysLock();
  osalDbgAssert((sdcp->state == BLK_STOP) || (sdcp->state == BLK_ACTIVE),
                "invalid state");

  sdcp->config = config;

#if defined(SDC_LLD_ENHANCED_API)
  msg = sdc_lld_start(sdcp);
  if (msg == HAL_RET_SUCCESS) {
    sdcp->state = BLK_ACTIVE;
  }
  else {
    sdcp->state = BLK_STOP;
  }
#else
  sdc_lld_start(sdcp);
  sdcp->state = BLK_ACTIVE;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Deactivates the SDC peripheral.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @api
 */
void sdcStop(SDCDriver *sdcp) {

  osalDbgCheck(sdcp != NULL);

  osalSysLock();

  osalDbgAssert((sdcp->state == BLK_STOP) || (sdcp->state == BLK_ACTIVE),
                "invalid state");

  sdc_lld_stop(sdcp);
  sdcp->config = NULL;
  sdcp->state  = BLK_STOP;

  osalSysUnlock();
}

/**
 * @brief   Performs the initialization procedure on the inserted card.
 * @details This function should be invoked when a card is inserted and
 *          brings the driver in the @p BLK_READY state where it is possible
 *          to perform read and write operations.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
bool sdcConnect(SDCDriver *sdcp) {
  uint32_t resp[1];
  sdcbusclk_t clk = SDC_CLK_25MHz;

  osalDbgCheck(sdcp != NULL);
  osalDbgAssert((sdcp->state == BLK_ACTIVE) || (sdcp->state == BLK_READY),
                "invalid state");

  /* Connection procedure in progress.*/
  sdcp->state = BLK_CONNECTING;

  /* Card clock initialization.*/
  sdc_lld_start_clk(sdcp);

  /* Enforces the initial card state.*/
  sdc_lld_send_cmd_none(sdcp, MMCSD_CMD_GO_IDLE_STATE, 0);

  /* Detect card type.*/
  if (HAL_FAILED == mode_detect(sdcp)) {
    goto failed;
  }

  /* Perform specific initialization procedure.*/
  if ((sdcp->cardmode &  SDC_MODE_CARDTYPE_MASK) == SDC_MODE_CARDTYPE_MMC) {
    if (HAL_FAILED == mmc_init(sdcp)) {
      goto failed;
    }
  }
  else {
    if (HAL_FAILED == sdc_init(sdcp)) {
      goto failed;
    }
  }

  /* Reads CID.*/
  if (sdc_lld_send_cmd_long_crc(sdcp, MMCSD_CMD_ALL_SEND_CID, 0, sdcp->cid)) {
    goto failed;
  }

  /* Asks for the RCA.*/
  if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_SEND_RELATIVE_ADDR,
                                 0, &sdcp->rca)) {
    goto failed;
  }

  /* Reads CSD.*/
  if (sdc_lld_send_cmd_long_crc(sdcp, MMCSD_CMD_SEND_CSD,
                                sdcp->rca, sdcp->csd)) {
    goto failed;
  }

  /* Selects the card for operations.*/
  if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_SEL_DESEL_CARD,
                                 sdcp->rca, resp)) {
    goto failed;
  }

  /* Switches to high speed.*/
  if (detect_bus_clk(sdcp, &clk) != HAL_SUCCESS) {
    goto failed;
  }
  sdc_lld_set_data_clk(sdcp, clk);

  /* Not checking CRC here according to JEDEC Standard No. 84-B51 6.6.2 */
  if (_sdc_wait_for_transfer_state_nocrc(sdcp) != HAL_SUCCESS) {
    goto failed;
  }

  /* Reads extended CSD if needed and possible.*/
  if (SDC_MODE_CARDTYPE_MMC == (sdcp->cardmode & SDC_MODE_CARDTYPE_MASK)) {

    /* The card is a MMC, checking if it is a large device.*/
    if (_mmcsd_get_slice(sdcp->csd, MMCSD_CSD_MMC_CSD_STRUCTURE_SLICE) > 1U) {
      uint8_t *ext_csd = sdcp->buf;
      osalThreadSleepMilliseconds(10);
      if (sdc_lld_read_special(sdcp, ext_csd, 512, MMCSD_CMD_SEND_EXT_CSD, 0)) {
        goto failed;
      }

      /* Capacity from the EXT_CSD.*/
      sdcp->capacity = _mmcsd_get_capacity_ext(ext_csd);
    }
    else {
      /* Capacity from the normal CSD.*/
      sdcp->capacity = _mmcsd_get_capacity(sdcp->csd);
    }
  }
  else {
    /* The card is an SDC, capacity from the normal CSD.*/
    sdcp->capacity = _mmcsd_get_capacity(sdcp->csd);
  }

  /* Switches to high speed.*/
  if (HAL_SUCCESS != detect_bus_clk(sdcp, &clk)) {
    goto failed;
  }
  sdc_lld_set_data_clk(sdcp, clk);

  /* Block length fixed at 512 bytes.*/
  if (sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_SET_BLOCKLEN,
                                 MMCSD_BLOCK_SIZE, resp) ||
      MMCSD_R1_ERROR(resp[0])) {
    goto failed;
  }

  /* Switches to wide bus mode.*/
  switch (sdcp->cardmode & SDC_MODE_CARDTYPE_MASK) {
  case SDC_MODE_CARDTYPE_SDV11:
  case SDC_MODE_CARDTYPE_SDV20:
    if (HAL_FAILED == sdc_set_bus_width(sdcp)) {
      goto failed;
    }
    break;
  case SDC_MODE_CARDTYPE_MMC:
    if (mmc_set_bus_width(sdcp) == HAL_FAILED) {
      goto failed;
    }
    break;
  default:
    /* Unknown type.*/
    goto failed;
  }

  /* Initialization complete.*/
  sdcp->state = BLK_READY;
  return HAL_SUCCESS;

  /* Connection failed, state reset to BLK_ACTIVE.*/
failed:
  sdc_lld_stop_clk(sdcp);
  sdcp->state = BLK_ACTIVE;
  return HAL_FAILED;
}

/**
 * @brief   Brings the driver in a state safe for card removal.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
bool sdcDisconnect(SDCDriver *sdcp) {

  osalDbgCheck(sdcp != NULL);

  osalSysLock();
  osalDbgAssert((sdcp->state == BLK_ACTIVE) || (sdcp->state == BLK_READY),
                "invalid state");
  if (sdcp->state == BLK_ACTIVE) {
    osalSysUnlock();
    return HAL_SUCCESS;
  }
  sdcp->state = BLK_DISCONNECTING;
  osalSysUnlock();

  /* Waits for eventual pending operations completion.*/
  if (_sdc_wait_for_transfer_state(sdcp)) {
    sdc_lld_stop_clk(sdcp);
    sdcp->state = BLK_ACTIVE;
    return HAL_FAILED;
  }

  /* Card clock stopped.*/
  sdc_lld_stop_clk(sdcp);
  sdcp->state = BLK_ACTIVE;
  return HAL_SUCCESS;
}

/**
 * @brief   Reads one or more blocks.
 * @pre     The driver must be in the @p BLK_READY state after a successful
 *          sdcConnect() invocation.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  first block to read
 * @param[out] buf      pointer to the read buffer
 * @param[in] n         number of blocks to read
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
bool sdcRead(SDCDriver *sdcp, uint32_t startblk, uint8_t *buf, uint32_t n) {
  bool status;

  osalDbgCheck((sdcp != NULL) && (buf != NULL) && (n > 0U));
  osalDbgAssert(sdcp->state == BLK_READY, "invalid state");

  if ((startblk >= sdcp->capacity) || (n > (sdcp->capacity - startblk))) {
    sdcp->errors |= SDC_OVERFLOW_ERROR;
    return HAL_FAILED;
  }

  /* Read operation in progress.*/
  sdcp->state = BLK_READING;

  status = sdc_lld_read(sdcp, startblk, buf, n);

  /* Read operation finished.*/
  sdcp->state = BLK_READY;
  return status;
}

/**
 * @brief   Writes one or more blocks.
 * @pre     The driver must be in the @p BLK_READY state after a successful
 *          sdcConnect() invocation.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  first block to write
 * @param[in] buf       pointer to the write buffer
 * @param[in] n         number of blocks to write
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
bool sdcWrite(SDCDriver *sdcp, uint32_t startblk,
              const uint8_t *buf, uint32_t n) {
  bool status;

  osalDbgCheck((sdcp != NULL) && (buf != NULL) && (n > 0U));
  osalDbgAssert(sdcp->state == BLK_READY, "invalid state");

  if ((startblk >= sdcp->capacity) || (n > (sdcp->capacity - startblk))) {
    sdcp->errors |= SDC_OVERFLOW_ERROR;
    return HAL_FAILED;
  }

  /* Write operation in progress.*/
  sdcp->state = BLK_WRITING;

  status = sdc_lld_write(sdcp, startblk, buf, n);

  /* Write operation finished.*/
  sdcp->state = BLK_READY;
  return status;
}

/**
 * @brief   Returns the errors mask associated to the previous operation.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @return              The errors mask.
 *
 * @api
 */
sdcflags_t sdcGetAndClearErrors(SDCDriver *sdcp) {
  sdcflags_t flags;

  osalDbgCheck(sdcp != NULL);
  osalDbgAssert(sdcp->state == BLK_READY, "invalid state");

  osalSysLock();
  flags = sdcp->errors;
  sdcp->errors = SDC_NO_ERROR;
  osalSysUnlock();
  return flags;
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
bool sdcSync(SDCDriver *sdcp) {
  bool result;

  osalDbgCheck(sdcp != NULL);

  if (sdcp->state != BLK_READY) {
    return HAL_FAILED;
  }

  /* Synchronization operation in progress.*/
  sdcp->state = BLK_SYNCING;

  result = sdc_lld_sync(sdcp);

  /* Synchronization operation finished.*/
  sdcp->state = BLK_READY;
  return result;
}

/**
 * @brief   Returns the media info.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[out] bdip     pointer to a @p BlockDeviceInfo structure
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  the operation succeeded.
 * @retval HAL_FAILED   the operation failed.
 *
 * @api
 */
bool sdcGetInfo(SDCDriver *sdcp, BlockDeviceInfo *bdip) {

  osalDbgCheck((sdcp != NULL) && (bdip != NULL));

  if (sdcp->state != BLK_READY) {
    return HAL_FAILED;
  }

  bdip->blk_num = sdcp->capacity;
  bdip->blk_size = MMCSD_BLOCK_SIZE;

  return HAL_SUCCESS;
}

/**
 * @brief   Erases the supplied blocks.
 *
 * @param[in] sdcp      pointer to the @p SDCDriver object
 * @param[in] startblk  starting block number
 * @param[in] endblk    ending block number
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  the operation succeeded.
 * @retval HAL_FAILED   the operation failed.
 *
 * @api
 */
bool sdcErase(SDCDriver *sdcp, uint32_t startblk, uint32_t endblk) {
  uint32_t resp[1];

  osalDbgCheck((sdcp != NULL));
  osalDbgAssert(sdcp->state == BLK_READY, "invalid state");

  if ((startblk >= sdcp->capacity) ||
      (endblk >= sdcp->capacity) ||
      (endblk < startblk)) {
    sdcp->errors |= SDC_OVERFLOW_ERROR;
    return HAL_FAILED;
  }

  /* Erase operation in progress.*/
  sdcp->state = BLK_WRITING;

  /* Handling command differences between HC and normal cards.*/
  if ((sdcp->cardmode & SDC_MODE_HIGH_CAPACITY) == 0U) {
    startblk *= MMCSD_BLOCK_SIZE;
    endblk *= MMCSD_BLOCK_SIZE;
  }

  if (_sdc_wait_for_transfer_state(sdcp)) {
    goto failed;
  }

  if ((sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_ERASE_RW_BLK_START,
                                  startblk, resp) != HAL_SUCCESS) ||
      MMCSD_R1_ERROR(resp[0])) {
    goto failed;
  }

  if ((sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_ERASE_RW_BLK_END,
                                  endblk, resp) != HAL_SUCCESS) ||
      MMCSD_R1_ERROR(resp[0])) {
    goto failed;
  }

  if ((sdc_lld_send_cmd_short_crc(sdcp, MMCSD_CMD_ERASE,
                                  0, resp) != HAL_SUCCESS) ||
      MMCSD_R1_ERROR(resp[0])) {
    goto failed;
  }

  /* Quick sleep to allow it to transition to programming or receiving state */
  /* CHTODO: ??????????????????????????? */

  /* Wait for it to return to transfer state to indicate it has finished erasing */
  if (_sdc_wait_for_transfer_state(sdcp)) {
    goto failed;
  }

  sdcp->state = BLK_READY;
  return HAL_SUCCESS;

failed:
  sdcp->state = BLK_READY;
  return HAL_FAILED;
}

#endif /* HAL_USE_SDC == TRUE */

/** @} */
