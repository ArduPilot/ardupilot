/*
    ChibiOS/HAL - Copyright (C) 2016 Uladzimir Pylinsky aka barthess

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
 * @file    lib_scsi.c
 * @brief   SCSI target driver source code.
 *
 * @addtogroup SCSI
 * @{
 */

#include <string.h>

#include "hal.h"

#include "lib_scsi.h"

#define DEBUG_TRACE_PRINT     FALSE
#define DEBUG_TRACE_WARNING   FALSE
#define DEBUG_TRACE_ERROR     FALSE
// #include "dbgtrace.h"

#define ARCH_LITTLE_ENDIAN
#include "bswap.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

typedef struct {
  uint32_t first_lba;
  uint16_t blk_cnt;
} data_request_t;

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Combines data request from byte array.
 *
 * @notapi
 */
static data_request_t decode_data_request(const uint8_t *cmd) {

  data_request_t req;
  uint32_t lba;
  uint16_t blk;

  memcpy(&lba, &cmd[2], sizeof(lba));
  memcpy(&blk, &cmd[7], sizeof(blk));

  req.first_lba = be32_to_cpu(lba);
  req.blk_cnt = be16_to_cpu(blk);

  return req;
}

/**
 * @brief   Fills sense structure.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] key     SCSI sense key
 * @param[in] code    SCSI sense code
 * @param[in] qual    SCSI sense qualifier
 *
 * @notapi
 */
static void set_sense(SCSITarget *scsip, uint8_t key,
                      uint8_t code, uint8_t qual) {

  scsi_sense_response_t *sense = &scsip->sense;
  memset(sense, 0 , sizeof(scsi_sense_response_t));

  sense->byte[0]  = 0x70;
  sense->byte[2]  = key;
  sense->byte[7]  = 8;
  sense->byte[12] = code;
  sense->byte[13] = qual;
}

/**
 * @brief   Sets all values in sense data to 'success' condition.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 *
 * @notapi
 */
static void set_sense_ok(SCSITarget *scsip) {
  set_sense(scsip, SCSI_SENSE_KEY_GOOD,
                   SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
                   SCSI_ASENSEQ_NO_QUALIFIER);
}

/**
 * @brief   Transmits data via transport channel.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] data    pointer to data buffer
 * @param[in] len     number of bytes to be transmitted
 *
 * @return            The operation status.
 *
 * @notapi
 */
static bool transmit_data(SCSITarget *scsip, const uint8_t *data, uint32_t len) {

  const SCSITransport *trp = scsip->config->transport;
  const uint32_t residue = len - trp->transmit(trp, data, len);

  if (residue > 0) {
    scsip->residue = residue;
    return SCSI_FAILED;
  }
  else {
    return SCSI_SUCCESS;
  }
}

/**
 * @brief   Stub for unhandled SCSI commands.
 * @details Sets error flags in sense data structure and returns error error.
 */
static bool cmd_unhandled(SCSITarget *scsip, const uint8_t *cmd) {
  (void)cmd;

  set_sense(scsip, SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                   SCSI_ASENSE_INVALID_COMMAND,
                   SCSI_ASENSEQ_NO_QUALIFIER);
  return SCSI_FAILED;
}

/**
 * @brief   Stub for unrealized but required SCSI commands.
 * @details Sets sense data in 'all OK' condition and returns success status.
 */
static bool cmd_ignored(SCSITarget *scsip, const uint8_t *cmd) {
  (void)scsip;
  (void)cmd;

  return SCSI_SUCCESS;
}

/**
 * @brief   SCSI inquiry command handler.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] cmd     pointer to SCSI command data
 *
 * @return            The operation status.
 *
 * @notapi
 */
static bool inquiry(SCSITarget *scsip, const uint8_t *cmd) {

  if ((cmd[1] & 0b1) && cmd[2] == 0x80) {
    /* Unit serial number page */
    return transmit_data(scsip, (const uint8_t *)scsip->config->unit_serial_number_inquiry_response,
                                sizeof(scsi_unit_serial_number_inquiry_response_t));
  }
  else if ((cmd[1] & 0b11) || cmd[2] != 0) {
    set_sense(scsip, SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                     SCSI_ASENSE_INVALID_FIELD_IN_CDB,
                     SCSI_ASENSEQ_NO_QUALIFIER);
    return SCSI_FAILED;
  }
  else {
    return transmit_data(scsip, (const uint8_t *)scsip->config->inquiry_response,
                                sizeof(scsi_inquiry_response_t));
  }
}

/**
 * @brief   SCSI request sense command handler.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] cmd     pointer to SCSI command data
 *
 * @return            The operation status.
 *
 * @notapi
 */
static bool request_sense(SCSITarget *scsip, const uint8_t *cmd) {

  if (((cmd[1] & 0x01) != 0) || (cmd[4] != sizeof(scsi_sense_response_t))) {
    set_sense(scsip, SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                     SCSI_ASENSE_INVALID_FIELD_IN_CDB,
                     SCSI_ASENSEQ_NO_QUALIFIER);
    return SCSI_FAILED;
  }
  else {
    return transmit_data(scsip, (uint8_t *)&scsip->sense,
                                 sizeof(scsi_sense_response_t));
  }
}

/**
 * @brief   SCSI mode sense (6) command handler.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] cmd     pointer to SCSI command data
 *
 * @return            The operation status.
 *
 * @notapi
 */
static bool mode_sense6(SCSITarget *scsip, const uint8_t *cmd) {
  (void)cmd;

  scsip->mode_sense.byte[0] = sizeof(scsi_mode_sense6_response_t) - 1;
  scsip->mode_sense.byte[1] = 0;
  if (blkIsWriteProtected(scsip->config->blkdev)) {
    scsip->mode_sense.byte[2] = 0x01 << 7;
  }
  else {
    scsip->mode_sense.byte[2] = 0;
  }
  scsip->mode_sense.byte[3] = 0;

  return transmit_data(scsip, (uint8_t *)&scsip->mode_sense,
                              sizeof(scsi_mode_sense6_response_t));
}

/**
 * @brief   SCSI read format capacities command handler.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] cmd     pointer to SCSI command data
 *
 * @return            The operation status.
 *
 * @notapi
 */
static bool read_format_capacities(SCSITarget *scsip, const uint8_t *cmd) {

  /* An Allocation Length of zero indicates that no data shall be transferred.
     This condition shall not be considered as an error. The Logical Unit
     shall terminate the data transfer when Allocation Length bytes have
     been transferred or when all available data have been transferred to
     the Initiator, whatever is less. */

  uint16_t len = cmd[7] << 8 | cmd[8];

  if (0 == len) {
    return SCSI_SUCCESS;
  }
  else {
    scsi_read_format_capacities_response_t ret;
    BlockDeviceInfo bdi;
    blkGetInfo(scsip->config->blkdev, &bdi);

    uint32_t tmp = cpu_to_be32(bdi.blk_num);
    memcpy(ret.blocknum, &tmp, 4);

    uint8_t formatted_media = 0b10;
    uint16_t blocklen = bdi.blk_size;
    ret.blocklen[0] = formatted_media;
    ret.blocklen[1] = 0;
    ret.blocklen[2] = blocklen >> 8;
    ret.blocklen[3] = blocklen & 0xFF;

    ret.header[3] = 1 * 8;

    return transmit_data(scsip, (uint8_t *)&ret,
                         sizeof(scsi_read_format_capacities_response_t));
  }
}

/**
 * @brief   SCSI read capacity (10) command handler.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] cmd     pointer to SCSI command data
 *
 * @return            The operation status.
 *
 * @notapi
 */
static bool read_capacity10(SCSITarget *scsip, const uint8_t *cmd) {

  (void)cmd;

  BlockDeviceInfo bdi;
  blkGetInfo(scsip->config->blkdev, &bdi);
  scsi_read_capacity10_response_t ret;
  ret.block_size      = cpu_to_be32(bdi.blk_size);
  ret.last_block_addr = cpu_to_be32(bdi.blk_num - 1);

  return transmit_data(scsip, (uint8_t *)&ret,
                        sizeof(scsi_read_capacity10_response_t));
}

/**
 * @brief   Checks data request for media overflow.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] cmd     pointer to SCSI command data
 *
 * @return            The operation status.
 * @retval true       When media overflow detected.
 * @retval false      Otherwise.
 *
 * @notapi
 */
static bool data_overflow(SCSITarget *scsip, const data_request_t *req) {

  BlockDeviceInfo bdi;
  blkGetInfo(scsip->config->blkdev, &bdi);

  if (req->first_lba + req->blk_cnt > bdi.blk_num) {
    set_sense(scsip, SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                     SCSI_ASENSE_LBA_OUT_OF_RANGE,
                     SCSI_ASENSEQ_NO_QUALIFIER);
    return true;
  }
  else {
    return false;
  }
}

/**
 * @brief   SCSI read/write (10) command handler.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] cmd     pointer to SCSI command data
 *
 * @return            The operation status.
 *
 * @notapi
 */
static bool data_read_write10(SCSITarget *scsip, const uint8_t *cmd) {

  data_request_t req = decode_data_request(cmd);

  if (data_overflow(scsip, &req)) {
    return SCSI_FAILED;
  }
  else {
    const SCSITransport *tr = scsip->config->transport;
    BaseBlockDevice *blkdev = scsip->config->blkdev;
    BlockDeviceInfo bdi;
    blkGetInfo(blkdev, &bdi);
    size_t bs = bdi.blk_size;
    uint8_t *buf = scsip->config->blkbuf;

    size_t i = 0;
    for (i=0; i<req.blk_cnt; i++) {
      if (cmd[0] == SCSI_CMD_READ_10) {
        // TODO: block error handling
        blkRead(blkdev, req.first_lba + i, buf, 1);
        tr->transmit_async(tr, buf, bs);
      }
      else {
        // TODO: block error handling
        tr->receive(tr, buf, bs);
        blkWrite(blkdev, req.first_lba + i, buf, 1);
      }
    }
  }
  return SCSI_SUCCESS;
}

/**
 * @brief   SCSI test unit ready command handler
 * @details If block device is inserted, sets sense data in 'all OK' condition
 *          and returns success status.
 *          If block device is not inserted, sets sense data to 'Medium not present' considion,
 *          and returns check condition status.
 */
static bool test_unit_ready(SCSITarget *scsip, const uint8_t *cmd) {
  (void)cmd;

  if (blkIsInserted(scsip->config->blkdev)) {
    return SCSI_SUCCESS;
  }
  else {
    // warnprintf("SCSI Medium is not inserted.\r\n");
    set_sense(scsip, SCSI_SENSE_KEY_NOT_READY,
                     SCSI_ASENSE_MEDIUM_NOT_PRESENT,
                     SCSI_ASENSEQ_NO_QUALIFIER);
    return SCSI_FAILED;
  }

}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Executes SCSI command encoded in byte array.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] cmd     pointer to SCSI command data
 *
 * @return            The operation status.
 *
 * @api
 */
bool scsiExecCmd(SCSITarget *scsip, const uint8_t *cmd) {

  bool ret = SCSI_SUCCESS;

  switch (cmd[0]) {
  case SCSI_CMD_INQUIRY:
    // dbgprintf("SCSI_CMD_INQUIRY\r\n");
    ret = inquiry(scsip, cmd);
    break;

  case SCSI_CMD_REQUEST_SENSE:
    // dbgprintf("SCSI_CMD_REQUEST_SENSE\r\n");
    ret = request_sense(scsip, cmd);
    break;

  case SCSI_CMD_READ_CAPACITY_10:
    // dbgprintf("SCSI_CMD_READ_CAPACITY_10\r\n");
    ret = read_capacity10(scsip, cmd);
    break;

  case SCSI_CMD_READ_10:
    // dbgprintf("SCSI_CMD_READ_10\r\n");
    ret = data_read_write10(scsip, cmd);
    break;

  case SCSI_CMD_WRITE_10:
    // dbgprintf("SCSI_CMD_WRITE_10\r\n");
    ret = data_read_write10(scsip, cmd);
    break;

  case SCSI_CMD_TEST_UNIT_READY:
    // dbgprintf("SCSI_CMD_TEST_UNIT_READY\r\n");
    ret = test_unit_ready(scsip, cmd);
    break;

  case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
    // dbgprintf("SCSI_CMD_ALLOW_MEDIUM_REMOVAL\r\n");
    ret = cmd_ignored(scsip, cmd);
    break;

  case SCSI_CMD_MODE_SENSE_6:
    // dbgprintf("SCSI_CMD_MODE_SENSE_6\r\n");
    ret = mode_sense6(scsip, cmd);
    break;

  case SCSI_CMD_READ_FORMAT_CAPACITIES:
    // dbgprintf("SCSI_CMD_READ_FORMAT_CAPACITIES\r\n");
    ret = read_format_capacities(scsip, cmd);
    break;

  case SCSI_CMD_VERIFY_10:
    // dbgprintf("SCSI_CMD_VERIFY_10\r\n");
    ret = cmd_ignored(scsip, cmd);
    break;

  default:
    // warnprintf("SCSI unhandled command: %X\r\n", cmd[0]);
    ret = cmd_unhandled(scsip, cmd);
    break;
  }

  if (ret == SCSI_SUCCESS)
    set_sense_ok(scsip);

  return ret;
}

/**
 * @brief   Driver structure initialization.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 *
 * @api
 */
void scsiObjectInit(SCSITarget *scsip) {

  scsip->config = NULL;
  scsip->residue = 0;
  memset(&scsip->sense, 0 , sizeof(scsi_sense_response_t));
  scsip->state = SCSI_TRGT_STOP;
}

/**
 * @brief   Starts SCSITarget driver.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 * @param[in] config  pointer to @p SCSITargetConfig structure
 *
 * @api
 */
void scsiStart(SCSITarget *scsip, const SCSITargetConfig *config) {

  scsip->config = config;
  scsip->state = SCSI_TRGT_READY;
}

/**
 * @brief   Stops SCSITarget driver.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 *
 * @api
 */
void scsiStop(SCSITarget *scsip) {

  scsip->config = NULL;
  scsip->state = SCSI_TRGT_STOP;
}

/**
 * @brief   Retrieves residue bytes.
 *
 * @param[in] scsip   pointer to @p SCSITarget structure
 *
 * @return            Residue bytes.
 *
 * @api
 */
uint32_t scsiResidue(const SCSITarget *scsip) {

  return scsip->residue;
}

/** @} */