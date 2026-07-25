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
 * @file    wdg_lld.h
 * @brief   WDG Driver subsystem low level driver header template.
 *
 * @addtogroup WDG
 * @{
 */

#ifndef LIB_SCSI_H_
#define LIB_SCSI_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define SCSI_CMD_TEST_UNIT_READY                0x00
#define SCSI_CMD_REQUEST_SENSE                  0x03
#define SCSI_CMD_INQUIRY                        0x12
#define SCSI_CMD_MODE_SENSE_6                   0x1A
#define SCSI_CMD_START_STOP_UNIT                0x1B
#define SCSI_CMD_SEND_DIAGNOSTIC                0x1D
#define SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL   0x1E
#define SCSI_CMD_READ_CAPACITY_10               0x25
#define SCSI_CMD_READ_FORMAT_CAPACITIES         0x23
#define SCSI_CMD_READ_10                        0x28
#define SCSI_CMD_WRITE_10                       0x2A
#define SCSI_CMD_VERIFY_10                      0x2F

#define SCSI_SENSE_KEY_GOOD                     0x00
#define SCSI_SENSE_KEY_RECOVERED_ERROR          0x01
#define SCSI_SENSE_KEY_NOT_READY                0x02
#define SCSI_SENSE_KEY_MEDIUM_ERROR             0x03
#define SCSI_SENSE_KEY_HARDWARE_ERROR           0x04
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST          0x05
#define SCSI_SENSE_KEY_UNIT_ATTENTION           0x06
#define SCSI_SENSE_KEY_DATA_PROTECT             0x07
#define SCSI_SENSE_KEY_BLANK_CHECK              0x08
#define SCSI_SENSE_KEY_VENDOR_SPECIFIC          0x09
#define SCSI_SENSE_KEY_COPY_ABORTED             0x0A
#define SCSI_SENSE_KEY_ABORTED_COMMAND          0x0B
#define SCSI_SENSE_KEY_VOLUME_OVERFLOW          0x0D
#define SCSI_SENSE_KEY_MISCOMPARE               0x0E

#define SCSI_ASENSE_NO_ADDITIONAL_INFORMATION   0x00
#define SCSI_ASENSE_LOGICAL_UNIT_NOT_READY      0x04
#define SCSI_ASENSE_INVALID_FIELD_IN_CDB        0x24
#define SCSI_ASENSE_NOT_READY_TO_READY_CHANGE   0x28
#define SCSI_ASENSE_WRITE_PROTECTED             0x27
#define SCSI_ASENSE_FORMAT_ERROR                0x31
#define SCSI_ASENSE_INVALID_COMMAND             0x20
#define SCSI_ASENSE_LBA_OUT_OF_RANGE            0x21
#define SCSI_ASENSE_MEDIUM_NOT_PRESENT          0x3A

#define SCSI_ASENSEQ_NO_QUALIFIER               0x00
#define SCSI_ASENSEQ_FORMAT_COMMAND_FAILED      0x01
#define SCSI_ASENSEQ_INIT_COMMAND_REQUIRED      0x02
#define SCSI_ASENSEQ_OPERATION_IN_PROGRESS      0x07

#define SCSI_SUCCESS                            HAL_SUCCESS
#define SCSI_FAILED                             HAL_FAILED

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an SCSI target.
 */
typedef struct SCSITarget     SCSITarget;

/**
 * @brief   Type of a structure representing an SCSI transport.
 */
typedef struct SCSITransport  SCSITransport;

/**
 * @brief   State of SCSI target.
 */
typedef enum {
  SCSI_TRGT_UNINIT = 0,
  SCSI_TRGT_STOP,
  SCSI_TRGT_READY,
} scsitrgtstate_t;

/**
 * @brief   Represents SCSI sense data structure.
 * @details See SCSI specification.
 */
typedef struct CC_PACK {
  uint8_t byte[18];
} scsi_sense_response_t;

/**
 * @brief   Represents SCSI inquiry response structure.
 * @details See SCSI specification.
 */
typedef struct CC_PACK {
  uint8_t peripheral;
  uint8_t removable;
  uint8_t version;
  uint8_t response_data_format;
  uint8_t additional_length;
  uint8_t sccstp;
  uint8_t bqueetc;
  uint8_t cmdque;
  uint8_t vendorID[8];
  uint8_t productID[16];
  uint8_t productRev[4];
} scsi_inquiry_response_t;

/**
 * @brief   Represents SCSI unit serial number inquiry response structure.
 * @details See SCSI specification.
 */
typedef struct CC_PACK {
  uint8_t peripheral;
  uint8_t page_code;
  uint8_t reserved;
  uint8_t page_length;
  uint8_t serialNumber[8];
} scsi_unit_serial_number_inquiry_response_t;
/**
 * @brief   Represents SCSI mode sense (6) request structure.
 * @details See SCSI specification.
 */
typedef struct CC_PACK {
  uint8_t   byte[6];
} scsi_mode_sense6_request_t;

/**
 * @brief   Represents SCSI mode sense (6) response structure.
 * @details See SCSI specification.
 */
typedef struct CC_PACK{
  uint8_t   byte[4];
} scsi_mode_sense6_response_t;

/**
 * @brief   Represents SCSI read capacity (10) response structure.
 * @details See SCSI specification.
 */
typedef struct CC_PACK {
  uint32_t last_block_addr;
  uint32_t block_size;
} scsi_read_capacity10_response_t;

/**
 * @brief   Represents SCSI read format capacity response structure.
 * @details See SCSI specification.
 */
typedef struct CC_PACK {
  uint8_t header[4];
  uint8_t blocknum[4];
  uint8_t blocklen[4];
} scsi_read_format_capacities_response_t;

/**
 * @brief   Type of a SCSI transport transmit call.
 *
 * @param[in] usbp      pointer to the @p SCSITransport object
 * @param[in] data      pointer to payload buffer
 * @param[in] len       payload length
 */
typedef uint32_t (*scsi_transport_transmit_t)(const SCSITransport *transport,
                                              const uint8_t *data, size_t len);

/**
 * @brief   Type of a SCSI transport transmit call.
 *
 * @param[in] usbp      pointer to the @p SCSITransport object
 * @param[out] data     pointer to receive buffer
 * @param[in] len       number of bytes to be received
 */
typedef uint32_t (*scsi_transport_receive_t)(const SCSITransport *transport,
                                             uint8_t *data, size_t len);

/**
 * @brief Type of block filesystem call.
 * 
 */
typedef void (*scsi_block_filesystem_access_t)(void);

/**
 * @brief Type of free filesystem call.
 * 
 */
typedef void (*scsi_free_filesystem_access_t)(void);

/**
 * @brief   SCSI transport structure.
 */
struct SCSITransport {
  /**
   * @brief   Transmit call provided by lower level driver.
   */
  scsi_transport_transmit_t     transmit;

  /**
   * @brief   Transmit asynchronous
   */
  scsi_transport_transmit_t   transmit_async;

  /**
   * @brief   Receive call provided by lower level driver.
   */
  scsi_transport_receive_t      receive;

  /**
   * @brief   Block Filesystem access.
   */
  scsi_block_filesystem_access_t block_filesystem_access;

  /**
   * @brief   Free Filesystem access.
   */
  scsi_free_filesystem_access_t free_filesystem_access;

  /**
   * @brief   Transport handler provided by lower level driver.
   */
  void                          *handler;
};

/**
 * @brief   SCSI target config structure.
 */
typedef struct {
  /**
   * @brief   Pointer to @p SCSITransport object.
   */
  const SCSITransport           *transport;
  /**
   * @brief   Pointer to @p BaseBlockDevice object.
   */
  BaseBlockDevice               *blkdev;
  /**
   * @brief   Pointer to data buffer for single block.
   */
  uint8_t                       *blkbuf;
  /**
   * @brief   Pointer to SCSI inquiry response object.
   */
  const scsi_inquiry_response_t *inquiry_response;
  /**
   * @brief   Pointer to SCSI unit serial number inquiry response object.
   */
  const scsi_unit_serial_number_inquiry_response_t *unit_serial_number_inquiry_response;
} SCSITargetConfig;

/**
 *
 */
struct SCSITarget {
  /**
   * @brief   Pointer to @p SCSITargetConfig object.
   */
  const SCSITargetConfig        *config;
  /**
   * @brief   Target state.
   */
  scsitrgtstate_t               state;
  /**
   * @brief   SCSI sense response structure.
   */
  scsi_sense_response_t         sense;
  /**
   * @brief   SCSI mode sense (6) response structure.
   */
  scsi_mode_sense6_response_t   mode_sense;
  /**
   * @brief   Residue bytes.
   */
  uint32_t                      residue;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void scsiObjectInit(SCSITarget *scsip);
  void scsiStart(SCSITarget *scsip, const SCSITargetConfig *config);
  void scsiStop(SCSITarget *scsip);
  bool scsiExecCmd(SCSITarget *scsip, const uint8_t *cmd);
  uint32_t scsiResidue(const SCSITarget *scsip);
#ifdef __cplusplus
}
#endif

#endif /* LIB_SCSI_H_ */

/** @} */