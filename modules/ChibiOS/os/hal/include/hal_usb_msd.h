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
 * @file    hal_usb_msd.h
 * @brief   USM mass storage device driver macros and structures.
 *
 * @addtogroup usb_msd
 * @{
 */

#ifndef HAL_USB_MSD_H
#define HAL_USB_MSD_H

#if (HAL_USE_USB_MSD == TRUE) || defined(__DOXYGEN__)

#include "lib_scsi.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define USB_MSD_DATA_EP                 0x01

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief Set the size of the USB MSD thread's stack working area
 */
#if !defined(USB_MSD_THREAD_WA_SIZE) || defined(__DOXYGEN__)
#define USB_MSD_THREAD_WA_SIZE          256
#endif

/**
 * @brief Set the priority of the USB MSD thread.  Defaults to NORMALPRIO.
 */
#if !defined(MSD_THD_PRIO) || defined(__DOXYGEN__)
#define MSD_THD_PRIO                    NORMALPRIO
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !HAL_USE_USB
#error "Mass storage Driver requires HAL_USE_USB"
#endif

#if !USB_USE_WAIT
#error "Mass storage Driver requires USB_USE_WAIT"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an USB mass storage driver.
 */
typedef struct USBMassStorageDriver USBMassStorageDriver;

/**
 * @brief   Type of a driver state machine possible states.
 */
typedef enum {
  USB_MSD_UNINIT = 0,
  USB_MSD_STOP,
  USB_MSD_READY,
} usbmsdstate_t;

/**
 * @brief   Represents command block wrapper structure.
 * @details See USB Mass Storage Class Specification.
 */
typedef struct CC_PACK {
  uint32_t  signature;
  uint32_t  tag;
  uint32_t  data_len;
  uint8_t   flags;
  uint8_t   lun;
  uint8_t   cmd_len;
  uint8_t   cmd_data[16];
} msd_cbw_t;

/**
 * @brief   Represents command status wrapper structure.
 * @details See USB Mass Storage Class Specification.
 */
typedef struct CC_PACK {
  uint32_t  signature;
  uint32_t  tag;
  uint32_t  data_residue;
  uint8_t   status;
} msd_csw_t;

/**
 * @brief   Transport handler passed to SCSI layer.
 */
typedef struct {
  /**
   * @brief   Pointer to the @p USBDriver object.
   */
  USBDriver *usbp;
  /**
   * @brief   USB endpoint number.
   */
  usbep_t   ep;


  /**
   * @brief   Tx Thread working area.
   */
  THD_WORKING_AREA(             waMSDTxWorker, USB_MSD_THREAD_WA_SIZE);

  /**
   * @brief   Tx Thread handler.
   */
  thread_reference_t            txworker;

  /**
   * @brief   USB Transmit Buffer
   */
  uint8_t *txbuf;

  /**
   * @brief   USB Transmit Length
   */
  size_t txlen;

  /**
   * @brief   USB Transmit mutex
   */
  mutex_t txmtx;

} usb_scsi_transport_handler_t;


/**
 * @brief   Structure representing an USB mass storage driver.
 */
struct USBMassStorageDriver {
  /**
   * @brief   Pointer to the @p USBDriver object.
   */
  USBDriver                     *usbp;
  /**
   * @brief   Driver state.
   */
  usbmsdstate_t                 state;
  /**
   * @brief   CBW structure.
   */
  msd_cbw_t                     cbw;
  /**
   * @brief   CSW structure.
   */
  msd_csw_t                     csw;
  /**
   * @brief   Thread working area.
   */
  THD_WORKING_AREA(             waMSDWorker, USB_MSD_THREAD_WA_SIZE);

  /**
   * @brief   Worker thread handler.
   */
  thread_reference_t            worker;

  /**
   * @brief   SCSI target driver structure.
   */
  SCSITarget                    scsi_target;
  /**
   * @brief   SCSI target configuration structure.
   */
  SCSITargetConfig              scsi_config;
  /**
   * @brief   SCSI transport structure.
   */
  SCSITransport                 scsi_transport;
  /**
   * @brief   SCSI over USB transport handler structure.
   */
  usb_scsi_transport_handler_t  usb_scsi_transport_handler;
};


/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern USBMassStorageDriver USBMSD1;

#ifdef __cplusplus
extern "C" {
#endif
  void msdObjectInit(USBMassStorageDriver *msdp);
  void msdStart(USBMassStorageDriver *msdp, USBDriver *usbp,
                BaseBlockDevice *blkdev, uint8_t *blkbuf,
                uint8_t *txbuf,
                const scsi_inquiry_response_t *scsi_inquiry_response,
                const scsi_unit_serial_number_inquiry_response_t *serialInquiry,
                scsi_block_filesystem_access_t blockFilesystemAccess,
                scsi_free_filesystem_access_t freeFilesystemAccess);
  void msdStop(USBMassStorageDriver *msdp);
  bool msd_request_hook(USBDriver *usbp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_USB_MSD */

#endif /* HAL_USB_MSD_H */

/** @} */
