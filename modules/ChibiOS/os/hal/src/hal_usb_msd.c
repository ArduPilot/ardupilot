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
 * @file    hal_usb_msd.c
 * @brief   USM mass storage device code.
 *
 * @addtogroup usb_msd
 * @{
 */

#include "hal.h"

#if (HAL_USE_USB_MSD == TRUE) || defined(__DOXYGEN__)

#include <string.h>

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define MSD_REQ_RESET                   0xFF
#define MSD_REQ_GET_MAX_LUN             0xFE

#define MSD_CBW_SIGNATURE               0x43425355
#define MSD_CSW_SIGNATURE               0x53425355

#define CBW_FLAGS_RESERVED_MASK         0b01111111
#define CBW_LUN_RESERVED_MASK           0b11110000
#define CBW_CMD_LEN_RESERVED_MASK       0b11000000

#define CSW_STATUS_PASSED               0x00
#define CSW_STATUS_FAILED               0x01
#define CSW_STATUS_PHASE_ERROR          0x02

#define MSD_SETUP_WORD(setup, index) (uint16_t)(((uint16_t)setup[index+1] << 8)\
                                                | (setup[index] & 0x00FF))

#define MSD_SETUP_VALUE(setup)  MSD_SETUP_WORD(setup, 2)
#define MSD_SETUP_INDEX(setup)  MSD_SETUP_WORD(setup, 4)
#define MSD_SETUP_LENGTH(setup) MSD_SETUP_WORD(setup, 6)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
/**
 * @brief   USB mass storage driver identifier.
 */
USBMassStorageDriver USBMSD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Hardcoded default SCSI inquiry response structure.
 */
static const scsi_inquiry_response_t default_scsi_inquiry_response = {
    0x00,           /* direct access block device     */
    0x80,           /* removable                      */
    0x04,           /* SPC-2                          */
    0x02,           /* response data format           */
    0x20,           /* response has 0x20 + 4 bytes    */
    0x00,
    0x00,
    0x00,
    "Chibios",
    "Mass Storage",
    {'v',CH_KERNEL_MAJOR+'0','.',CH_KERNEL_MINOR+'0'}
};

/**
 * @brief   Hardcoded default SCSI unit serial number inquiry response structure.
 */
static const scsi_unit_serial_number_inquiry_response_t default_scsi_unit_serial_number_inquiry_response =
{
    0x00,
    0x80,
    0x00,
    0x08,
    "00000000"
};


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Checks validity of CBW content.
 * @details The device shall consider the CBW valid when:
 *          • The CBW was received after the device had sent a CSW or after a reset,
 *          • the CBW is 31 (1Fh) bytes in length,
 *          • and the dCBWSignature is equal to 43425355h.
 *
 * @param[in] cbw       pointer to the @p msd_cbw_t object
 * @param[in] recvd     number of received bytes
 *
 * @return              Operation status.
 * @retval true         CBW is meaningful.
 * @retval false        CBW is bad.
 *
 * @notapi
 */
static bool cbw_valid(const msd_cbw_t *cbw, msg_t recvd) {
  if ((sizeof(msd_cbw_t) != recvd) || (cbw->signature != MSD_CBW_SIGNATURE)) {
    return false;
  }
  else {
    return true;
  }
}

/**
 * @brief   Checks meaningfulness of CBW content.
 * @details The device shall consider the contents of a valid CBW meaningful when:
 *          • no reserved bits are set,
 *          • the bCBWLUN contains a valid LUN supported by the device,
 *          • and both bCBWCBLength and the content of the CBWCB are in
 *            accordance with bInterfaceSubClass.
 *
 * @param[in] cbw       pointer to the @p msd_cbw_t object
 *
 * @return              Operation status.
 * @retval true         CBW is meaningful.
 * @retval false        CBW is bad.
 *
 * @notapi
 */
static bool cbw_meaningful(const msd_cbw_t *cbw) {
  if (((cbw->cmd_len & CBW_CMD_LEN_RESERVED_MASK) != 0)
      || ((cbw->flags & CBW_FLAGS_RESERVED_MASK) != 0)
      || (cbw->lun != 0)) {
    return false;
  }
  else {
    return true;
  }
}

/**
 * @brief   SCSI transport transmit function.
 *
 * @param[in] transport pointer to the @p SCSITransport object
 * @param[in] data      payload
 * @param[in] len       number of bytes to be transmitted
 *
 * @return              Number of successfully transmitted bytes.
 * @notapi
 */
static uint32_t scsi_transport_transmit(const SCSITransport *transport,
                                        const uint8_t *data, size_t len) {

  usb_scsi_transport_handler_t *trp = transport->handler;
  osalMutexLock(&trp->txmtx);
  msg_t status = usbTransmit(trp->usbp, trp->ep, data, len);
  osalMutexUnlock(&trp->txmtx);
  if (MSG_OK == status)
    return len;
  else
    return 0;
}

/**
 * @brief SCSI transport transmit async function.
 * 
 * @param[in] transport pointer to the @p SCSITransport object
 * @param[in] data      payload
 * @param[in] len       number of bytes to be transmitted
 * 
 * @return              Number of bytes put into buffer.
 * @notapi
 */
static uint32_t scsi_transport_transmit_async(const SCSITransport *transport,
                                        const uint8_t *data, size_t len) {

  usb_scsi_transport_handler_t *trp = transport->handler;
  // wait for previous tx to finish, if unfinished
  osalMutexLock(&trp->txmtx);
  memcpy(trp->txbuf, data, len);
  trp->txlen = len;
  osalMutexUnlock(&trp->txmtx);
  osalSysLock();
  osalThreadResumeS(&trp->txworker, MSG_OK);
  osalSysUnlock();
  // if (trp->txlen > 0) {
  //   usbTransmit(trp->usbp, trp->ep, trp->txbuf,
  //             trp->txlen);
  // }

  return len;
}

/**
 * @brief   SCSI transport receive function.
 *
 * @param[in] transport pointer to the @p SCSITransport object
 * @param[in] data      payload
 * @param[in] len       number bytes to be received
 *
 * @return              Number of successfully received bytes.
 * @notapi
 */
static uint32_t scsi_transport_receive(const SCSITransport *transport,
                                       uint8_t *data, size_t len) {

  usb_scsi_transport_handler_t *trp = transport->handler;
  osalMutexLock(&trp->txmtx);
  msg_t status = usbReceive(trp->usbp, trp->ep, data, len);
  osalMutexUnlock(&trp->txmtx);
  if (MSG_RESET != status)
    return status;
  else
    return 0;
}

/**
 * @brief   Fills and sends CSW message.
 *
 * @param[in] msdp      pointer to the @p USBMassStorageDriver object
 * @param[in] status    status returned by SCSI layer
 * @param[in] residue   number of residue bytes in case of failed transaction
 *
 * @notapi
 */
static void send_csw(USBMassStorageDriver *msdp, uint8_t status,
                     uint32_t residue) {

  msdp->csw.signature = MSD_CSW_SIGNATURE;
  msdp->csw.data_residue = residue;
  msdp->csw.tag = msdp->cbw.tag;
  msdp->csw.status = status;

  osalMutexLock(&msdp->usb_scsi_transport_handler.txmtx);
  usbTransmit(msdp->usbp, USB_MSD_DATA_EP, (uint8_t *)&msdp->csw,
      sizeof(msd_csw_t));
  osalMutexUnlock(&msdp->usb_scsi_transport_handler.txmtx);
}

/**
 * @brief   Mass storage worker thread.
 *
 * @param[in] arg     pointer to the @p USBMassStorageDriver object
 *
 * @notapi
 */
static THD_FUNCTION(usb_msd_worker, arg) {
  USBMassStorageDriver *msdp = arg;
  chRegSetThreadName("usb_msd_worker");

  while(! chThdShouldTerminateX()) {
    osalMutexLock(&msdp->usb_scsi_transport_handler.txmtx);
    const msg_t status = usbReceive(msdp->usbp, USB_MSD_DATA_EP,
                                   (uint8_t *)&msdp->cbw, sizeof(msd_cbw_t));
    osalMutexUnlock(&msdp->usb_scsi_transport_handler.txmtx);
    if (MSG_RESET == status) {
      osalThreadSleepMilliseconds(50);
    }
    else if (cbw_valid(&msdp->cbw, status) && cbw_meaningful(&msdp->cbw)) {
      if (msdp->scsi_transport.block_filesystem_access) {
        msdp->scsi_transport.block_filesystem_access();
      }
      if (SCSI_SUCCESS == scsiExecCmd(&msdp->scsi_target, msdp->cbw.cmd_data)) {
        send_csw(msdp, CSW_STATUS_PASSED, 0);
      }
      else {
        send_csw(msdp, CSW_STATUS_FAILED, scsiResidue(&msdp->scsi_target));
      }
      if (msdp->scsi_transport.free_filesystem_access) {
        msdp->scsi_transport.free_filesystem_access();
      }
    }
    else {
      ; /* do NOT send CSW here. Incorrect CBW must be silently ignored */
    }
  }

  chThdExit(MSG_OK);
}

/**
 * @brief USB Transmit worker thread.
 * 
 * @param[in] arg     pointer to the @p USBMassStorageDriver object
 * 
 * @notapi
 */
static THD_FUNCTION(usb_msd_tx_worker, arg) {
  USBMassStorageDriver *msdp = arg;
  chRegSetThreadName("usb_msd_tx_worker");
  
  while(! chThdShouldTerminateX()) {
    osalSysLock();
    osalThreadSuspendS(&msdp->usb_scsi_transport_handler.txworker);
    osalSysUnlock();
    osalMutexLock(&msdp->usb_scsi_transport_handler.txmtx);
    if (msdp->usb_scsi_transport_handler.txlen > 0) {
      usbTransmit(msdp->usbp, USB_MSD_DATA_EP, msdp->usb_scsi_transport_handler.txbuf,
                  msdp->usb_scsi_transport_handler.txlen);
      msdp->usb_scsi_transport_handler.txlen = 0;
    }
    osalMutexUnlock(&msdp->usb_scsi_transport_handler.txmtx);
  }
  
  chThdExit(MSG_OK);
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Mass storage specific request hook for USB.
 *
 * @param[in] usbp     pointer to the @p USBDriver object
 *
 * @notapi
 */
bool msd_request_hook(USBDriver *usbp) {
  /* check that the request is for interface 0.*/
  if (MSD_SETUP_INDEX(usbp->setup) != 0)
    return false;

  if (usbp->setup[0] == (USB_RTYPE_TYPE_CLASS | USB_RTYPE_RECIPIENT_INTERFACE | USB_RTYPE_DIR_HOST2DEV)
    && usbp->setup[1] == MSD_REQ_RESET) {
    /* Bulk-Only Mass Storage Reset (class-specific request)
    This request is used to reset the mass storage device and its associated interface.
    This class-specific request shall ready the device for the next CBW from the host. */
    /* Do any special reset code here. */
    /* The device shall NAK the status stage of the device request until
     * the Bulk-Only Mass Storage Reset is complete.
     * NAK EP1 in and out */
    // usbp->otg->ie[1].DIEPCTL = DIEPCTL_SNAK;
    // usbp->otg->oe[1].DOEPCTL = DOEPCTL_SNAK;
    /* response to this request using EP0 */
    usbSetupTransfer(usbp, 0, 0, NULL);
    return true;
  } else if (usbp->setup[0] == (USB_RTYPE_TYPE_CLASS | USB_RTYPE_RECIPIENT_INTERFACE | USB_RTYPE_DIR_DEV2HOST)
    && usbp->setup[1] == MSD_REQ_GET_MAX_LUN) {
    /* Return the maximum supported LUN. */
    static uint8_t zero = 0;
    usbSetupTransfer(usbp, &zero, 1, NULL);
    return true;
    /* OR */
    /* Return false to stall to indicate that we don't support LUN */
    // return false;
  }

  return false;
}

/**
 * @brief   Initializes the standard part of a @p USBMassStorageDriver structure.
 *
 * @param[out] msdp     pointer to the @p USBMassStorageDriver object
 *
 * @init
 */
void msdObjectInit(USBMassStorageDriver *msdp) {

  memset(msdp, 0x55, sizeof(USBMassStorageDriver));
  msdp->state = USB_MSD_STOP;
  msdp->usbp = NULL;
  msdp->worker = NULL;
  msdp->usb_scsi_transport_handler.txworker = NULL;

  scsiObjectInit(&msdp->scsi_target);
}

/**
 * @brief   Stops the USB mass storage driver.
 *
 * @param[in] msdp      pointer to the @p USBMassStorageDriver object
 *
 * @api
 */
void msdStop(USBMassStorageDriver *msdp) {

  osalDbgCheck(msdp != NULL);
  osalDbgAssert((msdp->state == USB_MSD_READY), "invalid state");

  chThdTerminate(msdp->worker);
  chThdWait(msdp->worker);

  chThdTerminate(msdp->usb_scsi_transport_handler.txworker);
  osalSysLock();
  thread_t* tp = msdp->usb_scsi_transport_handler.txworker;
  // resume thread so it can terminate
  osalThreadResumeS(&tp, MSG_OK);
  osalSysUnlock();
  chThdWait(msdp->usb_scsi_transport_handler.txworker);

  scsiStop(&msdp->scsi_target);

  msdp->worker = NULL;
  msdp->usb_scsi_transport_handler.txworker = NULL;
  msdp->state = USB_MSD_STOP;
  msdp->usbp = NULL;
}

/**
 * @brief   Configures and activates the USB mass storage driver.
 *
 * @param[in] msdp      pointer to the @p USBMassStorageDriver object
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] blkdev    pointer to the @p BaseBlockDevice object
 * @param[in] blkbuf    pointer to the working area buffer, must be allocated
 *                      by user, must be big enough to store 1 data block
 * @param[in] inquiry   pointer to the SCSI inquiry response structure,
 *                      set it to @p NULL to use default hardcoded value.
 *
 * @api
 */
void msdStart(USBMassStorageDriver *msdp, USBDriver *usbp,
              BaseBlockDevice *blkdev, uint8_t *blkbuf,
              uint8_t *txbuf,
              const scsi_inquiry_response_t *inquiry,
              const scsi_unit_serial_number_inquiry_response_t *serialInquiry,
              scsi_block_filesystem_access_t blockFilesystemAccess,
              scsi_free_filesystem_access_t freeFilesystemAccess) {

  osalDbgCheck((msdp != NULL) && (usbp != NULL)
              && (blkdev != NULL) && (blkbuf != NULL));
  osalDbgAssert((msdp->state == USB_MSD_STOP), "invalid state");

  msdp->usbp = usbp;

  msdp->usb_scsi_transport_handler.usbp = msdp->usbp;
  msdp->usb_scsi_transport_handler.ep   = USB_MSD_DATA_EP;
  msdp->usb_scsi_transport_handler.txbuf = txbuf;
  osalMutexObjectInit(&msdp->usb_scsi_transport_handler.txmtx);
  msdp->usb_scsi_transport_handler.txworker = chThdCreateStatic(msdp->usb_scsi_transport_handler.waMSDTxWorker, sizeof(msdp->usb_scsi_transport_handler.waMSDTxWorker),
                                      MSD_THD_PRIO, usb_msd_tx_worker, msdp);

  msdp->scsi_transport.handler  = &msdp->usb_scsi_transport_handler;
  msdp->scsi_transport.transmit = scsi_transport_transmit;
  msdp->scsi_transport.transmit_async = scsi_transport_transmit_async;
  msdp->scsi_transport.receive  = scsi_transport_receive;

  msdp->scsi_transport.block_filesystem_access = blockFilesystemAccess;
  msdp->scsi_transport.free_filesystem_access = freeFilesystemAccess;

  if (NULL == inquiry) {
    msdp->scsi_config.inquiry_response = &default_scsi_inquiry_response;
  }
  else {
    msdp->scsi_config.inquiry_response = inquiry;
  }
  if (NULL == serialInquiry) {
    msdp->scsi_config.unit_serial_number_inquiry_response = &default_scsi_unit_serial_number_inquiry_response;
  }
  else {
    msdp->scsi_config.unit_serial_number_inquiry_response = serialInquiry;
  }
  msdp->scsi_config.blkbuf = blkbuf;
  msdp->scsi_config.blkdev = blkdev;
  msdp->scsi_config.transport = &msdp->scsi_transport;

  scsiStart(&msdp->scsi_target, &msdp->scsi_config);

  msdp->state = USB_MSD_READY;
  msdp->worker = chThdCreateStatic(msdp->waMSDWorker, sizeof(msdp->waMSDWorker),
                                   MSD_THD_PRIO-1, usb_msd_worker, msdp);
}

#endif /* HAL_USE_USB_MSD */

/** @} */
