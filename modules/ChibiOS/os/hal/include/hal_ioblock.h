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
 * @file    hal_ioblock.h
 * @brief   I/O block devices access.
 * @details This header defines an abstract interface useful to access generic
 *          I/O block devices in a standardized way.
 *
 * @addtogroup IO_BLOCK
 * @details This module defines an abstract interface for accessing generic
 *          block devices.<br>
 *          Note that no code is present, just abstract interfaces-like
 *          structures, you should look at the system as to a set of
 *          abstract C++ classes (even if written in C). This system
 *          has then advantage to make the access to block devices
 *          independent from the implementation logic.
 * @{
 */

#ifndef HAL_IOBLOCK_H
#define HAL_IOBLOCK_H

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  BLK_UNINIT = 0,                   /**< Not initialized.                   */
  BLK_STOP = 1,                     /**< Stopped.                           */
  BLK_ACTIVE = 2,                   /**< Interface active.                  */
  BLK_CONNECTING = 3,               /**< Connection in progress.            */
  BLK_DISCONNECTING = 4,            /**< Disconnection in progress.         */
  BLK_READY = 5,                    /**< Device ready.                      */
  BLK_READING = 6,                  /**< Read operation in progress.        */
  BLK_WRITING = 7,                  /**< Write operation in progress.       */
  BLK_SYNCING = 8                   /**< Sync. operation in progress.       */
} blkstate_t;

/**
 * @brief   Block device info.
 */
typedef struct {
  uint32_t      blk_size;           /**< @brief Block size in bytes.        */
  uint32_t      blk_num;            /**< @brief Total number of blocks.     */
} BlockDeviceInfo;

/**
 * @brief   @p BaseBlockDevice specific methods.
 */
#define _base_block_device_methods                                          \
  _base_object_methods                                                      \
  /* Removable media detection.*/                                           \
  bool (*is_inserted)(void *instance);                                      \
  /* Removable write protection detection.*/                                \
  bool (*is_protected)(void *instance);                                     \
  /* Connection to the block device.*/                                      \
  bool (*connect)(void *instance);                                          \
  /* Disconnection from the block device.*/                                 \
  bool (*disconnect)(void *instance);                                       \
  /* Reads one or more blocks.*/                                            \
  bool (*read)(void *instance, uint32_t startblk,                           \
                 uint8_t *buffer, uint32_t n);                              \
  /* Writes one or more blocks.*/                                           \
  bool (*write)(void *instance, uint32_t startblk,                          \
                  const uint8_t *buffer, uint32_t n);                       \
  /* Write operations synchronization.*/                                    \
  bool (*sync)(void *instance);                                             \
  /* Obtains info about the media.*/                                        \
  bool (*get_info)(void *instance, BlockDeviceInfo *bdip);

/**
 * @brief   @p BaseBlockDevice specific data.
 */
#define _base_block_device_data                                             \
  _base_object_data                                                         \
  /* Driver state.*/                                                        \
  blkstate_t            state;

/**
 * @brief   @p BaseBlockDevice virtual methods table.
 */
struct BaseBlockDeviceVMT {
  _base_block_device_methods
};

/**
 * @extends BaseObject
 *
 * @brief   Base block device class.
 * @details This class represents a generic, block-accessible, device.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseBlockDeviceVMT *vmt;
  _base_block_device_data
} BaseBlockDevice;

/**
 * @name    Macro Functions (BaseBlockDevice)
 * @{
 */
/**
 * @brief   Returns the driver state.
 * @note    Can be called in ISR context.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 *
 * @return              The driver state.
 *
 * @special
 */
#define blkGetDriverState(ip) ((ip)->state)

/**
 * @brief   Determines if the device is transferring data.
 * @note    Can be called in ISR context.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 *
 * @return              The driver state.
 * @retval false        the device is not transferring data.
 * @retval true         the device not transferring data.
 *
 * @special
 */
#define blkIsTransferring(ip) ((((ip)->state) == BLK_CONNECTING) ||         \
                               (((ip)->state) == BLK_DISCONNECTING) ||      \
                               (((ip)->state) == BLK_READING) ||            \
                               (((ip)->state) == BLK_WRITING))

/**
 * @brief   Returns the media insertion status.
 * @note    On some implementations this function can only be called if the
 *          device is not transferring data.
 *          The function @p blkIsTransferring() should be used before calling
 *          this function.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 *
 * @return              The media state.
 * @retval false        media not inserted.
 * @retval true         media inserted.
 *
 * @api
 */
#define blkIsInserted(ip) ((ip)->vmt->is_inserted(ip))

/**
 * @brief   Returns the media write protection status.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 *
 * @return              The media state.
 * @retval false        writable media.
 * @retval true         non writable media.
 *
 * @api
 */
#define blkIsWriteProtected(ip) ((ip)->vmt->is_protected(ip))

/**
 * @brief   Performs the initialization procedure on the block device.
 * @details This function should be performed before I/O operations can be
 *          attempted on the block device and after insertion has been
 *          confirmed using @p blkIsInserted().
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
#define blkConnect(ip) ((ip)->vmt->connect(ip))

/**
 * @brief   Terminates operations on the block device.
 * @details This operation safely terminates operations on the block device.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
#define blkDisconnect(ip) ((ip)->vmt->disconnect(ip))

/**
 * @brief   Reads one or more blocks.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
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
#define blkRead(ip, startblk, buf, n)                                       \
  ((ip)->vmt->read(ip, startblk, buf, n))

/**
 * @brief   Writes one or more blocks.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 * @param[in] startblk  first block to write
 * @param[out] buf      pointer to the write buffer
 * @param[in] n         number of blocks to write
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
#define blkWrite(ip, startblk, buf, n)                                      \
  ((ip)->vmt->write(ip, startblk, buf, n))

/**
 * @brief   Ensures write synchronization.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
#define blkSync(ip) ((ip)->vmt->sync(ip))

/**
 * @brief   Returns a media information structure.
 *
 * @param[in] ip        pointer to a @p BaseBlockDevice or derived class
 * @param[out] bdip     pointer to a @p BlockDeviceInfo structure
 *
 * @return              The operation status.
 * @retval HAL_SUCCESS  operation succeeded.
 * @retval HAL_FAILED   operation failed.
 *
 * @api
 */
#define blkGetInfo(ip, bdip) ((ip)->vmt->get_info(ip, bdip))

/** @} */

#endif /* HAL_IOBLOCK_H */

/** @} */
