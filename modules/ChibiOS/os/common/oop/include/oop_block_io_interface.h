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
 * @file        oop_block_io_interface.h
 * @brief       Generated Block I/O Device Interface header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  OOP_BLOCK_IO_INTERFACE
 * @brief       Block I/O device interface.
 * @{
 */

#ifndef OOP_BLOCK_IO_INTERFACE_H
#define OOP_BLOCK_IO_INTERFACE_H

#include "oop_base_interface.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#if (defined(OOP_USE_LEGACY)) || defined (__DOXYGEN__)
/**
 * @name    Legacy interface method names
 * @{
 */
#define blkIsInserted                       bioIsInserted
#define blkIsWriteProtected                 bioIsWriteProtected
#define blkConnect                          bioConnect
#define blkDisconnect                       bioDisconnect
#define blkRead                             bioRead
#define blkWrite                            bioWrite
#define blkSync                             bioSync
#define blkGetInfo                          bioGetInfo
/** @} */
#endif /* defined(OOP_USE_LEGACY) */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief       Type of a block I/O device information.
 */
typedef struct block_io_info block_io_info_t;

/**
 * @brief       Structure representing a block I/O device information.
 */
struct block_io_info {
  /**
   * @brief       Block size in bytes.
   */
  uint32_t                  blk_size;
  /**
   * @brief       Total number of blocks.
   */
  uint32_t                  blk_num;
};

/**
 * @interface   block_io_i
 * @extends     base_interface_i
 *
 * @brief       Block I/O interface.
 * @details     This module define an interface for generic devices performing
 *              block I/O.
 *              This interface is meant to be implemented in classes requiring
 *              block I/O capability.
 * @note        This interface is meant to be compatible with legacy HAL @p
 *              BaseBlockDevice interface.
 *
 * @name        Interface @p block_io_i structures
 * @{
 */

/**
 * @brief       Type of a block I/O interface.
 */
typedef struct block_io block_io_i;

/**
 * @brief       Interface @p block_io_i virtual methods table.
 */
struct block_io_vmt {
  /* Memory offset between this interface structure and begin of
     the implementing class structure.*/
  size_t instance_offset;
  /* From base_interface_i.*/
  /* From block_io_i.*/
  bool (*is_inserted)(void *ip);
  bool (*is_protected)(void *ip);
  bool (*connect)(void *ip);
  bool (*disconnect)(void *ip);
  bool (*read)(void *ip, uint32_t startblk, uint8_t buf, uint32_t n);
  bool (*write)(void *ip, uint32_t startblk, const uint8_t buf, uint32_t n);
  bool (*sync)(void *ip);
  bool (*get_info)(void *ip, block_io_info_t *bdip);
};

/**
 * @brief       Structure representing a block I/O interface.
 */
struct block_io {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct block_io_vmt *vmt;
};
/** @} */

#if (defined(OOP_USE_LEGACY)) || defined (__DOXYGEN__)
/**
 * @brief       For compatibility with legacy @p BlockDeviceInfo.
 */
typedef block_io_info_i BlockDeviceInfo;

/**
 * @brief       For compatibility with legacy @p BaseSequentialStream.
 */
typedef block_io_i BaseBlockDevice;
#endif /* defined(OOP_USE_LEGACY) */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Virtual methods of block_io_i
 * @{
 */
/**
 * @brief       Returns the media insertion status.
 * @note        On some implementations this function can only be called if the
 *              device is not transferring data.
 *
 * @param[in,out] ip            Pointer to a @p block_io_i instance.
 * @return                      The driver state.
 * @retval false                If media is not inserted.
 * @retval true                 If media is inserted.
 */
CC_FORCE_INLINE
static inline bool bioIsInserted(void *ip) {
  block_io_i *self = (block_io_i *)ip;

  return self->vmt->is_inserted(ip);
}

/**
 * @brief       Returns the media write protection status.
 *
 * @param[in,out] ip            Pointer to a @p block_io_i instance.
 * @return                      The media state.
 * @retval false                If media is writable.
 * @retval true                 If media is not writable.
 */
CC_FORCE_INLINE
static inline bool bioIsWriteProtected(void *ip) {
  block_io_i *self = (block_io_i *)ip;

  return self->vmt->is_protected(ip);
}

/**
 * @brief       Performs the initialization procedure on the block device.
 * @details     This operation must be performed before I/O operations can be
 *              attempted on the block device and after insertion has been
 *              confirmed using @p bioIsInserted().
 *
 * @param[in,out] ip            Pointer to a @p block_io_i instance.
 * @return                      The operation status.
 * @retval false                If the operation succeeded.
 * @retval true                 If the operation failed.
 */
CC_FORCE_INLINE
static inline bool bioConnect(void *ip) {
  block_io_i *self = (block_io_i *)ip;

  return self->vmt->connect(ip);
}

/**
 * @brief       Terminates operations on the block device.
 * @details     This operation safely terminates operations on the block
 *              device. Should be performed before the media is extracted or
 *              powered down.
 *
 * @param[in,out] ip            Pointer to a @p block_io_i instance.
 * @return                      The operation status.
 * @retval false                If the operation succeeded.
 * @retval true                 If the operation failed.
 */
CC_FORCE_INLINE
static inline bool bioDisconnect(void *ip) {
  block_io_i *self = (block_io_i *)ip;

  return self->vmt->disconnect(ip);
}

/**
 * @brief       Reads one or more blocks.
 *
 * @param[in,out] ip            Pointer to a @p block_io_i instance.
 * @return                      The operation status.
 * @retval false                If the operation succeeded.
 * @retval true                 If the operation failed.
 */
CC_FORCE_INLINE
static inline bool bioRead(void *ip, uint32_t startblk, uint8_t buf,
                           uint32_t n) {
  block_io_i *self = (block_io_i *)ip;

  return self->vmt->read(ip, startblk, buf, n);
}

/**
 * @brief       Writes one or more blocks.
 *
 * @param[in,out] ip            Pointer to a @p block_io_i instance.
 * @return                      The operation status.
 * @retval false                If the operation succeeded.
 * @retval true                 If the operation failed.
 */
CC_FORCE_INLINE
static inline bool bioWrite(void *ip, uint32_t startblk, const uint8_t buf,
                            uint32_t n) {
  block_io_i *self = (block_io_i *)ip;

  return self->vmt->write(ip, startblk, buf, n);
}

/**
 * @brief       Ensures write synchronization.
 *
 * @param[in,out] ip            Pointer to a @p block_io_i instance.
 * @return                      The operation status.
 * @retval false                If the operation succeeded.
 * @retval true                 If the operation failed.
 */
CC_FORCE_INLINE
static inline bool BioSync(void *ip) {
  block_io_i *self = (block_io_i *)ip;

  return self->vmt->sync(ip);
}

/**
 * @brief       Returns a media information structure.
 *
 * @param[in,out] ip            Pointer to a @p block_io_i instance.
 * @return                      The operation status.
 * @retval false                If the operation succeeded.
 * @retval true                 If the operation failed.
 */
CC_FORCE_INLINE
static inline bool bioGetInfo(void *ip, block_io_info_t *bdip) {
  block_io_i *self = (block_io_i *)ip;

  return self->vmt->get_info(ip, bdip);
}
/** @} */

#endif /* OOP_BLOCK_IO_INTERFACE_H */

/** @} */
