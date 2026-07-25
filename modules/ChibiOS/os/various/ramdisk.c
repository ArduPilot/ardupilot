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
 * @file    ramdisk.c
 * @brief   Virtual block devise driver source.
 *
 * @addtogroup ramdisk
 * @{
 */

#include "hal.h"

#include "ramdisk.h"

#include <string.h>

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*
 * Interface implementation.
 */
static bool overflow(const RamDisk *rd, uint32_t startblk, uint32_t n) {
  return (startblk + n) > rd->blk_num;
}

static bool is_inserted(void *instance) {
  (void)instance;
  return true;
}

static bool is_protected(void *instance) {
  RamDisk *rd = instance;
  if (BLK_READY == rd->state) {
    return rd->readonly;
  }
  else {
    return true;
  }
}

static bool connect(void *instance) {
  RamDisk *rd = instance;
  if (BLK_STOP == rd->state) {
    rd->state = BLK_READY;
  }
  return HAL_SUCCESS;
}

static bool disconnect(void *instance) {
  RamDisk *rd = instance;
  if (BLK_STOP != rd->state) {
    rd->state = BLK_STOP;
  }
  return HAL_SUCCESS;
}

static bool read(void *instance, uint32_t startblk,
                 uint8_t *buffer, uint32_t n) {

  RamDisk *rd = instance;

  if (overflow(rd, startblk, n)) {
    return HAL_FAILED;
  }
  else {
    const uint32_t bs = rd->blk_size;
    memcpy(buffer, &rd->storage[startblk * bs], n * bs);
    return HAL_SUCCESS;
  }
}

static bool write(void *instance, uint32_t startblk,
                const uint8_t *buffer, uint32_t n) {

  RamDisk *rd = instance;
  if (overflow(rd, startblk, n)) {
    return HAL_FAILED;
  }
  else {
    const uint32_t bs = rd->blk_size;
    memcpy(&rd->storage[startblk * bs], buffer, n * bs);
    return HAL_SUCCESS;
  }
}

static bool sync(void *instance) {

  RamDisk *rd = instance;
  if (BLK_READY != rd->state) {
    return HAL_FAILED;
  }
  else {
    return HAL_SUCCESS;
  }
}

static bool get_info(void *instance, BlockDeviceInfo *bdip) {

  RamDisk *rd = instance;
  if (BLK_READY != rd->state) {
    return HAL_FAILED;
  }
  else {
    bdip->blk_num = rd->blk_num;
    bdip->blk_size = rd->blk_size;
    return HAL_SUCCESS;
  }
}

/**
 *
 */
static const struct BaseBlockDeviceVMT vmt = {
    (size_t)0,
    is_inserted,
    is_protected,
    connect,
    disconnect,
    read,
    write,
    sync,
    get_info
};

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   RAM disk object initialization.
 *
 * @param[in] rdp   pointer to @p RamDisk object
 *
 * @init
 */
void ramdiskObjectInit(RamDisk *rdp) {

  rdp->vmt = &vmt;
  rdp->state = BLK_STOP;
}

/**
 * @brief   Starts RAM disk.
 *
 * @param[in] rdp       pointer to @p RamDisk object
 * @param[in] storage   pointer to array representing disk storage
 * @param[in] blksize   size of blocks in bytes
 * @param[in] blknum    total number of blocks in device
 * @param[in] readonly  read only flag
 *
 * @api
 */
void ramdiskStart(RamDisk *rdp, uint8_t *storage, uint32_t blksize,
                  uint32_t blknum, bool readonly) {

  osalDbgCheck(rdp != NULL);

  osalSysLock();
  osalDbgAssert((rdp->state == BLK_STOP) || (rdp->state == BLK_READY),
                "invalid state");
  rdp->blk_num  = blknum;
  rdp->blk_size = blksize;
  rdp->readonly = readonly;
  rdp->storage  = storage;
  rdp->state    = BLK_READY;
  osalSysUnlock();
}

/**
 * @brief   Stops RAM disk.
 *
 * @param[in] rdp       pointer to @p RamDisk object
 *
 * @api
 */
void ramdiskStop(RamDisk *rdp) {

  osalDbgCheck(rdp != NULL);

  osalSysLock();
  osalDbgAssert((rdp->state == BLK_STOP) || (rdp->state == BLK_READY),
                "invalid state");
  rdp->storage = NULL;
  rdp->state = BLK_STOP;
  osalSysUnlock();
}

/** @} */