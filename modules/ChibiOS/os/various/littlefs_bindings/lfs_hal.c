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
 * Parts of this file are:
 *
 * Copyright (c) 2017, Arm Limited. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    lfs_hal.c
 * @brief   LittleFS-HAL bindings code.
 *
 * @addtogroup LITTLEFS_BINDINGS
 * @{
 */

#include "lfs_hal.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static flash_offset_t get_offset(const struct lfs_config *c,
                                 flash_sector_t base,
                                 lfs_block_t block,
                                 lfs_off_t off) {

  return (flash_offset_t)((((flash_offset_t)base + (flash_offset_t)block) *
                          (flash_offset_t)c->block_size) + (flash_offset_t)off);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

uint32_t lfs_crc(uint32_t crc, const void *buffer, size_t size) {
  static const uint32_t rtable[16] = {0x00000000, 0x1db71064, 0x3b6e20c8,
                                      0x26d930ac, 0x76dc4190, 0x6b6b51f4,
                                      0x4db26158, 0x5005713c, 0xedb88320,
                                      0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
                                      0x9b64c2b0, 0x86d3d2d4, 0xa00ae278,
                                      0xbdbdf21c, };
  const uint8_t *data = buffer;

  for (size_t i = 0; i < size; i++) {
    crc = (crc >> 4) ^ rtable[(crc ^ (data[i] >> 0)) & 0xf];
    crc = (crc >> 4) ^ rtable[(crc ^ (data[i] >> 4)) & 0xf];
  }

  return crc;
}

void *lfs_malloc(size_t size) {

  return chHeapAlloc(NULL, size);
}

void lfs_free(void *p) {

  if (p != NULL) {
    chHeapFree(p);
  }
}

int __lfs_read(const struct lfs_config *c, lfs_block_t block,
               lfs_off_t off, void *buffer, lfs_size_t size) {
  const hal_lfs_binding_t *bnp = (const hal_lfs_binding_t *)c->context;
  flash_error_t err;

  err = flashRead(bnp->flp,
                  get_offset(c, bnp->base, block, off),
                  (size_t)size,
                  (uint8_t *)buffer);
  if (err != FLASH_NO_ERROR) {
    return LFS_ERR_IO;
  }

  return 0;
}

int __lfs_prog(const struct lfs_config *c, lfs_block_t block,
               lfs_off_t off, const void *buffer, lfs_size_t size) {
  const hal_lfs_binding_t *bnp = (const hal_lfs_binding_t *)c->context;
  flash_error_t err;

  err = flashProgram(bnp->flp,
                     get_offset(c, bnp->base, block, off),
                     (size_t)size,
                     (uint8_t *)buffer);
  if (err != FLASH_NO_ERROR) {
    return LFS_ERR_IO;
  }

  return 0;
}

int __lfs_erase(const struct lfs_config *c, lfs_block_t block) {
  const hal_lfs_binding_t *bnp = (const hal_lfs_binding_t *)c->context;
  flash_error_t err;

  err = flashStartEraseSector(bnp->flp, bnp->base + (flash_sector_t)block);
  if (err != FLASH_NO_ERROR) {
    return LFS_ERR_IO;
  }

  err = flashWaitErase(bnp->flp);
  if (err != FLASH_NO_ERROR) {
    return LFS_ERR_IO;
  }

  return 0;
}

int __lfs_sync(const struct lfs_config *c) {
  const hal_lfs_binding_t *bnp = (const hal_lfs_binding_t *)c->context;

  (void)bnp;

  return 0;
}

int __lfs_lock(const struct lfs_config *c) {
  const hal_lfs_binding_t *bnp = (const hal_lfs_binding_t *)c->context;

  flashAcquireExclusive(bnp->flp);

  return 0;
}

int __lfs_unlock(const struct lfs_config *c) {
  const hal_lfs_binding_t *bnp = (const hal_lfs_binding_t *)c->context;

  flashReleaseExclusive(bnp->flp);

  return 0;
}

/** @} */
