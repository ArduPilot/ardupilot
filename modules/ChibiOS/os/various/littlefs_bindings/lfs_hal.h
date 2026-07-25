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
 * @file    lfs_hal.h
 * @brief   LittleFS-HAL bindings header.
 *
 * @addtogroup LITTLEFS_BINDINGS
 * @{
 */

#ifndef LFS_WSPI_H
#define LFS_WSPI_H

#include "hal.h"
#include "lfs.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Binding object between LFS and HAL.
 */
typedef struct hal_lfs_binding {
  /**
   * @brief   Flash offset of an LFS area on flash.
   * @note    Must be a multiple of the block size.
   */
  flash_sector_t            base;
  /**
   * @brief   Pointer to a flash interface.
   */
  BaseFlash                 *flp;
} hal_lfs_binding_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  int __lfs_read(const struct lfs_config *c, lfs_block_t block,
                 lfs_off_t off, void *buffer, lfs_size_t size);
  int __lfs_prog(const struct lfs_config *c, lfs_block_t block,
                 lfs_off_t off, const void *buffer, lfs_size_t size);
  int __lfs_erase(const struct lfs_config *c, lfs_block_t block);
  int __lfs_sync(const struct lfs_config *c);
  int __lfs_lock(const struct lfs_config *c);
  int __lfs_unlock(const struct lfs_config *c);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* LFS_WSPI_H */

/** @} */
