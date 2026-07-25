/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    vfs/src/vfsbuffers.c
 * @brief   VFS shared path buffers code.
 *
 * @addtogroup VFS_BUFFERS
 * @{
 */

#include "vfs.h"

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

/**
 * @brief   VFS static data.
 */
static struct {
  /**
   * @brief   Guarded pool of path buffers.
   */
  guarded_memory_pool_t             buffers_pool;
  /**
   * @brief   Shared path buffers.
   */
  vfs_shared_buffer_t               buffers[VFS_CFG_PATHBUFS_NUM];
} vfs_buffers_static;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   VFS path buffers initialization.
 *
 * @init
 */
void __vfs_buffers_init(void) {

  chGuardedPoolObjectInit(&vfs_buffers_static.buffers_pool,
                          sizeof (vfs_shared_buffer_t));
  chGuardedPoolLoadArray(&vfs_buffers_static.buffers_pool,
                         &vfs_buffers_static.buffers[0],
                         VFS_CFG_PATHBUFS_NUM);
}

/**
 * @brief   Claims a path buffer from the fixed pool, waiting if not available.
 *
 * @return                      Pointer to the taken buffer.
 */
vfs_shared_buffer_t *vfs_buffer_take_wait(void) {

  return (vfs_shared_buffer_t *)chGuardedPoolAllocTimeout(&vfs_buffers_static.buffers_pool,
                                                          TIME_INFINITE);
}

/**
 * @brief   Claims a path buffer from the fixed pool without waiting.
 *
 * @return                      Pointer to the taken buffer.
 * @retval NULL                 If the buffer is not available.
 */
vfs_shared_buffer_t *vfs_buffer_take_immediate(void) {

  return (vfs_shared_buffer_t *)chGuardedPoolAllocTimeout(&vfs_buffers_static.buffers_pool,
                                                          TIME_IMMEDIATE);
}

/**
 * @brief   Releases a path buffer into the fixed pool.
 *
 * @param[in] buf               Buffer to be released.
 */
void vfs_buffer_release(vfs_shared_buffer_t *shbuf) {

  chGuardedPoolFree(&vfs_buffers_static.buffers_pool, (void *)shbuf);
}

/** @} */
