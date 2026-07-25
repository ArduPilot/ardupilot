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
 * @file    vfs/include/vfs.h
 * @brief   VFS header file.
 * @details Main header of the ChibiOS Virtual File System.
 *
 * @addtogroup VFS
 * @{
 */

#ifndef VFS_H
#define VFS_H

/**
 * @brief   ChibiOS/VFS identification macro.
 */
#define __CHIBIOS_VFS__

/**
 * @brief   Stable release flag.
 */
#define CH_VFS_STABLE           0

/**
 * @name    ChibiOS/VFS version identification
 * @{
 */
/**
 * @brief   Kernel version string.
 */
#define CH_VFS_VERSION          "1.0.0"

/**
 * @brief   Kernel version major number.
 */
#define CH_VFS_MAJOR            1

/**
 * @brief   Kernel version minor number.
 */
#define CH_VFS_MINOR            0

/**
 * @brief   Kernel version patch number.
 */
#define CH_VFS_PATCH            0
/** @} */

#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/stat.h>

/* Dependencies.*/
#include "ch.h"
#include "errcodes.h"
#include "oop_base_object.h"
#include "oop_referenced_object.h"
#include "oop_sequential_stream.h"

/* License.*/
#include "chlicense.h"

/* Configuration headers, checks and licensing restrictions.*/
#include "vfsconf.h"
#include "vfschecks.h"

/* Base VFS headers.*/
#include "vfspaths.h"
#include "vfsparser.h"
#include "vfsbuffers.h"
#include "vfsnodes.h"
#include "vfsdrivers.h"

/* File System drivers.*/
#if VFS_CFG_ENABLE_DRV_OVERLAY == TRUE
#include "drvoverlay.h"
#endif

#if VFS_CFG_ENABLE_DRV_STREAMS == TRUE
#include "drvstreams.h"
#endif

#if VFS_CFG_ENABLE_DRV_CHFS == TRUE
#include "drvchfs.h"
#endif

#if VFS_CFG_ENABLE_DRV_FATFS == TRUE
#include "drvfatfs.h"
#endif

#if VFS_CFG_ENABLE_DRV_LITTLEFS == TRUE
#include "drvlittlefs.h"
#endif

/* Only for testing, not a real driver.*/
#define VFS_CFG_ENABLE_DRV_TEMPLATE         FALSE
#define DRV_CFG_TEMPLATE_DIR_NODES_NUM      1
#define DRV_CFG_TEMPLATE_FILE_NODES_NUM     1
#if VFS_CFG_ENABLE_DRV_TEMPLATE == TRUE
#include "drvtmplfs.h"
#endif

/* Application code is supposed to export this symbol, it is expected to
   exists.*/
extern vfs_driver_c *vfs_root;

#ifdef __cplusplus
extern "C" {
#endif
  void vfsInit(void);
  msg_t vfsChangeCurrentDirectory(const char *path);
  msg_t vfsGetCurrentDirectory(char *buf, size_t size);
  msg_t vfsStat(const char *path, vfs_stat_t *sp);
  msg_t vfsOpen(const char *path, int flags, vfs_node_c **vnpp);
  msg_t vfsOpenDirectory(const char *name, vfs_directory_node_c **vdnpp);
  msg_t vfsOpenFile(const char *name, int flags, vfs_file_node_c **vfnpp);
  msg_t vfsUnlink(const char *path);
  msg_t vfsRename(const char *oldpath, const char *newpath);
  msg_t vfsMkdir(const char *path, vfs_mode_t mode);
  msg_t vfsRmdir(const char *path);
  msg_t vfsGetNodeStat(vfs_node_c *np, vfs_stat_t *sp);
  void vfsClose(vfs_node_c *vnp);
  msg_t vfsReadDirectoryFirst(vfs_directory_node_c *vdnp,
                              vfs_direntry_info_t *dip);
  msg_t vfsReadDirectoryNext(vfs_directory_node_c *vdnp,
                             vfs_direntry_info_t *dip);
  ssize_t vfsReadFile(vfs_file_node_c *vfnp, uint8_t *buf, size_t n);
  ssize_t vfsWriteFile(vfs_file_node_c *vfnp, const uint8_t *buf, size_t n);
  msg_t vfsSetFilePosition(vfs_file_node_c *vfnp,
                           vfs_offset_t offset,
                           vfs_seekmode_t whence);
  vfs_offset_t vfsGetFilePosition(vfs_file_node_c *vfnp);
  sequential_stream_i *vfsGetFileStream(vfs_file_node_c *vfnp);
#ifdef __cplusplus
}
#endif

#endif /* VFS_H */

/** @} */
