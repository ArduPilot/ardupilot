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
 * @file        drvfatfs.h
 * @brief       Generated VFS FatFS Driver header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVFATFS
 * @{
 */

#ifndef DRVFATFS_H
#define DRVFATFS_H

#if (VFS_CFG_ENABLE_DRV_FATFS == TRUE) || defined(__DOXYGEN__)

#include "oop_sequential_stream.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief       Maximum number of FatFS file systems mounted.
 */
#if !defined(DRV_CFG_FATFS_FS_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_FATFS_FS_NUM                1
#endif

/**
 * @brief       Number of directory nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_FATFS_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_FATFS_DIR_NODES_NUM         1
#endif

/**
 * @brief       Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_FATFS_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_FATFS_FILE_NODES_NUM        1
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on DRV_CFG_FATFS_FS_NUM configuration.*/
#if DRV_CFG_FATFS_FS_NUM < 1
#error "invalid DRV_CFG_FATFS_FS_NUM value"
#endif

/* Checks on DRV_CFG_FATFS_DIR_NODES_NUM configuration.*/
#if DRV_CFG_FATFS_DIR_NODES_NUM < 1
#error "invalid DRV_CFG_FATFS_DIR_NODES_NUM value"
#endif

/* Checks on DRV_CFG_FATFS_FILE_NODES_NUM configuration.*/
#if DRV_CFG_FATFS_FILE_NODES_NUM < 1
#error "invalid DRV_CFG_FATFS_FILE_NODES_NUM value"
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @class       vfs_fatfs_driver_c
 * @extends     vfs_driver_c
 *
 *
 * @name        Class @p vfs_fatfs_driver_c structures
 * @{
 */

/**
 * @brief       Type of a VFS fatfs driver class.
 */
typedef struct vfs_fatfs_driver vfs_fatfs_driver_c;

/**
 * @brief       Class @p vfs_fatfs_driver_c virtual methods table.
 */
struct vfs_fatfs_driver_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
  /* From vfs_driver_c.*/
  msg_t (*setcwd)(void *ip, const char *path);
  msg_t (*getcwd)(void *ip, char *buf, size_t size);
  msg_t (*stat)(void *ip, const char *path, vfs_stat_t *sp);
  msg_t (*opendir)(void *ip, const char *path, vfs_directory_node_c **vdnpp);
  msg_t (*openfile)(void *ip, const char *path, int flags, vfs_file_node_c **vfnpp);
  msg_t (*unlink)(void *ip, const char *path);
  msg_t (*rename)(void *ip, const char *oldpath, const char *newpath);
  msg_t (*mkdir)(void *ip, const char *path, vfs_mode_t mode);
  msg_t (*rmdir)(void *ip, const char *path);
  /* From vfs_fatfs_driver_c.*/
};

/**
 * @brief       Structure representing a VFS fatfs driver class.
 */
struct vfs_fatfs_driver {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_fatfs_driver_vmt *vmt;
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern struct vfs_fatfs_driver_static_struct vfs_fatfs_driver_static;
extern struct vfs_fatfs_driver_static_nc_struct __nocache_vfs_fatfs_driver_static;

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of vfs_fatfs_driver_c.*/
  void *__ffdrv_objinit_impl(void *ip, const void *vmt);
  void __ffdrv_dispose_impl(void *ip);
  msg_t __ffdrv_setcwd_impl(void *ip, const char *path);
  msg_t __ffdrv_getcwd_impl(void *ip, char *buf, size_t size);
  msg_t __ffdrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp);
  msg_t __ffdrv_opendir_impl(void *ip, const char *path,
                             vfs_directory_node_c **vdnpp);
  msg_t __ffdrv_openfile_impl(void *ip, const char *path, int flags,
                              vfs_file_node_c **vfnpp);
  msg_t __ffdrv_unlink_impl(void *ip, const char *path);
  msg_t __ffdrv_rename_impl(void *ip, const char *oldpath, const char *newpath);
  msg_t __ffdrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode);
  msg_t __ffdrv_rmdir_impl(void *ip, const char *path);
  /* Regular functions.*/
  void __drv_fatfs_init(void);
  msg_t ffdrvMount(const char *name, bool mountnow);
  msg_t ffdrvUnmount(const char *name);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of vfs_fatfs_driver_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_fatfs_driver_c.
 *
 * @param[out]    self          Pointer to a @p vfs_fatfs_driver_c instance to
 *                              be initialized.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_fatfs_driver_c *ffdrvObjectInit(vfs_fatfs_driver_c *self) {
  extern const struct vfs_fatfs_driver_vmt __vfs_fatfs_driver_vmt;

  return __ffdrv_objinit_impl(self, &__vfs_fatfs_driver_vmt);
}
/** @} */

#endif /* VFS_CFG_ENABLE_DRV_FATFS == TRUE */

#endif /* DRVFATFS_H */

/** @} */
