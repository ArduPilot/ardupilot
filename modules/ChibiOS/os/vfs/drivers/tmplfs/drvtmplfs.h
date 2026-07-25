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
 * @file        drvtmplfs.h
 * @brief       Generated VFS Template Driver header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVTMPLFS
 * @{
 */

#ifndef DRVTMPLFS_H
#define DRVTMPLFS_H

#if (VFS_CFG_ENABLE_DRV_TEMPLATE == TRUE) || defined(__DOXYGEN__)

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
 * @brief       Number of directory nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_TMPL_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_TMPL_DIR_NODES_NUM          1
#endif

/**
 * @brief       Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_TMPL_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_TMPL_FILE_NODES_NUM         1
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on DRV_CFG_TMPL_DIR_NODES_NUM configuration.*/
#if DRV_CFG_TMPL_DIR_NODES_NUM < 1
#error "invalid DRV_CFG_TMPL_DIR_NODES_NUM value"
#endif

/* Checks on DRV_CFG_TMPL_FILE_NODES_NUM configuration.*/
#if DRV_CFG_TMPL_FILE_NODES_NUM < 1
#error "invalid DRV_CFG_TMPL_FILE_NODES_NUM value"
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @class       vfs_tmpl_driver_c
 * @extends     vfs_driver_c
 *
 *
 * @name        Class @p vfs_tmpl_driver_c structures
 * @{
 */

/**
 * @brief       Type of a VFS Tmpl driver class.
 */
typedef struct vfs_tmpl_driver vfs_tmpl_driver_c;

/**
 * @brief       Class @p vfs_tmpl_driver_c virtual methods table.
 */
struct vfs_tmpl_driver_vmt {
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
  /* From vfs_tmpl_driver_c.*/
};

/**
 * @brief       Structure representing a VFS Tmpl driver class.
 */
struct vfs_tmpl_driver {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_tmpl_driver_vmt *vmt;
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern struct vfs_tmpl_driver_static_struct vfs_tmpl_driver_static;

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of vfs_tmpl_driver_c.*/
  void *__tmpldrv_objinit_impl(void *ip, const void *vmt);
  void __tmpldrv_dispose_impl(void *ip);
  msg_t __tmpldrv_setcwd_impl(void *ip, const char *path);
  msg_t __tmpldrv_getcwd_impl(void *ip, char *buf, size_t size);
  msg_t __tmpldrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp);
  msg_t __tmpldrv_opendir_impl(void *ip, const char *path,
                               vfs_directory_node_c **vdnpp);
  msg_t __tmpldrv_openfile_impl(void *ip, const char *path, int flags,
                                vfs_file_node_c **vfnpp);
  msg_t __tmpldrv_unlink_impl(void *ip, const char *path);
  msg_t __tmpldrv_rename_impl(void *ip, const char *oldpath,
                              const char *newpath);
  msg_t __tmpldrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode);
  msg_t __tmpldrv_rmdir_impl(void *ip, const char *path);
  msg_t tmpldrvMount(void *ip, const void *config);
  msg_t tmpldrvUnmount(void *ip);
  /* Regular functions.*/
  void __drv_tmpl_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of vfs_tmpl_driver_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_tmpl_driver_c.
 *
 * @param[out]    self          Pointer to a @p vfs_tmpl_driver_c instance to
 *                              be initialized.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_tmpl_driver_c *tmpldrvObjectInit(vfs_tmpl_driver_c *self) {
  extern const struct vfs_tmpl_driver_vmt __vfs_tmpl_driver_vmt;

  return __tmpldrv_objinit_impl(self, &__vfs_tmpl_driver_vmt);
}
/** @} */

#endif /* VFS_CFG_ENABLE_DRV_TEMPLATE == TRUE */

#endif /* DRVTMPLFS_H */

/** @} */
