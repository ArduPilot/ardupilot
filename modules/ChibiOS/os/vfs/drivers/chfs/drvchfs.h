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
 * @file        drvchfs.h
 * @brief       Generated VFS ChibiFS Driver header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVCHFS
 * @{
 */

#ifndef DRVCHFS_H
#define DRVCHFS_H

#if (VFS_CFG_ENABLE_DRV_CHFS == TRUE) || defined(__DOXYGEN__)

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
#if !defined(DRV_CFG_CHFS_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_CHFS_DIR_NODES_NUM          2
#endif

/**
 * @brief       Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_CHFS_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_CHFS_FILE_NODES_NUM         2
#endif

/**
 * @brief       Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_CHFS_CACHE_BUFFERS_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_CHFS_CACHE_BUFFERS_NUM      2
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on DRV_CFG_CHFS_DIR_NODES_NUM configuration.*/
#if DRV_CFG_CHFS_DIR_NODES_NUM < 1
#error "invalid DRV_CFG_CHFS_DIR_NODES_NUM value"
#endif

/* Checks on DRV_CFG_CHFS_FILE_NODES_NUM configuration.*/
#if DRV_CFG_CHFS_FILE_NODES_NUM < 1
#error "invalid DRV_CFG_CHFS_FILE_NODES_NUM value"
#endif

/* Checks on DRV_CFG_CHFS_CACHE_BUFFERS_NUM configuration.*/
#if DRV_CFG_CHFS_CACHE_BUFFERS_NUM < 2
#error "invalid DRV_CFG_CHFS_CACHE_BUFFERS_NUM value"
#endif

#if CH_CFG_USE_OBJ_CACHES != TRUE
#error "VFS CHFS driver requires CH_CFG_USE_OBJ_CACHES"
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief       Type of a ChibiFS configuration structure.
 */
typedef struct chfs_config chfs_config_t;

/**
 * @brief       Structure representing a ChibiFS configuration.
 */
struct chfs_config {
  /**
   * @brief       Block device associated to this ChibiFS instance.
   */
  const void                *blkdev;
};

/**
 * @class       vfs_chfs_driver_c
 * @extends     vfs_driver_c
 *
 *
 * @name        Class @p vfs_chfs_driver_c structures
 * @{
 */

/**
 * @brief       Type of a VFS ChibiFS driver class.
 */
typedef struct vfs_chfs_driver vfs_chfs_driver_c;

/**
 * @brief       Class @p vfs_chfs_driver_c virtual methods table.
 */
struct vfs_chfs_driver_vmt {
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
  /* From vfs_chfs_driver_c.*/
};

/**
 * @brief       Structure representing a VFS ChibiFS driver class.
 */
struct vfs_chfs_driver {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_chfs_driver_vmt *vmt;
  /**
   * @brief       ChibiFS driver mounted flag.
   */
  bool                      mounted;
  /**
   * @brief       Associated ChibiFS configuration.
   */
  const struct chfs_config  *cfgp;
  /**
   * @brief       Current working directory node.
   */
  struct vfs_chfs_dir_node  *cwd;
  /**
   * @brief       Current working directory path.
   */
  char                      path_cwd[VFS_CFG_PATHLEN_MAX + 1];
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern struct vfs_chfs_driver_static_struct vfs_chfs_driver_static;

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of vfs_chfs_driver_c.*/
  void *__chfsdrv_objinit_impl(void *ip, const void *vmt,
                               const chfs_config_t *cfgp);
  void __chfsdrv_dispose_impl(void *ip);
  msg_t __chfsdrv_setcwd_impl(void *ip, const char *path);
  msg_t __chfsdrv_getcwd_impl(void *ip, char *buf, size_t size);
  msg_t __chfsdrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp);
  msg_t __chfsdrv_opendir_impl(void *ip, const char *path,
                               vfs_directory_node_c **vdnpp);
  msg_t __chfsdrv_openfile_impl(void *ip, const char *path, int flags,
                                vfs_file_node_c **vfnpp);
  msg_t __chfsdrv_unlink_impl(void *ip, const char *path);
  msg_t __chfsdrv_rename_impl(void *ip, const char *oldpath,
                              const char *newpath);
  msg_t __chfsdrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode);
  msg_t __chfsdrv_rmdir_impl(void *ip, const char *path);
  msg_t chfsdrvMount(void *ip);
  msg_t chfsdrvUnmount(void *ip);
  msg_t chfsdrvFormat(void *ip);
  /* Regular functions.*/
  void __drv_chfs_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of vfs_chfs_driver_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_chfs_driver_c.
 *
 * @param[out]    self          Pointer to a @p vfs_chfs_driver_c instance to
 *                              be initialized.
 * @param[in]     cfgp          Pointer to @p chfs_config_t configuration.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_chfs_driver_c *chfsdrvObjectInit(vfs_chfs_driver_c *self,
                                                   const chfs_config_t *cfgp) {
  extern const struct vfs_chfs_driver_vmt __vfs_chfs_driver_vmt;

  return __chfsdrv_objinit_impl(self, &__vfs_chfs_driver_vmt, cfgp);
}
/** @} */

#endif /* VFS_CFG_ENABLE_DRV_CHFS == TRUE */

#endif /* DRVCHFS_H */

/** @} */
