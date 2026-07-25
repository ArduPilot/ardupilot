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
 * @file        drvlittlefs.h
 * @brief       Generated VFS LittleFS Driver header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVLITTLEFS
 * @{
 */

#ifndef DRVLITTLEFS_H
#define DRVLITTLEFS_H

#if (VFS_CFG_ENABLE_DRV_LITTLEFS == TRUE) || defined(__DOXYGEN__)

#include "oop_sequential_stream.h"
#include "lfs.h"

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
#if !defined(DRV_CFG_LITTLEFS_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_LITTLEFS_DIR_NODES_NUM      2
#endif

/**
 * @brief       Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_LITTLEFS_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_LITTLEFS_FILE_NODES_NUM     2
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on DRV_CFG_LITTLEFS_DIR_NODES_NUM configuration.*/
#if DRV_CFG_LITTLEFS_DIR_NODES_NUM < 2
#error "invalid DRV_CFG_LITTLEFS_DIR_NODES_NUM value"
#endif

/* Checks on DRV_CFG_LITTLEFS_FILE_NODES_NUM configuration.*/
#if DRV_CFG_LITTLEFS_FILE_NODES_NUM < 1
#error "invalid DRV_CFG_LITTLEFS_FILE_NODES_NUM value"
#endif

/* Check on paths length settings.*/
#if VFS_CFG_PATHLEN_MAX < LFS_NAME_MAX
#error "VFS_CFG_PATHLEN_MAX is lower than LFS_NAME_MAX"
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @class       vfs_littlefs_driver_c
 * @extends     vfs_driver_c
 *
 *
 * @name        Class @p vfs_littlefs_driver_c structures
 * @{
 */

/**
 * @brief       Type of a VFS LittleFS driver class.
 */
typedef struct vfs_littlefs_driver vfs_littlefs_driver_c;

/**
 * @brief       Class @p vfs_littlefs_driver_c virtual methods table.
 */
struct vfs_littlefs_driver_vmt {
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
  /* From vfs_littlefs_driver_c.*/
};

/**
 * @brief       Structure representing a VFS LittleFS driver class.
 */
struct vfs_littlefs_driver {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_littlefs_driver_vmt *vmt;
  /**
   * @brief       LittleFS driver mounted flag.
   */
  bool                      mounted;
  /**
   * @brief       LittleFS driver structure.
   */
  lfs_t                     lfs;
  /**
   * @brief       Associated LittleFS configuration.
   */
  const struct lfs_config   *cfgp;
  /**
   * @brief       Current working directory node.
   */
  struct vfs_littlefs_dir_node *cwd;
  /**
   * @brief       Current working directory path.
   */
  char                      path_cwd[LFS_NAME_MAX + 1];
  /**
   * @brief       Path scratch pad.
   */
  char                      scratch[LFS_NAME_MAX + 1];
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern struct vfs_littlefs_driver_static_struct vfs_littlefs_driver_static;

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of vfs_littlefs_driver_c.*/
  void *__lfsdrv_objinit_impl(void *ip, const void *vmt,
                              const struct lfs_config *cfgp);
  void __lfsdrv_dispose_impl(void *ip);
  msg_t __lfsdrv_setcwd_impl(void *ip, const char *path);
  msg_t __lfsdrv_getcwd_impl(void *ip, char *buf, size_t size);
  msg_t __lfsdrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp);
  msg_t __lfsdrv_opendir_impl(void *ip, const char *path,
                              vfs_directory_node_c **vdnpp);
  msg_t __lfsdrv_openfile_impl(void *ip, const char *path, int flags,
                               vfs_file_node_c **vfnpp);
  msg_t __lfsdrv_unlink_impl(void *ip, const char *path);
  msg_t __lfsdrv_rename_impl(void *ip, const char *oldpath,
                             const char *newpath);
  msg_t __lfsdrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode);
  msg_t __lfsdrv_rmdir_impl(void *ip, const char *path);
  msg_t lfsdrvMount(void *ip);
  msg_t lfsdrvUnmount(void *ip);
  msg_t lfsdrvFormat(void *ip);
  /* Regular functions.*/
  void __drv_littlefs_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of vfs_littlefs_driver_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_littlefs_driver_c.
 *
 * @param[out]    self          Pointer to a @p vfs_littlefs_driver_c instance
 *                              to be initialized.
 * @param[in]     cfgp          Pointer to @p lfs_config configuration.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_littlefs_driver_c *lfsdrvObjectInit(vfs_littlefs_driver_c *self,
                                                      const struct lfs_config *cfgp) {
  extern const struct vfs_littlefs_driver_vmt __vfs_littlefs_driver_vmt;

  return __lfsdrv_objinit_impl(self, &__vfs_littlefs_driver_vmt, cfgp);
}
/** @} */

#endif /* VFS_CFG_ENABLE_DRV_LITTLEFS == TRUE */

#endif /* DRVLITTLEFS_H */

/** @} */
