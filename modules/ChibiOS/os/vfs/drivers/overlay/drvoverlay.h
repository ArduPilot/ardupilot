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
 * @file        drvoverlay.h
 * @brief       Generated VFS Template Driver header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVOVERLAY
 * @{
 */

#ifndef DRVOVERLAY_H
#define DRVOVERLAY_H

#if (VFS_CFG_ENABLE_DRV_OVERLAY == TRUE) || defined(__DOXYGEN__)

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
 * @brief       Maximum number of overlay directories.
 */
#if !defined(DRV_CFG_OVERLAY_DRV_MAX) || defined(__DOXYGEN__)
#define DRV_CFG_OVERLAY_DRV_MAX             1
#endif

/**
 * @brief       Number of directory nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_OVERLAY_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_OVERLAY_DIR_NODES_NUM       1
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on DRV_CFG_OVERLAY_DRV_MAX configuration.*/
#if DRV_CFG_OVERLAY_DRV_MAX < 1
#error "invalid DRV_CFG_OVERLAY_DRV_MAX value"
#endif

/* Checks on DRV_CFG_OVERLAY_DIR_NODES_NUM configuration.*/
#if DRV_CFG_OVERLAY_DIR_NODES_NUM < 1
#error "invalid DRV_CFG_OVERLAY_DIR_NODES_NUM value"
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @class       vfs_overlay_dir_node_c
 * @extends     vfs_directory_node_c
 *
 *
 * @name        Class @p vfs_overlay_dir_node_c structures
 * @{
 */

/**
 * @brief       Type of a VFS overlay directory node class.
 */
typedef struct vfs_overlay_dir_node vfs_overlay_dir_node_c;

/**
 * @brief       Class @p vfs_overlay_dir_node_c virtual methods table.
 */
struct vfs_overlay_dir_node_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
  /* From referenced_object_c.*/
  void * (*addref)(void *ip);
  object_references_t (*release)(void *ip);
  /* From vfs_node_c.*/
  msg_t (*stat)(void *ip, vfs_stat_t *sp);
  /* From vfs_directory_node_c.*/
  msg_t (*first)(void *ip, vfs_direntry_info_t *dip);
  msg_t (*next)(void *ip, vfs_direntry_info_t *dip);
  /* From vfs_overlay_dir_node_c.*/
};

/**
 * @brief       Structure representing a VFS overlay directory node class.
 */
struct vfs_overlay_dir_node {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_overlay_dir_node_vmt *vmt;
  /**
   * @brief       Number of references to the object.
   */
  object_references_t       references;
  /**
   * @brief       Driver handling this node.
   */
  vfs_driver_c              *driver;
  /**
   * @brief       Node mode information.
   */
  vfs_mode_t                mode;
  /**
   * @brief       Next directory entry to be read.
   */
  unsigned                  index;
  /**
   * @brief       File system to be overlaid.
   */
  vfs_directory_node_c      *overlaid_root;
};
/** @} */

/**
 * @class       vfs_overlay_driver_c
 * @extends     vfs_driver_c
 *
 *
 * @name        Class @p vfs_overlay_driver_c structures
 * @{
 */

/**
 * @brief       Type of a VFS overlay driver class.
 */
typedef struct vfs_overlay_driver vfs_overlay_driver_c;

/**
 * @brief       Class @p vfs_overlay_driver_c virtual methods table.
 */
struct vfs_overlay_driver_vmt {
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
  /* From vfs_overlay_driver_c.*/
};

/**
 * @brief       Structure representing a VFS overlay driver class.
 */
struct vfs_overlay_driver {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_overlay_driver_vmt *vmt;
  vfs_driver_c              *overlaid_drv;
  const char                *path_prefix;
  char                      *path_cwd;
  unsigned                  next_driver;
  const char                *names[DRV_CFG_OVERLAY_DRV_MAX];
  vfs_driver_c              *drivers[DRV_CFG_OVERLAY_DRV_MAX];
  char                      buf[VFS_CFG_PATHLEN_MAX + 1];
};
/** @} */

/**
 * @brief       Structure representing the global state of @p
 *              vfs_overlay_driver_c.
 */
struct vfs_overlay_driver_static_struct {
  /**
   * @brief       Pool of directory nodes.
   */
  memory_pool_t             dir_nodes_pool;
  /**
   * @brief       Static storage of directory nodes.
   */
  vfs_overlay_dir_node_c    dir_nodes[DRV_CFG_OVERLAY_DIR_NODES_NUM];
};

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of vfs_overlay_dir_node_c.*/
  void *__ovldir_objinit_impl(void *ip, const void *vmt,
                              vfs_overlay_driver_c *driver, vfs_mode_t mode);
  void __ovldir_dispose_impl(void *ip);
  msg_t __ovldir_stat_impl(void *ip, vfs_stat_t *sp);
  msg_t __ovldir_first_impl(void *ip, vfs_direntry_info_t *dip);
  msg_t __ovldir_next_impl(void *ip, vfs_direntry_info_t *dip);
  /* Methods of vfs_overlay_driver_c.*/
  void *__ovldrv_objinit_impl(void *ip, const void *vmt,
                              vfs_driver_c *overlaid_drv,
                              const char *path_prefix);
  void __ovldrv_dispose_impl(void *ip);
  msg_t __ovldrv_setcwd_impl(void *ip, const char *path);
  msg_t __ovldrv_getcwd_impl(void *ip, char *buf, size_t size);
  msg_t __ovldrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp);
  msg_t __ovldrv_opendir_impl(void *ip, const char *path,
                              vfs_directory_node_c **vdnpp);
  msg_t __ovldrv_openfile_impl(void *ip, const char *path, int flags,
                               vfs_file_node_c **vfnpp);
  msg_t __ovldrv_unlink_impl(void *ip, const char *path);
  msg_t __ovldrv_rename_impl(void *ip, const char *oldpath,
                             const char *newpath);
  msg_t __ovldrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode);
  msg_t __ovldrv_rmdir_impl(void *ip, const char *path);
  msg_t ovldrvRegisterDriver(void *ip, vfs_driver_c *vdp, const char *name);
  msg_t ovldrvUnregisterDriver(void *ip, const char *name);
  /* Regular functions.*/
  void __drv_overlay_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of vfs_overlay_dir_node_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_overlay_dir_node_c.
 *
 * @param[out]    self          Pointer to a @p vfs_overlay_dir_node_c instance
 *                              to be initialized.
 * @param[in]     driver        Pointer to the controlling driver.
 * @param[in]     mode          Node mode flags.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_overlay_dir_node_c *ovldirObjectInit(vfs_overlay_dir_node_c *self,
                                                       vfs_overlay_driver_c *driver,
                                                       vfs_mode_t mode) {
  extern const struct vfs_overlay_dir_node_vmt __vfs_overlay_dir_node_vmt;

  return __ovldir_objinit_impl(self, &__vfs_overlay_dir_node_vmt, driver, mode);
}
/** @} */

/**
 * @name        Default constructor of vfs_overlay_driver_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_overlay_driver_c.
 *
 * @param[out]    self          Pointer to a @p vfs_overlay_driver_c instance
 *                              to be initialized.
 * @param[in]     overlaid_drv  Pointer to a driver to be overlaid or @p NULL.
 * @param[in]     path_prefix   Prefix to be added to the paths or @p NULL, it
 *                              must be a normalized absolute path.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_overlay_driver_c *ovldrvObjectInit(vfs_overlay_driver_c *self,
                                                     vfs_driver_c *overlaid_drv,
                                                     const char *path_prefix) {
  extern const struct vfs_overlay_driver_vmt __vfs_overlay_driver_vmt;

  return __ovldrv_objinit_impl(self, &__vfs_overlay_driver_vmt, overlaid_drv,
                               path_prefix);
}
/** @} */

#endif /* VFS_CFG_ENABLE_DRV_OVERLAY == TRUE */

#endif /* DRVOVERLAY_H */

/** @} */
