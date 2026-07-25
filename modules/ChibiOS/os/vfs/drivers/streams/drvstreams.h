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
 * @file        drvstreams.h
 * @brief       Generated VFS Streams Driver header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  DRVSTREAMS
 * @{
 */

#ifndef DRVSTREAMS_H
#define DRVSTREAMS_H

#if (VFS_CFG_ENABLE_DRV_STREAMS == TRUE) || defined(__DOXYGEN__)

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
#if !defined(DRV_CFG_STREAMS_DIR_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_STREAMS_DIR_NODES_NUM       1
#endif

/**
 * @brief       Number of file nodes pre-allocated in the pool.
 */
#if !defined(DRV_CFG_STREAMS_FILE_NODES_NUM) || defined(__DOXYGEN__)
#define DRV_CFG_STREAMS_FILE_NODES_NUM      1
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on DRV_CFG_STREAMS_DIR_NODES_NUM configuration.*/
#if DRV_CFG_STREAMS_DIR_NODES_NUM < 1
#error "invalid DRV_CFG_STREAMS_DIR_NODES_NUM value"
#endif

/* Checks on DRV_CFG_STREAMS_FILE_NODES_NUM configuration.*/
#if DRV_CFG_STREAMS_FILE_NODES_NUM < 1
#error "invalid DRV_CFG_STREAMS_FILE_NODES_NUM value"
#endif

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief       Type of a stream association structure.
 */
typedef struct drv_streams_element drv_streams_element_t;

/**
 * @brief       Structure representing a stream association.
 */
struct drv_streams_element {
  /**
   * @brief       Filename for the stream.
   */
  const char                *name;
  /**
   * @brief       Pointer to the stream.
   */
  sequential_stream_i       *stm;
  /**
   * @brief       Stream mode.
   */
  vfs_mode_t                mode;
};

/**
 * @class       vfs_streams_dir_node_c
 * @extends     vfs_directory_node_c
 *
 *
 * @name        Class @p vfs_streams_dir_node_c structures
 * @{
 */

/**
 * @brief       Type of a VFS streams directory node class.
 */
typedef struct vfs_streams_dir_node vfs_streams_dir_node_c;

/**
 * @brief       Class @p vfs_streams_dir_node_c virtual methods table.
 */
struct vfs_streams_dir_node_vmt {
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
  /* From vfs_streams_dir_node_c.*/
};

/**
 * @brief       Structure representing a VFS streams directory node class.
 */
struct vfs_streams_dir_node {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_streams_dir_node_vmt *vmt;
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
   * @brief       Current directory entry during scanning.
   */
  unsigned                  index;
};
/** @} */

/**
 * @class       vfs_streams_file_node_c
 * @extends     vfs_file_node_c
 *
 *
 * @name        Class @p vfs_streams_file_node_c structures
 * @{
 */

/**
 * @brief       Type of a VFS streams file node class.
 */
typedef struct vfs_streams_file_node vfs_streams_file_node_c;

/**
 * @brief       Class @p vfs_streams_file_node_c virtual methods table.
 */
struct vfs_streams_file_node_vmt {
  /* From base_object_c.*/
  void (*dispose)(void *ip);
  /* From referenced_object_c.*/
  void * (*addref)(void *ip);
  object_references_t (*release)(void *ip);
  /* From vfs_node_c.*/
  msg_t (*stat)(void *ip, vfs_stat_t *sp);
  /* From vfs_file_node_c.*/
  ssize_t (*read)(void *ip, uint8_t *buf, size_t n);
  ssize_t (*write)(void *ip, const uint8_t *buf, size_t n);
  msg_t (*setpos)(void *ip, vfs_offset_t offset, vfs_seekmode_t whence);
  vfs_offset_t (*getpos)(void *ip);
  sequential_stream_i * (*getstream)(void *ip);
  /* From vfs_streams_file_node_c.*/
};

/**
 * @brief       Structure representing a VFS streams file node class.
 */
struct vfs_streams_file_node {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_streams_file_node_vmt *vmt;
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
   * @brief       Stream interface for this file.
   */
  sequential_stream_i       *stm;
};
/** @} */

/**
 * @class       vfs_streams_driver_c
 * @extends     vfs_driver_c
 *
 *
 * @name        Class @p vfs_streams_driver_c structures
 * @{
 */

/**
 * @brief       Type of a VFS streams driver class.
 */
typedef struct vfs_streams_driver vfs_streams_driver_c;

/**
 * @brief       Class @p vfs_streams_driver_c virtual methods table.
 */
struct vfs_streams_driver_vmt {
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
  /* From vfs_streams_driver_c.*/
};

/**
 * @brief       Structure representing a VFS streams driver class.
 */
struct vfs_streams_driver {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_streams_driver_vmt *vmt;
  /**
   * @brief       Pointer to the stream elements to be exposed.
   */
  const drv_streams_element_t *streams;
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of vfs_streams_dir_node_c.*/
  void *__stmdir_objinit_impl(void *ip, const void *vmt, vfs_driver_c *driver,
                              vfs_mode_t mode);
  void __stmdir_dispose_impl(void *ip);
  msg_t __stmdir_stat_impl(void *ip, vfs_stat_t *sp);
  msg_t __stmdir_first_impl(void *ip, vfs_direntry_info_t *dip);
  msg_t __stmdir_next_impl(void *ip, vfs_direntry_info_t *dip);
  /* Methods of vfs_streams_file_node_c.*/
  void *__stmfile_objinit_impl(void *ip, const void *vmt, vfs_driver_c *driver,
                               vfs_mode_t mode, sequential_stream_i *stream);
  void __stmfile_dispose_impl(void *ip);
  msg_t __stmfile_stat_impl(void *ip, vfs_stat_t *sp);
  ssize_t __stmfile_read_impl(void *ip, uint8_t *buf, size_t n);
  ssize_t __stmfile_write_impl(void *ip, const uint8_t *buf, size_t n);
  msg_t __stmfile_setpos_impl(void *ip, vfs_offset_t offset,
                              vfs_seekmode_t whence);
  vfs_offset_t __stmfile_getpos_impl(void *ip);
  sequential_stream_i *__stmfile_getstream_impl(void *ip);
  /* Methods of vfs_streams_driver_c.*/
  void *__stmdrv_objinit_impl(void *ip, const void *vmt,
                              const drv_streams_element_t *streams);
  void __stmdrv_dispose_impl(void *ip);
  msg_t __stmdrv_setcwd_impl(void *ip, const char *path);
  msg_t __stmdrv_getcwd_impl(void *ip, char *buf, size_t size);
  msg_t __stmdrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp);
  msg_t __stmdrv_opendir_impl(void *ip, const char *path,
                              vfs_directory_node_c **vdnpp);
  msg_t __stmdrv_openfile_impl(void *ip, const char *path, int flags,
                               vfs_file_node_c **vfnpp);
  msg_t __stmdrv_unlink_impl(void *ip, const char *path);
  msg_t __stmdrv_rename_impl(void *ip, const char *oldpath,
                             const char *newpath);
  msg_t __stmdrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode);
  msg_t __stmdrv_rmdir_impl(void *ip, const char *path);
  /* Regular functions.*/
  void __drv_streams_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Default constructor of vfs_streams_dir_node_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_streams_dir_node_c.
 *
 * @param[out]    self          Pointer to a @p vfs_streams_dir_node_c instance
 *                              to be initialized.
 * @param[in]     driver        Pointer to the controlling driver.
 * @param[in]     mode          Node mode flags.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_streams_dir_node_c *stmdirObjectInit(vfs_streams_dir_node_c *self,
                                                       vfs_driver_c *driver,
                                                       vfs_mode_t mode) {
  extern const struct vfs_streams_dir_node_vmt __vfs_streams_dir_node_vmt;

  return __stmdir_objinit_impl(self, &__vfs_streams_dir_node_vmt, driver, mode);
}
/** @} */

/**
 * @name        Default constructor of vfs_streams_file_node_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_streams_file_node_c.
 *
 * @param[out]    self          Pointer to a @p vfs_streams_file_node_c
 *                              instance to be initialized.
 * @param[in]     driver        Pointer to the controlling driver.
 * @param[in]     mode          Node mode flags.
 * @param[in]     stream        Stream to be associated.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_streams_file_node_c *stmfileObjectInit(vfs_streams_file_node_c *self,
                                                         vfs_driver_c *driver,
                                                         vfs_mode_t mode,
                                                         sequential_stream_i *stream) {
  extern const struct vfs_streams_file_node_vmt __vfs_streams_file_node_vmt;

  return __stmfile_objinit_impl(self, &__vfs_streams_file_node_vmt, driver,
                                mode, stream);
}
/** @} */

/**
 * @name        Default constructor of vfs_streams_driver_c
 * @{
 */
/**
 * @brief       Default initialization function of @p vfs_streams_driver_c.
 *
 * @param[out]    self          Pointer to a @p vfs_streams_driver_c instance
 *                              to be initialized.
 * @param[in]     streams       Pointer to a @p vfs_streams_driver_c structure.
 * @return                      Pointer to the initialized object.
 *
 * @objinit
 */
CC_FORCE_INLINE
static inline vfs_streams_driver_c *stmdrvObjectInit(vfs_streams_driver_c *self,
                                                     const drv_streams_element_t *streams) {
  extern const struct vfs_streams_driver_vmt __vfs_streams_driver_vmt;

  return __stmdrv_objinit_impl(self, &__vfs_streams_driver_vmt, streams);
}
/** @} */

#endif /* VFS_CFG_ENABLE_DRV_STREAMS == TRUE */

#endif /* DRVSTREAMS_H */

/** @} */
