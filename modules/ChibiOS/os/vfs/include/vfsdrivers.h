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
 * @file        vfsdrivers.h
 * @brief       Generated VFS Drivers header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  VFSDRIVERS
 * @brief       Common ancestor class of all reference-counted objects.
 * @{
 */

#ifndef VFSDRIVERS_H
#define VFSDRIVERS_H

#include <fcntl.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    File open flags compatible with Posix
 * @{
 */
#define VO_SUPPORTED_FLAGS_MASK             (O_ACCMODE | O_APPEND | O_CREAT | O_TRUNC | O_EXCL)
#define VO_ACCMODE                          O_ACCMODE
#define VO_RDONLY                           O_RDONLY
#define VO_WRONLY                           O_WRONLY
#define VO_RDWR                             O_RDWR
#define VO_APPEND                           O_APPEND
#define VO_CREAT                            O_CREAT
#define VO_TRUNC                            O_TRUNC
#define VO_EXCL                             O_EXCL
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @class       vfs_driver_c
 * @extends     base_object_c
 *
 * @brief       Common ancestor class of all VFS driver classes.
 * @details     Base class for objects that implement a Posix-like file system
 *              interface.
 *
 * @name        Class @p vfs_driver_c structures
 * @{
 */

/**
 * @brief       Type of a VFS driver class.
 */
typedef struct vfs_driver vfs_driver_c;

/**
 * @brief       Class @p vfs_driver_c virtual methods table.
 */
struct vfs_driver_vmt {
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
};

/**
 * @brief       Structure representing a VFS driver class.
 */
struct vfs_driver {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct vfs_driver_vmt *vmt;
};
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  /* Methods of vfs_driver_c.*/
  void *__vfsdrv_objinit_impl(void *ip, const void *vmt);
  void __vfsdrv_dispose_impl(void *ip);
  msg_t __vfsdrv_setcwd_impl(void *ip, const char *path);
  msg_t __vfsdrv_getcwd_impl(void *ip, char *buf, size_t size);
  msg_t __vfsdrv_stat_impl(void *ip, const char *path, vfs_stat_t *sp);
  msg_t __vfsdrv_opendir_impl(void *ip, const char *path,
                              vfs_directory_node_c **vdnpp);
  msg_t __vfsdrv_openfile_impl(void *ip, const char *path, int flags,
                               vfs_file_node_c **vfnpp);
  msg_t __vfsdrv_unlink_impl(void *ip, const char *path);
  msg_t __vfsdrv_rename_impl(void *ip, const char *oldpath,
                             const char *newpath);
  msg_t __vfsdrv_mkdir_impl(void *ip, const char *path, vfs_mode_t mode);
  msg_t __vfsdrv_rmdir_impl(void *ip, const char *path);
  /* Regular functions.*/
  msg_t vfsDrvOpen(vfs_driver_c *drvp, const char *path, int flags,
                   vfs_node_c **vnpp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Virtual methods of vfs_driver_c
 * @{
 */
/**
 * @brief       Changes the current VFS directory.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[in]     path          Path of the new current directory.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvChangeCurrentDirectory(void *ip, const char *path) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->setcwd(ip, path);
}

/**
 * @brief       Returns the current VFS directory.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[out]    buf           Buffer for the path string.
 * @param[in]     size          Size of the buffer.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvGetCurrentDirectory(void *ip, char *buf, size_t size) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->getcwd(ip, buf, size);
}

/**
 * @brief       Returns file or directory information.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[in]     path          Absolute path of the node to be examined.
 * @param[out]    sp            Pointer to a @p vfs_stat_t structure.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvStat(void *ip, const char *path, vfs_stat_t *sp) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->stat(ip, path, sp);
}

/**
 * @brief       Opens a VFS directory.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[in]     path          Absolute path of the directory to be opened.
 * @param[out]    vdnpp         Pointer to the pointer to the instantiated @p
 *                              vfs_directory_node_c object.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvOpenDirectory(void *ip, const char *path,
                                        vfs_directory_node_c **vdnpp) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->opendir(ip, path, vdnpp);
}

/**
 * @brief       Opens a VFS file.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[in]     path          Absolute path of the directory to be opened.
 * @param[in]     flags         File open flags.
 * @param[out]    vfnpp         Pointer to the pointer to the instantiated @p
 *                              vfs_file_node_c object.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvOpenFile(void *ip, const char *path, int flags,
                                   vfs_file_node_c **vfnpp) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->openfile(ip, path, flags, vfnpp);
}

/**
 * @brief       Unlinks and possibly deletes a file.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[in]     path          Path of the file to be unlinked.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvUnlink(void *ip, const char *path) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->unlink(ip, path);
}

/**
 * @brief       Renames a file or directory.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[in]     oldpath       Path of the node to be renamed.
 * @param[in]     newpath       New path of the renamed node.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvRename(void *ip, const char *oldpath,
                                 const char *newpath) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->rename(ip, oldpath, newpath);
}

/**
 * @brief       Creates a directory.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[in]     path          Path of the directory to be created.
 * @param[in]     mode          Mode flags for the directory.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvMkdir(void *ip, const char *path, vfs_mode_t mode) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->mkdir(ip, path, mode);
}

/**
 * @brief       Removes a directory.
 *
 * @param[in,out] ip            Pointer to a @p vfs_driver_c instance.
 * @param[in]     path          Path of the directory to be removed.
 * @return                      The operation result.
 *
 * @api
 */
CC_FORCE_INLINE
static inline msg_t vfsDrvRmdir(void *ip, const char *path) {
  vfs_driver_c *self = (vfs_driver_c *)ip;

  return self->vmt->rmdir(ip, path);
}
/** @} */

#endif /* VFSDRIVERS_H */

/** @} */
