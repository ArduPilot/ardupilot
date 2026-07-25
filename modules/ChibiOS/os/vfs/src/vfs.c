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
 * @file    vfs/src/vfs.c
 * @brief   VFS API code.
 *
 * @addtogroup VFS
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

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   VFS initialization.
 *
 * @init
 */
void vfsInit(void) {

  /* Shared buffers manager initialization.*/
  __vfs_buffers_init();

#if VFS_CFG_ENABLE_DRV_OVERLAY == TRUE
  __drv_overlay_init();
#endif

#if VFS_CFG_ENABLE_DRV_STREAMS == TRUE
  __drv_streams_init();
#endif

#if VFS_CFG_ENABLE_DRV_CHFS == TRUE
  __drv_chfs_init();
#endif

#if VFS_CFG_ENABLE_DRV_FATFS == TRUE
  __drv_fatfs_init();
#endif

#if VFS_CFG_ENABLE_DRV_LITTLEFS == TRUE
  __drv_littlefs_init();
#endif
}

/**
 * @brief   Changes the current VFS directory.
 *
 * @param[in] path      Path of the new current directory.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsChangeCurrentDirectory(const char *path) {

  return vfsDrvChangeCurrentDirectory(vfs_root, path);
}

/**
 * @brief   Returns the current VFS directory.
 *
 * @param[out] buf      Buffer for the path string.
 * @param[in] size      Size of the buffer.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsGetCurrentDirectory(char *buf, size_t size) {

  return vfsDrvGetCurrentDirectory(vfs_root, buf, size);
}

/**
 * @brief   Returns file or directory information.
 *
 * @param[in] path      Absolute path of the node to be examined.
 * @param[out] sp       Pointer to a @p vfs_stat_t structure.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsStat(const char *path, vfs_stat_t *sp) {

  return vfsDrvStat(vfs_root, path, sp);
}

/**
 * @brief   Opens a VFS file or directory.
 *
 * @param[in] path      Absolute path of the node to be opened.
 * @param[in] flags     Open flags.
 * @param[out] vnpp     Pointer to the pointer to the instantiated
 *                      @p vfs_node_c object.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsOpen(const char *path, int flags, vfs_node_c **vnpp) {

  return vfsDrvOpen(vfs_root, path, flags, vnpp);
}

/**
 * @brief   Opens a VFS directory.
 *
 * @param[in] path      Absolute path of the directory to be opened.
 * @param[out] vdnpp    Pointer to the pointer to the instantiated
 *                      @p vfs_directory_node_c object.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsOpenDirectory(const char *path, vfs_directory_node_c **vdnpp) {

  return vfsDrvOpenDirectory(vfs_root, path, vdnpp);
}

/**
 * @brief   Opens a VFS file.
 *
 * @param[in] path      Path of the file to be opened.
 * @param[in] flags     File open flags.
 * @param[out] vfnpp    Pointer to the pointer to the instantiated
 *                      @p vfs_file_node_c object.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsOpenFile(const char *path, int flags, vfs_file_node_c **vfnpp) {

  return vfsDrvOpenFile(vfs_root, path, flags, vfnpp);
}

/**
 * @brief   Unlinks and possibly deletes a file.
 *
 * @param[in] path      Path of the file to be unlinked.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsUnlink(const char *path) {

  return vfsDrvUnlink(vfs_root, path);
}

/**
 * @brief   Renames a file or directory.
 *
 * @param[in] oldpath   Path of the file to be renamed.
 * @param[in] newpath   New path of the renamed file.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsRename(const char *oldpath, const char *newpath) {

  return vfsDrvRename(vfs_root, oldpath, newpath);
}

/**
 * @brief   Creates a directory.
 *
 * @param[in] path      Path of the directory to be created.
 * @param[in] mode      Mode flags for the directory.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsMkdir(const char *path, vfs_mode_t mode) {

  return vfsDrvMkdir(vfs_root, path, mode);
}

/**
 * @brief   Removes a directory.
 *
 * @param[in] path      Path of the directory to be removed.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsRmdir(const char *path) {

  return vfsDrvRmdir(vfs_root, path);
}

/**
 * @brief   Returns node information.
 *
 * @param[in] np        Pointer to the @p vfs_node_c object
 * @param[out] sp       Pointer to a @p vfs_stat_t structure.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsGetNodeStat(vfs_node_c *np, vfs_stat_t *sp) {

  chDbgAssert(np->references > 0U, "zero count");

  return vfsNodeStat((void *)np, sp);
}

/**
 * @brief   Releases a @p vfs_node_c or descendant object.
 *
 * @param[in] vnp       Pointer to the @p vfs_node_c object to be released.
 *
 * @api
 */
void vfsClose(vfs_node_c *vnp) {

  chDbgAssert(vnp->references > 0U, "zero count");

  roRelease((void *)vnp);
}

/**
 * @brief   First directory entry.
 *
 * @param[in] vdnp      Pointer to the @p vfs_directory_node_c object.
 * @param[out] dip      Pointer to a @p vfs_direntry_info_t structure.
 * @return              The operation result.
 * @retval 0            Zero entries read, end-of-directory condition.
 * @retval 1            One directory entry read.
 *
 * @api
 */
msg_t vfsReadDirectoryFirst(vfs_directory_node_c *vdnp,
                            vfs_direntry_info_t *dip) {

  chDbgAssert(vdnp->references > 0U, "zero count");

  return vfsDirReadFirst((void *)vdnp, dip);
}

/**
 * @brief   Next directory entry.
 *
 * @param[in] vdnp      Pointer to the @p vfs_directory_node_c object.
 * @param[out] dip      Pointer to a @p vfs_direntry_info_t structure.
 * @return              The operation result.
 * @retval 0            Zero entries read, end-of-directory condition.
 * @retval 1            One directory entry read.
 *
 * @api
 */
msg_t vfsReadDirectoryNext(vfs_directory_node_c *vdnp,
                           vfs_direntry_info_t *dip) {

  chDbgAssert(vdnp->references > 0U, "zero count");

  return vfsDirReadNext((void *)vdnp, dip);
}

/**
 * @brief   File node read.
 * @details The function reads data from a file node into a buffer.
 *
 * @param[in] vfnp      Pointer to the @p vfs_file_node_c object.
 * @param[in] buf       Pointer to the data buffer.
 * @param[in] n         Maximum amount of data to be transferred.
 * @return              The transferred number of bytes or an error.
 *
 * @api
 */
ssize_t vfsReadFile(vfs_file_node_c *vfnp, uint8_t *buf, size_t n) {

  chDbgAssert(vfnp->references > 0U, "zero count");

  return vfsFileRead((void *)vfnp, buf, n);
}

/**
 * @brief   File node write.
 * @details The function writes data from a buffer to a file node.
 *
 * @param[in] vfnp      Pointer to the @p vfs_file_node_c object.
 * @param[out] buf      Pointer to the data buffer.
 * @param[in] n         Maximum amount of data to be transferred.
 * @return              The transferred number of bytes or an error.
 *
 * @api
 */
ssize_t vfsWriteFile(vfs_file_node_c *vfnp, const uint8_t *buf, size_t n) {

  chDbgAssert(vfnp->references > 0U, "zero count");

  return vfsFileWrite((void *)vfnp, buf, n);
}

/**
 * @brief   Changes the current file position.
 *
 * @param[in] vfnp      Pointer to the @p vfs_file_node_c object.
 * @param[in] offset    New absolute position.
 * @param[in] whence    Seek mode to be used.
 * @return              The operation result.
 *
 * @api
 */
msg_t vfsSetFilePosition(vfs_file_node_c *vfnp,
                         vfs_offset_t offset,
                         vfs_seekmode_t whence) {

  chDbgAssert(vfnp->references > 0U, "zero count");

  return vfsFileSetPosition((void *)vfnp, offset, whence);
}

/**
 * @brief   Returns the current file position.
 *
 * @param[in] vfnp      Pointer to the @p vfs_file_node_c object.
 * @return              The current file position.
 *
 * @api
 */
vfs_offset_t vfsGetFilePosition(vfs_file_node_c *vfnp) {

  chDbgAssert(vfnp->references > 0U, "zero count");

  return vfsFileGetPosition((void *)vfnp);
}

/**
 * @brief   Returns the inner stream associated to the file.
 *
 * @param[in] vfnp      Pointer to the @p vfs_file_node_c object.
 * @return              The inner stream interface.
 *
 * @api
 */
sequential_stream_i *vfsGetFileStream(vfs_file_node_c *vfnp) {

  chDbgAssert(vfnp->references > 0U, "zero count");

  return vfsFileGetStream((void *)vfnp);
}

/** @} */
