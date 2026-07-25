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
 * @file    vfs/src/vfspaths.h
 * @brief   VFS paths utilities header file.
 *
 * @addtogroup VFS_PATHS
 * @{
 */

#ifndef VFSPATHS_H
#define VFSPATHS_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  size_t vfs_path_append(char *dst, const char *src, size_t size);
  size_t vfs_path_prepend(char *dst, const char *src, size_t size);
  size_t vfs_path_add_separator(char *dst, size_t size);
  size_t vfs_path_add_extension(char *dst, const char *ext, size_t size);
  size_t vfs_path_copy_element(const char **pathp, char *dst, size_t size);
  size_t vfs_path_get_element(const char **pathp, char *dst, size_t size);
  size_t vfs_path_match_element(const char *path, const char *match, size_t size);
  size_t vfs_path_normalize(char *dst, const char *src, size_t size);
  size_t vfs_path_make_absolute(char *dst, const char *src,
                                size_t size, const char *cwd);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Checks for a valid path separator.
 *
 * @param[in] c                 The character to be checked.
 * @return                      The operation result.
 */
static inline bool vfs_path_is_separator(char c) {

  return (bool)(c == '/');
}

#endif /* VFSPATHS_H */

/** @} */
