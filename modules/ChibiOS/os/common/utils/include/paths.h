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
 * @file    paths.h
 * @brief   Path utilities header file.
 *
 * @addtogroup UTILS_PATHS
 * @{
 */

#ifndef PATHS_H
#define PATHS_H

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
  size_t path_append(char *dst, const char *src, size_t size);
  size_t path_prepend(char *dst, const char *src, size_t size);
  size_t path_add_separator(char *dst, size_t size);
  size_t path_add_extension(char *dst, const char *ext, size_t size);
  size_t path_copy_element(const char **pathp, char *dst, size_t size);
  size_t path_get_element(const char **pathp, char *dst, size_t size);
  size_t path_match_element(const char *path, const char *match, size_t size);
  size_t path_normalize(char *dst, const char *src, size_t size);
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
static inline bool path_is_separator(char c) {

  return (bool)(c == '/');
}

#endif /* vfspaths.h */

/** @} */
