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
 * @file    vfs/src/vfsparser.c
 * @brief   VFS parser utilities code.
 *
 * @addtogroup VFS_PARSER
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
 * @brief   Matches a path separator.
 *
 * @param[in, out]  pathp       Pointer to the path under parsing.
 */
msg_t vfs_parse_match_separator(const char **pathp) {
  msg_t err;
  const char *p = *pathp;

  if (!vfs_path_is_separator(*p++)) {
    err = CH_RET_ENOENT;
  }
  else {
    err = CH_RET_SUCCESS;
    *pathp = p;
  }

  return err;
}

/**
 * @brief   Matches a string end.
 *
 * @param[in, out]  pathp       Pointer to the path under parsing.
 */
msg_t vfs_parse_match_end(const char **pathp) {
  msg_t err;

  if (**pathp != '\0') {
    err = CH_RET_ENOENT;
  }
  else {
    err = CH_RET_SUCCESS;
  }

  return err;
}

/** @} */
