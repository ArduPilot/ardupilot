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
 * @file    sglob.h
 * @brief   Simplified glob macros and structures.
 *
 * @addtogroup SGLOB
 * @{
 */

#ifndef SGLOB_H
#define SGLOB_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#define SGLOB_NOSPACE                   1
#define SGLOB_NOMATCH                   3

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct lstring {
  struct lstring        *next;
  char                  string[];
} lstring_t;

typedef struct {
  lstring_t             sgl_hdr;
  lstring_t             *sgl_last;
  char                  *sgl_buf;
  int                   sgl_pathc;
  char                  **sgl_pathv;
} sglob_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sglob_init(sglob_t *psglob);
  int sglob_add(sglob_t *psglob, const char *s);
  int sglob_match(sglob_t *psglob, const char *pattern);
  int sglob_build(sglob_t *psglob, size_t offs);
  void sglob_free(sglob_t *psglob);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* SGLOB_H */

/** @} */
