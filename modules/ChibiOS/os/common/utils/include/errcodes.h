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
 * @file    errcodes.h
 * @brief   Errors handling header file.
 *
 * @addtogroup UTILS_ERRCODES
 * @{
 */

#ifndef ERRCODES_H
#define ERRCODES_H

#include <errno.h>

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    Error codes
 * @{
 */
#define CH_RET_SUCCESS          (int)MSG_OK                 /* Success */
#define CH_RET_TIMEOUT          (int)MSG_TIMEOUT            /* Timeout */
#define CH_RET_INNER_ERROR      (int)-3                     /* Unexpected condition */
/** @} */

/**
 * @name    Extra error codes mapped on Posix errors
 * @{
 */
#define CH_RET_ENOENT           CH_ENCODE_ERROR(ENOENT)     /* No such file or directory */
#define CH_RET_EIO              CH_ENCODE_ERROR(EIO)        /* I/O error */
#define CH_RET_EBADF            CH_ENCODE_ERROR(EBADF)      /* Bad file number */
#define CH_RET_ENOMEM           CH_ENCODE_ERROR(ENOMEM)     /* Not enough space */
#define CH_RET_EACCES           CH_ENCODE_ERROR(EACCES)     /* Permission denied */
#define CH_RET_EFAULT           CH_ENCODE_ERROR(EFAULT)     /* Bad address */
#define CH_RET_EBUSY            CH_ENCODE_ERROR(EBUSY)      /* Device or resource busy */
#define CH_RET_EEXIST           CH_ENCODE_ERROR(EEXIST)     /* File exists */
#define CH_RET_ENOTDIR          CH_ENCODE_ERROR(ENOTDIR)    /* Not a directory */
#define CH_RET_EISDIR           CH_ENCODE_ERROR(EISDIR)     /* Is a directory */
#define CH_RET_EINVAL           CH_ENCODE_ERROR(EINVAL)     /* Invalid argument */
#define CH_RET_EMFILE           CH_ENCODE_ERROR(EMFILE)     /* Too many open files in process */
#define CH_RET_ENFILE           CH_ENCODE_ERROR(ENFILE)     /* Too many open files in system */
#define CH_RET_EFBIG            CH_ENCODE_ERROR(EFBIG)      /* File too large */
#define CH_RET_ENOSPC           CH_ENCODE_ERROR(ENOSPC)     /* No space left on device */
#define CH_RET_ESPIPE           CH_ENCODE_ERROR(ESPIPE)     /* Illegal seek */
#define CH_RET_EROFS            CH_ENCODE_ERROR(EROFS)      /* Read-only file system */
#define CH_RET_ERANGE           CH_ENCODE_ERROR(ERANGE)     /* Result too large */
#define CH_RET_ENAMETOOLONG     CH_ENCODE_ERROR(ENAMETOOLONG)/* File or path name too long */
#define CH_RET_ENOSYS           CH_ENCODE_ERROR(ENOSYS)     /* Syscall not implemented */
#define CH_RET_EOVERFLOW        CH_ENCODE_ERROR(EOVERFLOW)  /* File offset overflow */
#define CH_RET_ENOEXEC          CH_ENCODE_ERROR(ENOEXEC)    /* Invalid executable */
#define CH_RET_EXDEV            CH_ENCODE_ERROR(EXDEV)      /* Not same volume */
/** @} */

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

/**
 * @name    Errors handling macros
 * @{
 */
#define CH_ERRORS_MASK              (int)0xFF
#define CH_ENCODE_ERROR(posixerr)   (~CH_ERRORS_MASK | (int)(posixerr))
#define CH_DECODE_ERROR(err)        ((int)(err) & CH_ERRORS_MASK)
#define CH_RET_IS_ERROR(x)          (((int)(x) & ~CH_ERRORS_MASK) == ~CH_ERRORS_MASK)
#define CH_RET_IS_SUCCESS(x)        (((int)(x) & ~CH_ERRORS_MASK) != ~CH_ERRORS_MASK)

#define CH_BREAK_ON_ERROR(err)                                              \
  if (CH_RET_IS_ERROR(err)) break

#define CH_BREAK_ON_SUCCESS(err)                                            \
  if (CH_RET_IS_SUCCESS(err)) break

#define CH_RETURN_ON_ERROR(err) do {                                        \
  int __ret = (err);                                                        \
  if (CH_RET_IS_ERROR(__ret)) {                                             \
    return __ret;                                                           \
  }                                                                         \
} while (false)

#define CH_RETURN_ON_SUCCESS(err) do {                                      \
  int __ret = (err);                                                        \
  if (CH_RET_IS_SUCCESS(__ret)) {                                           \
    return __ret;                                                           \
  }                                                                         \
} while (false)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* ERRCODES_H */

/** @} */
