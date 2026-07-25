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
 * @file    sb/common/sberr.h
 * @brief   ARMv7-M sandbox common macros and structures.
 *
 * @addtogroup ARM_SANDBOX_ERRORS
 * @{
 */

#ifndef SBERR_H
#define SBERR_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    Sandbox API error codes
 * @{
 */
#define SB_ERR_NOERROR          0U
#define SB_ERR_ENOENT           ((uint32_t)(-2))
#define SB_ERR_EFAULT           ((uint32_t)(-14))
#define SB_ERR_EBUSY            ((uint32_t)(-16))
#define SB_ERR_EINVAL           ((uint32_t)(-22))
#define SB_ERR_ESPIPE           ((uint32_t)(-29))
#define SB_ERR_EBADFD           ((uint32_t)(-81))
#define SB_ERR_ENOSYS           ((uint32_t)(-88))

#define SB_ERR_ERRORMASK        0xFFFFFF00U
#define SB_ERR_ISERROR(x)       (((uint32_t)(x) & SB_ERR_ERRORMASK) == SB_ERR_ERRORMASK)
/** @} */

/**
 * @name    Posix-like function codes
 * @{
 */
#define SB_POSIX_OPEN           1
#define SB_POSIX_CLOSE          2
#define SB_POSIX_READ           3
#define SB_POSIX_WRITE          4
#define SB_POSIX_LSEEK          5
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

#endif /* SBERR_H */

/** @} */
