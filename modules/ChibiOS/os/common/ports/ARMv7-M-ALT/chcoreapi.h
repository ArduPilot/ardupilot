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
 * @file    ARMv7-M-ALT/chcoreapi.h
 * @brief   ARMv7-M (alternate) specific API macros and structures.
 *
 * @addtogroup ARMV7M_ALT_COREAPI
 * @{
 */

#ifndef CHCOREAPI_H
#define CHCOREAPI_H

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

/**
 * @brief   Type of an MPU region registers structure.
 *
 */
typedef struct {
  uint32_t        rbar;
  uint32_t        rasr;
} mpureg_t;

/**
 * @brief   Type of an unprivileged thread descriptor.
 */
typedef struct {
  /**
   * @brief   Thread name.
   */
  const char        *name;
  /**
   * @brief   Pointer to the working area base.
   */
  stkalign_t        *wbase;
  /**
   * @brief   Pointer to the working area end.
   */
  stkalign_t        *wend;
  /**
   * @brief   Thread priority.
   */
  tprio_t           prio;
  /**
   * @brief   Initial unprivileged PC value.
   */
  uint32_t          u_pc;
  /**
   * @brief   Initial unprivileged PSP value.
   */
  uint32_t          u_psp;
  /**
   * @brief   Thread argument.
   */
  void              *arg;
  /**
   * @brief   MPU regions to be initialized.
   */
  mpureg_t          regions [PORT_SWITCHED_REGIONS_NUMBER];
} unprivileged_thread_descriptor_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)

#ifdef __cplusplus
extern "C" {
#endif
  thread_t *chThdCreateUnprivileged(const unprivileged_thread_descriptor_t *utdp);
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* Module late inclusions.                                                   */
/*===========================================================================*/

#endif /* CHCOREAPI_H */

/** @} */
