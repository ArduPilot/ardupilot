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
 * @file    rt/include/chrfcu.h
 * @brief   Runtime Faults Collection Unit macros and structures.
 *
 * @addtogroup rfcu
 * @{
 */

#ifndef CHRFCU_H
#define CHRFCU_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name    Predefined Faults
 * @{
 */
#define CH_RFCU_VT_INSUFFICIENT_DELTA       1U
#define CH_RFCU_VT_SKIPPED_DEADLINE         2U
/** @} */

/**
 * @brief   Mask of all faults.
 */
#define CH_RFCU_ALL_FAULTS                  ((rfcu_mask_t)-1)

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
 * @brief   Type of a faults mask.
 */
typedef uint32_t rfcu_mask_t;

/**
 * @brief   Type of an RFCU structure.
 */
typedef struct ch_rfcu {
  /**
   * @brief   Mask of the pending runtime faults.
   */
  rfcu_mask_t                   mask;
} rfcu_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chRFCUCollectFaultsI(rfcu_mask_t mask);
  rfcu_mask_t chRFCUGetAndClearFaultsI(rfcu_mask_t mask);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Runtime Faults Collection Unit initialization.
 * @note    Internal use only.
 *
 * @param[out] rfcup    pointer to the @p rfcu_t structure
 *
 * @notapi
 */
static inline void __rfcu_object_init(rfcu_t *rfcup) {

  rfcup->mask = (rfcu_mask_t)0;
}

#endif /* CHRFCU_H */

/** @} */
