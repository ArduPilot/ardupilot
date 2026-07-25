/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/hal_pal_lld.c
 * @brief   ADUCM36x PAL low level driver code.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   PAL driver initialization.
 *
 * @notapi
 */
void _pal_lld_init(void) {

}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    @p PAL_MODE_UNCONNECTED is implemented as push pull at minimum
 *          speed.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {

  uint32_t con = (mode & PAL_ADUCM_CON_MASK) >> PAL_ADUCM_CON_OFF;
  uint32_t oen = (mode & PAL_ADUCM_OEN_MASK) >> PAL_ADUCM_OEN_OFF;
  uint32_t pul = (mode & PAL_ADUCM_PUL_MASK) >> PAL_ADUCM_PUL_OFF;
  uint32_t oce = (mode & PAL_ADUCM_OCE_MASK) >> PAL_ADUCM_OCE_OFF;
  uint32_t bit = 0;
  while (true) {
    if ((mask & 1) != 0) {
      uint32_t m1, m2;
      m1 = 3 << (bit * 2);
      m2 = 1 << bit;

      port->CON = (port->CON & ~m1) | con;
      port->OEN = (port->OEN & ~m2) | oen;
      port->PUL = (port->PUL & ~m2) | pul;
      port->OCE = (port->OCE & ~m2) | oce;

    }
    mask >>= 1;
    if (!mask)
      return;
    con <<= 2;
    oen <<= 1;
    pul <<= 1;
    oce <<= 1;
    bit++;
  }
}

#endif /* HAL_USE_PAL */

/** @} */
