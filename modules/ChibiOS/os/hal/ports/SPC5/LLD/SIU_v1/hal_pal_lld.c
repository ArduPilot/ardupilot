/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC5xx/SIU_v1/hal_pal_lld.c
 * @brief   SPC5xx SIU low level driver code.
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

#if defined(SPC5_SIU_SYSTEM_PINS)
static const unsigned system_pins[] = {SPC5_SIU_SYSTEM_PINS};
#endif

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
 * @brief   SPC5xx I/O ports configuration.
 *
 * @param[in] config    the SPC5xx ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {
  unsigned i;

  /* Initialize PCR registers for defined pads.*/
  i = 0;
  while (config->inits[i].pcr_index != -1) {
    SIU.GPDO[config->inits[i].pcr_index].R = config->inits[i].gpdo_value;
    SIU.PCR[config->inits[i].pcr_index].R  = config->inits[i].pcr_value;
    i++;
  }
}

/**
 * @brief   Reads a group of bits.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @return              The group logical states.
 *
 * @notapi
 */
ioportmask_t _pal_lld_readgroup(ioportid_t port,
                                ioportmask_t mask,
                                uint_fast8_t offset) {

  (void)port;
  (void)mask;
  (void)offset;
  return 0;
}

/**
 * @brief   Writes a group of bits.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] bits      bits to be written. Values exceeding the group width
 *                      are masked.
 *
 * @notapi
 */
void _pal_lld_writegroup(ioportid_t port,
                         ioportmask_t mask,
                         uint_fast8_t offset,
                         ioportmask_t bits) {

  (void)port;
  (void)mask;
  (void)offset;
  (void)bits;
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
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
  unsigned pcr_index = (unsigned)(port * PAL_IOPORTS_WIDTH);
  ioportmask_t m1 = 0x8000;
  while (m1) {
    if (mask & m1)
      SIU.PCR[pcr_index].R = mode;
    m1 >>= 1;
    ++pcr_index;
  }
}

#endif /* HAL_USE_PAL */

/** @} */
