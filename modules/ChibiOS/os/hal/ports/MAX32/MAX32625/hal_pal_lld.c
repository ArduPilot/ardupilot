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
 * @file    MAX32625/hal_pal_lld.c
 * @brief   MAX32625 PAL low level driver code.
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
  clkmanEnableGPIO();
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

  uint32_t omode = (mode & PAL_MAX32_OMODE_MASK) >> PAL_MAX32_OMODE_OFF;
  uint32_t funcsel = (mode & PAL_MAX32_FUNC_MASK) >> PAL_MAX32_FUNC_OFF;
  uint32_t imode = (mode & PAL_MAX32_IMODE_MASK) >> PAL_MAX32_IMODE_OFF;
  uint32_t bit = 0;
  while (true) {
    if ((mask & 1) != 0) {
      uint32_t m1, m2;
      /* Mask for bitfields large four bits. */
      m1 = 0x0FU << (bit * 4);
      
      /* Mask for bitfields large two bit. */
      m2 = 0x03U << (bit * 2);

      port->OUT_MODE = (port->OUT_MODE & ~m1) | omode;
      port->FUNC_SEL = (port->FUNC_SEL & ~m1) | funcsel;
      port->IN_MODE = (port->IN_MODE & ~m2) | imode;

    }
    mask >>= 1;
    
    /* Mask is over. End the loop. */
    
    if (!mask)
      return;
    omode <<= 4;
    funcsel <<= 4;
    imode <<= 2;
    bit++;
  }
}


/**
 * @brief   Sets a bits mask on a I/O port.
 * @details This function is implemented with a read modify write.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
void pal_lld_setport(ioportid_t port, uint32_t bits) {
  uint32_t val = pal_lld_readlatch(port);
  val |= bits;
  pal_lld_writeport(port, val);
}

/**
 * @brief   Clears a bits mask on a I/O port.
 * @details This function is implemented with a read modify write.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
void pal_lld_clearport(ioportid_t port, uint32_t bits) {
  uint32_t val = pal_lld_readlatch(port);
  val &= ~bits;
  pal_lld_writeport(port, val);
}
#endif /* HAL_USE_PAL */

/** @} */
