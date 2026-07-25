/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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
 * @file    ATXMEGAxxxA4U/hal_lld.c
 * @brief   AVR HAL subsystem low level driver code.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  OSC_CTRL |= OSC_RC32MEN_bm;             /* Setup 32Mhz crystal.           */

  while(!(OSC_STATUS & OSC_RC32MRDY_bm)); /* Wait the systeme clock to
                                              stabilize.                    */

  CCP = CCP_IOREG_gc;                     /* Trigger protection mechanism.  */
  CLK_CTRL = CLK_SCLKSEL_RC32M_gc;        /* Enable internal  32Mhz crystal.*/
}

/** @} */
