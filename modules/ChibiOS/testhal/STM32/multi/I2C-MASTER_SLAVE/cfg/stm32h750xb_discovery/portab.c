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
 * @file    portab.c
 * @brief   Application portability module code.
 *
 * @addtogroup application_portability
 * @{
 */

#include "hal.h"

#include "portab.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/
/*
 * HSE: 25 MHz
 * SYSCLOCK: 120MHz
 * I2C Standard mode 100kHz
 */
const I2CConfig i2c_cfg = {
  STM32_TIMINGR_PRESC(0x3U) |
  STM32_TIMINGR_SCLDEL(0x7U) | STM32_TIMINGR_SDADEL(0x0U) |
  STM32_TIMINGR_SCLH(0x75U)  | STM32_TIMINGR_SCLL(0xB1U),
  0,
  0
};

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

void portab_setup(void) {

  /* Master I2C1 SCL on PB6, SDA on PB7 */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN |
                          PAL_STM32_PUPDR_PULLUP);
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN |
                          PAL_STM32_PUPDR_PULLUP);

  /* Slave I2C4 SCL on PD12, SDA on PD13 */
  palSetPadMode(GPIOD, 12, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN |
                          PAL_STM32_PUPDR_PULLUP);
  palSetPadMode(GPIOD, 13, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN |
                          PAL_STM32_PUPDR_PULLUP);
}

/** @} */
