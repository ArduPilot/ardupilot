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

#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config = {
  {VAL_GPIO0DATA, VAL_GPIO0DIR},
  {VAL_GPIO1DATA, VAL_GPIO1DIR}
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
__inline void __early_init(void) {

  lpc_clock_init();
}

/*
 * Board-specific initialization code.
 */
__inline void boardInit(void) {

  /* LCD */
  LPC_IOCON->TMS_PIO0_12  = 0x91;       /* LCD_EN: GPIO - pull-up     */
  LPC_IOCON->TDO_PIO0_13  = 0x81;       /* LCD_RW: GPIO - No pull-up  */
  LPC_IOCON->TRST_PIO0_14 = 0x81;       /* LCD_RS: GPIO - No pull-up  */

  /* USART */
  LPC_IOCON->PIO0_18 = 0x81;            /* RDX: RXD - No pull-up      */
  LPC_IOCON->PIO0_19 = 0x81;            /* TDX: TXD - No pull-up      */

  /* Test LEDs */
  LPC_IOCON->PIO0_22 = 0x80;            /* LED_TEST1: GPIO - No pull-up */
  LPC_IOCON->PIO0_23 = 0x80;            /* LED_TEST2: GPIO - No pull-up */

}

