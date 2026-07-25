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

#if HAL_USE_PAL || defined(__DOXYGEN__)
/* Initial setup of all defined pads, the list is terminated by a {-1, 0, 0}.*/
static const spc_siu_init_t spc_siu_init[] = {
  {PCR(PORT5, P5_ESCI_A_TX),      PAL_HIGH,   PAL_MODE_INPUT_ALTERNATE_PRIMARY},
  {PCR(PORT5, P5_ESCI_A_RX),      PAL_HIGH,   PAL_MODE_INPUT_ALTERNATE_PRIMARY},
  {PCR(PORT11, P11_BUTTON1),      PAL_LOW,    PAL_MODE_INPUT},
  {PCR(PORT11, P11_BUTTON2),      PAL_LOW,    PAL_MODE_INPUT},
  {PCR(PORT11, P11_BUTTON3),      PAL_LOW,    PAL_MODE_INPUT},
  {PCR(PORT11, P11_BUTTON4),      PAL_LOW,    PAL_MODE_INPUT},
  {PCR(PORT11, P11_LED1),         PAL_HIGH,   PAL_MODE_OUTPUT_PUSHPULL},
  {PCR(PORT11, P11_LED2),         PAL_HIGH,   PAL_MODE_OUTPUT_PUSHPULL},
  {PCR(PORT11, P11_LED3),         PAL_HIGH,   PAL_MODE_OUTPUT_PUSHPULL},
  {PCR(PORT11, P11_LED4),         PAL_HIGH,   PAL_MODE_OUTPUT_PUSHPULL},
  {-1, 0, 0}
};

/**
 * @brief   PAL setup.
 */
const PALConfig pal_default_config = {
  spc_siu_init
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  spc_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
}
