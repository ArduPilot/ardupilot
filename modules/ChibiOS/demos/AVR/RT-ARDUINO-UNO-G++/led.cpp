/*
    ChibiOS - Copyright (C) 2006..2022 Theodore Ateba

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
 * @file    led.cpp
 * @brief   led class source file.
 */

#include "hal.h"
#include "led.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Configure the led object port and pad.
 *
 * @param[in] port      port of the led
 * @param[in] pad       pad of the led
 */
void Led::init(ioportid_t port, iopadid_t pad) {

  ledPort = port;
  ledPad  = pad;
}

/**
 * @brief   Toggle the led.
 */
void Led::toggle(void) {

  palTogglePad(ledPort, ledPad);
}

/**
 * @brief   Turn on the led.
 */
void Led::on(void) {

  palSetPad(ledPort, ledPad);
}

/**
 * @brief   Turn off the led.
 */
void Led::off(void) {

  palClearPad(ledPort, ledPad);
}

#ifdef __cplusplus
}
#endif
