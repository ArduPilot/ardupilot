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
 * Not responding slave test
 */
#include "ch.h"
#include "hal.h"

#define addr 0b1001100

static uint8_t rx_data[2];
static i2cflags_t errors = 0;

/* This is main function. */
void request_fake(void){
  msg_t status = MSG_OK;
  sysinterval_t tmo = TIME_MS2I(4);

  i2cAcquireBus(&I2CD1);
  status = i2cMasterReceiveTimeout(&I2CD1, addr, rx_data, 2, tmo);
  i2cReleaseBus(&I2CD1);

  if (status == MSG_RESET){
    errors = i2cGetErrors(&I2CD1);
    osalDbgCheck(I2C_ACK_FAILURE == errors);
  }
}


