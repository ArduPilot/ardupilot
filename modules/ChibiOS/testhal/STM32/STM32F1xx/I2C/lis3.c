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
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "lis3.h"

/* device I2C address */
#define addr 0b0011101

/* enable single byte read checks. Note: it does not work on STM32F1x */
#define TEST_SINGLE_BYTE_READ     TRUE

/* autoincrement bit position. This bit needs to perform reading of
 * multiple bytes at one request */
#define AUTO_INCREMENT_BIT (1<<7)

/* slave specific addresses */
#define ACCEL_STATUS_REG  0x27
#define ACCEL_CTRL_REG1   0x20
#define ACCEL_OUT_DATA    0x28

/* buffers */
static uint8_t accel_rx_data[8];
static uint8_t accel_tx_data[8];

/*
 *
 */
void lis3Start(void){
  msg_t status = MSG_OK;
  sysinterval_t tmo = TIME_MS2I(4);

  /* configure accelerometer */
  accel_tx_data[0] = ACCEL_CTRL_REG1 | AUTO_INCREMENT_BIT;
  accel_tx_data[1] = 0b11100111;
  accel_tx_data[2] = 0b01000001;
  accel_tx_data[3] = 0b00000000;

  /* sending */
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, addr,
                          accel_tx_data, 4, NULL, 0, tmo);
  i2cReleaseBus(&I2CD1);

  osalDbgCheck(MSG_OK == status);
}

/*
 *
 */
static void raw2g(uint8_t *raw, float *g) {
  int16_t tmp;

  for (size_t i=0; i<3; i++){
    tmp = raw[i*2] | (raw[i*2+1] << 8);
    g[i] = (float)tmp / 16384.0; /* convert raw value to G */
  }
}

/*
 *
 */
void lis3GetAcc(float *result) {
  msg_t status = MSG_OK;
  sysinterval_t tmo = TIME_MS2I(4);

  /* read in burst mode */
  memset(accel_rx_data, 0x55, sizeof(accel_rx_data));
  accel_tx_data[0] = ACCEL_OUT_DATA | AUTO_INCREMENT_BIT;
  i2cAcquireBus(&I2CD1);
  status = i2cMasterTransmitTimeout(&I2CD1, addr,
                          accel_tx_data, 1, accel_rx_data, 6, tmo);
  i2cReleaseBus(&I2CD1);
  osalDbgCheck(MSG_OK == status);
  raw2g(accel_rx_data, result);

#if TEST_SINGLE_BYTE_READ
  float accel_single_byte_check[3];
  const float check_threshold = 0.1;

  /* read data byte at a time */
  memset(accel_rx_data, 0x55, sizeof(accel_rx_data));
  accel_tx_data[0] = ACCEL_OUT_DATA;
  i2cAcquireBus(&I2CD1);
  for (size_t i=0; i<6; i++) {
    status = i2cMasterTransmitTimeout(&I2CD1, addr,
                            accel_tx_data, 1, &accel_rx_data[i], 1, tmo);
    osalDbgCheck(MSG_OK == status);
    accel_tx_data[0]++;
  }
  i2cReleaseBus(&I2CD1);
  raw2g(accel_rx_data, accel_single_byte_check);

  /* check results */
  for (size_t i=0; i<3; i++) {
    osalDbgCheck(fabsf(result[i] - accel_single_byte_check[i]) < check_threshold);
  }
#endif /* TEST_SINGLE_BYTE_READ */
}


