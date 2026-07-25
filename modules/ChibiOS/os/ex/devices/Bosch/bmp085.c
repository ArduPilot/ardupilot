/*
    ChibiOS - Copyright (C) 2016..2017 Theodore Ateba

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/**
 * @file        bmp085.c
 * @brief       BMP085 Digital pressure sensor interface module code.
 *
 * @addtogroup BMP085
 * @ingroup EX_BOSCH
 * @{
 */

#include "hal.h"
#include "bmp085.h"

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

#define BMP085_SAD                  0x77

#define BMP085_CR_P_VAL0            0x34
#define BMP085_CR_P_VAL1            0x74
#define BMP085_CR_P_VAL2            0xB4
#define BMP085_CR_P_VAL3            0xF4

#define BMP085_CR_T_VAL             0x2E

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

#if (BMP085_USE_I2C) || defined(__DOXYGEN__)
/**
 * @brief   Reads registers value using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in]  i2cp      pointer to the I2C interface
 * @param[in]  reg       first sub-register address
 * @param[out] rxbufp    pointer to an output buffer
 * @param[in]  n         number of consecutive register to read
 *
 * @return               the operation status
 * @notapi
 */
msg_t bmp085I2CReadRegister(I2CDriver *i2cp, uint8_t reg, uint8_t *rxbufp,
                            size_t n) {
  uint8_t txbuf = reg;

  return i2cMasterTransmitTimeout(i2cp, BMP085_SAD, &txbuf, 1, rxbufp, n,
                                  TIME_INFINITE);
}

/**
 * @brief   Writes a value into a register using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] i2cp    pointer to the I2C interface
 * @param[in] txbufp  buffer containing sub-address value in first position
 *                    and values to write
 * @param[in] n       size of txbuf less one (not considering the first
 *                       element)
 *
 * @return               the operation status
 * @notapi
 */
msg_t bmp085I2CWriteRegister(I2CDriver *i2cp, uint8_t *txbufp, size_t n) {

  return i2cMasterTransmitTimeout(i2cp, BMP085_SAD, txbufp, n + 1, NULL, 0,
                                  TIME_INFINITE);
}
#endif /* BMP085_USE_I2C */

/**
 * @brief   Read all the calibration data from the BMP085 EEPROM.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] devp	pointer to the BMP085 device driver sensor
 * @param[in] reg   first calibration coefficient register to read
 *
 * @return    msg   the operation status
 */
static msg_t bmp085ReadCoefficient(BMP085Driver *devp, uint8_t reg) {

  uint8_t rxbuf[22];

#if BMP085_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
#endif /* BMP085_SHARED_I2C */

  msg_t msg = bmp085I2CReadRegister(devp->config->i2cp, reg, rxbuf, 22);

#if BMP085_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* BMP085_SHARED_I2C */

  if (msg == MSG_OK) {
    devp->calibrationdata.ac1 = ((rxbuf[ 0] << 8) | rxbuf[ 1]);
    devp->calibrationdata.ac2 = ((rxbuf[ 2] << 8) | rxbuf[ 3]);
    devp->calibrationdata.ac3 = ((rxbuf[ 4] << 8) | rxbuf[ 5]);
    devp->calibrationdata.ac4 = ((rxbuf[ 6] << 8) | rxbuf[ 7]);
    devp->calibrationdata.ac5 = ((rxbuf[ 8] << 8) | rxbuf[ 9]);
    devp->calibrationdata.ac6 = ((rxbuf[10] << 8) | rxbuf[11]);
    devp->calibrationdata.b1  = ((rxbuf[12] << 8) | rxbuf[13]);
    devp->calibrationdata.b2  = ((rxbuf[14] << 8) | rxbuf[15]);
    devp->calibrationdata.mb  = ((rxbuf[16] << 8) | rxbuf[17]);
    devp->calibrationdata.mc  = ((rxbuf[18] << 8) | rxbuf[19]);
    devp->calibrationdata.md  = ((rxbuf[20] << 8) | rxbuf[21]);
  }

  return msg;
}

/**
 * @brief   Calcul the true temperature.
 *
 * @param[in]   devp  pointer to the BMP085 device driver sensor
 * @param[in]   ut    uncompensated temperature
 * @param[out]  ctp   pointer of the compensated temperature
 */
static void calcul_t(BMP085Driver *devp, int32_t ut, float *ctp) {

  int32_t x1, x2;

  /* Converting the temperature value. */
  x1 = ((ut - devp->calibrationdata.ac6) * devp->calibrationdata.ac5) >> 15;
  x2 = (devp->calibrationdata.mc << 11) / (x1 + devp->calibrationdata.md);
  devp->calibrationdata.b5 = x1 + x2;
  *ctp = (float)((devp->calibrationdata.b5 + 8) >> 4)*BMP085_T_RES;
}

/**
 * @brief   Calcul the true pressure.
 *
 * @param[in]   devp  pointer to the BMP085 device driver sensor
 * @param[in]   up    uncompensated pressure
 * @param[in]   oss   over sampling setting
 * @param[out]  cpp   pointer of the compensated pressure
 */
static void calcul_p(BMP085Driver *devp, int32_t up, uint8_t oss, float *cpp) {

  int32_t   press;
  int32_t		x1,x2,x3;
  int32_t		b3,b6;
  uint32_t	b4,b7;

  /* Converting the pressure value. */
  b6 = devp->calibrationdata.b5 - 4000;
  x1 = (devp->calibrationdata.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (devp->calibrationdata.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = ((((int32_t)devp->calibrationdata.ac1 * 4 + x3) << oss) + 2) >> 2;
  x1 = ((devp->calibrationdata.ac3)*b6) >> 13;
  x2 = (devp->calibrationdata.b1 * (b6*b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = devp->calibrationdata.ac4 * (uint32_t)(x3 + 32768) >> 15;
  b7 = ((uint32_t)up - b3)*(50000 >> oss);

  if (b7 < 0x80000000)
    press = (b7*2)/b4;
  else
    press = (b7/b4)*2;

  x1 = (press >> 8)*(press >> 8);
  x1 = (x1*3038) >> 16;
  x2 = (-7357*press) >> 16;
  *cpp =(float)((press + ((x1 + x2 + 3791) >> 4))*BMP085_P_RES);
}

/**
 * @brief   Start temperature measurement.
 *
 * @param[in] devp  pointer to the BMP085 device driver
 *
 * @return          the operation status
 */
static msg_t start_t_measurement(BMP085Driver *devp) {

  uint8_t txbuf[2] = {BMP085_AD_CR, BMP085_CR_T_VAL};

  i2cAcquireBus(devp->config->i2cp);
  msg_t msg = bmp085I2CWriteRegister(devp->config->i2cp, txbuf, 2);
  i2cReleaseBus(devp->config->i2cp);

  /* Conversion time for the temperature. */
  chThdSleepMilliseconds(BMP085_THERMO_CT_LOW);
  //chThdSleepMilliseconds(devp->config.tct); // TODO: use this instead of the top line:

  return msg;
}

/**
 * @brief   Start the pressure measurment.
 *
 * @param[in] devp  pointer to the BMP085 driver
 * @return    msg   the operation status
 */
static msg_t start_p_measurement(BMP085Driver *devp) {

  uint8_t oss, delay;
  uint8_t txbuf[2];

  oss = devp->config->oss;
  txbuf[0] = BMP085_AD_CR;

  /* Check the oss according the bmp085 mode. */
  if (oss == BMP085_BARO_OSS_0) {
    txbuf[1] = BMP085_CR_P_VAL0 + (oss << 6);
    delay = BMP085_BARO_CT_LOW;
  }
  else if (oss == BMP085_BARO_OSS_1) {
    txbuf[1] = BMP085_CR_P_VAL1 + (oss << 6);
    delay = BMP085_BARO_CT_STD;
  }
  else if (oss == BMP085_BARO_OSS_2) {
    txbuf[1] = BMP085_CR_P_VAL2 + (oss << 6);
    delay = BMP085_BARO_CT_HR;
  }
  else {
    txbuf[1] = BMP085_CR_P_VAL3 + (oss << 6);
    delay = BMP085_BARO_CT_LUHR;
  }

  /* Start the sensor for sampling. */
#if BMP085_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
#endif /* BMP085_SHARED_I2C */

  msg_t msg = bmp085I2CWriteRegister(devp->config->i2cp, txbuf, 2);

#if BMP085_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* BMP085_SHARED_I2C */

  /* Conversion time for the pressure, max time for the moment. */
  chThdSleepMilliseconds(delay);

  return msg;
}

/**
 * @brief   Read the uncompensated temperature from the BMP085 register.
 *
 * @param[in]   devp    pointer to the BMP085 driver
 * @param[out]  utemp   uncompensated temperature read from the sensor register
 *
 * @return      msg     the operation status
 */
static msg_t acquire_ut(BMP085Driver *devp, int32_t *utemp) {

  uint8_t rxbuf[2];
  msg_t msg;

  /* Start the temperature measurement. */
  start_t_measurement(devp);

  /* Start the sensor for sampling. */
#if BMP085_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
#endif /* BMP085_SHARED_I2C */

   /* Get the temperature. */
  msg = bmp085I2CReadRegister(devp->config->i2cp, BMP085_AD_T_DR_MSB, rxbuf,
                              2);

#if BMP085_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* BMP085_SHARED_I2C */

  if(msg == MSG_OK){
    /* Building the uncompensated temperature value. */
    *utemp = (int32_t)((rxbuf[0] << 8) | rxbuf[1]);
  }

  return msg;
}

/**
 * @brief   Read the uncompensated pressure from the BMP085 register.
 *
 * @param[in]   devp    pointer to the BMP085 driver
 * @param[out]  upress  uncompensated pressure read from the sensor register
 *
 * @return      msg     the operation status
 */
static msg_t acquire_up(BMP085Driver *devp, int32_t *upress) {

  uint8_t rxbuf[3];
  uint8_t oss;
  msg_t msg;

  /* Get the oversampling setting from the driver configuratioin. */
  oss = devp->config->oss;

  /* Start the pressure measurement. */
  start_p_measurement(devp);

  /* Start the sensor for sampling. */
#if BMP085_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
#endif /* BMP085_SHARED_I2C */

 /* Get the pressure */
  msg = bmp085I2CReadRegister(devp->config->i2cp, BMP085_AD_P_DR_MSB, rxbuf,
                              3);
                              
#if BMP085_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* BMP085_SHARED_I2C */

  if (msg == MSG_OK) {
    /* Building the uncompensated pressure value. */
    *upress = (int32_t)((rxbuf[0] << 16)|(rxbuf[1] << 8)|rxbuf[2]);
    *upress = *upress >> (8-oss);
  }

  return msg;
}

/*==========================================================================*/
/* Interface implementation.                                                */
/*==========================================================================*/

/**
 * @brief   Get the barometer number of axes.
 *
 * @param[in]   ip  interface pointer of the BMP085 sensor
 *
 * @return          barometer number of axes
 */
static size_t baro_get_axes_number(void *ip) {

  osalDbgCheck(ip != NULL);

  return BMP085_BARO_NUMBER_OF_AXES;
}

/**
 * @brief   Get the thermometer number of axes.
 *
 * @param[in]   ip  interface pointer of the BMP085 sensor
 *
 * @return          thermometer number of axes
 */
static size_t thermo_get_axes_number(void *ip) {

  osalDbgCheck(ip != NULL);

  return BMP085_THERMO_NUMBER_OF_AXES;
}

/**
 * @brief   Get the sensor number of axes.
 *
 * @param[in]   ip  interface pointer of the BMP085 sensor
 *
 * @return          sensor number of axes
 */
static size_t sens_get_axes_number(void *ip) {

  osalDbgCheck(ip != NULL);

  return (baro_get_axes_number(ip) + thermo_get_axes_number(ip));
}

/**
 * @brief   Read baromether raw data.
 *
 * @param[in]   ip    interface pointer of the sensor
 * @param[in]   axes  buffer for various axes data
 *
 * @return      msg   the result of the reading operation
 */
static msg_t baro_read_raw(void *ip, int32_t axes[]) {

  osalDbgCheck((ip != NULL) && (axes != NULL));
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY),
                "baro_read_raw(), invalid state");

#if BMP085_USE_I2C
  osalDbgAssert((((BMP085Driver *)ip)->config->i2cp->state == I2C_READY),
      "baro_read_raw(), channel not ready");
#if BMP085_SHARED_I2C
  i2cAcquireBus(((BMP085Driver *)ip)->config->i2cp);
  i2cStart(((BMP085Driver *)ip)->config->i2cp,
           ((BMP085Driver *)ip)->config->i2ccfg);
#endif /* BMP085_SHARED_I2C.              */

  /* Measure the uncompensated pressure.  */
  msg_t msg = acquire_up(((BMP085Driver *)ip), axes);

#if BMP085_SHARED_I2C
  i2cReleaseBus(((BMP085Driver *)ip)->config->i2cp);
#endif /* BMP085_SHARED_I2C.              */
#endif /* BMP085_USE_I2C.                 */

  return msg;
}

/**
 * @brief   Read thermometer raw data.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   axes  buffer for various axes data
 *
 * @return      msg   the result of the reading operation
 */
static msg_t thermo_read_raw(void *ip, int32_t axes[]) {

  osalDbgCheck((ip != NULL) && (axes != NULL));
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY),
                "thermo_read_raw(), invalid state");

#if BMP085_USE_I2C
  osalDbgAssert((((BMP085Driver *)ip)->config->i2cp->state == I2C_READY),
                "thermo_read_raw(), channel not ready");
#if BMP085_SHARED_I2C
  i2cAcquireBus(((BMP085Driver *)ip)->config->i2cp);
  i2cStart(((BMP085Driver *)ip)->config->i2cp,
           ((BMP085Driver *)ip)->config->i2ccfg);
#endif /* BMP085_SHARED_I2C.                */

  /* Measure the uncompensated temperature. */
  msg_t msg = acquire_ut(((BMP085Driver *)ip), axes);

#if BMP085_SHARED_I2C
  i2cReleaseBus(((LSM303DLHCDriver *)ip)->config->i2cp);
#endif /* BMP085_SHARED_I2C.                */
#endif /* BMP085_USE_I2C.                   */

  return msg;
}

/**
 * @brief   Read BMP085 sensor raw data.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   axes  buffer for various axes data
 *
 * @return      msg   the result of the reading operation
 */
static msg_t sens_read_raw(void *ip, int32_t axes[]) {

  int32_t* bp = axes;
  msg_t msg;

  msg = baro_read_raw(ip, bp);

  if (msg != MSG_OK)
    return msg;

  bp += BMP085_BARO_NUMBER_OF_AXES;

  msg = thermo_read_raw(ip, bp);

  return msg;
}

/**
 * @brief   Read barometer cooked data.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   axes  buffer for various axes data
 *
 * @return      msg   the result of the reading operation
 */
static msg_t baro_read_cooked(void *ip, float axes[]) {

  uint32_t i;
  int32_t raw[BMP085_BARO_NUMBER_OF_AXES];
  msg_t msg;
  uint8_t oss;

  osalDbgCheck((ip != NULL) && (axes != NULL));
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY),
                "baro_read_cooked(), invalid state");

  msg = baro_read_raw(ip, raw);
  oss = ((BMP085Driver *)ip)->config->oss;

  for (i = 0; i < BMP085_BARO_NUMBER_OF_AXES; i++)
    calcul_p(ip, raw[i], oss, &axes[i]);

  return msg;
}

/**
 * @brief   Read thermometer cooked data.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   axes  buffer for various axes data
 *
 * @return      msg   the result of the reading operation
 */
static msg_t thermo_read_cooked(void *ip, float axes[]) {

  uint32_t  i;
  int32_t raw[BMP085_THERMO_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck(((ip != NULL) && (axes != NULL)));
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY),
      "thermo_read_cooked(), invalid state");
  msg = thermo_read_raw(ip, raw);

  for (i = 0; i < BMP085_THERMO_NUMBER_OF_AXES; i++)
    calcul_t(ip, raw[i], &axes[i]);

  return msg;
}

/**
 * @brief   Read BMP085 sensor cooked data.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   axes  buffer for various axes data
 *
 * @return      msg   the result of the reading operation
 */
static msg_t sens_read_cooked(void *ip, float axes[]) {

  float* bp = axes;
  msg_t msg;

  msg = baro_read_cooked(ip, bp);

  if (msg != MSG_OK)
    return msg;

  bp += BMP085_BARO_NUMBER_OF_AXES;
  msg = thermo_read_cooked(ip, bp);

  return msg;
}

/**
 * @brief   Set the barometer bias.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   bp    pointer to the bias value
 *
 * @return      msg   the result of the setting operation
 */
static msg_t baro_set_bias(void *ip, float *bp) {

  osalDbgCheck((ip != NULL) && (bp !=NULL));
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY) ||
                (((BMP085Driver *)ip)->state == BMP085_STOP),
                "baro_set_bias(), invalid state");
  return MSG_OK;
}

/**
 * @brief   Set the thermometer bias.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   bp    pointer to the bias value
 *
 * @return      msg   the result of the setting operation
 */
static msg_t thermo_set_bias(void *ip, float *bp) {

  osalDbgCheck((ip != NULL) && (bp !=NULL));
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY) ||
                (((BMP085Driver *)ip)->state == BMP085_STOP),
                "thermo_set_bias(), invalid state");
  return MSG_OK;
}

/**
 * @brief   Reset the barometer bias.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 *
 * @return      msg   the result of the reset operation
 */
static msg_t baro_reset_bias(void *ip) {

  osalDbgCheck(ip != NULL);
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY) ||
                (((BMP085Driver *)ip)->state == BMP085_STOP),
                "baro_reset_bias(), invalid state");

  return MSG_OK;
}

/**
 * @brief   Reset the thermometer bias.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 *
 * @return      msg   the result of the reset operation
 */
static msg_t thermo_reset_bias(void *ip) {

  osalDbgCheck(ip != NULL);
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY) ||
                (((BMP085Driver *)ip)->state == BMP085_STOP),
                "thermo_reset_bias(), invalid state");

  return MSG_OK;
}

/**
 * @brief   Set the barometer sensivity.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   sp    pointer to the sensivity value
 *
 * @return      msg   the result of the setting operation
 */
static msg_t baro_set_sensivity(void *ip, float *sp) {

  osalDbgCheck((ip != NULL) && (sp !=NULL));
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY),
                "baro_set_sensivity(), invalid state");

  return MSG_OK;
}

/**
 * @brief   Set the thermometer sensivity.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 * @param[in]   sp    pointer to the sensivity value
 *
 * @return      msg   the result of the setting operation
 */
static msg_t thermo_set_sensivity(void *ip, float *sp) {

  osalDbgCheck((ip != NULL) && (sp !=NULL));
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY),
                "thermo_set_sensivity(), invalid state");

  return MSG_OK;
}

/**
 * @brief   Reset the barometer sensivity.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 *
 * @return      msg   the result of the reset operation
 */
static msg_t baro_reset_sensivity(void *ip) {

  osalDbgCheck(ip != NULL);
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY),
                "baro_reset_sensivity(), invalid state");

  return MSG_OK;
}

/**
 * @brief   Reset the thermometer sensivity.
 *
 * @param[in]   ip    interface pointer of the BMP085 sensor
 *
 * @return      msg   the result of the reset operation
 */
static msg_t thermo_reset_sensivity(void *ip) {

  osalDbgCheck(ip != NULL);
  osalDbgAssert((((BMP085Driver *)ip)->state == BMP085_READY),
                "thermo_reset_sensivity(), invalid state");

  return MSG_OK;
}

static const struct BaseSensorVMT vmt_basesensor = {
  sens_get_axes_number, sens_read_raw, sens_read_cooked
};

static const struct BaseBarometerVMT vmt_basebarometer = {
  baro_get_axes_number, baro_read_raw, baro_read_cooked,
  baro_set_bias, baro_reset_bias,
  baro_set_sensivity, baro_reset_sensivity
};

static const struct BaseThermometerVMT vmt_basethermometer = {
  thermo_get_axes_number, thermo_read_raw, thermo_read_cooked,
  thermo_set_bias, thermo_reset_bias,
  thermo_set_sensivity, thermo_reset_sensivity
};

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out]  devp  pointer to the @p BMP085Driver object
 *
 * @init
 */
void bmp085ObjectInit(BMP085Driver *devp) {

  devp->vmt_basesensor = &vmt_basesensor;
  devp->vmt_basebarometer = &vmt_basebarometer;
  devp->vmt_basethermometer = &vmt_basethermometer;
  devp->config = NULL;
  devp->state  = BMP085_STOP;
}

/**
 * @brief   Configures and activates BMP085 Complex Driver peripheral.
 *
 * @param[in] devp    pointer to the @p BMP085Driver object
 * @param[in] config  pointer to the @p BMP085Config object
 *
 * @api
 */
void bmp085Start(BMP085Driver *devp, const BMP085Config *config) {

  osalDbgCheck((devp != NULL) && (config != NULL));
  osalDbgAssert((devp->state == BMP085_STOP) ||
                (devp->state == BMP085_READY),
                "bmp085cStart(), invalid state");
  devp->config = config;
#if BMP085_USE_I2C
#if	BMP085_SHARED_I2C
  i2cAcquireBus((devp)->config->i2cp);
#endif  /* BMP085_SHARED_I2C.     */
  /* Read the Calibrations data.  */
  i2cStart((devp)->config->i2cp, (devp)->config->i2ccfg);
  bmp085ReadCoefficient(devp, BMP085_AD_CC_AC1_MSB);
#if	BMP085_SHARED_I2C
  i2cReleaseBus((devp)->config->i2cp);
#endif /* BMP085_SHARED_I2C.      */
#endif /* BMP085_USE_I2C.         */
  if(devp->state != BMP085_READY)
    devp->state = BMP085_READY;
}

/**
 * @brief   Deactivates the BMP085 Complex Driver peripheral.
 *
 * @param[in] devp  pointer to the @p BMP085Driver object
 *
 * @api
 */
void bmp085Stop(BMP085Driver *devp) {

  osalDbgCheck(devp != NULL);
  osalDbgAssert((devp->state == BMP085_STOP) ||
                (devp->state == BMP085_READY),
                "bmp085Stop(), invalid state");
#if (BMP085_USE_I2C)
  if (devp->state == BMP085_STOP) {
#if	BMP085_SHARED_I2C
    i2cAcquireBus((devp)->config->i2cp);
    i2cStart((devp)->config->i2cp, (devp)->config->i2ccfg);
#endif  /* BMP085_SHARED_I2C. */
#if	BMP085_SHARED_I2C
    i2cReleaseBus((devp)->config->i2cp);
#endif  /* BMP085_SHARED_I2C. */
  }
#endif  /* BMP085_USE_I2C.    */
  if (devp->state != BMP085_STOP)
    devp->state = BMP085_STOP;
}
/** @} */
