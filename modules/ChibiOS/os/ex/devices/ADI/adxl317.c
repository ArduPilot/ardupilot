/*
    ChibiOS - Copyright (C) 2016..2023 Simona Di Domenico

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
 * @file    adxl317.c
 * @brief   ADXL317 MEMS interface module code.
 *
 * @addtogroup ADXL317
 * @ingroup EX_ADI
 * @{
 */

#include "hal.h"
#include "adxl317.h"

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

#if (ADXL317_USE_I2C) || defined(__DOXYGEN__)
/**
 * @brief   Reads a generic register value using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] devp      pointer to @p ADXL317Driver interface.
 * @param[in] reg       starting register address
 * @param[in] b         pointer to an output buffer
 * @param[in] n         number of consecutive registers to read.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t adxl317I2CReadRegister(ADXL317Driver *devp, uint8_t reg,
                                    uint8_t *b, size_t n) {
  msg_t ret = i2cMasterTransmitTimeout(devp->config->i2cp, ADXL317_SAD_GND,
                                       &reg, 1, b, n, TIME_INFINITE);

  return ret;
}

/**
 * @brief   Writes a value into a generic register using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] devp      pointer to @p ADXL317Driver interface.
 * @param[in] reg       starting register address
 * @param[in] b         pointer to a buffer of values
 * @param[in] n         number of adjacent registers to write.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t adxl317I2CWriteRegister(ADXL317Driver *devp, uint8_t reg,
                                     uint8_t *b, size_t n) {
  msg_t ret = MSG_OK;
  uint8_t txbuf[ADXL317_MAX_BUFF_SIZE + 1];
  uint8_t i;

  if (n > ADXL317_MAX_BUFF_SIZE) {
    n = ADXL317_MAX_BUFF_SIZE;
  }

  txbuf [0] = reg;
  for(i = 0; i < n; i++, b++) {
    txbuf[i + 1] = *b;
  }

  ret = i2cMasterTransmitTimeout(devp->config->i2cp, ADXL317_SAD_GND,
                                 txbuf, n + 1, NULL, 0, TIME_INFINITE);

  return ret;
}
#endif /* ADXL317_USE_I2C*/

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              the number of axes.
 */
static size_t acc_get_axes_number(void *ip) {
  (void)ip;

  return ADXL317_ACC_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t acc_read_raw(void *ip, int32_t axes[]) {
  ADXL317Driver* devp;
  uint8_t buff [ADXL317_ACC_NUMBER_OF_AXES * 2], i;
  int32_t tmp;
  msg_t ret = MSG_OK;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL317Driver*, (BaseAccelerometer*)ip);
  osalDbgAssert((devp->state == ADXL317_READY),
                "acc_read_raw(), invalid state");

#if ADXL317_USE_I2C
  ret = adxl317I2CReadRegister(devp, ADXL317_AD_X_DATA_LO, buff,
                               ADXL317_ACC_NUMBER_OF_AXES * 2);

  if(ret == MSG_OK) {
    for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++) {
      tmp = buff[2 * i] >>2 | (buff[2 * i + 1] << 6);
      if(tmp & 0x2000) {
        tmp |= 0xFFFFC000;
      }
      axes[i] = tmp;
    }
  }
#endif /* ADXL317_USE_I2C*/

  return ret;
}

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t acc_read_cooked(void *ip, float axes[]) {
  ADXL317Driver* devp;
  uint32_t i;
  int32_t raw[ADXL317_ACC_NUMBER_OF_AXES];
  msg_t ret = MSG_OK;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL317Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == ADXL317_READY),
                "acc_read_cooked(), invalid state");

  ret = acc_read_raw(ip, raw);

  if(ret == MSG_OK) {
    for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++) {
      axes[i] = ((float)raw[i] * devp->accsensitivity[i]) - devp->accbias[i];
    }
  }

  return ret;
}

/**
 * @brief   Set bias values for the BaseAccelerometer.
 * @note    Bias must be expressed as milli-G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t acc_set_bias(void *ip, float *bp) {
  ADXL317Driver* devp;
  uint32_t i;
  msg_t ret = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL317Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == ADXL317_READY),
                "acc_set_bias(), invalid state");

  for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++) {
    devp->accbias[i] = bp[i];
  }
  return ret;
}

/**
 * @brief   Reset bias values for the BaseAccelerometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t acc_reset_bias(void *ip) {
  ADXL317Driver* devp;
  uint32_t i;
  msg_t ret = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL317Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == ADXL317_READY),
                "acc_reset_bias(), invalid state");

  for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++)
    devp->accbias[i] = ADXL317_ACC_BIAS;
  return ret;
}

/**
 * @brief   Set sensitivity values for the BaseAccelerometer.
 * @note    Sensitivity must be expressed as milli-G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t acc_set_sensivity(void *ip, float *sp) {
  ADXL317Driver* devp;
  uint32_t i;
  msg_t ret = MSG_OK;

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL317Driver*, (BaseAccelerometer*)ip);

  osalDbgCheck((ip != NULL) && (sp != NULL));

  osalDbgAssert((devp->state == ADXL317_READY),
                "acc_set_sensivity(), invalid state");

  for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++) {
    devp->accsensitivity[i] = sp[i];
  }
  return ret;
}

/**
 * @brief   Reset sensitivity values for the BaseAccelerometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t acc_reset_sensivity(void *ip) {
  ADXL317Driver* devp;
  uint32_t i;
  msg_t ret = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL317Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == ADXL317_READY),
                "acc_reset_sensivity(), invalid state");

  for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++)
    devp->accbias[i] = ADXL317_ACC_SENS;
  return ret;
}

static const struct ADXL317VMT vmt_device = {
  (size_t)0
};

static const struct BaseAccelerometerVMT vmt_accelerometer = {
  sizeof(struct ADXL317VMT*),
  acc_get_axes_number, acc_read_raw, acc_read_cooked,
  acc_set_bias, acc_reset_bias, acc_set_sensivity, acc_reset_sensivity
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 * @details The buffer should be at least large @p ADXL317_COMM_BUFF_SIZE.
 * @note    The communication buffer could be used by DMAs.
 *
 * @param[out] devp     pointer to the @p ADXL317Driver object
 * @param[in] txbp      pointer to a buffer used as communication tx buffer
 * @param[in] rxbp      pointer to a buffer used as communication rx buffer
 *
 * @init
 */
void adxl317ObjectInit(ADXL317Driver *devp, uint8_t* txbp, uint8_t* rxbp) {
  devp->vmt = &vmt_device;
  devp->acc_if.vmt = &vmt_accelerometer;

  devp->config = NULL;
  devp->commtxp = txbp;
  devp->commrxp = rxbp;

  devp->accaxes = ADXL317_ACC_NUMBER_OF_AXES;

  devp->state = ADXL317_STOP;
}

/**
 * @brief   Configures and activates ADXL317 Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p ADXL317Driver object
 * @param[in] config    pointer to the @p ADXL317Config object
 *
 * @api
 */
msg_t adxl317Start(ADXL317Driver *devp, const ADXL317Config *config) {
  msg_t ret = MSG_OK;
  uint32_t i;
  uint8_t reg_val;

  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == ADXL317_STOP) ||
                (devp->state == ADXL317_READY),
                "adxl317Start(), invalid state");

  devp->config = config;

  /* Starting the I2C driver.*/
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);

  /* Checking the device ID.*/
  ret = adxl317I2CReadRegister(devp, ADXL317_AD_DEVID_0, &reg_val, 1);
  osalDbgAssert((reg_val == ADXL317_DEVID_0), "Invalid MEMS ID");

  /* Enabling the write on the ADXL317 registers.*/
  if(ret == MSG_OK) {
    reg_val = ADXL317_USER_REG_KEY_0;
    ret = adxl317I2CWriteRegister(devp, ADXL317_AD_USER_REG_KEY, &reg_val, 1);
  }

  if(ret == MSG_OK) {
    reg_val = ADXL317_USER_REG_KEY_1;
    ret = adxl317I2CWriteRegister(devp, ADXL317_AD_USER_REG_KEY, &reg_val, 1);
  }

  /* Filter register configuration block.*/
  if(ret == MSG_OK) {
    /* Filter register configuration for X axis.*/
    reg_val = ADXL317_DEFAULT_FILTER;
#if ADXL317_USE_ADVANCED || defined(__DOXYGEN__)
    reg_val = devp->config->acclowpass.x | devp->config->acchighpass.x;
#endif
#if ADXL317_USE_I2C
    ret = adxl317I2CWriteRegister(devp, ADXL317_AD_X_FILT, &reg_val, 1);
#endif /* ADXL317_USE_ADVANCED*/
  }

  if(ret == MSG_OK) {
    /* Filter register configuration for Y axis.*/
    reg_val = ADXL317_DEFAULT_FILTER;
#if ADXL317_USE_ADVANCED || defined(__DOXYGEN__)
    reg_val = devp->config->acclowpass.y | devp->config->acchighpass.y;
#endif
#if ADXL317_USE_I2C
    ret = adxl317I2CWriteRegister(devp, ADXL317_AD_Y_FILT, &reg_val, 1);
#endif /* ADXL317_USE_ADVANCED*/
  }

  if(ret == MSG_OK) {
    /* Filter register configuration for Z axis.*/
    reg_val = ADXL317_DEFAULT_FILTER;
#if ADXL317_USE_ADVANCED || defined(__DOXYGEN__)
    reg_val = devp->config->acclowpass.z | devp->config->acchighpass.z;
#endif
#if ADXL317_USE_I2C
    ret = adxl317I2CWriteRegister(devp, ADXL317_AD_Z_FILT, &reg_val, 1);
#endif /* ADXL317_USE_ADVANCED*/
  }

  if(ret == MSG_OK) {
    /* Storing sensitivity information according to user setting*/
    if(devp->config->accsensitivity != NULL)
      for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
    else
      for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = ADXL317_ACC_SENS;

    /* Storing bias information according to user setting*/
    if(devp->config->accbias != NULL)
      for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++)
        devp->accbias[i] = devp->config->accbias[i];
    else
      for(i = 0; i < ADXL317_ACC_NUMBER_OF_AXES; i++)
        devp->accbias[i] = ADXL317_ACC_BIAS;

    /* This is the Accelerometer transient recovery time*/
    osalThreadSleepMilliseconds(10);

    devp->state = ADXL317_READY;
  }
  else {
    devp->state = ADXL317_STOP;
  }
  return ret;
}

/**
 * @brief   Deactivates the ADXL317 Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p ADXL317Driver object
 *
 * @api
 */
void adxl317Stop(ADXL317Driver *devp) {
  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == ADXL317_STOP) ||
                (devp->state == ADXL317_READY),
                "adxl317Stop(), invalid state");

  if (devp->state == ADXL317_READY) {
#if (ADXL317_USE_I2C)
    /* Stopping the I2C driver.*/
    i2cStop(devp->config->i2cp);
#endif /* ADXL317_USE_I2C*/
  }
  devp->state = ADXL317_STOP;
}
/** @} */
