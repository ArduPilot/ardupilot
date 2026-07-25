/*
    ChibiOS - Copyright (C) 2016..2023 Rocco Marco Guglielmi

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
 * @file    lsm6ds0.c
 * @brief   LSM6DS0 MEMS interface module code.
 *
 * @addtogroup LSM6DS0
 * @ingroup EX_ST
 * @{
 */

#include "hal.h"
#include "lsm6ds0.h"

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

#if (LSM6DS0_USE_I2C) || defined(__DOXYGEN__)
/**
 * @brief   Reads registers value using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 * @note    IF_ADD_INC bit must be 1 in CTRL_REG8
 *
 * @param[in]  i2cp      pointer to the I2C interface
 * @param[in]  sad       slave address without R bit
 * @param[in]  reg       first sub-register address
 * @param[out] rxbuf     pointer to an output buffer
 * @param[in]  n         number of consecutive register to read
 * @return               the operation status.
 * @notapi
 */
msg_t lsm6ds0I2CReadRegister(I2CDriver *i2cp, lsm6ds0_sad_t sad, uint8_t reg,
                             uint8_t* rxbuf, size_t n) {

  return i2cMasterTransmitTimeout(i2cp, sad, &reg, 1, rxbuf, n,
                                  TIME_INFINITE);
}

/**
 * @brief   Writes a value into a register using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] i2cp       pointer to the I2C interface
 * @param[in] sad        slave address without R bit
 * @param[in] txbuf      buffer containing sub-address value in first position
 *                       and values to write
 * @param[in] n          size of txbuf less one (not considering the first
 *                       element)
 * @return               the operation status.
 * @notapi
 */
#define lsm6ds0I2CWriteRegister(i2cp, sad, txbuf, n)                        \
        i2cMasterTransmitTimeout(i2cp, sad, txbuf, n + 1, NULL, 0,          \
                                  TIME_INFINITE)
#endif /* LSM6DS0_USE_I2C */

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              the number of axes.
 */
static size_t acc_get_axes_number(void *ip) {
  (void)ip;

  return LSM6DS0_ACC_NUMBER_OF_AXES;
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
  LSM6DS0Driver* devp;
  uint8_t buff [LSM6DS0_ACC_NUMBER_OF_AXES * 2], i;
  int16_t tmp;
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "acc_read_raw(), invalid state");
#if LSM6DS0_USE_I2C
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "acc_read_raw(), channel not ready");

#if LSM6DS0_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

  msg = lsm6ds0I2CReadRegister(devp->config->i2cp, devp->config->slaveaddress,
                               LSM6DS0_AD_OUT_X_L_XL, buff,
                               LSM6DS0_ACC_NUMBER_OF_AXES * 2);

#if LSM6DS0_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */
  if(msg == MSG_OK)
    for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
      tmp = buff[2 * i] + (buff[2 * i + 1] << 8);
      axes[i] = (int32_t)tmp;
    }
  return msg;
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
  LSM6DS0Driver* devp;
  uint32_t i;
  int32_t raw[LSM6DS0_ACC_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "acc_read_cooked(), invalid state");

  msg = acc_read_raw(ip, raw);
  for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
    axes[i] = (raw[i] * devp->accsensitivity[i]) - devp->accbias[i];
  }
  return msg;
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
  LSM6DS0Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "acc_set_bias(), invalid state");

  for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
    devp->accbias[i] = bp[i];
  }
  return msg;
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
  LSM6DS0Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "acc_reset_bias(), invalid state");

  for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++)
    devp->accbias[i] = LSM6DS0_ACC_BIAS;
  return msg;
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
  LSM6DS0Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseAccelerometer*)ip);

  osalDbgCheck((ip != NULL) && (sp != NULL));

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "acc_set_sensivity(), invalid state");

  for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
    devp->accsensitivity[i] = sp[i];
  }
  return msg;
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
  LSM6DS0Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "acc_reset_sensivity(), invalid state");

  if(devp->config->accfullscale == LSM6DS0_ACC_FS_2G)
    for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LSM6DS0_ACC_SENS_2G;
  else if(devp->config->accfullscale == LSM6DS0_ACC_FS_4G)
	for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LSM6DS0_ACC_SENS_4G;
  else if(devp->config->accfullscale == LSM6DS0_ACC_FS_8G)
	for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LSM6DS0_ACC_SENS_8G;
  else if(devp->config->accfullscale == LSM6DS0_ACC_FS_16G)
	for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LSM6DS0_ACC_SENS_16G;
  else {
    osalDbgAssert(FALSE, "reset_sensivity(), accelerometer full scale issue");
    msg = MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the LSM6DS0Driver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LSM6DS0Driver interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t acc_set_full_scale(LSM6DS0Driver *devp, lsm6ds0_acc_fs_t fs) {
  float newfs, scale;
  uint8_t i, buff[2];
  msg_t msg;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "acc_set_full_scale(), invalid state");
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "acc_set_full_scale(), channel not ready");

  /* Computing new fullscale value.*/
  if(fs == LSM6DS0_ACC_FS_2G) {
    newfs = LSM6DS0_ACC_2G;
    msg = MSG_OK;
  }
  else if(fs == LSM6DS0_ACC_FS_4G) {
    newfs = LSM6DS0_ACC_4G;
    msg = MSG_OK;
  }
  else if(fs == LSM6DS0_ACC_FS_8G) {
    newfs = LSM6DS0_ACC_8G;
    msg = MSG_OK;
  }
  else if(fs == LSM6DS0_ACC_FS_16G) {
    newfs = LSM6DS0_ACC_16G;
    msg = MSG_OK;
  }
  else {
    msg = MSG_RESET;
  }

  if((msg == MSG_OK) &&
     (newfs != devp->accfullscale))  {
    /* Computing scale value.*/
    scale = newfs / devp->accfullscale;
    devp->accfullscale = newfs;

#if LSM6DS0_SHARED_I2C
		i2cAcquireBus(devp->config->i2cp);
		i2cStart(devp->config->i2cp,
						 devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

    /* Updating register.*/
    msg = lsm6ds0I2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DS0_AD_CTRL_REG6_XL, &buff[1], 1);

#if LSM6DS0_SHARED_I2C
        i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */

    if(msg != MSG_OK) {

      buff[1] &= ~(LSM6DS0_CTRL_REG6_XL_FS_MASK);
      buff[1] |= fs;
      buff[0] = LSM6DS0_AD_CTRL_REG6_XL;

#if LSM6DS0_SHARED_I2C
      i2cAcquireBus(devp->config->i2cp);
      i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

      msg = lsm6ds0I2CWriteRegister(devp->config->i2cp,
                                    devp->config->slaveaddress, buff, 1);

#if LSM6DS0_SHARED_I2C
      i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
    }
    if(msg != MSG_OK) {

      /* Scaling sensitivity and bias. Re-calibration is suggested anyway.*/
      for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
        devp->accsensitivity[i] *= scale;
        devp->accbias[i] *= scale;
      }
    }
  }
  return msg;
}

/**
 * @brief   Return the number of axes of the BaseGyroscope.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              the number of axes.
 */
static size_t gyro_get_axes_number(void *ip) {
  (void)ip;

  return LSM6DS0_GYRO_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseGyroscope.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t gyro_read_raw(void *ip, int32_t axes[LSM6DS0_GYRO_NUMBER_OF_AXES]) {
  LSM6DS0Driver* devp;
  int16_t tmp;
  uint8_t i, buff [2 * LSM6DS0_GYRO_NUMBER_OF_AXES];
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "gyro_read_raw(), invalid state");
#if LSM6DS0_USE_I2C
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "gyro_read_raw(), channel not ready");

#if LSM6DS0_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

  msg = lsm6ds0I2CReadRegister(devp->config->i2cp, devp->config->slaveaddress,
                               LSM6DS0_AD_OUT_X_L_G, buff,
                               LSM6DS0_GYRO_NUMBER_OF_AXES * 2);

#if	LSM6DS0_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */

    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
      tmp = buff[2 * i] + (buff[2 * i + 1] << 8);
      axes[i] = (int32_t)tmp;
    }
  return msg;
}

/**
 * @brief   Retrieves cooked data from the BaseGyroscope.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as DPS.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t gyro_read_cooked(void *ip, float axes[]) {
  LSM6DS0Driver* devp;
  uint32_t i;
  int32_t raw[LSM6DS0_GYRO_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "gyro_read_cooked(), invalid state");

  msg = gyro_read_raw(ip, raw);
  for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++){
    axes[i] = (raw[i] * devp->gyrosensitivity[i]) - devp->gyrobias[i];
  }
  return msg;
}

/**
 * @brief   Samples bias values for the BaseGyroscope.
 * @note    The LSM6DS0 shall not be moved during the whole procedure.
 * @note    After this function internal bias is automatically updated.
 * @note    The behavior of this function depends on @p LSM6DS0_BIAS_ACQ_TIMES
 *          and @p LSM6DS0_BIAS_SETTLING_US.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_sample_bias(void *ip) {
  LSM6DS0Driver* devp;
  uint32_t i, j;
  int32_t raw[LSM6DS0_GYRO_NUMBER_OF_AXES];
  int32_t buff[LSM6DS0_GYRO_NUMBER_OF_AXES] = {0, 0, 0};
  msg_t msg;
	
  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "gyro_sample_bias(), invalid state");
#if LSM6DS0_USE_I2C
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "gyro_sample_bias(), channel not ready");
#endif

  for(i = 0; i < LSM6DS0_GYRO_BIAS_ACQ_TIMES; i++){
    msg = gyro_read_raw(ip, raw);
		if(msg != MSG_OK)
			return msg;
    for(j = 0; j < LSM6DS0_GYRO_NUMBER_OF_AXES; j++){
      buff[j] += raw[j];
    }
    osalThreadSleepMicroseconds(LSM6DS0_GYRO_BIAS_SETTLING_US);
  }

  for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++){
    devp->gyrobias[i] = (buff[i] / LSM6DS0_GYRO_BIAS_ACQ_TIMES);
    devp->gyrobias[i] *= devp->gyrosensitivity[i];
  }
  return msg;
}

/**
 * @brief   Set bias values for the BaseGyroscope.
 * @note    Bias must be expressed as DPS.
 * @note    The bias buffer must be at least the same size of the BaseGyroscope
 *          axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_set_bias(void *ip, float *bp) {
  LSM6DS0Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "gyro_set_bias(), invalid state");

  for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
    devp->gyrobias[i] = bp[i];
  }
  return msg;
}

/**
 * @brief   Reset bias values for the BaseGyroscope.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_reset_bias(void *ip) {
  LSM6DS0Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "gyro_reset_bias(), invalid state");

  for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++)
    devp->gyrobias[i] = LSM6DS0_GYRO_BIAS;
  return msg;
}

/**
 * @brief   Set sensitivity values for the BaseGyroscope.
 * @note    Sensitivity must be expressed as DPS/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t gyro_set_sensivity(void *ip, float *sp) {
  LSM6DS0Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (sp !=NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "gyro_set_sensivity(), invalid state");

  for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
    devp->gyrosensitivity[i] = sp[i];
  }
  return msg;
}

/**
 * @brief   Reset sensitivity values for the BaseGyroscope.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseGyroscope interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t gyro_reset_sensivity(void *ip) {
  LSM6DS0Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LSM6DS0Driver*, (BaseGyroscope*)ip);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "gyro_reset_sensivity(), invalid state");

  if(devp->config->gyrofullscale == LSM6DS0_GYRO_FS_245DPS)
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = LSM6DS0_GYRO_SENS_245DPS;
  else if(devp->config->gyrofullscale == LSM6DS0_GYRO_FS_500DPS)
	for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = LSM6DS0_GYRO_SENS_500DPS;
  else if(devp->config->gyrofullscale == LSM6DS0_GYRO_FS_2000DPS)
	for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrosensitivity[i] = LSM6DS0_GYRO_SENS_2000DPS;
  else {
    osalDbgAssert(FALSE, "gyro_reset_sensivity(), full scale issue");
    return MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the LSM6DS0Driver gyroscope fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p BaseGyroscope interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t gyro_set_full_scale(LSM6DS0Driver *devp, lsm6ds0_gyro_fs_t fs) {
  float newfs, scale;
  uint8_t i, buff[2];
  msg_t msg = MSG_OK;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LSM6DS0_READY),
                "gyro_set_full_scale(), invalid state");
#if LSM6DS0_USE_I2C
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "gyro_set_full_scale(), channel not ready");
#endif

  if(fs == LSM6DS0_GYRO_FS_245DPS) {
    newfs = LSM6DS0_GYRO_245DPS;
  }
  else if(fs == LSM6DS0_GYRO_FS_500DPS) {
    newfs = LSM6DS0_GYRO_500DPS;
  }
  else if(fs == LSM6DS0_GYRO_FS_2000DPS) {
    newfs = LSM6DS0_GYRO_2000DPS;
  }
  else {
    return MSG_RESET;
  }

  if(newfs != devp->gyrofullscale) {
    scale = newfs / devp->gyrofullscale;
    devp->gyrofullscale = newfs;

#if LSM6DS0_USE_I2C
#if	LSM6DS0_SHARED_I2C
		i2cAcquireBus(devp->config->i2cp);
		i2cStart(devp->config->i2cp,
						 devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

    /* Updating register.*/
    msg = lsm6ds0I2CReadRegister(devp->config->i2cp,
                                 devp->config->slaveaddress,
                                 LSM6DS0_AD_CTRL_REG1_G, &buff[1], 1);

#if	LSM6DS0_SHARED_I2C
		i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */

    buff[1] &= ~(LSM6DS0_CTRL_REG1_G_FS_MASK);
    buff[1] |= fs;
    buff[0] = LSM6DS0_AD_CTRL_REG1_G;

#if LSM6DS0_USE_I2C
#if	LSM6DS0_SHARED_I2C
		i2cAcquireBus(devp->config->i2cp);
		i2cStart(devp->config->i2cp,
						 devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

    lsm6ds0I2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                            buff, 1);

#if	LSM6DS0_SHARED_I2C
		i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */

    /* Scaling sensitivity and bias. Re-calibration is suggested anyway. */
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
      devp->gyrosensitivity[i] *= scale;
      devp->gyrobias[i] *= scale;
    }
  }
  return msg;
}

static const struct LSM6DS0VMT vmt_device = {
  (size_t)0,
  acc_set_full_scale, gyro_set_full_scale
};

static const struct BaseAccelerometerVMT vmt_accelerometer = {
  sizeof(struct LSM6DS0VMT*),
  acc_get_axes_number, acc_read_raw, acc_read_cooked,
  acc_set_bias, acc_reset_bias, acc_set_sensivity, acc_reset_sensivity
};

static const struct BaseGyroscopeVMT vmt_gyroscope = {
  sizeof(struct LSM6DS0VMT*) + sizeof(BaseAccelerometer),
  gyro_get_axes_number, gyro_read_raw, gyro_read_cooked,
  gyro_sample_bias, gyro_set_bias, gyro_reset_bias,
  gyro_set_sensivity, gyro_reset_sensivity
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] devp     pointer to the @p LSM6DS0Driver object
 *
 * @init
 */
void lsm6ds0ObjectInit(LSM6DS0Driver *devp) {
  devp->vmt = &vmt_device;
  devp->acc_if.vmt = &vmt_accelerometer;
  devp->gyro_if.vmt = &vmt_gyroscope;

  devp->config = NULL;

  devp->accaxes = LSM6DS0_ACC_NUMBER_OF_AXES;
  devp->gyroaxes = LSM6DS0_GYRO_NUMBER_OF_AXES;

  devp->state = LSM6DS0_STOP;
}

/**
 * @brief   Configures and activates LSM6DS0 Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p LSM6DS0Driver object
 * @param[in] config    pointer to the @p LSM6DS0Config object
 *
 * @api
 */
void lsm6ds0Start(LSM6DS0Driver *devp, const LSM6DS0Config *config) {
  uint32_t i;
  uint8_t cr[5];
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == LSM6DS0_STOP) ||
                (devp->state == LSM6DS0_READY),
                "lsm6ds0Start(), invalid state");

  devp->config = config;

  /* Configuring common registers.*/

  /* Control register 8 configuration block.*/
  {
    cr[0] = LSM6DS0_AD_CTRL_REG8;
    cr[1] = LSM6DS0_CTRL_REG8_IF_ADD_INC;
#if LSM6DS0_USE_ADVANCED || defined(__DOXYGEN__)
    cr[1] |= devp->config->endianness | devp->config->bdu;
#endif
  }
#if LSM6DS0_USE_I2C
#if LSM6DS0_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */

  i2cStart(devp->config->i2cp, devp->config->i2ccfg);
  lsm6ds0I2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                          cr, 1);

#if LSM6DS0_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */

  /* Configuring Accelerometer subsystem.*/
  /* Multiple write starting address.*/
  cr[0] = LSM6DS0_AD_CTRL_REG5_XL;
  /* Control register 5 configuration block.*/
  {
      cr[1] = LSM6DS0_CTRL_REG5_XL_XEN_XL | LSM6DS0_CTRL_REG5_XL_YEN_XL |
              LSM6DS0_CTRL_REG5_XL_ZEN_XL;
#if LSM6DS0_USE_ADVANCED || defined(__DOXYGEN__)
      cr[1] |= devp->config->accdecmode;
#endif
  }

  /* Control register 6 configuration block.*/
  {
    cr[2] = devp->config->accodr |
            devp->config->accfullscale;
  }

#if LSM6DS0_USE_I2C
#if LSM6DS0_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

  lsm6ds0I2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                          cr, 2);

#if LSM6DS0_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */

  /* Storing sensitivity according to user settings */
  if(devp->config->accfullscale == LSM6DS0_ACC_FS_2G) {
    for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
     if(devp->config->accsensitivity == NULL)
       devp->accsensitivity[i] = LSM6DS0_ACC_SENS_2G;
     else
       devp->accsensitivity[i] = devp->config->accsensitivity[i];
    }
    devp->accfullscale = LSM6DS0_ACC_2G;
  }
  else if(devp->config->accfullscale == LSM6DS0_ACC_FS_4G) {
   for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
     if(devp->config->accsensitivity == NULL)
       devp->accsensitivity[i] = LSM6DS0_ACC_SENS_4G;
     else
       devp->accsensitivity[i] = devp->config->accsensitivity[i];
    }
   devp->accfullscale = LSM6DS0_ACC_4G;
  }
  else if(devp->config->accfullscale == LSM6DS0_ACC_FS_8G) {
   for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
     if(devp->config->accsensitivity == NULL)
       devp->accsensitivity[i] = LSM6DS0_ACC_SENS_8G;
     else
       devp->accsensitivity[i] = devp->config->accsensitivity[i];
    }
   devp->accfullscale = LSM6DS0_ACC_8G;
  }
  else if(devp->config->accfullscale == LSM6DS0_ACC_FS_16G) {
    for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++) {
      if(devp->config->accsensitivity == NULL)
        devp->accsensitivity[i] = LSM6DS0_ACC_SENS_16G;
      else
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
    }
    devp->accfullscale = LSM6DS0_ACC_16G;
  }
  else
    osalDbgAssert(FALSE, "lsm6ds0Start(), accelerometer full scale issue");

  /* Storing bias information */
  if(devp->config->accbias != NULL)
    for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = devp->config->accbias[i];
  else
    for(i = 0; i < LSM6DS0_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = LSM6DS0_ACC_BIAS;

  /* Configuring Gyroscope subsystem.*/
  /* Multiple write starting address.*/
  cr[0] = LSM6DS0_AD_CTRL_REG1_G;

  /* Control register 1 configuration block.*/
  {
    cr[1] = devp->config->gyrofullscale |
            devp->config->gyroodr;
  }

  /* Control register 2 configuration block.*/
  {
    cr[2] = 0;
#if LSM6DS0_USE_ADVANCED || defined(__DOXYGEN__)
    cr[2] |= devp->config->gyrooutsel;
#endif
  }

  /* Control register 3 configuration block.*/
  {
    cr[3] = 0;
#if LSM6DS0_USE_ADVANCED || defined(__DOXYGEN__)
    cr[3] |= devp->config->gyrohpfenable |
             devp->config->gyrolowmodecfg |
             devp->config->gyrohpcfg;
#endif
  }

  /* Control register 4 configuration block.*/
  {
    cr[4] = LSM6DS0_CTRL_REG4_XEN_G | LSM6DS0_CTRL_REG4_YEN_G |
            LSM6DS0_CTRL_REG4_ZEN_G;
  }
#if LSM6DS0_USE_I2C
#if LSM6DS0_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

  lsm6ds0I2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                          cr, 4);

#if LSM6DS0_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */

  cr[0] = LSM6DS0_AD_CTRL_REG9;
  /* Control register 9 configuration block.*/
  {
      cr[1] = 0;
  }
#if LSM6DS0_USE_I2C
#if LSM6DS0_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

  lsm6ds0I2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                          cr, 1);

#if LSM6DS0_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */

  if(devp->config->gyrofullscale == LSM6DS0_GYRO_FS_245DPS) {
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
      if(devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = LSM6DS0_GYRO_SENS_245DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
    devp->gyrofullscale = LSM6DS0_GYRO_245DPS;
  }
  else if(devp->config->gyrofullscale == LSM6DS0_GYRO_FS_500DPS) {
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
      if(devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = LSM6DS0_GYRO_SENS_500DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
    devp->gyrofullscale = LSM6DS0_GYRO_500DPS;
  }
  else if(devp->config->gyrofullscale == LSM6DS0_GYRO_FS_2000DPS) {
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++) {
      if(devp->config->gyrosensitivity == NULL)
        devp->gyrosensitivity[i] = LSM6DS0_GYRO_SENS_2000DPS;
      else
        devp->gyrosensitivity[i] = devp->config->gyrosensitivity[i];
    }
    devp->gyrofullscale = LSM6DS0_GYRO_2000DPS;
  }
  else
    osalDbgAssert(FALSE, "lsm6ds0Start(), gyroscope full scale issue");

  /* Storing bias information */
  if(devp->config->gyrobias != NULL)
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrobias[i] = devp->config->gyrobias[i];
  else
    for(i = 0; i < LSM6DS0_GYRO_NUMBER_OF_AXES; i++)
      devp->gyrobias[i] = LSM6DS0_GYRO_BIAS;

  /* This is the MEMS transient recovery time */
  osalThreadSleepMilliseconds(5);

  devp->state = LSM6DS0_READY;
}

/**
 * @brief   Deactivates the LSM6DS0 Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p LSM6DS0Driver object
 *
 * @api
 */
void lsm6ds0Stop(LSM6DS0Driver *devp) {
  uint8_t cr[2];

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LSM6DS0_STOP) || (devp->state == LSM6DS0_READY),
                "lsm6ds0Stop(), invalid state");

  if (devp->state == LSM6DS0_READY) {
#if LSM6DS0_USE_I2C
#if LSM6DS0_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LSM6DS0_SHARED_I2C */

    /* Disabling accelerometer.*/
    cr[0] = LSM6DS0_AD_CTRL_REG6_XL;
    cr[1] = 0;
    lsm6ds0I2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                            cr, 1);

    /* Disabling gyroscope.*/
    cr[0] = LSM6DS0_AD_CTRL_REG9;
    cr[1] = LSM6DS0_CTRL_REG9_SLEEP_G;
    lsm6ds0I2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                            cr, 1);

    i2cStop(devp->config->i2cp);
#if LSM6DS0_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
#endif /* LSM6DS0_SHARED_I2C */
#endif /* LSM6DS0_USE_I2C */
  }
  devp->state = LSM6DS0_STOP;
}
/** @} */
