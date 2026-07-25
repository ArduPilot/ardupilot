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
 * @file    adxl355.c
 * @brief   ADXL355 MEMS interface module code.
 *
 * @addtogroup ADXL355
 * @ingroup EX_ADI
 * @{
 */

#include "hal.h"
#include "adxl355.h"
#include "string.h"

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

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              the number of axes.
 */
static size_t acc_get_axes_number(void *ip) {
  (void)ip;

  return ADXL355_ACC_NUMBER_OF_AXES;
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
  ADXL355Driver* devp;
  uint8_t buff [ADXL355_ACC_NUMBER_OF_AXES * 3], i;
  int32_t tmp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL355Driver*, (BaseAccelerometer*)ip);
  osalDbgAssert((devp->state == ADXL355_READY),
                "acc_read_raw(), invalid state");

#if ADXL355_USE_SPI
#if	ADXL355_SHARED_SPI
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "acc_read_raw(), channel not ready");

  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip,
           devp->config->spicfg);
#endif /* ADXL355_SHARED_SPI */

  adxl355SPIReadRegister(devp, ADXL355_AD_XDATA3,
                         ADXL355_ACC_NUMBER_OF_AXES * 3, buff);

#if	ADXL355_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */
#endif /* ADXL355_USE_SPI */

  for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++) {
    tmp = (buff[3 * i] << 12) | (buff[3 * i + 1] << 4) | (buff[3 * i + 2] >> 4);
    if(tmp & 0x80000) {
      tmp |= 0xFFF00000U;
    }
    axes[i] = tmp;
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
  ADXL355Driver* devp;
  uint32_t i;
  int32_t raw[ADXL355_ACC_NUMBER_OF_AXES];
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL355Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == ADXL355_READY),
                "acc_read_cooked(), invalid state");

  msg = acc_read_raw(ip, raw);
  for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++) {
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
  ADXL355Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL355Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == ADXL355_READY),
                "acc_set_bias(), invalid state");

  for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++) {
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
  ADXL355Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL355Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == ADXL355_READY),
                "acc_reset_bias(), invalid state");

  for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
    devp->accbias[i] = ADXL355_ACC_BIAS;
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
  ADXL355Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL355Driver*, (BaseAccelerometer*)ip);

  osalDbgCheck((ip != NULL) && (sp != NULL));

  osalDbgAssert((devp->state == ADXL355_READY),
                "acc_set_sensivity(), invalid state");

  for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++) {
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
  ADXL355Driver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(ADXL355Driver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == ADXL355_READY),
                "acc_reset_sensivity(), invalid state");

  if(devp->config->accfullscale == ADXL355_ACC_FS_2G)
    for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = ADXL355_ACC_SENS_2G;
  else if(devp->config->accfullscale == ADXL355_ACC_FS_4G)
    for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = ADXL355_ACC_SENS_4G;
  else if(devp->config->accfullscale == ADXL355_ACC_FS_8G)
    for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = ADXL355_ACC_SENS_8G;
  else {
    osalDbgAssert(FALSE,
                  "acc_reset_sensivity(), accelerometer full scale issue");
    return MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the ADXL355Driver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p ADXL355Driver interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t acc_set_full_scale(ADXL355Driver *devp, adxl355_acc_fs_t fs) {
  float newfs, scale;
  uint8_t i, reg_val;
  msg_t msg = MSG_OK;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == ADXL355_READY),
                "acc_set_full_scale(), invalid state");
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "acc_set_full_scale(), channel not ready");

  /* Computing new fullscale value.*/
  if(fs == ADXL355_ACC_FS_2G) {
    newfs = ADXL355_ACC_2G;
    msg = MSG_OK;
  }
  else if(fs == ADXL355_ACC_FS_4G) {
    newfs = ADXL355_ACC_4G;
    msg = MSG_OK;
  }
  else if(fs == ADXL355_ACC_FS_8G) {
    newfs = ADXL355_ACC_8G;
    msg = MSG_OK;
  }
  else {
    msg = MSG_RESET;
  }

  if((msg == MSG_OK) &&
     (newfs != devp->accfullscale)) {
    /* Computing scale value.*/
    scale = newfs / devp->accfullscale;
    devp->accfullscale = newfs;

#if ADXL355_USE_SPI
#if ADXL355_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* ADXL355_SHARED_SPI */

    /* Getting data from register.*/
    adxl355SPIReadRegister(devp, ADXL355_AD_RANGE, 1, &reg_val);

#if ADXL355_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */
#endif /* ADXL355_USE_SPI */

    reg_val &= ~(ADXL355_RANGE_RANGE_MASK);
    reg_val |= fs;

#if ADXL355_USE_SPI
#if ADXL355_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* ADXL355_SHARED_SPI */

    /* Getting data from register.*/
    adxl355SPIWriteRegister(devp, ADXL355_AD_RANGE, 1, &reg_val);

#if ADXL355_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */
#endif /* ADXL355_USE_SPI */

    /* Scaling sensitivity and bias. Re-calibration is suggested anyway. */
    for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++) {
      devp->accsensitivity[i] *= scale;
      devp->accbias[i] *= scale;
    }
  }
  return msg;
}

static const struct ADXL355VMT vmt_device = {
  (size_t)0,
  acc_set_full_scale
};

static const struct BaseAccelerometerVMT vmt_accelerometer = {
  sizeof(struct ADXL355VMT*),
  acc_get_axes_number, acc_read_raw, acc_read_cooked,
  acc_set_bias, acc_reset_bias, acc_set_sensivity, acc_reset_sensivity
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

#if (ADXL355_USE_SPI) || defined(__DOXYGEN__)
/**
 * @brief   Reads a generic register value using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] devp      pointer to @p ADXL355Driver interface.
 * @param[in] reg       starting register address
 * @param[in] n         number of consecutive registers to read
 * @param[in] b         pointer to an output buffer.
 */
void adxl355SPIReadRegister(ADXL355Driver *devp, uint8_t reg, size_t n,
                            uint8_t* b) {
  
  osalDbgCheck(n < ADXL355_COMM_BUFF_SIZE);
  
  /* Preparing a read. */
  devp->commtxp[0] = (reg << 1) | ADXL355_RW;
  
  /* Reading. */
  cacheBufferFlush(&devp->commtxp[0], sizeof(devp->commtxp));
  spiSelect(devp->config->spip);
  spiSend(devp->config->spip, 1, devp->commtxp);
  spiReceive(devp->config->spip, n, devp->commrxp);
  spiUnselect(devp->config->spip);
  cacheBufferInvalidate(&devp->commrxp[0], sizeof(devp->commrxp));
  
  /* Copying the data in the final buffer. */
  memcpy(b, devp->commrxp, n);
}

/**
 * @brief   Writes a value into a generic register using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] devp      pointer to @p ADXL355Driver interface.
 * @param[in] reg       starting register address
 * @param[in] n         number of adjacent registers to write
 * @param[in] b         pointer to a buffer of values.
 */
void adxl355SPIWriteRegister(ADXL355Driver *devp, uint8_t reg, size_t n,
                             uint8_t* b) {
  unsigned i;
  
  osalDbgCheck(n < ADXL355_COMM_BUFF_SIZE);
  
  /* Preparing a write. */
  devp->commtxp[0] = (reg << 1); 
  for(i = 0; i < n; i++, b++) {
    devp->commtxp[i + 1] = *b;
  }
  
  /* Writing. */
  cacheBufferFlush(&devp->commtxp[0], sizeof(devp->commtxp));
  spiSelect(devp->config->spip);
  spiSend(devp->config->spip, n + 1, devp->commtxp);
  spiUnselect(devp->config->spip);
}
#endif /* ADXL355_USE_SPI */

/**
 * @brief   Initializes an instance.
 * @details The buffer should be at least large @p ADXL355_COMM_BUFF_SIZE.
 * @note    The communication buffer could be used by DMAs.
 *
 * @param[out] devp     pointer to the @p ADXL355Driver object
 * @param[in] txbp      pointer to a buffer used as communication tx buffer
 * @param[in] rxbp      pointer to a buffer used as communication rx buffer
 *
 * @init
 */
void adxl355ObjectInit(ADXL355Driver *devp, uint8_t* txbp, uint8_t* rxbp) {
  devp->vmt = &vmt_device;
  devp->acc_if.vmt = &vmt_accelerometer;

  devp->config = NULL;
  devp->commtxp = txbp;
  devp->commrxp = rxbp;

  devp->accaxes = ADXL355_ACC_NUMBER_OF_AXES;

  devp->state = ADXL355_STOP;
}

/**
 * @brief   Configures and activates ADXL355 Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p ADXL355Driver object
 * @param[in] config    pointer to the @p ADXL355Config object
 *
 * @api
 */
void adxl355Start(ADXL355Driver *devp, const ADXL355Config *config) {
  uint32_t i;
  uint8_t reg_val;
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == ADXL355_STOP) ||
                (devp->state == ADXL355_READY),
                "adxl355Start(), invalid state");		

  devp->config = config;

  /* Checking the device ID.*/
  {
#if ADXL355_SHARED_SPI
    spiAcquireBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */
    spiStart(devp->config->spip, devp->config->spicfg);
    adxl355SPIReadRegister(devp, ADXL355_AD_DEVID_MST, 1, &reg_val);
    osalDbgAssert((reg_val == ADXL355_DEVID_MST), "Invalid MEMS ID");
#if ADXL355_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */
  }

  /* Range register configuration block.*/
  {
    reg_val = ADXL355_RANGE_I2C_HS | devp->config->accfullscale;

#if ADXL355_USE_SPI
#if ADXL355_SHARED_SPI
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip, devp->config->spicfg);
#endif /* ADXL355_SHARED_SPI */

  adxl355SPIWriteRegister(devp, ADXL355_AD_RANGE, 1, &reg_val);

#if ADXL355_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */
#endif /* ADXL355_USE_SPI */
  }

  /* Filter register configuration block.*/
  {
    reg_val = devp->config->accodr;
#if ADXL355_USE_ADVANCED || defined(__DOXYGEN__)
    reg_val |= devp->config->acchighpass;
#endif

#if ADXL355_USE_SPI
#if ADXL355_SHARED_SPI
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip, devp->config->spicfg);
#endif /* ADXL355_SHARED_SPI */

  adxl355SPIWriteRegister(devp, ADXL355_AD_FILTER, 1, &reg_val);

#if ADXL355_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */
#endif /* ADXL355_USE_SPI */
  }

  /* Power control configuration block.*/
  {
    reg_val = 0;

#if ADXL355_USE_SPI
#if ADXL355_SHARED_SPI
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip, devp->config->spicfg);
#endif /* ADXL355_SHARED_SPI */

  adxl355SPIWriteRegister(devp, ADXL355_AD_POWER_CTL, 1, &reg_val);

#if	ADXL355_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */
#endif /* ADXL355_USE_SPI */
  }

  /* Storing sensitivity information according to user setting */
  if(devp->config->accfullscale == ADXL355_ACC_FS_2G) {
    devp->accfullscale = ADXL355_ACC_2G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = ADXL355_ACC_SENS_2G;
    else
      for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else if(devp->config->accfullscale == ADXL355_ACC_FS_4G) {
    devp->accfullscale = ADXL355_ACC_4G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = ADXL355_ACC_SENS_4G;
    else
      for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else if(devp->config->accfullscale == ADXL355_ACC_FS_8G) {
    devp->accfullscale = ADXL355_ACC_8G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = ADXL355_ACC_SENS_8G;
    else
      for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else {
    osalDbgAssert(FALSE, "adxl355Start(), accelerometer full scale issue");
  }

  /* Storing bias information according to user setting */
  if(devp->config->accbias != NULL)
    for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = devp->config->accbias[i];
  else
    for(i = 0; i < ADXL355_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = ADXL355_ACC_BIAS;

  /* This is the Accelerometer transient recovery time */
  osalThreadSleepMilliseconds(10);

  devp->state = ADXL355_READY;
}

/**
 * @brief   Deactivates the ADXL355 Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p ADXL355Driver object
 *
 * @api
 */
void adxl355Stop(ADXL355Driver *devp) {
  uint8_t reg_val;
  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == ADXL355_STOP) ||
                (devp->state == ADXL355_READY),
                "adxl355Stop(), invalid state");

  if (devp->state == ADXL355_READY) {
#if (ADXL355_USE_SPI)
#if	ADXL355_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* ADXL355_SHARED_SPI */
    /* Disabling all axes and enabling power down mode.*/
    reg_val = 1;
    adxl355SPIWriteRegister(devp, ADXL355_AD_POWER_CTL, 1, &reg_val);

    spiStop(devp->config->spip);
#if	ADXL355_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* ADXL355_SHARED_SPI */    		
#endif /* ADXL355_USE_SPI */
  }	
  devp->state = ADXL355_STOP;
}
/** @} */
