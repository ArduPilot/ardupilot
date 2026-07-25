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
 * @file    lis3dsh.c
 * @brief   LIS3DSH MEMS interface module code.
 *
 * @addtogroup LIS3DSH
 * @ingroup EX_ST
 * @{
 */

#include "hal.h"
#include "lis3dsh.h"

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

#if (LIS3DSH_USE_SPI) || defined(__DOXYGEN__)
/**
 * @brief   Reads a generic register value using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 * @note    Multiple write/read requires proper settings in CTRL_REG6.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       starting register address
 * @param[in] n         number of adjacent registers to write
 * @param[in] b         pointer to a buffer.
 */
static void lis3dshSPIReadRegister(SPIDriver *spip, uint8_t reg, size_t n,
                                   uint8_t* b) {
  uint8_t cmd;
  cmd = reg | LIS3DSH_RW;
  spiSelect(spip);
  spiSend(spip, 1, &cmd);
  spiReceive(spip, n, b);
  spiUnselect(spip);
}

/**
 * @brief   Writes a value into a generic register using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 * @note    Multiple write/read requires proper settings in CTRL_REG6.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       starting register address
 * @param[in] n         number of adjacent registers to write
 * @param[in] b         pointer to a buffer of values.
 */
static void lis3dshSPIWriteRegister(SPIDriver *spip, uint8_t reg, size_t n,
                                    uint8_t* b) {
  uint8_t cmd;
  cmd = reg;
  spiSelect(spip);
  spiSend(spip, 1, &cmd);
  spiSend(spip, n, b);
  spiUnselect(spip);
}
#endif /* LIS3DSH_USE_SPI */

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              the number of axes.
 */
static size_t acc_get_axes_number(void *ip) {
  (void)ip;

  return LIS3DSH_ACC_NUMBER_OF_AXES;
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
  LIS3DSHDriver* devp;
  uint8_t buff [LIS3DSH_ACC_NUMBER_OF_AXES * 2], i;
  int16_t tmp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3DSHDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS3DSH_READY),
                "acc_read_raw(), invalid state");

#if LIS3DSH_USE_SPI
#if	LIS3DSH_SHARED_SPI
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "acc_read_raw(), channel not ready");

  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip,
           devp->config->spicfg);
#endif /* LIS3DSH_SHARED_SPI */

  lis3dshSPIReadRegister(devp->config->spip, LIS3DSH_AD_OUT_X_L,
                         LIS3DSH_ACC_NUMBER_OF_AXES * 2, buff);

#if	LIS3DSH_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* LIS3DSH_SHARED_SPI */
#endif /* LIS3DSH_USE_SPI */

  for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++) {
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
  LIS3DSHDriver* devp;
  uint32_t i;
  int32_t raw[LIS3DSH_ACC_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3DSHDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS3DSH_READY),
                "acc_read_cooked(), invalid state");

  msg = acc_read_raw(ip, raw);
  for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++) {
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
  LIS3DSHDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3DSHDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS3DSH_READY),
                "acc_set_bias(), invalid state");

  for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++) {
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
  LIS3DSHDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3DSHDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS3DSH_READY),
                "acc_reset_bias(), invalid state");

  for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
    devp->accbias[i] = LIS3DSH_ACC_BIAS;
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
  LIS3DSHDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3DSHDriver*, (BaseAccelerometer*)ip);

  osalDbgCheck((ip != NULL) && (sp != NULL));

  osalDbgAssert((devp->state == LIS3DSH_READY),
                "acc_set_sensivity(), invalid state");

  for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++) {
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
  LIS3DSHDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3DSHDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS3DSH_READY),
                "acc_reset_sensivity(), invalid state");

  if(devp->config->accfullscale == LIS3DSH_ACC_FS_2G)
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LIS3DSH_ACC_SENS_2G;
  else if(devp->config->accfullscale == LIS3DSH_ACC_FS_4G)
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LIS3DSH_ACC_SENS_4G;
  else if(devp->config->accfullscale == LIS3DSH_ACC_FS_6G)
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LIS3DSH_ACC_SENS_6G;
  else if(devp->config->accfullscale == LIS3DSH_ACC_FS_8G)
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LIS3DSH_ACC_SENS_8G;
  else if(devp->config->accfullscale == LIS3DSH_ACC_FS_16G)
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LIS3DSH_ACC_SENS_16G;
  else {
    osalDbgAssert(FALSE,
                  "acc_reset_sensivity(), accelerometer full scale issue");
    return MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the LIS3DSHDriver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t acc_set_full_scale(LIS3DSHDriver *devp, lis3dsh_acc_fs_t fs) {
  float newfs, scale;
  uint8_t i, cr;
  msg_t msg;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LIS3DSH_READY),
                "acc_set_full_scale(), invalid state");
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "acc_set_full_scale(), channel not ready");

  /* Computing new fullscale value.*/
  if(fs == LIS3DSH_ACC_FS_2G) {
    newfs = LIS3DSH_ACC_2G;
    msg = MSG_OK;
  }
  else if(fs == LIS3DSH_ACC_FS_4G) {
    newfs = LIS3DSH_ACC_4G;
    msg = MSG_OK;
  }
  else if(fs == LIS3DSH_ACC_FS_6G) {
    newfs = LIS3DSH_ACC_6G;
    msg = MSG_OK;
  }
  else if(fs == LIS3DSH_ACC_FS_8G) {
    newfs = LIS3DSH_ACC_8G;
    msg = MSG_OK;
  }
  else if(fs == LIS3DSH_ACC_FS_16G) {
    newfs = LIS3DSH_ACC_16G;
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

#if LIS3DSH_USE_SPI
#if LIS3DSH_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* LIS3DSH_SHARED_SPI */

    /* Getting data from register.*/
    lis3dshSPIReadRegister(devp->config->spip, LIS3DSH_AD_CTRL_REG5, 1, &cr);

#if LIS3DSH_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* LIS3DSH_SHARED_SPI */
#endif /* LIS3DSH_USE_SPI */

    cr &= ~(LIS3DSH_CTRL_REG5_FS_MASK);
    cr |= fs;

#if LIS3DSH_USE_SPI
#if LIS3DSH_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* LIS3DSH_SHARED_SPI */

    /* Getting data from register.*/
    lis3dshSPIWriteRegister(devp->config->spip, LIS3DSH_AD_CTRL_REG5, 1, &cr);

#if LIS3DSH_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* LIS3DSH_SHARED_SPI */
#endif /* LIS3DSH_USE_SPI */

    /* Scaling sensitivity and bias. Re-calibration is suggested anyway. */
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++) {
      devp->accsensitivity[i] *= scale;
      devp->accbias[i] *= scale;
    }
  }
  return msg;
}

static const struct LIS3DSHVMT vmt_device = {
  (size_t)0,
  acc_set_full_scale
};

static const struct BaseAccelerometerVMT vmt_accelerometer = {
  sizeof(struct LIS3DSHVMT*),
  acc_get_axes_number, acc_read_raw, acc_read_cooked,
  acc_set_bias, acc_reset_bias, acc_set_sensivity, acc_reset_sensivity
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] devp     pointer to the @p LIS3DSHDriver object
 *
 * @init
 */
void lis3dshObjectInit(LIS3DSHDriver *devp) {
  devp->vmt = &vmt_device;
  devp->acc_if.vmt = &vmt_accelerometer;

  devp->config = NULL;

  devp->accaxes = LIS3DSH_ACC_NUMBER_OF_AXES;

  devp->state = LIS3DSH_STOP;
}

/**
 * @brief   Configures and activates LIS3DSH Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p LIS3DSHDriver object
 * @param[in] config    pointer to the @p LIS3DSHConfig object
 *
 * @api
 */
void lis3dshStart(LIS3DSHDriver *devp, const LIS3DSHConfig *config) {
  uint32_t i;
  uint8_t cr;
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == LIS3DSH_STOP) ||
                (devp->state == LIS3DSH_READY),
                "lis3dshStart(), invalid state");		

  devp->config = config;

  /* Control register 4 configuration block.*/
  {
    cr = LIS3DSH_CTRL_REG4_XEN | LIS3DSH_CTRL_REG4_YEN | LIS3DSH_CTRL_REG4_ZEN |
         devp->config->accodr;
#if LIS3DSH_USE_ADVANCED || defined(__DOXYGEN__)
    cr |= devp->config->accbdu;
#endif
  }

#if LIS3DSH_USE_SPI
#if LIS3DSH_SHARED_SPI
  spiAcquireBus(devp->config->spip);
#endif /* LIS3DSH_SHARED_SPI */
  spiStart(devp->config->spip, devp->config->spicfg);

  lis3dshSPIWriteRegister(devp->config->spip, LIS3DSH_AD_CTRL_REG4, 1, &cr);

#if LIS3DSH_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* LIS3DSH_SHARED_SPI */
#endif /* LIS3DSH_USE_SPI */

  /* Control register 5 configuration block.*/
  {
    cr = devp->config->accfullscale;
#if LIS3DSH_USE_ADVANCED || defined(__DOXYGEN__)
    cr |= devp->config->accantialiasing;
#endif
  }

#if LIS3DSH_USE_SPI
#if LIS3DSH_SHARED_SPI
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip, devp->config->spicfg);
#endif /* LIS3DSH_SHARED_SPI */

  lis3dshSPIWriteRegister(devp->config->spip, LIS3DSH_AD_CTRL_REG5, 1, &cr);

#if LIS3DSH_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* LIS3DSH_SHARED_SPI */
#endif /* LIS3DSH_USE_SPI */

  /* Control register 6 configuration block.*/
  {
    cr = LIS3DSH_CTRL_REG6_ADD_INC;
#if LIS3DSH_USE_ADVANCED || defined(__DOXYGEN__)
    cr |= devp->config->accbdu;
#endif
  }

#if LIS3DSH_USE_SPI
#if LIS3DSH_SHARED_SPI
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip, devp->config->spicfg);
#endif /* LIS3DSH_SHARED_SPI */

  lis3dshSPIWriteRegister(devp->config->spip, LIS3DSH_AD_CTRL_REG6, 1, &cr);

#if	LIS3DSH_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* LIS3DSH_SHARED_SPI */
#endif /* LIS3DSH_USE_SPI */

  /* Storing sensitivity information according to user setting */
  if(devp->config->accfullscale == LIS3DSH_ACC_FS_2G) {
    devp->accfullscale = LIS3DSH_ACC_2G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = LIS3DSH_ACC_SENS_2G;
    else
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else if(devp->config->accfullscale == LIS3DSH_ACC_FS_4G) {
    devp->accfullscale = LIS3DSH_ACC_4G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = LIS3DSH_ACC_SENS_4G;
    else
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else if(devp->config->accfullscale == LIS3DSH_ACC_FS_6G) {
    devp->accfullscale = LIS3DSH_ACC_6G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = LIS3DSH_ACC_SENS_6G;
    else
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else if(devp->config->accfullscale == LIS3DSH_ACC_FS_8G) {
    devp->accfullscale = LIS3DSH_ACC_8G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = LIS3DSH_ACC_SENS_8G;
    else
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else if(devp->config->accfullscale == LIS3DSH_ACC_FS_16G) {
    devp->accfullscale = LIS3DSH_ACC_16G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = LIS3DSH_ACC_SENS_16G;
    else
      for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else {
    osalDbgAssert(FALSE, "lis3dshStart(), accelerometer full scale issue");
  }

  /* Storing bias information according to user setting */
  if(devp->config->accbias != NULL)
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = devp->config->accbias[i];
  else
    for(i = 0; i < LIS3DSH_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = LIS3DSH_ACC_BIAS;

  /* This is the Accelerometer transient recovery time */
  osalThreadSleepMilliseconds(10);

  devp->state = LIS3DSH_READY;
}

/**
 * @brief   Deactivates the LIS3DSH Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p LIS3DSHDriver object
 *
 * @api
 */
void lis3dshStop(LIS3DSHDriver *devp) {
  uint8_t cr4;
  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LIS3DSH_STOP) ||
                (devp->state == LIS3DSH_READY),
                "lis3dshStop(), invalid state");

  if (devp->state == LIS3DSH_READY) {
#if (LIS3DSH_USE_SPI)
#if	LIS3DSH_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* LIS3DSH_SHARED_SPI */
    /* Disabling all axes and enabling power down mode.*/
    cr4 = 0;
    lis3dshSPIWriteRegister(devp->config->spip, LIS3DSH_AD_CTRL_REG4,
                           1, &cr4);

    spiStop(devp->config->spip);
#if	LIS3DSH_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* LIS3DSH_SHARED_SPI */    		
#endif /* LIS3DSH_USE_SPI */
  }	
  devp->state = LIS3DSH_STOP;
}
/** @} */
