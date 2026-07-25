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
 * @file    lis302dl.c
 * @brief   LIS302DL MEMS interface module code.
 *
 * @addtogroup LIS302DL
 * @ingroup EX_ST
 * @{
 */

#include "hal.h"
#include "lis302dl.h"

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

#if (LIS302DL_USE_SPI) || defined(__DOXYGEN__)
/**
 * @brief   Reads a generic register value using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       starting register address
 * @param[in] n         number of adjacent registers to write
 * @param[in] b         pointer to a buffer.
 */
static void lis302dlSPIReadRegister(SPIDriver *spip, uint8_t reg,  size_t n,
                                    uint8_t* b) {
  uint8_t cmd;
  (n == 1) ? (cmd = reg | LIS302DL_RW) : (cmd = reg | LIS302DL_RW | LIS302DL_MS);
  spiSelect(spip);
  spiSend(spip, 1, &cmd);
  spiReceive(spip, n, b);
  spiUnselect(spip);
}

/**
 * @brief   Writes a value into a generic register using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       starting register address
 * @param[in] n         number of adjacent registers to write
 * @param[in] b         pointer to a buffer of values.
 */
static void lis302dlSPIWriteRegister(SPIDriver *spip, uint8_t reg, size_t n,
                                     uint8_t* b) {
  uint8_t cmd;
  (n == 1) ? (cmd = reg) : (cmd = reg | LIS302DL_MS);
  spiSelect(spip);
  spiSend(spip, 1, &cmd);
  spiSend(spip, n, b);
  spiUnselect(spip);
}
#endif /* LIS302DL_USE_SPI */

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 *
 * @return              the number of axes.
 */
static size_t acc_get_axes_number(void *ip) {
  (void)ip;

  return LIS302DL_ACC_NUMBER_OF_AXES;
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
  LIS302DLDriver* devp;
  uint8_t i, tmp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS302DLDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS302DL_READY),
                "acc_read_raw(), invalid state");

#if LIS302DL_USE_SPI
#if	LIS302DL_SHARED_SPI
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "acc_read_raw(), channel not ready");

  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip,
           devp->config->spicfg);
#endif /* LIS302DL_SHARED_SPI */

    for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++) {
      lis302dlSPIReadRegister(devp->config->spip,
                              LIS302DL_AD_OUT_X + (i * 2), 1, &tmp);
      axes[i] = (int32_t)((int8_t)tmp);
    }

#if	LIS302DL_SHARED_SPI
  spiReleaseBus(devp->config->spip);
#endif /* LIS302DL_SHARED_SPI */
#endif /* LIS302DL_USE_SPI */
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
  LIS302DLDriver* devp;
  uint32_t i;
  int32_t raw[LIS302DL_ACC_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS302DLDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS302DL_READY),
                "acc_read_cooked(), invalid state");

  msg = acc_read_raw(ip, raw);
  for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++) {
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
  LIS302DLDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS302DLDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS302DL_READY),
                "acc_set_bias(), invalid state");

  for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++) {
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
  LIS302DLDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS302DLDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS302DL_READY),
                "acc_reset_bias(), invalid state");

  for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
    devp->accbias[i] = LIS302DL_ACC_BIAS;
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
  LIS302DLDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS302DLDriver*, (BaseAccelerometer*)ip);

  osalDbgCheck((ip != NULL) && (sp != NULL));

  osalDbgAssert((devp->state == LIS302DL_READY),
                "acc_set_sensivity(), invalid state");

  for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++) {
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
  LIS302DLDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS302DLDriver*, (BaseAccelerometer*)ip);

  osalDbgAssert((devp->state == LIS302DL_READY),
                "acc_reset_sensivity(), invalid state");

  if(devp->config->accfullscale == LIS302DL_ACC_FS_2G)
    for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LIS302DL_ACC_SENS_2G;
  else if(devp->config->accfullscale == LIS302DL_ACC_FS_8G)
    for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
      devp->accsensitivity[i] = LIS302DL_ACC_SENS_8G;
  else {
    osalDbgAssert(FALSE,
                  "acc_reset_sensivity(), accelerometer full scale issue");
    return MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the LIS302DLDriver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LIS302DLDriver interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t acc_set_full_scale(LIS302DLDriver *devp, lis302dl_acc_fs_t fs) {
  float newfs, scale;
  uint8_t i, cr;
  msg_t msg;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LIS302DL_READY),
                "acc_set_full_scale(), invalid state");
  osalDbgAssert((devp->config->spip->state == SPI_READY),
                "acc_set_full_scale(), channel not ready");

  /* Computing new fullscale value.*/
  if(fs == LIS302DL_ACC_FS_2G) {
    newfs = LIS302DL_ACC_2G;
    msg = MSG_OK;
  }
  else if(fs == LIS302DL_ACC_FS_8G) {
    newfs = LIS302DL_ACC_8G;
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

#if LIS302DL_USE_SPI
#if LIS302DL_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* LIS302DL_SHARED_SPI */

    /* Getting data from register.*/
    lis302dlSPIReadRegister(devp->config->spip, LIS302DL_AD_CTRL_REG1, 1, &cr);

#if LIS302DL_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* LIS302DL_SHARED_SPI */
#endif /* LIS302DL_USE_SPI */

    cr &= ~(LIS302DL_CTRL_REG1_FS_MASK);
    cr |= fs;

#if LIS302DL_USE_SPI
#if LIS302DL_SHARED_SPI
    spiAcquireBus(devp->config->spip);
    spiStart(devp->config->spip,
             devp->config->spicfg);
#endif /* LIS302DL_SHARED_SPI */

    /* Getting data from register.*/
    lis302dlSPIWriteRegister(devp->config->spip, LIS302DL_AD_CTRL_REG1, 1, &cr);

#if LIS302DL_SHARED_SPI
    spiReleaseBus(devp->config->spip);
#endif /* LIS302DL_SHARED_SPI */
#endif /* LIS302DL_USE_SPI */

    /* Scaling sensitivity and bias. Re-calibration is suggested anyway. */
    for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++) {
      devp->accsensitivity[i] *= scale;
      devp->accbias[i] *= scale;
    }
  }
  return msg;
}

static const struct LIS302DLVMT vmt_device = {
  (size_t)0,
  acc_set_full_scale
};

static const struct BaseAccelerometerVMT vmt_accelerometer = {
  sizeof(struct LIS302DLVMT*),
  acc_get_axes_number, acc_read_raw, acc_read_cooked,
  acc_set_bias, acc_reset_bias, acc_set_sensivity, acc_reset_sensivity
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] devp     pointer to the @p LIS302DLDriver object
 *
 * @init
 */
void lis302dlObjectInit(LIS302DLDriver *devp) {
  devp->vmt = &vmt_device;
  devp->acc_if.vmt = &vmt_accelerometer;

  devp->config = NULL;

  devp->accaxes = LIS302DL_ACC_NUMBER_OF_AXES;

  devp->state = LIS302DL_STOP;
}

/**
 * @brief   Configures and activates LIS302DL Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p LIS302DLDriver object
 * @param[in] config    pointer to the @p LIS302DLConfig object
 *
 * @api
 */
void lis302dlStart(LIS302DLDriver *devp, const LIS302DLConfig *config) {
  uint32_t i;
  uint8_t cr[2] = {0, 0};
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == LIS302DL_STOP) || (devp->state == LIS302DL_READY),
              "lis302dlStart(), invalid state");

  devp->config = config;

  /* Control register 1 configuration block.*/
  {
    cr[0] = LIS302DL_CTRL_REG1_XEN | LIS302DL_CTRL_REG1_YEN |
            LIS302DL_CTRL_REG1_ZEN | LIS302DL_CTRL_REG1_PD |
            devp->config->accodr |
            devp->config->accfullscale;
  }

  /* Control register 2 configuration block.*/
  {
#if LIS302DL_USE_ADVANCED || defined(__DOXYGEN__)
  if(devp->config->hpmode != LIS302DL_HPM_BYPASSED)
    cr[1] = devp->config->acchighpass;
#endif
  }

#if LIS302DL_USE_SPI
#if LIS302DL_SHARED_SPI
  spiAcquireBus((devp)->config->spip);
#endif /* LIS302DL_SHARED_SPI */
  spiStart((devp)->config->spip, (devp)->config->spicfg);

  lis302dlSPIWriteRegister(devp->config->spip, LIS302DL_AD_CTRL_REG1,
                           2, cr);

#if	LIS302DL_SHARED_SPI
  spiReleaseBus((devp)->config->spip);
#endif /* LIS302DL_SHARED_SPI */
#endif /* LIS302DL_USE_SPI */

  /* Storing sensitivity information according to full scale value */
  if(devp->config->accfullscale == LIS302DL_ACC_FS_2G) {
    devp->accfullscale = LIS302DL_ACC_2G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = LIS302DL_ACC_SENS_2G;
    else
      for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else if(devp->config->accfullscale == LIS302DL_ACC_FS_8G) {
    devp->accfullscale = LIS302DL_ACC_8G;
    if(devp->config->accsensitivity == NULL)
      for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = LIS302DL_ACC_SENS_8G;
    else
      for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
        devp->accsensitivity[i] = devp->config->accsensitivity[i];
  }
  else {
    osalDbgAssert(FALSE, "lis302dlStart(), accelerometer full scale issue");
  }

  /* Storing bias information according to user setting */
  if(devp->config->accbias != NULL)
    for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = devp->config->accbias[i];
  else
    for(i = 0; i < LIS302DL_ACC_NUMBER_OF_AXES; i++)
      devp->accbias[i] = LIS302DL_ACC_BIAS;

  /* This is the Accelerometer transient recovery time */
  osalThreadSleepMilliseconds(10);

  devp->state = LIS302DL_READY;
}

/**
 * @brief   Deactivates the LIS302DL Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p LIS302DLDriver object
 *
 * @api
 */
void lis302dlStop(LIS302DLDriver *devp) {
  uint8_t cr1;
  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LIS302DL_STOP) ||
                (devp->state == LIS302DL_READY),
                "lis302dlStop(), invalid state");

  if (devp->state == LIS302DL_READY) {
#if LIS302DL_USE_SPI
#if	LIS302DL_SHARED_SPI
    spiAcquireBus((devp)->config->spip);
    spiStart((devp)->config->spip,
             (devp)->config->spicfg);
#endif /* LIS302DL_SHARED_SPI */
    /* Disabling all axes and enabling power down mode.*/
    cr1 = 0;
    lis302dlSPIWriteRegister(devp->config->spip, LIS302DL_AD_CTRL_REG1, 1, &cr1);
    spiStop((devp)->config->spip);
#if	LIS302DL_SHARED_SPI
    spiReleaseBus((devp)->config->spip);
#endif /* LIS302DL_SHARED_SPI */
#endif /* LIS302DL_USE_SPI */
  }
  devp->state = LIS302DL_STOP;
}
/** @} */
