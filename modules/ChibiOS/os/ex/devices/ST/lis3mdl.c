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
 * @file    lis3mdl.c
 * @brief   LIS3MDL MEMS interface module code.
 *
 * @addtogroup LIS3MDL
 * @ingroup EX_ST
 * @{
 */

#include "hal.h"
#include "lis3mdl.h"

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

#if (LIS3MDL_USE_I2C) || defined(__DOXYGEN__)
/**
 * @brief   Reads registers value using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in]  i2cp      pointer to the I2C interface
 * @param[in]  sad       slave address without R bit
 * @param[in]  reg       first sub-register address
 * @param[out] rxbuf     pointer to an output buffer
 * @param[in]  n         number of consecutive register to read
 * @return               the operation status.
 * @notapi
 */
msg_t lis3mdlI2CReadRegister(I2CDriver *i2cp, lis3mdl_sad_t sad, uint8_t reg,
                             uint8_t* rxbuf, size_t n) {
  uint8_t txbuf = reg;
  if(n > 1)
    txbuf |= LIS3MDL_SUB_MS;

  return i2cMasterTransmitTimeout(i2cp, sad, &txbuf, 1, rxbuf, n,
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
msg_t lis3mdlI2CWriteRegister(I2CDriver *i2cp, lis3mdl_sad_t sad, uint8_t* txbuf,
                              uint8_t n) {
  if (n > 1)
    (*txbuf) |= LIS3MDL_SUB_MS;

  return i2cMasterTransmitTimeout(i2cp, sad, txbuf, n + 1, NULL, 0,
                                  TIME_INFINITE);
}
#endif /* LIS3MDL_USE_I2C */

/**
 * @brief   Return the number of axes of the BaseCompass.
 *
 * @param[in] ip        pointer to @p BaseCompass interface
 *
 * @return              the number of axes.
 */
static size_t comp_get_axes_number(void *ip) {

  osalDbgCheck(ip != NULL);
  return LIS3MDL_COMP_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseCompass.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseCompass axes number.
 *
 * @param[in] ip        pointer to @p BaseCompass interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t comp_read_raw(void *ip, int32_t axes[]) {
  LIS3MDLDriver* devp;
  uint8_t buff [LIS3MDL_COMP_NUMBER_OF_AXES * 2], i;
  int16_t tmp;
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3MDLDriver*, (BaseCompass*)ip);

  osalDbgAssert((devp->state == LIS3MDL_READY),
                "comp_read_raw(), invalid state");
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "comp_read_raw(), channel not ready");

#if LIS3MDL_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* LIS3MDL_SHARED_I2C */
  msg = lis3mdlI2CReadRegister(devp->config->i2cp, devp->config->slaveaddress,
                               LIS3MDL_AD_OUT_X_L, buff,
                               LIS3MDL_COMP_NUMBER_OF_AXES * 2);

#if LIS3MDL_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LIS3MDL_SHARED_I2C */

  if(msg == MSG_OK)
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++) {
      tmp = buff[2 * i] + (buff[2 * i + 1] << 8);
      axes[i] = (int32_t)tmp;
    }
  return msg;
}

/**
 * @brief   Retrieves cooked data from the BaseCompass.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as G.
 * @note    The axes array must be at least the same size of the
 *          BaseCompass axes number.
 *
 * @param[in] ip        pointer to @p BaseCompass interface.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t comp_read_cooked(void *ip, float axes[]) {
  LIS3MDLDriver* devp;
  uint32_t i;
  int32_t raw[LIS3MDL_COMP_NUMBER_OF_AXES];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));


  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3MDLDriver*, (BaseCompass*)ip);

  osalDbgAssert((devp->state == LIS3MDL_READY),
                "comp_read_cooked(), invalid state");

  msg = comp_read_raw(ip, raw);
  for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES ; i++) {
    axes[i] = (raw[i] * devp->compsensitivity[i]) - devp->compbias[i];
  }
  return msg;
}

/**
 * @brief   Set bias values for the BaseCompass.
 * @note    Bias must be expressed as G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseCompass axes number.
 *
 * @param[in] ip        pointer to @p BaseCompass interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t comp_set_bias(void *ip, float *bp) {
  LIS3MDLDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3MDLDriver*, (BaseCompass*)ip);

  osalDbgAssert((devp->state == LIS3MDL_READY),
                "comp_set_bias(), invalid state");

  for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++) {
    devp->compbias[i] = bp[i];
  }
  return msg;
}

/**
 * @brief   Reset bias values for the BaseCompass.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseCompass interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t comp_reset_bias(void *ip) {
  LIS3MDLDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3MDLDriver*, (BaseCompass*)ip);

  osalDbgAssert((devp->state == LIS3MDL_READY),
                "comp_reset_bias(), invalid state");

  for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++)
    devp->compbias[i] = LIS3MDL_COMP_BIAS;
  return msg;
}

/**
 * @brief   Set sensitivity values for the BaseCompass.
 * @note    Sensitivity must be expressed as G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseCompass axes number.
 *
 * @param[in] ip        pointer to @p BaseCompass interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t comp_set_sensivity(void *ip, float *sp) {
  LIS3MDLDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3MDLDriver*, (BaseCompass*)ip);

  osalDbgCheck((ip != NULL) && (sp != NULL));

  osalDbgAssert((devp->state == LIS3MDL_READY),
                "comp_set_sensivity(), invalid state");

  for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++) {
    devp->compsensitivity[i] = sp[i];
  }
  return msg;
}

/**
 * @brief   Reset sensitivity values for the BaseCompass.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseCompass interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t comp_reset_sensivity(void *ip) {
  LIS3MDLDriver* devp;
  uint32_t i;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LIS3MDLDriver*, (BaseCompass*)ip);

  osalDbgAssert((devp->state == LIS3MDL_READY),
                "comp_reset_sensivity(), invalid state");

  if(devp->config->compfullscale == LIS3MDL_COMP_FS_4GA)
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++)
      devp->compsensitivity[i] = LIS3MDL_COMP_SENS_4GA;
  else if(devp->config->compfullscale == LIS3MDL_COMP_FS_8GA)
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++)
      devp->compsensitivity[i] = LIS3MDL_COMP_SENS_8GA;
  else if(devp->config->compfullscale == LIS3MDL_COMP_FS_12GA)
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++)
      devp->compsensitivity[i] = LIS3MDL_COMP_SENS_12GA;
  else if(devp->config->compfullscale == LIS3MDL_COMP_FS_16GA)
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++)
      devp->compsensitivity[i] = LIS3MDL_COMP_SENS_16GA;
  else {
    osalDbgAssert(FALSE, "comp_reset_sensivity(), compass full scale issue");
    msg = MSG_RESET;
  }
  return msg;
}

/**
 * @brief   Changes the LIS3MDLDriver compass fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LIS3MDLDriver interface.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 */
static msg_t comp_set_full_scale(LIS3MDLDriver *devp, lis3mdl_comp_fs_t fs) {
  float newfs, scale;
  uint8_t i, buff[2];
  msg_t msg;

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LIS3MDL_READY),
                "comp_set_full_scale(), invalid state");
  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "comp_set_full_scale(), channel not ready");

  /* Computing new fullscale value.*/
  if(fs == LIS3MDL_COMP_FS_4GA) {
    newfs = LIS3MDL_COMP_4GA;
    msg = MSG_OK;
  }
  else if(fs == LIS3MDL_COMP_FS_8GA) {
    newfs = LIS3MDL_COMP_8GA;
    msg = MSG_OK;
  }
  else if(fs == LIS3MDL_COMP_FS_12GA) {
    newfs = LIS3MDL_COMP_12GA;
    msg = MSG_OK;
  }
  else if(fs == LIS3MDL_COMP_FS_16GA) {
    newfs = LIS3MDL_COMP_16GA;
    msg = MSG_OK;
  }
  else {
    msg = MSG_RESET;
  }

  if((msg == MSG_OK) &&
     (newfs != devp->compfullscale)) {
    /* Computing scale value.*/
    scale = newfs / devp->compfullscale;
    devp->compfullscale = newfs;

#if LIS3MDL_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LIS3MDL_SHARED_I2C */

    /* Updating register.*/
    msg = lis3mdlI2CReadRegister(devp->config->i2cp, devp->config->slaveaddress,
                                 LIS3MDL_AD_CTRL_REG2, &buff[1], 1);

#if LIS3MDL_SHARED_I2C
        i2cReleaseBus(devp->config->i2cp);
#endif /* LIS3MDL_SHARED_I2C */

    if(msg == MSG_OK) {

      buff[1] &= ~(LIS3MDL_CTRL_REG2_FS_MASK);
      buff[1] |= fs;
      buff[0] = LIS3MDL_AD_CTRL_REG2;

#if LIS3MDL_SHARED_I2C
      i2cAcquireBus(devp->config->i2cp);
      i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* LIS3MDL_SHARED_I2C */

      msg = lis3mdlI2CWriteRegister(devp->config->i2cp,
                                    devp->config->slaveaddress,
                                    buff, 1);

#if LIS3MDL_SHARED_I2C
      i2cReleaseBus(devp->config->i2cp);
#endif /* LIS3MDL_SHARED_I2C */
    }
    if(msg == MSG_OK) {

      /* Scaling sensitivity and bias. Re-calibration is suggested anyway.*/
      for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++) {
        devp->compsensitivity[i] *= scale;
        devp->compbias[i] *= scale;
      }
    }
  }
  return msg;
}

static const struct LIS3MDLVMT vmt_device = {
  (size_t)0,
  comp_set_full_scale
};

static const struct BaseCompassVMT vmt_compass = {
  sizeof(struct LIS3MDLVMT*),
  comp_get_axes_number, comp_read_raw, comp_read_cooked,
  comp_set_bias, comp_reset_bias, comp_set_sensivity, comp_reset_sensivity
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] devp     pointer to the @p LIS3MDLDriver object
 *
 * @init
 */
void lis3mdlObjectInit(LIS3MDLDriver *devp) {
  devp->vmt = &vmt_device;
  devp->comp_if.vmt = &vmt_compass;

  devp->config = NULL;

  devp->compaxes = LIS3MDL_COMP_NUMBER_OF_AXES;

  devp->state  = LIS3MDL_STOP;
}

/**
 * @brief   Configures and activates LIS3MDL Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p LIS3MDLDriver object
 * @param[in] config    pointer to the @p LIS3MDLConfig object
 *
 * @api
 */
void lis3mdlStart(LIS3MDLDriver *devp, const LIS3MDLConfig *config) {
  uint32_t i;
  uint8_t cr[6];
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == LIS3MDL_STOP) || (devp->state == LIS3MDL_READY),
              "lis3mdlStart(), invalid state");

  devp->config = config;

  /* Control register 1 configuration block.*/
  {
    cr[0] = LIS3MDL_AD_CTRL_REG1;
    cr[1] = devp->config->compodr;
#if LIS3MDL_USE_ADVANCED || defined(__DOXYGEN__)
    cr[1] |= devp->config->compopmodexy;
#else
    cr[1] |= LIS3MDL_CTRL_REG1_OM0 | LIS3MDL_CTRL_REG1_OM1;
#endif
  }

  /* Control register 2 configuration block.*/
  {
    cr[2] = devp->config->compfullscale;
  }

  /* Control register 3 configuration block.*/
  {
    cr[3] = 0;
#if LIS3MDL_USE_ADVANCED || defined(__DOXYGEN__)
    cr[3] = devp->config->compconvmode;
#endif
  }

  /* Control register 4 configuration block.*/
  {
    cr[4] = 0;
#if LIS3MDL_USE_ADVANCED || defined(__DOXYGEN__)
    cr[4] = devp->config->compopmodez | devp->config->endianness;
#endif
  }

  /* Control register 5 configuration block.*/
  {
    cr[5] = 0;
#if LIS3MDL_USE_ADVANCED || defined(__DOXYGEN__)
    cr[5] = devp->config->bdu;
#endif
  }

#if LIS3MDL_USE_I2C
#if LIS3MDL_SHARED_I2C
  i2cAcquireBus((devp)->config->i2cp);
#endif /* LIS3MDL_SHARED_I2C */
  i2cStart((devp)->config->i2cp,
           (devp)->config->i2ccfg);

  lis3mdlI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                          cr, 5);

#if LIS3MDL_SHARED_I2C
  i2cReleaseBus((devp)->config->i2cp);
#endif /* LIS3MDL_SHARED_I2C */
#endif /* LIS3MDL_USE_I2C */

  if(devp->config->compfullscale == LIS3MDL_COMP_FS_4GA) {
    devp->compfullscale = LIS3MDL_COMP_4GA;
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++) {
      if(devp->config->compsensitivity == NULL) {
        devp->compsensitivity[i] = LIS3MDL_COMP_SENS_4GA;
      }
      else {
        devp->compsensitivity[i] = devp->config->compsensitivity[i];
      }
    }
  }
  else if(devp->config->compfullscale == LIS3MDL_COMP_FS_8GA) {
    devp->compfullscale = LIS3MDL_COMP_8GA;
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++) {
      if(devp->config->compsensitivity == NULL) {
        devp->compsensitivity[i] = LIS3MDL_COMP_SENS_8GA;
      }
      else {
        devp->compsensitivity[i] = devp->config->compsensitivity[i];
      }
    }
  }
  else if(devp->config->compfullscale == LIS3MDL_COMP_FS_12GA) {
    devp->compfullscale = LIS3MDL_COMP_12GA;
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++) {
      if(devp->config->compsensitivity == NULL) {
        devp->compsensitivity[i] = LIS3MDL_COMP_SENS_12GA;
      }
      else {
        devp->compsensitivity[i] = devp->config->compsensitivity[i];
      }
    }
  }
  else if(devp->config->compfullscale == LIS3MDL_COMP_FS_16GA) {
    devp->compfullscale = LIS3MDL_COMP_16GA;
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++) {
      if(devp->config->compsensitivity == NULL) {
        devp->compsensitivity[i] = LIS3MDL_COMP_SENS_16GA;
      }
      else {
        devp->compsensitivity[i] = devp->config->compsensitivity[i];
      }
    }
  }
  else
    osalDbgAssert(FALSE, "lis3mdlStart(), compass full scale issue");

  /* Storing bias information */
  if(devp->config->compbias != NULL)
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++)
      devp->compbias[i] = devp->config->compbias[i];
  else
    for(i = 0; i < LIS3MDL_COMP_NUMBER_OF_AXES; i++)
      devp->compbias[i] = LIS3MDL_COMP_BIAS;

  /* This is the MEMS transient recovery time */
  osalThreadSleepMilliseconds(5);

  devp->state = LIS3MDL_READY;
}

/**
 * @brief   Deactivates the LIS3MDL Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p LIS3MDLDriver object
 *
 * @api
 */
void lis3mdlStop(LIS3MDLDriver *devp) {
  uint8_t cr[2];
  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LIS3MDL_STOP) || (devp->state == LIS3MDL_READY),
                "lis3mdlStop(), invalid state");

  if (devp->state == LIS3MDL_READY) {
#if (LIS3MDL_USE_I2C)
#if LIS3MDL_SHARED_I2C
    i2cAcquireBus((devp)->config->i2cp);
    i2cStart((devp)->config->i2cp,
             (devp)->config->i2ccfg);
#endif /* LIS3MDL_SHARED_I2C */

    /* Disabling compass. */
    cr[0] = LIS3MDL_AD_CTRL_REG3;
    cr[1] = LIS3MDL_CTRL_REG3_MD0 | LIS3MDL_CTRL_REG3_MD1;
    lis3mdlI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                            cr, 1);

    i2cStop((devp)->config->i2cp);
#if LIS3MDL_SHARED_I2C
    i2cReleaseBus((devp)->config->i2cp);
#endif /* LIS3MDL_SHARED_I2C */
#endif /* LIS3MDL_USE_I2C */
  }
  devp->state = LIS3MDL_STOP;
}
/** @} */
