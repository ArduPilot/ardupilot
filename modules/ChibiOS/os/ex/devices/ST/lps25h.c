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
 * @file    lps25h.c
 * @brief   LPS25H MEMS interface module code.
 *
 * @addtogroup LPS25H
 * @ingroup EX_ST
 * @{
 */

#include "hal.h"
#include "lps25h.h"

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

#if (LPS25H_USE_I2C) || defined(__DOXYGEN__)
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
 *
 * @notapi
 */
static msg_t lps25hI2CReadRegister(I2CDriver *i2cp, lps25h_sad_t sad,
                                   uint8_t reg, uint8_t* rxbuf, size_t n) {
  uint8_t txbuf = reg;
  if(n > 1)
    txbuf |= LPS25H_SUB_MS;

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
 *
 * @notapi
 */
static msg_t lps25hI2CWriteRegister(I2CDriver *i2cp, lps25h_sad_t sad,
                                    uint8_t* txbuf, size_t n) {
  if (n > 1)
    (*txbuf) |= LPS25H_SUB_MS;
  return i2cMasterTransmitTimeout(i2cp, sad, txbuf, n + 1, NULL, 0,
                                  TIME_INFINITE);
}
#endif /* LPS25H_USE_I2C */

/**
 * @brief   Return the number of axes of the BaseBarometer.
 *
 * @param[in] ip        pointer to @p BaseBarometer interface.
 *
 * @return              the number of axes.
 */
static size_t baro_get_axes_number(void *ip) {
  (void)ip;

  return LPS25H_BARO_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseBarometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseBarometer axes number.
 *
 * @param[in] ip        pointer to @p BaseBarometer interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t baro_read_raw(void *ip, int32_t axes[]) {
  LPS25HDriver* devp;
  uint8_t buff[3];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseBarometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "baro_read_raw(), invalid state");

  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "baro_read_raw(), channel not ready");

#if LPS25H_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* LPS25H_SHARED_I2C */

  msg = lps25hI2CReadRegister(devp->config->i2cp, devp->config->slaveaddress,
                              LPS25H_AD_PRESS_OUT_XL, buff, 3);

#if LPS25H_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LPS25H_SHARED_I2C */

  if(msg == MSG_OK) {
    *axes = buff[0] + (buff[1] << 8) + (buff[2] << 16);
  }
  return msg;
}

/**
 * @brief   Retrieves cooked data from the BaseBarometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as hPa.
 * @note    The axes array must be at least the same size of the
 *          BaseBarometer axes number.
 *
 * @param[in] ip        pointer to @p BaseBarometer interface.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t baro_read_cooked(void *ip, float axes[]) {
  LPS25HDriver* devp;
  int32_t raw;
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseBarometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "baro_read_cooked(), invalid state");

  msg = baro_read_raw(ip, &raw);

  *axes = (raw * devp->barosensitivity) - devp->barobias;

  return msg;
}

/**
 * @brief   Set bias values for the BaseBarometer.
 * @note    Bias must be expressed as hPa.
 * @note    The bias buffer must be at least the same size of the
 *          BaseBarometer axes number.
 *
 * @param[in] ip        pointer to @p BaseBarometer interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t baro_set_bias(void *ip, float *bp) {
  LPS25HDriver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseBarometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "baro_set_bias(), invalid state");

  devp->barobias = *bp;
  return msg;
}

/**
 * @brief   Reset bias values for the BaseBarometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseBarometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t baro_reset_bias(void *ip) {
  LPS25HDriver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseBarometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "baro_reset_bias(), invalid state");

  devp->barobias = LPS25H_BARO_SENS;
  return msg;
}

/**
 * @brief   Set sensitivity values for the BaseBarometer.
 * @note    Sensitivity must be expressed as hPa/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseBarometer axes number.
 *
 * @param[in] ip        pointer to @p BaseBarometer interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t baro_set_sensitivity(void *ip, float *sp) {
  LPS25HDriver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (sp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseBarometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "baro_set_sensitivity(), invalid state");

  devp->barosensitivity = *sp;
  return msg;
}

/**
 * @brief   Reset sensitivity values for the BaseBarometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseBarometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t baro_reset_sensitivity(void *ip) {
  LPS25HDriver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

    /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseBarometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "baro_reset_sensitivity(), invalid state");

  devp->barosensitivity = LPS25H_BARO_SENS;
  return msg;
}

/**
 * @brief   Return the number of axes of the BaseThermometer.
 *
 * @param[in] ip        pointer to @p BaseThermometer interface.
 *
 * @return              the number of axes.
 */
static size_t thermo_get_axes_number(void *ip) {
  (void)ip;

  return LPS25H_THERMO_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseThermometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] ip        pointer to @p BaseThermometer interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t thermo_read_raw(void *ip, int32_t axes[]) {
  LPS25HDriver* devp;
  int16_t tmp;
  uint8_t buff[2];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "thermo_read_raw(), invalid state");

  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "thermo_read_raw(), channel not ready");

#if LPS25H_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* LPS25H_SHARED_I2C */

  msg = lps25hI2CReadRegister(devp->config->i2cp, devp->config->slaveaddress,
                              LPS25H_AD_TEMP_OUT_L, buff, 2);

#if LPS25H_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* LPS25H_SHARED_I2C */

  if (msg == MSG_OK) {
    tmp = buff[0] + (buff[1] << 8);
    *axes = (int32_t)tmp;
  }
  return msg;
}

/**
 * @brief   Retrieves cooked data from the BaseThermometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as °C.
 * @note    The axes array must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] ip        pointer to @p BaseThermometer interface.
 * @param[out] axis     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t thermo_read_cooked(void *ip, float* axis) {
  LPS25HDriver* devp;
  int32_t raw;
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axis != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "thermo_read_cooked(), invalid state");

  msg = thermo_read_raw(devp, &raw);

  *axis = (raw * devp->thermosensitivity) - devp->thermobias;

  return msg;
}

/**
 * @brief   Set bias values for the BaseThermometer.
 * @note    Bias must be expressed as °C.
 * @note    The bias buffer must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] ip        pointer to @p BaseThermometer interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t thermo_set_bias(void *ip, float *bp) {
  LPS25HDriver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "thermo_set_bias(), invalid state");

  devp->thermobias = *bp;

  return msg;
}

/**
 * @brief   Reset bias values for the BaseThermometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseThermometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t thermo_reset_bias(void *ip) {
  LPS25HDriver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "thermo_reset_bias(), invalid state");

  devp->thermobias = LPS25H_THERMO_BIAS;

  return msg;
}

/**
 * @brief   Set sensitivity values for the BaseThermometer.
 * @note    Sensitivity must be expressed as °C/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] ip        pointer to @p BaseThermometer interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t thermo_set_sensitivity(void *ip, float *sp) {
  LPS25HDriver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (sp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "thermo_set_sensitivity(), invalid state");

  devp->thermosensitivity = *sp;

  return msg;
}

/**
 * @brief   Reset sensitivity values for the BaseThermometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseThermometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t thermo_reset_sensitivity(void *ip) {
  LPS25HDriver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(LPS25HDriver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == LPS25H_READY),
                "thermo_reset_sensitivity(), invalid state");

  devp->thermosensitivity = LPS25H_THERMO_SENS;

  return msg;
}

static const struct LPS25HVMT vmt_device = {
  (size_t)0
};

static const struct BaseBarometerVMT vmt_barometer = {
  sizeof(struct LPS25HVMT*),
  baro_get_axes_number, baro_read_raw, baro_read_cooked,
  baro_set_bias, baro_reset_bias, baro_set_sensitivity,
  baro_reset_sensitivity
};

static const struct BaseThermometerVMT vmt_thermometer = {
  sizeof(struct LPS25HVMT*) + sizeof(BaseBarometer),
  thermo_get_axes_number, thermo_read_raw, thermo_read_cooked,
  thermo_set_bias, thermo_reset_bias, thermo_set_sensitivity,
  thermo_reset_sensitivity
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] devp     pointer to the @p LPS25HDriver object
 *
 * @init
 */
void lps25hObjectInit(LPS25HDriver *devp) {

  devp->vmt = &vmt_device;
  devp->baro_if.vmt = &vmt_barometer;
  devp->thermo_if.vmt = &vmt_thermometer;

  devp->config = NULL;

  devp->baroaxes = LPS25H_BARO_NUMBER_OF_AXES;
  devp->thermoaxes = LPS25H_THERMO_NUMBER_OF_AXES;

  devp->state = LPS25H_STOP;
}

/**
 * @brief   Configures and activates LPS25H Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p LPS25HDriver object
 * @param[in] config    pointer to the @p LPS25HConfig object
 *
 * @api
 */
void lps25hStart(LPS25HDriver *devp, const LPS25HConfig *config) {
  uint8_t cr[2];
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == LPS25H_STOP) || (devp->state == LPS25H_READY),
                "lps25hStart(), invalid state");

  devp->config = config;

  /* Control register 1 configuration block.*/
  {
    cr[0] = LPS25H_AD_CTRL_REG1;
    cr[1] = devp->config->odr | LPS25H_CTRL_REG1_PD;
#if LPS25H_USE_ADVANCED || defined(__DOXYGEN__)
    cr[1] |= devp->config->bdu;
#endif
  }

#if  LPS25H_SHARED_I2C
  i2cAcquireBus((devp)->config->i2cp);
#endif /* LPS25H_SHARED_I2C */
  i2cStart((devp)->config->i2cp,
           (devp)->config->i2ccfg);

  lps25hI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress, cr, 1);

#if  LPS25H_SHARED_I2C
  i2cReleaseBus((devp)->config->i2cp);
#endif /* LPS25H_SHARED_I2C */

  /* Resolution configuration block.*/
  {
    cr[0] = LPS25H_AD_RES_CONF;
    cr[1] = 0x05;
#if LPS25H_USE_ADVANCED || defined(__DOXYGEN__)
    cr[1] = devp->config->baroresolution | devp->config->thermoresolution;
#endif

  }
#if  LPS25H_SHARED_I2C
  i2cAcquireBus((devp)->config->i2cp);
  i2cStart((devp)->config->i2cp,
           (devp)->config->i2ccfg);
#endif /* LPS25H_SHARED_I2C */

  lps25hI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                         cr, 1);

#if  LPS25H_SHARED_I2C
  i2cReleaseBus((devp)->config->i2cp);
#endif /* LPS25H_SHARED_I2C */

  if(devp->config->barosensitivity == NULL) {
    devp->barosensitivity = LPS25H_BARO_SENS;
  }
  else{
    /* Taking barometer sensitivity from user configurations */
    devp->barosensitivity = *(devp->config->barosensitivity);
  }

  if(devp->config->barobias == NULL) {
    devp->barobias = LPS25H_BARO_BIAS;
  }
  else{
    /* Taking barometer bias from user configurations */
    devp->barobias = *(devp->config->barobias);
  }

  if(devp->config->thermosensitivity == NULL) {
    devp->thermosensitivity = LPS25H_THERMO_SENS;
  }
  else{
    /* Taking thermometer sensitivity from user configurations */
    devp->thermosensitivity = *(devp->config->thermosensitivity);
  }

  if(devp->config->thermobias == NULL) {
    devp->thermobias = LPS25H_THERMO_BIAS;
  }
  else{
    /* Taking thermometer bias from user configurations */
    devp->thermobias = *(devp->config->thermobias);
  }

  /* This is the Barometer transient recovery time */
  osalThreadSleepMilliseconds(5);

  devp->state = LPS25H_READY;
}

/**
 * @brief   Deactivates the LPS25H Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p LPS25HDriver object
 *
 * @api
 */
void lps25hStop(LPS25HDriver *devp) {
  uint8_t cr[2];

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == LPS25H_STOP) || (devp->state == LPS25H_READY),
                "lps25hStop(), invalid state");

  if (devp->state == LPS25H_READY) {
#if  LPS25H_SHARED_I2C
    i2cAcquireBus((devp)->config->i2cp);
    i2cStart((devp)->config->i2cp,
             (devp)->config->i2ccfg);
#endif /* LPS25H_SHARED_I2C */

    cr[0] = LPS25H_AD_CTRL_REG1;
    cr[1] = 0;
    lps25hI2CWriteRegister(devp->config->i2cp, devp->config->slaveaddress,
                           cr, 1);

    i2cStop((devp)->config->i2cp);
#if  LPS25H_SHARED_I2C
    i2cReleaseBus((devp)->config->i2cp);
#endif /* LPS25H_SHARED_I2C */
  }
  devp->state = LPS25H_STOP;
}
/** @} */
