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
 * @file    hts221.c
 * @brief   HTS221 MEMS interface module code.
 *
 * @addtogroup HTS221
 * @ingroup EX_ST
 * @{
 */

#include "hal.h"
#include "hts221.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define HTS221_SEL(mask, offset)    (int16_t)(mask << offset)

#define HTS221_FLAG_HYGRO_BIAS      0x01
#define HTS221_FLAG_HYGRO_SENS      0x02
#define HTS221_FLAG_THERMO_BIAS     0x04
#define HTS221_FLAG_THERMO_SENS     0x08

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if (HTS221_USE_I2C) || defined(__DOXYGEN__)
/**
 * @brief   Reads registers value using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in]  i2cp      pointer to the I2C interface
 * @param[in]  reg       first sub-register address
 * @param[out] rxbuf     pointer to an output buffer
 * @param[in]  n         number of consecutive register to read
 * @return               the operation status.
 *
 * @notapi
 */
static msg_t hts221I2CReadRegister(I2CDriver *i2cp, uint8_t reg, uint8_t* rxbuf,
                             size_t n) {
  uint8_t txbuf = reg;
  if (n > 1)
    txbuf |= HTS221_SUB_MS;

  return i2cMasterTransmitTimeout(i2cp, HTS221_SAD, &txbuf, 1, rxbuf, n,
                                  TIME_INFINITE);
}

/**
 * @brief   Writes a value into a register using I2C.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] i2cp       pointer to the I2C interface
 * @param[in] txbuf      buffer containing sub-address value in first position
 *                       and values to write
 * @param[in] n          size of txbuf less one (not considering the first
 *                       element)
 * @return               the operation status.
 *
 * @notapi
 */
static msg_t hts221I2CWriteRegister(I2CDriver *i2cp, uint8_t* txbuf, size_t n) {
  if (n > 1)
    (*txbuf) |= HTS221_SUB_MS;

  return i2cMasterTransmitTimeout(i2cp, HTS221_SAD, txbuf, n + 1, NULL, 0,
                                  TIME_INFINITE);
}
#endif /* HTS221_USE_I2C */

/**
 * @brief   Computes biases and sensitivities starting from data stored in
 *          calibration registers.
 * @note    Factory bias and sensitivity values are stored into the driver
 *          structure.
 *
 * @param[in] devp       pointer to the HTS221 interface
 * @return               the operation status.
 *
 * @notapi
 */
static msg_t hts221Calibrate(HTS221Driver *devp) {
  msg_t msg;
  uint8_t calib[16], H0_rH_x2, H1_rH_x2, msb;
  int16_t H0_T0_OUT, H1_T0_OUT, T0_degC_x8, T1_degC_x8, T0_OUT, T1_OUT;

  /* Retrieving rH values from Calibration registers */
  msg = hts221I2CReadRegister(devp->config->i2cp,
                               HTS221_AD_CALIB_0, calib, 16);

  H0_rH_x2 = calib[0];
  H1_rH_x2 = calib[1];
  H0_T0_OUT = calib[6];
  H0_T0_OUT += calib[7] << 8;
  H1_T0_OUT = calib[10];
  H1_T0_OUT += calib[11] << 8;

  T0_degC_x8 = calib[2];

  /* Completing T0_degC_x8 value */
  msb = (calib[5] & HTS221_SEL(0x03, 0));
  if (msb & HTS221_SEL(0x01, 1)) {
    msb |= HTS221_SEL(0x3F, 2);
  }
  T0_degC_x8 += msb << 8;

  T1_degC_x8 = calib[3];
  /* Completing T1_degC_x8 value */
  msb = ((calib[5] & HTS221_SEL(0x03, 2)) >> 2);
  if (msb & HTS221_SEL(0x01, 1)) {
    msb |= HTS221_SEL(0x3F, 2);
  }
  T1_degC_x8 += msb << 8;

  T0_OUT = calib[12];
  T0_OUT += calib[13] << 8;
  T1_OUT = calib[14];
  T1_OUT += calib[15] << 8;

  devp->hygrofactorysensitivity = ((float)H1_rH_x2 - (float)H0_rH_x2) /
                                  (((float)H1_T0_OUT - (float)H0_T0_OUT) * 2.0f);


  devp->hygrofactorybias = (devp->hygrofactorysensitivity * (float)H0_T0_OUT) -
                           ((float)H0_rH_x2 / 2.0f);

  devp->thermofactorysensitivity = ((float)T1_degC_x8 - (float)T0_degC_x8) /
                                   (((float)T1_OUT - (float)T0_OUT) * 8.0f);

  devp->thermofactorybias = (devp->thermofactorysensitivity * (float)T0_OUT) -
                            ((float)T0_degC_x8 / 8.0f);

  return msg;
}

/**
 * @brief   Return the number of axes of the BaseHygrometer.
 *
 * @param[in] ip        pointer to @p BaseHygrometer interface.
 *
 * @return              the number of axes.
 */
static size_t hygro_get_axes_number(void *ip) {
  (void)ip;

  return HTS221_HYGRO_NUMBER_OF_AXES;
}

/**
 * @brief   Retrieves raw data from the BaseHygrometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseHygrometer axes number.
 *
 * @param[in] ip        pointer to @p BaseHygrometer interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t hygro_read_raw(void *ip, int32_t axes[]) {
  HTS221Driver* devp;
  uint8_t buff[2];
  int16_t tmp;
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseHygrometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
              "hygro_read_raw(), invalid state");

  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "hygro_read_raw(), channel not ready");

#if HTS221_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* HTS221_SHARED_I2C */

  msg = hts221I2CReadRegister(devp->config->i2cp, HTS221_AD_HUMIDITY_OUT_L,
                              buff, 2);

#if HTS221_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* HTS221_SHARED_I2C */

  if (msg == MSG_OK) {
    tmp = buff[0] + (buff[1] << 8);
    *axes = (int32_t)tmp;
  }
  return msg;
}

/**
 * @brief   Retrieves cooked data from the BaseHygrometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as %rH.
 * @note    The axes array must be at least the same size of the
 *          BaseHygrometer axes number.
 *
 * @param[in] ip        pointer to @p BaseHygrometer interface.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t hygro_read_cooked(void *ip, float axes[]) {
  HTS221Driver* devp;
  int32_t raw;
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseHygrometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
                "hygro_read_cooked(), invalid state");

  msg = hygro_read_raw(ip, &raw);

  *axes = (raw * devp->hygrosensitivity) - devp->hygrobias;

  return msg;
}

/**
 * @brief   Set bias values for the BaseHygrometer.
 * @note    Bias must be expressed as %rH.
 * @note    The bias buffer must be at least the same size of the
 *          BaseHygrometer axes number.
 *
 * @param[in] ip        pointer to @p BaseHygrometer interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
static msg_t hygro_set_bias(void *ip, float *bp) {
  HTS221Driver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseHygrometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
                "hygro_set_bias(), invalid state");

  devp->hygrobias = *bp;
  return msg;
}

/**
 * @brief   Reset bias values for the BaseHygrometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] ip        pointer to @p BaseHygrometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t hygro_reset_bias(void *ip) {
  HTS221Driver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseHygrometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
                "hygro_reset_bias(), invalid state");

  devp->hygrobias = devp->hygrofactorybias;
  return msg;
}

/**
 * @brief   Set sensitivity values for the BaseHygrometer.
 * @note    Sensitivity must be expressed as %rH/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseHygrometer axes number.
 *
 * @param[in] ip        pointer to @p BaseHygrometer interface.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t hygro_set_sensitivity(void *ip, float *sp) {
  HTS221Driver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (sp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseHygrometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
                "hygro_set_sensitivity(), invalid state");

  devp->hygrosensitivity = *sp;
  return msg;
}

/**
 * @brief   Reset sensitivity values for the BaseHygrometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] ip        pointer to @p BaseHygrometer interface.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 */
static msg_t hygro_reset_sensitivity(void *ip) {
  HTS221Driver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

    /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseHygrometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
                "hygro_reset_sensitivity(), invalid state");

  devp->hygrosensitivity = devp->hygrofactorysensitivity;
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

  return HTS221_THERMO_NUMBER_OF_AXES;
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
  HTS221Driver* devp;
  int16_t tmp;
  uint8_t buff[2];
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axes != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
                "thermo_read_raw(), invalid state");

  osalDbgAssert((devp->config->i2cp->state == I2C_READY),
                "thermo_read_raw(), channel not ready");

#if HTS221_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp,
           devp->config->i2ccfg);
#endif /* HTS221_SHARED_I2C */

  msg = hts221I2CReadRegister(devp->config->i2cp, HTS221_AD_TEMP_OUT_L,
                              buff, 2);

#if HTS221_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* HTS221_SHARED_I2C */

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
  HTS221Driver* devp;
  int32_t raw;
  msg_t msg;

  osalDbgCheck((ip != NULL) && (axis != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
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
  HTS221Driver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (bp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
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
  HTS221Driver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
                "thermo_reset_bias(), invalid state");

  devp->thermobias = devp->thermofactorybias;

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
  HTS221Driver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck((ip != NULL) && (sp != NULL));

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
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
  HTS221Driver* devp;
  msg_t msg = MSG_OK;

  osalDbgCheck(ip != NULL);

  /* Getting parent instance pointer.*/
  devp = objGetInstance(HTS221Driver*, (BaseThermometer*)ip);

  osalDbgAssert((devp->state == HTS221_READY),
                "thermo_reset_sensitivity(), invalid state");

  devp->thermosensitivity = devp->thermofactorysensitivity;

  return msg;
}

static const struct HTS221VMT vmt_device = {
  (size_t)0
};

static const struct BaseHygrometerVMT vmt_hygrometer = {
  sizeof(struct HTS221VMT*),
  hygro_get_axes_number, hygro_read_raw, hygro_read_cooked,
  hygro_set_bias, hygro_reset_bias, hygro_set_sensitivity,
  hygro_reset_sensitivity
};

static const struct BaseThermometerVMT vmt_thermometer = {
  sizeof(struct HTS221VMT*) + sizeof(BaseHygrometer),
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
 * @param[out] devp     pointer to the @p HTS221Driver object
 *
 * @init
 */
void hts221ObjectInit(HTS221Driver *devp) {

  devp->vmt = &vmt_device;
  devp->hygro_if.vmt = &vmt_hygrometer;
  devp->thermo_if.vmt = &vmt_thermometer;

  devp->config = NULL;

  devp->hygroaxes = HTS221_HYGRO_NUMBER_OF_AXES;
  devp->thermoaxes = HTS221_THERMO_NUMBER_OF_AXES;

  devp->hygrobias = 0.0f;
  devp->thermobias = 0.0f;

  devp->state = HTS221_STOP;
}

/**
 * @brief   Configures and activates HTS221 Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p HTS221Driver object
 * @param[in] config    pointer to the @p HTS221Config object
 *
 * @api
 */
void hts221Start(HTS221Driver *devp, const HTS221Config *config) {
  uint8_t cr[2];
  osalDbgCheck((devp != NULL) && (config != NULL));

  osalDbgAssert((devp->state == HTS221_STOP) || (devp->state == HTS221_READY),
                 "hts221Start(), invalid state");

  devp->config = config;

#if HTS221_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
#endif /* HTS221_SHARED_I2C */

  /* Intializing the I2C. */
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);

  hts221Calibrate(devp);

#if HTS221_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
#endif /* HTS221_SHARED_I2C */


  if(devp->config->hygrosensitivity == NULL) {
    devp->hygrosensitivity = devp->hygrofactorysensitivity;
  }
  else{
    /* Taking hygrometer sensitivity from user configurations */
    devp->hygrosensitivity = *(devp->config->hygrosensitivity);
  }

  if(devp->config->hygrobias == NULL) {
    devp->hygrobias = devp->hygrofactorybias;
  }
  else{
    /* Taking hygrometer bias from user configurations */
    devp->hygrobias = *(devp->config->hygrobias);
  }

  if(devp->config->thermosensitivity == NULL) {
    devp->thermosensitivity = devp->thermofactorysensitivity;
  }
  else{
    /* Taking thermometer sensitivity from user configurations */
    devp->thermosensitivity = *(devp->config->thermosensitivity);
  }

  if(devp->config->thermobias == NULL) {
    devp->thermobias = devp->thermofactorybias;
  }
  else{
    /* Taking thermometer bias from user configurations */
    devp->thermobias = *(devp->config->thermobias);
  }

  /* Control register 1 configuration block.*/
  {
    cr[0] = HTS221_AD_CTRL_REG1;
    cr[1] = devp->config->outputdatarate | HTS221_CTRL_REG1_PD;
#if HTS221_USE_ADVANCED || defined(__DOXYGEN__)
    cr[1] |= devp->config->blockdataupdate;
#endif

#if HTS221_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* HTS221_SHARED_I2C */

  hts221I2CWriteRegister(devp->config->i2cp, cr, 1);

#if HTS221_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* HTS221_SHARED_I2C */
  }

  /* Average register configuration block.*/
  {
    cr[0] = HTS221_AD_AV_CONF;
    cr[1] = 0x05;
#if HTS221_USE_ADVANCED || defined(__DOXYGEN__)
    cr[1] = devp->config->hygroresolution | devp->config->thermoresolution;
#endif

#if HTS221_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* HTS221_SHARED_I2C */

    hts221I2CWriteRegister(devp->config->i2cp, cr, 1);

#if HTS221_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
#endif /* HTS221_SHARED_I2C */
  }

  /* This is the MEMS transient recovery time */
  osalThreadSleepMilliseconds(5);

  devp->state = HTS221_READY;
}

/**
 * @brief   Deactivates the HTS221 Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p HTS221Driver object
 *
 * @api
 */
void hts221Stop(HTS221Driver *devp) {
  uint8_t cr[2];

  osalDbgCheck(devp != NULL);

  osalDbgAssert((devp->state == HTS221_STOP) || (devp->state == HTS221_READY),
                "hts221Stop(), invalid state");

  if (devp->state == HTS221_READY) {

#if HTS221_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* HTS221_SHARED_I2C */

  cr[0] = HTS221_AD_CTRL_REG1;
  cr[1] = 0;
  hts221I2CWriteRegister(devp->config->i2cp, cr, 1);

  i2cStop(devp->config->i2cp);
#if HTS221_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* HTS221_SHARED_I2C */
  }
  devp->state = HTS221_STOP;
}
/** @} */
