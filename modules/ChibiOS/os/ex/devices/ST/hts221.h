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
 * @file    hts221.h
 * @brief   HTS221 MEMS interface module header.
 *
 *
 * @addtogroup HTS221
 * @ingroup EX_ST
 * @{
 */
#ifndef _HTS221_H_
#define _HTS221_H_

#include "ex_hygrometer.h"
#include "ex_thermometer.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   HTS221 driver version string.
 */
#define EX_HTS221_VERSION                   "1.1.2"

/**
 * @brief   HTS221 driver version major number.
 */
#define EX_HTS221_MAJOR                     1

/**
 * @brief   HTS221 driver version minor number.
 */
#define EX_HTS221_MINOR                     1

/**
 * @brief   HTS221 driver version patch number.
 */
#define EX_HTS221_PATCH                     2
/** @} */

/**
 * @brief   HTS221 hygrometer subsystem characteristics.
 * @note    Sensitivity is expressed as %rH/LSB whereas %rH stand for percentage
 *          of relative humidity.
 * @note    Bias is expressed as %rH.
 * @{
 */
#define HTS221_HYGRO_NUMBER_OF_AXES         1U

#define HTS221_HYGRO_SENS                   0.00390625f
#define HTS221_HYGRO_BIAS                   0.0f
/** @} */

/**
 * @brief   HTS221 thermometer subsystem characteristics.
 * @note    Sensitivity is expressed as �C/LSB.
 * @note    Bias is expressed as �C.
 *
 * @{
 */
#define HTS221_THERMO_NUMBER_OF_AXES        1U

#define HTS221_THERMO_SENS                  0.0015625f
#define HTS221_THERMO_BIAS                  0.0f
/** @} */

/**
 * @name    HTS221 communication interfaces related bit masks
 * @{
 */
#define HTS221_DI_MASK                      0xFF
#define HTS221_DI(n)                        (1 << n)
#define HTS221_AD_MASK                      0x3F
#define HTS221_AD(n)                        (1 << n)
#define HTS221_MS                           (1 << 6)
#define HTS221_RW                           (1 << 7)

#define HTS221_SUB_MS                       (1 << 7)

#define HTS221_SAD                          0x5F
/** @} */

/**
 * @name    HTS221 register addresses
 * @{
 */
#define HTS221_AD_WHO_AM_I                  0x0F
#define HTS221_AD_AV_CONF                   0x10
#define HTS221_AD_CTRL_REG1                 0x20
#define HTS221_AD_CTRL_REG2                 0x21
#define HTS221_AD_CTRL_REG3                 0x22
#define HTS221_AD_STATUS_REG                0x27
#define HTS221_AD_HUMIDITY_OUT_L            0x28
#define HTS221_AD_HUMIDITY_OUT_H            0x29
#define HTS221_AD_TEMP_OUT_L                0x2A
#define HTS221_AD_TEMP_OUT_H                0x2B
#define HTS221_AD_CALIB_0                   0x30
#define HTS221_AD_CALIB_1                   0x31
#define HTS221_AD_CALIB_2                   0x32
#define HTS221_AD_CALIB_3                   0x33
#define HTS221_AD_CALIB_4                   0x34
#define HTS221_AD_CALIB_5                   0x35
#define HTS221_AD_CALIB_6                   0x36
#define HTS221_AD_CALIB_7                   0x37
#define HTS221_AD_CALIB_8                   0x38
#define HTS221_AD_CALIB_9                   0x39
#define HTS221_AD_CALIB_A                   0x3A
#define HTS221_AD_CALIB_B                   0x3B
#define HTS221_AD_CALIB_C                   0x3C
#define HTS221_AD_CALIB_D                   0x3D
#define HTS221_AD_CALIB_E                   0x3E
#define HTS221_AD_CALIB_F                   0x3F
/** @} */

/**
 * @name    HTS221_CTRL_REG1 register bits definitions
 * @{
 */
#define HTS221_CTRL_REG1_MASK               0x87
#define HTS221_CTRL_REG1_ODR0               (1 << 0)
#define HTS221_CTRL_REG1_ODR1               (1 << 1)
#define HTS221_CTRL_REG1_BDU                (1 << 2)
#define HTS221_CTRL_REG1_PD                 (1 << 7)
/** @} */

/**
 * @name    HTS221_CTRL_REG2 register bits definitions
 * @{
 */
#define HTS221_CTRL_REG2_MASK               0x83
#define HTS221_CTRL_REG2_ONE_SHOT           (1 << 0)
#define HTS221_CTRL_REG2_HEATER             (1 << 1)
#define HTS221_CTRL_REG2_BOOT               (1 << 7)
/** @} */

/**
 * @name    HTS221_CTRL_REG3 register bits definitions
 * @{
 */
#define HTS221_CTRL_REG3_MASK               0xC4
#define HTS221_CTRL_REG3_DRDY               (1 << 2)
#define HTS221_CTRL_REG3_PP_OD              (1 << 6)
#define HTS221_CTRL_REG3_INT_H_L            (1 << 7)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   HTS221 SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p FALSE.
 */
#if !defined(HTS221_USE_SPI) || defined(__DOXYGEN__)
#define HTS221_USE_SPI                      FALSE
#endif

/**
 * @brief   HTS221 shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION
 */
#if !defined(HTS221_SHARED_SPI) || defined(__DOXYGEN__)
#define HTS221_SHARED_SPI                   FALSE
#endif

/**
 * @brief   HTS221 I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(HTS221_USE_I2C) || defined(__DOXYGEN__)
#define HTS221_USE_I2C                      TRUE
#endif

/**
 * @brief   HTS221 shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION
 */
#if !defined(HTS221_SHARED_I2C) || defined(__DOXYGEN__)
#define HTS221_SHARED_I2C                   FALSE
#endif

/**
 * @brief   HTS221 advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(HTS221_USE_ADVANCED) || defined(__DOXYGEN__)
#define HTS221_USE_ADVANCED                 FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(HTS221_USE_SPI ^ HTS221_USE_I2C)
#error "HTS221_USE_SPI and HTS221_USE_I2C cannot be both true or both false"
#endif

#if HTS221_USE_SPI && !HAL_USE_SPI
#error "HTS221_USE_SPI requires HAL_USE_SPI"
#endif

#if HTS221_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "HTS221_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if HTS221_USE_I2C && !HAL_USE_I2C
#error "HTS221_USE_I2C requires HAL_USE_I2C"
#endif

#if HTS221_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "HTS221_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for HTS221 over SPI.
 */
#if HTS221_USE_SPI
#error "HTS221 over SPI still not supported."
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    HTS221 data structures and types.
 * @{
 */
/**
 * @brief   Structure representing a HTS221 driver.
 */
typedef struct HTS221Driver HTS221Driver;

/**
 * @brief   HTS221 output data rate and bandwidth.
 */
typedef enum {
  HTS221_ODR_ONE_SHOT = 0x00,       /**< One shot.                          */
  HTS221_ODR_1HZ = 0x01,            /**< Output data rate 1 Hz.             */
  HTS221_ODR_7HZ = 0x02,            /**< Output data rate 7 Hz.             */
  HTS221_ODR_12P5HZ = 0x03,         /**< Output data rate 12.5 Hz.          */
}hts221_odr_t;

/**
 * @brief   HTS221 humidity resolution.
 */
typedef enum {
  HTS221_AVGH_4 = 0x00,             /**< Number of internal average is 4.   */
  HTS221_AVGH_8 = 0x01,             /**< Number of internal average is 8.   */
  HTS221_AVGH_16 = 0x02,            /**< Number of internal average is 16.  */
  HTS221_AVGH_32 = 0x03,            /**< Number of internal average is 32.  */
  HTS221_AVGH_64 = 0x04,            /**< Number of internal average is 64.  */
  HTS221_AVGH_128 = 0x05,           /**< Number of internal average is 128. */
  HTS221_AVGH_256 = 0x06,           /**< Number of internal average is 256. */
  HTS221_AVGH_512 = 0x07            /**< Number of internal average is 512. */
}hts221_avgh_t;

/**
 * @brief   HTS221 temperature resolution.
 */
typedef enum {
  HTS221_AVGT_2 = 0x00,             /**< Number of internal average is 2.   */
  HTS221_AVGT_4 = 0x08,             /**< Number of internal average is 4.   */
  HTS221_AVGT_8 = 0x10,             /**< Number of internal average is 8.   */
  HTS221_AVGT_16 = 0x18,            /**< Number of internal average is 16.  */
  HTS221_AVGT_32 = 0x20,            /**< Number of internal average is 32.  */
  HTS221_AVGT_64 = 0x28,            /**< Number of internal average is 64.  */
  HTS221_AVGT_128 = 0x30,           /**< Number of internal average is 128. */
  HTS221_AVGT_256 = 0x38,           /**< Number of internal average is 256. */
}hts221_avgt_t;

/**
 * @brief   HTS221 block data update.
 */
typedef enum {
  HTS221_BDU_CONTINUOUS = 0x00,     /**< Block data continuously updated.   */
  HTS221_BDU_BLOCKED = 0x40         /**< Block data updated after reading.  */
}hts221_bdu_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  HTS221_UNINIT = 0,                /**< Not initialized.                   */
  HTS221_STOP = 1,                  /**< Stopped.                           */
  HTS221_READY = 2,                 /**< Ready.                             */
} hts221_state_t;

/**
 * @brief   HTS221 configuration structure.
 */
typedef struct {

#if HTS221_USE_SPI || defined(__DOXYGEN__)
  /**
   * @brief SPI driver associated to this HTS221.
   */
  SPIDriver                 *spip;
  /**
   * @brief SPI configuration associated to this HTS221.
   */
  const SPIConfig           *spicfg;
#endif /* HTS221_USE_SPI */
#if HTS221_USE_I2C || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this HTS221.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this HTS221.
   */
  const I2CConfig           *i2ccfg;
#endif /* HTS221_USE_I2C */
  /**
   * @brief HTS221 hygrometer subsystem initial sensitivity.
   */
  float                     *hygrosensitivity;
  /**
   * @brief HTS221 hygrometer subsystem initial bias.
   */
  float                     *hygrobias;
  /**
   * @brief HTS221 thermometer subsystem initial sensitivity.
   */
  float                     *thermosensitivity;
  /**
   * @brief HTS221 thermometer subsystem initial bias.
   */
  float                     *thermobias;
  /**
   * @brief HTS221 output data rate selection.
   */
  hts221_odr_t              outputdatarate;
#if HTS221_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief HTS221 block data update.
   */
  hts221_bdu_t              blockdataupdate;
  /**
   * @brief HTS221 hygrometer subsystem resolution.
   */
  hts221_avgh_t             hygroresolution;
  /**
   * @brief HTS221 thermometer subsystem resolution.
   */
  hts221_avgt_t             thermoresolution;
#endif
} HTS221Config;

/**
 * @brief   @p HTS221 specific methods.
 * @note    No methods so far, just a common ancestor interface.
 */
#define _hts221_methods_alone

/**
 * @brief   @p HTS221 specific methods with inherited ones.
 */
#define _hts221_methods                                                     \
  _base_object_methods                                                      \
  _hts221_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief @p HTS221 virtual methods table.
 */
struct HTS221VMT {
  _hts221_methods
};

/**
 * @brief   @p HTS221Driver specific data.
 */
#define _hts221_data                                                        \
  /* Driver state.*/                                                        \
  hts221_state_t            state;                                          \
  /* Current configuration data.*/                                          \
  const HTS221Config        *config;                                        \
  /* Hygrometer subsystem axes number.*/                                    \
  size_t                    hygroaxes;                                      \
  /* Hygrometer subsystem current sensitivity.*/                            \
  float                     hygrosensitivity;                               \
  /* Hygrometer subsystem current bias .*/                                  \
  float                     hygrobias;                                      \
  /* Hygrometer subsystem factory sensitivity.*/                            \
  float                     hygrofactorysensitivity;                        \
  /* Hygrometer subsystem factory bias .*/                                  \
  float                     hygrofactorybias;                               \
  /* Thermometer subsystem axes number.*/                                   \
  size_t                    thermoaxes;                                     \
  /* Thermometer subsystem current sensitivity.*/                           \
  float                     thermosensitivity;                              \
  /* Thermometer subsystem current bias.*/                                  \
  float                     thermobias;                                     \
  /* Thermometer subsystem factory sensitivity.*/                           \
  float                     thermofactorysensitivity;                       \
  /* Thermometer subsystem factory bias.*/                                  \
  float                     thermofactorybias;

/**
 * @brief   HTS221 2-axis hygrometer/thermometer class.
 */
struct HTS221Driver {
  /** @brief Virtual Methods Table.*/
  const struct HTS221VMT    *vmt;
  /** @brief Base hygrometer interface.*/
  BaseHygrometer            hygro_if;
  /** @brief Base thermometer interface.*/
  BaseThermometer           thermo_if;
  _hts221_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseHygrometer.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define hts221HygrometerGetAxesNumber(devp)                                 \
        hygrometerGetAxesNumber(&((devp)->hygro_if))

/**
 * @brief   Retrieves raw data from the BaseHygrometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseHygrometer axes number.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define hts221HygrometerReadRaw(devp, axes)                                 \
        hygrometerReadRaw(&((devp)->hygro_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseHygrometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as %rH.
 * @note    The axes array must be at least the same size of the
 *          BaseHygrometer axes number.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define hts221HygrometerReadCooked(devp, axes)                              \
        hygrometerReadCooked(&((devp)->hygro_if), axes)

/**
 * @brief   Set bias values for the BaseHygrometer.
 * @note    Bias must be expressed as %rH.
 * @note    The bias buffer must be at least the same size of the
 *          BaseHygrometer axes number.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define hts221HygrometerSetBias(devp, bp)                                   \
        hygrometerSetBias(&((devp)->hygro_if), bp)

/**
 * @brief   Reset bias values for the BaseHygrometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define hts221HygrometerResetBias(devp)                                     \
        hygrometerResetBias(&((devp)->hygro_if))

/**
 * @brief   Set sensitivity values for the BaseHygrometer.
 * @note    Sensitivity must be expressed as %rH/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseHygrometer axes number.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define hts221HygrometerSetSensitivity(devp, sp)                            \
        hygrometerSetSensitivity(&((devp)->hygro_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseHygrometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define hts221HygrometerResetSensitivity(devp)                              \
        hygrometerResetSensitivity(&((devp)->hygro_if))

/**
 * @brief   Return the number of axes of the BaseThermometer.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define hts221ThermometerGetAxesNumber(devp)                                \
        thermometerGetAxesNumber(&((devp)->thermo_if))

/**
 * @brief   Retrieves raw data from the BaseThermometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define hts221ThermometerReadRaw(devp, axes)                                \
        thermometerReadRaw(&((devp)->thermo_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseThermometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as �C.
 * @note    The axes array must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define hts221ThermometerReadCooked(devp, axes)                             \
        thermometerReadCooked(&((devp)->thermo_if), axes)

/**
 * @brief   Set bias values for the BaseThermometer.
 * @note    Bias must be expressed as �C.
 * @note    The bias buffer must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define hts221ThermometerSetBias(devp, bp)                                  \
        thermometerSetBias(&((devp)->thermo_if), bp)

/**
 * @brief   Reset bias values for the BaseThermometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define hts221ThermometerResetBias(devp)                                    \
        thermometerResetBias(&((devp)->thermo_if))

/**
 * @brief   Set sensitivity values for the BaseThermometer.
 * @note    Sensitivity must be expressed as �C/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define hts221ThermometerSetSensitivity(devp, sp)                           \
        thermometerSetSensitivity(&((devp)->thermo_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseThermometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p HTS221Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define hts221ThermometerResetSensitivity(devp)                             \
        thermometerResetSensitivity(&((devp)->thermo_if))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void hts221ObjectInit(HTS221Driver *devp);
  void hts221Start(HTS221Driver *devp, const HTS221Config *config);
  void hts221Stop(HTS221Driver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _HTS221_H_ */

/** @} */
