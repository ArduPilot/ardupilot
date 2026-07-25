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
 * @file    bmp085.h
 * @brief   BMP085 Digital pressure sensor interface module header.
 *
 * @addtogroup BMP085
 * @ingroup EX_BOSCH
 * @{
 */

#ifndef BMP085_H
#define BMP085_H

#include "ex_barometer.h"
#include "ex_thermometer.h"

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/**
 * @name  Version identification
 * @{
 */

/**
 * @brief   BMP085 driver version string.
 */
#define EX_BMP085_VERSION           "1.0.1"

/**
 * @brief   BMP085 driver version major number.
 */
#define EX_BMP085_MAJOR             1

/**
 * @brief   BMP085 driver version minor number.
 */
#define EX_BMP085_MINOR             0

/**
 * @brief   BMP085 driver version patch number.
 */
#define EX_BMP085_PATCH             1
/** @}*/

/**
 * @brief   BMP085 barometer subsystem characteristics.
 * @{
 */
#define BMP085_BARO_NUMBER_OF_AXES  1U    /**< Number of axes               */

#define BMP085_P_RES                0.01  /**< LSB/hP                       */
/** @} */

/**
 * @brief   BMP085 thermometer subsystem characteristics.
 * @{
 */
#define BMP085_THERMO_NUMBER_OF_AXES 1U  /**< Number of axes                */

#define BMP085_T_RES                0.1   /**< LSB/C°                       */
/** @} */

/**
 * @name  BMP085 Registers addresses.
 * @{
 */
#define BMP085_AD_CR                0xF4 /**< Control register address.     */
#define BMP085_AD_T_DR_MSB          0xF6 /**< Temp MSB data register addr.  */
#define BMP085_AD_T_DR_LSB          0xF7 /**< Temp LSB data register addr.  */
#define BMP085_AD_P_DR_MSB          0xF6 /**< Press MSB data register addr. */
#define BMP085_AD_P_DR_LSB          0xF7 /**< Press LSB data register addr. */
#define BMP085_AD_P_DR_XLSB         0xF8 /**< Press XLSB data register addr.*/
#define BMP085_AD_CC_AC1_MSB        0xAA /**< AC1 MSB calib coef reg addr.  */
#define BMP085_AD_CC_AC1_LSB        0xAB /**< AC1 LSB calib coef reg addr.  */
#define BMP085_AD_CC_AC2_MSB        0xAC /**< AC2 MSB calib coef reg addr.  */
#define BMP085_AD_CC_AC2_LSB        0xAD /**< AC2 LSB calib coef reg addr.  */
#define BMP085_AD_CC_AC3_MSB        0xAE /**< AC3 MSB calib coef reg addr.  */
#define BMP085_AD_CC_AC3_LSB        0xAF /**< AC3 LSB calib coef reg addr.  */
#define BMP085_AD_CC_AC4_MSB        0xB0 /**< AC4 MSB calib coef reg addr.  */
#define BMP085_AD_CC_AC4_LSB        0xB1 /**< AC4 LSB calib coef reg addr.  */
#define BMP085_AD_CC_AC5_MSB        0xB2 /**< AC5 MSB calib coef reg addr.  */
#define BMP085_AD_CC_AC5_LSB        0xB3 /**< AC5 LSB calib coef reg addr.  */
#define BMP085_AD_CC_AC6_MSB        0xB4 /**< AC6 MSB calib coef reg addr.  */
#define BMP085_AD_CC_AC6_LSB        0xB5 /**< AC6 LSB calib coef reg addr.  */
#define BMP085_AD_CC_B1_MSB         0xB6 /**< B1 MSB calib coef reg addr.   */
#define BMP085_AD_CC_B1_LSB         0xB7 /**< B1 LSB calib coef reg addr.   */
#define BMP085_AD_CC_B2_MSB         0xB8 /**< B2 MSB calib coef reg addr.   */
#define BMP085_AD_CC_B2_LSB         0xB9 /**< B2 LSB calib coef reg addr.   */
#define BMP085_AD_CC_MB_MSB         0xBA /**< MB MSB calib coef reg addr.   */
#define BMP085_AD_CC_MB_LSB         0xBB /**< MB LSB calib coef reg addr.   */
#define BMP085_AD_CC_MC_MSB         0xBC /**< MC MSB calib coef reg addr.   */
#define BMP085_AD_CC_MC_LSB         0xBD /**< MC LSB calib coef reg addr.   */
#define BMP085_AD_CC_MD_MSB         0xBE /**< MD MSB calib coef reg addr.   */
#define BMP085_AD_CC_MD_LSB         0xBF /**< MD LSB calib coef reg addr.   */
/** @} */

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    Configuration options
 * @{
 */

/**
 * @brief   BMP085 I2C interface selector.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(BMP085_USE_I2C) || defined(__DOXYGEN__)
#define BMP085_USE_I2C                     TRUE
#endif

/**
 * @brief   BMP085 sensor subsystem advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p TRUE.
 */
#if !defined(BMP085_USE_ADVANCED) || defined(__DOXYGEN__)
#define BMP085_USE_ADVANCED            TRUE
#endif

/**
 * @brief   BMP085 shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(BMP085_SHARED_I2C) || defined(__DOXYGEN__)
#define BMP085_SHARED_I2C                  FALSE
#endif
/** @} */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

#if !HAL_USE_I2C
#error "BMP085_USE_I2C requires HAL_USE_I2C"
#endif

#if BMP085_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "BMP085_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @name  BMP085 barometer subsystem data structures and types.
 * @{
 */

/**
 * @brief BMP085 barometer subsystem pressure conversion time.
 */
typedef enum {
  BMP085_BARO_CT_LOW  = 0x05,   /**< Convers time in ultra low power mode.  */
  BMP085_BARO_CT_STD  = 0x18,   /**< Convers time in standard mode.         */
  BMP085_BARO_CT_HR   = 0x0E,   /**< Convers time in high resolution mode.  */
  BMP085_BARO_CT_LUHR = 0x1A    /**< Convers time in ultra high res. mode.  */
} bmp085_baro_ct_t;

/**
 * @brief BMP085 barometer subsystem mode.
 */
typedef enum {
  BMP085_BARO_MODE_LOW  = 0x00, /**< BMP085 ultra low power mode.           */
  BMP085_BARO_MODE_STD  = 0x01, /**< BMP085 standard mode.                  */
  BMP085_BARO_MODE_HR   = 0x02, /**< BMP085 high resolution mode.           */
  BMP085_BARO_MODE_LUHR = 0x03  /**< BMP085 ultra high res. mode.           */
} bmp085_baro_mode_t;

/**
 * @brief BMP085 barometer oversampling setting.
 */
typedef enum {
  BMP085_BARO_OSS_0 = 0x00,     /**< Ultra low power sampling rate.         */
  BMP085_BARO_OSS_1 = 0x01,     /**< Standard mode sampling rate.           */
  BMP085_BARO_OSS_2 = 0x02,     /**< High resolution sampling rate.         */
  BMP085_BARO_OSS_3 = 0x04      /**< ultra high resolution sampling rate.   */
}bmp085_baro_oss_t;

/**
 * @brief BMP085 barometer subsystem calibration data.
 */
typedef struct {
  int16_t   ac1;
  int16_t   ac2;
  int16_t   ac3;
  int16_t   b1;
  int16_t   b2;
  int16_t   mb;
  int16_t   mc;
  int16_t   md;
  uint16_t  ac4;
  uint16_t  ac5;
  uint16_t  ac6;
  int32_t   b5;
} bmp085_cd_t;

/** @} */

/**
 * @name  BMP085 thermometer subsystem data structures and types.
 * @{
 */

/**
 * @brief BMP085 thermometer subsystem temperature conversion time.
 */
typedef enum {
  BMP085_THERMO_CT_LOW  = 0x05, /**< Conv time in ultra low power mode.     */
  BMP085_THERMO_CT_STD  = 0x18, /**< Conv time in standard mode.            */
  BMP085_THERMO_CT_HR   = 0x0E, /**< Conv time in high resolution mode.     */
  BMP085_THERMO_CT_LUHR = 0x1A  /**< Conv time in ultra high res. mode.     */
} bmp085_thermo_ct_t;

/** @} */

/**
 * @name  BMP085 main system data structures and types.
 * @{
 */

/**
 * @brief Driver state machine possible states.
 */
typedef enum {
  BMP085_UNINIT = 0,  /**< Not initialized.                                 */
  BMP085_STOP   = 1,  /**< Stopped.                                         */
  BMP085_READY  = 2   /**< Ready.                                           */
} bmp085_state_t;

/**
 * @brief BMP085 configuration structure.
 */
typedef struct {
#if BMP085_USE_I2C || defined(__DOXYGEN__)
 /**
  * @brief I2C driver associated to this BMP085.
  */
  I2CDriver                 *i2cp;

  /**
   * @brief I2C configuration associated to this BMP085 subsystem.
   */
  const I2CConfig           *i2ccfg;
#endif /* BMP085_USE_I2C */
  /**
   * @brief HTS221 initial sensitivity.
   * @note  Value are respectively related to hygrometer
   *        and thermometer.
   */
  float*                    sensitivity;
  /**
   * @brief HTS221 initial bias.
   * @note  Value are respectively related to hygrometer
   *        and thermometer.
   */
  float*                    bias;
  /**
   * @brief HTS221 output data rate selection.
   */
  float*                    outputdatarate;
#if BMP085_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief BMP085 barometer subsystem pressure conversion time.
   */
  bmp085_baro_ct_t          bct;

  /**
   * @brief BMP085 barometer subsystem mode.
   */
  bmp085_baro_mode_t        mode;

  /**
   * @brief BMP085 barometer subsystem oversampling setting.
   */
  bmp085_baro_oss_t         oss;

  /**
   * @brief BMP085 thermometer subsystem temperature conversion time.
   */
  bmp085_thermo_ct_t        tct;
#endif /* BMP085_USE_ADVANCED */
} BMP085Config;

/**
 * @brief Structure representing a BMP085 driver.
 */
typedef struct BMP085Driver BMP085Driver;

/**
 * @brief @p BMP085 barometer subsystem specific methods.
 */
#define _bmp085_baro_methods                                                 \
  _base_barometer_methods

/**
 * @brief   @p  BMP085 thermometer subsystem specific methods.
 */
#define _bmp085_thermo_methods                                               \
  _base_thermometer_methods

/**
 * @extends BaseBarometerVMT
 *
 * @brief @p  BMP085 barometer virtual methods table.
 */
struct BMP085BAROVMT {
  _bmp085_baro_methods
};

/**
 * @extends BaseThermometerVMT
 *
 * @brief @p  BMP085 thermometer virtual methods table.
 */
struct BMP085THERMOVMT {
  _bmp085_thermo_methods
};

/**
 * @brief @p  BMP085Driver specific data.
 */
#define _bmp085_data                                                         \
  _base_barometer_data                                                       \
  _base_thermometer_data                                                     \
  /* Driver state.                    */                                     \
  bmp085_state_t      state;                                                 \
  /* Current configuration data.      */                                     \
  const BMP085Config  *config;                                               \
  /* Current barometer sensitivity.   */                                     \
  float               barosensitivity[BMP085_BARO_NUMBER_OF_AXES];           \
  /* Barometer bias data.             */                                     \
  int32_t             barobias[BMP085_BARO_NUMBER_OF_AXES];                  \
  /* Current thermometer sensitivity. */                                     \
  float               thermosensitivity[BMP085_THERMO_NUMBER_OF_AXES];       \
  /* Thermometer bias data.           */                                     \
  int32_t             thermobias[BMP085_THERMO_NUMBER_OF_AXES];              \
  /* BMP085 calibration data. */                                             \
  bmp085_cd_t         calibrationdata;

/**
 * @brief   BMP085 driver structure.
 */
struct BMP085Driver {
  /** @brief BaseSensor Virtual Methods Table.                              */
  const struct BaseSensorVMT      *vmt_basesensor;
  /** @brief BaseBarometer Virtual Methods Table.                           */
  const struct BaseBarometerVMT   *vmt_basebarometer;
  /** @brief BaseThermometer Virtual Methods Table.                         */
  const struct BaseThermometerVMT *vmt_basethermometer;
  _bmp085_data;
};

/** @} */

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void bmp085ObjectInit(BMP085Driver *devp);
  void bmp085Start(BMP085Driver *devp, const BMP085Config *config);
  void bmp085Stop(BMP085Driver *devp);
#ifdef __cplusplus
}
#endif

#endif /* BMP085_H */

/** @} */
