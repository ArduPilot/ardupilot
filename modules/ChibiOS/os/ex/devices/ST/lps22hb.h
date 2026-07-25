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
 * @file    lps22hb.h
 * @brief   LPS22HB MEMS interface module header.
 *
 * @addtogroup LPS22HB
 * @ingroup EX_ST
 * @{
 */
#ifndef _LPS22HB_H_
#define _LPS22HB_H_

#include "ex_barometer.h"
#include "ex_thermometer.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   LPS22HB driver version string.
 */
#define EX_LPS22HB_VERSION                  "1.0.2"

/**
 * @brief   LPS22HB driver version major number.
 */
#define EX_LPS22HB_MAJOR                    1

/**
 * @brief   LPS22HB driver version minor number.
 */
#define EX_LPS22HB_MINOR                    0

/**
 * @brief   LPS22HB driver version patch number.
 */
#define EX_LPS22HB_PATCH                    2
/** @} */

/**
 * @brief   LPS22HB barometer subsystem characteristics.
 * @note    Sensitivity is expressed as hPa/LSB whereas hPa stand for
 *          hectopascal.
 * @note    Bias is expressed as hPa.
 *
 * @{
 */
#define LPS22HB_BARO_NUMBER_OF_AXES         1U

#define LPS22HB_BARO_SENS                   0.00024414f
#define LPS22HB_BARO_BIAS                   0.0f
/** @} */

/**
 * @brief   LPS22HB thermometer subsystem characteristics.
 * @note    Sensitivity is expressed as �C/LSB.
 * @note    Bias is expressed as �C.
 *
 * @{
 */
#define LPS22HB_THERMO_NUMBER_OF_AXES       1U

#define LPS22HB_THERMO_SENS                 0.01f
#define LPS22HB_THERMO_BIAS                 0.0f
/** @} */

/**
 * @name    LPS22HB communication interfaces related bit masks
 * @{
 */
#define LPS22HB_DI_MASK                     0xFF
#define LPS22HB_DI(n)                       (1 << n)
#define LPS22HB_AD_MASK                     0x3F
#define LPS22HB_AD(n)                       (1 << n)
#define LPS22HB_MS                          (1 << 6)
#define LPS22HB_RW                          (1 << 7)

#define LPS22HB_SUB_MS                      (1 << 7)
/** @} */

/**
 * @name    LPS22HB register addresses
 * @{
 */
#define LPS22HB_AD_INT_CFG                  0x0B
#define LPS22HB_AD_THS_P_L                  0x0C
#define LPS22HB_AD_THS_P_H                  0x0D
#define LPS22HB_AD_WHO_AM_I                 0x0F
#define LPS22HB_AD_CTRL_REG1                0x10
#define LPS22HB_AD_CTRL_REG2                0x11
#define LPS22HB_AD_CTRL_REG3                0x12
#define LPS22HB_AD_FIFO_CTRL                0x14
#define LPS22HB_AD_REF_P_XL                 0x15
#define LPS22HB_AD_REF_P_L                  0x16
#define LPS22HB_AD_REF_P_H                  0x17
#define LPS22HB_AD_RPDS_L                   0x18
#define LPS22HB_AD_RPDS_H                   0x19
#define LPS22HB_AD_RES_CONF                 0x1A
#define LPS22HB_AD_INT_SRC                  0x25
#define LPS22HB_AD_FIFO_SRC                 0x26
#define LPS22HB_AD_STATUS_REG               0x27
#define LPS22HB_AD_PRESS_OUT_XL             0x28
#define LPS22HB_AD_PRESS_OUT_L              0x29
#define LPS22HB_AD_PRESS_OUT_H              0x2A
#define LPS22HB_AD_TEMP_OUT_L               0x2B
#define LPS22HB_AD_TEMP_OUT_H               0x2C
#define LPS22HB_AD_LPFP_RES                 0x33
/** @} */

/**
 * @name    LPS22HB_INT_CFG register bits definitions
 * @{
 */
#define LPS22HB_INT_CFG_MASK                0xFF
#define LPS22HB_INT_CFG_PHE                 (1 << 0)
#define LPS22HB_INT_CFG_PLE                 (1 << 1)
#define LPS22HB_INT_CFG_LIR                 (1 << 2)
#define LPS22HB_INT_CFG_DIFF_EN             (1 << 3)
#define LPS22HB_INT_CFG_RESET_AZ            (1 << 4)
#define LPS22HB_INT_CFG_AUTOZERO            (1 << 5)
#define LPS22HB_INT_CFG_RESET_ARP           (1 << 6)
#define LPS22HB_INT_CFG_AUTORIFP            (1 << 7)
/** @} */

/**
 * @name    LPS22HB_CTRL_REG1 register bits definitions
 * @{
 */
#define LPS22HB_CTRL_REG1_MASK              0x7F
#define LPS22HB_CTRL_REG1_SIM               (1 << 0)
#define LPS22HB_CTRL_REG1_BDU               (1 << 1)
#define LPS22HB_CTRL_REG1_LPFP_CFG          (1 << 2)
#define LPS22HB_CTRL_REG1_LPFP_EN           (1 << 3)
#define LPS22HB_CTRL_REG1_ODR0              (1 << 4)
#define LPS22HB_CTRL_REG1_ODR1              (1 << 5)
#define LPS22HB_CTRL_REG1_ODR2              (1 << 6)
/** @} */

/**
 * @name    LPS22HB_CTRL_REG2 register bits definitions
 * @{
 */
#define LPS22HB_CTRL_REG2_MASK              0xFD
#define LPS22HB_CTRL_REG2_ONE_SHOT          (1 << 0)
#define LPS22HB_CTRL_REG2_SWRESET           (1 << 2)
#define LPS22HB_CTRL_REG2_I2C_DIS           (1 << 3)
#define LPS22HB_CTRL_REG2_IF_ADD_INC        (1 << 4)
#define LPS22HB_CTRL_REG2_STOP_ON_FTH       (1 << 5)
#define LPS22HB_CTRL_REG2_FIFO_EN           (1 << 6)
#define LPS22HB_CTRL_REG2_BOOT              (1 << 7)
/** @} */

/**
 * @name    LPS22HB_CTRL_REG3 register bits definitions
 * @{
 */
#define LPS22HB_CTRL_REG3_MASK              0xFF
#define LPS22HB_CTRL_REG3_INT_S1            (1 << 0)
#define LPS22HB_CTRL_REG3_INT_S2            (1 << 1)
#define LPS22HB_CTRL_REG3_DRDY              (1 << 2)
#define LPS22HB_CTRL_REG3_F_OVR             (1 << 3)
#define LPS22HB_CTRL_REG3_F_FTH             (1 << 4)
#define LPS22HB_CTRL_REG3_F_FSS5            (1 << 5)
#define LPS22HB_CTRL_REG3_PP_OD             (1 << 6)
#define LPS22HB_CTRL_REG3_INT_H_L           (1 << 7)
/** @} */

/**
 * @name    LPS22HB_INT_SRC register bits definitions
 * @{
 */
#define LPS22HB_INT_SRC_MASK                0x87
#define LPS22HB_INT_SRC_PH                  (1 << 0)
#define LPS22HB_INT_SRC_PL                  (1 << 1)
#define LPS22HB_INT_SRC_IA                  (1 << 2)
#define LPS22HB_INT_SRC_BOOT_STATUS         (1 << 8)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LPS22HB SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LPS22HB_USE_SPI) || defined(__DOXYGEN__)
#define LPS22HB_USE_SPI                     FALSE
#endif

/**
 * @brief   LPS22HB shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LPS22HB_SHARED_SPI) || defined(__DOXYGEN__)
#define LPS22HB_SHARED_SPI                  FALSE
#endif

/**
 * @brief   LPS22HB I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LPS22HB_USE_I2C) || defined(__DOXYGEN__)
#define LPS22HB_USE_I2C                     TRUE
#endif

/**
 * @brief   LPS22HB shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LPS22HB_SHARED_I2C) || defined(__DOXYGEN__)
#define LPS22HB_SHARED_I2C                  FALSE
#endif

/**
 * @brief   LPS22HB advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LPS22HB_USE_ADVANCED) || defined(__DOXYGEN__)
#define LPS22HB_USE_ADVANCED                FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(LPS22HB_USE_SPI ^ LPS22HB_USE_I2C)
#error "LPS22HB_USE_SPI and LPS22HB_USE_I2C cannot be both true or both false"
#endif

#if LPS22HB_USE_SPI && !HAL_USE_SPI
#error "LPS22HB_USE_SPI requires HAL_USE_SPI"
#endif

#if LPS22HB_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "LPS22HB_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if LPS22HB_USE_I2C && !HAL_USE_I2C
#error "LPS22HB_USE_I2C requires HAL_USE_I2C"
#endif

#if LPS22HB_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "LPS22HB_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for LPS22HB over SPI.
 */
#if LPS22HB_USE_SPI
#error "LPS22HB over SPI still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    LPS22HB data structures and types.
 * @{
 */
/**
 * @brief   Structure representing a LPS22HB driver.
 */
typedef struct LPS22HBDriver LPS22HBDriver;

/**
 * @brief  LPS22HB slave address
 */
typedef enum {
  LPS22HB_SAD_GND = 0x5C,           /**< Slave Address when SA0 is to GND   */
  LPS22HB_SAD_VCC = 0x5D            /**< Slave Address when SA0 is to VCC   */
}lps22hb_sad_t;
 
/**
 * @brief   LPS22HB output data rate and bandwidth.
 */
typedef enum {
  LPS22HB_ODR_PD = 0x00,            /**< Power down.                        */
  LPS22HB_ODR_1HZ = 0x10,           /**< Output data rate 1 Hz.             */
  LPS22HB_ODR_10HZ = 0x20,          /**< Output data rate 10 Hz.            */
  LPS22HB_ODR_25HZ = 0x30,          /**< Output data rate 25 Hz.            */
  LPS22HB_ODR_50HZ = 0x40,          /**< Output data rate 50 Hz.            */
  LPS22HB_ODR_75HZ = 0x50           /**< Output data rate 75 Hz.            */
}lps22hb_odr_t;

/**
 * @brief   LPS22HB pressure resolution.
 */
typedef enum {
  LPS22HB_LP_DISABLED = 0x00,       /**< LP Filter disabled.                */
  LPS22HB_LP_ODR_9 = 0x08,          /**< LP Filter enabled. Cut-off ORD/9.  */
  LPS22HB_LP_ODR_20 = 0x0C          /**< LP Filter enabled. Cut-off ORD/20. */
}lps22hb_lp_t;

/**
 * @brief   LPS22HB block data update.
 */
typedef enum {
  LPS22HB_BDU_CONTINUOUS = 0x00,    /**< Block data continuously updated.   */
  LPS22HB_BDU_BLOCKED = 0x40        /**< Block data updated after reading.  */
}lps22hb_bdu_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LPS22HB_UNINIT = 0,               /**< Not initialized.                   */
  LPS22HB_STOP = 1,                 /**< Stopped.                           */
  LPS22HB_READY = 2,                /**< Ready.                             */
} lps22hb_state_t;

/**
 * @brief   LPS22HB configuration structure.
 */
typedef struct {

#if LPS22HB_USE_SPI || defined(__DOXYGEN__)
  /**
   * @brief SPI driver associated to this LPS22HB.
   */
  SPIDriver                 *spip;
  /**
   * @brief SPI configuration associated to this LPS22HB.
   */
  const SPIConfig           *spicfg;
#endif /* LPS22HB_USE_SPI */
#if LPS22HB_USE_I2C || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this LPS22HB.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this LPS22HB.
   */
  const I2CConfig           *i2ccfg;
  /**
   * @brief LPS22HB slave address
   */
  lps22hb_sad_t             slaveaddress;
#endif /* LPS22HB_USE_I2C */
  /**
   * @brief LPS22HB barometer subsystem initial sensitivity.
   */
  float                     *barosensitivity;
  /**
   * @brief LPS22HB barometer subsystem initial bias.
   */
  float                     *barobias;
  /**
   * @brief LPS22HB thermometer subsystem initial sensitivity.
   */
  float                     *thermosensitivity;
  /**
   * @brief LPS22HB thermometer subsystem initial bias.
   */
  float                     *thermobias;
  /**
   * @brief LPS22HB output data rate selection.
   */
  lps22hb_odr_t              outputdatarate;
#if LPS22HB_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief LPS22HB block data update.
   */
  lps22hb_bdu_t              blockdataupdate;
  /**
   * @brief LPS22HB barometer subsystem resolution.
   */
  lps22hb_lp_t               lowpass_filter;
#endif
} LPS22HBConfig;

/**
 * @brief   @p LPS22HB specific methods.
 * @note    No methods so far, just a common ancestor interface.
 */
#define _lps22hb_methods_alone

/**
 * @brief @p LPS22HB specific methods with inherited ones.
 */
#define _lps22hb_methods                                                    \
  _base_object_methods                                                      \
  _lps22hb_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief @p LPS22HB virtual methods table.
 */
struct LPS22HBVMT {
  _lps22hb_methods
};
  
/**
 * @brief   @p LPS22HBDriver specific data.
 */
#define _lps22hb_data                                                       \
  /* Driver state.*/                                                        \
  lps22hb_state_t           state;                                          \
  /* Current configuration data.*/                                          \
  const LPS22HBConfig       *config;                                        \
  /* Barometer subsystem axes number.*/                                     \
  size_t                    baroaxes;                                       \
  /* Barometer subsystem current sensitivity.*/                             \
  float                     barosensitivity;                                \
  /* Barometer subsystem current bias .*/                                   \
  float                     barobias;                                       \
  /* Thermometer subsystem axes number.*/                                   \
  size_t                    thermoaxes;                                     \
  /* Thermometer subsystem current sensitivity.*/                           \
  float                     thermosensitivity;                              \
  /* Thermometer subsystem current bias.*/                                  \
  float                     thermobias;

/**
 * @brief   LPS22HB 2-axis barometer/thermometer class.
 */
struct LPS22HBDriver {
  /** @brief Virtual Methods Table.*/
  const struct LPS22HBVMT   *vmt;
  /** @brief Base barometer interface.*/
  BaseBarometer             baro_if;
  /** @brief Base thermometer interface.*/
  BaseThermometer           thermo_if;
  _lps22hb_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseBarometer.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lps22hbBarometerGetAxesNumber(devp)                                 \
        barometerGetAxesNumber(&((devp)->baro_if))

/**
 * @brief   Retrieves raw data from the BaseBarometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseBarometer axes number.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
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
#define lps22hbBarometerReadRaw(devp, axes)                                 \
        barometerReadRaw(&((devp)->baro_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseBarometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as hPa.
 * @note    The axes array must be at least the same size of the
 *          BaseBarometer axes number.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
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
#define lps22hbBarometerReadCooked(devp, axes)                              \
        barometerReadCooked(&((devp)->baro_if), axes)

/**
 * @brief   Set bias values for the BaseBarometer.
 * @note    Bias must be expressed as hPa.
 * @note    The bias buffer must be at least the same size of the
 *          BaseBarometer axes number.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lps22hbBarometerSetBias(devp, bp)                                   \
        barometerSetBias(&((devp)->baro_if), bp)

/**
 * @brief   Reset bias values for the BaseBarometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lps22hbBarometerResetBias(devp)                                     \
        barometerResetBias(&((devp)->baro_if))

/**
 * @brief   Set sensitivity values for the BaseBarometer.
 * @note    Sensitivity must be expressed as hPa/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseBarometer axes number.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lps22hbBarometerSetSensitivity(devp, sp)                            \
        barometerSetSensitivity(&((devp)->baro_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseBarometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lps22hbBarometerResetSensitivity(devp)                              \
        barometerResetSensitivity(&((devp)->baro_if))

/**
 * @brief   Return the number of axes of the BaseThermometer.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lps22hbThermometerGetAxesNumber(devp)                               \
        thermometerGetAxesNumber(&((devp)->thermo_if))

/**
 * @brief   Retrieves raw data from the BaseThermometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
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
#define lps22hbThermometerReadRaw(devp, axes)                               \
        thermometerReadRaw(&((devp)->thermo_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseThermometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as °C.
 * @note    The axes array must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
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
#define lps22hbThermometerReadCooked(devp, axes)                            \
        thermometerReadCooked(&((devp)->thermo_if), axes)

/**
 * @brief   Set bias values for the BaseThermometer.
 * @note    Bias must be expressed as °C.
 * @note    The bias buffer must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lps22hbThermometerSetBias(devp, bp)                                 \
        thermometerSetBias(&((devp)->thermo_if), bp)

/**
 * @brief   Reset bias values for the BaseThermometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lps22hbThermometerResetBias(devp)                                   \
        thermometerResetBias(&((devp)->thermo_if))

/**
 * @brief   Set sensitivity values for the BaseThermometer.
 * @note    Sensitivity must be expressed as °C/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseThermometer axes number.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lps22hbThermometerSetSensitivity(devp, sp)                          \
        thermometerSetSensitivity(&((devp)->thermo_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseThermometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LPS22HBDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lps22hbThermometerResetSensitivity(devp)                            \
        thermometerResetSensitivity(&((devp)->thermo_if))
        
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void lps22hbObjectInit(LPS22HBDriver *devp);
  void lps22hbStart(LPS22HBDriver *devp, const LPS22HBConfig *config);
  void lps22hbStop(LPS22HBDriver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _LPS22HB_H_ */

/** @} */

