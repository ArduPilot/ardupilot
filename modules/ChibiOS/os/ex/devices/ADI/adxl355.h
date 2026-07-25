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
 * @file    adxl355.h
 * @brief   ADXL355 MEMS interface module header.
 *
 * @addtogroup ADXL355
 * @ingroup EX_ADI
 * @{
 */

#ifndef _ADXL355_H_
#define _ADXL355_H_

#include "ex_accelerometer.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   ADXL355 driver version string.
 */
#define EX_ADXL355_VERSION                  "1.1.0"

/**
 * @brief   ADXL355 driver version major number.
 */
#define EX_ADXL355_MAJOR                    1

/**
 * @brief   ADXL355 driver version minor number.
 */
#define EX_ADXL355_MINOR                    1

/**
 * @brief   ADXL355 driver version patch number.
 */
#define EX_ADXL355_PATCH                    0
/** @} */

/**
 * @brief   ADXL355 accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define ADXL355_ACC_NUMBER_OF_AXES          3U

#define ADXL355_ACC_2G                      2.048f
#define ADXL355_ACC_4G                      4.096f
#define ADXL355_ACC_8G                      8.0192f

#define ADXL355_ACC_SENS_2G                 0.003906f
#define ADXL355_ACC_SENS_4G                 0.007813f
#define ADXL355_ACC_SENS_8G                 0.015625f

#define ADXL355_ACC_BIAS                    0.0f
/** @} */

/**
 * @name    ADXL355 communication interfaces related bit masks
 * @{
 */
#define ADXL355_DI_MASK                     0xFF
#define ADXL355_DI(n)                       (1 << n)
#define ADXL355_AD_MASK                     0xFE
#define ADXL355_RW                          (1 << 0)
#define ADXL355_AD(n)                       (1 << (n + 1))
/** @} */

/**
 * @name    ADXL355 register addresses
 * @{
 */
#define ADXL355_AD_DEVID_AD                 0x00
#define ADXL355_AD_DEVID_MST                0x01
#define ADXL355_AD_PARTID                   0x02
#define ADXL355_AD_REVID                    0x03
#define ADXL355_AD_STATUS                   0x04
#define ADXL355_AD_FIFO_ENTRIES             0x05
#define ADXL355_AD_TEMP2                    0x06
#define ADXL355_AD_TEMP1                    0x07
#define ADXL355_AD_XDATA3                   0x08
#define ADXL355_AD_XDATA2                   0x09
#define ADXL355_AD_XDATA1                   0x0A
#define ADXL355_AD_YDATA3                   0x0B
#define ADXL355_AD_YDATA2                   0x0C
#define ADXL355_AD_YDATA1                   0x0D
#define ADXL355_AD_ZDATA3                   0x0E
#define ADXL355_AD_ZDATA2                   0x0F
#define ADXL355_AD_ZDATA1                   0x10
#define ADXL355_AD_FIFO_DATA                0x11
#define ADXL355_AD_OFFSET_X_H               0x1E
#define ADXL355_AD_OFFSET_X_L               0x1F
#define ADXL355_AD_OFFSET_Y_H               0x20
#define ADXL355_AD_OFFSET_Y_L               0x21
#define ADXL355_AD_OFFSET_Z_H               0x22
#define ADXL355_AD_OFFSET_Z_L               0x23
#define ADXL355_AD_ACT_EN                   0x24
#define ADXL355_AD_ACT_THRES_L              0x25
#define ADXL355_AD_ACT_THRES_H              0x26
#define ADXL355_AD_ACT_COUNTER              0x27
#define ADXL355_AD_FILTER                   0x28
#define ADXL355_AD_FIFO_SAMPLES             0x29
#define ADXL355_AD_INT_MAP                  0x2A
#define ADXL355_AD_SYNC                     0x2B
#define ADXL355_AD_RANGE                    0x2C
#define ADXL355_AD_POWER_CTL                0x2D
#define ADXL355_AD_SELF_TEST                0x2E
#define ADXL355_AD_RESET                    0x2F
/** @} */

/**
 * @name    ADXL355 Device Identifier
 * @{
 */
#define ADXL355_DEVID_MST                   0x1D
/** @} */

/**
 * @name    ADXL355_FILTER register bits definitions
 * @{
 */
#define ADXL355_FILTER_MASK                 0x7F
#define ADXL355_FILTER_ORD_LPF_0            (1 << 0)
#define ADXL355_FILTER_ORD_LPF_1            (1 << 1)
#define ADXL355_FILTER_ORD_LPF_2            (1 << 2)
#define ADXL355_FILTER_ORD_LPF_3            (1 << 3)
#define ADXL355_FILTER_HPF_CORNER_0         (1 << 4)
#define ADXL355_FILTER_HPF_CORNER_1         (1 << 5)
#define ADXL355_FILTER_HPF_CORNER_2         (1 << 6)
/** @} */

/**
 * @name    ADXL355_FIFO_SAMPLES register bits definitions
 * @{
 */
#define ADXL355_FIFO_SAMPLES_MASK           0x7F
#define ADXL355_FIFO_SAMPLES_BIT_0          (1 << 0)
#define ADXL355_FIFO_SAMPLES_BIT_1          (1 << 1)
#define ADXL355_FIFO_SAMPLES_BIT_2          (1 << 2)
#define ADXL355_FIFO_SAMPLES_BIT_3          (1 << 3)
#define ADXL355_FIFO_SAMPLES_BIT_4          (1 << 4)
#define ADXL355_FIFO_SAMPLES_BIT_5          (1 << 5)
#define ADXL355_FIFO_SAMPLES_BIT_6          (1 << 6)
/** @} */

/**
 * @name    ADXL355_INT_MAP register bits definitions
 * @{
 */
#define ADXL355_INT_MAP_MASK                0xFF
#define ADXL355_INT_MAP_RDY_EN1             (1 << 0)
#define ADXL355_INT_MAP_FULL_EN1            (1 << 1)
#define ADXL355_INT_MAP_OVR_EN1             (1 << 2)
#define ADXL355_INT_MAP_ACT_EN1             (1 << 3)
#define ADXL355_INT_MAP_RDY_EN2             (1 << 4)
#define ADXL355_INT_MAP_FULL_EN2            (1 << 5)
#define ADXL355_INT_MAP_OVR_EN2             (1 << 6)
#define ADXL355_INT_MAP_ACT_EN2             (1 << 7)
/** @} */

/**
 * @name    ADXL355_SYNC register bits definitions
 * @{
 */
#define ADXL355_SYNC_MASK                   0x07
#define ADXL355_SYNC_EXT_SYNC_0             (1 << 0)
#define ADXL355_SYNC_EXT_SYNC_1             (1 << 1)
#define ADXL355_SYNC_EXT_CLK                (1 << 2)
/** @} */

/**
 * @name    ADXL355_RANGE register bits definitions
 * @{
 */
#define ADXL355_RANGE_MASK                  0xC3
#define ADXL355_RANGE_RANGE_MASK            0x03
#define ADXL355_RANGE_RANGE_1               (1 << 1)
#define ADXL355_RANGE_RANGE_0               (1 << 0)
#define ADXL355_RANGE_RANGE_1               (1 << 1)
#define ADXL355_RANGE_INT_POL               (1 << 6)
#define ADXL355_RANGE_I2C_HS                (1 << 7)
/** @} */
 
/**
 * @name    ADXL355_POWER_CTL register bits definitions
 * @{
 */
#define ADXL355_POWER_CTL_MASK              0x07
#define ADXL355_POWER_CTL_STANDBY           (1 << 0)
#define ADXL355_POWER_CTL_TEMP_OFF          (1 << 1)
#define ADXL355_POWER_CTL_DRDY_OFF          (1 << 2)
/** @} */

/**
 * @name    ADXL355_SELT_TEST register bits definitions
 * @{
 */
#define ADXL355_SELF_TEST_MASK              0x03
#define ADXL355_SELF_TEST_ST1               (1 << 0)
#define ADXL355_SELF_TEST_ST2               (1 << 1)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   ADXL355 SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p TRUE.
 */
#if !defined(ADXL355_USE_SPI) || defined(__DOXYGEN__)
#define ADXL355_USE_SPI                     TRUE
#endif

/**
 * @brief   ADXL355 shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(ADXL355_SHARED_SPI) || defined(__DOXYGEN__)
#define ADXL355_SHARED_SPI                  FALSE
#endif

/**
 * @brief   ADXL355 I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ADXL355_USE_I2C) || defined(__DOXYGEN__)
#define ADXL355_USE_I2C                     FALSE
#endif

/**
 * @brief   ADXL355 shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(ADXL355_SHARED_I2C) || defined(__DOXYGEN__)
#define ADXL355_SHARED_I2C                  FALSE
#endif

/**
 * @brief   ADXL355 advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(ADXL355_USE_ADVANCED) || defined(__DOXYGEN__)
#define ADXL355_USE_ADVANCED                FALSE
#endif

/**
 * @brief   ADXL355 internal communication buffer sizes.
 */
#if !defined(ADXL355_COMM_BUFF_SIZE) || defined(__DOXYGEN__)
#define ADXL355_COMM_BUFF_SIZE              32
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(ADXL355_USE_SPI ^ ADXL355_USE_I2C)
#error "ADXL355_USE_SPI and ADXL355_USE_I2C cannot be both true or both false"
#endif

#if ADXL355_USE_SPI && !HAL_USE_SPI
#error "ADXL355_USE_SPI requires HAL_USE_SPI"
#endif

#if ADXL355_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "ADXL355_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if ADXL355_USE_I2C && !HAL_USE_I2C
#error "ADXL355_USE_I2C requires HAL_USE_I2C"
#endif

#if ADXL355_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "ADXL355_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for ADXL355 over I2C.
 */
#if ADXL355_USE_I2C
#error "ADXL355 over I2C still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    ADXL355 data structures and types
 * @{
 */
/**
 * @brief   Structure representing a ADXL355 driver.
 */
typedef struct ADXL355Driver ADXL355Driver;

/**
 * @brief   ADXL355 full scale.
 */
typedef enum {
  ADXL355_ACC_FS_2G = 0x01,         /**< Full scale ±2g.                    */
  ADXL355_ACC_FS_4G = 0x02,         /**< Full scale ±4g.                    */
  ADXL355_ACC_FS_8G = 0x03,         /**< Full scale ±8g.                    */
}adxl355_acc_fs_t;

/**
 * @brief   ADXL355 output data rate.
 */
typedef enum {
  ADXL355_ACC_ODR_4000HZ = 0x00,    /**< ODR 4000 Hz, LP cut-off ODR/4.     */
  ADXL355_ACC_ODR_2000HZ = 0x01,    /**< ODR 2000 Hz, LP cut-off ODR/4.     */
  ADXL355_ACC_ODR_1000HZ = 0x02,    /**< ODR 1000 Hz, LP cut-off ODR/4.     */
  ADXL355_ACC_ODR_500HZ = 0x03,     /**< ODR 500 Hz, LP cut-off ODR/4.      */
  ADXL355_ACC_ODR_250HZ = 0x04,     /**< ODR 250 Hz, LP cut-off ODR/4.      */
  ADXL355_ACC_ODR_125HZ = 0x06,     /**< ODR 125 Hz, LP cut-off ODR/4.      */
  ADXL355_ACC_ODR_62P5HZ = 0x06,    /**< ODR 62.5 Hz, LP cut-off ODR/4.     */
  ADXL355_ACC_ODR_31P25HZ = 0x07,   /**< ODR 31.25 Hz, LP cut-off ODR/4.    */
  ADXL355_ACC_ODR_15P625HZ = 0x08,  /**< ODR 15.625 Hz, LP cut-off ODR/4.   */
  ADXL355_ACC_ODR_7P913HZ = 0x09,   /**< ODR 7.913 Hz, LP cut-off ODR/4.    */
  ADXL355_ACC_ODR_3P906HZ = 0x0A    /**< ODR 3.906 Hz, LP cut-off ODR/4.    */
}adxl355_acc_odr_t;

/**
 * @brief   ADXL355 HP filter.
 */
typedef enum {
  ADXL355_ACC_HP_DISABLED = 0x00,   /**< HP disabled.                       */  
  ADXL355_ACC_HP_LEV_1 = 0x10,      /**< HP cut-off 247 x 10^-3 x ODR.      */
  ADXL355_ACC_HP_LEV_2 = 0x20,      /**< HP cut-off 62.084 x 10^-3 x ODR.   */
  ADXL355_ACC_HP_LEV_3 = 0x30,      /**< HP cut-off 15.545 x 10^-3 x ODR.   */
  ADXL355_ACC_HP_LEV_4 = 0x40,      /**< HP cut-off 3.892 x 10^-3 x ODR.    */
  ADXL355_ACC_HP_LEV_5 = 0x50,      /**< HP cut-off 0.954 x 10^-3 x ODR.    */
  ADXL355_ACC_HP_LEV_6 = 0x60,      /**< HP cut-off 0.238 x 10^-3 x ODR.    */
}adxl355_acc_hp_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  ADXL355_UNINIT = 0,               /**< Not initialized.                   */
  ADXL355_STOP = 1,                 /**< Stopped.                           */
  ADXL355_READY = 2,                /**< Ready.                             */
} adxl355_state_t;

/**
 * @brief   ADXL355 configuration structure.
 */
typedef struct {

#if (ADXL355_USE_SPI) || defined(__DOXYGEN__)
  /**
   * @brief SPI driver associated to this ADXL355.
   */
  SPIDriver                 *spip;
  /**
   * @brief SPI configuration associated to this ADXL355.
   */
  const SPIConfig           *spicfg;
#endif /* ADXL355_USE_SPI */
#if (ADXL355_USE_I2C) || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this ADXL355.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this ADXL355.
   */
  const I2CConfig           *i2ccfg;
#endif /* ADXL355_USE_I2C */
  /**
   * @brief ADXL355 accelerometer subsystem initial sensitivity.
   */
  float                     *accsensitivity;
  /**
   * @brief ADXL355 accelerometer subsystem initial bias.
   */
  float                     *accbias;
  /**
   * @brief ADXL355 accelerometer subsystem initial full scale.
   */
  adxl355_acc_fs_t          accfullscale;
  /**
   * @brief ADXL355 output data rate selection.
   */
  adxl355_acc_odr_t         accodr;
#if ADXL355_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief   ADXL355 HP filter bandwidth.
   */
  adxl355_acc_hp_t          acchighpass;
#endif
} ADXL355Config;

/**
 * @brief   @p ADXL355 specific methods.
 */
#define _adxl355_methods_alone                                              \
  /* Change full scale value of ADXL355 accelerometer subsystem.*/          \
  msg_t (*acc_set_full_scale)(ADXL355Driver *devp, adxl355_acc_fs_t fs);


/**
 * @brief   @p ADXL355 specific methods with inherited ones.
 */
#define _adxl355_methods                                                    \
  _base_object_methods                                                      \
  _adxl355_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief   @p ADXL355 virtual methods table.
 */
struct ADXL355VMT {
  _adxl355_methods
};

/**
 * @brief   @p ADXL355Driver specific data.
 */
#define _adxl355_data                                                       \
  _base_sensor_data                                                         \
  /* Driver state.*/                                                        \
  adxl355_state_t           state;                                          \
  /* Current configuration data.*/                                          \
  const ADXL355Config       *config;                                        \
  /* Accelerometer subsystem axes number.*/                                 \
  size_t                    accaxes;                                        \
  /* Accelerometer subsystem current sensitivity.*/                         \
  float                     accsensitivity[ADXL355_ACC_NUMBER_OF_AXES];     \
  /* Accelerometer subsystem current bias .*/                               \
  float                     accbias[ADXL355_ACC_NUMBER_OF_AXES];            \
  /* Accelerometer subsystem current full scale value.*/                    \
  float                     accfullscale;                                   \
  /* Accelerometer communication tx buffer pointer.*/                       \
  uint8_t*                  commtxp;                                        \
  /* Accelerometer communication rx buffer pointer.*/                       \
  uint8_t*                  commrxp;

/**
 * @brief   ADXL355 3-axis accelerometer class.
 */
struct ADXL355Driver {
  /** @brief Virtual Methods Table.*/
  const struct ADXL355VMT     *vmt;
  /** @brief Base accelerometer interface.*/
  BaseAccelerometer           acc_if;
  _adxl355_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] devp      pointer to @p ADXL355Driver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define adxl355AccelerometerGetAxesNumber(devp)                             \
        accelerometerGetAxesNumber(&((devp)->acc_if))

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p ADXL355Driver.
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
#define adxl355AccelerometerReadRaw(devp, axes)                             \
        accelerometerReadRaw(&((devp)->acc_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p ADXL355Driver.
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
#define adxl355AccelerometerReadCooked(devp, axes)                          \
        accelerometerReadCooked(&((devp)->acc_if), axes)

/**
 * @brief   Set bias values for the BaseAccelerometer.
 * @note    Bias must be expressed as milli-G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p ADXL355Driver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define adxl355AccelerometerSetBias(devp, bp)                               \
        accelerometerSetBias(&((devp)->acc_if), bp)

/**
 * @brief   Reset bias values for the BaseAccelerometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p ADXL355Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define adxl355AccelerometerResetBias(devp)                                 \
        accelerometerResetBias(&((devp)->acc_if))

/**
 * @brief   Set sensitivity values for the BaseAccelerometer.
 * @note    Sensitivity must be expressed as milli-G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p ADXL355Driver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define adxl355AccelerometerSetSensitivity(devp, sp)                        \
        accelerometerSetSensitivity(&((devp)->acc_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseAccelerometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p ADXL355Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define adxl355AccelerometerResetSensitivity(devp)                          \
        accelerometerResetSensitivity(&((devp)->acc_if))

/**
 * @brief   Changes the ADXL355Driver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p ADXL355Driver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define adxl355AccelerometerSetFullScale(devp, fs)                          \
        (devp)->vmt->acc_set_full_scale(devp, fs)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#if (ADXL355_USE_SPI) || defined(__DOXYGEN__)
  void adxl355SPIReadRegister(ADXL355Driver *devp, uint8_t reg, size_t n,
                              uint8_t* b);
  void adxl355SPIWriteRegister(ADXL355Driver *devp, uint8_t reg, size_t n,
                               uint8_t* b);
#endif /* ADXL355_USE_SPI */
  void adxl355ObjectInit(ADXL355Driver *devp, uint8_t* txbp, uint8_t* rxbp);
  void adxl355Start(ADXL355Driver *devp, const ADXL355Config *config);
  void adxl355Stop(ADXL355Driver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _ADXL355_H_ */

/** @} */

