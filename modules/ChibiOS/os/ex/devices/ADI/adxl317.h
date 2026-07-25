/*
    ChibiOS - Copyright (C) 2016..2023 Simona Di Domenico

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
 * @file    adxl317.h
 * @brief   ADXL317 MEMS interface module header.
 *
 * @addtogroup ADXL317
 * @ingroup EX_ADI
 * @{
 */

#ifndef _ADXL317_H_
#define _ADXL317_H_

#include "ex_accelerometer.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   ADXL317 driver version string.
 */
#define EX_ADXL317_VERSION                  "1.0.0"

/**
 * @brief   ADXL317 driver version major number.
 */
#define EX_ADXL317_MAJOR                    1

/**
 * @brief   ADXL317 driver version minor number.
 */
#define EX_ADXL317_MINOR                    0

/**
 * @brief   ADXL317 driver version patch number.
 */
#define EX_ADXL317_PATCH                    0
/** @} */

/**
 * @brief   ADXL317 accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define ADXL317_ACC_NUMBER_OF_AXES          3U


#define ADXL317_ACC_SENS                    2.0f
#define ADXL317_ACC_BIAS                    0.0f

/** @} */

/**
 * @name    ADXL317 register addresses
 * @{
 */
#define ADXL317_AD_DEVID_0                  0x00
#define ADXL317_AD_DEVID_1                  0x01
#define ADXL317_AD_USER_REG_KEY             0x80
#define ADXL317_AD_I2C_CFGO                 0x81
#define ADXL317_AD_I2C_CFG1                 0x82
#define ADXL317_AD_CLK_RATE                 0x83
#define ADXL317_AD_X_ST                     0x84
#define ADXL317_AD_X_FILT                   0x85
#define ADXL317_AD_Y_ST                     0x86
#define ADXL317_AD_Y_FILT                   0x87
#define ADXL317_AD_Z_ST                     0x88
#define ADXL317_AD_Z_FILT                   0x89
#define ADXL317_AD_X_DATA_LO                0x8A
#define ADXL317_AD_X_DATA_HI                0x8B
#define ADXL317_AD_Y_DATA_LO                0x8C
#define ADXL317_AD_Y_DATA_HI                0x8D
#define ADXL317_AD_Z_DATA_LO                0x8E
#define ADXL317_AD_Z_DATA_HI                0x8F
/** @} */

/**
 * @name    ADXL317 Device Identifier
 * @{
 */
#define ADXL317_DEVID_0                     0x22
/** @} */

/**
 * @name    ADXL317 Device User Reg Key for Register Write Enable
 * @{
 */
#define ADXL317_USER_REG_KEY_0              0xBC
#define ADXL317_USER_REG_KEY_1              0x43
/** @} */

/**
 * @name    ADXL317 Default filter
 * @details LP (CIC and IIR cascade) with cut off at 4kHz
 *          HP disabled
 * @{
 */
#define ADXL317_DEFAULT_FILTER              0x40
/** @} */

/**
 * @name    ADXL317_I2S_CFG0 register bits definitions
 * @{
 */
#define ADXL317_I2S_CFG0_MASK               0xF3
#define ADXL317_I2S_CFG0_TDMMODE_0          (1 << 0)
#define ADXL317_I2S_CFG0_TDMMODE_1          (1 << 1)
#define ADXL317_I2S_CFG0_TDMSS              (1 << 4)
#define ADXL317_I2S_CFG0_ALT                (1 << 5)
#define ADXL317_I2S_CFG0_EARLY              (1 << 6)
#define ADXL317_I2S_CFG0_INV                (1 << 7)
/** @} */

/**
 * @name    ADXL317_I2S_CFG1 register bits definitions
 * @{
 */
#define ADXL317_I2S_CFG1_MASK               0x07
#define ADXL317_I2S_CFG1_TX0EN              (1 << 0)
#define ADXL317_I2S_CFG1_TX1EN              (1 << 1)
#define ADXL317_I2S_CFG1_TXBCLKINV          (1 << 3)
/** @} */

/**
 * @name    ADXL317_CLOCK_RATE register bits definitions
 * @{
 */
#define ADXL317_CLOCK_RATE_MASK               0x07
#define ADXL317_CLOCK_RATE_A2B_CLK_RATE_0     (1 << 0)
#define ADXL317_CLOCK_RATE_A2B_CLK_RATE_1     (1 << 1)
#define ADXL317_CLOCK_RATE_A2B_CLK_RATE_2     (1 << 3)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   ADXL317 I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ADXL317_USE_I2C) || defined(__DOXYGEN__)
#define ADXL317_USE_I2C                     TRUE
#endif

/**
 * @brief   ADXL317 shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(ADXL317_SHARED_I2C) || defined(__DOXYGEN__)
#define ADXL317_SHARED_I2C                  FALSE
#endif

/**
 * @brief   ADXL317 advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(ADXL317_USE_ADVANCED) || defined(__DOXYGEN__)
#define ADXL317_USE_ADVANCED                FALSE
#endif

/**
 * @brief   ADXL317 internal maximum communication buffer sizes.
 */
#if !defined(ADXL317_COMM_BUFF_SIZE) || defined(__DOXYGEN__)
#define ADXL317_MAX_BUFF_SIZE               6
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if ADXL317_USE_I2C && !HAL_USE_I2C
#error "ADXL317_USE_I2C requires HAL_USE_I2C"
#endif

#if ADXL317_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "ADXL317_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    ADXL317 data structures and types
 * @{
 */
/**
 * @brief   Structure representing a ADXL317 driver.
 */
typedef struct ADXL317Driver ADXL317Driver;

/**
 * @brief   ADXL317 Slave Address.
 */
typedef enum {
  ADXL317_SAD_GND = 0x53,           /**< SAD pin connected to GND.          */
  ADXL317_SAD_VCC = 0x1D            /**< SAD pin connected to VCC.          */
} adxl317_sad_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  ADXL317_UNINIT = 0,               /**< Not initialized.                   */
  ADXL317_STOP = 1,                 /**< Stopped.                           */
  ADXL317_READY = 2,                /**< Ready.                             */
} adxl317_state_t;

/**
 * @brief   ADXL317 X LP filter.
 * @details The X LP filter is a cascade of the 2 LP filters: CIC, always
 *          enabled, and IIR, that can be disabled.
 */
typedef enum {
  ADXL317_ACC_X_LP_DISABLED = 0x00, /**< LP disabled.                       */
  ADXL317_ACC_X_LP_500 = 0x4F,      /**< LP cut-off 506 Hz.                 */
  ADXL317_ACC_X_LP_1000 = 0x4A,     /**< LP cut-off 1012 Hz.                */
  ADXL317_ACC_X_LP_2000 = 0x45,     /**< LP cut-off 2025 Hz.                */
  ADXL317_ACC_X_LP_4000 = 0x40,     /**< LP cut-off 4051 Hz.                */
} adxl317_acc_x_lp_t;

/**
 * @brief   ADXL317 Y LP filter.
 * @details The Y LP filter is a cascade of the 2 LP filters: CIC, always
 *          enabled, and IIR, that can be disabled.
 */
typedef enum {
  ADXL317_ACC_Y_LP_DISABLED = 0x00, /**< LP disabled.                       */
  ADXL317_ACC_Y_LP_500 = 0x4F,      /**< LP cut-off 506 Hz.                 */
  ADXL317_ACC_Y_LP_1000 = 0x4A,     /**< LP cut-off 1012 Hz.                */
  ADXL317_ACC_Y_LP_2000 = 0x45,     /**< LP cut-off 2025 Hz.                */
  ADXL317_ACC_Y_LP_4000 = 0x40,     /**< LP cut-off 4051 Hz.                */
} adxl317_acc_y_lp_t;

/**
 * @brief   ADXL317 Z LP filter.
 * @details The Z LP filter is a cascade of the 2 LP filters: CIC, always
 *          enabled, and IIR, that can be disabled.
 */
typedef enum {
  ADXL317_ACC_Z_LP_DISABLED = 0x00, /**< LP disabled.                       */
  ADXL317_ACC_Z_LP_500 = 0x4F,      /**< LP cut-off 506 Hz.                 */
  ADXL317_ACC_Z_LP_1000 = 0x4A,     /**< LP cut-off 1012 Hz.                */
  ADXL317_ACC_Z_LP_2000 = 0x45,     /**< LP cut-off 2025 Hz.                */
  ADXL317_ACC_Z_LP_4000 = 0x40,     /**< LP cut-off 4051 Hz.                */
} adxl317_acc_z_lp_t;

/**
 * @brief   ADXL317 LP filter configuration.
 */
typedef struct {
  adxl317_acc_x_lp_t x;
  adxl317_acc_y_lp_t y;
  adxl317_acc_z_lp_t z;
} adxl317_acc_lp_t;

/**
 * @brief   ADXL317 X HP filter.
 * @details The X HP filter is HPF filter.
 */
typedef enum {
  ADXL317_ACC_X_HP_DISABLED = 0x00, /**< HP disabled.                       */
  ADXL317_ACC_X_HP_29 = 0x80,       /**< HP enabled with cut-off 29.8 Hz.   */
  ADXL317_ACC_X_HP_7 = 0x90,        /**< HP enabled with cut-off 7.46 Hz.   */
  ADXL317_ACC_X_HP_1 = 0xA0,        /**< HP enabled with cut-off 1.85 Hz.   */
  ADXL317_ACC_X_HP_0_5 = 0xB0,      /**< HP enabled with cut-off 0.46 Hz.   */
} adxl317_acc_x_hp_t;

/**
 * @brief   ADXL317 Y HP filter.
 * @details The Y HP filter is HPF filter.
 */
typedef enum {
  ADXL317_ACC_Y_HP_DISABLED = 0x00, /**< HP disabled.                       */
  ADXL317_ACC_Y_HP_29 = 0x80,       /**< HP enabled with cut-off 29.8 Hz.   */
  ADXL317_ACC_Y_HP_7 = 0x90,        /**< HP enabled with cut-off 7.46 Hz.   */
  ADXL317_ACC_Y_HP_1 = 0xA0,        /**< HP enabled with cut-off 1.85 Hz.   */
  ADXL317_ACC_Y_HP_0_5 = 0xB0,      /**< HP enabled with cut-off 0.46 Hz.   */
} adxl317_acc_y_hp_t;

/**
 * @brief   ADXL317 Z HP filter.
 * @details The Z HP filter is HPF filter.
 */
typedef enum {
  ADXL317_ACC_Z_HP_DISABLED = 0x00, /**< HP disabled.                       */
  ADXL317_ACC_Z_HP_29 = 0x80,       /**< HP enabled with cut-off 29.8 Hz.   */
  ADXL317_ACC_Z_HP_7 = 0x90,        /**< HP enabled with cut-off 7.46 Hz.   */
  ADXL317_ACC_Z_HP_1 = 0xA0,        /**< HP enabled with cut-off 1.85 Hz.   */
  ADXL317_ACC_Z_HP_0_5 = 0xB0,      /**< HP enabled with cut-off 0.46 Hz.   */
} adxl317_acc_z_hp_t;

/**
 * @brief   ADXL317 HP filter configuration.
 */
typedef struct {
  adxl317_acc_x_hp_t x;
  adxl317_acc_y_hp_t y;
  adxl317_acc_z_hp_t z;
} adxl317_acc_hp_t;

/**
 * @brief   ADXL317 configuration structure.
 */
typedef struct {

#if (ADXL317_USE_I2C) || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this ADXL317.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this ADXL317.
   */
  const I2CConfig           *i2ccfg;
#endif /* ADXL317_USE_I2C */
  /**
   * @brief ADXL317 accelerometer subsystem initial sensitivity.
   */
  float                     *accsensitivity;
  /**
   * @brief ADXL317 accelerometer subsystem initial bias.
   */
  float                     *accbias;
#if ADXL317_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief   ADXL317 LP filter bandwidth.
   */
  adxl317_acc_lp_t          acclowpass;
  /**
   * @brief   ADXL317 HP filter bandwidth.
   */
  adxl317_acc_hp_t          acchighpass;
#endif
} ADXL317Config;

/**
 * @brief   @p ADXL317 specific methods.
 */
#define _adxl317_methods_alone


/**
 * @brief   @p ADXL317 specific methods with inherited ones.
 */
#define _adxl317_methods                                                    \
  _base_object_methods                                                      \
  _adxl317_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief   @p ADXL317 virtual methods table.
 */
struct ADXL317VMT {
  _adxl317_methods
};

/**
 * @brief   @p ADXL317Driver specific data.
 */
#define _adxl317_data                                                       \
  _base_sensor_data                                                         \
  /* Driver state.*/                                                        \
  adxl317_state_t           state;                                          \
  /* Current configuration data.*/                                          \
  const ADXL317Config       *config;                                        \
  /* Accelerometer subsystem axes number.*/                                 \
  size_t                    accaxes;                                        \
  /* Accelerometer subsystem current sensitivity.*/                         \
  float                     accsensitivity[ADXL317_ACC_NUMBER_OF_AXES];     \
  /* Accelerometer subsystem current bias .*/                               \
  float                     accbias[ADXL317_ACC_NUMBER_OF_AXES];            \
  /* Accelerometer subsystem current full scale value.*/                    \
  float                     accfullscale;                                   \
  /* Accelerometer communication tx buffer pointer.*/                       \
  uint8_t*                  commtxp;                                        \
  /* Accelerometer communication rx buffer pointer.*/                       \
  uint8_t*                  commrxp;

/**
 * @brief   ADXL317 3-axis accelerometer class.
 */
struct ADXL317Driver {
  /** @brief Virtual Methods Table.*/
  const struct ADXL317VMT     *vmt;
  /** @brief Base accelerometer interface.*/
  BaseAccelerometer           acc_if;
  _adxl317_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] devp      pointer to @p ADXL317Driver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define adxl317AccelerometerGetAxesNumber(devp)                             \
        accelerometerGetAxesNumber(&((devp)->acc_if))

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p ADXL317Driver.
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
#define adxl317AccelerometerReadRaw(devp, axes)                             \
        accelerometerReadRaw(&((devp)->acc_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p ADXL317Driver.
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
#define adxl317AccelerometerReadCooked(devp, axes)                          \
        accelerometerReadCooked(&((devp)->acc_if), axes)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void adxl317ObjectInit(ADXL317Driver *devp, uint8_t* txbp, uint8_t* rxbp);
  msg_t adxl317Start(ADXL317Driver *devp, const ADXL317Config *config);
  void adxl317Stop(ADXL317Driver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _ADXL317_H_ */

/** @} */

