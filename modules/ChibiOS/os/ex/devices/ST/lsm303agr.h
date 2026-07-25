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
 * @file    lsm303agr.h
 * @brief LSM303AGR MEMS interface module header.
 *
 * @addtogroup LSM303AGR
 * @ingroup EX_ST
 * @{
 */
#ifndef _LSM303AGR_H_
#define _LSM303AGR_H_

#include "ex_accelerometer.h"
#include "ex_compass.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   LSM303AGR driver version string.
 */
#define EX_LSM303AGR_VERSION                "1.0.2"

/**
 * @brief   LSM303AGR driver version major number.
 */
#define EX_LSM303AGR_MAJOR                  1

/**
 * @brief   LSM303AGR driver version minor number.
 */
#define EX_LSM303AGR_MINOR                  0

/**
 * @brief   LSM303AGR driver version patch number.
 */
#define EX_LSM303AGR_PATCH                  2
/** @} */

/**
 * @brief   LSM303AGR accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define LSM303AGR_ACC_NUMBER_OF_AXES        3U

#define LSM303AGR_ACC_2G                    2.0f
#define LSM303AGR_ACC_4G                    4.0f
#define LSM303AGR_ACC_8G                    8.0f
#define LSM303AGR_ACC_16G                   16.0f

#define LSM303AGR_ACC_SENS_2G               0.060f
#define LSM303AGR_ACC_SENS_4G               0.120f
#define LSM303AGR_ACC_SENS_8G               0.240f
#define LSM303AGR_ACC_SENS_16G              0.750f

#define LSM303AGR_ACC_BIAS                  0.0f
/** @} */

/**
 * @brief   LSM303AGR compass subsystem characteristics.
 * @note    Sensitivity is expressed as G/LSB whereas G stands for Gauss.
 * @note    Bias is expressed as G.
 *
 * @{
 */
#define LSM303AGR_COMP_NUMBER_OF_AXES       3U

#define LSM303AGR_COMP_50GA                 50.0f

#define LSM303AGR_COMP_SENS_50GA            0.00015f

#define LSM303AGR_COMP_BIAS                 0.0f
/** @} */

/**
 * @name   LSM303AGR communication interfaces related bit masks
 * @{
 */
#define LSM303AGR_DI_MASK                   0xFF
#define LSM303AGR_DI(n)                     (1 << n)
#define LSM303AGR_AD_MASK                   0x7F
#define LSM303AGR_AD(n)                     (1 << n)
#define LSM303AGR_MS                        (1 << 7)
/** @} */

/**
 * @name    LSM303AGR register addresses
 * @{
 */
#define LSM303AGR_AD_STATUS_REG_AUX_A       0x07
#define LSM303AGR_AD_OUT_TEMP_L_A           0x0C
#define LSM303AGR_AD_OUT_TEMP_H_A           0x0D
#define LSM303AGR_AD_INT_COUNTER_REG_A      0x0E
#define LSM303AGR_AD_WHO_AM_I_A             0x0F
#define LSM303AGR_AD_TEMP_CFG_REG_A         0x1F
#define LSM303AGR_AD_CTRL_REG1_A            0x20
#define LSM303AGR_AD_CTRL_REG2_A            0x21
#define LSM303AGR_AD_CTRL_REG3_A            0x22
#define LSM303AGR_AD_CTRL_REG4_A            0x23
#define LSM303AGR_AD_CTRL_REG5_A            0x24
#define LSM303AGR_AD_CTRL_REG6_A            0x25
#define LSM303AGR_AD_REFERENCE_A            0x26
#define LSM303AGR_AD_STATUS_REG_A           0x27
#define LSM303AGR_AD_OUT_X_L_A              0x28
#define LSM303AGR_AD_OUT_X_H_A              0x29
#define LSM303AGR_AD_OUT_Y_L_A              0x2A
#define LSM303AGR_AD_OUT_Y_H_A              0x2B
#define LSM303AGR_AD_OUT_Z_L_A              0x2C
#define LSM303AGR_AD_OUT_Z_H_A              0x2D
#define LSM303AGR_AD_FIFO_CTRL_REG_A        0x2E
#define LSM303AGR_AD_FIFO_SRC_REG_A         0x2F
#define LSM303AGR_AD_INT1_CFG_A             0x30
#define LSM303AGR_AD_INT1_SRC_A             0x31
#define LSM303AGR_AD_INT1_THS_A             0x32
#define LSM303AGR_AD_INT1_DURATION_A        0x33
#define LSM303AGR_AD_INT2_CFG_A             0x34
#define LSM303AGR_AD_INT2_SRC_A             0x35
#define LSM303AGR_AD_INT2_THS_A             0x36
#define LSM303AGR_AD_INT2_DURATION_A        0x37
#define LSM303AGR_AD_CLICK_CFG_A            0x38
#define LSM303AGR_AD_CLICK_SRC_A            0x39
#define LSM303AGR_AD_CLICK_THS_A            0x3A
#define LSM303AGR_AD_TIME_LIMIT_A           0x3B
#define LSM303AGR_AD_TIME_LATENCY_A         0x3C
#define LSM303AGR_AD_TIME_WINDOW_A          0x3D
#define LSM303AGR_AD_ACT_THS_A              0x3E
#define LSM303AGR_AD_ACT_DUR_A              0x3F
#define LSM303AGR_AD_OFFSET_X_REG_L_M       0x45
#define LSM303AGR_AD_OFFSET_X_REG_H_M       0x46
#define LSM303AGR_AD_OFFSET_Y_REG_L_M       0x47
#define LSM303AGR_AD_OFFSET_Y_REG_H_M       0x48
#define LSM303AGR_AD_OFFSET_Z_REG_L_M       0x49
#define LSM303AGR_AD_OFFSET_Z_REG_H_M       0x4A
#define LSM303AGR_AD_WHO_AM_I_M             0x4F
#define LSM303AGR_AD_CFG_REG_A_M            0x60
#define LSM303AGR_AD_CFG_REG_B_M            0x61
#define LSM303AGR_AD_CFG_REG_C_M            0x62
#define LSM303AGR_AD_INT_CRTL_REG_M         0x63
#define LSM303AGR_AD_INT_SOURCE_REG_M       0x64
#define LSM303AGR_AD_INT_THS_L_REG_M        0x65
#define LSM303AGR_AD_INT_THS_H_REG_M        0x66
#define LSM303AGR_AD_STATUS_REG_M           0x67
#define LSM303AGR_AD_OUTX_L_REG_M           0x68
#define LSM303AGR_AD_OUTX_H_REG_M           0x69
#define LSM303AGR_AD_OUTY_L_REG_M           0x6A
#define LSM303AGR_AD_OUTY_H_REG_M           0x6B
#define LSM303AGR_AD_OUTZ_L_REG_M           0x6C
#define LSM303AGR_AD_OUTZ_H_REG_M           0x6D
/** @} */

/**
 * @name    LSM303AGR_TEMP_CFG_REG_A register bits definitions
 * @{
 */
#define LSM303AGR_TEMP_CFG_REG_A_TEMP_EN0   (1 << 0)
#define LSM303AGR_TEMP_CFG_REG_A_TEMP_EN1   (1 << 0)
/** @} */

/**
 * @name    LSM303AGR_CTRL_REG1_A register bits definitions
 * @{
 */
#define LSM303AGR_CTRL_REG1_A_XEN           (1 << 0)
#define LSM303AGR_CTRL_REG1_A_YEN           (1 << 1)
#define LSM303AGR_CTRL_REG1_A_ZEN           (1 << 2)
#define LSM303AGR_CTRL_REG1_A_LPEN          (1 << 3)
#define LSM303AGR_CTRL_REG1_A_ODR0          (1 << 4)
#define LSM303AGR_CTRL_REG1_A_ODR1          (1 << 5)
#define LSM303AGR_CTRL_REG1_A_ODR2          (1 << 6)
#define LSM303AGR_CTRL_REG1_A_ODR3          (1 << 7)
/** @} */

/**
 * @name    LSM303AGR_CTRL_REG2_A register bits definitions
 * @{
 */ 
#define LSM303AGR_CTRL_REG2_A_HPIS1         (1 << 0)
#define LSM303AGR_CTRL_REG2_A_HPIS2         (1 << 1)
#define LSM303AGR_CTRL_REG2_A_HPCLICK       (1 << 2)
#define LSM303AGR_CTRL_REG2_A_FDS           (1 << 3)
#define LSM303AGR_CTRL_REG2_A_HPCF1         (1 << 4)
#define LSM303AGR_CTRL_REG2_A_HPCF2         (1 << 5)
#define LSM303AGR_CTRL_REG2_A_HPM0          (1 << 6)
#define LSM303AGR_CTRL_REG2_A_HPM1          (1 << 7)
/** @} */

/**
 * @name    LSM303AGR_CTRL_REG3_A register bits definitions
 * @{
 */
#define LSM303AGR_CTRL_REG3_A_I1_OVERRUN    (1 << 1)
#define LSM303AGR_CTRL_REG3_A_I1_WTM        (1 << 2)
#define LSM303AGR_CTRL_REG3_A_I1_DRDY2      (1 << 3)
#define LSM303AGR_CTRL_REG3_A_I1_DRDY1      (1 << 4)
#define LSM303AGR_CTRL_REG3_A_I1_AOI2       (1 << 5)
#define LSM303AGR_CTRL_REG3_A_I1_AOI1       (1 << 6)
#define LSM303AGR_CTRL_REG3_A_I1_CLICK      (1 << 7)
/** @} */

/**
 * @name    LSM303AGR_CTRL_REG4_A register bits definitions
 * @{
 */
#define LSM303AGR_CTRL_REG4_A_SPI_ENABLE    (1 << 0) 
#define LSM303AGR_CTRL_REG4_A_ST0           (1 << 1)
#define LSM303AGR_CTRL_REG4_A_ST1           (1 << 2)
#define LSM303AGR_CTRL_REG4_A_HR            (1 << 3)
#define LSM303AGR_CTRL_REG4_A_FS_MASK       0x30
#define LSM303AGR_CTRL_REG4_A_FS0           (1 << 4)
#define LSM303AGR_CTRL_REG4_A_FS1           (1 << 5)
#define LSM303AGR_CTRL_REG4_A_BLE           (1 << 6)
#define LSM303AGR_CTRL_REG4_A_BDU           (1 << 7)
/** @} */

/**
 * @name    LSM303AGR_CTRL_REG5_A register bits definitions
 * @{
 */
#define LSM303AGR_CTRL_REG5_A_D4D_INT2      (1 << 0)
#define LSM303AGR_CTRL_REG5_A_LIR_INT2      (1 << 1)
#define LSM303AGR_CTRL_REG5_A_D4D_INT1      (1 << 2)
#define LSM303AGR_CTRL_REG5_A_LIR_INT1      (1 << 3)
#define LSM303AGR_CTRL_REG5_A_FIFO_EN       (1 << 6)
#define LSM303AGR_CTRL_REG5_A_BOOT          (1 << 7)
/** @} */

/**
 * @name    LSM303AGR_CTRL_REG6_A register bits definitions
 * @{
 */
#define LSM303AGR_CTRL_REG6_A_H_LACTIVE     (1 << 1)
#define LSM303AGR_CTRL_REG6_A_P2_ACT        (1 << 3)
#define LSM303AGR_CTRL_REG6_A_BOOT_I2       (1 << 4)
#define LSM303AGR_CTRL_REG6_A_I2_INT2       (1 << 5)
#define LSM303AGR_CTRL_REG6_A_I2_INT1       (1 << 6)
#define LSM303AGR_CTRL_REG6_A_I2_CLICKEN    (1 << 7)
/** @} */

/**
 * @name    LSM303AGR_CFG_REG_A register bits definitions
 * @{
 */
#define LSM303AGR_CFG_REG_A_M_MD0           (1 << 0)
#define LSM303AGR_CFG_REG_A_M_MD1           (1 << 1)
#define LSM303AGR_CFG_REG_A_M_ODR0          (1 << 2)
#define LSM303AGR_CFG_REG_A_M_ODR1          (1 << 3)
#define LSM303AGR_CFG_REG_A_M_LP            (1 << 4)
#define LSM303AGR_CFG_REG_A_M_SOFT_RST      (1 << 5)
#define LSM303AGR_CFG_REG_A_M_REBOOT        (1 << 6)
#define LSM303AGR_CFG_REG_A_M_COMP_TEMP_EN  (1 << 7)
/** @} */

/**
 * @name    LSM303AGR_CFG_REG_B register bits definitions
 * @{
 */
#define LSM303AGR_CFG_REG_B_M_LPF           (1 << 0)
#define LSM303AGR_CFG_REG_B_M_OFF_CANC      (1 << 1)
#define LSM303AGR_CFG_REG_B_M_SET_FREQ      (1 << 2)
#define LSM303AGR_CFG_REG_B_M_INT_ON        (1 << 3)
#define LSM303AGR_CFG_REG_B_M_OFF_CANC_OS   (1 << 4)
/** @} */

/**
 * @name    LSM303AGR_CFG_REG_C register bits definitions
 * @{
 */
#define LSM303AGR_CFG_REG_C_M_INT_MAG       (1 << 0)
#define LSM303AGR_CFG_REG_C_M_SELF_TEST     (1 << 1)
#define LSM303AGR_CFG_REG_C_M_BLE           (1 << 3)
#define LSM303AGR_CFG_REG_C_M_BDU           (1 << 4)
#define LSM303AGR_CFG_REG_C_M_I2C_DIS       (1 << 5)
#define LSM303AGR_CFG_REG_C_M_INT_MAG_PIN   (1 << 6)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LSM303AGR SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM303AGR_USE_SPI) || defined(__DOXYGEN__)
#define LSM303AGR_USE_SPI                   FALSE
#endif

/**
 * @brief   LSM303AGR shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LSM303AGR_SHARED_SPI) || defined(__DOXYGEN__)
#define LSM303AGR_SHARED_SPI                FALSE
#endif

/**
 * @brief   LSM303AGR I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LSM303AGR_USE_I2C) || defined(__DOXYGEN__)
#define LSM303AGR_USE_I2C                   TRUE
#endif

/**
 * @brief   LSM303AGR shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LSM303AGR_SHARED_I2C) || defined(__DOXYGEN__)
#define LSM303AGR_SHARED_I2C                FALSE
#endif

/**
 * @brief   LSM303AGR advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM303AGR_USE_ADVANCED) || defined(__DOXYGEN__)
#define LSM303AGR_USE_ADVANCED          FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(LSM303AGR_USE_SPI ^ LSM303AGR_USE_I2C)
#error "LSM303AGR_USE_SPI and LSM303AGR_USE_I2C cannot be both true or both false"
#endif

#if LSM303AGR_USE_SPI && !HAL_USE_SPI
#error "LSM303AGR_USE_SPI requires HAL_USE_SPI"
#endif

#if LSM303AGR_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "LSM303AGR_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if LSM303AGR_USE_I2C && !HAL_USE_I2C
#error "LSM303AGR_USE_I2C requires HAL_USE_I2C"
#endif

#if LSM303AGR_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "LSM303AGR_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for LSM303AGR over SPI.
 */
#if LSM303AGR_USE_SPI
#error "LSM303AGR over SPI still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    LSM303AGR accelerometer subsystem data structures and types.
 * @{
 */
/**
 * @brief Structure representing a LSM303AGR driver.
 */
typedef struct LSM303AGRDriver LSM303AGRDriver;

/**
 * @brief LSM303AGR accelerometer subsystem full scale.
 */
typedef enum {
  LSM303AGR_ACC_FS_2G = 0x00,      /**< Full scale �2g.                    */
  LSM303AGR_ACC_FS_4G = 0x10,      /**< Full scale �4g.                    */
  LSM303AGR_ACC_FS_8G = 0x20,      /**< Full scale �8g.                    */
  LSM303AGR_ACC_FS_16G  = 0x30     /**< Full scale �16g.                   */
} lsm303agr_acc_fs_t;

/**
 * @brief LSM303AGR accelerometer subsystem output data rate.
 */
typedef enum {
  LSM303AGR_ACC_ODR_PD = 0x00,     /**< Power down                         */
  LSM303AGR_ACC_ODR_1Hz = 0x10,    /**< ODR 1 Hz                           */
  LSM303AGR_ACC_ODR_10Hz = 0x20,   /**< ODR 10 Hz                          */
  LSM303AGR_ACC_ODR_25Hz = 0x30,   /**< ODR 25 Hz                          */
  LSM303AGR_ACC_ODR_50Hz = 0x40,   /**< ODR 50 Hz                          */
  LSM303AGR_ACC_ODR_100Hz = 0x50,  /**< ODR 100 Hz                         */
  LSM303AGR_ACC_ODR_200Hz = 0x60,  /**< ODR 200 Hz                         */
  LSM303AGR_ACC_ODR_400Hz = 0x70,  /**< ODR 400 Hz                         */
  LSM303AGR_ACC_ODR_1620Hz = 0x80, /**< ODR 1620 Hz (LP only)              */
  LSM303AGR_ACC_ODR_1344Hz = 0x90  /**< ODR 1344 Hz or 5376 Hz in LP       */
} lsm303agr_acc_odr_t;

/**
 * @brief LSM303AGR accelerometer subsystem axes enabling.
 */
typedef enum {
  LSM303AGR_ACC_AE_DISABLED = 0x00,/**< All axes disabled.                 */
  LSM303AGR_ACC_AE_X = 0x01,       /**< Only X-axis enabled.               */
  LSM303AGR_ACC_AE_Y = 0x02,       /**< Only Y-axis enabled.               */
  LSM303AGR_ACC_AE_XY = 0x03,      /**< X and Y axes enabled.              */
  LSM303AGR_ACC_AE_Z = 0x04,       /**< Only Z-axis enabled.               */
  LSM303AGR_ACC_AE_XZ = 0x05,      /**< X and Z axes enabled.              */
  LSM303AGR_ACC_AE_YZ = 0x06,      /**< Y and Z axes enabled.              */
  LSM303AGR_ACC_AE_XYZ = 0x07      /**< All axes enabled.                  */
} lsm303agr_acc_ae_t;

/**
 * @brief LSM303AGR accelerometer subsystem operation mode.
 */
typedef enum {
  LSM303AGR_ACC_MODE_NORM = 0,     /**< Normal mode.                       */
  LSM303AGR_ACC_MODE_LPOW = 1,     /**< Low power mode.                    */
  LSM303AGR_ACC_MODE_HRES = 2      /**< High resolution mode.              */
} lsm303agr_acc_mode_t;

/**
 * @brief LSM303AGR accelerometer subsystem block data update.
 */
typedef enum {
  LSM303AGR_ACC_BDU_CONT = 0x00,   /**< Continuous update                  */
  LSM303AGR_ACC_BDU_BLOCK = 0x80   /**< Update blocked                     */
} lsm303agr_acc_bdu_t;

/**
 * @brief  LSM303AGR accelerometer endianness.
 */
typedef enum {
  LSM303AGR_ACC_END_LITTLE = 0x00, /**< Little Endian                      */
  LSM303AGR_ACC_END_BIG = 0x40     /**< Big Endian                         */
} lsm303agr_acc_end_t;
/** @} */

/**
 * @name    LSM303AGR compass subsystem data structures and types.
 * @{
 */
/**
 * @brief LSM303AGR compass subsystem output data rate.
 */
typedef enum {
  LSM303AGR_COMP_ODR_10HZ = 0x00,  /**< ODR 10 Hz                          */
  LSM303AGR_COMP_ODR_20HZ = 0x04,  /**< ODR 20 Hz                          */
  LSM303AGR_COMP_ODR_50HZ = 0x08,  /**< ODR 50 Hz                          */
  LSM303AGR_COMP_ODR_100HZ = 0x0C  /**< ODR 100 Hz                         */
} lsm303agr_comp_odr_t;

/**
 * @brief LSM303AGR compass subsystem working mode.
 */
typedef enum {
  LSM303AGR_COMP_MODE_NORM = 0x00,  /**< Continuous-Conversion Mode         */
  LSM303AGR_COMP_MODE_SINGLE = 0x01,/**< Single-Conversion Mode             */
  LSM303AGR_COMP_MODE_IDLE = 0x02   /**< Sleep Mode                         */
} lsm303agr_comp_mode_t;

/**
 * @brief LSM303AGR compass subsystem working mode.
 */
typedef enum {
  LSM303AGR_COMP_LPOW_DIS = 0x00,   /**< High Resolution Mode               */
  LSM303AGR_COMP_LPOW_EN = 0x10     /**< Low Power Mode                     */
} lsm303agr_comp_lpow_t;
/** @} */

/**
 * @name    LSM303AGR main system data structures and types.
 * @{
 */
/**
 * @brief Driver state machine possible states.
 */
typedef enum {
  LSM303AGR_UNINIT = 0,            /**< Not initialized.                   */
  LSM303AGR_STOP = 1,              /**< Stopped.                           */
  LSM303AGR_READY = 2,             /**< Ready.                             */
} lsm303agr_state_t;

/**
 * @brief LSM303AGR configuration structure.
 */
typedef struct {
  /**
   * @brief I2C driver associated to this LSM303AGR.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this LSM303AGR.
   */
  const I2CConfig           *i2ccfg;
  /**
   * @brief LSM303AGR accelerometer subsystem initial sensitivity.
   */
  float                     *accsensitivity;
  /**
   * @brief LSM303AGR accelerometer subsystem initial bias.
   */
  float                     *accbias;
  /**
   * @brief LSM303AGR accelerometer subsystem initial full scale.
   */
  lsm303agr_acc_fs_t        accfullscale;
  /**
   * @brief LSM303AGR accelerometer subsystem output data rate.
   */
  lsm303agr_acc_odr_t       accodr;
#if LSM303AGR_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief LSM303AGR accelerometer subsystem mode.
   */
  lsm303agr_acc_mode_t      accmode;
  /**
   * @brief LSM303AGR accelerometer subsystem block data update.
   */
  lsm303agr_acc_bdu_t       accbdu;
  /**
   * @brief  LSM303AGR accelerometer endianness.
   */
  lsm303agr_acc_end_t       accendianess;
#endif
  /**
   * @brief LSM303AGR compass initial sensitivity.
   */
  float                     *compsensitivity;
  /**
   * @brief LSM303AGR compass initial bias.
   */
  float                     *compbias;
  /**
   * @brief LSM303AGR compass subsystem output data rate.
   */
  lsm303agr_comp_odr_t      compodr;
#if LSM303AGR_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief LSM303AGR compass subsystem working mode.
   */
  lsm303agr_comp_mode_t     compmode;
  /**
   * @brief LSM303AGR compass subsystem lowpower mode.
   */  
  lsm303agr_comp_lpow_t     complp;
#endif
} LSM303AGRConfig;

/**
 * @brief   @p LSM303AGR specific methods.
 */
#define _lsm303agr_methods_alone                                            \
  /* Change full scale value of LSM303AGR accelerometer subsystem.*/        \
  msg_t (*acc_set_full_scale)(LSM303AGRDriver *devp,                        \
                              lsm303agr_acc_fs_t fs);

/**
 * @brief   @p LSM303AGR specific methods with inherited ones.
 */
#define _lsm303agr_methods                                                  \
  _base_object_methods                                                      \
  _lsm303agr_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief @p LSM303AGR virtual methods table.
 */
struct LSM303AGRVMT {
  _lsm303agr_methods
};

/**
 * @brief @p LSM303AGRDriver specific data.
 */
#define _lsm303agr_data                                                     \
  _base_sensor_data                                                         \
  /* Driver state.*/                                                        \
  lsm303agr_state_t        state;                                           \
  /* Current configuration data.*/                                          \
  const LSM303AGRConfig    *config;                                         \
  /* Accelerometer subsystem axes number.*/                                 \
  size_t                    accaxes;                                        \
  /* Accelerometer subsystem current sensitivity.*/                         \
  float                     accsensitivity[LSM303AGR_ACC_NUMBER_OF_AXES];   \
  /* Accelerometer subsystem current bias .*/                               \
  float                     accbias[LSM303AGR_ACC_NUMBER_OF_AXES];          \
  /* Accelerometer subsystem current full scale value.*/                    \
  float                     accfullscale;                                   \
  /* Compass subsystem axes number.*/                                       \
  size_t                    compaxes;                                       \
  /* Compass subsystem current sensitivity.*/                               \
  float                     compsensitivity[LSM303AGR_COMP_NUMBER_OF_AXES]; \
  /* Compass subsystem current bias.*/                                      \
  float                     compbias[LSM303AGR_COMP_NUMBER_OF_AXES];        \
  /* Compass subsystem current full scale value.*/                          \
  float                     compfullscale;

/**
 * @brief LSM303AGR 6-axis accelerometer/compass class.
 */
struct LSM303AGRDriver {
  /** @brief Virtual Methods Table.*/
  const struct LSM303AGRVMT  *vmt;
  /** @brief Base accelerometer interface.*/
  BaseAccelerometer           acc_if;
  /** @brief Base compass interface.*/
  BaseCompass                 comp_if;
  _lsm303agr_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lsm303agrAccelerometerGetAxesNumber(devp)                           \
        accelerometerGetAxesNumber(&((devp)->acc_if))

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
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
#define lsm303agrAccelerometerReadRaw(devp, axes)                           \
        accelerometerReadRaw(&((devp)->acc_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
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
#define lsm303agrAccelerometerReadCooked(devp, axes)                        \
        accelerometerReadCooked(&((devp)->acc_if), axes)

/**
 * @brief   Set bias values for the BaseAccelerometer.
 * @note    Bias must be expressed as milli-G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm303agrAccelerometerSetBias(devp, bp)                             \
        accelerometerSetBias(&((devp)->acc_if), bp)

/**
 * @brief   Reset bias values for the BaseAccelerometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm303agrAccelerometerResetBias(devp)                               \
        accelerometerResetBias(&((devp)->acc_if))

/**
 * @brief   Set sensitivity values for the BaseAccelerometer.
 * @note    Sensitivity must be expressed as milli-G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm303agrAccelerometerSetSensitivity(devp, sp)                      \
        accelerometerSetSensitivity(&((devp)->acc_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseAccelerometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm303agrAccelerometerResetSensitivity(devp)                        \
        accelerometerResetSensitivity(&((devp)->acc_if))

/**
 * @brief   Changes the LSM303AGRDriver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm303agrAccelerometerSetFullScale(devp, fs)                        \
        (devp)->vmt->acc_set_full_scale(devp, fs)

/**
 * @brief   Return the number of axes of the BaseCompass.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lsm303agrCompassGetAxesNumber(devp)                                 \
        compassGetAxesNumber(&((devp)->comp_if))

/**
 * @brief   Retrieves raw data from the BaseCompass.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseCompass axes number.
 *
 * @param[in] devp      pointer to @p BaseCompass interface.
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
#define lsm303agrCompassReadRaw(devp, axes)                                 \
        compassReadRaw(&((devp)->comp_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseCompass.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as G.
 * @note    The axes array must be at least the same size of the
 *          BaseCompass axes number.
 *
 * @param[in] devp      pointer to @p BaseCompass interface.
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
#define lsm303agrCompassReadCooked(devp, axes)                              \
        compassReadCooked(&((devp)->comp_if), axes)

/**
 * @brief   Set bias values for the BaseCompass.
 * @note    Bias must be expressed as G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseCompass axes number.
 *
 * @param[in] devp      pointer to @p BaseCompass interface.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm303agrCompassSetBias(devp, bp)                                   \
        compassSetBias(&((devp)->comp_if), bp)

/**
 * @brief   Reset bias values for the BaseCompass.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm303agrCompassResetBias(devp)                                     \
        compassResetBias(&((devp)->comp_if))

/**
 * @brief   Set sensitivity values for the BaseCompass.
 * @note    Sensitivity must be expressed as G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseCompass axes number.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm303agrCompassSetSensitivity(devp, sp)                            \
        compassSetSensitivity(&((devp)->comp_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseCompass.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LSM303AGRDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm303agrCompassResetSensitivity(devp)                              \
        compassResetSensitivity(&((devp)->comp_if))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void lsm303agrObjectInit(LSM303AGRDriver *devp);
  void lsm303agrStart(LSM303AGRDriver *devp, const LSM303AGRConfig *config);
  void lsm303agrStop(LSM303AGRDriver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _LSM303AGR_H_ */

/** @} */
