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
 * @file    lsm6dsl.h
 * @brief   LSM6DSL MEMS interface module header.
 *
 * @addtogroup LSM6DSL
 * @ingroup EX_ST
 * @{
 */
#ifndef _LSM6DSL_H_
#define _LSM6DSL_H_

#include "ex_accelerometer.h"
#include "ex_gyroscope.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   LSM6DSL driver version string.
 */
#define EX_LSM6DSL_VERSION                  "1.0.2"

/**
 * @brief   LSM6DSL driver version major number.
 */
#define EX_LSM6DSL_MAJOR                    1

/**
 * @brief   LSM6DSL driver version minor number.
 */
#define EX_LSM6DSL_MINOR                    0

/**
 * @brief   LSM6DSL driver version patch number.
 */
#define EX_LSM6DSL_PATCH                    2
/** @} */

/**
 * @brief   LSM6DSL accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define LSM6DSL_ACC_NUMBER_OF_AXES          3U

#define LSM6DSL_ACC_2G                      2.0f
#define LSM6DSL_ACC_4G                      4.0f
#define LSM6DSL_ACC_8G                      8.0f
#define LSM6DSL_ACC_16G                     16.0f

#define LSM6DSL_ACC_SENS_2G                 0.061f
#define LSM6DSL_ACC_SENS_4G                 0.122f
#define LSM6DSL_ACC_SENS_8G                 0.244f
#define LSM6DSL_ACC_SENS_16G                0.488f

#define LSM6DSL_ACC_BIAS                    0.0f
/** @} */

/**
 * @brief   L3GD20 gyroscope system characteristics.
 * @note    Sensitivity is expressed as DPS/LSB whereas DPS stand for Degree
 *          per second [�/s].
 * @note    Bias is expressed as DPS.
 *
 * @{
 */
#define LSM6DSL_GYRO_NUMBER_OF_AXES         3U

#define LSM6DSL_GYRO_125DPS                 125.0f
#define LSM6DSL_GYRO_250DPS                 250.0f
#define LSM6DSL_GYRO_500DPS                 500.0f
#define LSM6DSL_GYRO_1000DPS                1000.0f
#define LSM6DSL_GYRO_2000DPS                2000.0f

#define LSM6DSL_GYRO_SENS_125DPS            0.004375f
#define LSM6DSL_GYRO_SENS_250DPS            0.008750f
#define LSM6DSL_GYRO_SENS_500DPS            0.017500f
#define LSM6DSL_GYRO_SENS_1000DPS           0.035000f
#define LSM6DSL_GYRO_SENS_2000DPS           0.070000f

#define LSM6DSL_GYRO_BIAS                   0.0f
/** @} */

/**
 * @name   LSM6DSL communication interfaces related bit masks
 * @{
 */
#define LSM6DSL_DI_MASK                     0xFF
#define LSM6DSL_DI(n)                       (1 << n)
#define LSM6DSL_AD_MASK                     0x7F
#define LSM6DSL_AD(n)                       (1 << n)
#define LSM6DSL_MS                          (1 << 7)
/** @} */

/**
 * @name   LSM6DSL register addresses
 * @{
 */
#define LSM6DSL_AD_FUNC_CFG_ACCESS          0x01
#define LSM6DSL_AD_SENSOR_SYNC_TIME_FRAME   0x04
#define LSM6DSL_AD_SENSOR_SYNC_RES_RATIO    0x05
#define LSM6DSL_AD_FIFO_CTRL1               0x06
#define LSM6DSL_AD_FIFO_CTRL2               0x07
#define LSM6DSL_AD_FIFO_CTRL3               0x08
#define LSM6DSL_AD_FIFO_CTRL4               0x09
#define LSM6DSL_AD_FIFO_CTRL5               0x0A
#define LSM6DSL_AD_DRDY_PULSE_CFG_G         0x0B
#define LSM6DSL_AD_INT1_CTRL                0x0D
#define LSM6DSL_AD_INT2_CTRL                0x0E
#define LSM6DSL_AD_WHO_AM_I                 0x0F
#define LSM6DSL_AD_CTRL1_XL                 0x10
#define LSM6DSL_AD_CTRL2_G                  0x11
#define LSM6DSL_AD_CTRL3_C                  0x12
#define LSM6DSL_AD_CTRL4_C                  0x13
#define LSM6DSL_AD_CTRL5_C                  0x14
#define LSM6DSL_AD_CTRL6_C                  0x15
#define LSM6DSL_AD_CTRL7_G                  0x16
#define LSM6DSL_AD_CTRL8_XL                 0x17
#define LSM6DSL_AD_CTRL9_XL                 0x18
#define LSM6DSL_AD_CTRL10_C                 0x19
#define LSM6DSL_AD_MASTER_CONFIG            0x1A
#define LSM6DSL_AD_WAKE_UP_SRC              0x1B
#define LSM6DSL_AD_TAP_SRC                  0x1C
#define LSM6DSL_AD_D6D_SRC                  0x1D
#define LSM6DSL_AD_STATUS_REG               0x1E
#define LSM6DSL_AD_OUT_TEMP_L               0x20
#define LSM6DSL_AD_OUT_TEMP_H               0x21
#define LSM6DSL_AD_OUTX_L_G                 0x22
#define LSM6DSL_AD_OUTX_H_G                 0x23
#define LSM6DSL_AD_OUTY_L_G                 0x24
#define LSM6DSL_AD_OUTY_H_G                 0x25
#define LSM6DSL_AD_OUTZ_L_G                 0x26
#define LSM6DSL_AD_OUTZ_H_G                 0x27
#define LSM6DSL_AD_OUTX_L_XL                0x28
#define LSM6DSL_AD_OUTX_H_XL                0x29
#define LSM6DSL_AD_OUTY_L_XL                0x2A
#define LSM6DSL_AD_OUTY_H_XL                0x2B
#define LSM6DSL_AD_OUTZ_L_XL                0x2C
#define LSM6DSL_AD_OUTZ_H_XL                0x2D
#define LSM6DSL_AD_SENSORHUB1_REG           0x2E
#define LSM6DSL_AD_SENSORHUB2_REG           0x2F
#define LSM6DSL_AD_SENSORHUB3_REG           0x30
#define LSM6DSL_AD_SENSORHUB4_REG           0x31
#define LSM6DSL_AD_SENSORHUB5_REG           0x32
#define LSM6DSL_AD_SENSORHUB6_REG           0x33
#define LSM6DSL_AD_SENSORHUB7_REG           0x34
#define LSM6DSL_AD_SENSORHUB8_REG           0x35
#define LSM6DSL_AD_SENSORHUB9_REG           0x36
#define LSM6DSL_AD_SENSORHUB10_REG          0x37
#define LSM6DSL_AD_SENSORHUB11_REG          0x38
#define LSM6DSL_AD_SENSORHUB12_REG          0x39
#define LSM6DSL_AD_FIFO_STATUS1             0x3A
#define LSM6DSL_AD_FIFO_STATUS2             0x3B
#define LSM6DSL_AD_FIFO_STATUS3             0x3C
#define LSM6DSL_AD_FIFO_STATUS4             0x3D
#define LSM6DSL_AD_FIFO_DATA_OUT_L          0x3E
#define LSM6DSL_AD_FIFO_DATA_OUT_H          0x3F
#define LSM6DSL_AD_TIMESTAMP0_REG           0x40
#define LSM6DSL_AD_TIMESTAMP1_REG           0x41
#define LSM6DSL_AD_TIMESTAMP2_REG           0x42
#define LSM6DSL_AD_STEP_TIMESTAMP_L         0x49
#define LSM6DSL_AD_STEP_TIMESTAMP_H         0x4A
#define LSM6DSL_AD_STEP_COUNTER_L           0x4B
#define LSM6DSL_AD_STEP_COUNTER_H           0x4C
#define LSM6DSL_AD_SENSORHUB13_REG          0x4D
#define LSM6DSL_AD_SENSORHUB14_REG          0x4E
#define LSM6DSL_AD_SENSORHUB15_REG          0x4F
#define LSM6DSL_AD_SENSORHUB16_REG          0x50
#define LSM6DSL_AD_SENSORHUB17_REG          0x51
#define LSM6DSL_AD_SENSORHUB18_REG          0x52
#define LSM6DSL_AD_FUNC_SRC1                0x53
#define LSM6DSL_AD_FUNC_SRC2                0x54
#define LSM6DSL_AD_WRIST_TILT_IA            0x55
#define LSM6DSL_AD_TAP_CFG                  0x58
#define LSM6DSL_AD_TAP_THS_6D               0x59
#define LSM6DSL_AD_INT_DUR2                 0x5A
#define LSM6DSL_AD_WAKE_UP_THS              0x5B
#define LSM6DSL_AD_WAKE_UP_DUR              0x5C
#define LSM6DSL_AD_FREE_FALL                0x5D
#define LSM6DSL_AD_MD1_CFG                  0x5E
#define LSM6DSL_AD_MD2_CFG                  0x5F
#define LSM6DSL_AD_MASTER_CMD_CODE          0x60
#define LSM6DSL_AD_SENS_SYNC_SPI_ERROR_CODE 0x61
#define LSM6DSL_AD_OUT_MAG_RAW_X_L          0x66
#define LSM6DSL_AD_OUT_MAG_RAW_X_H          0x67
#define LSM6DSL_AD_OUT_MAG_RAW_Y_L          0x68
#define LSM6DSL_AD_OUT_MAG_RAW_Y_H          0x69
#define LSM6DSL_AD_OUT_MAG_RAW_Z_L          0x6A
#define LSM6DSL_AD_OUT_MAG_RAW_Z_H          0x6B
#define LSM6DSL_AD_X_OFS_USR                0x73
#define LSM6DSL_AD_Y_OFS_USR                0x74
#define LSM6DSL_AD_Z_OFS_USR                0x75
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL1_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL1_XL_BW0_XL              (1 << 0)
#define LSMDSL_CTRL1_XL_LPF1_BW_SEL         (1 << 1)
#define LSMDSL_CTRL1_XL_FS_MASK             0x0C
#define LSMDSL_CTRL1_XL_FS_XL0              (1 << 2)
#define LSMDSL_CTRL1_XL_FS_XL1              (1 << 3)
#define LSMDSL_CTRL1_XL_ODR_XL0             (1 << 4)
#define LSMDSL_CTRL1_XL_ODR_XL1             (1 << 5)
#define LSMDSL_CTRL1_XL_ODR_XL2             (1 << 6)
#define LSMDSL_CTRL1_XL_ODR_XL3             (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL2_G register bits definitions
 * @{
 */
#define LSMDSL_CTRL2_G_FS_MASK              0x0E
#define LSMDSL_CTRL2_G_FS_125               (1 << 1)
#define LSMDSL_CTRL2_G_FS_G0                (1 << 2)
#define LSMDSL_CTRL2_G_FS_G1                (1 << 3)
#define LSMDSL_CTRL2_G_ODR_G0               (1 << 4)
#define LSMDSL_CTRL2_G_ODR_G1               (1 << 5)
#define LSMDSL_CTRL2_G_ODR_G2               (1 << 6)
#define LSMDSL_CTRL2_G_ODR_G3               (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL3_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL3_C_SW_RESET             (1 << 0)
#define LSMDSL_CTRL3_C_BLE                  (1 << 1)
#define LSMDSL_CTRL3_C_IF_INC               (1 << 2)
#define LSMDSL_CTRL3_C_SIM                  (1 << 3)
#define LSMDSL_CTRL3_C_PP_OD                (1 << 4)
#define LSMDSL_CTRL3_C_H_LACTIVE            (1 << 5)
#define LSMDSL_CTRL3_C_BDU                  (1 << 6)
#define LSMDSL_CTRL3_C_BOOT                 (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL4_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL4_C_NOT_USED_01          (1 << 0)
#define LSMDSL_CTRL4_C_LPF1_SEL_G           (1 << 1)
#define LSMDSL_CTRL4_C_I2C_DISABLE          (1 << 2)
#define LSMDSL_CTRL4_C_DRDY_MASK            (1 << 3)
#define LSMDSL_CTRL4_C_DEN_DRDY_IN          (1 << 4)
#define LSMDSL_CTRL4_C_INT2_ON_INT          (1 << 5)
#define LSMDSL_CTRL4_C_SLEEP                (1 << 6)
#define LSMDSL_CTRL4_C_DEN_XL_EN            (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL5_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL5_C_ST0_XL               (1 << 0)
#define LSMDSL_CTRL5_C_ST1_XL               (1 << 1)
#define LSMDSL_CTRL5_C_ST0_G                (1 << 2)
#define LSMDSL_CTRL5_C_ST1_G                (1 << 3)
#define LSMDSL_CTRL5_C_DEN_LH               (1 << 4)
#define LSMDSL_CTRL5_C_ROUNDING0            (1 << 5)
#define LSMDSL_CTRL5_C_ROUNDING1            (1 << 6)
#define LSMDSL_CTRL5_C_ROUNDING2            (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL6_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL6_C_FTYPE_0              (1 << 0)
#define LSMDSL_CTRL6_C_FTYPE_1              (1 << 1)
#define LSMDSL_CTRL6_C_USR_OFF_W            (1 << 3)
#define LSMDSL_CTRL6_C_XL_HM_MODE           (1 << 4)
#define LSMDSL_CTRL6_C_LVL2_EN              (1 << 5)
#define LSMDSL_CTRL6_C_LVL_EN               (1 << 6)
#define LSMDSL_CTRL6_C_TRIG_EN              (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL7_G register bits definitions
 * @{
 */
#define LSMDSL_CTRL7_G_ROUNDING_ST          (1 << 2)
#define LSMDSL_CTRL7_G_HPM0_G               (1 << 4)
#define LSMDSL_CTRL7_G_HPM1_G               (1 << 5)
#define LSMDSL_CTRL7_G_HP_EN_G              (1 << 6)
#define LSMDSL_CTRL7_G_G_HM_MODE            (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL8_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL8_XL_LOW_PASS_ON         (1 << 0)
#define LSMDSL_CTRL8_XL_HP_SLOPE_XL         (1 << 2)
#define LSMDSL_CTRL8_XL_INPUT_COMPO         (1 << 3)
#define LSMDSL_CTRL8_XL_HP_REF_MODE         (1 << 4)
#define LSMDSL_CTRL8_XL_HPCF_XL0            (1 << 5)
#define LSMDSL_CTRL8_XL_HPCF_XL1            (1 << 6)
#define LSMDSL_CTRL8_XL_LPF2_XL_EN          (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL9_XL register bits definitions
 * @{
 */
#define LSMDSL_CTRL9_XL_SOFT_EN             (1 << 2)
#define LSMDSL_CTRL9_XL_DEN_XL_G            (1 << 4)
#define LSMDSL_CTRL9_XL_DEN_Z               (1 << 5)
#define LSMDSL_CTRL9_XL_DEN_Y               (1 << 6)
#define LSMDSL_CTRL9_XL_DEN_X               (1 << 7)
/** @} */

/**
 * @name    LSM6DSL_AD_CTRL10_C register bits definitions
 * @{
 */
#define LSMDSL_CTRL10_C_SIGN_MOTION         (1 << 0)
#define LSMDSL_CTRL10_C_PEDO_RST_ST         (1 << 1)
#define LSMDSL_CTRL10_C_FUNC_EN             (1 << 2)
#define LSMDSL_CTRL10_C_TILT_EN             (1 << 3)
#define LSMDSL_CTRL10_C_PEDO_EN             (1 << 4)
#define LSMDSL_CTRL10_C_TIMER_EN            (1 << 5)
#define LSMDSL_CTRL10_C_WRIST_TILT          (1 << 7)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LSM6DSL SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM6DSL_USE_SPI) || defined(__DOXYGEN__)
#define LSM6DSL_USE_SPI                     FALSE
#endif

/**
 * @brief   LSM6DSL shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LSM6DSL_SHARED_SPI) || defined(__DOXYGEN__)
#define LSM6DSL_SHARED_SPI                  FALSE
#endif

/**
 * @brief   LSM6DSL I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LSM6DSL_USE_I2C) || defined(__DOXYGEN__)
#define LSM6DSL_USE_I2C                     TRUE
#endif

/**
 * @brief   LSM6DSL shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LSM6DSL_SHARED_I2C) || defined(__DOXYGEN__)
#define LSM6DSL_SHARED_I2C                  FALSE
#endif

/**
 * @brief   LSM6DSL advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LSM6DSL_USE_ADVANCED) || defined(__DOXYGEN__)
#define LSM6DSL_USE_ADVANCED                FALSE
#endif

/**
 * @brief   Number of acquisitions for gyroscope bias removal.
 * @details This is the number of acquisitions performed to compute the
 *          bias. A repetition is required in order to remove noise.
 */
#if !defined(LSM6DSL_GYRO_BIAS_ACQ_TIMES) || defined(__DOXYGEN__)
#define LSM6DSL_GYRO_BIAS_ACQ_TIMES         50
#endif

/**
 * @brief   Settling time for gyroscope bias removal.
 * @details This is the time between each bias acquisition.
 */
#if !defined(LSM6DSL_GYRO_BIAS_SETTLING_US) || defined(__DOXYGEN__)
#define LSM6DSL_GYRO_BIAS_SETTLING_US       5000
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(LSM6DSL_USE_SPI ^ LSM6DSL_USE_I2C)
#error "LSM6DSL_USE_SPI and LSM6DSL_USE_I2C cannot be both true or both false"
#endif

#if LSM6DSL_USE_SPI && !HAL_USE_SPI
#error "LSM6DSL_USE_SPI requires HAL_USE_SPI"
#endif

#if LSM6DSL_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "LSM6DSL_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if LSM6DSL_USE_I2C && !HAL_USE_I2C
#error "LSM6DSL_USE_I2C requires HAL_USE_I2C"
#endif

#if LSM6DSL_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "LSM6DSL_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for LSM6DSL over SPI.
 */
#if LSM6DSL_USE_SPI
#error "LSM6DSL over SPI still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    LSM6DSL data structures and types.
 * @{
 */
/**
 * @brief   Structure representing a LSM6DSL driver.
 */
typedef struct LSM6DSLDriver LSM6DSLDriver;

/**
 * @brief  Accelerometer and Gyroscope Slave Address.
 */
typedef enum {
  LSM6DSL_SAD_GND = 0x6A,           /**< SAD pin connected to GND.          */
  LSM6DSL_SAD_VCC = 0x6B            /**< SAD pin connected to VCC.          */
} lsm6dsl_sad_t;

/**
 * @brief   LSM6DSL accelerometer subsystem full scale.
 */
typedef enum {
  LSM6DSL_ACC_FS_2G = 0x00,         /**< Full scale �2g.                    */
  LSM6DSL_ACC_FS_4G = 0x40,         /**< Full scale �4g.                    */
  LSM6DSL_ACC_FS_8G = 0x80,         /**< Full scale �8g.                    */
  LSM6DSL_ACC_FS_16G = 0xC0         /**< Full scale �16g.                   */
} lsm6dsl_acc_fs_t;

/**
 * @brief   LSM6DSL accelerometer subsystem output data rate.
 */
typedef enum {
  LSM6DSL_ACC_ODR_PD = 0x00,        /**< Power down                         */
  LSM6DSL_ACC_ODR_1P6HZ = 0xB0,     /**< ODR 1.6 Hz (Low Power only)        */
  LSM6DSL_ACC_ODR_12P5HZ = 0x10,    /**< ODR 12.5 Hz                        */
  LSM6DSL_ACC_ODR_26HZ = 0x20,      /**< ODR 26 Hz                          */
  LSM6DSL_ACC_ODR_52HZ = 0x30,      /**< ODR 52 Hz                          */
  LSM6DSL_ACC_ODR_104HZ = 0x40,     /**< ODR 104 Hz                         */
  LSM6DSL_ACC_ODR_208HZ = 0x50,     /**< ODR 208 Hz                         */
  LSM6DSL_ACC_ODR_416HZ = 0x60,     /**< ODR 416 Hz                         */
  LSM6DSL_ACC_ODR_833HZ = 0x70,     /**< ODR 833 Hz                         */
  LSM6DSL_ACC_ODR_1P66HZ = 0x80,    /**< ODR 1.66 kHz                       */
  LSM6DSL_ACC_ODR_3P33HZ = 0x90,    /**< ODR 3.33 kHz                       */
  LSM6DSL_ACC_ODR_6P66HZ = 0xA0     /**< ODR 6.66 kHz                       */
} lsm6dsl_acc_odr_t;

/**
 * @brief   LSM6DSL accelerometer subsystem output data rate.
 */
typedef enum {
  LSM6DSL_ACC_LP_DISABLED = 0x00,   /**< Low power disabled                 */
  LSM6DSL_ACC_LP_ENABLED = 0x10     /**< Low power enabled                  */
} lsm6dsl_acc_lp_t;

/**
 * @brief LSM6DSL gyroscope subsystem full scale.
 */
typedef enum {
  LSM6DSL_GYRO_FS_125DPS  = 0x02,   /**< Full scale �125 degree per second  */
  LSM6DSL_GYRO_FS_250DPS  = 0x00,   /**< Full scale �250 degree per second  */
  LSM6DSL_GYRO_FS_500DPS  = 0x04,   /**< Full scale �500 degree per second  */
  LSM6DSL_GYRO_FS_1000DPS = 0x08,   /**< Full scale �1000 degree per second */
  LSM6DSL_GYRO_FS_2000DPS = 0x0C    /**< Full scale �2000 degree per second */
} lsm6dsl_gyro_fs_t;

/**
 * @brief   LSM6DSL gyroscope subsystem output data rate.
 */
typedef enum {
  LSM6DSL_GYRO_ODR_PD = 0x00,       /**< Power down                         */
  LSM6DSL_GYRO_ODR_12P5HZ = 0x10,   /**< ODR 12.5 HZ                        */
  LSM6DSL_GYRO_ODR_26HZ = 0x20,     /**< ODR 26 Hz                          */
  LSM6DSL_GYRO_ODR_52HZ = 0x30,     /**< ODR 52 Hz                          */
  LSM6DSL_GYRO_ODR_104HZ = 0x40,    /**< ODR 104 Hz                         */
  LSM6DSL_GYRO_ODR_208HZ = 0x50,    /**< ODR 208 Hz                         */
  LSM6DSL_GYRO_ODR_416HZ = 0x60,    /**< ODR 416 Hz                         */
  LSM6DSL_GYRO_ODR_833HZ = 0x70,    /**< ODR 833 Hz                         */
  LSM6DSL_GYRO_ODR_1P66KHZ = 0x80,  /**< ODR 1.66 kHz                       */
  LSM6DSL_GYRO_ODR_3P33KHZ = 0x90,  /**< ODR 3.33 kHz                       */
  LSM6DSL_GYRO_ODR_6P66KHZ = 0xA0   /**< ODR 6.66 kHz                       */
} lsm6dsl_gyro_odr_t;

/**
 * @brief LSM6DSL gyroscope subsystem low mode configuration.
 */
typedef enum {
  LSM6DSL_GYRO_LP_DISABLED = 0x00,  /**< Low power mode disabled.           */
  LSM6DSL_GYRO_LP_ENABLED = 0x80    /**< Low power mode enabled.            */
} lsm6dsl_gyro_lp_t;

/**
 * @brief  LSM6DSL gyroscope subsystem output selection.
 */
typedef enum {
  LSM6DSL_GYRO_LPF_DISABLED = -1,   /**< Low pass filter disabled.          */
  LSM6DSL_GYRO_LPF_FTYPE0 = 0x00,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSL_GYRO_LPF_FTYPE1 = 0x01,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSL_GYRO_LPF_FTYPE2 = 0x10,   /**< Refer to table 68 of Datasheet.    */
  LSM6DSL_GYRO_LPF_FTYPE3 = 0x11    /**< Refer to table 68 of Datasheet.    */
} lsm6dsl_gyro_lpf_t;

/**
 * @brief LSM6DSL block data update.
 */
typedef enum {
  LSM6DSL_BDU_CONTINUOUS = 0x00,    /**< Block data continuously updated.   */
  LSM6DSL_BDU_BLOCKED = 0x40        /**< Block data updated after reading.  */
} lsm6dsl_bdu_t;

/**
 * @brief LSM6DSL endianness.
 */
typedef enum {
  LSM6DSL_END_LITTLE = 0x00,        /**< Little endian.                     */
  LSM6DSL_END_BIG = 0x20            /**< Big endian.                        */
} lsm6dsl_end_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LSM6DSL_UNINIT = 0,               /**< Not initialized.                   */
  LSM6DSL_STOP = 1,                 /**< Stopped.                           */
  LSM6DSL_READY = 2,                /**< Ready.                             */
} lsm6dsl_state_t;

/**
 * @brief LSM6DSL configuration structure.
 */
typedef struct {
#if (LSM6DSL_USE_SPI) || defined(__DOXYGEN__)
  /**
   * @brief SPI driver associated to this LSM6DSL.
   */
  SPIDriver                 *spip;
  /**
   * @brief SPI configuration associated to this LSM6DSL accelerometer
   *        subsystem.
   */
  const SPIConfig           *accspicfg;
#endif /* LSM6DSL_USE_SPI */
#if (LSM6DSL_USE_I2C) || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this LSM6DSL.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this LSM6DSL accelerometer
   *        subsystem.
   */
  const I2CConfig           *i2ccfg;
  /**
   * @brief LSM6DSL Slave Address
   */
  lsm6dsl_sad_t             slaveaddress;
#endif /* LSM6DSL_USE_I2C */
  /**
   * @brief LSM6DSL accelerometer subsystem initial sensitivity.
   */
  float                     *accsensitivity;
  /**
   * @brief LSM6DSL accelerometer subsystem initial bias.
   */
  float                     *accbias;
  /**
   * @brief LSM6DSL accelerometer subsystem full scale.
   */
  lsm6dsl_acc_fs_t          accfullscale;
  /**
   * @brief LSM6DSL accelerometer subsystem output data rate.
   */
  lsm6dsl_acc_odr_t         accodr;
#if LSM6DSL_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief LSM6DSL accelerometer subsystem low power mode.
   */
  lsm6dsl_acc_lp_t          acclpmode;
#endif /* LSM6DSL_USE_ADVANCED */
  /**
   * @brief LSM6DSL gyroscope subsystem initial sensitivity.
   */
  float                     *gyrosensitivity;
  /**
   * @brief LSM6DSL gyroscope subsystem initial bias.
   */
  float                     *gyrobias;
  /**
   * @brief LSM6DSL gyroscope subsystem full scale.
   */
  lsm6dsl_gyro_fs_t         gyrofullscale;
  /**
   * @brief LSM6DSL gyroscope subsystem output data rate.
   */
  lsm6dsl_gyro_odr_t        gyroodr;
#if LSM6DSL_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief LSM6DSL gyroscope subsystem low mode configuration.
   */
  lsm6dsl_gyro_lp_t         gyrolpmode;
  /**
   * @brief LSM6DSL gyroscope subsystem low pass filter configuration.
   */
  lsm6dsl_gyro_lpf_t        gyrolpfilter;
  /**
   * @brief LSM6DSL block data update
   */
  lsm6dsl_bdu_t             bdu;
  /**
   * @brief LSM6DSL  endianness
   */
  lsm6dsl_end_t             endianness;
#endif /* LSM6DSL_USE_ADVANCED */
} LSM6DSLConfig;

/**
 * @brief   @p LSM6DSL specific methods.
 */
#define _lsm6dsl_methods_alone                                              \
  /* Change full scale value of LSM6DSL accelerometer subsystem .*/         \
  msg_t (*acc_set_full_scale)(LSM6DSLDriver *devp, lsm6dsl_acc_fs_t fs);    \
  /* Change full scale value of LSM6DSL gyroscope subsystem .*/             \
  msg_t (*gyro_set_full_scale)(LSM6DSLDriver *devp, lsm6dsl_gyro_fs_t fs);

/**
 * @brief   @p LSM6DSL specific methods with inherited ones.
 */
#define _lsm6dsl_methods                                                    \
  _base_object_methods                                                      \
  _lsm6dsl_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief @p LSM6DSL virtual methods table.
 */
struct LSM6DSLVMT {
  _lsm6dsl_methods
};

/**
 * @brief   @p LSM6DSLDriver specific data.
 */
#define _lsm6dsl_data                                                       \
  _base_sensor_data                                                         \
  /* Driver state.*/                                                        \
  lsm6dsl_state_t           state;                                          \
  /* Current configuration data.*/                                          \
  const LSM6DSLConfig       *config;                                        \
  /* Accelerometer subsystem axes number.*/                                 \
  size_t                    accaxes;                                        \
  /* Accelerometer subsystem current sensitivity.*/                         \
  float                     accsensitivity[LSM6DSL_ACC_NUMBER_OF_AXES];     \
  /* Accelerometer subsystem current bias .*/                               \
  float                     accbias[LSM6DSL_ACC_NUMBER_OF_AXES];            \
  /* Accelerometer subsystem current full scale value.*/                    \
  float                     accfullscale;                                   \
  /* Gyroscope subsystem axes number.*/                                     \
  size_t                    gyroaxes;                                       \
  /* Gyroscope subsystem current sensitivity.*/                             \
  float                     gyrosensitivity[LSM6DSL_GYRO_NUMBER_OF_AXES];   \
  /* Gyroscope subsystem current Bias.*/                                    \
  float                     gyrobias[LSM6DSL_GYRO_NUMBER_OF_AXES];          \
  /* Gyroscope subsystem current full scale value.*/                        \
  float                     gyrofullscale;

/**
 * @brief LSM6DSL 6-axis accelerometer/gyroscope class.
 */
struct LSM6DSLDriver {
  /** @brief Virtual Methods Table.*/
  const struct LSM6DSLVMT     *vmt;
  /** @brief Base accelerometer interface.*/
  BaseAccelerometer           acc_if;
  /** @brief Base gyroscope interface.*/
  BaseGyroscope               gyro_if;
  _lsm6dsl_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lsm6dslAccelerometerGetAxesNumber(devp)                             \
        accelerometerGetAxesNumber(&((devp)->acc_if))

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
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
#define lsm6dslAccelerometerReadRaw(devp, axes)                             \
        accelerometerReadRaw(&((devp)->acc_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
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
#define lsm6dslAccelerometerReadCooked(devp, axes)                          \
        accelerometerReadCooked(&((devp)->acc_if), axes)

/**
 * @brief   Set bias values for the BaseAccelerometer.
 * @note    Bias must be expressed as milli-G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dslAccelerometerSetBias(devp, bp)                               \
        accelerometerSetBias(&((devp)->acc_if), bp)

/**
 * @brief   Reset bias values for the BaseAccelerometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dslAccelerometerResetBias(devp)                                 \
        accelerometerResetBias(&((devp)->acc_if))

/**
 * @brief   Set sensitivity values for the BaseAccelerometer.
 * @note    Sensitivity must be expressed as milli-G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dslAccelerometerSetSensitivity(devp, sp)                        \
        accelerometerSetSensitivity(&((devp)->acc_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseAccelerometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm6dslAccelerometerResetSensitivity(devp)                          \
        accelerometerResetSensitivity(&((devp)->acc_if))

/**
 * @brief   Changes the LSM6DSLDriver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm6dslAccelerometerSetFullScale(devp, fs)                          \
        (devp)->vmt->acc_set_full_scale(devp, fs)

/**
 * @brief   Return the number of axes of the BaseGyroscope.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lsm6dslGyroscopeGetAxesNumber(devp)                                 \
        gyroscopeGetAxesNumber(&((devp)->gyro_if))

/**
 * @brief   Retrieves raw data from the BaseGyroscope.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
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
#define lsm6dslGyroscopeReadRaw(devp, axes)                                 \
        gyroscopeReadRaw(&((devp)->gyro_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseGyroscope.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as DPS.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
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
#define lsm6dslGyroscopeReadCooked(devp, axes)                              \
        gyroscopeReadCooked(&((devp)->gyro_if), axes)

/**
 * @brief   Samples bias values for the BaseGyroscope.
 * @note    The LSM6DSL shall not be moved during the whole procedure.
 * @note    After this function internal bias is automatically updated.
 * @note    The behavior of this function depends on @p LSM6DSL_BIAS_ACQ_TIMES
 *          and @p LSM6DSL_BIAS_SETTLING_US.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define lsm6dslGyroscopeSampleBias(devp)                                    \
        gyroscopeSampleBias(&((devp)->gyro_if))

/**
 * @brief   Set bias values for the BaseGyroscope.
 * @note    Bias must be expressed as DPS.
 * @note    The bias buffer must be at least the same size of the BaseGyroscope
 *          axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dslGyroscopeSetBias(devp, bp)                                   \
        gyroscopeSetBias(&((devp)->gyro_if), bp)

/**
 * @brief   Reset bias values for the BaseGyroscope.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dslGyroscopeResetBias(devp)                                     \
        gyroscopeResetBias(&((devp)->gyro_if))

/**
 * @brief   Set sensitivity values for the BaseGyroscope.
 * @note    Sensitivity must be expressed as DPS/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lsm6dslGyroscopeSetSensitivity(devp, sp)                            \
        gyroscopeSetSensitivity(&((devp)->gyro_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseGyroscope.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm6dslGyroscopeResetSensitivity(devp)                              \
        gyroscopeResetSensitivity(&((devp)->gyro_if))

/**
 * @brief   Changes the LSM6DSLDriver gyroscope fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LSM6DSLDriver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lsm6dslGyroscopeSetFullScale(devp, fs)                              \
        (devp)->vmt->acc_set_full_scale(devp, fs)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void lsm6dslObjectInit(LSM6DSLDriver *devp);
  void lsm6dslStart(LSM6DSLDriver *devp, const LSM6DSLConfig *config);
  void lsm6dslStop(LSM6DSLDriver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _LSM6DSL_H_ */

/** @} */
