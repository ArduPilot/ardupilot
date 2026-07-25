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
 * @file    lis3dsh.h
 * @brief   LIS3DSH MEMS interface module header.
 *
 * @addtogroup LIS3DSH
 * @ingroup EX_ST
 * @{
 */

#ifndef _LIS3DSH_H_
#define _LIS3DSH_H_

#include "ex_accelerometer.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   LIS3DSH driver version string.
 */
#define EX_LIS3DSH_VERSION                  "1.1.3"

/**
 * @brief   LIS3DSH driver version major number.
 */
#define EX_LIS3DSH_MAJOR                    1

/**
 * @brief   LIS3DSH driver version minor number.
 */
#define EX_LIS3DSH_MINOR                    1

/**
 * @brief   LIS3DSH driver version patch number.
 */
#define EX_LIS3DSH_PATCH                    3
/** @} */

/**
 * @brief   LIS3DSH accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define LIS3DSH_ACC_NUMBER_OF_AXES          3U

#define LIS3DSH_ACC_2G                      2.0f
#define LIS3DSH_ACC_4G                      4.0f
#define LIS3DSH_ACC_6G                      6.0f
#define LIS3DSH_ACC_8G                      8.0f
#define LIS3DSH_ACC_16G                     16.0f

#define LIS3DSH_ACC_SENS_2G                 0.06f
#define LIS3DSH_ACC_SENS_4G                 0.12f
#define LIS3DSH_ACC_SENS_6G                 0.18f
#define LIS3DSH_ACC_SENS_8G                 0.24f
#define LIS3DSH_ACC_SENS_16G                0.73f

#define LIS3DSH_ACC_BIAS                    0.0f
/** @} */

/**
 * @name    LIS3DSH communication interfaces related bit masks
 * @{
 */
#define LIS3DSH_DI_MASK                     0xFF
#define LIS3DSH_DI(n)                       (1 << n)
#define LIS3DSH_AD_MASK                     0x3F
#define LIS3DSH_AD(n)                       (1 << n)
#define LIS3DSH_MS                          (1 << 6)
#define LIS3DSH_RW                          (1 << 7)
/** @} */

/**
 * @name    LIS3DSH register addresses
 * @{
 */
#define  LIS3DSH_AD_OUT_T                   0x0C
#define  LIS3DSH_AD_INFO1                   0x0D
#define  LIS3DSH_AD_INFO2                   0x0E
#define  LIS3DSH_AD_WHO_AM_I                0x0F
#define  LIS3DSH_AD_OFF_X                   0x10
#define  LIS3DSH_AD_OFF_Y                   0x11
#define  LIS3DSH_AD_OFF_Z                   0x12
#define  LIS3DSH_AD_CS_X                    0x13
#define  LIS3DSH_AD_CS_Y                    0x14
#define  LIS3DSH_AD_CS_Z                    0x15
#define  LIS3DSH_AD_LC_L                    0x16
#define  LIS3DSH_AD_LC_H                    0x17
#define  LIS3DSH_AD_STAT                    0x18
#define  LIS3DSH_AD_PEAK1                   0x19
#define  LIS3DSH_AD_PEAK2                   0x1A
#define  LIS3DSH_AD_VFC_1                   0x1B
#define  LIS3DSH_AD_VFC_2                   0x1C
#define  LIS3DSH_AD_VFC_3                   0x1D
#define  LIS3DSH_AD_VFC_4                   0x1E
#define  LIS3DSH_AD_THRS3                   0x1F
#define  LIS3DSH_AD_CTRL_REG4               0x20
#define  LIS3DSH_AD_CTRL_REG1               0x21
#define  LIS3DSH_AD_CTRL_REG2               0x22
#define  LIS3DSH_AD_CTRL_REG3               0x23
#define  LIS3DSH_AD_CTRL_REG5               0x24
#define  LIS3DSH_AD_CTRL_REG6               0x25
#define  LIS3DSH_AD_STATUS                  0x27
#define  LIS3DSH_AD_OUT_X_L                 0x28
#define  LIS3DSH_AD_OUT_X_H                 0x29
#define  LIS3DSH_AD_OUT_Y_L                 0x2A
#define  LIS3DSH_AD_OUT_Y_H                 0x2B
#define  LIS3DSH_AD_OUT_Z_L                 0x2C
#define  LIS3DSH_AD_OUT_Z_H                 0x2D
#define  LIS3DSH_AD_FIFO_CTRL               0x2E
#define  LIS3DSH_AD_FIFO_SRC                0x2F
#define  LIS3DSH_AD_ST1_0                   0x40
#define  LIS3DSH_AD_ST1_1                   0x41
#define  LIS3DSH_AD_ST1_2                   0x42
#define  LIS3DSH_AD_ST1_3                   0x43
#define  LIS3DSH_AD_ST1_4                   0x44
#define  LIS3DSH_AD_ST1_5                   0x45
#define  LIS3DSH_AD_ST1_6                   0x46
#define  LIS3DSH_AD_ST1_7                   0x47
#define  LIS3DSH_AD_ST1_8                   0x48
#define  LIS3DSH_AD_ST1_9                   0x49
#define  LIS3DSH_AD_ST1_A                   0x4A
#define  LIS3DSH_AD_ST1_B                   0x4B
#define  LIS3DSH_AD_ST1_C                   0x4C
#define  LIS3DSH_AD_ST1_D                   0x4D
#define  LIS3DSH_AD_ST1_E                   0x4E
#define  LIS3DSH_AD_ST1_F                   0x4F
#define  LIS3DSH_AD_TIM4_1                  0x50
#define  LIS3DSH_AD_TIM3_1                  0x51
#define  LIS3DSH_AD_TIM2_1_L                0x52
#define  LIS3DSH_AD_TIM2_1_H                0x53
#define  LIS3DSH_AD_TIM1_1_L                0x54
#define  LIS3DSH_AD_TIM1_1_H                0x55
#define  LIS3DSH_AD_THRS2_1                 0x56
#define  LIS3DSH_AD_THRS1_1                 0x57
#define  LIS3DSH_AD_MASK1_B                 0x59
#define  LIS3DSH_AD_MASK1_A                 0x5A
#define  LIS3DSH_AD_SETT1                   0x5B
#define  LIS3DSH_AD_PR1                     0x5C
#define  LIS3DSH_AD_TC1_L                   0x5D
#define  LIS3DSH_AD_TC1_H                   0x5E
#define  LIS3DSH_AD_OUTS1                   0x5F
#define  LIS3DSH_AD_ST2_0                   0x60
#define  LIS3DSH_AD_ST2_1                   0x61
#define  LIS3DSH_AD_ST2_2                   0x62
#define  LIS3DSH_AD_ST2_3                   0x63
#define  LIS3DSH_AD_ST2_4                   0x64
#define  LIS3DSH_AD_ST2_5                   0x65
#define  LIS3DSH_AD_ST2_6                   0x66
#define  LIS3DSH_AD_ST2_7                   0x67
#define  LIS3DSH_AD_ST2_8                   0x68
#define  LIS3DSH_AD_ST2_9                   0x69
#define  LIS3DSH_AD_ST2_A                   0x6A
#define  LIS3DSH_AD_ST2_B                   0x6B
#define  LIS3DSH_AD_ST2_C                   0x6C
#define  LIS3DSH_AD_ST2_D                   0x6D
#define  LIS3DSH_AD_ST2_E                   0x6E
#define  LIS3DSH_AD_ST2_F                   0x6F
#define  LIS3DSH_AD_TIM4_2                  0x70
#define  LIS3DSH_AD_TIM3_2                  0x71
#define  LIS3DSH_AD_TIM2_2_L                0x72
#define  LIS3DSH_AD_TIM2_2_H                0x73
#define  LIS3DSH_AD_TIM1_2_L                0x74
#define  LIS3DSH_AD_TIM1_2_H                0x75
#define  LIS3DSH_AD_THRS2_2                 0x76
#define  LIS3DSH_AD_THRS1_2                 0x77
#define  LIS3DSH_AD_DES2                    0x78
#define  LIS3DSH_AD_MASK2_B                 0x79
#define  LIS3DSH_AD_MASK2_A                 0x7A
#define  LIS3DSH_AD_SETT2                   0x7B
#define  LIS3DSH_AD_PR2                     0x7C
#define  LIS3DSH_AD_TC2_L                   0x7D
#define  LIS3DSH_AD_TC2_H                   0x7E
#define  LIS3DSH_AD_OUTS2                   0x7F
/** @} */

/**
 * @name    LIS3DSH_CTRL_REG1 register bits definitions
 * @{
 */
#define LIS3DSH_CTRL_REG1_MASK              0xE9
#define LIS3DSH_CTRL_REG1_SM1_EN            (1 << 0)
#define LIS3DSH_CTRL_REG1_SM1_PIN           (1 << 3)
#define LIS3DSH_CTRL_REG1_HYST0_1           (1 << 5)
#define LIS3DSH_CTRL_REG1_HYST1_1           (1 << 6)
#define LIS3DSH_CTRL_REG1_HYST2_1           (1 << 7)
/** @} */

/**
 * @name    LIS3DSH_CTRL_REG2 register bits definitions
 * @{
 */
#define LIS3DSH_CTRL_REG2_MASK              0xE9
#define LIS3DSH_CTRL_REG2_SM2_EN            (1 << 0)
#define LIS3DSH_CTRL_REG2_SM2_PIN           (1 << 3)
#define LIS3DSH_CTRL_REG2_HYST0_2           (1 << 5)
#define LIS3DSH_CTRL_REG2_HYST1_2           (1 << 6)
#define LIS3DSH_CTRL_REG2_HYST2_2           (1 << 7)
/** @} */

/**
 * @name    LIS3DSH_CTRL_REG3 register bits definitions
 * @{
 */
#define LIS3DSH_CTRL_REG3_MASK              0xFF
#define LIS3DSH_CTRL_REG3_STRT              (1 << 0)
#define LIS3DSH_CTRL_REG3_VFILT             (1 << 2)
#define LIS3DSH_CTRL_REG3_INT1_EN           (1 << 3)
#define LIS3DSH_CTRL_REG3_INT2_EN           (1 << 4)
#define LIS3DSH_CTRL_REG3_IEL               (1 << 5)
#define LIS3DSH_CTRL_REG3_IEA               (1 << 6)
#define LIS3DSH_CTRL_REG3_DR_EN             (1 << 7)
/** @} */

/**
 * @name    LIS3DSH_CTRL_REG4 register bits definitions
 * @{
 */
#define LIS3DSH_CTRL_REG4_MASK              0xFF
#define LIS3DSH_CTRL_REG4_XEN               (1 << 0)
#define LIS3DSH_CTRL_REG4_YEN               (1 << 1)
#define LIS3DSH_CTRL_REG4_ZEN               (1 << 2)
#define LIS3DSH_CTRL_REG4_BDU               (1 << 3)
#define LIS3DSH_CTRL_REG4_ODR_0             (1 << 4)
#define LIS3DSH_CTRL_REG4_ODR_1             (1 << 5)
#define LIS3DSH_CTRL_REG4_ODR_2             (1 << 6)
#define LIS3DSH_CTRL_REG4_ODR_3             (1 << 7)
/** @} */

/**
 * @name    LIS3DSH_CTRL_REG5 register bits definitions
 * @{
 */
#define LIS3DSH_CTRL_REG5_MASK              0xFF
#define LIS3DSH_CTRL_REG5_SIM               (1 << 0)
#define LIS3DSH_CTRL_REG5_ST1               (1 << 1)
#define LIS3DSH_CTRL_REG5_ST2               (1 << 2)
#define LIS3DSH_CTRL_REG5_FS_MASK           0x38
#define LIS3DSH_CTRL_REG5_FS0               (1 << 3)
#define LIS3DSH_CTRL_REG5_FS1               (1 << 4)
#define LIS3DSH_CTRL_REG5_FS2               (1 << 5)
#define LIS3DSH_CTRL_REG5_BW1               (1 << 6)
#define LIS3DSH_CTRL_REG5_BW2               (1 << 7)
/** @} */

/**
 * @name    LIS3DSH_CTRL_REG6 register bits definitions
 * @{
 */
#define LIS3DSH_CTRL_REG6_MASK              0xFF
#define LIS3DSH_CTRL_REG6_P2_BOOT           (1 << 0)
#define LIS3DSH_CTRL_REG6_P1_OVRUN          (1 << 1)
#define LIS3DSH_CTRL_REG6_P1_WTM            (1 << 2)
#define LIS3DSH_CTRL_REG6_P1_EMPTY          (1 << 3)
#define LIS3DSH_CTRL_REG6_ADD_INC           (1 << 4)
#define LIS3DSH_CTRL_REG6_WTM_EN            (1 << 5)
#define LIS3DSH_CTRL_REG6_FIFO_EN           (1 << 6)
#define LIS3DSH_CTRL_REG6_BOOT              (1 << 7)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LIS3DSH SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LIS3DSH_USE_SPI) || defined(__DOXYGEN__)
#define LIS3DSH_USE_SPI                     TRUE
#endif

/**
 * @brief   LIS3DSH shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LIS3DSH_SHARED_SPI) || defined(__DOXYGEN__)
#define LIS3DSH_SHARED_SPI                  FALSE
#endif

/**
 * @brief   LIS3DSH I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LIS3DSH_USE_I2C) || defined(__DOXYGEN__)
#define LIS3DSH_USE_I2C                     FALSE
#endif

/**
 * @brief   LIS3DSH shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LIS3DSH_SHARED_I2C) || defined(__DOXYGEN__)
#define LIS3DSH_SHARED_I2C                  FALSE
#endif

/**
 * @brief   LIS3DSH advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(LIS3DSH_USE_ADVANCED) || defined(__DOXYGEN__)
#define LIS3DSH_USE_ADVANCED                FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(LIS3DSH_USE_SPI ^ LIS3DSH_USE_I2C)
#error "LIS3DSH_USE_SPI and LIS3DSH_USE_I2C cannot be both true or both false"
#endif

#if LIS3DSH_USE_SPI && !HAL_USE_SPI
#error "LIS3DSH_USE_SPI requires HAL_USE_SPI"
#endif

#if LIS3DSH_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "LIS3DSH_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if LIS3DSH_USE_I2C && !HAL_USE_I2C
#error "LIS3DSH_USE_I2C requires HAL_USE_I2C"
#endif

#if LIS3DSH_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "LIS3DSH_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for LIS3DSH over I2C.
 */
#if LIS3DSH_USE_I2C
#error "LIS3DSH over I2C still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    LIS3DSH data structures and types
 * @{
 */
/**
 * @brief   Structure representing a LIS3DSH driver.
 */
typedef struct LIS3DSHDriver LIS3DSHDriver;

/**
 * @brief   LIS3DSH full scale.
 */
typedef enum {
  LIS3DSH_ACC_FS_2G = 0x00,         /**< Full scale �2g.                    */
  LIS3DSH_ACC_FS_4G = 0x08,         /**< Full scale �4g.                    */
  LIS3DSH_ACC_FS_6G = 0x10,         /**< Full scale �6g.                    */
  LIS3DSH_ACC_FS_8G = 0x18,         /**< Full scale �8g.                    */
  LIS3DSH_ACC_FS_16G = 0x20         /**< Full scale �16g.                   */
}lis3dsh_acc_fs_t;

/**
 * @brief   LIS3DSH output data rate.
 */
typedef enum {
  LIS3DSH_ACC_ODR_PD = 0x00,        /**< ODR 100 Hz.                        */
  LIS3DSH_ACC_ODR_3_125HZ = 0x10,   /**< ODR 3.125 Hz.                      */
  LIS3DSH_ACC_ODR_6_25HZ = 0x20,    /**< ODR 6.25 Hz.                       */
  LIS3DSH_ACC_ODR_12_5HZ = 0x30,    /**< ODR 12.5 Hz.                       */
  LIS3DSH_ACC_ODR_25HZ = 0x40,      /**< ODR 25 Hz.                         */
  LIS3DSH_ACC_ODR_50HZ = 0x50,      /**< ODR 50 Hz.                         */
  LIS3DSH_ACC_ODR_100HZ = 0x60,     /**< ODR 100 Hz.                        */
  LIS3DSH_ACC_ODR_400HZ = 0x70,     /**< ODR 400 Hz.                        */
  LIS3DSH_ACC_ODR_800HZ = 0x80,     /**< ODR 800 Hz.                        */
  LIS3DSH_ACC_ODR_1600HZ = 0x90     /**< ODR 1600 Hz.                       */
}lis3dsh_acc_odr_t;

/**
 * @brief   LIS3DSH anti-aliasing bandwidth.
 */
typedef enum {
  LIS3DSH_ACC_BW_800HZ = 0x00,      /**< AA filter BW 800Hz.                */
  LIS3DSH_ACC_BW_200HZ = 0x40,      /**< AA filter BW 200Hz.                */
  LIS3DSH_ACC_BW_400HZ = 0x80,      /**< AA filter BW 400Hz.                */
  LIS3DSH_ACC_BW_50HZ = 0xC0        /**< AA filter BW 50Hz.                 */
}lis3dsh_acc_bw_t;

/**
 * @brief   LIS3DSH block data update.
 */
typedef enum {
  LIS3DSH_ACC_BDU_CONTINUOUS = 0x00,/**< Block data continuously updated.   */
  LIS3DSH_ACC_BDU_BLOCKED = 0x80    /**< Block data updated after reading.  */
} lis3dsh_acc_bdu_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LIS3DSH_UNINIT = 0,               /**< Not initialized.                   */
  LIS3DSH_STOP = 1,                 /**< Stopped.                           */
  LIS3DSH_READY = 2,                /**< Ready.                             */
} lis3dsh_state_t;

/**
 * @brief   LIS3DSH configuration structure.
 */
typedef struct {

#if (LIS3DSH_USE_SPI) || defined(__DOXYGEN__)
  /**
   * @brief SPI driver associated to this LIS3DSH.
   */
  SPIDriver                 *spip;
  /**
   * @brief SPI configuration associated to this LIS3DSH.
   */
  const SPIConfig           *spicfg;
#endif /* LIS3DSH_USE_SPI */
#if (LIS3DSH_USE_I2C) || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this LIS3DSH.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this LIS3DSH.
   */
  const I2CConfig           *i2ccfg;
#endif /* LIS3DSH_USE_I2C */
  /**
   * @brief LIS3DSH accelerometer subsystem initial sensitivity.
   */
  float                     *accsensitivity;
  /**
   * @brief LIS3DSH accelerometer subsystem initial bias.
   */
  float                     *accbias;
  /**
   * @brief LIS3DSH accelerometer subsystem initial full scale.
   */
  lis3dsh_acc_fs_t          accfullscale;
  /**
   * @brief LIS3DSH output data rate selection.
   */
  lis3dsh_acc_odr_t         accodr;
#if LIS3DSH_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief   LIS3DSH anti-aliasing bandwidth.
   */
  lis3dsh_acc_bw_t          accantialiasing;
  /**
   * @brief   LIS3DSH block data update.
   */
  lis3dsh_acc_bdu_t         accbdu;
#endif
} LIS3DSHConfig;

/**
 * @brief   @p LIS3DSH specific methods.
 */
#define _lis3dsh_methods_alone                                              \
  /* Change full scale value of LIS3DSH accelerometer subsystem.*/          \
  msg_t (*acc_set_full_scale)(LIS3DSHDriver *devp, lis3dsh_acc_fs_t fs);


/**
 * @brief   @p LIS3DSH specific methods with inherited ones.
 */
#define _lis3dsh_methods                                                    \
  _base_object_methods                                                      \
  _lis3dsh_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief   @p LIS3DSH virtual methods table.
 */
struct LIS3DSHVMT {
  _lis3dsh_methods
};

/**
 * @brief   @p LIS3DSHDriver specific data.
 */
#define _lis3dsh_data                                                       \
  _base_sensor_data                                                         \
  /* Driver state.*/                                                        \
  lis3dsh_state_t           state;                                          \
  /* Current configuration data.*/                                          \
  const LIS3DSHConfig       *config;                                        \
  /* Accelerometer subsystem axes number.*/                                 \
  size_t                    accaxes;                                        \
  /* Accelerometer subsystem current sensitivity.*/                         \
  float                     accsensitivity[LIS3DSH_ACC_NUMBER_OF_AXES];     \
  /* Accelerometer subsystem current bias .*/                               \
  float                     accbias[LIS3DSH_ACC_NUMBER_OF_AXES];            \
  /* Accelerometer subsystem current full scale value.*/                    \
  float                     accfullscale;

/**
 * @brief   LIS3DSH 3-axis accelerometer class.
 */
struct LIS3DSHDriver {
  /** @brief Virtual Methods Table.*/
  const struct LIS3DSHVMT     *vmt;
  /** @brief Base accelerometer interface.*/
  BaseAccelerometer           acc_if;
  _lis3dsh_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lis3dshAccelerometerGetAxesNumber(devp)                             \
        accelerometerGetAxesNumber(&((devp)->acc_if))

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver.
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
#define lis3dshAccelerometerReadRaw(devp, axes)                             \
        accelerometerReadRaw(&((devp)->acc_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver.
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
#define lis3dshAccelerometerReadCooked(devp, axes)                          \
        accelerometerReadCooked(&((devp)->acc_if), axes)

/**
 * @brief   Set bias values for the BaseAccelerometer.
 * @note    Bias must be expressed as milli-G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lis3dshAccelerometerSetBias(devp, bp)                               \
        accelerometerSetBias(&((devp)->acc_if), bp)

/**
 * @brief   Reset bias values for the BaseAccelerometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lis3dshAccelerometerResetBias(devp)                                 \
        accelerometerResetBias(&((devp)->acc_if))

/**
 * @brief   Set sensitivity values for the BaseAccelerometer.
 * @note    Sensitivity must be expressed as milli-G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lis3dshAccelerometerSetSensitivity(devp, sp)                        \
        accelerometerSetSensitivity(&((devp)->acc_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseAccelerometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lis3dshAccelerometerResetSensitivity(devp)                          \
        accelerometerResetSensitivity(&((devp)->acc_if))

/**
 * @brief   Changes the LIS3DSHDriver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LIS3DSHDriver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lis3dshAccelerometerSetFullScale(devp, fs)                          \
        (devp)->vmt->acc_set_full_scale(devp, fs)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void lis3dshObjectInit(LIS3DSHDriver *devp);
  void lis3dshStart(LIS3DSHDriver *devp, const LIS3DSHConfig *config);
  void lis3dshStop(LIS3DSHDriver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _LIS3DSH_H_ */

/** @} */

