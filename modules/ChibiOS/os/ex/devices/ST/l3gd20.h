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
 * @file    l3gd20.h
 * @brief   L3GD20 MEMS interface module header.
 *
 * @addtogroup L3GD20
 * @ingroup EX_ST
 * @{
 */
#ifndef _L3GD20_H_
#define _L3GD20_H_

#include "ex_gyroscope.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   L3GD20 driver version string.
 */
#define EX_L3GD20_VERSION                   "1.1.3"

/**
 * @brief   L3GD20 driver version major number.
 */
#define EX_L3GD20_MAJOR                     1

/**
 * @brief   L3GD20 driver version minor number.
 */
#define EX_L3GD20_MINOR                     1

/**
 * @brief   L3GD20 driver version patch number.
 */
#define EX_L3GD20_PATCH                     3
/** @} */

/**
 * @brief   L3GD20 gyroscope system characteristics.
 * @note    Sensitivity is expressed as DPS/LSB whereas DPS stand for Degree
 *          per second [°/s].
 * @note    Bias is expressed as DPS.
 *
 * @{
 */
#define L3GD20_GYRO_NUMBER_OF_AXES          3U
                                            
#define L3GD20_250DPS                       250.0f
#define L3GD20_500DPS                       500.0f
#define L3GD20_2000DPS                      2000.0f
                                            
#define L3GD20_GYRO_SENS_250DPS             0.00875f
#define L3GD20_GYRO_SENS_500DPS             0.01750f
#define L3GD20_GYRO_SENS_2000DPS            0.07000f

#define L3GD20_GYRO_BIAS                    0.0f
/** @} */

/**
 * @name    L3GD20 communication interfaces related bit masks
 * @{
 */
#define L3GD20_DI_MASK                      0xFF
#define L3GD20_DI(n)                        (1 << n)
#define L3GD20_AD_MASK                      0x3F
#define L3GD20_AD(n)                        (1 << n)
#define L3GD20_MS                           (1 << 6)
#define L3GD20_RW                           (1 << 7)
/** @} */

/**
 * @name    L3GD20 register addresses
 * @{
 */
#define L3GD20_AD_WHO_AM_I                  0x0F
#define L3GD20_AD_CTRL_REG1                 0x20
#define L3GD20_AD_CTRL_REG2                 0x21
#define L3GD20_AD_CTRL_REG3                 0x22
#define L3GD20_AD_CTRL_REG4                 0x23
#define L3GD20_AD_CTRL_REG5                 0x24
#define L3GD20_AD_REFERENCE                 0x25
#define L3GD20_AD_OUT_TEMP                  0x26
#define L3GD20_AD_STATUS_REG                0x27
#define L3GD20_AD_OUT_X_L                   0x28
#define L3GD20_AD_OUT_X_H                   0x29
#define L3GD20_AD_OUT_Y_L                   0x2A
#define L3GD20_AD_OUT_Y_H                   0x2B
#define L3GD20_AD_OUT_Z_L                   0x2C
#define L3GD20_AD_OUT_Z_H                   0x2D
#define L3GD20_AD_FIFO_CTRL_REG             0x2E
#define L3GD20_AD_FIFO_SRC_REG              0x2F
#define L3GD20_AD_INT1_CFG                  0x30
#define L3GD20_AD_INT1_SRC                  0x31
#define L3GD20_AD_INT1_THS_XH               0x32
#define L3GD20_AD_INT1_THS_XL               0x33
#define L3GD20_AD_INT1_THS_YH               0x34
#define L3GD20_AD_INT1_THS_YL               0x35
#define L3GD20_AD_INT1_THS_ZH               0x36
#define L3GD20_AD_INT1_THS_ZL               0x37
#define L3GD20_AD_INT1_DURATION             0x38
/** @} */

/**
 * @name    L3GD20_CTRL_REG1 register bits definitions
 * @{
 */
#define L3GD20_CTRL_REG1_MASK               0xFF
#define L3GD20_CTRL_REG1_XEN                (1 << 0)
#define L3GD20_CTRL_REG1_YEN                (1 << 1)
#define L3GD20_CTRL_REG1_ZEN                (1 << 2)
#define L3GD20_CTRL_REG1_PD                 (1 << 3)
#define L3GD20_CTRL_REG1_BW0                (1 << 4)
#define L3GD20_CTRL_REG1_BW1                (1 << 5)
#define L3GD20_CTRL_REG1_DR0                (1 << 6)
#define L3GD20_CTRL_REG1_DR1                (1 << 7)
/** @} */

/**
 * @name    L3GD20_CTRL_REG2 register bits definitions
 * @{
 */                                              
#define L3GD20_CTRL_REG2_MASK               0x3F
#define L3GD20_CTRL_REG2_HPCF0              (1 << 0)
#define L3GD20_CTRL_REG2_HPCF1              (1 << 1)
#define L3GD20_CTRL_REG2_HPCF2              (1 << 2)
#define L3GD20_CTRL_REG2_HPCF3              (1 << 3)
#define L3GD20_CTRL_REG2_HPM0               (1 << 4)
#define L3GD20_CTRL_REG2_HPM1               (1 << 5)
/** @} */

/**
 * @name    L3GD20_CTRL_REG3 register bits definitions
 * @{
 */                                               
#define L3GD20_CTRL_REG3_MASK               0xFF    
#define L3GD20_CTRL_REG3_I2_EMPTY           (1 << 0)
#define L3GD20_CTRL_REG3_I2_ORUN            (1 << 1)
#define L3GD20_CTRL_REG3_I2_WTM             (1 << 2)
#define L3GD20_CTRL_REG3_I2_DRDY            (1 << 3)
#define L3GD20_CTRL_REG3_PP_OD              (1 << 4)
#define L3GD20_CTRL_REG3_H_LACTIVE          (1 << 5)
#define L3GD20_CTRL_REG3_I1_BOOT            (1 << 6)
#define L3GD20_CTRL_REG3_I1_INT1            (1 << 7)
/** @} */

/**
 * @name    L3GD20_CTRL_REG4 register bits definitions
 * @{
 */
#define L3GD20_CTRL_REG4_MASK               0xF1    
#define L3GD20_CTRL_REG4_SIM                (1 << 0)
#define L3GD20_CTRL_REG4_FS_MASK            0x30    
#define L3GD20_CTRL_REG4_FS0                (1 << 4)
#define L3GD20_CTRL_REG4_FS1                (1 << 5)
#define L3GD20_CTRL_REG4_BLE                (1 << 6)
#define L3GD20_CTRL_REG4_BDU                (1 << 7)
/** @} */

/**
 * @name    L3GD20_CTRL_REG5 register bits definitions
 * @{
 */                                               
#define L3GD20_CTRL_REG5_MASK               0xDF    
#define L3GD20_CTRL_REG5_OUT_SEL0           (1 << 0)
#define L3GD20_CTRL_REG5_OUT_SEL1           (1 << 1)
#define L3GD20_CTRL_REG5_INT1_SEL0          (1 << 2)
#define L3GD20_CTRL_REG5_INT1_SEL1          (1 << 3)
#define L3GD20_CTRL_REG5_HPEN               (1 << 4)
#define L3GD20_CTRL_REG5_FIFO_EN            (1 << 6)
#define L3GD20_CTRL_REG5_BOOT               (1 << 7)
/** @} */

/**
 * @name    L3GD20_INT1_CFG register bits definitions
 * @{
 */
#define L3GD20_INT1_CFG_MASK                0xFF
#define L3GD20_INT1_CFG_XLIE                (1 << 0)
#define L3GD20_INT1_CFG_XHIE                (1 << 1)
#define L3GD20_INT1_CFG_YLIE                (1 << 2)
#define L3GD20_INT1_CFG_YHIE                (1 << 3)
#define L3GD20_INT1_CFG_ZLIE                (1 << 4)
#define L3GD20_INT1_CFG_ZHIE                (1 << 5)
#define L3GD20_INT1_CFG_LIR                 (1 << 6)
#define L3GD20_INT1_CFG_AND_OR              (1 << 7)
/** @} */

/**
 * @name    L3GD20_INT1_SRC register bits definitions
 * @{
 */
#define L3GD20_INT1_SRC_MASK                0x7F
#define L3GD20_INT1_SRC_XL                  (1 << 0)
#define L3GD20_INT1_SRC_XH                  (1 << 1)
#define L3GD20_INT1_SRC_YL                  (1 << 2)
#define L3GD20_INT1_SRC_YH                  (1 << 3)
#define L3GD20_INT1_SRC_ZL                  (1 << 4)
#define L3GD20_INT1_SRC_ZH                  (1 << 5)
#define L3GD20_INT1_SRC_IA                  (1 << 6)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   L3GD20 SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p TRUE.
 */
#if !defined(L3GD20_USE_SPI) || defined(__DOXYGEN__)
#define L3GD20_USE_SPI                      TRUE
#endif

/**
 * @brief   L3GD20 shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(L3GD20_SHARED_SPI) || defined(__DOXYGEN__)
#define L3GD20_SHARED_SPI                   FALSE
#endif

/**
 * @brief   L3GD20 I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p FALSE.
 */
#if !defined(L3GD20_USE_I2C) || defined(__DOXYGEN__)
#define L3GD20_USE_I2C                      FALSE
#endif

/**
 * @brief   L3GD20 shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(L3GD20_SHARED_I2C) || defined(__DOXYGEN__)
#define L3GD20_SHARED_I2C                   FALSE
#endif

/**
 * @brief   L3GD20 advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(L3GD20_USE_ADVANCED) || defined(__DOXYGEN__)
#define L3GD20_USE_ADVANCED                 FALSE
#endif

/**
 * @brief   Number of acquisitions for bias removal
 * @details This is the number of acquisitions performed to compute the
 *          bias. A repetition is required in order to remove noise.
 */
#if !defined(L3GD20_BIAS_ACQ_TIMES) || defined(__DOXYGEN__)
#define L3GD20_BIAS_ACQ_TIMES               50
#endif

/**
 * @brief   Settling time for bias removal
 * @details This is the time between each bias acquisition.
 */
#if !defined(L3GD20_BIAS_SETTLING_US) || defined(__DOXYGEN__)
#define L3GD20_BIAS_SETTLING_US             5000
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(L3GD20_USE_SPI ^ L3GD20_USE_I2C)
#error "L3GD20_USE_SPI and L3GD20_USE_I2C cannot be both true or both false"
#endif

#if L3GD20_USE_SPI && !HAL_USE_SPI
#error "L3GD20_USE_SPI requires HAL_USE_SPI"
#endif

#if L3GD20_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "L3GD20_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if L3GD20_USE_I2C && !HAL_USE_I2C
#error "L3GD20_USE_I2C requires HAL_USE_I2C"
#endif

#if L3GD20_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "L3GD20_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for L3GD20 over I2C.
 */
#if L3GD20_USE_I2C
#error "L3GD20 over I2C still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    L3GD20 data structures and types.
 * @{
 */
/**
 * @brief   Structure representing a L3GD20 driver.
 */
typedef struct L3GD20Driver L3GD20Driver;

/**
 * @brief   L3GD20 full scale.
 */
typedef enum {
  L3GD20_FS_250DPS = 0x00,          /**< Full scale 250 degree per second.  */
  L3GD20_FS_500DPS = 0x10,          /**< Full scale 500 degree per second.  */
  L3GD20_FS_2000DPS = 0x20          /**< Full scale 2000 degree per second. */
} l3gd20_fs_t;

/**
 * @brief   L3GD20 output data rate and bandwidth.
 */
typedef enum {
  L3GD20_ODR_95HZ = 0x00,           /**< Output data rate 95 Hz.            */
  L3GD20_ODR_190HZ = 0x40,          /**< Output data rate 190 Hz.           */
  L3GD20_ODR_380HZ = 0x80,          /**< Output data rate 380 Hz.           */
  L3GD20_ODR_760HZ = 0xC0           /**< Output data rate 760 Hz.           */
} l3gd20_odr_t;

/**
 * @brief   L3GD20 low pass filter 1 bandwidth.
 */
typedef enum {
  L3GD20_BW0 = 0x00,                /**< LPF1 bandwidth. Depends on ODR.    */
  L3GD20_BW1 = 0x40,                /**< LPF1 bandwidth. Depends on ODR.    */
  L3GD20_BW2 = 0x80,                /**< LPF1 bandwidth. Depends on ODR.    */
  L3GD20_BW3 = 0xC0                 /**< LPF1 bandwidth. Depends on ODR.    */
} l3gd20_bw_t;

/**
 * @brief   L3GD20 block data update.
 */
typedef enum {
  L3GD20_BDU_CONTINUOUS = 0x00,     /**< Block data continuously updated.   */
  L3GD20_BDU_BLOCKED = 0x80         /**< Block data updated after reading.  */
} l3gd20_bdu_t;

/**
 * @brief   L3GD20 HP filter mode.
 */
typedef enum {
  L3GD20_HPM_NORMAL = 0x00,         /**< Normal mode.                       */
  L3GD20_HPM_REFERENCE = 0x10,      /**< Reference signal for filtering.    */
  L3GD20_HPM_AUTORESET = 0x30,      /**< Autoreset on interrupt event.      */  
  L3GD20_HPM_BYPASSED = 0xFF        /**< HP filter bypassed                 */
} l3gd20_hpm_t;

/**
 * @brief   L3GD20 HP configuration.
 */
typedef enum {
  L3GD20_HPCF_0 = 0x00,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_1 = 0x01,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_2 = 0x02,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_3 = 0x03,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_4 = 0x04,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_5 = 0x05,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_6 = 0x06,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_7 = 0x07,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_8 = 0x08,             /**< Depends on ODR (Table 26 for more).*/
  L3GD20_HPCF_9 = 0x09              /**< Depends on ODR (Table 26 for more).*/
} l3gd20_hpcf_t;

/**
 * @brief   L3GD20 LP2 filter mode.
 * @details To activate LP2 HP should be active
 */
typedef enum {
  L3GD20_LP2M_ON = 0x00,            /**< LP2 filter activated.              */
  L3GD20_LP2M_BYPASSED = 0xFF,      /**< LP2 filter bypassed.               */ 
} l3gd20_lp2m_t;

/**
 * @brief   L3GD20 endianness.
 */
typedef enum {
  L3GD20_END_LITTLE = 0x00,         /**< Little endian.                     */
  L3GD20_END_BIG = 0x40             /**< Big endian.                        */
} l3gd20_end_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  L3GD20_UNINIT = 0,                /**< Not initialized.                   */
  L3GD20_STOP = 1,                  /**< Stopped.                           */
  L3GD20_READY = 2                  /**< Ready.                             */
} l3gd20_state_t;

/**
 * @brief   L3GD20 configuration structure.
 */
typedef struct {

#if L3GD20_USE_SPI || defined(__DOXYGEN__)
  /**
   * @brief   SPI driver associated to this L3GD20.
   */
  SPIDriver                 *spip;
  /**
   * @brief   SPI configuration associated to this L3GD20.
   */
  const SPIConfig           *spicfg;
#endif /* L3GD20_USE_SPI */
#if L3GD20_USE_I2C || defined(__DOXYGEN__)
  /**
   * @brief   I2C driver associated to this L3GD20.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief   I2C configuration associated to this L3GD20.
   */
  const I2CConfig           *i2ccfg;
#endif /* L3GD20_USE_I2C */
  /**
   * @brief   L3GD20 gyroscope system initial sensitivity.
   */
  float                     *gyrosensitivity;
  /**
   * @brief   L3GD20 gyroscope system initial bias.
   */
  float                     *gyrobias;
  /** 
   * @brief   L3GD20 gyroscope system initial full scale value.
   */
  l3gd20_fs_t               gyrofullscale;
  /**
   * @brief   L3GD20 gyroscope system output data rate selection.
   */
  l3gd20_odr_t              gyroodr;
#if L3GD20_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief   L3GD20 gyroscope system block data update.
   */
  l3gd20_bdu_t              gyrobdu;
  /**
   * @brief   L3GD20 gyroscope system endianness.
   */
  l3gd20_end_t              gyroendianness;
  /**
   * @brief   L3GD20 gyroscope system LP1 filter bandwidth.
   */
  l3gd20_bw_t               gyrobandwidth;
  /**
   * @brief   L3GD20 gyroscope system HP filter mode.
   */
  l3gd20_hpm_t              gyrohpmode;
  /**
   * @brief   L3GD20 gyroscope system HP configuration.
   */
  l3gd20_hpcf_t             gyrohpcfg;
  /**
   * @brief   L3GD20 gyroscope system LP2 filter mode.
   * @details To activate LP2 HP should be active
   */
  l3gd20_lp2m_t             gyrolp2mode;
#endif
} L3GD20Config;

/**
 * @brief   @p L3GD20 specific methods.
 */
#define _l3gd20_methods_alone                                               \
  /* Change full scale value of L3GD20.*/                                   \
  msg_t (*gyro_set_full_scale)(L3GD20Driver *devp, l3gd20_fs_t fs);

/**
 * @brief   @p L3GD20 specific methods with inherited ones.
 */
#define _l3gd20_methods                                                     \
  _base_object_methods                                                      \
  _l3gd20_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief   @p L3GD20 virtual methods table.
 */
struct L3GD20VMT {
  _l3gd20_methods
};

/**
 * @brief   @p L3GD20Driver specific data.
 */
#define _l3gd20_data                                                        \
  _base_sensor_data                                                         \
  /* Driver state.*/                                                        \
  l3gd20_state_t            state;                                          \
  /* Current configuration data.*/                                          \
  const L3GD20Config        *config;                                        \
  /* Gyroscope subsystem axes number.*/                                     \
  size_t                    gyroaxes;                                       \
  /* Gyroscope subsystem current sensitivity.*/                             \
  float                     gyrosensitivity[L3GD20_GYRO_NUMBER_OF_AXES];    \
  /* Gyroscope subsystem current Bias.*/                                    \
  float                     gyrobias[L3GD20_GYRO_NUMBER_OF_AXES];           \
  /* Gyroscope subsystem current full scale value.*/                        \
  float                     gyrofullscale;

/**
 * @brief   L3GD20 3-axis gyroscope class.
 */
struct L3GD20Driver {
  /** @brief Virtual Methods Table. */
  const struct L3GD20VMT    *vmt;
  /** @brief Base gyroscope interface.*/
  BaseGyroscope             gyro_if;
  _l3gd20_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseGyroscope.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define l3gd20GyroscopeGetAxesNumber(devp)                                  \
        gyroscopeGetAxesNumber(&((devp)->gyro_if))
        
/**
 * @brief   Retrieves raw data from the BaseGyroscope.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define l3gd20GyroscopeReadRaw(devp, axes)                                  \
        gyroscopeReadRaw(&((devp)->gyro_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseGyroscope.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as DPS.
 * @note    The axes array must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define l3gd20GyroscopeReadCooked(devp, axes)                               \
        gyroscopeReadCooked(&((devp)->gyro_if), axes)

/**
 * @brief   Samples bias values for the BaseGyroscope.
 * @note    The L3GD20 shall not be moved during the whole procedure.
 * @note    After this function internal bias is automatically updated.
 * @note    The behavior of this function depends on @p L3GD20_BIAS_ACQ_TIMES
 *          and @p L3GD20_BIAS_SETTLING_US.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define l3gd20GyroscopeSampleBias(devp)                                      \
        gyroscopeSampleBias(&((devp)->gyro_if))

/**
 * @brief   Set bias values for the BaseGyroscope.
 * @note    Bias must be expressed as DPS.
 * @note    The bias buffer must be at least the same size of the BaseGyroscope
 *          axes number.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define l3gd20GyroscopeSetBias(devp, bp)                                    \
        gyroscopeSetBias(&((devp)->gyro_if), bp)

/**
 * @brief   Reset bias values for the BaseGyroscope.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define l3gd20GyroscopeResetBias(devp)                                      \
        gyroscopeResetBias(&((devp)->gyro_if))

/**
 * @brief   Set sensitivity values for the BaseGyroscope.
 * @note    Sensitivity must be expressed as DPS/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseGyroscope axes number.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define l3gd20GyroscopeSetSensitivity(devp, sp)                             \
        gyroscopeSetSensitivity(&((devp)->gyro_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseGyroscope.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define l3gd20GyroscopeResetSensitivity(devp)                               \
        gyroscopeResetSensitivity(&((devp)->gyro_if))

/**
 * @brief   Changes the L3GD20Driver gyroscope fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p L3GD20Driver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define l3gd20GyroscopeSetFullScale(devp, fs)                               \
        (devp)->vmt->acc_set_full_scale(devp, fs)
        
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void l3gd20ObjectInit(L3GD20Driver *devp);
  void l3gd20Start(L3GD20Driver *devp, const L3GD20Config *config);
  void l3gd20Stop(L3GD20Driver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _L3GD20_H_ */

/** @} */

