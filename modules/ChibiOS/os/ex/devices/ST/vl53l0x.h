/*
    ChibiOS - Copyright (C) 2016..2018 Edoardo Lombardi, Rocco Marco Guglielmi

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
 * @file    vl53l0x.h
 * @brief   VL53L0X MEMS interface module header.
 *
 * @addtogroup VL53L0X
 * @ingroup EX_ST
 * @{
 */
#ifndef _VL53L0X_H_
#define _VL53L0X_H_

#include "ex_rangefinder.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   VL53L0X driver version string.
 */
#define EX_VL53L0X_VERSION                  "1.0.0"

/**
 * @brief   VL53L0X driver version major number.
 */
#define EX_VL53L0X_MAJOR                    1

/**
 * @brief   VL53L0X driver version minor number.
 */
#define EX_VL53L0X_MINOR                    0

/**
 * @brief   VL53L0X driver version patch number.
 */
#define EX_VL53L0X_PATCH                    0
/** @} */

/**
 * @brief   VL53L0X rangefinder subsystem characteristics.
 * @note    Sensitivity is expressed as hPa/LSB whereas hPa stand for
 *          hectopascal.
 * @note    Bias is expressed as hPa.
 *
 * @{
 */
#define VL53L0X_RANGE_NUMBER_OF_AXES        1U

/* CHTODO: Check this. */
#define VL53L0X_RANGE_SENS                  0.00024414f
#define VL53L0X_RANGE_BIAS                  0.0f
/** @} */

/**
 * @name    VL53L0X register addresses
 * @{
 */
#define VL53L0X_REG_SYSRANGE_START                                0x00
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                        0x01
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD                0x04
#define VL53L0X_REG_SYSTEM_RANGE_CONFIG                           0x09
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                  0x0A
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                        0x0B
#define VL53L0X_REG_SYSTEM_THRESH_HIGH                            0x0C
#define VL53L0X_REG_SYSTEM_THRESH_LOW                             0x0E

#define VL53L0X_REG_RESULT_INTERRUPT_STATUS                       0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS                           0x14

#define VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS         0x20
#define VL53L0X_REG_PRE_RANGE_CONFIG_MIN_SNR                      0x27
#define VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM             0x28
#define VL53L0X_REG_ALGO_PHASECAL_LIM                             0x30
#define VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT                  0x30
#define VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH                     0x32
#define VL53L0X_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT         0x33

#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT   0x44
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW            0x47
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH           0x48
#define VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD           0x4E
#define VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET              0x4F

#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD                 0x50
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI            0x51
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO            0x52
#define VL53L0X_REG_HISTOGRAM_CONFIG_READOUT_CTRL                 0x55
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW              0x56
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH             0x57

#define VL53L0X_REG_MSRC_CONFIG_CONTROL                           0x60
#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI              0x61
#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO              0x62
#define VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT            0x64
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_SNR                    0x67

#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD               0x70
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x71
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x72

#define VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE              0x80
#define VL53L0X_REG_SYSTEM_HISTOGRAM_BIN                          0x81
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                       0x84
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV             0x89
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                      0x8A

#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0              0xB0
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1              0xB1
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2              0xB2
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3              0xB3
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4              0xB4
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5              0xB5
#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT             0xB6
#define VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF                   0xB6
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN         0xBC
#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N                   0xBF

#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN          0xC0
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID                       0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID                    0xC2

#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF         0xD0
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF          0xD4

#define VL53L0X_REG_OSC_CALIBRATE_VAL                             0xF8
/** @} */

/**
 * @name    VL53L0X_INT_CFG register bits definitions
 * @{
 */
#define VL53L0X_INT_CFG_MASK                0xFF
#define VL53L0X_INT_CFG_PHE                 (1 << 0)
#define VL53L0X_INT_CFG_PLE                 (1 << 1)
#define VL53L0X_INT_CFG_LIR                 (1 << 2)
#define VL53L0X_INT_CFG_DIFF_EN             (1 << 3)
#define VL53L0X_INT_CFG_RESET_AZ            (1 << 4)
#define VL53L0X_INT_CFG_AUTOZERO            (1 << 5)
#define VL53L0X_INT_CFG_RESET_ARP           (1 << 6)
#define VL53L0X_INT_CFG_AUTORIFP            (1 << 7)
/** @} */

/**
 * @name    VL53L0X_CTRL_REG1 register bits definitions
 * @{
 */
#define VL53L0X_CTRL_REG1_MASK              0x7F
#define VL53L0X_CTRL_REG1_SIM               (1 << 0)
#define VL53L0X_CTRL_REG1_BDU               (1 << 1)
#define VL53L0X_CTRL_REG1_LPFP_CFG          (1 << 2)
#define VL53L0X_CTRL_REG1_LPFP_EN           (1 << 3)
#define VL53L0X_CTRL_REG1_ODR0              (1 << 4)
#define VL53L0X_CTRL_REG1_ODR1              (1 << 5)
#define VL53L0X_CTRL_REG1_ODR2              (1 << 6)
/** @} */

/**
 * @name    VL53L0X_CTRL_REG2 register bits definitions
 * @{
 */
#define VL53L0X_CTRL_REG2_MASK              0xFD
#define VL53L0X_CTRL_REG2_ONE_SHOT          (1 << 0)
#define VL53L0X_CTRL_REG2_SWRESET           (1 << 2)
#define VL53L0X_CTRL_REG2_I2C_DIS           (1 << 3)
#define VL53L0X_CTRL_REG2_IF_ADD_INC        (1 << 4)
#define VL53L0X_CTRL_REG2_STOP_ON_FTH       (1 << 5)
#define VL53L0X_CTRL_REG2_FIFO_EN           (1 << 6)
#define VL53L0X_CTRL_REG2_BOOT              (1 << 7)
/** @} */

/**
 * @name    VL53L0X_CTRL_REG3 register bits definitions
 * @{
 */
#define VL53L0X_CTRL_REG3_MASK              0xFF
#define VL53L0X_CTRL_REG3_INT_S1            (1 << 0)
#define VL53L0X_CTRL_REG3_INT_S2            (1 << 1)
#define VL53L0X_CTRL_REG3_DRDY              (1 << 2)
#define VL53L0X_CTRL_REG3_F_OVR             (1 << 3)
#define VL53L0X_CTRL_REG3_F_FTH             (1 << 4)
#define VL53L0X_CTRL_REG3_F_FSS5            (1 << 5)
#define VL53L0X_CTRL_REG3_PP_OD             (1 << 6)
#define VL53L0X_CTRL_REG3_INT_H_L           (1 << 7)
/** @} */

/**
 * @name    VL53L0X_INT_SRC register bits definitions
 * @{
 */
#define VL53L0X_INT_SRC_MASK                0x87
#define VL53L0X_INT_SRC_PH                  (1 << 0)
#define VL53L0X_INT_SRC_PL                  (1 << 1)
#define VL53L0X_INT_SRC_IA                  (1 << 2)
#define VL53L0X_INT_SRC_BOOT_STATUS         (1 << 8)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   VL53L0X I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(VL53L0X_USE_I2C) || defined(__DOXYGEN__)
#define VL53L0X_USE_I2C                     TRUE
#endif

/**
 * @brief   VL53L0X shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(VL53L0X_SHARED_I2C) || defined(__DOXYGEN__)
#define VL53L0X_SHARED_I2C                  FALSE
#endif

/**
 * @brief   VL53L0X advanced configurations switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#if !defined(VL53L0X_USE_ADVANCED) || defined(__DOXYGEN__)
#define VL53L0X_USE_ADVANCED                FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if VL53L0X_USE_I2C && !HAL_USE_I2C
#error "VL53L0X_USE_I2C requires HAL_USE_I2C"
#endif

#if VL53L0X_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "VL53L0X_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    VL53L0X data structures and types.
 * @{
 */
/**
 * @brief   Structure representing a VL53L0X driver.
 */
typedef struct VL53L0XDriver VL53L0XDriver;

/**
 * @brief  VL53L0X slave address
 */
typedef enum {
  VL53L0X_SAD_DEFAULT = 0x29,       /**< Default slave address             */
}vl53l0x_sad_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  VL53L0X_UNINIT = 0,               /**< Not initialized.                   */
  VL53L0X_STOP = 1,                 /**< Stopped.                           */
  VL53L0X_READY = 2,                /**< Ready.                             */
} vl53l0x_state_t;

/**
 * @brief   VL53L0X configuration structure.
 */
typedef struct {
#if VL53L0X_USE_I2C || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this VL53L0X.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this VL53L0X.
   */
  const I2CConfig           *i2ccfg;
  /**
   * @brief VL53L0X slave address
   */
  vl53l0x_sad_t             slaveaddress;
#endif /* VL53L0X_USE_I2C */
#if VL53L0X_USE_ADVANCED || defined(__DOXYGEN__)
  /**
   * @brief Dummy placeholder
   */
#endif
} VL53L0XConfig;

/**
 * @brief   @p VL53L0X specific methods.
 * @note    No methods so far, just a common ancestor interface.
 */
#define _vl53l0x_methods_alone

/**
 * @brief @p VL53L0X specific methods with inherited ones.
 */
#define _vl53l0x_methods                                                    \
  _base_object_methods                                                      \
  _vl53l0x_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief @p VL53L0X virtual methods table.
 */
struct VL53L0XVMT {
  _vl53l0x_methods
};
  
/**
 * @brief   @p VL53L0XDriver specific data.
 */
#define _vl53l0x_data                                                       \
  /* Driver state.*/                                                        \
  vl53l0x_state_t           state;                                          \
  /* Current configuration data.*/                                          \
  const VL53L0XConfig       *config;                                        \
  /* RangeFinder subsystem axes number.*/                                   \
  size_t                    rangeaxes;

/**
 * @brief   VL53L0X 2-axis rangemeter/thermometer class.
 */
struct VL53L0XDriver {
  /** @brief Virtual Methods Table.*/
  const struct VL53L0XVMT   *vmt;
  /** @brief Base rangemeter interface.*/
  BaseRangeFinder           range_if;
  _vl53l0x_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseRangeFinder.
 *
 * @param[in] devp      pointer to @p VL53L0XDriver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define vl53l0xRangeFinderGetAxesNumber(devp)                                 \
        rangemeterGetAxesNumber(&((devp)->range_if))

/**
 * @brief   Retrieves raw data from the BaseRangeFinder.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseRangeFinder axes number.
 *
 * @param[in] devp      pointer to @p VL53L0XDriver.
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
#define vl53l0xRangeFinderReadRaw(devp, axes)                                 \
        rangemeterReadRaw(&((devp)->range_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseRangeFinder.
 * @note    Final data is expressed as mm.
 * @note    The axes array must be at least the same size of the
 *          BaseRangeFinder axes number.
 *
 * @param[in] devp      pointer to @p VL53L0XDriver.
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
#define vl53l0xRangeFinderReadCooked(devp, axes)                              \
        rangemeterReadCooked(&((devp)->range_if), axes)
        
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void vl53l0xObjectInit(VL53L0XDriver *devp);
  void vl53l0xStart(VL53L0XDriver *devp, const VL53L0XConfig *config);
  void vl53l0xStop(VL53L0XDriver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _VL53L0X_H_ */

/** @} */

