/*
    ChibiOS - Copyright (C) 2006..2018 Rocco Marco Guglielmi

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    ex_accelerometer.h
 * @brief   Generic accelerometer interface header.
 *
 * @addtogroup EX_ACCELEROMETER
 * @{
 */

#ifndef EX_ACCELEROMETER_H
#define EX_ACCELEROMETER_H

#include "ex_sensors.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   BaseAccelerometer specific methods.
 */
#define _base_accelerometer_methods_alone                                   \
  /* Invoke the set bias procedure.*/                                       \
  msg_t (*set_bias)(void *instance, float biases[]);                        \
  /* Remove bias stored data.*/                                             \
  msg_t (*reset_bias)(void *instance);                                      \
  /* Invoke the set sensitivity procedure.*/                                \
  msg_t (*set_sensitivity)(void *instance, float sensitivities[]);          \
  /* Restore sensitivity stored data to default.*/                          \
  msg_t (*reset_sensitivity)(void *instance);

/**
 * @brief   BaseAccelerometer specific methods with inherited ones.
 */
#define _base_accelerometer_methods                                         \
  _base_sensor_methods                                                      \
  _base_accelerometer_methods_alone

/**
 * @brief   @p BaseAccelerometer virtual methods table.
 */
struct BaseAccelerometerVMT {
  _base_accelerometer_methods
};

/**
 * @brief   @p BaseAccelerometer specific data.
 */
#define _base_accelerometer_data                                            \
  _base_sensor_data
	
/**
 * @extends BaseSensor
 *
 * @brief   Base accelerometer class.
 * @details This class represents a generic a generic accelerometer.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseAccelerometerVMT *vmt;
  _base_accelerometer_data
} BaseAccelerometer;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions (BaseAccelerometer)
 * @{
 */
/**
 * @brief   Accelerometer get axes number.
 *
 * @param[in] ip        pointer to a @p BaseAccelerometer class.
 * @return              The number of axes of the BaseAccelerometer
 *
 * @api
 */
#define accelerometerGetAxesNumber(ip)                                      \
        (ip)->vmt->get_channels_number(ip)

/**
 * @brief   Accelerometer read raw data.
 *
 * @param[in] ip        pointer to a @p BaseAccelerometer class.
 * @param[in] dp        pointer to a data array.
 * 
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define accelerometerReadRaw(ip, dp)                                        \
        (ip)->vmt->read_raw(ip, dp)

/**
 * @brief   Accelerometer read cooked data.
 *
 * @param[in] ip        pointer to a @p BaseAccelerometer class.
 * @param[in] dp        pointer to a data array.
 * 
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define accelerometerReadCooked(ip, dp)                                     \
        (ip)->vmt->read_cooked(ip, dp)

/**
 * @brief   Updates accelerometer bias data from received buffer.
 * @note    The bias buffer must have the same length of the
 *          the accelerometer axes number.
 *
 *
 * @param[in] ip        pointer to a @p BaseAccelerometer class.
 * @param[in] bp        pointer to a buffer of bias values.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define accelerometerSetBias(ip, bp)                                        \
        (ip)->vmt->set_bias(ip, bp)

/**
 * @brief   Reset accelerometer bias data restoring it to zero.
 *
 * @param[in] ip        pointer to a @p BaseAccelerometer class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define accelerometerResetBias(ip)                                          \
        (ip)->vmt->reset_bias(ip)

/**
 * @brief   Updates accelerometer sensitivity data from received buffer.
 * @note    The sensitivity buffer must have the same length of the
 *          the accelerometer axes number.
 *
 * @param[in] ip        pointer to a @p BaseAccelerometer class.
 * @param[in] sp        pointer to a buffer of sensitivity values.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define accelerometerSetSensitivity(ip, sp)                                 \
        (ip)->vmt->set_sensitivity(ip, sp)

/**
 * @brief   Reset accelerometer sensitivity data restoring it to its typical
 *          value.
 *
 * @param[in] ip        pointer to a @p BaseAccelerometer class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define accelerometerResetSensitivity(ip)                                   \
        (ip)->vmt->reset_sensitivity(ip)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* EX_ACCELEROMETER_H */

/** @} */
