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
 * @file    ex_hygrometer.h
 * @brief   Generic hygrometer interface header.
 *
 * @addtogroup EX_HYGROMETER
 * @{
 */

#ifndef EX_HYGROMETER_H
#define EX_HYGROMETER_H

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
 * @brief   BaseHygrometer specific methods.
 */
#define _base_hygrometer_methods_alone                                      \
  /* Invoke the set bias procedure.*/                                       \
  msg_t (*set_bias)(void *instance, float biases[]);                        \
  /* Remove bias stored data.*/                                             \
  msg_t (*reset_bias)(void *instance);                                      \
  /* Invoke the set sensitivity procedure.*/                                \
  msg_t (*set_sensitivity)(void *instance, float sensitivities[]);          \
  /* Restore sensitivity stored data to default.*/                          \
  msg_t (*reset_sensitivity)(void *instance);


/**
 * @brief   BaseHygrometer specific methods with inherited ones.
 */
#define _base_hygrometer_methods                                            \
  _base_sensor_methods                                                      \
  _base_hygrometer_methods_alone

/**
 * @brief   @p BaseHygrometer virtual methods table.
 */
struct BaseHygrometerVMT {
  _base_hygrometer_methods
};

/**
 * @brief   @p BaseHygrometer specific data.
 */
#define _base_hygrometer_data                                               \
  _base_sensor_data
	
/**
 * @extends BaseSensor
 *
 * @brief   Base hygrometer class.
 * @details This class represents a generic hygrometer.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseHygrometerVMT *vmt;
  _base_hygrometer_data
} BaseHygrometer;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/
/**
 * @name    Macro Functions (BaseHygrometer)
 * @{
 */
/**
 * @brief   Hygrometer get channels number.
 *
 * @param[in] ip        pointer to a @p BaseHygrometer class.
 * @return              The number of channels of the BaseHygrometer
 *
 * @api
 */
#define hygrometerGetChannelsNumber(ip)                                     \
        (ip)->vmt->get_channels_number(ip)

/**
 * @brief   Hygrometer read raw data.
 *
 * @param[in] ip        pointer to a @p BaseHygrometer class.
 * @param[in] dp        pointer to a data array.
 * 
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define hygrometerReadRaw(ip, dp)                                           \
        (ip)->vmt->read_raw(ip, dp)

/**
 * @brief   Hygrometer read cooked data.
 *
 * @param[in] ip        pointer to a @p BaseHygrometer class.
 * @param[in] dp        pointer to a data array.
 * 
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define hygrometerReadCooked(ip, dp)                                        \
        (ip)->vmt->read_cooked(ip, dp)

/**
 * @brief   Updates hygrometer bias data from received buffer.
 * @note    The bias buffer must have the same length of the
 *          the hygrometer channels number.
 *
 * @param[in] ip        pointer to a @p BaseHygrometer class.
 * @param[in] bp        pointer to a buffer of bias values.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define hygrometerSetBias(ip, bp)                                           \
        (ip)->vmt->set_bias(ip, bp)

/**
 * @brief   Reset hygrometer bias data restoring it to zero.
 *
 * @param[in] ip        pointer to a @p BaseHygrometer class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define hygrometerResetBias(ip)                                             \
        (ip)->vmt->reset_bias(ip)

/**
 * @brief   Updates hygrometer sensitivity data from received buffer.
 * @note    The sensitivity buffer must have the same length of the
 *          the hygrometer channels number.
 *
 * @param[in] ip        pointer to a @p BaseHygrometer class.
 * @param[in] sp        pointer to a buffer of sensitivity values.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define hygrometerSetSensitivity(ip, sp)                                    \
        (ip)->vmt->set_sensitivity(ip, sp)

/**
 * @brief   Reset hygrometer sensitivity data restoring it to its typical
 *          value.
 *
 * @param[in] ip        pointer to a @p BaseHygrometer class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define hygrometerResetSensitivity(ip)                                      \
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

#endif /* EX_HYGROMETER_H */

/** @} */
