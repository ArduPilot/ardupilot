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
 * @file    ex_barometer.h
 * @brief   Generic barometer interface header.
 *
 * @addtogroup EX_BAROMETER
 * @{
 */

#ifndef EX_BAROMETER_H
#define EX_BAROMETER_H

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
 * @brief   BaseBarometer specific methods.
 */
#define _base_barometer_methods_alone                                       \
  /* Invoke the set bias procedure.*/                                       \
  msg_t (*set_bias)(void *instance, float biases[]);                        \
  /* Remove bias stored data.*/                                             \
  msg_t (*reset_bias)(void *instance);                                      \
  /* Invoke the set sensitivity procedure.*/                                \
  msg_t (*set_sensitivity)(void *instance, float sensitivities[]);          \
  /* Restore sensitivity stored data to default.*/                          \
  msg_t (*reset_sensitivity)(void *instance);


/**
 * @brief   BaseBarometer specific methods with inherited ones.
 */
#define _base_barometer_methods                                             \
  _base_sensor_methods                                                      \
  _base_barometer_methods_alone

/**
 * @brief   @p BaseBarometer virtual methods table.
 */
struct BaseBarometerVMT {
  _base_barometer_methods
};

/**
 * @brief   @p BaseBarometer specific data.
 */
#define _base_barometer_data                                                \
  _base_sensor_data
	
/**
 * @extends BaseSensor
 *
 * @brief   Base barometer class.
 * @details This class represents a generic barometer.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseBarometerVMT *vmt;
  _base_barometer_data
} BaseBarometer;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/
/**
 * @name    Macro Functions (BaseBarometer)
 * @{
 */
/**
 * @brief   Barometer get channels number.
 *
 * @param[in] ip        pointer to a @p BaseBarometer class.
 * @return              The number of channels of the BaseBarometer
 *
 * @api
 */
#define barometerGetChannelsNumber(ip)                                      \
        (ip)->vmt->get_channels_number(ip)

/**
 * @brief   Barometer read raw data.
 *
 * @param[in] ip        pointer to a @p BaseBarometer class.
 * @param[in] dp        pointer to a data array.
 * 
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define barometerReadRaw(ip, dp)                                              \
        (ip)->vmt->read_raw(ip, dp)

/**
 * @brief   Barometer read cooked data.
 *
 * @param[in] ip        pointer to a @p BaseBarometer class.
 * @param[in] dp        pointer to a data array.
 * 
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define barometerReadCooked(ip, dp)                                           \
        (ip)->vmt->read_cooked(ip, dp)

/**
 * @brief   Updates barometer bias data from received buffer.
 * @note    The bias buffer must have the same length of the
 *          the barometer channels number.
 *
 * @param[in] ip        pointer to a @p BaseBarometer class.
 * @param[in] bp        pointer to a buffer of bias values.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define barometerSetBias(ip, bp)                                            \
        (ip)->vmt->set_bias(ip, bp)

/**
 * @brief   Reset barometer bias data restoring it to zero.
 *
 * @param[in] ip        pointer to a @p BaseBarometer class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define barometerResetBias(ip)                                               \
        (ip)->vmt->reset_bias(ip)

/**
 * @brief   Updates barometer sensitivity data from received buffer.
 * @note    The sensitivity buffer must have the same length of the
 *          the barometer channels number.
 *
 * @param[in] ip        pointer to a @p BaseBarometer class.
 * @param[in] sp        pointer to a buffer of sensitivity values.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define barometerSetSensitivity(ip, sp)                                     \
        (ip)->vmt->set_sensitivity(ip, sp)

/**
 * @brief   Reset barometer sensitivity data restoring it to its typical
 *          value.
 *
 * @param[in] ip        pointer to a @p BaseBarometer class.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more errors occurred.
 *
 * @api
 */
#define barometerResetSensitivity(ip)                                       \
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

#endif /* EX_BAROMETER_H */

/** @} */
