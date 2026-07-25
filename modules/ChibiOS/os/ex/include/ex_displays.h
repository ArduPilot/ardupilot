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
 * @file    ex_displays.h
 * @brief   Generic display interface header.
 *
 * @addtogroup EX_DISPLAYS
 * @{
 */

#ifndef EX_DISPLAYS_H
#define EX_DISPLAYS_H

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
 * @brief   @p BaseDisplay specific methods.
 * @note    No methods so far, just a common ancestor interface.
 */
#define _base_display_methods_alone

/**
 * @brief   @p BaseDisplay specific methods with inherited ones.
 */
#define _base_display_methods                                               \
  _base_display_methods_alone

/**
 * @brief   @p BaseDisplay virtual methods table.
 */
struct BaseDisplayVMT {
  _base_display_methods
};

/**
 * @brief   @p BaseDisplay specific data.
 * @note    It is empty because @p BaseDisplay is only an interface
 *          without implementation.
 */
#define _base_display_data

/**
 * @brief   Base display class.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseDisplayVMT *vmt_basedisplay;
  _base_display_data
} BaseDisplay;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions (BaseDisplay)
 * @{
 */
/**
 * @brief   Sensors get axes number.
 *
 * @param[in] ip        pointer to a @p BaseDisplay or derived class.
 * @return              The number of axes of the BaseDisplay
 *
 * @api
 */
#define displayGetType(ip) (ip)->vmt_basedisplay->get_type(ip)
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

#endif /* EX_DISPLAYS_H */

/** @} */
