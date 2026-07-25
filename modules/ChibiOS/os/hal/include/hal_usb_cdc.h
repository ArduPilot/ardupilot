/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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
 * @file    hal_usb_cdc.h
 * @brief   USB CDC macros and structures.
 *
 * @addtogroup USB_CDC
 * @{
 */

#ifndef USB_CDC_H
#define USB_CDC_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    CDC specific messages.
 * @{
 */
#define CDC_SEND_ENCAPSULATED_COMMAND       0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE       0x01U
#define CDC_SET_COMM_FEATURE                0x02U
#define CDC_GET_COMM_FEATURE                0x03U
#define CDC_CLEAR_COMM_FEATURE              0x04U
#define CDC_SET_AUX_LINE_STATE              0x10U
#define CDC_SET_HOOK_STATE                  0x11U
#define CDC_PULSE_SETUP                     0x12U
#define CDC_SEND_PULSE                      0x13U
#define CDC_SET_PULSE_TIME                  0x14U
#define CDC_RING_AUX_JACK                   0x15U
#define CDC_SET_LINE_CODING                 0x20U
#define CDC_GET_LINE_CODING                 0x21U
#define CDC_SET_CONTROL_LINE_STATE          0x22U
#define CDC_SEND_BREAK                      0x23U
#define CDC_SET_RINGER_PARMS                0x30U
#define CDC_GET_RINGER_PARMS                0x31U
#define CDC_SET_OPERATION_PARMS             0x32U
#define CDC_GET_OPERATION_PARMS             0x33U
/** @} */

/**
 * @name    CDC classes
 * @{
 */
#define CDC_COMMUNICATION_INTERFACE_CLASS   0x02U
#define CDC_DATA_INTERFACE_CLASS            0x0AU
/** @} */

/**
 * @name    CDC subclasses
 * @{
 */
#define CDC_ABSTRACT_CONTROL_MODEL          0x02U
/** @} */

/**
 * @name    CDC descriptors
 * @{
 */
#define CDC_CS_INTERFACE                    0x24U
/** @} */

/**
 * @name    CDC subdescriptors
 * @{
 */
#define CDC_HEADER                          0x00U
#define CDC_CALL_MANAGEMENT                 0x01U
#define CDC_ABSTRACT_CONTROL_MANAGEMENT     0x02U
#define CDC_UNION                           0x06U
/** @} */

/**
 * @name    Line Control bit definitions.
 * @{
 */
#define LC_STOP_1                           0U
#define LC_STOP_1P5                         1U
#define LC_STOP_2                           2U

#define LC_PARITY_NONE                      0U
#define LC_PARITY_ODD                       1U
#define LC_PARITY_EVEN                      2U
#define LC_PARITY_MARK                      3U
#define LC_PARITY_SPACE                     4U
/** @} */

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
 * @brief   Type of Line Coding structure.
 */
typedef struct {
  uint8_t                       dwDTERate[4];
  uint8_t                       bCharFormat;
  uint8_t                       bParityType;
  uint8_t                       bDataBits;
} cdc_linecoding_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* USB_CDC_H */

/** @} */
