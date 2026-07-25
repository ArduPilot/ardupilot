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
 * @file    hal_serial_lld.h
 * @brief   Win32 simulator low level serial driver header.
 *
 * @addtogroup WIN32_SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Serial buffers size.
 * @details Configuration parameter, you can change the depth of the queue
 *          buffers depending on the requirements of your application.
 */
#if !defined(SERIAL_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define SERIAL_BUFFERS_SIZE                 1024
#endif

/**
 * @brief   SD1 driver enable switch.
 * @details If set to @p TRUE the support for SD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_WIN32_SERIAL1) || defined(__DOXYGEN__)
#define USE_WIN32_SERIAL1                   TRUE
#endif

/**
 * @brief   SD2 driver enable switch.
 * @details If set to @p TRUE the support for SD2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_WIN32_SERIAL2) || defined(__DOXYGEN__)
#define USE_WIN32_SERIAL2                   TRUE
#endif

/**
 * @brief   Listen port for SD1.
 */
#if !defined(SD1_PORT) || defined(__DOXYGEN__)
#define SD1_PORT                            29001
#endif

/**
 * @brief   Listen port for SD2.
 */
#if !defined(SD2_PORT) || defined(__DOXYGEN__)
#define SD2_PORT                            29002
#endif

/*===========================================================================*/
/* Unsupported event flags and custom events.                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Generic Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct hal_serial_config {
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  input_queue_t             iqueue;                                         \
  /* Output queue.*/                                                        \
  output_queue_t            oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Listen socket for simulated serial port.*/                             \
  SOCKET                    com_listen;                                     \
  /* Data socket for simulated serial port.*/                               \
  SOCKET                    com_data;                                       \
  /* Port readable name.*/                                                  \
  const char                *com_name;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if USE_WIN32_SERIAL1 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if USE_WIN32_SERIAL2 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
  bool sd_lld_interrupt_pending(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
