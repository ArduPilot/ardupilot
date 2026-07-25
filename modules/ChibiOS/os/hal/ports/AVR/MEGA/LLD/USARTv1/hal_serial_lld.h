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
 * @file    USARTv1/hal_serial_lld.h
 * @brief   AVR/MEGA SERIAL subsystem low level driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @brief   USART0 driver enable switch.
 * @details If set to @p TRUE the support for USART0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_SERIAL_USE_USART0) || defined(__DOXYGEN__)
  #define AVR_SERIAL_USE_USART0              TRUE
#endif

/**
 * @brief   USART1 driver enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(AVR_SERIAL_USE_USART1) || defined(__DOXYGEN__)
#define AVR_SERIAL_USE_USART1              TRUE
#endif

/**
 * @brief   USART2 driver enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_SERIAL_USE_USART2) || defined(__DOXYGEN__)
#define AVR_SERIAL_USE_USART2              FALSE
#endif

/**
 * @brief   USART3 driver enable switch.
 * @details If set to @p TRUE the support for USART3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_SERIAL_USE_USART3) || defined(__DOXYGEN__)
#define AVR_SERIAL_USE_USART3              FALSE
#endif

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   AVR Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct hal_serial_config {
  /**
   * @brief Initialization value for the BRR register.
   */
  uint16_t                  sc_brr;
  /**
   * @brief Number of bits per character (USART_CHAR_SIZE_5 to USART_CHAR_SIZE_9).
   */
  uint8_t                   sc_bits_per_char;
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state. */                                                       \
  sdstate_t                 state;                                          \
  /* Input queue. */                                                        \
  input_queue_t             iqueue;                                         \
  /* Output queue. */                                                       \
  output_queue_t            oqueue;                                         \
  /* Input circular buffer. */                                              \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer. */                                             \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields. */

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/**
 * @brief   Macro for baud rate computation.
 * @note    Make sure the final baud rate is within tolerance.
 */
#define UBRR(b)     (((F_CPU / b) >> 4) - 1)

/**
 * @brief   Macro for baud rate computation when U2Xn == 1.
 * @note    Make sure the final baud rate is within tolerance.
 */
#define UBRR2x(b)    (((F_CPU / b) >> 3) - 1)

/**
* @brief   Macro for baud rate computation.
* @note    Make sure the final baud rate is within tolerance.
* @note    This version uses floating point math for greater accuracy.
*/
#define UBRR_F(b)   ((((double) F_CPU / (double) b) / 16.0) - 0.5)

/**
* @brief   Macro for baud rate computation when U2Xn == 1.
* @note    Make sure the final baud rate is within tolerance.
* @note    This version uses floating point math for greater accuracy.
*/
#define UBRR2x_F(b)  ((((double) F_CPU / (double) b) / 8.0) - 0.5)

#define USART_CHAR_SIZE_5      0
#define USART_CHAR_SIZE_6      1
#define USART_CHAR_SIZE_7      2
#define USART_CHAR_SIZE_8      3
#define USART_CHAR_SIZE_9      4

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if AVR_SERIAL_USE_USART0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if AVR_SERIAL_USE_USART1 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif
#if AVR_SERIAL_USE_USART2 && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif
#if AVR_SERIAL_USE_USART3 && !defined(__DOXYGEN__)
extern SerialDriver SD4;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
