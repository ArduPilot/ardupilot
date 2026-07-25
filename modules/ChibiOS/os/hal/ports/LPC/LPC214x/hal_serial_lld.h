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
 * @file    LPC214x/hal_serial_lld.h
 * @brief   LPC214x low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   UART0 driver enable switch.
 * @details If set to @p TRUE the support for UART0 is included.
 * @note    The default is @p TRUE .
 */
#if !defined(USE_LPC214x_UART0) || defined(__DOXYGEN__)
#define USE_LPC214x_UART0           TRUE
#endif

/**
 * @brief   UART1 driver enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(USE_LPC214x_UART1) || defined(__DOXYGEN__)
#define USE_LPC214x_UART1           TRUE
#endif

/**
 * @brief   FIFO preload parameter.
 * @details Configuration parameter, this values defines how many bytes are
 *          preloaded in the HW transmit FIFO for each interrupt, the maximum
 *          value is 16 the minimum is 1.
 * @note    An high value reduces the number of interrupts generated but can
 *          also increase the worst case interrupt response time because the
 *          preload loops.
 */
#if !defined(LPC214x_UART_FIFO_PRELOAD) || defined(__DOXYGEN__)
#define LPC214x_UART_FIFO_PRELOAD   16
#endif

/**
 * @brief   UART0 interrupt priority level setting.
 */
#if !defined(LPC214x_UART0_PRIORITY) || defined(__DOXYGEN__)
#define LPC214x_UART0_PRIORITY      1
#endif

/**
 * @brief   UART1 interrupt priority level setting.
 */
#if !defined(LPC214x_UART1_PRIORITY) || defined(__DOXYGEN__)
#define LPC214x_UART1_PRIORITY      2
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (LPC214x_UART_FIFO_PRELOAD < 1) || (LPC214x_UART_FIFO_PRELOAD > 16)
#error "invalid LPC214x_UART_FIFO_PRELOAD setting"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   LPC214x Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct hal_serial_config {
  /**
   * @brief Bit rate.
   */
  uint32_t                  sc_speed;
  /**
   * @brief Initialization value for the LCR register.
   */
  uint32_t                  sc_lcr;
  /**
   * @brief Initialization value for the FCR register.
   */
  uint32_t                  sc_fcr;
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
  /* Pointer to the USART registers block.*/                                \
  UART                      *uart;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if USE_LPC214x_UART0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if USE_LPC214x_UART1 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
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
