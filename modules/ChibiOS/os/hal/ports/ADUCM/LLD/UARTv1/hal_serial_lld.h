/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    UARTv1/hal_serial_lld.c
 * @brief   ADUCM low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#include "aducm_uart.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Advanced buffering support switch.
 * @details This constants enables the advanced buffering support in the
 *          low level driver, the queue buffer is no more part of the
 *          @p SerialDriver structure, each driver can have a different
 *          queue size.
 */
#define SERIAL_ADVANCED_BUFFERING_SUPPORT   TRUE

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    ADUCM configuration options
 * @{
 */
/**
 * @brief   UART0 driver enable switch.
 * @details If set to @p TRUE the support for UART0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ADUCM_SERIAL_USE_UART0) || defined(__DOXYGEN__)
#define ADUCM_SERIAL_USE_UART0              FALSE
#endif

/**
 * @brief   UART1 driver enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ADUCM_SERIAL_USE_UART1) || defined(__DOXYGEN__)
#define ADUCM_SERIAL_USE_UART1              FALSE
#endif

/**
 * @brief   UART0 interrupt priority level setting.
 */
#if !defined(ADUCM_SERIAL_UART0_PRIORITY) || defined(__DOXYGEN__)
#define ADUCM_SERIAL_UART0_PRIORITY         12
#endif

/**
 * @brief   UART1 interrupt priority level setting.
 */
#if !defined(ADUCM_SERIAL_UART1_PRIORITY) || defined(__DOXYGEN__)
#define ADUCM_SERIAL_UART1_PRIORITY         12
#endif

/**
 * @brief   Input buffer size for UART0.
 */
#if !defined(ADUCM_SERIAL_UART0_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define ADUCM_SERIAL_UART0_IN_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for UART0.
 */
#if !defined(ADUCM_SERIAL_UART0_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define ADUCM_SERIAL_UART0_OUT_BUF_SIZE     SERIAL_BUFFERS_SIZE
#endif
/** @} */

/**
 * @brief   Input buffer size for UART1.
 */
#if !defined(ADUCM_SERIAL_UART1_IN_BUF_SIZE) || defined(__DOXYGEN__)
#define ADUCM_SERIAL_UART1_IN_BUF_SIZE      SERIAL_BUFFERS_SIZE
#endif

/**
 * @brief   Output buffer size for UART1.
 */
#if !defined(ADUCM_SERIAL_UART1_OUT_BUF_SIZE) || defined(__DOXYGEN__)
#define ADUCM_SERIAL_UART1_OUT_BUF_SIZE     SERIAL_BUFFERS_SIZE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if ADUCM_SERIAL_USE_UART0 && !ADUCM_HAS_UART0
#error "UART0 not present in the selected device"
#endif

#if ADUCM_SERIAL_USE_UART1 && !ADUCM_HAS_UART1
#error "UART1 not present in the selected device"
#endif

#if !ADUCM_SERIAL_USE_UART0 && !ADUCM_SERIAL_USE_UART1
#error "SERIAL driver activated but no UART peripheral assigned"
#endif

#if  ADUCM_SERIAL_USE_UART0 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(ADUCM_SERIAL_UART0_PRIORITY)
#error "Invalid IRQ priority assigned to UART0"
#endif

#if  ADUCM_SERIAL_USE_UART1 &&                                              \
    !OSAL_IRQ_IS_VALID_PRIORITY(ADUCM_SERIAL_UART1_PRIORITY)
#error "Invalid IRQ priority assigned to UART1"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ADUCM Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct hal_serial_config {
  /**
   * @brief Bit rate.
   */
  uint32_t                  speed;
  /* End of the mandatory fields.*/
  /**
   * @brief Initialization value for the Line Control Register.
   */
  uint32_t                  lcr;
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
  /* End of the mandatory fields.*/                                         \
  /* Pointer to the UART registers block.*/                                 \
  aducm_uart_t              *uart;                                          \
  /* Clock frequency for the associated UART.*/                             \
  uint32_t                  clock;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if ADUCM_SERIAL_USE_UART0 && !defined(__DOXYGEN__)
extern SerialDriver SD0;
#endif
#if ADUCM_SERIAL_USE_UART1 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
  void sd_lld_serve_interrupt(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL == TRUE */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
