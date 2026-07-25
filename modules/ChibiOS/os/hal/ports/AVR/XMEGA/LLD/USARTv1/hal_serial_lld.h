/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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
 * @brief   AVR serial subsystem low level driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/**
 * @brief   USART communication mode enumerations.
 */
typedef enum {
  SERIAL_CMODE_ASYNCHRONOUS  = 0x00, /**< SERIAL asynchronous mode. */
  SERIAL_CMODE_SYNCHRONOUS   = 0x01, /**< SERIAL synchronous mode.  */
} serialcmode_t;

/**
 * @brief   USART parity mode enumerations.
 */
typedef enum {
  SERIAL_PMODE_DISABLE = 0x00, /**< SERIAL use no parity.   */
  SERIAL_PMODE_EVEN    = 0x10, /**< SERIAL use even parity. */
  SERIAL_PMODE_ODD     = 0x11  /**< SERIAL use odd parity.  */
} serialpmode_t;

/**
 * @brief  USART stop bit mode enumerations.
 */
typedef enum {
  SERIAL_SBMODE_1BIT = FALSE,  /**< Serial use 1 stop bit.  */
  SERIAL_SBMODE_2BIT = TRUE    /**< Serial use 2 stop bit.  */
} serialsbmode_t;

/**
 * @brief   character size enumerations.
 */
typedef enum {
  SERIAL_CHSIZE_5BIT = 0x00, /**< Serial use 5 bytes for data.  */
  SERIAL_CHSIZE_6BIT = 0x01, /**< Serial use 6 bytes for data.  */
  SERIAL_CHSIZE_7BIT = 0x02, /**< Serial use 7 bytes for data.  */
  SERIAL_CHSIZE_8BIT = 0x03, /**< Serial use 8 bytes for data.  */
  SERIAL_CHSIZE_9BIT = 0x07  /**< Serial use 9 bytes for data.  */
} serialchsize_t;

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    configuration options
 * @{
 */
/**
 * @brief   USART1 driver enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_SERIAL_USE_USART1) || defined(__DOXYGEN__)
#define AVR_SERIAL_USE_USART1             FALSE
#endif
#if !defined(AVR_SERIAL_USE_USART2) || defined(__DOXYGEN__)
#define AVR_SERIAL_USE_USART2             FALSE
#endif
#if !defined(AVR_SERIAL_USE_USART3) || defined(__DOXYGEN__)
#define AVR_SERIAL_USE_USART3             FALSE
#endif
#if !defined(AVR_SERIAL_USE_USART4) || defined(__DOXYGEN__)
#define AVR_SERIAL_USE_USART4             FALSE
#endif
#if !defined(AVR_SERIAL_USE_USART5) || defined(__DOXYGEN__)
#define AVR_SERIAL_USE_USART5             FALSE
#endif
/** @} */

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
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct hal_serial_config {
  /**
   * @brief Bit rate.
   */
  uint32_t                  speed;
  /**
   * @brief   Double transmission speed.
   */
  bool                      clk2x;
  /**
   * @brief   Multiprocessor communication mode bit.
   */
  bool                      mpcm;
  /**
   * @brief   Transmission bit 8.
   */
  bool                      txb8;
  /**
   * @brief   Communication mode.
   */
  uint8_t                   cmode;
  /**
   * @brief   Parity mode.
   */
  uint8_t                   pmode;
  /**
   * @brief   Stop bit mode.
   */
  bool                      sbmode;
  /**
   * @brief   Caractere size.
   */
  uint8_t                   chsize;
  /* End of the mandatory fields. */
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
  /* End of the mandatory fields. */                                        \
  /* Pointer to the USART registers block. */                               \
  USART_t                   *usart;

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/**
 * @brief   This is a macro function to calcul BSEL value according to
 *          the baudrate selected by the user.
 *
 * @param[in] baud  the baudrate to be configure
 */
#define get_bsel(baud) (F_CPU/(16*baud))-1

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if (AVR_SERIAL_USE_USART1 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if (AVR_SERIAL_USE_USART2 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif
#if (AVR_SERIAL_USE_USART3 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif
#if (AVR_SERIAL_USE_USART4 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD4;
#endif
#if (AVR_SERIAL_USE_USART5 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD5;
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

#endif /* HAL_USE_SERIAL == TRUE */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
