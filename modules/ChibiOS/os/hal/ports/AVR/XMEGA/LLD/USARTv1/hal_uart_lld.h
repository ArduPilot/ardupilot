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
 * @file    USARTv1/hal_uart_lld.h
 * @brief   AVR UART subsystem low level driver header.
 *
 * @addtogroup UART
 * @{
 */

#ifndef HAL_UART_LLD_H
#define HAL_UART_LLD_H

#if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/**
 * @brief   USART communication mode enumerations.
 */
typedef enum {
  USART_CMODE_ASYNCHRONOUS  = 0x00, /**< USART asynchronous mode. */
  USART_CMODE_SYNCHRONOUS   = 0x01, /**< USART synchronous mode.  */
  USART_CMODE_IRCOM         = 0x10, /**< USART IRCOM mode.        */
  USART_CMODE_MSPI          = 0x11, /**< USART MSPI mode.         */
} usartcmode_t;

/**
 * @brief   USART parity mode enumerations.
 */
typedef enum {
  USART_PMODE_DISABLE = 0x00, /**< USART use no parity.   */
  USART_PMODE_EVEN    = 0x10, /**< USART use even parity. */
  USART_PMODE_ODD     = 0x11  /**< USART use odd parity.  */
} usartpmode_t;

/**
 * @brief  USART stop bit mode enumerations.
 */
typedef enum {
  USART_SBMODE_1BIT = FALSE,  /**< USART use 1 stop bit.  */
  USART_SBMODE_2BIT = TRUE    /**< USART use 2 stop bit.  */
} usartsbmode_t;

/**
 * @brief   character size enumerations.
 */
typedef enum {
  USART_CHSIZE_5BIT = 0x00, /**< USART use 5 bytes for data.  */
  USART_CHSIZE_6BIT = 0x01, /**< USART use 6 bytes for data.  */
  USART_CHSIZE_7BIT = 0x02, /**< USART use 7 bytes for data.  */
  USART_CHSIZE_8BIT = 0x03, /**< USART use 8 bytes for data.  */
  USART_CHSIZE_9BIT = 0x07  /**< USART use 9 bytes for data.  */
} usartchsize_t;

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    PLATFORM configuration options
 * @{
 */

/**
 * @brief   UART driver on USART1 (USARTC0) enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_UART_USE_USART1) || defined(__DOXYGEN__)
#define AVR_UART_USE_USART1                   FALSE
#endif

/**
 * @brief   UART driver on USART2 (USARTC1) enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_UART_USE_USART2) || defined(__DOXYGEN__)
#define AVR_UART_USE_UART1                    FALSE
#endif

/**
 * @brief   UART driver on USART3 (USARTD0) enable switch.
 * @details If set to @p TRUE the support for USART3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_UART_USE_USART3) || defined(__DOXYGEN__)
#define AVR_UART_USE_USART3                   FALSE
#endif

/**
 * @brief   UART driver on USART4 (USARTD1) enable switch.
 * @details If set to @p TRUE the support for USART4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_UART_USE_USART4) || defined(__DOXYGEN__)
#define AVR_UART_USE_USART4                   FALSE
#endif

/**
 * @brief   UART driver on USARTE0 enable switch.
 * @details If set to @p TRUE the support for USARTE0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_UART_USE_USART5) || defined(__DOXYGEN__)
#define AVR_UART_USE_USART5                   FALSE
#endif

/**
 * @brief   USART1 (USARTC0) interrupt priority level setting.
 */
#if !defined(AVR_UART_USART1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART1_IRQ_PRIORITY          12
#endif

/**
 * @brief   USART2 (USARTC1) interrupt priority level setting.
 */
#if !defined(AVR_UART_USART2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART2_IRQ_PRIORITY          12
#endif

/**
 * @brief   USART3 (USARTD0) interrupt priority level setting.
 */
#if !defined(AVR_UART_USART3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART3_IRQ_PRIORITY          12
#endif

/**
 * @brief   USART4 (USARTD1) interrupt priority level setting.
 */
#if !defined(AVR_UART_USART4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART4_IRQ_PRIORITY          12
#endif

/**
 * @brief   USART5 (USARTE0) interrupt priority level setting.
 */
#if !defined(AVR_UART_USART5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART5_IRQ_PRIORITY          12
#endif

/**
 * @brief   USART1 (USARTC0) DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(AVR_UART_USART1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART1_DMA_PRIORITY      0
#endif

/**
 * @brief   USART2 (USARTC1) DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(AVR_UART_USART2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART2_DMA_PRIORITY      0
#endif

/**
 * @brief   USART3 (USARTD0) DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(AVR_UART_USART3_DMA_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART3_DMA_PRIORITY      0
#endif

/**
 * @brief   USART4 (USARTD1) DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(AVR_UART_USART4_DMA_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART4_DMA_PRIORITY      0
#endif

/**
 * @brief   USART5 (USARTE0) DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(AVR_UART_USART5_DMA_PRIORITY) || defined(__DOXYGEN__)
#define AVR_UART_USART5_DMA_PRIORITY      0
#endif
/** @} */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

#if !AVR_UART_USE_USART1 && !AVR_UART_USE_USART2 &&   \
    !AVR_UART_USE_USART3 && !AVR_UART_USE_USART4 &&   \
    !AVR_UART_USE_USART5
#error "UART driver activated but no USART peripheral assigned"
#endif

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   UART driver condition flags type.
 */
typedef uint8_t uartflags_t;

/**
 * @brief   Type of an UART driver.
 */
typedef struct hal_uart_driver UARTDriver;

/**
 * @brief   Generic UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
typedef void (*uartcb_t)(UARTDriver *uartp);

/**
 * @brief   Character received UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object triggering the
 *                      callback
 * @param[in] c         received character
 */
typedef void (*uartccb_t)(UARTDriver *uartp, uint16_t c);

/**
 * @brief   Receive error UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object triggering the
 *                      callback
 * @param[in] e         receive error mask
 */
typedef void (*uartecb_t)(UARTDriver *uartp, uartflags_t e);

/**
 * @brief   Type of an UART configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct hal_uart_config {
  /**
   * @brief End of transmission buffer callback.
   */
  uartcb_t                  txend1_cb;
  /**
   * @brief Physical end of transmission callback.
   */
  uartcb_t                  txend2_cb;
  /**
   * @brief Receive buffer filled callback.
   */
  uartcb_t                  rxend_cb;
  /**
   * @brief Character received while out if the @p UART_RECEIVE state.
   */
  uartccb_t                 rxchar_cb;
  /**
   * @brief Receive error callback.
   */
  uartecb_t                 rxerr_cb;
  /* End of tihe mandatory fields. */
  /**
   * @brief   Bit rate.
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
} UARTConfig;

/**
 * @brief   Structure representing an UART driver.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
struct hal_uart_driver {
  /**
   * @brief Driver state.
   */
  uartstate_t               state;
  /**
   * @brief Transmitter state.
   */
  uarttxstate_t             txstate;
  /**
   * @brief Receiver state.
   */
  uartrxstate_t             rxstate;
  /**
   * @brief Current configuration data.
   */
  const UARTConfig          *config;
#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Synchronization flag for transmit operations.
   */
  bool                      early;
  /**
   * @brief   Waiting thread on RX.
   */
  thread_reference_t        threadrx;
  /**
   * @brief   Waiting thread on TX.
   */
  thread_reference_t        threadtx;
#endif /* UART_USE_WAIT */
#if (UART_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the peripheral.
   */
  mutex_t                   mutex;
#endif /* UART_USE_MUTUAL_EXCLUSION */
#if defined(UART_DRIVER_EXT_FIELDS)
  UART_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields. */
  /**
   * @brief   Pointer to the USART registers block.
   */
  USART_t                   *usart;
  /**
   * @brief   Clock frequency for the associated USART/UART.
   */
  uint32_t                  clock;
  /**
   * @brief   DMA mode bit mask.
   */
  uint32_t                  dmamode;
  /**
   * @brief   Receive DMA channel.
   */
  const DMA_t               *dmarx;
  /**
   * @brief   Transmit DMA channel.
   */
  const DMA_t               *dmatx;
  /**
   * @brief   Default receive buffer while into @p UART_RX_IDLE state.
   */
  volatile uint16_t         rxbuf;
};

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

#if (AVR_UART_USE_USART1 == TRUE) && !defined(__DOXYGEN__)
extern UARTDriver USART1D;
#endif

#if (AVR_UART_USE_USART2 == TRUE) && !defined(__DOXYGEN__)
extern UARTDriver USART2D;
#endif

#if (AVR_UART_USE_USART3 == TRUE) && !defined(__DOXYGEN__)
extern UARTDriver USART3D;
#endif

#if (AVR_UART_USE_USART4 == TRUE) && !defined(__DOXYGEN__)
extern UARTDriver USART4D;
#endif

#if (AVR_UART_USE_USART5 == TRUE) && !defined(__DOXYGEN__)
extern UARTDriver USART5D;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void uart_lld_init(void);
  void uart_lld_start(UARTDriver *uartp);
  void uart_lld_stop(UARTDriver *uartp);
  void uart_lld_start_send(UARTDriver *uartp, size_t n, const uint8_t *txbuf);
  size_t uart_lld_stop_send(UARTDriver *uartp);
  void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf);
  size_t uart_lld_stop_receive(UARTDriver *uartp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_UART == TRUE */

#endif /* HAL_UART_LLD_H */

/** @} */
