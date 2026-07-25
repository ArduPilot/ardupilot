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
 * @file    USARTv2/hal_sio_lld.h
 * @brief   STM32 SIO subsystem low level driver header.
 *
 * @addtogroup SIO
 * @{
 */

#ifndef HAL_SIO_LLD_H
#define HAL_SIO_LLD_H

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

#include "stm32_usart.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Mask of RX-related errors in the ISR register.
 */
#define SIO_LLD_ISR_RX_ERRORS           (USART_ISR_NE   | USART_ISR_FE   |  \
                                         USART_ISR_PE   | USART_ISR_ORE  |  \
                                         USART_ISR_LBDF)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    STM32 configuration options
 * @{
 */
/**
 * @brief   SIO driver 1 enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_USART1) || defined(__DOXYGEN__)
#define STM32_SIO_USE_USART1                FALSE
#endif

/**
 * @brief   SIO driver 2 enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_USART2) || defined(__DOXYGEN__)
#define STM32_SIO_USE_USART2                FALSE
#endif

/**
 * @brief   SIO driver 3 enable switch.
 * @details If set to @p TRUE the support for USART3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_USART3) || defined(__DOXYGEN__)
#define STM32_SIO_USE_USART3                FALSE
#endif

/**
 * @brief   SIO driver 4 enable switch.
 * @details If set to @p TRUE the support for UART4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_UART4) || defined(__DOXYGEN__)
#define STM32_SIO_USE_UART4                 FALSE
#endif

/**
 * @brief   SIO driver 5 enable switch.
 * @details If set to @p TRUE the support for UART5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_UART5) || defined(__DOXYGEN__)
#define STM32_SIO_USE_UART5                 FALSE
#endif

/**
 * @brief   SIO driver 6 enable switch.
 * @details If set to @p TRUE the support for USART6 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_USART6) || defined(__DOXYGEN__)
#define STM32_SIO_USE_USART6                FALSE
#endif

/**
 * @brief   SIO driver 7 enable switch.
 * @details If set to @p TRUE the support for UART7 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_UART7) || defined(__DOXYGEN__)
#define STM32_SIO_USE_UART7                 FALSE
#endif

/**
 * @brief   SIO driver 8 enable switch.
 * @details If set to @p TRUE the support for UART8 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_UART8) || defined(__DOXYGEN__)
#define STM32_SIO_USE_UART8                 FALSE
#endif

/**
 * @brief   LP SIO driver 1 enable switch.
 * @details If set to @p TRUE the support for LPUART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SIO_USE_LPUART1) || defined(__DOXYGEN__)
#define STM32_SIO_USE_LPUART1               FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_SIO_USE_USART1 && !STM32_HAS_USART1
#error "USART1 not present in the selected device"
#endif

#if STM32_SIO_USE_USART2 && !STM32_HAS_USART2
#error "USART2 not present in the selected device"
#endif

#if STM32_SIO_USE_USART3 && !STM32_HAS_USART3
#error "USART3 not present in the selected device"
#endif

#if STM32_SIO_USE_UART4 && !STM32_HAS_UART4
#error "UART4 not present in the selected device"
#endif

#if STM32_SIO_USE_UART5 && !STM32_HAS_UART5
#error "UART5 not present in the selected device"
#endif

#if STM32_SIO_USE_USART6 && !STM32_HAS_USART6
#error "USART6 not present in the selected device"
#endif

#if STM32_SIO_USE_UART7 && !STM32_HAS_UART7
#error "UART7 not present in the selected device"
#endif

#if STM32_SIO_USE_UART8 && !STM32_HAS_UART8
#error "UART8 not present in the selected device"
#endif

#if STM32_SIO_USE_LPUART1 && !STM32_HAS_LPUART1
#error "LPUART1 not present in the selected device"
#endif

#if !STM32_SIO_USE_USART1 && !STM32_SIO_USE_USART2 &&                       \
    !STM32_SIO_USE_USART3 && !STM32_SIO_USE_UART4  &&                       \
    !STM32_SIO_USE_UART5  && !STM32_SIO_USE_USART6 &&                       \
    !STM32_SIO_USE_UART7  && !STM32_SIO_USE_UART8  &&                       \
    !STM32_SIO_USE_LPUART1
#error "SIO driver activated but no USART/UART peripheral assigned"
#endif

/* Checks on allocation of USARTx units.*/
#if STM32_SIO_USE_USART1
#if defined(STM32_USART1_IS_USED)
#error "SIOD1 requires USART1 but it is already used"
#else
#define STM32_USART1_IS_USED
#endif
#endif

#if STM32_SIO_USE_USART2
#if defined(STM32_USART2_IS_USED)
#error "SIOD2 requires USART2 but it is already used"
#else
#define STM32_USART2_IS_USED
#endif
#endif

#if STM32_SIO_USE_USART3
#if defined(STM32_USART3_IS_USED)
#error "SIOD3 requires USART3 but it is already used"
#else
#define STM32_USART3_IS_USED
#endif
#endif

#if STM32_SIO_USE_UART4
#if defined(STM32_UART4_IS_USED)
#error "SIOD4 requires UART4 but it is already used"
#else
#define STM32_UART4_IS_USED
#endif
#endif

#if STM32_SIO_USE_UART5
#if defined(STM32_UART5_IS_USED)
#error "SIOD5 requires UART5 but it is already used"
#else
#define STM32_UART5_IS_USED
#endif
#endif

#if STM32_SIO_USE_USART6
#if defined(STM32_USART6_IS_USED)
#error "SIOD6 requires USART6 but it is already used"
#else
#define STM32_USART6_IS_USED
#endif
#endif

#if STM32_SIO_USE_UART7
#if defined(STM32_UART7_IS_USED)
#error "SIOD7 requires UART7 but it is already used"
#else
#define STM32_UART7_IS_USED
#endif
#endif

#if STM32_SIO_USE_UART8
#if defined(STM32_UART8_IS_USED)
#error "SIOD8 requires UART8 but it is already used"
#else
#define STM32_UART8_IS_USED
#endif
#endif

#if STM32_SIO_USE_LPUART1
#if defined(STM32_LPUART1_IS_USED)
#error "LPSIOD1 requires LPUART1 but it is already used"
#else
#define STM32_LPUART1_IS_USED
#endif
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the SIO driver structure.
 */
#define sio_lld_driver_fields                                               \
  /* Pointer to the USARTx registers block.*/                               \
  USART_TypeDef             *usart;                                         \
  /* Clock frequency for the associated USART/UART.*/                       \
  uint32_t                  clock;

/**
 * @brief   Low level fields of the SIO configuration structure.
 */
#define sio_lld_config_fields                                               \
  /* Desired baud rate.*/                                                   \
  uint32_t                  baud;                                           \
  /* USART PRESC register initialization data.*/                            \
  uint32_t                  presc;                                          \
  /* USART CR1 register initialization data.*/                              \
  uint32_t                  cr1;                                            \
  /* USART CR2 register initialization data.*/                              \
  uint32_t                  cr2;                                            \
  /* USART CR3 register initialization data.*/                              \
  uint32_t                  cr3;

/**
 * @brief   Determines the state of the RX FIFO.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX FIFO state.
 * @retval false        if RX FIFO is not empty
 * @retval true         if RX FIFO is empty
 *
 * @notapi
 */
#define sio_lld_is_rx_empty(siop)                                           \
  (bool)(((siop)->usart->ISR & USART_ISR_RXNE) == 0U)

/**
 * @brief   Determines the activity state of the receiver.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX activity state.
 * @retval false        if RX is in active state.
 * @retval true         if RX is in idle state.
 *
 * @notapi
 */
#define sio_lld_is_rx_idle(siop)                                            \
  (bool)(((siop)->usart->ISR & USART_ISR_IDLE) != 0U)

/**
 * @brief   Determines if RX has pending error events to be read and cleared.
 * @note    Only error and protocol errors are handled, data events are not
 *          considered.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX error events.
 * @retval false        if RX has no pending events
 * @retval true         if RX has pending events
 *
 * @notapi
 */
#define sio_lld_has_rx_errors(siop)                                         \
  (bool)(((siop)->usart->ISR & SIO_LLD_ISR_RX_ERRORS) != 0U)

/**
 * @brief   Determines the state of the TX FIFO.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The TX FIFO state.
 * @retval false        if TX FIFO is not full
 * @retval true         if TX FIFO is full
 *
 * @notapi
 */
#define sio_lld_is_tx_full(siop)                                            \
  (bool)(((siop)->usart->ISR & USART_ISR_TXE) == 0U)

/**
 * @brief   Determines the transmission state.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The TX FIFO state.
 * @retval false        if transmission is idle
 * @retval true         if transmission is ongoing
 *
 * @notapi
 */
#define sio_lld_is_tx_ongoing(siop)                                         \
  (bool)(((siop)->usart->ISR & USART_ISR_TC) == 0U)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (STM32_SIO_USE_USART1 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD1;
#endif

#if (STM32_SIO_USE_USART2 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD2;
#endif

#if (STM32_SIO_USE_USART3 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD3;
#endif

#if (STM32_SIO_USE_UART4 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD4;
#endif

#if (STM32_SIO_USE_UART5 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD5;
#endif

#if (STM32_SIO_USE_USART6 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD6;
#endif

#if (STM32_SIO_USE_UART7 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD7;
#endif

#if (STM32_SIO_USE_UART8 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD8;
#endif

#if (STM32_SIO_USE_LPUART1 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver LPSIOD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sio_lld_init(void);
  msg_t  sio_lld_start(SIODriver *siop);
  void sio_lld_stop(SIODriver *siop);
  void sio_lld_update_enable_flags(SIODriver *siop);
  sioevents_t sio_lld_get_and_clear_errors(SIODriver *siop);
  sioevents_t sio_lld_get_and_clear_events(SIODriver *siop);
  sioevents_t sio_lld_get_events(SIODriver *siop);
  size_t sio_lld_read(SIODriver *siop, uint8_t *buffer, size_t n);
  size_t sio_lld_write(SIODriver *siop, const uint8_t *buffer, size_t n);
  msg_t sio_lld_get(SIODriver *siop);
  void sio_lld_put(SIODriver *siop, uint_fast16_t data);
  msg_t sio_lld_control(SIODriver *siop, unsigned int operation, void *arg);
  void sio_lld_serve_interrupt(SIODriver *siop);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SIO == TRUE */

#endif /* HAL_SIO_LLD_H */

/** @} */
