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
 * @file    UARTv1/hal_sio_lld.h
 * @brief   RP SIO subsystem low level driver header.
 *
 * @addtogroup SIO
 * @{
 */

#ifndef HAL_SIO_LLD_H
#define HAL_SIO_LLD_H

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Mask of RX-related errors in the MIS/RIS registers.
 */
#define SIO_LLD_ISR_RX_ERRORS           (UART_UARTMIS_OEMIS |               \
                                         UART_UARTMIS_BEMIS |               \
                                         UART_UARTMIS_PEMIS |               \
                                         UART_UARTMIS_FEMIS)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    RP configuration options
 * @{
 */
/**
 * @brief   SIO driver 1 enable switch.
 * @details If set to @p TRUE the support for UART0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(RP_SIO_USE_UART0) || defined(__DOXYGEN__)
#define RP_SIO_USE_UART0                    FALSE
#endif

/**
 * @brief   SIO driver 2 enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(RP_SIO_USE_UART1) || defined(__DOXYGEN__)
#define RP_SIO_USE_UART1                    FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if RP_SIO_USE_UART0 && !RP_HAS_UART0
#error "UART0 not present in the selected device"
#endif

#if RP_SIO_USE_UART1 && !RP_HAS_UART1
#error "UART1 not present in the selected device"
#endif

#if !RP_SIO_USE_UART0 && !RP_SIO_USE_UART1
#error "SIO driver activated but no UART peripheral assigned"
#endif

/* Checks on allocation of USARTx units.*/
#if RP_SIO_USE_UART0
#if defined(RP_UART0_IS_USED)
#error "SIOD0 requires UART0 but it is already used"
#else
#define RP_UART0_IS_USED
#endif
#endif

#if RP_SIO_USE_UART1
#if defined(RP_UART1_IS_USED)
#error "SIOD1 requires UART1 but it is already used"
#else
#define RP_UART1_IS_USED
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
  UART_TypeDef             *uart;

/**
 * @brief   Low level fields of the SIO configuration structure.
 */
#define sio_lld_config_fields                                               \
  /* Desired baud rate.*/                                                   \
  uint32_t                  baud;                                           \
  /* Low level registers settings.*/                                        \
  uint32_t                  UARTLCR_H;                                      \
  uint32_t                  UARTCR;                                         \
  uint32_t                  UARTIFLS;                                       \
  uint32_t                  UARTDMACR;

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
  (bool)(((siop)->uart->UARTFR & UART_UARTFR_RXFE) != 0U)

/**
 * @brief   Determines the activity state of the receiver.
 *(bool)(((siop)->uart->UARTFR & UART_UARTFR_RXFE) != 0U)
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX activity state.
 * @retval false        if RX is in active state.
 * @retval true         if RX is in idle state.
 *
 * @notapi
 */
#define sio_lld_is_rx_idle(siop)                                            \
  (bool)(((siop)->uart->UARTRIS & UART_UARTRIS_RTRIS) != 0U)

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
  (bool)(((siop)->uart->UARTMIS & SIO_LLD_ISR_RX_ERRORS) != 0U)

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
  (bool)(((siop)->uart->UARTFR & UART_UARTFR_TXFF) != 0U)

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
  (bool)(((siop)->uart->UARTFR & UART_UARTFR_BUSY) != 0U)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (RP_SIO_USE_UART0 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD0;
#endif

#if (RP_SIO_USE_UART1 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD1;
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
