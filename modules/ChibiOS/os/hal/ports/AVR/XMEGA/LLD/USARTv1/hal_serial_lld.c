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
 * @file    hal_serial_lld.c
 * @brief   AVR serial subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/**
 * @brief USARTC0 serial driver identifier.
 */
#if (AVR_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/**
 * @brief USARTC1 serial driver identifier.
 */
#if (AVR_SERIAL_USE_USART2 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/**
 * @brief USARTD0 serial driver identifier.
 */
#if (AVR_SERIAL_USE_USART3 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

/**
 * @brief USARTD1 serial driver identifier.
 */
#if (AVR_SERIAL_USE_USART4 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD4;
#endif

/**
 * @brief USARTE0 serial driver identifier.
 */
#if (AVR_SERIAL_USE_USART5 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD5;
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  38400,                      /* Baud rate.                                 */
  false,                      /* Normal speed at default.                   */
  false,                      /* Disable the MPCM at default.               */
  false,                      /* Transmit 8 bits mode at default.           */
  SERIAL_CMODE_ASYNCHRONOUS,  /* Asynchronous communication mode at default.*/
  SERIAL_PMODE_DISABLE,       /* No parity at default.                      */
  SERIAL_SBMODE_1BIT,         /* One stop bit at default.                   */
  SERIAL_CHSIZE_8BIT,         /* 8 bits data frame at default.              */
};

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/**
 * @brief   Configure the multiprocessor communication mode.
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_cfg_mpcm(SerialDriver *sdp, const SerialConfig *config) {

  if (config->mpcm)
    sdp->usart->CTRLB |= (USART_MPCM_bm);
  else
    sdp->usart->CTRLB &= ~(USART_MPCM_bm);
}

/**
 * @brief   Configure the double speed.
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_cfg_clk2x(SerialDriver *sdp, const SerialConfig *config) {

  if (config->clk2x)
    sdp->usart->CTRLB |= (USART_CLK2X_bm);
  else
    sdp->usart->CTRLB &= ~(USART_CLK2X_bm);
}

/**
 * @brief   Configuration of transmission mode (8/9 bits).
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_cfg_txb8(SerialDriver *sdp, const SerialConfig *config) {

  if (config->txb8)
    sdp->usart->CTRLB |= (USART_TXB8_bm);
  else
    sdp->usart->CTRLB &= ~(USART_TXB8_bm);
}

/**
 * @brief   Configuration of communication mode.
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_cfg_cmode(SerialDriver *sdp, const SerialConfig *config) {

  if (config->cmode == SERIAL_CMODE_SYNCHRONOUS) {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_CMODE_gm) | \
                        (USART_CMODE_SYNCHRONOUS_gc);
  }
  if (config->cmode == SERIAL_CMODE_ASYNCHRONOUS) {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_CMODE_gm) | \
                        (USART_CMODE_ASYNCHRONOUS_gc);
  }
}

/**
 * @brief   Configuration of the number of stop to use during transmission.
 * @details @true set 2 stop bit and @false set 1 stop bit.
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_cfg_sbmode(SerialDriver *sdp, const SerialConfig *config) {

  if (config->sbmode) {
    sdp->usart->CTRLC |= USART_SBMODE_bm;
  }
  else {
    sdp->usart->CTRLC &= ~USART_SBMODE_bm;
  }
}

/**
 * @brief   Configuration of parity mode.
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_cfg_pmode(SerialDriver *sdp, const SerialConfig *config) {

  if (config->pmode == SERIAL_PMODE_EVEN) {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_PMODE_gm) | \
                        (USART_PMODE_EVEN_gc);
  }
  else if (config->chsize == SERIAL_PMODE_ODD) {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_PMODE_gm) | \
                        (USART_PMODE_ODD_gc);
  }
  else {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_PMODE_gm) | \
                        (USART_PMODE_DISABLED_gc);
  }
}

/**
 * @brief   Configuration of caracter size.
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_cfg_chsize(SerialDriver *sdp, const SerialConfig *config) {

  if (config->chsize == SERIAL_CHSIZE_5BIT) {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                          (USART_CHSIZE_5BIT_gc);
  }
  else if (config->chsize == SERIAL_CHSIZE_6BIT) {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                        (USART_CHSIZE_6BIT_gc);
  }
  else if (config->chsize == SERIAL_CHSIZE_7BIT) {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                        (USART_CHSIZE_7BIT_gc);
  }
  else if (config->chsize == SERIAL_CHSIZE_8BIT) {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                        (USART_CHSIZE_8BIT_gc);
  }
  else {
    sdp->usart->CTRLC = (sdp->usart->CTRLC & ~USART_CHSIZE_gm) | \
                        (USART_CHSIZE_9BIT_gc);
  }
}

/**
 * @brief   Configuration of the baud rate.
 * @note    BSCALE is set to 0 for the moment.
 * @TODO    Support all the BSCALE value
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_cfg_baudrate(SerialDriver *sdp, const SerialConfig *config) {

  /* BSCALE = 0. */
  #define BSCALE 0
  uint16_t br = get_bsel(config->speed);
  sdp->usart->BAUDCTRLA = (uint8_t)br;
  sdp->usart->BAUDCTRLB = (BSCALE << USART_BSCALE0_bp) | (br >> 8);
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 */
static void usart_stop(SerialDriver *sdp) {

  sdp->usart->CTRLB &= ~(USART_RXEN_bm);
  sdp->usart->CTRLB &= ~(USART_TXEN_bm);
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp     pointer to the @p Serial Driver object
 * @param[in] config  the architecture-dependent serial driver configuration
 */
static void usart_start(SerialDriver *sdp, const SerialConfig *config) {

  usart_stop(sdp);

  /* Resetting eventual pending status flags. */

  /* Starting the receiver idle loop. */
  /* Uart_enter_rx_idle_loop(uartp);  */

  usart_cfg_mpcm(sdp, config);
  usart_cfg_clk2x(sdp, config);
  usart_cfg_txb8(sdp, config);
  usart_cfg_cmode(sdp, config);
  usart_cfg_sbmode(sdp, config);
  usart_cfg_pmode(sdp, config);
  usart_cfg_chsize(sdp, config);
  usart_cfg_baudrate(sdp, config);
  sdp->usart->CTRLB |= (USART_RXEN_bm);
  sdp->usart->CTRLB |= (USART_TXEN_bm);
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void usart_init(SerialDriver *sdp, const SerialConfig *config) {

  USART_t *u = sdp->usart;

  usart_start(sdp, config);
  u->CTRLA = (u->CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
  u->CTRLA = (u->CTRLA & ~USART_TXCINTLVL_gm) | USART_TXCINTLVL_LO_gc;
  u->CTRLA = (u->CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
  PMIC.CTRL |= PMIC_LOLVLEX_bm;
  sei();
  u->CTRLB |= (USART_RXEN_bm);
  u->CTRLB |= (USART_TXEN_bm);
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an USART I/O block
 */
static void usart_deinit(USART_t *u) {

  u->CTRLB &= ~(USART_RXEN_bm);
  u->CTRLB &= ~(USART_TXEN_bm);
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        USART SR register value
 */
static void set_error(SerialDriver *sdp, uint8_t sr) {

  eventflags_t sts = 0;

  if (sr & USART_BUFOVF_bm)
    sts |= SD_OVERRUN_ERROR;
  if (sr & USART_PERR_bm)
    sts |= SD_PARITY_ERROR;
  if (sr & USART_FERR_bm)
    sts |= SD_FRAMING_ERROR;

  chnAddFlagsI(sdp, sts);
}

#if AVR_SERIAL_USE_USART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  USARTC0.CTRLA &= ~USART_DREINTLVL_gm;
  USARTC0.CTRLA |= USART_DREINTLVL_MED_gc;
}
#endif
#if AVR_SERIAL_USE_USART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  USARTC1.CTRLA &= ~USART_DREINTLVL_gm;
  USARTC1.CTRLA |= USART_DREINTLVL_MED_gc;
}
#endif
#if AVR_SERIAL_USE_USART3 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  USARTD0.CTRLA &= ~USART_DREINTLVL_gm;
  USARTD0.CTRLA |= USART_DREINTLVL_MED_gc;
}
#endif
#if AVR_SERIAL_USE_USART4 || defined(__DOXYGEN__)
static void notify4(io_queue_t *qp) {

  (void)qp;
  USARTD1.CTRLA &= ~USART_DREINTLVL_gm;
  USARTD1.CTRLA |= USART_DREINTLVL_MED_gc;
}
#endif
#if AVR_SERIAL_USE_USART5 || defined(__DOXYGEN__)
static void notify5(io_queue_t *qp) {

  (void)qp;
  USARTE0.CTRLA &= ~USART_DREINTLVL_gm;
  USARTE0.CTRLA |= USART_DREINTLVL_MED_gc;
}
#endif

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#if AVR_SERIAL_USE_USART1 || defined(__DOXYGEN__)
/**
 * @brief   USART1 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_TXC_vect) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  USARTC0.CTRLA = (USARTC0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART1 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_DRE_vect) {

  msg_t msg;

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  msg = oqGetI(&SD1.oqueue);
  osalSysUnlockFromISR();

  if (msg < MSG_OK) {
    USARTC0.CTRLA = (USARTC0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
  }
  else {
    USARTC0.DATA = msg;
  }

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART1 RX IRQ handler, reception complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_RXC_vect) {

  uint8_t status;

  OSAL_IRQ_PROLOGUE();

  status = USARTC0.STATUS;

  if (status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) {
    set_error(&SD1, status);
  }

  osalSysLockFromISR();
  sdIncomingDataI(&SD1, USARTC0.DATA);
  osalSysUnlockFromISR();
  OSAL_IRQ_EPILOGUE();
}
#endif /* AVR_UART_USE_USART1 */

#if AVR_SERIAL_USE_USART2 || defined(__DOXYGEN__)
/**
 * @brief   USART1 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_TXC_vect) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  USARTC1.CTRLA = (USARTC1.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART2 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC1_DRE_vect) {

  msg_t msg;

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  msg = oqGetI(&SD2.oqueue);
  osalSysUnlockFromISR();

  if (msg < MSG_OK) {
    USARTC1.CTRLA = (USARTC1.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
  }
  else {
    USARTC1.DATA = msg;
  }

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART2 RX IRQ handler, reception complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC1_RXC_vect) {

  uint8_t status;

  OSAL_IRQ_PROLOGUE();

  status = USARTC1.STATUS;
  if (status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm));
    set_error(&SD2, status);
  osalSysLockFromISR();
  sdIncomingDataI(&SD2, USARTC1.DATA);
  osalSysUnlockFromISR();
  OSAL_IRQ_EPILOGUE();
}
#endif /* AVR_UART_USE_USART2 */

#if AVR_SERIAL_USE_USART3 || defined(__DOXYGEN__)
/**
 * @brief   USART1 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTD0_TXC_vect) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART3 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTD0_DRE_vect) {

  msg_t msg;

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  msg = oqGetI(&SD3.oqueue);
  osalSysUnlockFromISR();

  if (msg < MSG_OK) {
    USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
  }
  else {
    USARTD0.DATA = msg;
  }

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART3 RX IRQ handler, reception complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTD0_RXC_vect) {

  uint8_t status;

  OSAL_IRQ_PROLOGUE();

  status = USARTD0.STATUS;
  if (status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm));
    set_error(&SD3, status);
  osalSysLockFromISR();
  sdIncomingDataI(&SD3, USARTD0.DATA);
  osalSysUnlockFromISR();
  OSAL_IRQ_EPILOGUE();
}
#endif /* AVR_UART_USE_USART3 */

#if AVR_SERIAL_USE_USART4 || defined(__DOXYGEN__)
/**
 * @brief   USART1 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTD1_TXC_vect) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  USARTD1.CTRLA = (USARTD1.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART4 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTD1_DRE_vect) {

  msg_t msg;

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  msg = oqGetI(&SD4.oqueue);
  osalSysUnlockFromISR();

  if (msg < MSG_OK) {
    USARTD1.CTRLA = (USARTD1.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
  }
  else {
    USARTD1.DATA = msg;
  }

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART4 RX IRQ handler, reception complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTD1_RXC_vect) {

  uint8_t status;

  OSAL_IRQ_PROLOGUE();

  status = USARTD1.STATUS;
  if (status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm));
    set_error(&SD4, status);
  osalSysLockFromISR();
  sdIncomingDataI(&SD4, USARTD1.DATA);
  osalSysUnlockFromISR();
  OSAL_IRQ_EPILOGUE();
}
#endif /* AVR_UART_USE_USART4 */

#if AVR_SERIAL_USE_USART5 || defined(__DOXYGEN__)
/**
 * @brief   USART1 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTE0_TXC_vect) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  USARTE0.CTRLA = (USARTE0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART5 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTE0_DRE_vect) {

  msg_t msg;

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  msg = oqGetI(&SD5.oqueue);
  osalSysUnlockFromISR();

  if (msg < MSG_OK) {
    USARTE0.CTRLA = (USARTE0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
  }
  else {
    USARTE0.DATA = msg;
  }

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART5 RX IRQ handler, reception complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTE0_RXC_vect) {

  uint8_t status;

  OSAL_IRQ_PROLOGUE();

  status = USARTE0.STATUS;
  if (status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm));
    set_error(&SD5, status);
  osalSysLockFromISR();
  sdIncomingDataI(&SD5, USARTE0.DATA);
  osalSysUnlockFromISR();
  OSAL_IRQ_EPILOGUE();
}
#endif /* AVR_UART_USE_USART5 */

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if AVR_SERIAL_USE_USART1 == TRUE
  sdObjectInit(&SD1, NULL, notify1);
  SD1.usart = &USARTC0;
#endif
#if AVR_SERIAL_USE_USART2 == TRUE
  sdObjectInit(&SD2, NULL, notify2);
  SD2.usart = &USARTC1;
#endif
#if AVR_SERIAL_USE_USART3 == TRUE
  sdObjectInit(&SD3, NULL, notify3);
  SD3.usart = &USARTD0;
#endif
#if AVR_SERIAL_USE_USART4 == TRUE
  sdObjectInit(&SD4, NULL, notify4);
  SD4.usart = &USARTD1;
#endif
#if AVR_SERIAL_USE_USART5 == TRUE
  sdObjectInit(&SD5, NULL, notify5);
  SD5.usart = &USARTE0;
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL) {
    config = &default_config;
  }

  if (sdp->state == SD_STOP) {
#if AVR_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {

    }
#endif
#if AVR_SERIAL_USE_USART2 == TRUE
    if (&SD2 == sdp) {

    }
#endif
#if AVR_SERIAL_USE_USART3 == TRUE
    if (&SD3 == sdp) {

    }
#endif
#if AVR_SERIAL_USE_USART4 == TRUE
    if (&SD4 == sdp) {

    }
#endif
#if AVR_SERIAL_USE_USART5 == TRUE
    if (&SD5 == sdp) {

    }
#endif
  }
  /* Configures the peripheral. */
  usart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  usart_deinit(sdp->usart);
  if (sdp->state == SD_READY) {
#if AVR_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {

    }
#endif
#if AVR_SERIAL_USE_USART2 == TRUE
    if (&SD2 == sdp) {

    }
#endif
#if AVR_SERIAL_USE_USART3 == TRUE
    if (&SD3 == sdp) {

    }
#endif
#if AVR_SERIAL_USE_USART4 == TRUE
    if (&SD4 == sdp) {

    }
#endif
#if AVR_SERIAL_USE_USART5 == TRUE
    if (&SD5 == sdp) {

    }
#endif
  }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
