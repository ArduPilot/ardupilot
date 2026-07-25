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
 * @file    LPC214x/hal_serial_lld.c
 * @brief   LPC214x low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if USE_LPC214x_UART0 || defined(__DOXYGEN__)
/** @brief UART0 serial driver identifier.*/
SerialDriver SD1;
#endif

#if USE_LPC214x_UART1 || defined(__DOXYGEN__)
/** @brief UART1 serial driver identifier.*/
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief  Driver default configuration.*/
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  UART *u = sdp->uart;

  uint32_t div = PCLK / (config->sc_speed << 4);
  u->UART_LCR = config->sc_lcr | LCR_DLAB;
  u->UART_DLL = div;
  u->UART_DLM = div >> 8;
  u->UART_LCR = config->sc_lcr;
  u->UART_FCR = FCR_ENABLE | FCR_RXRESET | FCR_TXRESET | config->sc_fcr;
  u->UART_ACR = 0;
  u->UART_FDR = 0x10;
  u->UART_TER = TER_ENABLE;
  u->UART_IER = IER_RBR | IER_STATUS;
}

/**
 * @brief   UART de-initialization.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(UART *u) {

  u->UART_LCR = LCR_DLAB;
  u->UART_DLL = 1;
  u->UART_DLM = 0;
  u->UART_LCR = 0;
  u->UART_FDR = 0x10;
  u->UART_IER = 0;
  u->UART_FCR = FCR_RXRESET | FCR_TXRESET;
  u->UART_ACR = 0;
  u->UART_TER = TER_ENABLE;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] err       UART LSR register value
 */
static void set_error(SerialDriver *sdp, IOREG32 err) {
  eventflags_t sts = 0;

  if (err & LSR_OVERRUN)
    sts |= SD_OVERRUN_ERROR;
  if (err & LSR_PARITY)
    sts |= SD_PARITY_ERROR;
  if (err & LSR_FRAMING)
    sts |= SD_FRAMING_ERROR;
  if (err & LSR_BREAK)
    sts |= SD_BREAK_DETECTED;
  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we dont want
 *          to go through the whole ISR and have another interrupt soon after.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  UART *u = sdp->uart;

  while (TRUE) {
    switch (u->UART_IIR & IIR_SRC_MASK) {
    case IIR_SRC_NONE:
      return;
    case IIR_SRC_ERROR:
      set_error(sdp, u->UART_LSR);
      break;
    case IIR_SRC_TIMEOUT:
    case IIR_SRC_RX:
      osalSysLockFromISR();
      if (iqIsEmptyI(&sdp->iqueue))
        chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
      osalSysUnlockFromISR();
      while (u->UART_LSR & LSR_RBR_FULL) {
        osalSysLockFromISR();
        if (iqPutI(&sdp->iqueue, u->UART_RBR) < MSG_OK)
          chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
        osalSysUnlockFromISR();
      }
      break;
    case IIR_SRC_TX:
      {
        int i = LPC214x_UART_FIFO_PRELOAD;
        do {
          msg_t b;

          osalSysLockFromISR();
          b = oqGetI(&sdp->oqueue);
          osalSysUnlockFromISR();
          if (b < MSG_OK) {
            u->UART_IER &= ~IER_THRE;
            osalSysLockFromISR();
            chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
            osalSysUnlockFromISR();
            break;
          }
          u->UART_THR = b;
        } while (--i);
      }
      break;
    default:
      (void) u->UART_THR;
      (void) u->UART_RBR;
    }
  }
}

/**
 * @brief   Attempts a TX FIFO preload.
 */
static void preload(SerialDriver *sdp) {
  UART *u = sdp->uart;

  if (u->UART_LSR & LSR_THRE) {
    int i = LPC214x_UART_FIFO_PRELOAD;
    do {
      msg_t b = oqGetI(&sdp->oqueue);
      if (b < MSG_OK) {
        chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
        return;
      }
      u->UART_THR = b;
    } while (--i);
  }
  u->UART_IER |= IER_THRE;
}

/**
 * @brief   Driver SD1 output notification.
 */
#if USE_LPC214x_UART0 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  preload(&SD1);
}
#endif

/**
 * @brief   Driver SD2 output notification.
 */
#if USE_LPC214x_UART1 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  preload(&SD2);
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   UART0 IRQ handler.
 *
 * @isr
 */
#if USE_LPC214x_UART0 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(UART0IrqHandler) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD1);
  VICVectAddr = 0;

  CH_IRQ_EPILOGUE();
}
#endif

/**
 * @brief   UART1 IRQ handler.
 *
 * @isr
 */
#if USE_LPC214x_UART1 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(UART1IrqHandler) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD2);
  VICVectAddr = 0;

  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if USE_LPC214x_UART0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = U0Base;
  SetVICVector(UART0IrqHandler, LPC214x_UART0_PRIORITY, SOURCE_UART0);
#endif
#if USE_LPC214x_UART1
  sdObjectInit(&SD2, NULL, notify2);
  SD2.uart = U1Base;
  SetVICVector(UART1IrqHandler, LPC214x_UART1_PRIORITY, SOURCE_UART1);
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

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if USE_LPC214x_UART0
    if (&SD1 == sdp) {
      PCONP = (PCONP & PCALL) | PCUART0;
      VICIntEnable = INTMASK(SOURCE_UART0);
    }
#endif
#if USE_LPC214x_UART1
    if (&SD2 == sdp) {
      PCONP = (PCONP & PCALL) | PCUART1;
      VICIntEnable = INTMASK(SOURCE_UART1);
    }
#endif
  }
  uart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if USE_LPC214x_UART0
    if (&SD1 == sdp) {
      PCONP = (PCONP & PCALL) & ~PCUART0;
      VICIntEnClear = INTMASK(SOURCE_UART0);
      return;
    }
#endif
#if USE_LPC214x_UART1
    if (&SD2 == sdp) {
      PCONP = (PCONP & PCALL) & ~PCUART1;
      VICIntEnClear = INTMASK(SOURCE_UART1);
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
