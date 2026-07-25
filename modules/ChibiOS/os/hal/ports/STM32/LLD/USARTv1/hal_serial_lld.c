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
 * @file    USARTv1/hal_serial_lld.c
 * @brief   STM32 low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 serial driver identifier.*/
#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief USART2 serial driver identifier.*/
#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

/** @brief UART4 serial driver identifier.*/
#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
SerialDriver SD4;
#endif

/** @brief UART5 serial driver identifier.*/
#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
SerialDriver SD5;
#endif

/** @brief USART6 serial driver identifier.*/
#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
SerialDriver SD6;
#endif

/** @brief UART7 serial driver identifier.*/
#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
SerialDriver SD7;
#endif

/** @brief UART8 serial driver identifier.*/
#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
SerialDriver SD8;
#endif

/** @brief UART9 serial driver identifier.*/
#if STM32_SERIAL_USE_UART9 || defined(__DOXYGEN__)
SerialDriver SD9;
#endif

/** @brief UART10 serial driver identifier.*/
#if STM32_SERIAL_USE_UART10 || defined(__DOXYGEN__)
SerialDriver SD10;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config =
{
  SERIAL_DEFAULT_BITRATE,
  0,
  USART_CR2_STOP1_BITS,
  0,
  NULL,
  NULL
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void usart_init(SerialDriver *sdp, const SerialConfig *config) {
  uint32_t brr;
  USART_TypeDef *u = sdp->usart;

  brr = (uint32_t)((sdp->clock + config->speed/2) / config->speed);

#if defined(USART_CR1_OVER8)
  /* Correcting BRR value when oversampling by 8 instead of 16.
     Fraction is still 4 bits wide, but only lower 3 bits used.
     Mantissa is doubled, but Fraction is left the same.*/
  if (config->cr1 & USART_CR1_OVER8)
    brr = ((brr & ~7) * 2) | (brr & 7);
#endif

  osalDbgAssert(brr < 0x10000, "invalid BRR value");

  u->BRR = brr;

  /* Note that some bits are enforced.*/
  u->CR2 = config->cr2 | USART_CR2_LBDIE;
  u->CR3 = config->cr3 | USART_CR3_EIE;
  u->CR1 = config->cr1 | USART_CR1_UE | USART_CR1_PEIE |
                         USART_CR1_RXNEIE | USART_CR1_TE |
                         USART_CR1_RE;
  u->SR = 0;
  (void)u->SR;  /* SR reset step 1.*/
  (void)u->DR;  /* SR reset step 2.*/

  /* Deciding mask to be applied on the data register on receive, this is
     required in order to mask out the parity bit.*/
  if ((config->cr1 & (USART_CR1_M | USART_CR1_PCE)) == USART_CR1_PCE) {
    sdp->rxmask = 0x7F;
  }
  else {
    sdp->rxmask = 0xFF;
  }
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an USART I/O block
 */
static void usart_deinit(USART_TypeDef *u) {

  u->CR1 = 0;
  u->CR2 = 0;
  u->CR3 = 0;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        USART SR register value
 */
static void set_error(SerialDriver *sdp, uint16_t sr) {
  eventflags_t sts = 0;

  if (sr & USART_SR_ORE)
    sts |= SD_OVERRUN_ERROR;
  if (sr & USART_SR_PE)
    sts |= SD_PARITY_ERROR;
  if (sr & USART_SR_FE)
    sts |= SD_FRAMING_ERROR;
  if (sr & USART_SR_NE)
    sts |= SD_NOISE_ERROR;
  chnAddFlagsI(sdp, sts);
}

#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  USART1->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  USART2->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  USART3->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
static void notify4(io_queue_t *qp) {

  (void)qp;
  UART4->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
static void notify5(io_queue_t *qp) {

  (void)qp;
  UART5->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
static void notify6(io_queue_t *qp) {

  (void)qp;
  USART6->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
static void notify7(io_queue_t *qp) {

  (void)qp;
  UART7->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
static void notify8(io_queue_t *qp) {

  (void)qp;
  UART8->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_UART9 || defined(__DOXYGEN__)
static void notify9(io_queue_t *qp) {

  (void)qp;
  UART9->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_UART10 || defined(__DOXYGEN__)
static void notify10(io_queue_t *qp) {

  (void)qp;
  UART10->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
#if !defined(STM32_USART1_SUPPRESS_ISR)
#if !defined(STM32_USART1_HANDLER)
#error "STM32_USART1_HANDLER not defined"
#endif
/**
 * @brief   USART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
#if !defined(STM32_USART2_SUPPRESS_ISR)
#if !defined(STM32_USART2_HANDLER)
#error "STM32_USART2_HANDLER not defined"
#endif
/**
 * @brief   USART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
#if !defined(STM32_USART3_SUPPRESS_ISR)
#if !defined(STM32_USART3_HANDLER)
#error "STM32_USART3_HANDLER not defined"
#endif
/**
 * @brief   USART3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
#if !defined(STM32_UART4_SUPPRESS_ISR)
#if !defined(STM32_UART4_HANDLER)
#error "STM32_UART4_HANDLER not defined"
#endif
/**
 * @brief   UART4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD4);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
#if !defined(STM32_UART5_SUPPRESS_ISR)
#if !defined(STM32_UART5_HANDLER)
#error "STM32_UART5_HANDLER not defined"
#endif
/**
 * @brief   UART5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD5);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
#if !defined(STM32_USART6_SUPPRESS_ISR)
#if !defined(STM32_USART6_HANDLER)
#error "STM32_USART6_HANDLER not defined"
#endif
/**
 * @brief   USART6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD6);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
#if !defined(STM32_UART7_SUPPRESS_ISR)
#if !defined(STM32_UART7_HANDLER)
#error "STM32_UART7_HANDLER not defined"
#endif
/**
 * @brief   UART7 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD7);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
#if !defined(STM32_UART8_SUPPRESS_ISR)
#if !defined(STM32_UART8_HANDLER)
#error "STM32_UART8_HANDLER not defined"
#endif
/**
 * @brief   UART8 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART8_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD8);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_UART9 || defined(__DOXYGEN__)
#if !defined(STM32_UART9_SUPPRESS_ISR)
#if !defined(STM32_UART9_HANDLER)
#error "STM32_UART9_HANDLER not defined"
#endif
/**
 * @brief   UART9 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART9_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD9);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_UART10 || defined(__DOXYGEN__)
#if !defined(STM32_UART10_SUPPRESS_ISR)
#if !defined(STM32_UART10_HANDLER)
#error "STM32_UART10_HANDLER not defined"
#endif
/**
 * @brief   UART10 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART10_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD10);

  OSAL_IRQ_EPILOGUE();
}
#endif
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

#if STM32_SERIAL_USE_USART1
  sdObjectInit(&SD1, NULL, notify1);
  SD1.usart = USART1;
  SD1.clock = STM32_PCLK2;
#if !defined(STM32_USART1_SUPPRESS_ISR) && defined(STM32_USART1_NUMBER)
  nvicEnableVector(STM32_USART1_NUMBER, STM32_SERIAL_USART1_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_USART2
  sdObjectInit(&SD2, NULL, notify2);
  SD2.usart = USART2;
  SD2.clock = STM32_PCLK1;
#if !defined(STM32_USART2_SUPPRESS_ISR) && defined(STM32_USART2_NUMBER)
  nvicEnableVector(STM32_USART2_NUMBER, STM32_SERIAL_USART2_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_USART3
  sdObjectInit(&SD3, NULL, notify3);
  SD3.usart = USART3;
  SD3.clock = STM32_PCLK1;
#if !defined(STM32_USART3_SUPPRESS_ISR) && defined(STM32_USART3_NUMBER)
  nvicEnableVector(STM32_USART3_NUMBER, STM32_SERIAL_USART3_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART4
  sdObjectInit(&SD4, NULL, notify4);
  SD4.usart = UART4;
  SD4.clock = STM32_PCLK1;
#if !defined(STM32_UART4_SUPPRESS_ISR) && defined(STM32_UART4_NUMBER)
  nvicEnableVector(STM32_UART4_NUMBER, STM32_SERIAL_UART4_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART5
  sdObjectInit(&SD5, NULL, notify5);
  SD5.usart = UART5;
  SD5.clock = STM32_PCLK1;
#if !defined(STM32_UART5_SUPPRESS_ISR) && defined(STM32_UART5_NUMBER)
  nvicEnableVector(STM32_UART5_NUMBER, STM32_SERIAL_UART5_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_USART6
  sdObjectInit(&SD6, NULL, notify6);
  SD6.usart = USART6;
  SD6.clock = STM32_PCLK2;
#if !defined(STM32_USART6_SUPPRESS_ISR) && defined(STM32_USART6_NUMBER)
  nvicEnableVector(STM32_USART6_NUMBER, STM32_SERIAL_USART6_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART7
  sdObjectInit(&SD7, NULL, notify7);
  SD7.usart = UART7;
  SD7.clock = STM32_PCLK1;
#if !defined(STM32_UART7_SUPPRESS_ISR) && defined(STM32_UART7_NUMBER)
  nvicEnableVector(STM32_UART7_NUMBER, STM32_SERIAL_UART7_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART8
  sdObjectInit(&SD8, NULL, notify8);
  SD8.usart = UART8;
  SD8.clock = STM32_PCLK1;
#if !defined(STM32_UART8_SUPPRESS_ISR) && defined(STM32_UART8_NUMBER)
  nvicEnableVector(STM32_UART8_NUMBER, STM32_SERIAL_UART8_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART9
  sdObjectInit(&SD9, NULL, notify9);
  SD9.usart = UART9;
  SD9.clock = STM32_PCLK2;
#if !defined(STM32_UART9_SUPPRESS_ISR) && defined(STM32_UART9_NUMBER)
  nvicEnableVector(STM32_UART9_NUMBER, STM32_SERIAL_UART9_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART10
  sdObjectInit(&SD10, NULL, notify10);
  SD10.usart = UART10;
  SD10.clock = STM32_PCLK2;
#if !defined(STM32_UART10_SUPPRESS_ISR) && defined(STM32_UART10_NUMBER)
  nvicEnableVector(STM32_UART10_NUMBER, STM32_SERIAL_UART10_PRIORITY);
#endif
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

  sdp->config = config;

  if (sdp->state == SD_STOP) {
#if STM32_SERIAL_USE_USART1
    if (&SD1 == sdp) {
      rccEnableUSART1(true);
    }
#endif
#if STM32_SERIAL_USE_USART2
    if (&SD2 == sdp) {
      rccEnableUSART2(true);
    }
#endif
#if STM32_SERIAL_USE_USART3
    if (&SD3 == sdp) {
      rccEnableUSART3(true);
    }
#endif
#if STM32_SERIAL_USE_UART4
    if (&SD4 == sdp) {
      rccEnableUART4(true);
    }
#endif
#if STM32_SERIAL_USE_UART5
    if (&SD5 == sdp) {
      rccEnableUART5(true);
    }
#endif
#if STM32_SERIAL_USE_USART6
    if (&SD6 == sdp) {
      rccEnableUSART6(true);
    }
#endif
#if STM32_SERIAL_USE_UART7
    if (&SD7 == sdp) {
      rccEnableUART7(true);
    }
#endif
#if STM32_SERIAL_USE_UART8
    if (&SD8 == sdp) {
      rccEnableUART8(true);
    }
#endif
#if STM32_SERIAL_USE_UART9
    if (&SD9 == sdp) {
      rccEnableUART9(true);
    }
#endif
#if STM32_SERIAL_USE_UART10
    if (&SD10 == sdp) {
      rccEnableUART10(true);
    }
#endif
  }
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

  if (sdp->state == SD_READY) {
    usart_deinit(sdp->usart);
#if STM32_SERIAL_USE_USART1
    if (&SD1 == sdp) {
      rccDisableUSART1();
      return;
    }
#endif
#if STM32_SERIAL_USE_USART2
    if (&SD2 == sdp) {
      rccDisableUSART2();
      return;
    }
#endif
#if STM32_SERIAL_USE_USART3
    if (&SD3 == sdp) {
      rccDisableUSART3();
      return;
    }
#endif
#if STM32_SERIAL_USE_UART4
    if (&SD4 == sdp) {
      rccDisableUART4();
      return;
    }
#endif
#if STM32_SERIAL_USE_UART5
    if (&SD5 == sdp) {
      rccDisableUART5();
      return;
    }
#endif
#if STM32_SERIAL_USE_USART6
    if (&SD6 == sdp) {
      rccDisableUSART6();
      return;
    }
#endif
#if STM32_SERIAL_USE_UART7
    if (&SD7 == sdp) {
      rccDisableUART7();
      return;
    }
#endif
#if STM32_SERIAL_USE_UART8
    if (&SD8 == sdp) {
      rccDisableUART8();
      return;
    }
#endif
#if STM32_SERIAL_USE_UART9
    if (&SD9 == sdp) {
      rccDisableUART9();
      return;
    }
#endif
#if STM32_SERIAL_USE_UART10
    if (&SD10 == sdp) {
      rccDisableUART10();
      return;
    }
#endif
  }
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the USART
 */
void sd_lld_serve_interrupt(SerialDriver *sdp) {
  USART_TypeDef *u = sdp->usart;
  uint16_t cr1;
  uint16_t sr = u->SR;

  /* Special case, LIN break detection.*/
  if (sr & USART_SR_LBD) {
    osalSysLockFromISR();
    chnAddFlagsI(sdp, SD_BREAK_DETECTED);
    u->SR = ~USART_SR_LBD;
    osalSysUnlockFromISR();
  }

  /* Data available.*/
  osalSysLockFromISR();
  while (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE |
               USART_SR_PE)) {
    uint8_t b;

    /* Error condition detection.*/
    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE  | USART_SR_PE))
      set_error(sdp, sr);
    b = (uint8_t)u->DR & sdp->rxmask;
    if (sr & USART_SR_RXNE)
      sdIncomingDataI(sdp, b);
    sr = u->SR;
  }
  osalSysUnlockFromISR();

  /* Caching CR1.*/
  cr1 = u->CR1;

  /* Transmission buffer empty.*/
  if ((cr1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
    msg_t b;
    osalSysLockFromISR();
    b = oqGetI(&sdp->oqueue);
    if (b < MSG_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      cr1 &= ~USART_CR1_TXEIE;
    }
    else
      u->DR = b;
    osalSysUnlockFromISR();
  }

  /* Physical transmission end.*/
  if ((cr1 & USART_CR1_TCIE) && (sr & USART_SR_TC)) {
    osalSysLockFromISR();
    if (oqIsEmptyI(&sdp->oqueue)) {
      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
      cr1 &= ~USART_CR1_TCIE;
    }
    osalSysUnlockFromISR();
  }

  /* Writing CR1 once.*/
  u->CR1 = cr1;
  _serial_irq_code(sdp);
}

#endif /* HAL_USE_SERIAL */

/** @} */
