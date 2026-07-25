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
 * @file    USARTv3/hal_serial_lld.c
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

/* For compatibility for those devices without LIN support in the USARTs.*/
#if !defined(USART_ISR_LBDF)
#define USART_ISR_LBDF                      0
#endif

#if !defined(USART_CR2_LBDIE)
#define USART_CR2_LBDIE                     0
#endif

/* Differences in L4+ headers.*/
#if !defined(USART_CR1_TXEIE)
#define USART_CR1_TXEIE                     USART_CR1_TXEIE_TXFNFIE
#endif

#if !defined(USART_CR1_RXNEIE)
#define USART_CR1_RXNEIE                    USART_CR1_RXNEIE_RXFNEIE
#endif

#if !defined(USART_ISR_TXE)
#define USART_ISR_TXE                       USART_ISR_TXE_TXFNF
#endif

#if !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE                      USART_ISR_RXNE_RXFNE
#endif

/* STM32L0xx/STM32F7xx ST headers difference.*/
#if !defined(USART_ISR_LBDF)
#define USART_ISR_LBDF                      USART_ISR_LBD
#endif

/* Handling differences in frame size bits.*/
#if !defined(USART_CR1_M_0)
#define USART_CR1_M_0                       (1 << 12)
#endif

#if !defined(USART_CR1_M_1)
#define USART_CR1_M_1                       (1 << 28)
#endif

/* Workarounds for those devices where UARTs are USARTs.*/
#if defined(USART4)
#define UART4 USART4
#endif
#if defined(USART5)
#define UART5 USART5
#endif
#if defined(USART7)
#define UART7 USART7
#endif
#if defined(USART8)
#define UART8 USART8
#endif

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

/** @brief USART10 serial driver identifier.*/
#if STM32_SERIAL_USE_USART10 || defined(__DOXYGEN__)
SerialDriver SD10;
#endif

/** @brief LPUART1 serial driver identifier.*/
#if STM32_SERIAL_USE_LPUART1 || defined(__DOXYGEN__)
SerialDriver LPSD1;
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

#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
/** @brief Input buffer for SD1.*/
static uint8_t sd_in_buf1[STM32_SERIAL_USART1_IN_BUF_SIZE];

/** @brief Output buffer for SD1.*/
static uint8_t sd_out_buf1[STM32_SERIAL_USART1_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
/** @brief Input buffer for SD2.*/
static uint8_t sd_in_buf2[STM32_SERIAL_USART2_IN_BUF_SIZE];

/** @brief Output buffer for SD2.*/
static uint8_t sd_out_buf2[STM32_SERIAL_USART2_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
/** @brief Input buffer for SD3.*/
static uint8_t sd_in_buf3[STM32_SERIAL_USART3_IN_BUF_SIZE];

/** @brief Output buffer for SD3.*/
static uint8_t sd_out_buf3[STM32_SERIAL_USART3_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
/** @brief Input buffer for SD4.*/
static uint8_t sd_in_buf4[STM32_SERIAL_UART4_IN_BUF_SIZE];

/** @brief Output buffer for SD4.*/
static uint8_t sd_out_buf4[STM32_SERIAL_UART4_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
/** @brief Input buffer for SD5.*/
static uint8_t sd_in_buf5[STM32_SERIAL_UART5_IN_BUF_SIZE];

/** @brief Output buffer for SD5.*/
static uint8_t sd_out_buf5[STM32_SERIAL_UART5_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
/** @brief Input buffer for SD6.*/
static uint8_t sd_in_buf6[STM32_SERIAL_USART6_IN_BUF_SIZE];

/** @brief Output buffer for SD6.*/
static uint8_t sd_out_buf6[STM32_SERIAL_USART6_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
/** @brief Input buffer for SD7.*/
static uint8_t sd_in_buf7[STM32_SERIAL_UART7_IN_BUF_SIZE];

/** @brief Output buffer for SD7.*/
static uint8_t sd_out_buf7[STM32_SERIAL_UART7_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
/** @brief Input buffer for SD8.*/
static uint8_t sd_in_buf8[STM32_SERIAL_UART8_IN_BUF_SIZE];

/** @brief Output buffer for SD8.*/
static uint8_t sd_out_buf8[STM32_SERIAL_UART8_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_UART9 || defined(__DOXYGEN__)
/** @brief Input buffer for SD9.*/
static uint8_t sd_in_buf9[STM32_SERIAL_UART9_IN_BUF_SIZE];

/** @brief Output buffer for SD9.*/
static uint8_t sd_out_buf9[STM32_SERIAL_UART9_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_USART10 || defined(__DOXYGEN__)
/** @brief Input buffer for SD10.*/
static uint8_t sd_in_buf10[STM32_SERIAL_USART10_IN_BUF_SIZE];

/** @brief Output buffer for SD10.*/
static uint8_t sd_out_buf10[STM32_SERIAL_USART10_OUT_BUF_SIZE];
#endif

#if STM32_SERIAL_USE_LPUART1 || defined(__DOXYGEN__)
/** @brief Input buffer for LPSD1.*/
static uint8_t sd_in_buflp1[STM32_SERIAL_LPUART1_IN_BUF_SIZE];

/** @brief Output buffer for LPSD1.*/
static uint8_t sd_out_buflp1[STM32_SERIAL_LPUART1_OUT_BUF_SIZE];
#endif

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
static void usart_init(SerialDriver *sdp,
                       const SerialConfig *config) {
  uint32_t brr, clock;
  USART_TypeDef *u = sdp->usart;

  sdp->config = config;

  /* Baud rate setting.*/
  clock = sdp->clock;
#if STM32_SERIAL_USE_LPUART1
  if (sdp == &LPSD1) {
    osalDbgAssert((clock >= config->speed * 3U) &&
                  (clock <= config->speed * 4096U),
                  "invalid baud rate vs input clock");

    brr = (uint32_t)(((uint64_t)clock * 256 + config->speed/2) / config->speed);

    osalDbgAssert((brr >= 0x300) && (brr < 0x100000), "invalid BRR value");
  }
  else
#endif
  {
    brr = (uint32_t)((clock + config->speed/2) / config->speed);

    /* Correcting BRR value when oversampling by 8 instead of 16.
       Fraction is still 4 bits wide, but only lower 3 bits used.
       Mantissa is doubled, but Fraction is left the same.*/
    if (config->cr1 & USART_CR1_OVER8)
      brr = ((brr & ~7) * 2) | (brr & 7);

    osalDbgAssert(brr < 0x10000, "invalid BRR value");
  }
  u->BRR = brr;

  /* Note that some bits are enforced.*/
  u->CR2 = config->cr2 | USART_CR2_LBDIE;
  u->CR3 = config->cr3 | USART_CR3_EIE;
  u->CR1 = config->cr1 | USART_CR1_UE | USART_CR1_PEIE |
                         USART_CR1_RXNEIE | USART_CR1_TE |
                         USART_CR1_RE;
  u->ICR = 0xFFFFFFFFU;

  /* Deciding mask to be applied on the data register on receive, this is
     required in order to mask out the parity bit.*/
  if ((config->cr1 & USART_CR1_PCE) != 0U) {
    switch (config->cr1 & (USART_CR1_M_1 | USART_CR1_M_0)) {
    case 0:
      sdp->rxmask = 0x7F;
      break;
    case USART_CR1_M_1:
      sdp->rxmask = 0x3F;
      break;
    default:
      sdp->rxmask = 0xFF;
    }
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
 * @param[in] isr       USART ISR register value
 */
static void set_error(SerialDriver *sdp, uint32_t isr) {
  eventflags_t sts = 0;

  if (isr & USART_ISR_ORE)
    sts |= SD_OVERRUN_ERROR;
  if (isr & USART_ISR_PE)
    sts |= SD_PARITY_ERROR;
  if (isr & USART_ISR_FE)
    sts |= SD_FRAMING_ERROR;
  if (isr & USART_ISR_NE)
    sts |= SD_NOISE_ERROR;
  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
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

#if STM32_SERIAL_USE_USART10 || defined(__DOXYGEN__)
static void notify10(io_queue_t *qp) {

  (void)qp;
  USART10->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
}
#endif

#if STM32_SERIAL_USE_LPUART1 || defined(__DOXYGEN__)
static void notifylp1(io_queue_t *qp) {

  (void)qp;
  LPUART1->CR1 |= USART_CR1_TXEIE | USART_CR1_TCIE;
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

#if STM32_SERIAL_USE_USART10 || defined(__DOXYGEN__)
#if !defined(STM32_USART10_SUPPRESS_ISR)
#if !defined(STM32_USART10_HANDLER)
#error "STM32_USART10_HANDLER not defined"
#endif
/**
 * @brief   USART10 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART10_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&SD10);

  OSAL_IRQ_EPILOGUE();
}
#endif
#endif

#if STM32_SERIAL_USE_LPUART1 || defined(__DOXYGEN__)
#if !defined(STM32_LPUART1_SUPPRESS_ISR)
#if !defined(STM32_LPUART1_HANDLER)
#error "STM32_LPUART1_HANDLER not defined"
#endif
/**
 * @brief   LPUART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_LPUART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  sd_lld_serve_interrupt(&LPSD1);

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
  sdObjectInit(&SD1);
  iqObjectInit(&SD1.iqueue, sd_in_buf1, sizeof sd_in_buf1, NULL, &SD1);
  oqObjectInit(&SD1.oqueue, sd_out_buf1, sizeof sd_out_buf1, notify1, &SD1);
  SD1.usart = USART1;
  SD1.clock = STM32_USART1CLK;
#if !defined(STM32_USART1_SUPPRESS_ISR) && defined(STM32_USART1_NUMBER)
  nvicEnableVector(STM32_USART1_NUMBER, STM32_SERIAL_USART1_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_USART2
  sdObjectInit(&SD2);
  iqObjectInit(&SD2.iqueue, sd_in_buf2, sizeof sd_in_buf2, NULL, &SD2);
  oqObjectInit(&SD2.oqueue, sd_out_buf2, sizeof sd_out_buf2, notify2, &SD2);
  SD2.usart = USART2;
  SD2.clock = STM32_USART2CLK;
#if !defined(STM32_USART2_SUPPRESS_ISR) && defined(STM32_USART2_NUMBER)
  nvicEnableVector(STM32_USART2_NUMBER, STM32_SERIAL_USART2_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_USART3
  sdObjectInit(&SD3);
  iqObjectInit(&SD3.iqueue, sd_in_buf3, sizeof sd_in_buf3, NULL, &SD3);
  oqObjectInit(&SD3.oqueue, sd_out_buf3, sizeof sd_out_buf3, notify3, &SD3);
  SD3.usart = USART3;
  SD3.clock = STM32_USART3CLK;
#if !defined(STM32_USART3_SUPPRESS_ISR) && defined(STM32_USART3_NUMBER)
  nvicEnableVector(STM32_USART3_NUMBER, STM32_SERIAL_USART3_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART4
  sdObjectInit(&SD4);
  iqObjectInit(&SD4.iqueue, sd_in_buf4, sizeof sd_in_buf4, NULL, &SD4);
  oqObjectInit(&SD4.oqueue, sd_out_buf4, sizeof sd_out_buf4, notify4, &SD4);
  SD4.usart = UART4;
  SD4.clock = STM32_UART4CLK;
#if !defined(STM32_UART4_SUPPRESS_ISR) && defined(STM32_UART4_NUMBER)
  nvicEnableVector(STM32_UART4_NUMBER, STM32_SERIAL_UART4_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART5
  sdObjectInit(&SD5);
  iqObjectInit(&SD5.iqueue, sd_in_buf5, sizeof sd_in_buf5, NULL, &SD5);
  oqObjectInit(&SD5.oqueue, sd_out_buf5, sizeof sd_out_buf5, notify5, &SD5);
  SD5.usart = UART5;
  SD5.clock = STM32_UART5CLK;
#if !defined(STM32_UART5_SUPPRESS_ISR) && defined(STM32_UART5_NUMBER)
  nvicEnableVector(STM32_UART5_NUMBER, STM32_SERIAL_UART5_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_USART6
  sdObjectInit(&SD6);
  iqObjectInit(&SD6.iqueue, sd_in_buf6, sizeof sd_in_buf6, NULL, &SD6);
  oqObjectInit(&SD6.oqueue, sd_out_buf6, sizeof sd_out_buf6, notify6, &SD6);
  SD6.usart = USART6;
  SD6.clock = STM32_USART6CLK;
#if !defined(STM32_USART6_SUPPRESS_ISR) && defined(STM32_USART6_NUMBER)
  nvicEnableVector(STM32_USART6_NUMBER, STM32_SERIAL_USART6_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART7
  sdObjectInit(&SD7);
  iqObjectInit(&SD7.iqueue, sd_in_buf7, sizeof sd_in_buf7, NULL, &SD7);
  oqObjectInit(&SD7.oqueue, sd_out_buf7, sizeof sd_out_buf7, notify7, &SD7);
  SD7.usart = UART7;
  SD7.clock = STM32_UART7CLK;
#if !defined(STM32_UART7_SUPPRESS_ISR) && defined(STM32_UART7_NUMBER)
  nvicEnableVector(STM32_UART7_NUMBER, STM32_SERIAL_UART7_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART8
  sdObjectInit(&SD8);
  iqObjectInit(&SD8.iqueue, sd_in_buf8, sizeof sd_in_buf8, NULL, &SD8);
  oqObjectInit(&SD8.oqueue, sd_out_buf8, sizeof sd_out_buf8, notify8, &SD8);
  SD8.usart = UART8;
  SD8.clock = STM32_UART8CLK;
#if !defined(STM32_UART8_SUPPRESS_ISR) && defined(STM32_UART8_NUMBER)
  nvicEnableVector(STM32_UART8_NUMBER, STM32_SERIAL_UART8_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_UART9
  sdObjectInit(&SD9);
  iqObjectInit(&SD9.iqueue, sd_in_buf9, sizeof sd_in_buf9, NULL, &SD9);
  oqObjectInit(&SD9.oqueue, sd_out_buf9, sizeof sd_out_buf9, notify9, &SD9);
  SD9.usart = UART9;
  SD9.clock = STM32_UART9CLK;
#if !defined(STM32_UART9_SUPPRESS_ISR) && defined(STM32_UART9_NUMBER)
  nvicEnableVector(STM32_UART9_NUMBER, STM32_SERIAL_UART9_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_USART10
  sdObjectInit(&SD10);
  iqObjectInit(&SD10.iqueue, sd_in_buf10, sizeof sd_in_buf10, NULL, &SD10);
  oqObjectInit(&SD10.oqueue, sd_out_buf10, sizeof sd_out_buf10, notify10, &SD10);
  SD10.usart = USART10;
  SD10.clock = STM32_USART10CLK;
#if !defined(STM32_USART10_SUPPRESS_ISR) && defined(STM32_USART10_NUMBER)
  nvicEnableVector(STM32_USART10_NUMBER, STM32_SERIAL_USART10_PRIORITY);
#endif
#endif

#if STM32_SERIAL_USE_LPUART1
  sdObjectInit(&LPSD1);
  iqObjectInit(&LPSD1.iqueue, sd_in_buflp1, sizeof sd_in_buflp1, NULL, &LPSD1);
  oqObjectInit(&LPSD1.oqueue, sd_out_buflp1, sizeof sd_out_buflp1, notifylp1, &LPSD1);
  LPSD1.usart = LPUART1;
  LPSD1.clock = STM32_LPUART1CLK;
#if !defined(STM32_LPUART1_SUPPRESS_ISR) && defined(STM32_LPUART1_NUMBER)
  nvicEnableVector(STM32_LPUART1_NUMBER, STM32_SERIAL_LPUART1_PRIORITY);
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
#if STM32_SERIAL_USE_USART10
    if (&SD10 == sdp) {
      rccEnableUSART10(true);
    }
#endif
#if STM32_SERIAL_USE_LPUART1
    if (&LPSD1 == sdp) {
      rccEnableLPUART1(true);
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
    /* UART is de-initialized then clocks are disabled.*/
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
#if STM32_SERIAL_USE_USART10
    if (&SD10 == sdp) {
      rccDisableUSART10();
      return;
    }
#endif
#if STM32_SERIAL_USE_LPUART1
    if (&LPSD1 == sdp) {
      rccDisableLPUART1();
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
  uint32_t cr1;
  uint32_t isr;

  /* Reading and clearing status.*/
  isr = u->ISR;
  u->ICR = isr;

  /* Error condition detection.*/
  if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE  | USART_ISR_PE))
    set_error(sdp, isr);

  /* Special case, LIN break detection.*/
  if (isr & USART_ISR_LBDF) {
    osalSysLockFromISR();
    chnAddFlagsI(sdp, SD_BREAK_DETECTED);
    osalSysUnlockFromISR();
  }

  /* Data available, note it is a while in order to handle two situations:
     1) Another byte arrived after removing the previous one, this would cause
        an extra interrupt to serve.
     2) FIFO mode is enabled on devices that support it, we need to empty
        the FIFO.*/
  while (isr & USART_ISR_RXNE) {
    osalSysLockFromISR();
    sdIncomingDataI(sdp, (uint8_t)u->RDR & sdp->rxmask);
    osalSysUnlockFromISR();

    isr = u->ISR;
  }

  /* Caching CR1.*/
  cr1 = u->CR1;

  /* Transmission buffer empty, note it is a while in order to handle two
     situations:
     1) The data registers has been emptied immediately after writing it, this
        would cause an extra interrupt to serve.
     2) FIFO mode is enabled on devices that support it, we need to fill
        the FIFO.*/
  if (cr1 & USART_CR1_TXEIE) {
    while (isr & USART_ISR_TXE) {
      msg_t b;

      osalSysLockFromISR();
      b = oqGetI(&sdp->oqueue);
      if (b < MSG_OK) {
        chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
        cr1 &= ~USART_CR1_TXEIE;
        osalSysUnlockFromISR();
        break;
      }
      u->TDR = b;
      osalSysUnlockFromISR();

      isr = u->ISR;
    }
  }

  /* Physical transmission end.*/
  if ((cr1 & USART_CR1_TCIE) && (isr & USART_ISR_TC)) {
    osalSysLockFromISR();
    if (oqIsEmptyI(&sdp->oqueue)) {
      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
      cr1 &= ~USART_CR1_TCIE;
    }
    osalSysUnlockFromISR();
  }

  /* Writing CR1 once.*/
  u->CR1 = cr1;
  if (isr & USART_ISR_IDLE) {
      _serial_irq_code(sdp);
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
