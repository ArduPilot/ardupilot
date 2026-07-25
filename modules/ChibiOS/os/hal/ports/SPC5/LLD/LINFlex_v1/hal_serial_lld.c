/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC5xx/LINFlex_v1/hal_serial_lld.c
 * @brief   SPC5xx low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   LIINFlex-0 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX0 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/**
 * @brief   LIINFlex-1 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX1 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/**
 * @brief   LIINFlex-2 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX2 || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

/**
 * @brief   LIINFlex-3 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX3 || defined(__DOXYGEN__)
SerialDriver SD4;
#endif

/**
 * @brief   LIINFlex-4 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX4 || defined(__DOXYGEN__)
SerialDriver SD5;
#endif

/**
 * @brief   LIINFlex-5 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX5 || defined(__DOXYGEN__)
SerialDriver SD6;
#endif

/**
 * @brief   LIINFlex-6 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX6 || defined(__DOXYGEN__)
SerialDriver SD7;
#endif

/**
 * @brief   LIINFlex-7 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX7 || defined(__DOXYGEN__)
SerialDriver SD8;
#endif

/**
 * @brief   LIINFlex-8 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX8 || defined(__DOXYGEN__)
SerialDriver SD9;
#endif

/**
 * @brief   LIINFlex-9 serial driver identifier.
 */
#if SPC5_SERIAL_USE_LINFLEX9 || defined(__DOXYGEN__)
SerialDriver SD10;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  SD_MODE_8BITS_PARITY_NONE
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   LINFlex initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void spc5_linflex_init(SerialDriver *sdp, const SerialConfig *config) {
  uint32_t div;
  volatile struct spc5_linflex *linflexp = sdp->linflexp;

  /* Enters the configuration mode.*/
  linflexp->LINCR1.R  = 1;                      /* INIT bit.                */

  /* Configures the LINFlex in UART mode with all the required
     parameters.*/
  linflexp->UARTCR.R  = SPC5_UARTCR_UART;       /* UART mode FIRST.         */
  linflexp->UARTCR.R  = SPC5_UARTCR_UART | SPC5_UARTCR_RXEN | config->mode;
  div = SPC5_LINFLEX0_CLK / config->speed;
  linflexp->LINFBRR.R = (uint16_t)(div & 15);   /* Fractional divider.      */
  linflexp->LINIBRR.R = (uint16_t)(div >> 4);   /* Integer divider.         */
  linflexp->UARTSR.R  = 0xFFFF;                 /* Clearing UARTSR register.*/
  linflexp->LINIER.R  = SPC5_LINIER_DTIE | SPC5_LINIER_DRIE |
                        SPC5_LINIER_BOIE | SPC5_LINIER_FEIE |
                        SPC5_LINIER_SZIE;       /* Interrupts enabled.      */

  /* Leaves the configuration mode.*/
  linflexp->LINCR1.R  = 0;
}

/**
 * @brief   LINFlex de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] linflexp  pointer to a LINFlex I/O block
 */
static void spc5_linflex_deinit(volatile struct spc5_linflex *linflexp) {

  /* Enters the configuration mode.*/
  linflexp->LINCR1.R  = 1;                      /* INIT bit.                */

  /* Resets the LINFlex registers.*/
  linflexp->LINFBRR.R = 0;                      /* Fractional divider.      */
  linflexp->LINIBRR.R = 0;                      /* Integer divider.         */
  linflexp->UARTSR.R  = 0xFFFF;                 /* Clearing UARTSR register.*/
  linflexp->UARTCR.R  = SPC5_UARTCR_UART;
  linflexp->LINIER.R  = 0;                      /* Interrupts disabled.     */

  /* Leaves the configuration mode.*/
  linflexp->LINCR1.R  = 0;
}

/**
 * @brief   Common RXI IRQ handler.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
static void spc5xx_serve_rxi_interrupt(SerialDriver *sdp) {
  eventflags_t sts = 0;
  uint16_t sr = sdp->linflexp->UARTSR.R;

  sdp->linflexp->UARTSR.R = SPC5_UARTSR_NF | SPC5_UARTSR_DRF |
                            SPC5_UARTSR_PE0;
  if (sr & SPC5_UARTSR_NF)
    sts |= SD_NOISE_ERROR;
  if (sr & SPC5_UARTSR_PE0)
    sts |= SD_PARITY_ERROR;
  osalSysLockFromISR();
  if (sts)
    chnAddFlagsI(sdp, sts);
  if (sr & SPC5_UARTSR_DRF) {
    sdIncomingDataI(sdp, sdp->linflexp->BDRM.B.DATA4);
    sdp->linflexp->UARTSR.R = SPC5_UARTSR_RMB;
  }
  osalSysUnlockFromISR();
}

/**
 * @brief   Common TXI IRQ handler.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
static void spc5xx_serve_txi_interrupt(SerialDriver *sdp) {
  msg_t b;

  sdp->linflexp->UARTSR.R = SPC5_UARTSR_DTF;
  osalSysLockFromISR();
  b = oqGetI(&sdp->oqueue);
  if (b < Q_OK) {
    chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
    sdp->linflexp->UARTCR.B.TXEN = 0;
  }
  else
    sdp->linflexp->BDRL.B.DATA0 = b;
  osalSysUnlockFromISR();
}

/**
 * @brief   Common ERR IRQ handler.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
static void spc5xx_serve_err_interrupt(SerialDriver *sdp) {
  eventflags_t sts = 0;
  uint16_t sr = sdp->linflexp->UARTSR.R;

  sdp->linflexp->UARTSR.R = SPC5_UARTSR_BOF | SPC5_UARTSR_FEF |
                            SPC5_UARTSR_SZF;
  if (sr & SPC5_UARTSR_BOF)
    sts |= SD_OVERRUN_ERROR;
  if (sr & SPC5_UARTSR_FEF)
    sts |= SD_FRAMING_ERROR;
  if (sr & SPC5_UARTSR_SZF)
    sts |= SD_BREAK_DETECTED;
  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
}

#if SPC5_SERIAL_USE_LINFLEX0 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  if (!SD1.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD1);
    if (b != Q_EMPTY) {
      SD1.linflexp->UARTCR.B.TXEN = 1;
      SD1.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX1 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  if (!SD2.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD2);
    if (b != Q_EMPTY) {
      SD2.linflexp->UARTCR.B.TXEN = 1;
      SD2.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX2 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  if (!SD3.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD3);
    if (b != Q_EMPTY) {
      SD3.linflexp->UARTCR.B.TXEN = 1;
      SD3.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX3 || defined(__DOXYGEN__)
static void notify4(io_queue_t *qp) {

  (void)qp;
  if (!SD4.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD4);
    if (b != Q_EMPTY) {
      SD4.linflexp->UARTCR.B.TXEN = 1;
      SD4.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX4 || defined(__DOXYGEN__)
static void notify5(io_queue_t *qp) {

  (void)qp;
  if (!SD5.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD5);
    if (b != Q_EMPTY) {
      SD5.linflexp->UARTCR.B.TXEN = 1;
      SD5.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX5 || defined(__DOXYGEN__)
static void notify6(io_queue_t *qp) {

  (void)qp;
  if (!SD6.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD6);
    if (b != Q_EMPTY) {
      SD6.linflexp->UARTCR.B.TXEN = 1;
      SD6.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX6 || defined(__DOXYGEN__)
static void notify7(io_queue_t *qp) {

  (void)qp;
  if (!SD7.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD7);
    if (b != Q_EMPTY) {
      SD7.linflexp->UARTCR.B.TXEN = 1;
      SD7.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX7 || defined(__DOXYGEN__)
static void notify8(io_queue_t *qp) {

  (void)qp;
  if (!SD8.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD8);
    if (b != Q_EMPTY) {
      SD8.linflexp->UARTCR.B.TXEN = 1;
      SD8.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX8 || defined(__DOXYGEN__)
static void notify9(io_queue_t *qp) {

  (void)qp;
  if (!SD9.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD9);
    if (b != Q_EMPTY) {
      SD9.linflexp->UARTCR.B.TXEN = 1;
      SD9.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

#if SPC5_SERIAL_USE_LINFLEX9 || defined(__DOXYGEN__)
static void notify10(io_queue_t *qp) {

  (void)qp;
  if (!SD10.linflexp->UARTCR.B.TXEN) {
    msg_t b = sdRequestDataI(&SD10);
    if (b != Q_EMPTY) {
      SD10.linflexp->UARTCR.B.TXEN = 1;
      SD10.linflexp->BDRL.B.DATA0 = b;
    }
  }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SPC5_SERIAL_USE_LINFLEX0 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX0_RXI_HANDLER)
#error "SPC5_LINFLEX0_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-0 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX0_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX0_TXI_HANDLER)
#error "SPC5_LINFLEX0_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-0 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX0_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX0_ERR_HANDLER)
#error "SPC5_LINFLEX0_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-0 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX0_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX1 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX1_RXI_HANDLER)
#error "SPC5_LINFLEX1_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-1 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX1_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX1_TXI_HANDLER)
#error "SPC5_LINFLEX1_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-1 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX1_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX1_ERR_HANDLER)
#error "SPC5_LINFLEX1_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-1 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX1_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX2 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX2_RXI_HANDLER)
#error "SPC5_LINFLEX2_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-2 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX2_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX2_TXI_HANDLER)
#error "SPC5_LINFLEX2_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-2 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX2_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX2_ERR_HANDLER)
#error "SPC5_LINFLEX2_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-2 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX2_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX3 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX3_RXI_HANDLER)
#error "SPC5_LINFLEX3_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-3 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX3_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD4);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX3_TXI_HANDLER)
#error "SPC5_LINFLEX3_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-3 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX3_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD4);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX3_ERR_HANDLER)
#error "SPC5_LINFLEX3_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-3 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX3_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX4 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX4_RXI_HANDLER)
#error "SPC5_LINFLEX4_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-4 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX4_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD5);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX4_TXI_HANDLER)
#error "SPC5_LINFLEX4_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-4 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX4_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD5);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX4_ERR_HANDLER)
#error "SPC5_LINFLEX4_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-4 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX4_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX5 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX5_RXI_HANDLER)
#error "SPC5_LINFLEX5_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-5 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX5_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD6);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX5_TXI_HANDLER)
#error "SPC5_LINFLEX5_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-5 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX5_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD6);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX5_ERR_HANDLER)
#error "SPC5_LINFLEX5_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-5 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX5_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX6 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX6_RXI_HANDLER)
#error "SPC5_LINFLEX6_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-6 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX6_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD7);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX6_TXI_HANDLER)
#error "SPC5_LINFLEX6_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-6 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX6_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD7);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX6_ERR_HANDLER)
#error "SPC5_LINFLEX6_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-6 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX6_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD7);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX7 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX7_RXI_HANDLER)
#error "SPC5_LINFLEX7_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-7 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX7_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD8);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX7_TXI_HANDLER)
#error "SPC5_LINFLEX7_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-7 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX7_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD8);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX7_ERR_HANDLER)
#error "SPC5_LINFLEX7_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-7 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX7_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD8);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX8 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX8_RXI_HANDLER)
#error "SPC5_LINFLEX8_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-8 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX8_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD9);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX8_TXI_HANDLER)
#error "SPC5_LINFLEX8_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-8 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX8_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD9);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX8_ERR_HANDLER)
#error "SPC5_LINFLEX8_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-8 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX8_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD9);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_SERIAL_USE_LINFLEX9 || defined(__DOXYGEN__)
#if !defined(SPC5_LINFLEX9_RXI_HANDLER)
#error "SPC5_LINFLEX9_RXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-9 RXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX9_RXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_rxi_interrupt(&SD10);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX9_TXI_HANDLER)
#error "SPC5_LINFLEX9_TXI_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-9 TXI interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX9_TXI_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_txi_interrupt(&SD10);

  OSAL_IRQ_EPILOGUE();
}

#if !defined(SPC5_LINFLEX9_ERR_HANDLER)
#error "SPC5_LINFLEX9_ERR_HANDLER not defined"
#endif
/**
 * @brief   LINFlex-9 ERR interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_LINFLEX9_ERR_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  spc5xx_serve_err_interrupt(&SD10);

  OSAL_IRQ_EPILOGUE();
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

#if SPC5_SERIAL_USE_LINFLEX0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.linflexp = &SPC5_LINFLEX0;
  INTC_PSR(SPC5_LINFLEX0_RXI_NUMBER) = SPC5_SERIAL_LINFLEX0_PRIORITY;
  INTC_PSR(SPC5_LINFLEX0_TXI_NUMBER) = SPC5_SERIAL_LINFLEX0_PRIORITY;
  INTC_PSR(SPC5_LINFLEX0_ERR_NUMBER) = SPC5_SERIAL_LINFLEX0_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX1
  sdObjectInit(&SD2, NULL, notify2);
  SD2.linflexp = &SPC5_LINFLEX1;
  INTC_PSR(SPC5_LINFLEX1_RXI_NUMBER) = SPC5_SERIAL_LINFLEX1_PRIORITY;
  INTC_PSR(SPC5_LINFLEX1_TXI_NUMBER) = SPC5_SERIAL_LINFLEX1_PRIORITY;
  INTC_PSR(SPC5_LINFLEX1_ERR_NUMBER) = SPC5_SERIAL_LINFLEX1_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX2
  sdObjectInit(&SD3, NULL, notify3);
  SD3.linflexp = &SPC5_LINFLEX2;
  INTC_PSR(SPC5_LINFLEX2_RXI_NUMBER) = SPC5_SERIAL_LINFLEX2_PRIORITY;
  INTC_PSR(SPC5_LINFLEX2_TXI_NUMBER) = SPC5_SERIAL_LINFLEX2_PRIORITY;
  INTC_PSR(SPC5_LINFLEX2_ERR_NUMBER) = SPC5_SERIAL_LINFLEX2_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX3
  sdObjectInit(&SD4, NULL, notify4);
  SD4.linflexp = &SPC5_LINFLEX3;
  INTC_PSR(SPC5_LINFLEX3_RXI_NUMBER) = SPC5_SERIAL_LINFLEX3_PRIORITY;
  INTC_PSR(SPC5_LINFLEX3_TXI_NUMBER) = SPC5_SERIAL_LINFLEX3_PRIORITY;
  INTC_PSR(SPC5_LINFLEX3_ERR_NUMBER) = SPC5_SERIAL_LINFLEX3_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX4
  sdObjectInit(&SD5, NULL, notify5);
  SD5.linflexp = &SPC5_LINFLEX4;
  INTC_PSR(SPC5_LINFLEX4_RXI_NUMBER) = SPC5_SERIAL_LINFLEX4_PRIORITY;
  INTC_PSR(SPC5_LINFLEX4_TXI_NUMBER) = SPC5_SERIAL_LINFLEX4_PRIORITY;
  INTC_PSR(SPC5_LINFLEX4_ERR_NUMBER) = SPC5_SERIAL_LINFLEX4_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX5
  sdObjectInit(&SD6, NULL, notify6);
  SD6.linflexp = &SPC5_LINFLEX5;
  INTC_PSR(SPC5_LINFLEX5_RXI_NUMBER) = SPC5_SERIAL_LINFLEX5_PRIORITY;
  INTC_PSR(SPC5_LINFLEX5_TXI_NUMBER) = SPC5_SERIAL_LINFLEX5_PRIORITY;
  INTC_PSR(SPC5_LINFLEX5_ERR_NUMBER) = SPC5_SERIAL_LINFLEX5_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX6
  sdObjectInit(&SD7, NULL, notify7);
  SD7.linflexp = &SPC5_LINFLEX6;
  INTC_PSR(SPC5_LINFLEX6_RXI_NUMBER) = SPC5_SERIAL_LINFLEX6_PRIORITY;
  INTC_PSR(SPC5_LINFLEX6_TXI_NUMBER) = SPC5_SERIAL_LINFLEX6_PRIORITY;
  INTC_PSR(SPC5_LINFLEX6_ERR_NUMBER) = SPC5_SERIAL_LINFLEX6_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX7
  sdObjectInit(&SD8, NULL, notify8);
  SD8.linflexp = &SPC5_LINFLEX7;
  INTC_PSR(SPC5_LINFLEX7_RXI_NUMBER) = SPC5_SERIAL_LINFLEX7_PRIORITY;
  INTC_PSR(SPC5_LINFLEX7_TXI_NUMBER) = SPC5_SERIAL_LINFLEX7_PRIORITY;
  INTC_PSR(SPC5_LINFLEX7_ERR_NUMBER) = SPC5_SERIAL_LINFLEX7_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX8
  sdObjectInit(&SD9, NULL, notify9);
  SD9.linflexp = &SPC5_LINFLEX8;
  INTC_PSR(SPC5_LINFLEX8_RXI_NUMBER) = SPC5_SERIAL_LINFLEX8_PRIORITY;
  INTC_PSR(SPC5_LINFLEX8_TXI_NUMBER) = SPC5_SERIAL_LINFLEX8_PRIORITY;
  INTC_PSR(SPC5_LINFLEX8_ERR_NUMBER) = SPC5_SERIAL_LINFLEX8_PRIORITY;
#endif

#if SPC5_SERIAL_USE_LINFLEX9
  sdObjectInit(&SD10, NULL, notify10);
  SD10.linflexp = &SPC5_LINFLEX9;
  INTC_PSR(SPC5_LINFLEX9_RXI_NUMBER) = SPC5_SERIAL_LINFLEX9_PRIORITY;
  INTC_PSR(SPC5_LINFLEX9_TXI_NUMBER) = SPC5_SERIAL_LINFLEX9_PRIORITY;
  INTC_PSR(SPC5_LINFLEX9_ERR_NUMBER) = SPC5_SERIAL_LINFLEX9_PRIORITY;
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
#if SPC5_SERIAL_USE_LINFLEX0
    if (&SD1 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX0_PCTL,
                                   SPC5_SERIAL_LINFLEX0_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX1
    if (&SD2 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX1_PCTL,
                                   SPC5_SERIAL_LINFLEX1_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX2
    if (&SD3 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX2_PCTL,
                                   SPC5_SERIAL_LINFLEX2_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX3
    if (&SD4 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX3_PCTL,
                                   SPC5_SERIAL_LINFLEX3_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX4
    if (&SD5 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX4_PCTL,
                                   SPC5_SERIAL_LINFLEX4_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX5
    if (&SD6 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX5_PCTL,
                                   SPC5_SERIAL_LINFLEX5_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX6
    if (&SD7 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX6_PCTL,
                                   SPC5_SERIAL_LINFLEX6_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX7
    if (&SD8 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX7_PCTL,
                                   SPC5_SERIAL_LINFLEX7_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX8
    if (&SD9 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX8_PCTL,
                                   SPC5_SERIAL_LINFLEX8_START_PCTL);
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX9
    if (&SD10 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX9_PCTL,
                                   SPC5_SERIAL_LINFLEX9_START_PCTL);
    }
#endif
  }
  spc5_linflex_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    spc5_linflex_deinit(sdp->linflexp);

#if SPC5_SERIAL_USE_LINFLEX0
    if (&SD1 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX0_PCTL,
                                   SPC5_SERIAL_LINFLEX0_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX1
    if (&SD2 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX1_PCTL,
                                   SPC5_SERIAL_LINFLEX1_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX2
    if (&SD3 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX2_PCTL,
                                   SPC5_SERIAL_LINFLEX2_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX3
    if (&SD4 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX3_PCTL,
                                   SPC5_SERIAL_LINFLEX3_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX4
    if (&SD5 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX4_PCTL,
                                   SPC5_SERIAL_LINFLEX4_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX5
    if (&SD6 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX5_PCTL,
                                   SPC5_SERIAL_LINFLEX5_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX6
    if (&SD7 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX6_PCTL,
                                   SPC5_SERIAL_LINFLEX6_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX7
    if (&SD8 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX7_PCTL,
                                   SPC5_SERIAL_LINFLEX7_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX8
    if (&SD9 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX8_PCTL,
                                   SPC5_SERIAL_LINFLEX8_STOP_PCTL);
      return;
    }
#endif
#if SPC5_SERIAL_USE_LINFLEX9
    if (&SD10 == sdp) {
      halSPCSetPeripheralClockMode(SPC5_LINFLEX9_PCTL,
                                   SPC5_SERIAL_LINFLEX9_STOP_PCTL);
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
