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
 * @file    SPC5xx/ESCI_v1/hal_serial_lld.c
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
 * @brief   eSCI-A serial driver identifier.
 */
#if SPC5_USE_ESCIA || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/**
 * @brief   eSCI-B serial driver identifier.
 */
#if SPC5_USE_ESCIB || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/**
 * @brief   eSCI-C serial driver identifier.
 */
#if SPC5_USE_ESCIC || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  SD_MODE_NORMAL | SD_MODE_PARITY_NONE
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   eSCI initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void esci_init(SerialDriver *sdp, const SerialConfig *config) {
  volatile struct ESCI_tag *escip = sdp->escip;
  uint8_t mode = config->sc_mode;

  escip->CR2.R  = 0;                /* MDIS off.                            */
  escip->CR1.R  = 0;
  escip->LCR.R  = 0;
  escip->CR1.B.SBR = SPC5_SYSCLK / (16 * config->sc_speed);
  if (mode & SD_MODE_LOOPBACK)
    escip->CR1.B.LOOPS = 1;
  switch (mode & SD_MODE_PARITY_MASK) {
  case SD_MODE_PARITY_ODD:
    escip->CR1.B.PT = 1;
  case SD_MODE_PARITY_EVEN:
    escip->CR1.B.PE = 1;
    escip->CR1.B.M  = 1;            /* Makes it 8 bits data + 1 bit parity. */
  default:
    ;
  }
  escip->LPR.R  = 0;
  escip->CR1.R |= 0x0000002C;       /* RIE, TE, RE to 1.                    */
  escip->CR2.R  = 0x000F;           /* ORIE, NFIE, FEIE, PFIE to 1.         */
}

/**
 * @brief   eSCI de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] escip     pointer to an eSCI I/O block
 */
static void esci_deinit(volatile struct ESCI_tag *escip) {

  escip->LPR.R  = 0;
  escip->SR.R   = 0xFFFFFFFF;
  escip->CR1.R  = 0;
  escip->CR2.R  = 0x8000;           /* MDIS on.                             */
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        eSCI SR register value
 */
static void set_error(SerialDriver *sdp, uint32_t sr) {
  eventflags_t sts = 0;

  if (sr & 0x08000000)
    sts |= SD_OVERRUN_ERROR;
  if (sr & 0x04000000)
    sts |= SD_NOISE_ERROR;
  if (sr & 0x02000000)
    sts |= SD_FRAMING_ERROR;
  if (sr & 0x01000000)
    sts |= SD_PARITY_ERROR;
/*  if (sr & 0x00000000)
    sts |= SD_BREAK_DETECTED;*/
  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
static void serve_interrupt(SerialDriver *sdp) {
  volatile struct ESCI_tag *escip = sdp->escip;

  uint32_t sr = escip->SR.R;
  escip->SR.R = 0x3FFFFFFF;                     /* Does not clear TDRE | TC.*/
  if (sr & 0x0F000000)                          /* OR | NF | FE | PF.       */
    set_error(sdp, sr);
  if (sr & 0x20000000) {                        /* RDRF.                    */
    osalSysLockFromISR();
    sdIncomingDataI(sdp, escip->DR.B.D);
    osalSysUnlockFromISR();
  }
  if (escip->CR1.B.TIE && (sr & 0x80000000)) {  /* TDRE.                    */
    msg_t b;
    osalSysLockFromISR();
    b = oqGetI(&sdp->oqueue);
    if (b < Q_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      escip->CR1.B.TIE = 0;
    }
    else {
      escip->SR.B.TDRE = 1;
      escip->DR.R = (uint16_t)b;
    }
    osalSysUnlockFromISR();
  }
}

#if SPC5_USE_ESCIA || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  if (ESCI_A.SR.B.TDRE) {
    msg_t b = sdRequestDataI(&SD1);
    if (b != Q_EMPTY) {
      ESCI_A.SR.B.TDRE = 1;
      ESCI_A.CR1.B.TIE = 1;
      ESCI_A.DR.R = (uint16_t)b;
    }
  }
}
#endif

#if SPC5_USE_ESCIB || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  if (ESCI_B.SR.B.TDRE) {
    msg_t b = sdRequestDataI(&SD2);
    if (b != Q_EMPTY) {
      ESCI_B.SR.B.TDRE = 1;
      ESCI_B.CR1.B.TIE = 1;
      ESCI_B.DR.R = (uint16_t)b;
    }
  }
}
#endif

#if SPC5_USE_ESCIC || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  if (ESCI_C.SR.B.TDRE) {
    msg_t b = sdRequestDataI(&SD3);
    if (b != Q_EMPTY) {
      ESCI_C.SR.B.TDRE = 1;
      ESCI_C.CR1.B.TIE = 1;
      ESCI_C.DR.R = (uint16_t)b;
    }
  }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SPC5_USE_ESCIA || defined(__DOXYGEN__)
#if !defined(SPC5_ESCIA_HANDLER)
#error "SPC5_ESCIA_HANDLER not defined"
#endif
/**
 * @brief   eSCI-A interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_ESCIA_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_USE_ESCIB || defined(__DOXYGEN__)
#if !defined(SPC5_ESCIB_HANDLER)
#error "SPC5_ESCIB_HANDLER not defined"
#endif
/**
 * @brief   eSCI-B interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_ESCIB_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SPC5_USE_ESCIC || defined(__DOXYGEN__)
#if !defined(SPC5_ESCIC_HANDLER)
#error "SPC5_ESCIC_HANDLER not defined"
#endif
/**
 * @brief   eSCI-C interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SPC5_ESCIC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD3);

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

#if SPC5_USE_ESCIA
  sdObjectInit(&SD1, NULL, notify1);
  SD1.escip       = &ESCI_A;
  ESCI_A.CR2.R    = 0x8000;                 /* MDIS ON.                     */
  INTC.PSR[SPC5_ESCIA_NUMBER].R = SPC5_ESCIA_PRIORITY;
#endif

#if SPC5_USE_ESCIB
  sdObjectInit(&SD2, NULL, notify2);
  SD2.escip       = &ESCI_B;
  ESCI_B.CR2.R    = 0x8000;                 /* MDIS ON.                     */
  INTC.PSR[SPC5_ESCIB_NUMBER].R = SPC5_ESCIB_PRIORITY;
#endif

#if SPC5_USE_ESCIC
  sdObjectInit(&SD3, NULL, notify3);
  SD3.escip       = &ESCI_C;
  ESCI_C.CR2.R    = 0x8000;                 /* MDIS ON.                     */
  INTC.PSR[SPC5_ESCIC_NUMBER].R = SPC5_ESCIC_PRIORITY;
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
  esci_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY)
    esci_deinit(sdp->escip);
}

#endif /* HAL_USE_SERIAL */

/** @} */
