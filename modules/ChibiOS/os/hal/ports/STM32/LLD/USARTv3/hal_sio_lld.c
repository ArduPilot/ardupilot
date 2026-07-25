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
 * @file    USARTv3/hal_sio_lld.c
 * @brief   STM32 SIO subsystem low level driver source.
 *
 * @addtogroup SIO
 * @{
 */

#include "hal.h"

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* Bits not present on all implementations.*/
#if !defined(USART_CR3_WUFIE)
#define USART_CR3_WUFIE                     0U
#endif

#define USART_CR1_CFG_FORBIDDEN             (USART_CR1_RXFFIE           |   \
                                             USART_CR1_TXFEIE           |   \
                                             USART_CR1_FIFOEN           |   \
                                             USART_CR1_EOBIE            |   \
                                             USART_CR1_RTOIE            |   \
                                             USART_CR1_CMIE             |   \
                                             USART_CR1_PEIE             |   \
                                             USART_CR1_TXEIE_TXFNFIE    |   \
                                             USART_CR1_TCIE             |   \
                                             USART_CR1_RXNEIE_RXFNEIE   |   \
                                             USART_CR1_IDLEIE           |   \
                                             USART_CR1_TE               |   \
                                             USART_CR1_RE               |   \
                                             USART_CR1_UE)
#define USART_CR2_CFG_FORBIDDEN             (USART_CR2_LBDIE)
#define USART_CR3_CFG_FORBIDDEN             (USART_CR3_RXFTIE           |   \
                                             USART_CR3_TCBGTIE          |   \
                                             USART_CR3_TXFTIE           |   \
                                             USART_CR3_WUFIE            |   \
                                             USART_CR3_CTSIE            |   \
                                             USART_CR3_EIE)

/* This mask includes IDLE, ORE, NE, FE, PE bits.*/
#define USART_ISR_IONFP_Pos     USART_ISR_PE_Pos
#define USART_ISR_IONFP_Msk     (0x1FUL << USART_ISR_IONFP_Pos)

/* This mask includes ORE, NE, FE, PE bits.*/
#define USART_ISR_ONFP_Pos      USART_ISR_PE_Pos
#define USART_ISR_ONFP_Msk      (0xFUL << USART_ISR_ONFP_Pos)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USART1 SIO driver identifier.
 */
#if (STM32_SIO_USE_USART1 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD1;
#endif

/**
 * @brief   USART2 SIO driver identifier.
 */
#if (STM32_SIO_USE_USART2 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD2;
#endif

/**
 * @brief   USART3 SIO driver identifier.
 */
#if (STM32_SIO_USE_USART3 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD3;
#endif

/**
 * @brief   UART4 SIO driver identifier.
 */
#if (STM32_SIO_USE_UART4 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD4;
#endif

/**
 * @brief   UART5 SIO driver identifier.
 */
#if (STM32_SIO_USE_UART5 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD5;
#endif

/**
 * @brief   USART6 SIO driver identifier.
 */
#if (STM32_SIO_USE_USART6 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD6;
#endif

/**
 * @brief   UART7 SIO driver identifier.
 */
#if (STM32_SIO_USE_UART7 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD7;
#endif

/**
 * @brief   UART8 SIO driver identifier.
 */
#if (STM32_SIO_USE_UART8 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD8;
#endif

/**
 * @brief   UART9 SIO driver identifier.
 */
#if (STM32_SIO_USE_UART9 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD9;
#endif

/**
 * @brief   USART10 SIO driver identifier.
 */
#if (STM32_SIO_USE_USART10 == TRUE) || defined(__DOXYGEN__)
SIODriver SIOD10;
#endif

/**
 * @brief   LPUART1 SIO driver identifier.
 */
#if (STM32_SIO_USE_LPUART1 == TRUE) || defined(__DOXYGEN__)
SIODriver LPSIOD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 * @note    In this implementation it is: 38400-8-N-1, RX and TX FIFO
 *          thresholds set tx-non-full and rx-non-empty.
 */
static const SIOConfig default_config = {
  .baud  = SIO_DEFAULT_BITRATE,
  .presc = USART_PRESC1,
  .cr1   = USART_CR1_DATA8 | USART_CR1_OVER16,
  .cr2   = USART_CR2_STOP1_BITS,
  .cr3   = USART_CR3_TXFTCFG_NONFULL | USART_CR3_RXFTCFG_NONEMPTY
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE void usart_enable_rx_irq(SIODriver *siop) {
  uint32_t cr1;

  cr1 = siop->usart->CR1;
  if ((siop->enabled & SIO_EV_RXNOTEMPY) != 0U) {
#if STM32_USART_MIXED == TRUE
    if (!siop->has_fifo) {
      cr1 |= USART_CR1_RXNEIE_RXFNEIE;
    }
    else
#endif
    if ((siop->config->cr3 & USART_CR3_RXFTCFG_Msk) == USART_CR3_RXFTCFG_NONEMPTY) {
      cr1 |= USART_CR1_RXNEIE_RXFNEIE;
    }
    else {
      siop->usart->CR3 |= USART_CR3_RXFTIE;
    }
  }
  if ((siop->enabled & SIO_EV_RXIDLE) != 0U) {
    cr1 |= USART_CR1_IDLEIE;
  }
  siop->usart->CR1 = cr1;
}

__STATIC_INLINE void usart_enable_rx_errors_irq(SIODriver *siop) {

  siop->usart->CR1 |= __sio_reloc_field(siop->enabled, SIO_EV_PARITY_ERR, SIO_EV_PARITY_ERR_POS, USART_CR1_PEIE_Pos);
  siop->usart->CR2 |= __sio_reloc_field(siop->enabled, SIO_EV_RXBREAK,    SIO_EV_RXBREAK_POS,    USART_CR2_LBDIE_Pos);

  /* The following 3 are grouped.*/
  if ((siop->enabled & (SIO_EV_FRAMING_ERR |
                        SIO_EV_OVERRUN_ERR |
                        SIO_EV_NOISE_ERR)) != 0U) {
    siop->usart->CR3 |= USART_CR3_EIE;
  }
}

__STATIC_INLINE void usart_enable_tx_irq(SIODriver *siop) {

  if ((siop->enabled & SIO_EV_TXNOTFULL) != 0U) {
#if STM32_USART_MIXED == TRUE
    if (!siop->has_fifo) {
      siop->usart->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    }
    else
#endif
    if ((siop->config->cr3 & USART_CR3_TXFTCFG_Msk) == USART_CR3_TXFTCFG_NONFULL) {
      siop->usart->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    }
    else {
      siop->usart->CR3 |= USART_CR3_TXFTIE;
    }
  }
}

__STATIC_INLINE void usart_enable_tx_end_irq(SIODriver *siop) {

  if ((siop->enabled & SIO_EV_TXDONE) != 0U) {
    siop->usart->CR1 |= USART_CR1_TCIE;
  }
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] siop       pointer to a @p SIODriver object
 */
__STATIC_INLINE void usart_init(SIODriver *siop) {
  USART_TypeDef *u = siop->usart;
  uint32_t presc, brr, clock, cr3;

  /* Clock input frequency, it could be dynamic.*/
  if (false) {
  }
#if STM32_SIO_USE_USART1 == TRUE
  else if (&SIOD1 == siop) {
    clock = STM32_USART1CLK;
  }
#endif
#if STM32_SIO_USE_USART2 == TRUE
  else if (&SIOD2 == siop) {
    clock = STM32_USART2CLK;
  }
#endif
#if STM32_SIO_USE_USART3 == TRUE
  else if (&SIOD3 == siop) {
    clock = STM32_USART3CLK;
  }
#endif
#if STM32_SIO_USE_UART4 == TRUE
  else if (&SIOD4 == siop) {
    clock = STM32_UART4CLK;
  }
#endif
#if STM32_SIO_USE_UART5 == TRUE
  else if (&SIOD5 == siop) {
    clock = STM32_UART5CLK;
  }
#endif
#if STM32_SIO_USE_USART6 == TRUE
  else if (&SIOD6 == siop) {
    clock = STM32_USART6CLK;
  }
#endif
#if STM32_SIO_USE_UART7 == TRUE
  else if (&SIOD7 == siop) {
    clock = STM32_UART7CLK;
  }
#endif
#if STM32_SIO_USE_UART8 == TRUE
  else if (&SIOD8 == siop) {
    clock = STM32_UART8CLK;
  }
#endif
#if STM32_SIO_USE_UART9 == TRUE
  else if (&SIOD9 == siop) {
    clock = STM32_UART9CLK;
  }
#endif
#if STM32_SIO_USE_USART10 == TRUE
  else if (&SIOD10 == siop) {
    clock = STM32_USART10CLK;
  }
#endif
#if STM32_SIO_USE_LPUART1 == TRUE
  else if (&LPSIOD1 == siop) {
    clock = STM32_LPUART1CLK;
  }
#endif
  else {
    osalDbgAssert(false, "invalid SIO instance");
  }

  /* Prescaler calculation.*/
  static const uint32_t prescvals[] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
  presc = prescvals[siop->config->presc];

 /* Baud rate setting.*/
#if STM32_SIO_USE_LPUART1
  if (siop == &LPSIOD1) {
    osalDbgAssert((clock >= siop->config->baud * 3U) &&
                  (clock <= siop->config->baud * 4096U),
                  "invalid baud rate vs input clock");

    brr = (uint32_t)(((uint64_t)(clock / presc) * (uint64_t)256) / siop->config->baud);

    osalDbgAssert((brr >= 0x300) && (brr < 0x100000), "invalid BRR value");
  }
  else
#endif
  {
    brr = (uint32_t)((clock / presc) / siop->config->baud);

    /* Correcting BRR value when oversampling by 8 instead of 16.
       Fraction is still 4 bits wide, but only lower 3 bits used.
       Mantissa is doubled, but Fraction is left the same.*/
    if ((siop->config->cr1 & USART_CR1_OVER8) != 0U) {
      brr = ((brr & ~7U) * 2U) | (brr & 7U);
    }

    osalDbgAssert(brr < 0x10000, "invalid BRR value");
  }

  /* Setting up USART, FIFO mode enforced but ignored in devices without FIFO.*/
  u->CR1   = (siop->config->cr1 & ~USART_CR1_CFG_FORBIDDEN) | USART_CR1_FIFOEN;
  u->CR2   = siop->config->cr2 & ~USART_CR2_CFG_FORBIDDEN;
  cr3      = siop->config->cr3 & ~USART_CR3_CFG_FORBIDDEN;

  /* Checking special cases where single character mode are specified as
     FIFO thresholds.*/
  if ((siop->config->cr3 & USART_CR3_RXFTCFG_Msk) == USART_CR3_RXFTCFG_NONEMPTY) {
    /* If single character mode specified by config mask off threshold setting.*/
    cr3 &= ~USART_CR3_RXFTCFG_Msk;
  }
  if ((siop->config->cr3 & USART_CR3_TXFTCFG_Msk) == USART_CR3_TXFTCFG_NONFULL) {
    /* If single character mode specified by config mask off threshold setting.*/
    cr3 &= ~USART_CR3_TXFTCFG_Msk;
  }
#if STM32_USART_MIXED == TRUE
  if (!siop->has_fifo) {
    /* TODO: check if those fields are simply ignored in devices without FIFO,
       this code could be removed.*/
    cr3 &= ~(USART_CR3_RXFTCFG_Msk | USART_CR3_TXFTCFG_Msk |
             USART_CR3_RXFTIE | USART_CR3_TXFTIE);
  }
#endif
  u->CR3   = cr3;
  u->PRESC = siop->config->presc;
  u->BRR   = brr;

  /* Starting operations.*/
  u->ICR   = u->ISR;
  u->CR1  |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SIO driver initialization.
 *
 * @notapi
 */
void sio_lld_init(void) {

  /* Driver instances initialization.*/
#if STM32_SIO_USE_USART1 == TRUE
  sioObjectInit(&SIOD1);
  SIOD1.usart       = USART1;
#if STM32_USART_MIXED == TRUE
  SIOD1.has_fifo    = (bool)(STM32_USART1_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_USART2 == TRUE
  sioObjectInit(&SIOD2);
  SIOD2.usart       = USART2;
#if STM32_USART_MIXED == TRUE
  SIOD2.has_fifo    = (bool)(STM32_USART2_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_USART3 == TRUE
  sioObjectInit(&SIOD3);
  SIOD3.usart       = USART3;
#if STM32_USART_MIXED == TRUE
  SIOD3.has_fifo    = (bool)(STM32_USART3_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_UART4 == TRUE
  sioObjectInit(&SIOD4);
  SIOD4.usart       = UART4;
#if STM32_USART_MIXED == TRUE
  SIOD4.has_fifo    = (bool)(STM32_UART4_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_UART5 == TRUE
  sioObjectInit(&SIOD5);
  SIOD5.usart       = UART5;
#if STM32_USART_MIXED == TRUE
  SIOD5.has_fifo    = (bool)(STM32_UART5_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_USART6 == TRUE
  sioObjectInit(&SIOD6);
  SIOD6.usart       = USART6;
#if STM32_USART_MIXED == TRUE
  SIOD6.has_fifo    = (bool)(STM32_USART6_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_UART7 == TRUE
  sioObjectInit(&SIOD7);
  SIOD7.usart       = UART7;
#if STM32_USART_MIXED == TRUE
  SIOD7.has_fifo    = (bool)(STM32_UART7_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_UART8 == TRUE
  sioObjectInit(&SIOD8);
  SIOD8.usart       = UART8;
#if STM32_USART_MIXED == TRUE
  SIOD8.has_fifo    = (bool)(STM32_UART8_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_UART9 == TRUE
  sioObjectInit(&SIOD9);
  SIOD9.usart       = UART9;
#if STM32_USART_MIXED == TRUE
  SIOD9.has_fifo    = (bool)(STM32_UART9_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_USART10 == TRUE
  sioObjectInit(&SIOD10);
  SIOD10.usart      = USART10;
#if STM32_USART_MIXED == TRUE
  SIOD10.has_fifo   = (bool)(STM32_USART10_HAS_FIFO == TRUE);
#endif
#endif
#if STM32_SIO_USE_LPUART1 == TRUE
  sioObjectInit(&LPSIOD1);
  LPSIOD1.usart     = LPUART1;
#if STM32_USART_MIXED == TRUE
  LPSIOD1.has_fifo  = true;
#endif
#endif
}

/**
 * @brief   Configures and activates the SIO peripheral.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The operation status.
 *
 * @notapi
 */
msg_t sio_lld_start(SIODriver *siop) {

  /* Using the default configuration if the application passed a
     NULL pointer.*/
  if (siop->config == NULL) {
    siop->config = &default_config;
  }

  if (siop->state == SIO_STOP) {

    /* Enables the peripheral.*/
    if (false) {
    }
#if STM32_SIO_USE_USART1 == TRUE
    else if (&SIOD1 == siop) {
      rccResetUSART1();
      rccEnableUSART1(true);
    }
#endif
#if STM32_SIO_USE_USART2 == TRUE
    else if (&SIOD2 == siop) {
      rccResetUSART2();
      rccEnableUSART2(true);
    }
#endif
#if STM32_SIO_USE_USART3 == TRUE
    else if (&SIOD3 == siop) {
      rccResetUSART3();
      rccEnableUSART3(true);
    }
#endif
#if STM32_SIO_USE_UART4 == TRUE
    else if (&SIOD4 == siop) {
      rccResetUART4();
      rccEnableUART4(true);
    }
#endif
#if STM32_SIO_USE_UART5 == TRUE
    else if (&SIOD5 == siop) {
      rccResetUART5();
      rccEnableUART5(true);
    }
#endif
#if STM32_SIO_USE_USART6 == TRUE
    else if (&SIOD6 == siop) {
      rccResetUSART6();
      rccEnableUSART6(true);
    }
#endif
#if STM32_SIO_USE_UART7 == TRUE
    else if (&SIOD7 == siop) {
      rccResetUART7();
      rccEnableUART7(true);
    }
#endif
#if STM32_SIO_USE_UART8 == TRUE
    else if (&SIOD8 == siop) {
      rccResetUART8();
      rccEnableUART8(true);
    }
#endif
#if STM32_SIO_USE_UART9 == TRUE
    else if (&SIOD9 == siop) {
      rccResetUART9();
      rccEnableUART9(true);
    }
#endif
#if STM32_SIO_USE_USART10 == TRUE
    else if (&SIOD10 == siop) {
      rccResetUSART10();
      rccEnableUSART10(true);
    }
#endif
#if STM32_SIO_USE_LPUART1 == TRUE
    else if (&LPSIOD1 == siop) {
      rccResetLPUART1();
      rccEnableLPUART1(true);
    }
#endif
    else {
      osalDbgAssert(false, "invalid SIO instance");
      return HAL_RET_IS_INVALID;
    }
  }

  /* Configures the peripheral.*/
  usart_init(siop);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the SIO peripheral.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
void sio_lld_stop(SIODriver *siop) {

  if (siop->state == SIO_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
    if (false) {
    }
#if STM32_SIO_USE_USART1 == TRUE
    else if (&SIOD1 == siop) {
      rccResetUSART1();
      rccDisableUSART1();
    }
#endif
#if STM32_SIO_USE_USART2 == TRUE
    else if (&SIOD2 == siop) {
      rccResetUSART2();
      rccDisableUSART2();
    }
#endif
#if STM32_SIO_USE_USART3 == TRUE
    else if (&SIOD3 == siop) {
      rccResetUSART3();
      rccDisableUSART3();
    }
#endif
#if STM32_SIO_USE_UART4 == TRUE
    else if (&SIOD4 == siop) {
      rccResetUART4();
      rccDisableUART4();
    }
#endif
#if STM32_SIO_USE_UART5 == TRUE
    else if (&SIOD5 == siop) {
      rccResetUART5();
      rccDisableUART5();
    }
#endif
#if STM32_SIO_USE_USART6 == TRUE
    else if (&SIOD6 == siop) {
      rccResetUSART6();
      rccDisableUSART6();
    }
#endif
#if STM32_SIO_USE_UART7 == TRUE
    else if (&SIOD7 == siop) {
      rccResetUART7();
      rccDisableUART7();
    }
#endif
#if STM32_SIO_USE_UART8 == TRUE
    else if (&SIOD8 == siop) {
      rccResetUART8();
      rccDisableUART8();
    }
#endif
#if STM32_SIO_USE_UART9 == TRUE
    else if (&SIOD9 == siop) {
      rccResetUART9();
      rccDisableUART9();
    }
#endif
#if STM32_SIO_USE_USART10 == TRUE
    else if (&SIOD10 == siop) {
      rccResetUSART10();
      rccDisableUSART10();
    }
#endif
#if STM32_SIO_USE_LPUART1 == TRUE
    else if (&LPSIOD1 == siop) {
      rccResetLPUART1();
      rccDisableLPUART1();
    }
#endif
    else {
      osalDbgAssert(false, "invalid SIO instance");
    }
  }
}

/**
 * @brief   Enable flags change notification.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 */
void sio_lld_update_enable_flags(SIODriver *siop) {
  uint32_t cr1, cr2, cr3;

  cr1 = siop->usart->CR1 & ~(USART_CR1_TXEIE_TXFNFIE | USART_CR1_RXNEIE_RXFNEIE |
                             USART_CR1_IDLEIE | USART_CR1_TCIE |
                             USART_CR1_PEIE);
  cr2 = siop->usart->CR2 & ~(USART_CR2_LBDIE);
  cr3 = siop->usart->CR3 & ~(USART_CR3_RXFTIE | USART_CR3_TXFTIE | USART_CR3_EIE);

  cr1 |= __sio_reloc_field(siop->enabled, SIO_EV_RXIDLE,     SIO_EV_RXIDLE_POS,     USART_CR1_IDLEIE_Pos) |
         __sio_reloc_field(siop->enabled, SIO_EV_TXDONE,     SIO_EV_TXDONE_POS,     USART_CR1_TCIE_Pos)   |
         __sio_reloc_field(siop->enabled, SIO_EV_PARITY_ERR, SIO_EV_PARITY_ERR_POS, USART_CR1_PEIE_Pos);
  cr2 |= __sio_reloc_field(siop->enabled, SIO_EV_RXBREAK,    SIO_EV_RXBREAK_POS,    USART_CR2_LBDIE_Pos);

  /* The following 3 are grouped.*/
  if ((siop->enabled & (SIO_EV_FRAMING_ERR |
                        SIO_EV_OVERRUN_ERR |
                        SIO_EV_NOISE_ERR)) != 0U) {
    cr3 |= USART_CR3_EIE;
  }

#if STM32_USART_MIXED == TRUE
  /* Managing the case with basic and full USARTs.*/
  if (siop->has_fifo) {
#endif
    /* Special case when TX FIFO threshold is set "non full".*/
    if ((siop->config->cr3 & USART_CR3_TXFTCFG_Msk) == USART_CR3_TXFTCFG_NONFULL) {
      cr1 |= __sio_reloc_field(siop->enabled, SIO_EV_TXNOTFULL, SIO_EV_TXNOTFULL_POS, USART_CR1_TXEIE_TXFNFIE_Pos);
    }
    else {
      cr3 |= __sio_reloc_field(siop->enabled, SIO_EV_TXNOTFULL, SIO_EV_TXNOTFULL_POS, USART_CR3_TXFTIE_Pos);
    }

    /* Special case when RX FIFO threshold is set "non-empty".*/
    if ((siop->config->cr3 & USART_CR3_RXFTCFG_Msk) == USART_CR3_RXFTCFG_NONEMPTY) {
      cr1 |= __sio_reloc_field(siop->enabled, SIO_EV_RXNOTEMPY, SIO_EV_RXNOTEMPY_POS, USART_CR1_RXNEIE_RXFNEIE_Pos);
    }
    else {
      cr3 |= __sio_reloc_field(siop->enabled, SIO_EV_RXNOTEMPY, SIO_EV_RXNOTEMPY_POS, USART_CR3_RXFTIE_Pos);
    }
#if STM32_USART_MIXED == TRUE
  }
  else {
    cr1 |= __sio_reloc_field(siop->enabled, SIO_EV_TXNOTFULL, SIO_EV_TXNOTFULL_POS, USART_CR1_TXEIE_TXFNFIE_Pos);
    cr1 |= __sio_reloc_field(siop->enabled, SIO_EV_RXNOTEMPY, SIO_EV_RXNOTEMPY_POS, USART_CR1_RXNEIE_RXFNEIE_Pos);
  }
#endif

  /* Setting up the operation.*/
  siop->usart->CR1 = cr1;
  siop->usart->CR2 = cr2;
  siop->usart->CR3 = cr3;
}

/**
 * @brief   Get and clears SIO error event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @notapi
 */
sioevents_t sio_lld_get_and_clear_errors(SIODriver *siop) {
  uint32_t isr;
  sioevents_t errors;

  /* Getting all error ISR flags (and only those).
     NOTE: Do not trust the position of other bits in ISR/ICR because
           some scientist decided to use different positions for some
           of them.*/
  isr = siop->usart->ISR & SIO_LLD_ISR_RX_ERRORS;

  /* Clearing captured events.*/
  siop->usart->ICR = isr;

  /* Status flags cleared, now the error-related interrupts can be
     enabled again.*/
  usart_enable_rx_errors_irq(siop);

  /* Translating the status flags in SIO events.*/
  errors = __sio_reloc_field(isr, USART_ISR_ONFP_Msk,  USART_ISR_ONFP_Pos,  SIO_EV_ALL_ERRORS_POS) |
           __sio_reloc_field(isr, USART_ISR_LBDF_Msk,  USART_ISR_LBDF_Pos,  SIO_EV_RXBREAK_POS);

  return errors;
}

/**
 * @brief   Get and clears SIO event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @notapi
 */
sioevents_t sio_lld_get_and_clear_events(SIODriver *siop) {
  uint32_t isr;
  sioevents_t events;

  /* Getting all ISR flags.
     NOTE: Do not trust the position of other bits in ISR/ICR because
           some scientist decided to use different positions for some
           of them.*/
  isr = siop->usart->ISR & (SIO_LLD_ISR_RX_ERRORS |
                            USART_ISR_RXNE_RXFNE  |
                            USART_ISR_IDLE        |
                            USART_ISR_TXE_TXFNF   |
                            USART_ISR_TC);

  /* Clearing captured events.*/
  siop->usart->ICR = isr;

  /* Status flags cleared, now the RX-related interrupts can be
     enabled again.*/
  usart_enable_rx_irq(siop);
  usart_enable_rx_errors_irq(siop);

  /* Translating the status flags in SIO events.*/
  events = __sio_reloc_field(isr, USART_ISR_RXNE_RXFNE_Msk, USART_ISR_RXNE_RXFNE_Pos,   SIO_EV_RXNOTEMPY_POS)  |
           __sio_reloc_field(isr, USART_ISR_TXE_TXFNF_Msk,  USART_ISR_TXE_TXFNF_Pos,    SIO_EV_TXNOTFULL_POS)  |
           __sio_reloc_field(isr, USART_ISR_TC_Msk,         USART_ISR_TC_Pos,           SIO_EV_TXDONE_POS)     |
           __sio_reloc_field(isr, USART_ISR_IONFP_Msk,      USART_ISR_IONFP_Pos,        SIO_EV_ALL_ERRORS_POS) |
           __sio_reloc_field(isr, USART_ISR_LBDF_Msk,       USART_ISR_LBDF_Pos,         SIO_EV_RXBREAK_POS);

  return events;
}

/**
 * @brief   Returns the pending SIO event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @notapi
 */
sioevents_t sio_lld_get_events(SIODriver *siop) {
  uint32_t isr;
  sioevents_t events;

  /* Getting all ISR flags.*/
  isr = siop->usart->ISR & (SIO_LLD_ISR_RX_ERRORS |
                            USART_ISR_RXNE_RXFNE  |
                            USART_ISR_IDLE        |
                            USART_ISR_TXE_TXFNF   |
                            USART_ISR_TC);

  /* Translating the status flags in SIO events.*/
  events = __sio_reloc_field(isr, USART_ISR_RXNE_RXFNE_Msk, USART_ISR_RXNE_RXFNE_Pos,   SIO_EV_RXNOTEMPY_POS)  |
           __sio_reloc_field(isr, USART_ISR_TXE_TXFNF_Msk,  USART_ISR_TXE_TXFNF_Pos,    SIO_EV_TXNOTFULL_POS)  |
           __sio_reloc_field(isr, USART_ISR_TC_Msk,         USART_ISR_TC_Pos,           SIO_EV_TXDONE_POS)     |
           __sio_reloc_field(isr, USART_ISR_IONFP_Msk,      USART_ISR_IONFP_Pos,        SIO_EV_ALL_ERRORS_POS) |
           __sio_reloc_field(isr, USART_ISR_LBDF_Msk,       USART_ISR_LBDF_Pos,         SIO_EV_RXBREAK_POS);

  return events;
}

/**
 * @brief   Reads data from the RX FIFO.
 * @details The function is not blocking, it writes frames until there
 *          is space available without waiting.
 *
 * @param[in] siop          pointer to an @p SIODriver structure
 * @param[in] buffer        pointer to the buffer for read frames
 * @param[in] n             maximum number of frames to be read
 * @return                  The number of frames copied from the buffer.
 * @retval 0                if the RX FIFO is empty.
 */
size_t sio_lld_read(SIODriver *siop, uint8_t *buffer, size_t n) {
  size_t rd;

  rd = 0U;
  while (true) {

    /* If the RX FIFO has been emptied then the RX FIFO and IDLE interrupts
       are enabled again.*/
    if (sio_lld_is_rx_empty(siop)) {
      usart_enable_rx_irq(siop);
      break;
    }

    /* Buffer filled condition.*/
    if (rd >= n) {
      break;
    }

    *buffer++ = (uint8_t)siop->usart->RDR;
    rd++;
  }

  return rd;
}

/**
 * @brief   Writes data into the TX FIFO.
 * @details The function is not blocking, it writes frames until there
 *          is space available without waiting.
 *
 * @param[in] siop          pointer to an @p SIODriver structure
 * @param[in] buffer        pointer to the buffer for read frames
 * @param[in] n             maximum number of frames to be written
 * @return                  The number of frames copied from the buffer.
 * @retval 0                if the TX FIFO is full.
 */
size_t sio_lld_write(SIODriver *siop, const uint8_t *buffer, size_t n) {
  size_t wr;

  wr = 0U;
  while (true) {

    /* If the TX FIFO has been filled then the interrupt is enabled again.*/
    if (sio_lld_is_tx_full(siop)) {
      usart_enable_tx_irq(siop);
      break;
    }

    /* Buffer emptied condition.*/
    if (wr >= n) {
      break;
    }

    siop->usart->TDR = (uint32_t)*buffer++;
    wr++;
  }

  /* The transmit complete interrupt is always re-enabled on write.*/
  usart_enable_tx_end_irq(siop);

  return wr;
}

/**
 * @brief   Returns one frame from the RX FIFO.
 * @note    If the FIFO is empty then the returned value is unpredictable.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The frame from RX FIFO.
 *
 * @notapi
 */
msg_t sio_lld_get(SIODriver *siop) {
  msg_t msg;

  msg = (msg_t)siop->usart->RDR;

  /* If the RX FIFO has been emptied then the interrupt is enabled again.*/
  if (sio_lld_is_rx_empty(siop)) {
    usart_enable_rx_irq(siop);
  }

  return msg;
}

/**
 * @brief   Pushes one frame into the TX FIFO.
 * @note    If the FIFO is full then the behavior is unpredictable.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] data      frame to be written
 *
 * @notapi
 */
void sio_lld_put(SIODriver *siop, uint_fast16_t data) {

  siop->usart->TDR = data;

  /* If the TX FIFO has been filled then the interrupt is enabled again.*/
  if (sio_lld_is_tx_full(siop)) {
    usart_enable_tx_irq(siop);
  }

  /* The transmit complete interrupt is always re-enabled on write.*/
  usart_enable_tx_end_irq(siop);
}

/**
 * @brief   Control operation on a serial port.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] operation control operation code
 * @param[in,out] arg   operation argument
 *
 * @return              The control operation status.
 * @retval MSG_OK       in case of success.
 * @retval MSG_TIMEOUT  in case of operation timeout.
 * @retval MSG_RESET    in case of operation reset.
 *
 * @notapi
 */
msg_t sio_lld_control(SIODriver *siop, unsigned int operation, void *arg) {

  (void)siop;
  (void)operation;
  (void)arg;

  return MSG_OK;
}

/**
 * @brief   Serves an USART interrupt.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
void sio_lld_serve_interrupt(SIODriver *siop) {
  USART_TypeDef *u = siop->usart;
  uint32_t cr1, cr2, cr3, isr, isrmask;

  osalDbgAssert(siop->state == SIO_READY, "invalid state");

  /* Read on control registers.*/
  cr1 = u->CR1;
  cr2 = u->CR2;
  cr3 = u->CR3;

  /* Calculating the mask of status bits that should be processed according
     to the state of the various CRx registers.*/
  isrmask = __sio_reloc_field(cr1, USART_CR1_TXEIE_TXFNFIE,     USART_CR1_TXEIE_TXFNFIE_Pos,    USART_ISR_TXE_TXFNF_Pos)  |
            __sio_reloc_field(cr1, USART_CR1_RXNEIE_RXFNEIE,    USART_CR1_RXNEIE_RXFNEIE_Pos,   USART_ISR_RXNE_RXFNE_Pos) |
             /* NOTE: ORE interrupt also enabled by USART_CR1_RXNEIE_RXFNEIE, not just USART_CR3_EIE.*/
            __sio_reloc_field(cr1, USART_CR1_RXNEIE_RXFNEIE,    USART_CR1_RXNEIE_RXFNEIE_Pos,   USART_ISR_ORE_Pos)  |
            __sio_reloc_field(cr1, USART_CR1_IDLEIE,            USART_CR1_IDLEIE_Pos,           USART_ISR_IDLE_Pos) |
            __sio_reloc_field(cr1, USART_CR1_TCIE,              USART_CR1_TCIE_Pos,             USART_ISR_TC_Pos)   |
            __sio_reloc_field(cr1, USART_CR1_PEIE,              USART_CR1_PEIE_Pos,             USART_ISR_PE_Pos)   |
            __sio_reloc_field(cr2, USART_CR2_LBDIE,             USART_CR2_LBDIE_Pos,            USART_ISR_LBDF_Pos) |
            __sio_reloc_field(cr3, USART_CR3_RXFTIE,            USART_CR3_RXFTIE_Pos,           USART_ISR_RXNE_RXFNE_Pos) |
            __sio_reloc_field(cr3, USART_CR3_TXFTIE,            USART_CR3_TXFTIE_Pos,           USART_ISR_TXE_TXFNF_Pos);
  if ((cr3 & USART_CR3_EIE) != 0U) {
    isrmask |= USART_ISR_NE | USART_ISR_FE | USART_ISR_ORE;
  }

  /* Status flags to be processed.*/
  isr = u->ISR & isrmask;
  if (isr != 0U) {

    /* Error flags handled as a group.
       NOTE: LBDF is mapped here too, so RX paths stay quiescent until the
             application clears errors.*/
    if ((isr & SIO_LLD_ISR_RX_ERRORS) != 0U) {
      /* NOTE: We could get here because various causes: PEIE, EIE, LBDIE and
               RXFNEIE. Because RXFNEIE we can get here even when the
               application has not enabled errors notifications.*/
#if SIO_USE_SYNCHRONIZATION
      /* The idle flag is forcibly cleared when an RX error event is
         detected.*/
      u->ICR = USART_ICR_IDLECF;
#endif

      /* All RX-related interrupt sources disabled.*/
      cr1 &= ~(USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE);
      cr2 &= ~(USART_CR2_LBDIE);
      cr3 &= ~(USART_CR3_EIE | USART_CR3_RXFTIE);

      /* Notifying the application only if it is interested in errors.*/
      if ((siop->enabled & SIO_EV_ALL_ERRORS) != (sioevents_t)0) {
        __sio_wakeup_errors(siop);
      }
    }
    /* If there are no errors then we check for the other RX-related
       status flags.*/
    else {
      /* Idle RX flag. Note: At start the USART will produce an IDLE interrupt.*/
      if ((isr & USART_ISR_IDLE) != 0U) {

        /* Interrupt source disabled.*/
        cr1 &= ~USART_CR1_IDLEIE;

        /* Conditionally enable interrupt on first character received after idle.
           This is required where a USART FIFO threshold set at minimum may be
           greater than one character thus a threshold interrupt will not be
           triggered at the next character. Any configuration setting other
           than non-empty will be handled according the specified threshold.
           Only enable if RXNOTEMPTY events are enabled. */
        if ((siop->enabled & SIO_EV_RXNOTEMPY) != 0U) {
          if ((siop->config->cr3 & USART_CR3_RXFTCFG_Msk) == USART_CR3_RXFTCFG_NONEMPTY) {
            cr1 |= USART_CR1_RXNEIE_RXFNEIE;
          }
          else {
            cr3 |= USART_CR3_RXFTIE;
          }
        }

        /* Waiting thread woken, if any.*/
        __sio_wakeup_rxidle(siop);
      }

      /* RX FIFO is non-empty.
         NOTE: Checking this flag instead of USART_ISR_RXFT because we want
         to greedily empty the RX FIFO, this could prevent more interrupts
         later. Additionally, this is valid for both full and basic USARTs.*/
      if ((isr & USART_ISR_RXNE_RXFNE) != 0U) {

#if SIO_USE_SYNCHRONIZATION
        /* The idle flag is forcibly cleared when an RX data event is
           detected.*/
        u->ICR = USART_ICR_IDLECF;
#endif

        /* Both possible interrupt sources disabled.*/
        cr3 &= ~USART_CR3_RXFTIE;
        cr1 &= ~USART_CR1_RXNEIE_RXFNEIE;

        /* Waiting thread woken, if any.*/
        __sio_wakeup_rx(siop);
      }
    }

    /* TX FIFO is non-full.
       NOTE: Checking this flag instead of USART_ISR_TXFT because we want
       to greedily fill the TX FIFO, this could prevent more interrupts
       later. Additionally, this is valid for both full and basic USARTs.*/
    if ((isr & USART_ISR_TXE_TXFNF) != 0U) {

      /* Both possible interrupt sources disabled.*/
      cr3 &= ~USART_CR3_TXFTIE;
      cr1 &= ~USART_CR1_TXEIE_TXFNFIE;

      /* Waiting thread woken, if any.*/
      __sio_wakeup_tx(siop);
    }

    /* Physical transmission end.*/
    if ((isr & USART_ISR_TC) != 0U) {

      /* Interrupt source disabled.*/
      cr1 &= ~USART_CR1_TCIE;

      /* Waiting thread woken, if any.*/
      __sio_wakeup_txend(siop);
    }

    /* Updating control registers, some sources could have been disabled.*/
    u->CR1 = cr1;
    u->CR2 = cr2;
    u->CR3 = cr3;

    /* The callback is invoked.*/
    __sio_callback(siop);
  }
  else {
    osalDbgAssert(false, "spurious interrupt");
  }
}

#endif /* HAL_USE_SIO == TRUE */

/** @} */
