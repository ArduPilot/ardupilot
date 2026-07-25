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
 * @file    GPIOv1/hal_pal_lld.c
 * @brief   STM32 PAL low level driver code.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if STM32_HAS_GPIOG
#define APB2_EN_MASK  (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |            \
                       RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN |            \
                       RCC_APB2ENR_IOPEEN | RCC_APB2ENR_IOPFEN |            \
                       RCC_APB2ENR_IOPGEN | RCC_APB2ENR_AFIOEN)
#elif STM32_HAS_GPIOE
#define APB2_EN_MASK  (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |            \
                       RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN |            \
                       RCC_APB2ENR_IOPEEN | RCC_APB2ENR_AFIOEN)
#else
#define APB2_EN_MASK  (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |            \
                       RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN |            \
                       RCC_APB2ENR_AFIOEN)
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Event records for the 16 GPIO EXTI channels.
 */
palevent_t _pal_events[16];
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   STM32 I/O ports configuration.
 * @details Ports A-D(E, F, G) clocks enabled, AFIO clock enabled.
 *
 * @param[in] config    the STM32 ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {

#if PAL_USE_CALLBACKS || PAL_USE_WAIT || defined(__DOXYGEN__)
  unsigned i;

  for (i = 0; i < 16; i++) {
    _pal_init_event(i);
  }
#endif

  /*
   * Enables the GPIO related clocks.
   */
  rccEnableAPB2(APB2_EN_MASK, true);

  /*
   * Initial GPIO setup.
   */
  GPIOA->ODR = config->PAData.odr;
  GPIOA->CRH = config->PAData.crh;
  GPIOA->CRL = config->PAData.crl;
  GPIOB->ODR = config->PBData.odr;
  GPIOB->CRH = config->PBData.crh;
  GPIOB->CRL = config->PBData.crl;
  GPIOC->ODR = config->PCData.odr;
  GPIOC->CRH = config->PCData.crh;
  GPIOC->CRL = config->PCData.crl;
  GPIOD->ODR = config->PDData.odr;
  GPIOD->CRH = config->PDData.crh;
  GPIOD->CRL = config->PDData.crl;
#if STM32_HAS_GPIOE || defined(__DOXYGEN__)
  GPIOE->ODR = config->PEData.odr;
  GPIOE->CRH = config->PEData.crh;
  GPIOE->CRL = config->PEData.crl;
#if STM32_HAS_GPIOF || defined(__DOXYGEN__)
  GPIOF->ODR = config->PFData.odr;
  GPIOF->CRH = config->PFData.crh;
  GPIOF->CRL = config->PFData.crl;
#if STM32_HAS_GPIOG || defined(__DOXYGEN__)
  GPIOG->ODR = config->PGData.odr;
  GPIOG->CRH = config->PGData.crh;
  GPIOG->CRL = config->PGData.crl;
#endif
#endif
#endif
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    @p PAL_MODE_UNCONNECTED is implemented as push pull output at 2MHz.
 * @note    Writing on pads programmed as pull-up or pull-down has the side
 *          effect to modify the resistor setting because the output latched
 *          data is used for the resistor selection.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {
  static const uint8_t cfgtab[] = {
    4,          /* PAL_MODE_RESET, implemented as input.*/
    2,          /* PAL_MODE_UNCONNECTED, implemented as push pull output 2MHz.*/
    4,          /* PAL_MODE_INPUT */
    8,          /* PAL_MODE_INPUT_PULLUP */
    8,          /* PAL_MODE_INPUT_PULLDOWN */
    0,          /* PAL_MODE_INPUT_ANALOG */
    3,          /* PAL_MODE_OUTPUT_PUSHPULL, 50MHz.*/
    7,          /* PAL_MODE_OUTPUT_OPENDRAIN, 50MHz.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    8,          /* Reserved.*/
    0xB,        /* PAL_MODE_STM32_ALTERNATE_PUSHPULL, 50MHz.*/
    0xF,        /* PAL_MODE_STM32_ALTERNATE_OPENDRAIN, 50MHz.*/
  };
  uint32_t mh, ml, crh, crl, cfg;
  unsigned i;

  if (mode == PAL_MODE_INPUT_PULLUP)
    port->BSRR = mask;
  else if (mode == PAL_MODE_INPUT_PULLDOWN)
    port->BRR = mask;
  cfg = cfgtab[mode];
  mh = ml = crh = crl = 0;
  for (i = 0; i < 8; i++) {
    ml <<= 4;
    mh <<= 4;
    crl <<= 4;
    crh <<= 4;
    if ((mask & 0x0080) == 0)
      ml |= 0xf;
    else
      crl |= cfg;
    if ((mask & 0x8000) == 0)
      mh |= 0xf;
    else
      crh |= cfg;
    mask <<= 1;
  }
  port->CRH = (port->CRH & mh) | crh;
  port->CRL = (port->CRL & ml) | crl;
}

#if PAL_USE_CALLBACKS || PAL_USE_WAIT || defined(__DOXYGEN__)
/**
 * @brief   Pad event enable.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad event mode
 *
 * @notapi
 */
void _pal_lld_enablepadevent(ioportid_t port,
                             iopadid_t pad,
                             ioeventmode_t mode) {

  uint32_t padmask, cridx, croff, crmask, portidx;

  /* Mask of the pad.*/
  padmask = 1U << (uint32_t)pad;

  /* Multiple channel setting of the same channel not allowed, first disable
     it. This is done because on STM32 the same channel cannot be mapped on
     multiple ports.*/
  osalDbgAssert(((EXTI->RTSR & padmask) == 0U) &&
                ((EXTI->FTSR & padmask) == 0U), "channel already in use");

  /* Index and mask of the SYSCFG CR register to be used.*/
  cridx  = (uint32_t)pad >> 2U;
  croff = ((uint32_t)pad & 3U) * 4U;
  crmask = ~(0xFU << croff);

  /* Port index is obtained assuming that GPIO ports are placed at regular
     0x400 intervals in memory space. So far this is true for all devices.*/
  portidx = (((uint32_t)port - (uint32_t)GPIOA) >> 10U) & 0xFU;

  /* Port selection in SYSCFG.*/
  AFIO->EXTICR[cridx] = (AFIO->EXTICR[cridx] & crmask) | (portidx << croff);

  /* Programming edge registers.*/
  if (mode & PAL_EVENT_MODE_RISING_EDGE)
    EXTI->RTSR |= padmask;
  else
    EXTI->RTSR &= ~padmask;
  if (mode & PAL_EVENT_MODE_FALLING_EDGE)
    EXTI->FTSR |= padmask;
  else
    EXTI->FTSR &= ~padmask;

  /* Programming interrupt and event registers.*/
  EXTI->IMR |= padmask;
  EXTI->EMR &= ~padmask;
}

/**
 * @brief   Pad event disable.
 * @details This function disables previously programmed event callbacks.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
void _pal_lld_disablepadevent(ioportid_t port, iopadid_t pad) {
  uint32_t padmask, rtsr1, ftsr1;

  rtsr1 = EXTI->RTSR;
  ftsr1 = EXTI->FTSR;

  /* Mask of the pad.*/
  padmask = 1U << (uint32_t)pad;

  /* If either RTRS1 or FTSR1 is enabled then the channel is in use.*/
  if (((rtsr1 | ftsr1) & padmask) != 0U) {
    uint32_t cridx, croff, crport, portidx;

    /* Index and mask of the SYSCFG CR register to be used.*/
    cridx  = (uint32_t)pad >> 2U;
    croff = ((uint32_t)pad & 3U) * 4U;

    /* Port index is obtained assuming that GPIO ports are placed at regular
       0x400 intervals in memory space. So far this is true for all devices.*/
    portidx = (((uint32_t)port - (uint32_t)GPIOA) >> 10U) & 0xFU;

    crport = (AFIO->EXTICR[cridx] >> croff) & 0xFU;

    osalDbgAssert(crport == portidx, "channel mapped on different port");

    /* Disabling channel.*/
    EXTI->IMR  &= ~padmask;
    EXTI->EMR  &= ~padmask;
    EXTI->RTSR  = rtsr1 & ~padmask;
    EXTI->FTSR  = ftsr1 & ~padmask;
    EXTI->PR    = padmask;

#if PAL_USE_CALLBACKS || PAL_USE_WAIT
  /* Callback cleared and/or thread reset.*/
  _pal_clear_event(pad);
#endif
  }
}
#endif /* PAL_USE_CALLBACKS || PAL_USE_WAIT */

#endif /* HAL_USE_PAL */

/** @} */
