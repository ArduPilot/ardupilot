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
 * @file    RCCv1/stm32_pll2.inc
 * @brief   Shared PLL2 handler.
 *
 * @addtogroup STM32_PLL2_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    PLL helpers
 */
#define STM32_PLL2VCOSEL_WIDE       (0U << RCC_PLL2CFGR_PLL2VCOSEL_Pos)
#define STM32_PLL2VCOSEL_MEDIUM     (1U << RCC_PLL2CFGR_PLL2VCOSEL_Pos)
#define STM32_PLL2RGE_0             (0U << RCC_PLL2CFGR_PLL2RGE_Pos)
#define STM32_PLL2RGE_1             (1U << RCC_PLL2CFGR_PLL2RGE_Pos)
#define STM32_PLL2RGE_2             (2U << RCC_PLL2CFGR_PLL2RGE_Pos)
#define STM32_PLL2RGE_3             (3U << RCC_PLL2CFGR_PLL2RGE_Pos)
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on registry.*/
#if !defined(STM32_RCC_HAS_PLL2)
#error "STM32_RCC_HAS_PLL2 not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL2 && !defined(STM32_RCC_PLL2_HAS_P)
#error "STM32_RCC_PLL2_HAS_P not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL2 && !defined(STM32_RCC_PLL2_HAS_Q)
#error "STM32_RCC_PLL2_HAS_Q not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL2 && !defined(STM32_RCC_PLL2_HAS_R)
#error "STM32_RCC_PLL2_HAS_R not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL2

/* Checks on configurations.*/
#if !defined(STM32_PLL2SRC)
#error "STM32_PLL2SRC not defined in mcuconf.h"
#endif

#if !defined(STM32_PLL2M_VALUE)
#error "STM32_PLL2M_VALUE not defined in mcuconf.h"
#endif

#if !defined(STM32_PLL2N_VALUE)
#error "STM32_PLL2N_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL2_HAS_P && !defined(STM32_PLL2P_VALUE)
#error "STM32_PLL2P_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL2_HAS_Q && !defined(STM32_PLL2Q_VALUE)
#error "STM32_PLL2Q_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL2_HAS_R && !defined(STM32_PLL2R_VALUE)
#error "STM32_PLL2R_VALUE not defined in mcuconf.h"
#endif

/* Check on valid values.*/
#if !defined(STM32_PLL2M_VALUE_MAX)
#error "STM32_PLL2M_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2M_VALUE_MIN)
#error "STM32_PLL2M_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2N_ODDVALID)
#error "STM32_PLL2N_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2N_VALUE_MAX)
#error "STM32_PLL2N_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2N_VALUE_MIN)
#error "STM32_PLL2N_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2P_ODDVALID)
#error "STM32_PLL2P_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2P_VALUE_MAX)
#error "STM32_PLL2P_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2P_VALUE_MIN)
#error "STM32_PLL2P_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2Q_ODDVALID)
#error "STM32_PLL2Q_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2Q_VALUE_MAX)
#error "STM32_PLL2Q_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2Q_VALUE_MIN)
#error "STM32_PLL2Q_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2R_ODDVALID)
#error "STM32_PLL2R_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2R_VALUE_MAX)
#error "STM32_PLL2R_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL2R_VALUE_MIN)
#error "STM32_PLL2R_VALUE_MIN not defined in hal_lld.h"
#endif

/* Check on limits.*/
#if !defined(STM32_PLLIN_MAX)
#error "STM32_PLLIN_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLIN_MIN)
#error "STM32_PLLIN_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLVCO_MAX)
#error "STM32_PLLVCO_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLVCO_MIN)
#error "STM32_PLLVCO_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLP_MAX)
#error "STM32_PLLP_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLP_MIN)
#error "STM32_PLLP_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLQ_MAX)
#error "STM32_PLLQ_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLQ_MIN)
#error "STM32_PLLQ_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLR_MAX)
#error "STM32_PLLR_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLR_MIN)
#error "STM32_PLLR_MIN not defined in hal_lld.h"
#endif

/* Input checks.*/
#if !defined(STM32_PLL2IN)
#error "STM32_PLL2IN not defined in hal_lld.h"
#endif

#if !defined(STM32_ACTIVATE_PLL2)
#error "STM32_ACTIVATE_PLL2 not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL2_HAS_P && !defined(STM32_PLL2PEN)
#error "STM32_PLL2PEN not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL2_HAS_Q && !defined(STM32_PLL2QEN)
#error "STM32_PLL2QEN not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL2_HAS_R && !defined(STM32_PLL2REN)
#error "STM32_PLL2REN not defined in hal_lld.h"
#endif

#if STM32_ACTIVATE_PLL2 && (STM32_PLL2IN == 0)
#error "PLL2 activation required but no PLL2 clock selected"
#endif

/**
 * @brief   STM32_PLL2M field.
 */
#if ((STM32_PLL2M_VALUE >= STM32_PLL2M_VALUE_MIN) &&                        \
     (STM32_PLL2M_VALUE <= STM32_PLL2M_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL2M                 (STM32_PLL2M_VALUE << RCC_PLL2CFGR_PLL2M_Pos)

#else
#error "invalid STM32_PLL2M_VALUE value specified"
#endif

/**
 * @brief   PLL comparator input frequency.
 */
#define STM32_PLL2CLKIN             (STM32_PLL2IN / STM32_PLL2M_VALUE)

#if STM32_PLL2CLKIN < STM32_PLLIN_MIN
#error "STM32_PLL2CLKIN below acceptable range"

#elif STM32_PLL2CLKIN > STM32_PLLIN_MAX
#error "STM32_PLL2CLKIN above acceptable range"

#elif (STM32_PLL2CLKIN < STM32_PLLIN_THRESHOLD1) || defined(__DOXYGEN__)
#define STM32_PLL2RGE               STM32_PLL2RGE_0
#define STM32_PLL2VCOSEL            STM32_PLL2VCOSEL_MEDIUM
#define STM32_PLL2VCO_MIN           STM32_PLLVCO_MEDIUM_MIN
#define STM32_PLL2VCO_MAX           STM32_PLLVCO_MEDIUM_MAX

#elif STM32_PLL2CLKIN < STM32_PLLIN_THRESHOLD2
#define STM32_PLL2RGE               STM32_PLL2RGE_1
#define STM32_PLL2VCOSEL            STM32_PLL2VCOSEL_WIDE
#define STM32_PLL2VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL2VCO_MAX           STM32_PLLVCO_WIDE_MAX

#elif STM32_PLL2CLKIN < STM32_PLLIN_THRESHOLD3
#define STM32_PLL2RGE               STM32_PLL2RGE_2
#define STM32_PLL2VCOSEL            STM32_PLL2VCOSEL_WIDE
#define STM32_PLL2VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL2VCO_MAX           STM32_PLLVCO_WIDE_MAX

#else
#define STM32_PLL2RGE               STM32_PLL2RGE_3
#define STM32_PLL2VCOSEL            STM32_PLL2VCOSEL_WIDE
#define STM32_PLL2VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL2VCO_MAX           STM32_PLLVCO_WIDE_MAX
#endif

/**
 * @brief   STM32_PLL2N field.
 */
#if ((STM32_PLL2N_VALUE >= STM32_PLL2N_VALUE_MIN) &&                        \
     (STM32_PLL2N_VALUE <= STM32_PLL2N_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL2N                 ((STM32_PLL2N_VALUE - 1U) << RCC_PLL2DIVR_PLL2N_Pos)

#else
#error "invalid STM32_PLL2N_VALUE value specified"
#endif

/**
 * @brief   PLL2 VCO frequency.
 */
#define STM32_PLL2VCO               (STM32_PLL2CLKIN * STM32_PLL2N_VALUE)

/*
 * PLL2 VCO frequency range check.
 */
#if STM32_ACTIVATE_PLL2 &&                                                  \
    ((STM32_PLL2VCO < STM32_PLL2VCO_MIN) || (STM32_PLL2VCO > STM32_PLL2VCO_MAX))
#error "STM32_PLL2VCO outside acceptable range (STM32_PLL2VCO_MIN...STM32_PLL2VCO_MAX)"
#endif

/*---------------------------------------------------------------------------*/
/* P output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL2_HAS_P || defined(__DOXYGEN__)

#if !STM32_PLL2P_ODDVALID && ((STM32_PLL2P_VALUE & 1) != 0)
#error "odd STM32_PLL2P_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL2P field.
 */
#if ((STM32_PLL2P_VALUE >= STM32_PLL2P_VALUE_MIN) &&                        \
     (STM32_PLL2P_VALUE <= STM32_PLL2P_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL2P                 ((STM32_PLL2P_VALUE - 1) << RCC_PLL2DIVR_PLL2P_Pos)
#else
#error "out of range STM32_PLL2P_VALUE value specified"
#endif

/**
 * @brief   PLL2P output clock frequency.
 */
#define STM32_PLL2_P_CLKOUT         (STM32_PLL2VCO / STM32_PLL2P_VALUE)

/*
 * PLL2P output frequency range check.
 */
#if STM32_ACTIVATE_PLL2 &&                                                  \
    ((STM32_PLL2_P_CLKOUT < STM32_PLLP_MIN) ||                              \
     (STM32_PLL2_P_CLKOUT > STM32_PLLP_MAX))
#error "STM32_PLL2_P_CLKOUT outside acceptable range (STM32_PLLP_MIN...STM32_PLLP_MAX)"
#endif

#else /* !STM32_RCC_PLL2_HAS_P */
#define STM32_PLL2P                 0
#endif /* !STM32_RCC_PLL2_HAS_P */

/*---------------------------------------------------------------------------*/
/* Q output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL2_HAS_Q || defined(__DOXYGEN__)

#if !STM32_PLL2Q_ODDVALID && ((STM32_PLL2Q_VALUE & 1) != 0)
#error "odd STM32_PLL2Q_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL2Q field.
 */
#if ((STM32_PLL2Q_VALUE >= STM32_PLL2Q_VALUE_MIN) &&                        \
     (STM32_PLL2Q_VALUE <= STM32_PLL2Q_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL2Q                 ((STM32_PLL2Q_VALUE - 1) << RCC_PLL2DIVR_PLL2Q_Pos)
#else
#error "out of range STM32_PLL2Q_VALUE value specified"
#endif

/**
 * @brief   PLL2Q output clock frequency.
 */
#define STM32_PLL2_Q_CLKOUT         (STM32_PLL2VCO / STM32_PLL2Q_VALUE)

/*
 * PLL2P output frequency range check.
 */
#if STM32_ACTIVATE_PLL2 &&                                                  \
    ((STM32_PLL2_Q_CLKOUT < STM32_PLLQ_MIN) ||                              \
     (STM32_PLL2_Q_CLKOUT > STM32_PLLQ_MAX))
#error "STM32_PLL2_Q_CLKOUT outside acceptable range (STM32_PLLQ_MIN...STM32_PLLQ_MAX)"
#endif

#else /* !STM32_RCC_PLL2_HAS_Q */
#define STM32_PLL2Q                 0
#endif /* !STM32_RCC_PLL2_HAS_Q */

/*---------------------------------------------------------------------------*/
/* R output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL2_HAS_R || defined(__DOXYGEN__)

#if !STM32_PLL2R_ODDVALID && ((STM32_PLL2R_VALUE & 1) != 0)
#error "odd STM32_PLL2R_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL2R field.
 */
#if ((STM32_PLL2R_VALUE >= STM32_PLL2R_VALUE_MIN) &&                        \
     (STM32_PLL2R_VALUE <= STM32_PLL2R_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL2R                 ((STM32_PLL2R_VALUE - 1) << RCC_PLL2DIVR_PLL2R_Pos)
#else
#error "out of range STM32_PLL2R_VALUE value specified"
#endif

/**
 * @brief   PLL2R output clock frequency.
 */
#define STM32_PLL2_R_CLKOUT         (STM32_PLL2VCO / STM32_PLL2R_VALUE)

/*
 * PLL2R output frequency range check.
 */
#if STM32_ACTIVATE_PLL2 &&                                                  \
    ((STM32_PLL2_R_CLKOUT < STM32_PLLR_MIN) ||                              \
     (STM32_PLL2_R_CLKOUT > STM32_PLLR_MAX))
#error "STM32_PLL2_R_CLKOUT outside acceptable range (STM32_PLLR_MIN...STM32_PLLR_MAX)"
#endif

#else /* !STM32_RCC_PLL2_HAS_R */
#define STM32_PLL2R                 0
#endif /* !STM32_RCC_PLL2_HAS_R */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE void pll2_enable(void) {

   RCC->CR |= RCC_CR_PLL2ON;
}

 __STATIC_INLINE void pll2_disable(void) {

   RCC->CR &= ~RCC_CR_PLL2ON;
 }

 __STATIC_INLINE bool pll2_not_locked(void) {

   return (bool)((RCC->CR & RCC_CR_PLL2RDY) == 0U);
 }

__STATIC_INLINE void pll2_wait_lock(void) {

  while (pll2_not_locked()) {
    /* Waiting for PLL2 lock.*/
  }
}

#endif /* STM32_RCC_HAS_PLL2 */

__STATIC_INLINE void pll2_setup(uint32_t cfgr, uint32_t divr, uint32_t fracr) {

#if STM32_RCC_HAS_PLL2
  /* PLL2 activation.*/
  RCC->PLL2CFGR  = cfgr & ~RCC_PLL2CFGR_PLL2FRACEN_Msk;
  RCC->PLL2DIVR  = divr;
  RCC->PLL2FRACR = fracr;
  RCC->PLL2CFGR  = cfgr;
#else
  (void)cfgr;
  (void)divr;
  (void)fracr;
#endif
}

__STATIC_INLINE void pll2_init(void) {

#if STM32_RCC_HAS_PLL2
#if STM32_ACTIVATE_PLL2
  /* PLL2 activation.*/
  pll2_setup(STM32_PLL2REN     | STM32_PLL2QEN     |
             STM32_PLL2PEN     | STM32_PLL2M       |
             STM32_PLL2RGE     | STM32_PLL2VCOSEL  |
             STM32_PLL2SRC,
             STM32_PLL2R       | STM32_PLL2Q       |
             STM32_PLL2P       | STM32_PLL2N,
             0U); /* TODO FRACR */
  pll2_enable();
  pll2_wait_lock();
#endif
#endif
}

__STATIC_INLINE void pll2_deinit(void) {

#if STM32_RCC_HAS_PLL2
#if STM32_ACTIVATE_PLL2
  /* PLL2 de-activation.*/
  pll2_disable();
#endif
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
