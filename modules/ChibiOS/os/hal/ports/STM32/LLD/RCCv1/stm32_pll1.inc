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
 * @file    RCCv1/stm32_pll1.inc
 * @brief   Shared PLL1 handler.
 *
 * @addtogroup STM32_PLL1_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    PLL helpers
 */
#define STM32_PLL1VCOSEL_WIDE       (0U << RCC_PLL1CFGR_PLL1VCOSEL_Pos)
#define STM32_PLL1VCOSEL_MEDIUM     (1U << RCC_PLL1CFGR_PLL1VCOSEL_Pos)
#define STM32_PLL1RGE_0             (0U << RCC_PLL1CFGR_PLL1RGE_Pos)
#define STM32_PLL1RGE_1             (1U << RCC_PLL1CFGR_PLL1RGE_Pos)
#define STM32_PLL1RGE_2             (2U << RCC_PLL1CFGR_PLL1RGE_Pos)
#define STM32_PLL1RGE_3             (3U << RCC_PLL1CFGR_PLL1RGE_Pos)
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on registry.*/
#if !defined(STM32_RCC_HAS_PLL1)
#error "STM32_RCC_HAS_PLL1 not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL1 && !defined(STM32_RCC_PLL1_HAS_P)
#error "STM32_RCC_PLL1_HAS_P not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL1 && !defined(STM32_RCC_PLL1_HAS_Q)
#error "STM32_RCC_PLL1_HAS_Q not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL1 && !defined(STM32_RCC_PLL1_HAS_R)
#error "STM32_RCC_PLL1_HAS_R not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL1

/* Checks on configurations.*/
#if !defined(STM32_PLL1SRC)
#error "STM32_PLL1SRC not defined in mcuconf.h"
#endif

#if !defined(STM32_PLL1M_VALUE)
#error "STM32_PLL1M_VALUE not defined in mcuconf.h"
#endif

#if !defined(STM32_PLL1N_VALUE)
#error "STM32_PLL1N_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL1_HAS_P && !defined(STM32_PLL1P_VALUE)
#error "STM32_PLL1P_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL1_HAS_Q && !defined(STM32_PLL1Q_VALUE)
#error "STM32_PLL1Q_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL1_HAS_R && !defined(STM32_PLL1R_VALUE)
#error "STM32_PLL1R_VALUE not defined in mcuconf.h"
#endif

/* Check on valid values.*/
#if !defined(STM32_PLL1M_VALUE_MAX)
#error "STM32_PLL1M_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1M_VALUE_MIN)
#error "STM32_PLL1M_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1N_ODDVALID)
#error "STM32_PLL1N_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1N_VALUE_MAX)
#error "STM32_PLL1N_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1N_VALUE_MIN)
#error "STM32_PLL1N_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1P_ODDVALID)
#error "STM32_PLL1P_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1P_VALUE_MAX)
#error "STM32_PLL1P_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1P_VALUE_MIN)
#error "STM32_PLL1P_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1Q_ODDVALID)
#error "STM32_PLL1Q_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1Q_VALUE_MAX)
#error "STM32_PLL1Q_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1Q_VALUE_MIN)
#error "STM32_PLL1Q_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1R_ODDVALID)
#error "STM32_PLL1R_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1R_VALUE_MAX)
#error "STM32_PLL1R_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL1R_VALUE_MIN)
#error "STM32_PLL1R_VALUE_MIN not defined in hal_lld.h"
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
#if !defined(STM32_PLL1IN)
#error "STM32_PLL1IN not defined in hal_lld.h"
#endif

#if !defined(STM32_ACTIVATE_PLL1)
#error "STM32_ACTIVATE_PLL1 not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL1_HAS_P && !defined(STM32_PLL1PEN)
#error "STM32_PLL1PEN not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL1_HAS_Q && !defined(STM32_PLL1QEN)
#error "STM32_PLL1QEN not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL1_HAS_R && !defined(STM32_PLL1REN)
#error "STM32_PLL1REN not defined in hal_lld.h"
#endif

#if STM32_ACTIVATE_PLL1 && (STM32_PLL1IN == 0)
#error "PLL1 activation required but no PLL1 clock selected"
#endif

/**
 * @brief   STM32_PLL1M field.
 */
#if ((STM32_PLL1M_VALUE >= STM32_PLL1M_VALUE_MIN) &&                        \
     (STM32_PLL1M_VALUE <= STM32_PLL1M_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL1M                 (STM32_PLL1M_VALUE << RCC_PLL1CFGR_PLL1M_Pos)

#else
#error "invalid STM32_PLL1M_VALUE value specified"
#endif

/**
 * @brief   PLL comparator input frequency.
 */
#define STM32_PLL1CLKIN             (STM32_PLL1IN / STM32_PLL1M_VALUE)

#if STM32_PLL1CLKIN < STM32_PLLIN_MIN
#error "STM32_PLL1CLKIN below acceptable range"

#elif STM32_PLL1CLKIN > STM32_PLLIN_MAX
#error "STM32_PLL1CLKIN above acceptable range"

#elif (STM32_PLL1CLKIN < STM32_PLLIN_THRESHOLD1) || defined(__DOXYGEN__)
#define STM32_PLL1RGE               STM32_PLL1RGE_0
#define STM32_PLL1VCOSEL            STM32_PLL1VCOSEL_MEDIUM
#define STM32_PLL1VCO_MIN           STM32_PLLVCO_MEDIUM_MIN
#define STM32_PLL1VCO_MAX           STM32_PLLVCO_MEDIUM_MAX

#elif STM32_PLL1CLKIN < STM32_PLLIN_THRESHOLD2
#define STM32_PLL1RGE               STM32_PLL1RGE_1
#define STM32_PLL1VCOSEL            STM32_PLL1VCOSEL_WIDE
#define STM32_PLL1VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL1VCO_MAX           STM32_PLLVCO_WIDE_MAX

#elif STM32_PLL1CLKIN < STM32_PLLIN_THRESHOLD3
#define STM32_PLL1RGE               STM32_PLL1RGE_2
#define STM32_PLL1VCOSEL            STM32_PLL1VCOSEL_WIDE
#define STM32_PLL1VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL1VCO_MAX           STM32_PLLVCO_WIDE_MAX

#else
#define STM32_PLL1RGE               STM32_PLL1RGE_3
#define STM32_PLL1VCOSEL            STM32_PLL1VCOSEL_WIDE
#define STM32_PLL1VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL1VCO_MAX           STM32_PLLVCO_WIDE_MAX
#endif

/**
 * @brief   STM32_PLL1N field.
 */
#if ((STM32_PLL1N_VALUE >= STM32_PLL1N_VALUE_MIN) &&                        \
     (STM32_PLL1N_VALUE <= STM32_PLL1N_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL1N                 ((STM32_PLL1N_VALUE - 1U) << RCC_PLL1DIVR_PLL1N_Pos)

#else
#error "invalid STM32_PLL1N_VALUE value specified"
#endif

/**
 * @brief   PLL1 VCO frequency.
 */
#define STM32_PLL1VCO               (STM32_PLL1CLKIN * STM32_PLL1N_VALUE)

/*
 * PLL1 VCO frequency range check.
 */
#if STM32_ACTIVATE_PLL1 &&                                                  \
    ((STM32_PLL1VCO < STM32_PLL1VCO_MIN) || (STM32_PLL1VCO > STM32_PLL1VCO_MAX))
#error "STM32_PLL1VCO outside acceptable range (STM32_PLL1VCO_MIN...STM32_PLL1VCO_MAX)"
#endif

/*---------------------------------------------------------------------------*/
/* P output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL1_HAS_P || defined(__DOXYGEN__)

#if !STM32_PLL1P_ODDVALID && ((STM32_PLL1P_VALUE & 1) != 0)
#error "odd STM32_PLL1P_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL1P field.
 */
#if ((STM32_PLL1P_VALUE >= STM32_PLL1P_VALUE_MIN) &&                        \
     (STM32_PLL1P_VALUE <= STM32_PLL1P_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL1P                 ((STM32_PLL1P_VALUE - 1) << RCC_PLL1DIVR_PLL1P_Pos)
#else
#error "out of range STM32_PLL1P_VALUE value specified"
#endif

/**
 * @brief   PLL1P output clock frequency.
 */
#define STM32_PLL1_P_CLKOUT         (STM32_PLL1VCO / STM32_PLL1P_VALUE)

/*
 * PLL1P output frequency range check.
 */
#if STM32_ACTIVATE_PLL1 &&                                                  \
    ((STM32_PLL1_P_CLKOUT < STM32_PLLP_MIN) ||                              \
     (STM32_PLL1_P_CLKOUT > STM32_PLLP_MAX))
#error "STM32_PLL1_P_CLKOUT outside acceptable range (STM32_PLLP_MIN...STM32_PLLP_MAX)"
#endif

#else /* !STM32_RCC_PLL1_HAS_P */
#define STM32_PLL1P                 0
#endif /* !STM32_RCC_PLL1_HAS_P */

/*---------------------------------------------------------------------------*/
/* Q output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL1_HAS_Q || defined(__DOXYGEN__)

#if !STM32_PLL1Q_ODDVALID && ((STM32_PLL1Q_VALUE & 1) != 0)
#error "odd STM32_PLL1Q_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL1Q field.
 */
#if ((STM32_PLL1Q_VALUE >= STM32_PLL1Q_VALUE_MIN) &&                        \
     (STM32_PLL1Q_VALUE <= STM32_PLL1Q_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL1Q                 ((STM32_PLL1Q_VALUE - 1) << RCC_PLL1DIVR_PLL1Q_Pos)
#else
#error "out of range STM32_PLL1Q_VALUE value specified"
#endif

/**
 * @brief   PLL1Q output clock frequency.
 */
#define STM32_PLL1_Q_CLKOUT         (STM32_PLL1VCO / STM32_PLL1Q_VALUE)

/*
 * PLL1P output frequency range check.
 */
#if STM32_ACTIVATE_PLL1 &&                                                  \
    ((STM32_PLL1_Q_CLKOUT < STM32_PLLQ_MIN) ||                              \
     (STM32_PLL1_Q_CLKOUT > STM32_PLLQ_MAX))
#error "STM32_PLL1_Q_CLKOUT outside acceptable range (STM32_PLLQ_MIN...STM32_PLLQ_MAX)"
#endif

#else /* !STM32_RCC_PLL1_HAS_Q */
#define STM32_PLL1Q                 0
#endif /* !STM32_RCC_PLL1_HAS_Q */

/*---------------------------------------------------------------------------*/
/* R output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL1_HAS_R || defined(__DOXYGEN__)

#if !STM32_PLL1R_ODDVALID && ((STM32_PLL1R_VALUE & 1) != 0)
#error "odd STM32_PLL1R_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL1R field.
 */
#if ((STM32_PLL1R_VALUE >= STM32_PLL1R_VALUE_MIN) &&                        \
     (STM32_PLL1R_VALUE <= STM32_PLL1R_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL1R                 ((STM32_PLL1R_VALUE - 1) << RCC_PLL1DIVR_PLL1R_Pos)
#else
#error "out of range STM32_PLL1R_VALUE value specified"
#endif

/**
 * @brief   PLL1R output clock frequency.
 */
#define STM32_PLL1_R_CLKOUT         (STM32_PLL1VCO / STM32_PLL1R_VALUE)

/*
 * PLL1R output frequency range check.
 */
#if STM32_ACTIVATE_PLL1 &&                                                  \
    ((STM32_PLL1_R_CLKOUT < STM32_PLLR_MIN) ||                              \
     (STM32_PLL1_R_CLKOUT > STM32_PLLR_MAX))
#error "STM32_PLL1_R_CLKOUT outside acceptable range (STM32_PLLR_MIN...STM32_PLLR_MAX)"
#endif

#else /* !STM32_RCC_PLL1_HAS_R */
#define STM32_PLL1R                 0
#endif /* !STM32_RCC_PLL1_HAS_R */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE void pll1_enable(void) {

   RCC->CR |= RCC_CR_PLL1ON;
}

 __STATIC_INLINE void pll1_disable(void) {

   RCC->CR &= ~RCC_CR_PLL1ON;
 }

 __STATIC_INLINE bool pll1_not_locked(void) {

   return (bool)((RCC->CR & RCC_CR_PLL1RDY) == 0U);
 }

__STATIC_INLINE void pll1_wait_lock(void) {

  while (pll1_not_locked()) {
    /* Waiting for PLL1 lock.*/
  }
}

#endif /* STM32_RCC_HAS_PLL1 */

__STATIC_INLINE void pll1_setup(uint32_t cfgr, uint32_t divr, uint32_t fracr) {

#if STM32_RCC_HAS_PLL1
  /* PLL1 activation.*/
  RCC->PLL1CFGR  = cfgr & ~RCC_PLL1CFGR_PLL1FRACEN_Msk;
  RCC->PLL1DIVR  = divr;
  RCC->PLL1FRACR = fracr;
  RCC->PLL1CFGR  = cfgr;
#else
  (void)cfgr;
  (void)divr;
  (void)fracr;
#endif
}

__STATIC_INLINE void pll1_init(void) {

#if STM32_RCC_HAS_PLL1
#if STM32_ACTIVATE_PLL1
  /* PLL1 activation.*/
  pll1_setup(STM32_PLL1REN     | STM32_PLL1QEN     |
             STM32_PLL1PEN     | STM32_PLL1M       |
             STM32_PLL1RGE     | STM32_PLL1VCOSEL  |
             STM32_PLL1SRC,
             STM32_PLL1R       | STM32_PLL1Q       |
             STM32_PLL1P       | STM32_PLL1N,
             0U); /* TODO FRACR */
  pll1_enable();
  pll1_wait_lock();
#endif
#endif
}

__STATIC_INLINE void pll1_deinit(void) {

#if STM32_RCC_HAS_PLL1
#if STM32_ACTIVATE_PLL1
  /* PLL1 de-activation.*/
  pll1_disable();
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
