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
 * @file    RCCv1/stm32_pll3.inc
 * @brief   Shared PLL3 handler.
 *
 * @addtogroup STM32_PLL3_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    PLL helpers
 */
#define STM32_PLL3VCOSEL_WIDE       (0U << RCC_PLL3CFGR_PLL3VCOSEL_Pos)
#define STM32_PLL3VCOSEL_MEDIUM     (1U << RCC_PLL3CFGR_PLL3VCOSEL_Pos)
#define STM32_PLL3RGE_0             (0U << RCC_PLL3CFGR_PLL3RGE_Pos)
#define STM32_PLL3RGE_1             (1U << RCC_PLL3CFGR_PLL3RGE_Pos)
#define STM32_PLL3RGE_2             (2U << RCC_PLL3CFGR_PLL3RGE_Pos)
#define STM32_PLL3RGE_3             (3U << RCC_PLL3CFGR_PLL3RGE_Pos)
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Checks on registry.*/
#if !defined(STM32_RCC_HAS_PLL3)
#error "STM32_RCC_HAS_PLL3 not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL3 && !defined(STM32_RCC_PLL3_HAS_P)
#error "STM32_RCC_PLL3_HAS_P not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL3 && !defined(STM32_RCC_PLL3_HAS_Q)
#error "STM32_RCC_PLL3_HAS_Q not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL3 && !defined(STM32_RCC_PLL3_HAS_R)
#error "STM32_RCC_PLL3_HAS_R not defined in stm32_registry.h"
#endif

#if STM32_RCC_HAS_PLL3

/* Checks on configurations.*/
#if !defined(STM32_PLL3SRC)
#error "STM32_PLL3SRC not defined in mcuconf.h"
#endif

#if !defined(STM32_PLL3M_VALUE)
#error "STM32_PLL3M_VALUE not defined in mcuconf.h"
#endif

#if !defined(STM32_PLL3N_VALUE)
#error "STM32_PLL3N_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL3_HAS_P && !defined(STM32_PLL3P_VALUE)
#error "STM32_PLL3P_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL3_HAS_Q && !defined(STM32_PLL3Q_VALUE)
#error "STM32_PLL3Q_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLL3_HAS_R && !defined(STM32_PLL3R_VALUE)
#error "STM32_PLL3R_VALUE not defined in mcuconf.h"
#endif

/* Check on valid values.*/
#if !defined(STM32_PLL3M_VALUE_MAX)
#error "STM32_PLL3M_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3M_VALUE_MIN)
#error "STM32_PLL3M_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3N_ODDVALID)
#error "STM32_PLL3N_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3N_VALUE_MAX)
#error "STM32_PLL3N_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3N_VALUE_MIN)
#error "STM32_PLL3N_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3P_ODDVALID)
#error "STM32_PLL3P_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3P_VALUE_MAX)
#error "STM32_PLL3P_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3P_VALUE_MIN)
#error "STM32_PLL3P_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3Q_ODDVALID)
#error "STM32_PLL3Q_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3Q_VALUE_MAX)
#error "STM32_PLL3Q_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3Q_VALUE_MIN)
#error "STM32_PLL3Q_VALUE_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3R_ODDVALID)
#error "STM32_PLL3R_ODDVALID not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3R_VALUE_MAX)
#error "STM32_PLL3R_VALUE_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLL3R_VALUE_MIN)
#error "STM32_PLL3R_VALUE_MIN not defined in hal_lld.h"
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
#if !defined(STM32_PLL3IN)
#error "STM32_PLL3IN not defined in hal_lld.h"
#endif

#if !defined(STM32_ACTIVATE_PLL3)
#error "STM32_ACTIVATE_PLL3 not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL3_HAS_P && !defined(STM32_PLL3PEN)
#error "STM32_PLL3PEN not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL3_HAS_Q && !defined(STM32_PLL3QEN)
#error "STM32_PLL3QEN not defined in hal_lld.h"
#endif

#if STM32_RCC_PLL3_HAS_R && !defined(STM32_PLL3REN)
#error "STM32_PLL3REN not defined in hal_lld.h"
#endif

#if STM32_ACTIVATE_PLL3 && (STM32_PLL3IN == 0)
#error "PLL3 activation required but no PLL3 clock selected"
#endif

/**
 * @brief   STM32_PLL3M field.
 */
#if ((STM32_PLL3M_VALUE >= STM32_PLL3M_VALUE_MIN) &&                        \
     (STM32_PLL3M_VALUE <= STM32_PLL3M_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL3M                 (STM32_PLL3M_VALUE << RCC_PLL3CFGR_PLL3M_Pos)

#else
#error "invalid STM32_PLL3M_VALUE value specified"
#endif

/**
 * @brief   PLL comparator input frequency.
 */
#define STM32_PLL3CLKIN             (STM32_PLL3IN / STM32_PLL3M_VALUE)

#if STM32_PLL3CLKIN < STM32_PLLIN_MIN
#error "STM32_PLL3CLKIN below acceptable range"

#elif STM32_PLL3CLKIN > STM32_PLLIN_MAX
#error "STM32_PLL3CLKIN above acceptable range"

#elif (STM32_PLL3CLKIN < STM32_PLLIN_THRESHOLD1) || defined(__DOXYGEN__)
#define STM32_PLL3RGE               STM32_PLL3RGE_0
#define STM32_PLL3VCOSEL            STM32_PLL3VCOSEL_MEDIUM
#define STM32_PLL3VCO_MIN           STM32_PLLVCO_MEDIUM_MIN
#define STM32_PLL3VCO_MAX           STM32_PLLVCO_MEDIUM_MAX

#elif STM32_PLL3CLKIN < STM32_PLLIN_THRESHOLD2
#define STM32_PLL3RGE               STM32_PLL3RGE_1
#define STM32_PLL3VCOSEL            STM32_PLL3VCOSEL_WIDE
#define STM32_PLL3VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL3VCO_MAX           STM32_PLLVCO_WIDE_MAX

#elif STM32_PLL3CLKIN < STM32_PLLIN_THRESHOLD3
#define STM32_PLL3RGE               STM32_PLL3RGE_2
#define STM32_PLL3VCOSEL            STM32_PLL3VCOSEL_WIDE
#define STM32_PLL3VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL3VCO_MAX           STM32_PLLVCO_WIDE_MAX

#else
#define STM32_PLL3RGE               STM32_PLL3RGE_3
#define STM32_PLL3VCOSEL            STM32_PLL3VCOSEL_WIDE
#define STM32_PLL3VCO_MIN           STM32_PLLVCO_WIDE_MIN
#define STM32_PLL3VCO_MAX           STM32_PLLVCO_WIDE_MAX
#endif

/**
 * @brief   STM32_PLL3N field.
 */
#if ((STM32_PLL3N_VALUE >= STM32_PLL3N_VALUE_MIN) &&                        \
     (STM32_PLL3N_VALUE <= STM32_PLL3N_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL3N                 ((STM32_PLL3N_VALUE - 1U) << RCC_PLL3DIVR_PLL3N_Pos)

#else
#error "invalid STM32_PLL3N_VALUE value specified"
#endif

/**
 * @brief   PLL3 VCO frequency.
 */
#define STM32_PLL3VCO               (STM32_PLL3CLKIN * STM32_PLL3N_VALUE)

/*
 * PLL3 VCO frequency range check.
 */
#if STM32_ACTIVATE_PLL3 &&                                                  \
    ((STM32_PLL3VCO < STM32_PLL3VCO_MIN) || (STM32_PLL3VCO > STM32_PLL3VCO_MAX))
#error "STM32_PLL3VCO outside acceptable range (STM32_PLL3VCO_MIN...STM32_PLL3VCO_MAX)"
#endif

/*---------------------------------------------------------------------------*/
/* P output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL3_HAS_P || defined(__DOXYGEN__)

#if !STM32_PLL3P_ODDVALID && ((STM32_PLL3P_VALUE & 1) != 0)
#error "odd STM32_PLL3P_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL3P field.
 */
#if ((STM32_PLL3P_VALUE >= STM32_PLL3P_VALUE_MIN) &&                        \
     (STM32_PLL3P_VALUE <= STM32_PLL3P_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL3P                 ((STM32_PLL3P_VALUE - 1) << RCC_PLL3DIVR_PLL3P_Pos)
#else
#error "out of range STM32_PLL3P_VALUE value specified"
#endif

/**
 * @brief   PLL3P output clock frequency.
 */
#define STM32_PLL3_P_CLKOUT         (STM32_PLL3VCO / STM32_PLL3P_VALUE)

/*
 * PLL3P output frequency range check.
 */
#if STM32_ACTIVATE_PLL3 &&                                                  \
    ((STM32_PLL3_P_CLKOUT < STM32_PLLP_MIN) ||                              \
     (STM32_PLL3_P_CLKOUT > STM32_PLLP_MAX))
#error "STM32_PLL3_P_CLKOUT outside acceptable range (STM32_PLLP_MIN...STM32_PLLP_MAX)"
#endif

#else /* !STM32_RCC_PLL3_HAS_P */
#define STM32_PLL3P                 0
#endif /* !STM32_RCC_PLL3_HAS_P */

/*---------------------------------------------------------------------------*/
/* Q output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL3_HAS_Q || defined(__DOXYGEN__)

#if !STM32_PLL3Q_ODDVALID && ((STM32_PLL3Q_VALUE & 1) != 0)
#error "odd STM32_PLL3Q_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL3Q field.
 */
#if ((STM32_PLL3Q_VALUE >= STM32_PLL3Q_VALUE_MIN) &&                        \
     (STM32_PLL3Q_VALUE <= STM32_PLL3Q_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL3Q                 ((STM32_PLL3Q_VALUE - 1) << RCC_PLL3DIVR_PLL3Q_Pos)
#else
#error "out of range STM32_PLL3Q_VALUE value specified"
#endif

/**
 * @brief   PLL3Q output clock frequency.
 */
#define STM32_PLL3_Q_CLKOUT         (STM32_PLL3VCO / STM32_PLL3Q_VALUE)

/*
 * PLL3P output frequency range check.
 */
#if STM32_ACTIVATE_PLL3 &&                                                  \
    ((STM32_PLL3_Q_CLKOUT < STM32_PLLQ_MIN) ||                              \
     (STM32_PLL3_Q_CLKOUT > STM32_PLLQ_MAX))
#error "STM32_PLL3_Q_CLKOUT outside acceptable range (STM32_PLLQ_MIN...STM32_PLLQ_MAX)"
#endif

#else /* !STM32_RCC_PLL3_HAS_Q */
#define STM32_PLL3Q                 0
#endif /* !STM32_RCC_PLL3_HAS_Q */

/*---------------------------------------------------------------------------*/
/* R output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLL3_HAS_R || defined(__DOXYGEN__)

#if !STM32_PLL3R_ODDVALID && ((STM32_PLL3R_VALUE & 1) != 0)
#error "odd STM32_PLL3R_VALUE value specified"
#endif

/**
 * @brief   STM32_PLL3R field.
 */
#if ((STM32_PLL3R_VALUE >= STM32_PLL3R_VALUE_MIN) &&                        \
     (STM32_PLL3R_VALUE <= STM32_PLL3R_VALUE_MAX)) ||                       \
    defined(__DOXYGEN__)
#define STM32_PLL3R                 ((STM32_PLL3R_VALUE - 1) << RCC_PLL3DIVR_PLL3R_Pos)
#else
#error "out of range STM32_PLL3R_VALUE value specified"
#endif

/**
 * @brief   PLL3R output clock frequency.
 */
#define STM32_PLL3_R_CLKOUT         (STM32_PLL3VCO / STM32_PLL3R_VALUE)

/*
 * PLL3R output frequency range check.
 */
#if STM32_ACTIVATE_PLL3 &&                                                  \
    ((STM32_PLL3_R_CLKOUT < STM32_PLLR_MIN) ||                              \
     (STM32_PLL3_R_CLKOUT > STM32_PLLR_MAX))
#error "STM32_PLL3_R_CLKOUT outside acceptable range (STM32_PLLR_MIN...STM32_PLLR_MAX)"
#endif

#else /* !STM32_RCC_PLL3_HAS_R */
#define STM32_PLL3R                 0
#endif /* !STM32_RCC_PLL3_HAS_R */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE void pll3_enable(void) {

   RCC->CR |= RCC_CR_PLL3ON;
}

 __STATIC_INLINE void pll3_disable(void) {

   RCC->CR &= ~RCC_CR_PLL3ON;
 }

 __STATIC_INLINE bool pll3_not_locked(void) {

   return (bool)((RCC->CR & RCC_CR_PLL3RDY) == 0U);
 }

__STATIC_INLINE void pll3_wait_lock(void) {

  while (pll3_not_locked()) {
    /* Waiting for PLL3 lock.*/
  }
}

#endif /* STM32_RCC_HAS_PLL3 */

__STATIC_INLINE void pll3_setup(uint32_t cfgr, uint32_t divr, uint32_t fracr) {

#if STM32_RCC_HAS_PLL3
  /* PLL3 activation.*/
  RCC->PLL3CFGR  = cfgr & ~RCC_PLL3CFGR_PLL3FRACEN_Msk;
  RCC->PLL3DIVR  = divr;
  RCC->PLL3FRACR = fracr;
  RCC->PLL3CFGR  = cfgr;
#else
  (void)cfgr;
  (void)divr;
  (void)fracr;
#endif
}

__STATIC_INLINE void pll3_init(void) {

#if STM32_RCC_HAS_PLL3
#if STM32_ACTIVATE_PLL3
  /* PLL3 activation.*/
  pll3_setup(STM32_PLL3REN     | STM32_PLL3QEN     |
             STM32_PLL3PEN     | STM32_PLL3M       |
             STM32_PLL3RGE     | STM32_PLL3VCOSEL  |
             STM32_PLL3SRC,
             STM32_PLL3R       | STM32_PLL3Q       |
             STM32_PLL3P       | STM32_PLL3N,
             0U); /* TODO FRACR */
  pll3_enable();
  pll3_wait_lock();
#endif
#endif
}

__STATIC_INLINE void pll3_deinit(void) {

#if STM32_RCC_HAS_PLL3
#if STM32_ACTIVATE_PLL3
  /* PLL3 de-activation.*/
  pll3_disable();
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
