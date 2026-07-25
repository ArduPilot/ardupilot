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
 * @file    RCCv1/stm32_pllsai2.inc
 * @brief   Shared PLLSAI2 handler.
 *
 * @addtogroup STM32_PLLSAI2_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* Required for compatibility with those devices with a common M divider.*/
#if !defined(RCC_PLLSAI1CFGR_PLLSAI2M_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI2M_Pos    RCC_PLLCFGR_PLLM_Pos
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_PLLSAI2)
#define STM32_RCC_HAS_PLLSAI2           FALSE
#endif

#if STM32_RCC_HAS_PLLSAI2

/* Checks on configurations.*/
#if !defined(STM32_PLLSRC)
#error "STM32_PLLSRC not defined in mcuconf.h"
#endif

#if !defined(STM32_PLLSAI2M_VALUE)
#error "STM32_PLLSAI2M_VALUE not defined in mcuconf.h"
#endif

#if !defined(STM32_PLLSAI2N_VALUE)
#error "STM32_PLLSAI2N_VALUE not defined in mcuconf.h"
#endif

#if !defined(STM32_PLLSAI2PDIV_VALUE)
#error "STM32_PLLSAI2PDIV_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLLSAI2_HAS_P && !defined(STM32_PLLSAI2P_VALUE)
#error "STM32_PLLSAI2P_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLLSAI2_HAS_Q && !defined(STM32_PLLSAI2Q_VALUE)
#error "STM32_PLLSAI2Q_VALUE not defined in mcuconf.h"
#endif

#if STM32_RCC_PLLSAI2_HAS_R && !defined(STM32_PLLSAI2R_VALUE)
#error "STM32_PLLSAI2R_VALUE not defined in mcuconf.h"
#endif

/* Check on limits.*/
#if !defined(STM32_PLLIN_MAX)
#error "STM32_PLLIN_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLIN_MIN)
#error "STM32_PLLIN_MIN not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLVCO_MAX)
#error "STM32_PLLIN_MAX not defined in hal_lld.h"
#endif

#if !defined(STM32_PLLVCO_MIN)
#error "STM32_PLLIN_MAX not defined in hal_lld.h"
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
#if !defined(STM32_ACTIVATE_PLLSAI2)
#error "STM32_ACTIVATE_PLLSAI2 not defined in hal_lld.h"
#endif

#if STM32_RCC_PLLSAI2_HAS_P && !defined(STM32_PLLSAI2PEN)
#error "STM32_PLLSAI2PEN not defined in hal_lld.h"
#endif

#if STM32_RCC_PLLSAI2_HAS_Q && !defined(STM32_PLLSAI2QEN)
#error "STM32_PLLSAI2QEN not defined in hal_lld.h"
#endif

#if STM32_RCC_PLLSAI2_HAS_R && !defined(STM32_PLLSAI2REN)
#error "STM32_PLLSAI2REN not defined in hal_lld.h"
#endif

#if STM32_ACTIVATE_PLLSAI1 && (STM32_PLLSAI1CLKIN == 0)
#error "PLLSAI1 activation required but no PLL clock selected"
#endif

#if (STM32_PLLSAI2CLKIN != 0) &&                                            \
    ((STM32_PLLSAI2CLKIN < STM32_PLLIN_MIN) || (STM32_PLLSAI2CLKIN > STM32_PLLIN_MAX))
#error "STM32_PLLSAI2CLKIN outside acceptable range (STM32_PLLIN_MIN...STM32_PLLIN_MAX)"
#endif

/**
 * @brief   STM32_PLLSAI2M field.
 */
#if ((STM32_PLLSAI2M_VALUE >= 1) && (STM32_PLLSAI2M_VALUE <= 16)) ||        \
    defined(__DOXYGEN__)
#define STM32_PLLSAI2M              ((STM32_PLLSAI2M_VALUE - 1U) << RCC_PLLSAI2CFGR_PLLSAI2M_Pos)

#else
#error "invalid STM32_PLLSAI2M_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLSAI2N field.
 */
#if ((STM32_PLLSAI2N_VALUE >= 8) && (STM32_PLLSAI2N_VALUE <= 127)) ||       \
    defined(__DOXYGEN__)
#define STM32_PLLSAI2N              (STM32_PLLSAI2N_VALUE << RCC_PLLSAI2CFGR_PLLSAI2N_Pos)

#else
#error "invalid STM32_PLLSAI2N_VALUE value specified"
#endif

/**
 * @brief   PLLSAI2 VCO frequency.
 */
#define STM32_PLLSAI2VCO            (STM32_PLLSAI2CLKIN * STM32_PLLSAI2N_VALUE)

/*
 * PLLSAI2 VCO frequency range check.
 */
#if STM32_ACTIVATE_PLLSAI2 &&                                               \
    ((STM32_PLLSAI2VCO < STM32_PLLVCO_MIN) || (STM32_PLLSAI2VCO > STM32_PLLVCO_MAX))
#error "STM32_PLLSAI2VCO outside acceptable range (STM32_PLLVCO_MIN...STM32_PLLVCO_MAX)"
#endif

/*---------------------------------------------------------------------------*/
/* P output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLLSAI2_HAS_P || defined(__DOXYGEN__)
/**
 * @brief   STM32_PLLSAI2P field.
 */
#if (STM32_PLLSAI2P_VALUE == 7) || defined(__DOXYGEN__)
#define STM32_PLLSAI2P              (0U << RCC_PLLSAI2CFGR_PLLSAI2P_Pos)

#elif STM32_PLLSAI2P_VALUE == 17
#define STM32_PLLSAI2P              (1U << RCC_PLLSAI2CFGR_PLLSAI2P_Pos)

#else
#error "invalid STM32_PLLSAI2P_VALUE value specified"
#endif

/* SAI2PDIV is not present on all devices.*/
#if defined(RCC_PLLSAI2CFGR_PLLSAI2PDIV_Pos) || defined(__DOXYGEN__)
/**
 * @brief   STM32_PLLSAI2PDIV field.
 */
#if ((STM32_PLLSAI2PDIV_VALUE != 1) && (STM32_PLLSAI2PDIV_VALUE <= 31)) ||  \
    defined(__DOXYGEN__)
#define STM32_PLLSAI2PDIV           (STM32_PLLSAI2PDIV_VALUE << RCC_PLLSAI2CFGR_PLLSAI2PDIV_Pos)
#else
#error "invalid STM32_PLLSAI2PDIV_VALUE value specified"
#endif

/**
 * @brief   PLLSAI2 P output clock frequency.
 */
#if (STM32_PLLSAI2PDIV_VALUE == 0) || defined(__DOXYGEN__)
#define STM32_PLLSAI2_P_CLKOUT      (STM32_PLLSAI2VCO / STM32_PLLSAI2P_VALUE)
#else
#define STM32_PLLSAI2_P_CLKOUT      (STM32_PLLSAI2VCO / STM32_PLLSAI2PDIV_VALUE)
#endif

#else
#define STM32_PLLSAI2PDIV           0U
#define STM32_PLLSAI2_P_CLKOUT      (STM32_PLLSAI2VCO / STM32_PLLSAI2P_VALUE)
#endif

/*
 * PLLSAI2-P output frequency range check.
 */
#if STM32_ACTIVATE_PLLSAI2 &&                                                   \
    ((STM32_PLLSAI2_P_CLKOUT < STM32_PLLP_MIN) || (STM32_PLLSAI2_P_CLKOUT > STM32_PLLP_MAX))
#error "STM32_PLLSAI2_P_CLKOUT outside acceptable range (STM32_PLLP_MIN...STM32_PLLP_MAX)"
#endif

#else /* !STM32_RCC_PLLSAI2_HAS_P */
#define STM32_PLLSAI2P              0U
#define STM32_PLLSAI2PEN            0U
#endif /* !STM32_RCC_PLLSAI2_HAS_P */

/*---------------------------------------------------------------------------*/
/* Q output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLLSAI2_HAS_Q || defined(__DOXYGEN__)
/**
 * @brief   STM32_PLLSAI2Q field.
 */
#if (STM32_PLLSAI2Q_VALUE == 2) || defined(__DOXYGEN__)
#define STM32_PLLSAI2Q              (0U << RCC_PLLSAI2CFGR_PLLSAI2Q_Pos)

#elif STM32_PLLSAI2Q_VALUE == 4
#define STM32_PLLSAI2Q              (1U << RCC_PLLSAI2CFGR_PLLSAI2Q_Pos)

#elif STM32_PLLSAI2Q_VALUE == 6
#define STM32_PLLSAI2Q              (2U << RCC_PLLSAI2CFGR_PLLSAI2Q_Pos)

#elif STM32_PLLSAI2Q_VALUE == 8
#define STM32_PLLSAI2Q              (3U << RCC_PLLSAI2CFGR_PLLSAI2Q_Pos)

#else
#error "invalid STM32_PLLSAI2Q_VALUE value specified"
#endif

/**
 * @brief   PLLSAI2 Q output clock frequency.
 */
#define STM32_PLLSAI2_Q_CLKOUT      (STM32_PLLSAI2VCO / STM32_PLLSAI2Q_VALUE)

/*
 * PLLSAI2-Q output frequency range check.
 */
#if STM32_ACTIVATE_PLLSAI2 &&                                               \
    ((STM32_PLLSAI2_Q_CLKOUT < STM32_PLLQ_MIN) || (STM32_PLLSAI2_Q_CLKOUT > STM32_PLLQ_MAX))
#error "STM32_PLLSAI2_Q_CLKOUT outside acceptable range (STM32_PLLQ_MIN...STM32_PLLQ_MAX)"
#endif

#else /* !STM32_RCC_PLLSAI2_HAS_Q */
#define STM32_PLLSAI2Q              0U
#define STM32_PLLSAI2QEN            0U
#endif /* !STM32_RCC_PLLSAI2_HAS_Q */

/*---------------------------------------------------------------------------*/
/* R output, if present.                                                     */
/*---------------------------------------------------------------------------*/
#if STM32_RCC_PLLSAI2_HAS_R || defined(__DOXYGEN__)
/**
 * @brief   STM32_PLLSAI2R field.
 */
#if (STM32_PLLSAI2R_VALUE == 2) || defined(__DOXYGEN__)
#define STM32_PLLSAI2R              (0U << RCC_PLLSAI2CFGR_PLLSAI2R_Pos)

#elif STM32_PLLSAI2R_VALUE == 4
#define STM32_PLLSAI2R              (1U << RCC_PLLSAI2CFGR_PLLSAI2R_Pos)

#elif STM32_PLLSAI2R_VALUE == 6
#define STM32_PLLSAI2R              (2U << RCC_PLLSAI2CFGR_PLLSAI2R_Pos)

#elif STM32_PLLSAI2R_VALUE == 8
#define STM32_PLLSAI2R              (3U << RCC_PLLSAI2CFGR_PLLSAI2R_Pos)

#else
#error "invalid STM32_PLLSAI2R_VALUE value specified"
#endif

/**
 * @brief   PLLSAI2 R output clock frequency.
 */
#define STM32_PLLSAI2_R_CLKOUT      (STM32_PLLSAI2VCO / STM32_PLLSAI2R_VALUE)

/*
 * PLLSAI2-R output frequency range check.
 */
#if STM32_ACTIVATE_PLLSAI2 &&                                                   \
    ((STM32_PLLSAI2_R_CLKOUT < STM32_PLLR_MIN) || (STM32_PLLSAI2_R_CLKOUT > STM32_PLLR_MAX))
#error "STM32_PLLSAI2_R_CLKOUT outside acceptable range (STM32_PLLR_MIN...STM32_PLLR_MAX)"
#endif

#else /* !STM32_RCC_PLLSAI2_HAS_R */
#define STM32_PLLSAI2R              0U
#define STM32_PLLSAI2REN            0U
#endif /* !STM32_RCC_PLLSAI2_HAS_R */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE bool pllsai2_not_locked(void) {

  if ((RCC->CR & RCC_CR_PLLSAI2ON) == 0U) {
    /* Disabled, no need to lock.*/
    return false;
  }
  return (bool)((RCC->CR & RCC_CR_PLLSAI2RDY) == 0U);
}

__STATIC_INLINE void pllsai2_wait_lock(void) {

  while (pllsai2_not_locked()) {
    /* Waiting for PLLSAI2 lock.*/
  }
}

#endif /* STM32_RCC_HAS_PLLSAI2 */

__STATIC_INLINE void pllsai2_init(void) {

#if STM32_RCC_HAS_PLLSAI2
#if STM32_ACTIVATE_PLLSAI2
  /* PLLSAI2 activation.*/
  RCC->PLLSAI2CFGR = STM32_PLLSAI2PDIV | STM32_PLLSAI2R   |
                     STM32_PLLSAI2REN  | STM32_PLLSAI2Q   |
                     STM32_PLLSAI2QEN  | STM32_PLLSAI2P   |
                     STM32_PLLSAI2PEN  | STM32_PLLSAI2N   |
                     STM32_PLLSAI2M;
  RCC->CR |= RCC_CR_PLLSAI2ON;

  /* Waiting for PLL lock.*/
  pllsai2_wait_lock();
#endif
#endif
}

__STATIC_INLINE void pllsai2_deinit(void) {

#if STM32_RCC_HAS_PLLSAI2
#if STM32_ACTIVATE_PLLSAI2
  /* PLLSAI2 de-activation.*/
  RCC->PLLSAI2CFGR &= ~RCC_CR_PLLSAI2ON;
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
