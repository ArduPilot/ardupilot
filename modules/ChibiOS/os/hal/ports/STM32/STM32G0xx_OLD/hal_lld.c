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
 * @file    STM32G0xx/hal_lld.c
 * @brief   STM32G0xx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Number of thresholds in the wait states array.
 */
#define STM32_WS_THRESHOLDS             5

/**
 * @brief   Flash ACR reset value.
 * @note    The data cache bit is set even though there is no data cache
 */
#define STM32_FLASH_ACR_RESET           0x00040600U

/**
 * @brief   PWR CR1 reset value.
 */
#define STM32_PWR_CR1_RESET             (PWR_CR1_VOS_0 | PWR_CR1_FPD_STOP)

/**
 * @brief   PWR CR2 reset value.
 */
#define STM32_PWR_CR2_RESET             0U

/**
 * @brief   RCC CR reset value.
 */
#define STM32_RCC_CR_RESET              (RCC_CR_HSION)

/**
 * @brief   PWR CR bits safe for fast switch.
 */
#define STM32_PWR_CR1_SAFE_ONLY_MASK    (PWR_CR1_FPD_LPSLP |                \
                                         PWR_CR1_FPD_LPRUN |                \
                                         PWR_CR1_FPD_STOP  |                \
                                         PWR_CR1_LPMS_Msk)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_stm32g0xx.h.
 */
uint32_t SystemCoreClock = STM32_HCLK;

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Post-reset clock configuration.
 */
const halclkcfg_t hal_clkcfg_reset = {
  .pwr_cr1              = STM32_PWR_CR1_RESET,
#if STM32_PWR_HAS_CR2
#if STM32_PWR_HAS_VDDIO2 == TRUE
  .pwr_cr2              = PWR_CR2_VDDIO2_MONITORING_ENABLED,
#else
  .pwr_cr2              = STM32_PWR_CR2_RESET,
#endif
#endif
  .rcc_cr               = STM32_RCC_CR_RESET,
  .rcc_cfgr             = RCC_CFGR_SW_HSI,
  .rcc_pllcfgr          = 0U,
  .flash_acr            = STM32_FLASH_ACR_RESET
};

/**
 * @brief   Default clock configuration.
 */
const halclkcfg_t hal_clkcfg_default = {
  .pwr_cr1              = STM32_VOS_RANGE1 | PWR_CR1_DBP,
#if STM32_PWR_HAS_CR2
  .pwr_cr2              = STM32_PWR_CR2,
#endif
  .rcc_cr               = STM32_HSIDIV
#if STM32_HSI16_ENABLED
                        | RCC_CR_HSIKERON | RCC_CR_HSION
#endif
#if STM32_HSI48_ENABLED
                        | RCC_CR_HSI48ON
#endif
#if STM32_HSE_ENABLED
                        | RCC_CR_HSEON
#endif
#if STM32_ACTIVATE_PLL
                        | RCC_CR_PLLON
#endif
                        ,
  .rcc_cfgr             = STM32_MCOPRE  | STM32_MCOSEL  |
#if STM32_RCC_HAS_MCO2 == TRUE
                          STM32_MCO2PRE | STM32_MCO2SEL |
#endif
                          STM32_PPRE    | STM32_HPRE    |
                          STM32_SW,
  .rcc_pllcfgr          = STM32_PLLR    | STM32_PLLREN  |
                          STM32_PLLQ    | STM32_PLLQEN  |
                          STM32_PLLP    | STM32_PLLPEN  |
                          STM32_PLLN    | STM32_PLLM    |
                          STM32_PLLSRC,
#if defined(FLASH_ACR_DBG_SWEN)
  .flash_acr            = FLASH_ACR_DBG_SWEN | FLASH_ACR_ICEN |
                          FLASH_ACR_PRFTEN   | STM32_FLASHBITS
#else
  .flash_acr            = FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | STM32_FLASHBITS
#endif
};
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Dynamic clock points for this device.
 */
static halfreq_t clock_points[CLK_ARRAY_SIZE] = {
  [CLK_SYSCLK]      = STM32_SYSCLK,
  [CLK_HSISYSCLK]   = STM32_HSISYSCLK,
  [CLK_PLLPCLK]     = STM32_PLL_P_CLKOUT,
  [CLK_PLLQCLK]     = STM32_PLL_Q_CLKOUT,
  [CLK_PLLRCLK]     = STM32_PLL_R_CLKOUT,
  [CLK_HCLK]        = STM32_HCLK,
  [CLK_PCLK]        = STM32_PCLK,
  [CLK_PCLKTIM]     = STM32_TIMPCLK,
  [CLK_MCO]         = STM32_MCOCLK,
#if STM32_RCC_HAS_MCO2 == TRUE
  [CLK_MCO2]        = STM32_MCO2CLK,
#endif
};

/**
 * @brief   Type of a structure representing system limits.
 */
typedef struct {
  halfreq_t     sysclk_max;
  halfreq_t     pllin_max;
  halfreq_t     pllin_min;
  halfreq_t     pllvco_max;
  halfreq_t     pllvco_min;
  halfreq_t     pllp_max;
  halfreq_t     pllp_min;
  halfreq_t     pllq_max;
  halfreq_t     pllq_min;
  halfreq_t     pllr_max;
  halfreq_t     pllr_min;
  halfreq_t     flash_thresholds[STM32_WS_THRESHOLDS];
} system_limits_t;

/**
 * @brief   System limits for VOS range 1.
 */
static const system_limits_t vos_range1 = {
  .sysclk_max           = STM32_VOS1_SYSCLK_MAX,
  .pllin_max            = STM32_VOS1_PLLIN_MAX,
  .pllin_min            = STM32_VOS1_PLLIN_MIN,
  .pllvco_max           = STM32_VOS1_PLLVCO_MAX,
  .pllvco_min           = STM32_VOS1_PLLVCO_MIN,
  .pllp_max             = STM32_VOS1_PLLP_MAX,
  .pllp_min             = STM32_VOS1_PLLP_MIN,
  .pllq_max             = STM32_VOS1_PLLQ_MAX,
  .pllq_min             = STM32_VOS1_PLLQ_MIN,
  .pllr_max             = STM32_VOS1_PLLR_MAX,
  .pllr_min             = STM32_VOS1_PLLR_MIN,
  .flash_thresholds     = {STM32_VOS1_0WS_THRESHOLD, STM32_VOS1_1WS_THRESHOLD,
                           STM32_VOS1_2WS_THRESHOLD, STM32_VOS1_3WS_THRESHOLD,
                           STM32_VOS1_4WS_THRESHOLD}
};

/**
 * @brief   System limits for VOS range 2.
 */
static const system_limits_t vos_range2 = {
  .sysclk_max           = STM32_VOS2_SYSCLK_MAX,
  .pllin_max            = STM32_VOS2_PLLIN_MAX,
  .pllin_min            = STM32_VOS2_PLLIN_MIN,
  .pllvco_max           = STM32_VOS2_PLLVCO_MAX,
  .pllvco_min           = STM32_VOS2_PLLVCO_MIN,
  .pllp_max             = STM32_VOS2_PLLP_MAX,
  .pllp_min             = STM32_VOS2_PLLP_MIN,
  .pllq_max             = STM32_VOS2_PLLQ_MAX,
  .pllq_min             = STM32_VOS2_PLLQ_MIN,
  .pllr_max             = STM32_VOS2_PLLR_MAX,
  .pllr_min             = STM32_VOS2_PLLR_MIN,
  .flash_thresholds     = {STM32_VOS2_0WS_THRESHOLD, STM32_VOS2_1WS_THRESHOLD,
                           STM32_VOS2_2WS_THRESHOLD, STM32_VOS2_3WS_THRESHOLD,
                           STM32_VOS2_4WS_THRESHOLD}
};
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#include "stm32_bd.inc"

/**
 * @brief   Safe setting of flash ACR register.
 *
 * @param[in] acr       value for the ACR register
 */
__STATIC_INLINE void flash_set_acr(uint32_t acr) {

#if defined(STM32G0B0xx)
  /* Set ACR. Note: retaining set bits in ACR required.*/
  FLASH->ACR |= acr;
#else
  FLASH->ACR = acr;
#endif
  while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != (acr & FLASH_ACR_LATENCY_Msk)) {
    /* Waiting for flash wait states setup.*/
  }
}

/**
 * @brief   Configures the PWR unit.
 * @note    CR1, CR2 and CR5 are not initialized inside this function.
 */
__STATIC_INLINE void hal_lld_set_static_pwr(void) {

  /* PWR clock enabled.*/
  rccEnablePWRInterface(false);

  /* Static PWR configurations.*/
#if defined(STM32G0B0xx) && HAL_USE_USB
  /* Special case, CR2 is handled as static. Note: retaining set bits in
     CR2 is required.*/
  PWR->CR2  |= PWR_CR2_USV;
#endif
  PWR->CR3   = STM32_PWR_CR3;
  PWR->CR4   = STM32_PWR_CR4;
  PWR->PUCRA = STM32_PWR_PUCRA;
  PWR->PDCRA = STM32_PWR_PDCRA;
  PWR->PUCRB = STM32_PWR_PUCRB;
  PWR->PDCRB = STM32_PWR_PDCRB;
  PWR->PUCRC = STM32_PWR_PUCRC;
  PWR->PDCRC = STM32_PWR_PDCRC;
#if STM32_HAS_GPIOD
  PWR->PUCRD = STM32_PWR_PUCRD;
  PWR->PDCRD = STM32_PWR_PDCRD;
#endif
#if STM32_HAS_GPIOE
  PWR->PUCRE = STM32_PWR_PUCRE;
  PWR->PDCRE = STM32_PWR_PDCRE;
#endif
#if STM32_HAS_GPIOF
  PWR->PUCRF = STM32_PWR_PUCRF;
  PWR->PDCRF = STM32_PWR_PDCRF;
#endif
#if STM32_HAS_GPIOG
  PWR->PUCRG = STM32_PWR_PUCRG;
  PWR->PDCRG = STM32_PWR_PDCRG;
#endif
#if STM32_HAS_GPIOH
  PWR->PUCRH = STM32_PWR_PUCRH;
  PWR->PDCRH = STM32_PWR_PDCRH;
#endif
#if STM32_HAS_GPIOI
  PWR->PUCRI = STM32_PWR_PUCRI;
  PWR->PDCRI = STM32_PWR_PDCRI;
#endif
}

/**
 * @brief   Initializes static muxes and dividers.
 */
__STATIC_INLINE void hal_lld_set_static_clocks(void) {

  /* Clock-related settings (dividers, MCO etc).*/
#if STM32_RCC_HAS_MCO2 == TRUE
  RCC->CFGR = STM32_MCOPRE | STM32_MCOSEL | STM32_MCO2PRE | STM32_MCO2SEL |
              STM32_PPRE   | STM32_HPRE;
#else
  RCC->CFGR = STM32_MCOPRE | STM32_MCOSEL |
              STM32_PPRE   | STM32_HPRE;
#endif

  /* Set HSISYS divisor.*/
  RCC->CR = (RCC->CR & ~STM32_HSIDIV_MASK) | STM32_HSIDIV;

#if STM32_RCC_HAS_CCIPR2
  /* CCIPR register initialization.*/
  RCC->CCIPR =  STM32_ADCSEL    | STM32_RNGDIV     | STM32_RNGSEL    |
                STM32_TIM15SEL  | STM32_TIM1SEL    | STM32_LPTIM2SEL |
                STM32_LPTIM1SEL | STM32_I2C2SEL    | STM32_I2C1SEL   |
                STM32_CECSEL    | STM32_USART3SEL  | STM32_USART2SEL |
                STM32_USART1SEL | STM32_LPUART2SEL | STM32_LPUART1SEL;

  /* CCIPR2 register initialization.*/
  RCC->CCIPR2 = STM32_USBSEL    | STM32_FDCANSEL   | STM32_I2S2SEL   |
                STM32_I2S1SEL;

#else
  /* CCIPR register initialization.*/
  RCC->CCIPR =  STM32_ADCSEL    | STM32_RNGDIV     | STM32_RNGSEL    |
                STM32_TIM15SEL  | STM32_TIM1SEL    | STM32_LPTIM2SEL |
                STM32_LPTIM1SEL | STM32_I2S1SEL    | STM32_I2C1SEL   |
                STM32_CECSEL    | STM32_USART3SEL  | STM32_USART2SEL |
                STM32_USART1SEL | STM32_LPUART2SEL | STM32_LPUART1SEL;
#endif
}

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Recalculates the clock tree frequencies.
 *
 * @param[in] ccp       pointer to clock a @p halclkcfg_t structure
 * @return              The frequency calculation result.
 * @retval false        if the clock settings look valid
 * @retval true         if the clock settings look invalid
 *
 * @notapi
 */
static bool hal_lld_clock_check_tree(const halclkcfg_t *ccp) {
  static const uint32_t hprediv[16] = {1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U,
                                       2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
  static const uint32_t pprediv[16] = {1U, 1U, 1U, 1U, 2U, 4U, 8U, 16U};
  const system_limits_t *slp;
  halfreq_t hsi16clk = 0U, hseclk = 0U, pllselclk, hsisysclk;
  halfreq_t pllpclk = 0U, pllqclk = 0U, pllrclk = 0U;
  halfreq_t sysclk, hclk, pclk, pclktim, mcoclk;
#if STM32_RCC_HAS_MCO2 == TRUE
  halfreq_t mco2clk;
#endif
  uint32_t mcodiv, flashws, hsidiv;

  /* System limits based on desired VOS settings.*/
  if ((ccp->pwr_cr1 & PWR_CR1_VOS_Msk) == PWR_CR1_VOS_1) {
    slp = &vos_range2;
  }
  else if ((ccp->pwr_cr1 & PWR_CR1_VOS_Msk) == PWR_CR1_VOS_0) {
    slp = &vos_range1;
  }
  else {
    return true;
  }

  /* HSI16 clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSION) != 0U) {
    hsi16clk = STM32_HSI16CLK;
  }

  /* HSISYS clock.*/
  hsidiv = 1U << ((ccp->rcc_cr & RCC_CR_HSIDIV_Msk) >> RCC_CR_HSIDIV_Pos);
  hsisysclk = hsi16clk / hsidiv;

  /* HSE clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    hseclk = STM32_HSECLK;
  }

  /* PLL MUX clock.*/
  switch (ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLSRC_Msk) {
  case RCC_PLLCFGR_PLLSRC_HSI:
    pllselclk = hsi16clk;
    break;
  case RCC_PLLCFGR_PLLSRC_HSE:
    pllselclk = hseclk;
    break;
  default:
    pllselclk = 0U;
  }

  /* PLL outputs.*/
  if ((ccp->rcc_cr & RCC_CR_PLLON) != 0U) {
    uint32_t pllmdiv, pllndiv, pllpdiv, pllqdiv, pllrdiv;
    halfreq_t pllvcoclk;

    /* PLL M divider.*/
    pllmdiv = ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos) + 1U;

    /* PLL N divider.*/
    pllndiv = (ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
    if (pllndiv < 8) {
      return true;
    }

    /* PLL VCO frequency.*/
    pllvcoclk = (pllselclk / (halfreq_t)pllmdiv) * (halfreq_t)pllndiv;

    if ((pllvcoclk < slp->pllvco_min) || (pllvcoclk > slp->pllvco_max)) {
      return true;
    }

    /* PLL P output frequency.*/
    pllpdiv = 1U + ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos);
    if ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLPEN) != 0U) {
      pllpclk = pllvcoclk / pllpdiv;

      if ((pllpclk < slp->pllp_min) || (pllpclk > slp->pllp_max)) {
        return true;
      }
    }

    /* PLL Q output frequency.*/
    pllqdiv = 1U + ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLQ_Msk) >> RCC_PLLCFGR_PLLQ_Pos);
    if ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLQEN) != 0U) {
      pllqclk = pllvcoclk / pllqdiv;

      if ((pllqclk < slp->pllq_min) || (pllqclk > slp->pllq_max)) {
        return true;
      }
    }

    /* PLL R output frequency.*/
    pllrdiv = 1U + ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos);
    if ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLREN) != 0U) {
      pllrclk = pllvcoclk / pllrdiv;

      if ((pllrclk < slp->pllr_min) || (pllrclk > slp->pllr_max)) {
        return true;
      }
    }
  }

  /* SYSCLK frequency.*/
  switch (ccp->rcc_cfgr & RCC_CFGR_SW_Msk) {
  case RCC_CFGR_SW_HSI:
    sysclk = hsisysclk;
    break;
  case RCC_CFGR_SW_HSE:
    sysclk = hseclk;
    break;
  case RCC_CFGR_SW_PLL:
    sysclk = pllrclk;
    break;
  case RCC_CFGR_SW_LSI:
    sysclk = STM32_LSICLK;
    break;
  case RCC_CFGR_SW_LSE:
    sysclk = STM32_LSECLK;
    break;
  default:
    sysclk = 0U;
  }

  if ((sysclk == 0U) || (sysclk > slp->sysclk_max)) {
    return true;
  }

  /* HCLK frequency.*/
  hclk = sysclk / hprediv[(ccp->rcc_cfgr & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos];

  /* PPRE frequency.*/
  pclk = hclk / pprediv[(ccp->rcc_cfgr & RCC_CFGR_PPRE_Msk) >> RCC_CFGR_PPRE_Pos];
  if ((ccp->rcc_cfgr & RCC_CFGR_PPRE_Msk) < RCC_CFGR_PPRE_DIV2) {
    pclktim = pclk;
  }
  else {
    pclktim = pclk * 2U;
  }

  /* MCO clock.*/
  switch (ccp->rcc_cfgr & RCC_CFGR_MCOSEL_Msk) {
  case STM32_MCOSEL_NOCLOCK:
    mcoclk = 0U;
    break;
  case STM32_MCOSEL_SYSCLK:
    mcoclk = sysclk;
    break;
  case STM32_MCOSEL_HSI16:
    mcoclk = hsi16clk;
    break;
#if STM32_RCC_HAS_HSI48
  case STM32_MCOSEL_HSI48:
    mcoclk = STM32_HSI48CLK;
    break;
#endif
  case STM32_MCOSEL_HSE:
    mcoclk = hseclk;
    break;
  case STM32_MCOSEL_PLLRCLK:
    mcoclk = pllrclk;
    break;
  case STM32_MCOSEL_LSI:
    mcoclk = STM32_LSICLK;
    break;
  case STM32_MCOSEL_LSE:
    mcoclk = STM32_LSECLK;
    break;
#if STM32_RCC_HAS_MCOSEL_EXT == TRUE
  case STM32_MCOSEL_PLLPCLK:
    mcoclk = pllpclk;
    break;
  case STM32_MCOSEL_PLLQCLK:
    mcoclk = pllqclk;
    break;
  case STM32_MCOSEL_RTCCLK:
    mcoclk = STM32_RTCCLK;
    break;
#endif
  default:
    mcoclk = 0U;
  }
  mcodiv = 1U << ((ccp->rcc_cfgr & RCC_CFGR_MCOPRE_Msk) >> RCC_CFGR_MCOPRE_Pos);
#if STM32_RCC_HAS_MCOPRE_EXT == TRUE
  if (mcodiv > 1024U) {
#else
  if (mcodiv > 128U) {
#endif
    return true;
  }
  mcoclk /= mcodiv;

#if STM32_RCC_HAS_MCO2 == TRUE
  /* MCO2 clock.*/
  switch (ccp->rcc_cfgr & RCC_CFGR_MCO2SEL_Msk) {
  case STM32_MCO2SEL_NOCLOCK:
    mco2clk = 0U;
    break;
  case STM32_MCO2SEL_SYSCLK:
    mco2clk = sysclk;
    break;
  case STM32_MCO2SEL_HSI16:
    mco2clk = hsi16clk;
    break;
#if STM32_RCC_HAS_HSI48
  case STM32_MCO2SEL_HSI48:
    mco2clk = STM32_HSI48CLK;
    break;
#endif
  case STM32_MCO2SEL_HSE:
    mco2clk = hseclk;
    break;
  case STM32_MCO2SEL_PLLRCLK:
    mco2clk = pllrclk;
    break;
  case STM32_MCO2SEL_LSI:
    mco2clk = STM32_LSICLK;
    break;
  case STM32_MCO2SEL_LSE:
    mco2clk = STM32_LSECLK;
    break;
  case STM32_MCO2SEL_PLLPCLK:
    mco2clk = pllpclk;
    break;
  case STM32_MCO2SEL_PLLQCLK:
    mco2clk = pllqclk;
    break;
  case STM32_MCO2SEL_RTCCLK:
    mco2clk = STM32_RTCCLK;
    break;
  default:
    mco2clk = 0U;
  }
  mcodiv = 1U << ((ccp->rcc_cfgr & RCC_CFGR_MCO2PRE_Msk) >> RCC_CFGR_MCO2PRE_Pos);
#if STM32_RCC_HAS_MCOPRE_EXT == TRUE
  if (mcodiv > 1024U) {
#else
  if (mcodiv > 128U) {
#endif
    return true;
  }
  mco2clk /= mcodiv;
#endif

  /* Flash settings.*/
  flashws = ((ccp->flash_acr & FLASH_ACR_LATENCY_Msk) >> FLASH_ACR_LATENCY_Pos);
  if (flashws >= STM32_WS_THRESHOLDS) {
    return true;
  }
  if (hclk > slp->flash_thresholds[flashws]) {
    return true;
  }

  /* Writing out results.*/
  clock_points[CLK_SYSCLK]    = sysclk;
  clock_points[CLK_HSISYSCLK] = hsisysclk;
  clock_points[CLK_PLLPCLK]   = pllpclk;
  clock_points[CLK_PLLQCLK]   = pllqclk;
  clock_points[CLK_PLLRCLK]   = pllrclk;
  clock_points[CLK_HCLK]      = hclk;
  clock_points[CLK_PCLK]      = pclk;
  clock_points[CLK_PCLKTIM]   = pclktim;
  clock_points[CLK_MCO]       = mcoclk;
#if STM32_RCC_HAS_MCO2 == TRUE
  clock_points[CLK_MCO2]      = mco2clk;
#endif

  return false;
}

/**
 * @brief   Configures full clock settings.
 *
 * @param[in] ccp       pointer to clock a @p halclkcfg_t structure
 * @return              The clock configuration result.
 * @retval false        if the clock switch succeeded
 * @retval true         if the clock switch failed
 *
 * @notapi
 */
static bool hal_lld_clock_raw_config(const halclkcfg_t *ccp) {

  /* Restoring default PWR settings related clocks and sleep modes.*/
  PWR->CR1 = PWR_CR1_VOS_0;

  /* Waiting for all regulator status bits to be cleared, this means that
     power levels are stable.*/
  while ((PWR->SR2 & (PWR_SR2_VOSF | PWR_SR2_REGLPF)) != 0U) {
    /* Waiting for the regulator to be ready.*/
  }

  /* If the clock source is not HSI then we switch to HSI and reset some
     other relevant registers to their default value.*/
  if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {

    /* Making sure HSI is activated and in use.*/
    hsi16_reset();

    /* Resetting flash ACR settings to the default value.*/
    flash_set_acr(STM32_FLASH_ACR_RESET);

    /* Resetting all other clock sources and PLLs.*/
    RCC->CR = STM32_RCC_CR_RESET;
    while ((RCC->CR & RCC_CR_HSERDY) != 0U) {
      /* Waiting for oscillators to shut down.*/
    }
  }

  /* HSE setup, if required, before starting the PLL.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    hse_enable();
  }

#if STM32_RCC_HAS_HSI48
  /* HSI48 setup, if required, before starting the PLL.*/
  if ((ccp->rcc_cr & RCC_CR_HSI48ON) != 0U) {
    hsi48_enable();
  }
#endif

  /* PLL setup.*/
  RCC->PLLCFGR = ccp->rcc_pllcfgr;

  /* PLLs enabled if specified, note, HSI16 is kept running.*/
  RCC->CR =  ccp->rcc_cr | RCC_CR_HSION;

  /* PLL activation polling if required.*/
  while (true) {
    if (((ccp->rcc_cr & RCC_CR_PLLON) != 0U) && pll_not_locked()) {
      continue;
    }
    break;
  }

  /* MCO and bus dividers first.*/
  RCC->CFGR = (RCC->CFGR & RCC_CFGR_SW_Msk) | (ccp->rcc_cfgr & ~RCC_CFGR_SW_Msk);

  /* Final flash ACR settings.*/
  flash_set_acr(ccp->flash_acr);

  /* Final PWR modes.*/
  PWR->CR1 = ccp->pwr_cr1;
#if STM32_PWR_HAS_CR2 == TRUE
  PWR->CR2 = ccp->pwr_cr2;
#endif

  /* Waiting for the correct regulator state.*/
  if ((ccp->pwr_cr1 & PWR_CR1_LPR) == 0U) {
    /* Main mode selected.*/

    while ((PWR->SR2 & PWR_SR2_REGLPF) != 0U) {
      /* Waiting for the regulator to be in main mode.*/
    }
  }
  else {
    /* Low power mode selected.*/

    while ((PWR->SR2 & PWR_SR2_REGLPF) == 0U) {
      /* Waiting for the regulator to be in low power mode.*/
    }
  }

  /* Switching to the final clock source.*/
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (ccp->rcc_cfgr & RCC_CFGR_SW_Msk);
  while ((RCC->CFGR & RCC_CFGR_SWS) != ((ccp->rcc_cfgr & RCC_CFGR_SW_Msk) << RCC_CFGR_SWS_Pos)) {
    /* Waiting for clock switch.*/
  }

  /* If HSI16 is not in configuration then it is finally shut down.*/
  if ((ccp->rcc_cr & RCC_CR_HSION) == 0U) {
    hsi16_disable();
  }

  return false;
}

#if 0
/**
 * @brief   Configures clock switch-only settings.
 * @note    This is a fast reconfiguration, clock sources settings are not
 *          touched, only switches and dividers are reprogrammed.
 *
 * @param[in] cwp       pointer to clock a @p halclkswc_t structure
 * @return              The clock configuration result.
 * @retval false        if the clock switch succeeded
 * @retval true         if the clock switch failed
 *
 * @notapi
 */
static bool hal_lld_clock_raw_switch(const halclkswc_t *cwp) {

  /* PWR modes.*/
  PWR->CR1 = (PWR->CR1 & ~STM32_PWR_CR1_SAFE_ONLY_MASK) |
             (cwp->pwr_cr1 & STM32_PWR_CR1_SAFE_ONLY_MASK);

  /* Flash ACR settings.*/
  flash_set_acr(cwp->flash_acr);

  /* Switching to the final clock source.*/
  RCC->CFGR = cwp->rcc_cfgr;
  while ((RCC->CFGR & RCC_CFGR_SWS) != ((cwp->rcc_cfgr & RCC_CFGR_SW_Msk) << RCC_CFGR_SWS_Pos)) {
    /* Waiting for clock switch.*/
  }

  return false;
}
#endif
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* DMA subsystems initialization.*/
#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* NVIC initialization.*/
  nvicInit();

  /* IRQ subsystem initialization.*/
  irqInit();
}

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   STM32G0xx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {

#if !STM32_NO_INIT
  /* Reset of all peripherals.*/
  rccResetAHB(~0);
  rccResetAPBR1(~0);
  rccResetAPBR2(~0);

  /* SYSCFG clock enabled here because it is a multi-functional unit shared
     among multiple drivers.*/
  rccEnableAPBR2(RCC_APBENR2_SYSCFGEN, false);

#if defined(RCC_APBENR1_DBGEN)
  /* Enable DBG clock. Required for TIM freeze to be enabled.*/
  rccEnableAPBR1(RCC_APBENR1_DBGEN, true);
#endif

#if defined(HAL_USE_RTC) && defined(RCC_APBENR1_RTCAPBEN)
  rccEnableAPBR1(RCC_APBENR1_RTCAPBEN, true);
#endif

  /* Static PWR configurations.*/
  hal_lld_set_static_pwr();

  /* Backup domain made accessible.*/
  PWR->CR1 |= PWR_CR1_DBP;

  /* Backup domain reset.*/
  bd_reset();

  /* Static clocks setup.*/
  lse_init();
  lsi_init();

  /* Static clocks setup.*/
  hal_lld_set_static_clocks();

  /* Selecting the default clock/power/flash configuration.*/
  if (hal_lld_clock_raw_config(&hal_clkcfg_default)) {
    osalSysHalt("clkswc");
  }

  /* Backup domain initializations.*/
  bd_init();
#endif /* STM32_NO_INIT */
}

#else /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */
void stm32_clock_init(void) {

#if !STM32_NO_INIT
  /* Reset of all peripherals.*/
  rccResetAHB(~0);
  rccResetAPBR1(~0);
  rccResetAPBR2(~0);

  /* SYSCFG clock enabled here because it is a multi-functional unit shared
     among multiple drivers.*/
  rccEnableAPBR2(RCC_APBENR2_SYSCFGEN, false);

#if defined(RCC_APBENR1_DBGEN)
  /* Enable DBG clock. Required for TIM freeze to be enabled.*/
  rccEnableAPBR1(RCC_APBENR1_DBGEN, true);
#endif

#if defined(HAL_USE_RTC) && defined(RCC_APBENR1_RTCAPBEN)
  rccEnableAPBR1(RCC_APBENR1_RTCAPBEN, true);
#endif

  /* Static PWR configurations.*/
  hal_lld_set_static_pwr();

  /* Additional PWR configurations.*/
#if STM32_PWR_HAS_CR2 == TRUE
  PWR->CR2 = STM32_PWR_CR2;
#endif

  /* Core voltage setup.*/
  PWR->CR1 = STM32_VOS | PWR_CR1_DBP;
  while ((PWR->SR2 & PWR_SR2_VOSF) != 0)    /* Wait until regulator is      */
    ;                                       /* stable.                      */

  /* Backup domain reset.*/
  bd_reset();

  /* Clocks setup.*/
  lse_init();
  lsi_init();
  hsi16_init();
  hsi48_init();
  hse_init();

  /* Backup domain initializations.*/
  bd_init();

  /* PLLs activation, if required.*/
  pll_init();

  /* Static clocks setup.*/
  hal_lld_set_static_clocks();

  /* Set flash WS's for SYSCLK source.*/
#if defined(FLASH_ACR_DBG_SWEN)
  flash_set_acr(FLASH_ACR_DBG_SWEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN |
                STM32_FLASHBITS);
#else
  flash_set_acr(FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | STM32_FLASHBITS);
#endif

  /* Switching to the configured SYSCLK source if it is different from HSI16.*/
#if STM32_SW != STM32_SW_HSISYS
  RCC->CFGR |= STM32_SW;        /* Switches on the selected clock source.   */
  /* Wait until SYSCLK is stable.*/
  while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << RCC_CFGR_SWS_Pos))
    ;
#endif

#endif /* !STM32_NO_INIT */
}
#endif /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Switches to a different clock configuration
 *
 * @param[in] ccp       pointer to clock a @p halclkcfg_t structure
 * @return              The clock switch result.
 * @retval false        if the clock switch succeeded
 * @retval true         if the clock switch failed
 *
 * @notapi
 */
bool hal_lld_clock_switch_mode(const halclkcfg_t *ccp) {

  if (hal_lld_clock_check_tree(ccp)) {
    return true;
  }

  if (hal_lld_clock_raw_config(ccp)) {
    return true;
  }

  /* Updating the CMSIS variable.*/
  SystemCoreClock = hal_lld_get_clock_point(CLK_HCLK);

  return false;
}

/**
 * @brief   Returns the frequency of a clock point in Hz.
 *
 * @param[in] clkpt     clock point to be returned
 * @return              The clock point frequency in Hz or zero if the
 *                      frequency is unknown.
 *
 * @notapi
 */
halfreq_t hal_lld_get_clock_point(halclkpt_t clkpt) {

  osalDbgAssert(clkpt < CLK_ARRAY_SIZE, "invalid clock point");

  return clock_points[clkpt];
}
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/** @} */
