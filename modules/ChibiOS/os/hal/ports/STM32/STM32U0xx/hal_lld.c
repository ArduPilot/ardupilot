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
 * @file    STM32U0xx/hal_lld.c
 * @brief   STM32U0xx HAL subsystem low level driver source.
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
#define STM32_WS_THRESHOLDS                 3

/**
 * @name    Registers reset values
 * @{
 */
#define STM32_PWR_CR1_RESET                 0x00000208U
#define STM32_FLASH_ACR_RESET               0x00040600U
#define STM32_RCC_CR_RESET                  0x00000083U
#define STM32_RCC_CFGR_RESET                0x00000000U
#define STM32_RCC_PLLCFGR_RESET             0x00001000U
#define STM32_RCC_CRRCR_RESET               0x00008800U
/** @} */

/* Reserved bit to be kept at 1, ST mysteries.*/
#define FLASH_ACR_RESVD10                   (1U << 10)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_stm32u3xx.h.
 */
uint32_t SystemCoreClock = STM32_HCLK;

/**
 * @brief   Post-reset clock configuration.
 */
const halclkcfg_t hal_clkcfg_reset = {
  .pwr_cr1              = STM32_PWR_CR1_RESET | PWR_CR1_DBP,
  .rcc_cr               = STM32_RCC_CR_RESET,
  .rcc_cfgr             = STM32_RCC_CFGR_RESET,
  .rcc_pllcfgr          = STM32_RCC_PLLCFGR_RESET,
  .rcc_crrcr            = STM32_RCC_CRRCR_RESET,
  .flash_acr            = STM32_FLASH_ACR_RESET
};

/**
 * @brief   Default clock configuration.
 * @note    This is the configuration defined in mcuconf.h.
 */
const halclkcfg_t hal_clkcfg_default = {
  .pwr_cr1              = STM32_PWR_CR1 | PWR_CR1_DBP,
  .rcc_cr               = 0U
#if STM32_ACTIVATE_PLL == TRUE
                        | RCC_CR_PLLON
#endif
#if defined(STM32_HSE_BYPASS)
                        | RCC_CR_HSEBYP
#endif
#if STM32_HSE_ENABLED == TRUE
                        | RCC_CR_HSEON
#endif
#if STM32_HSI16_ENABLED == TRUE
                        | RCC_CR_HSION | RCC_CR_HSIKERON
#endif
                        | STM32_MSIRANGE
                        | RCC_CR_MSIRGSEL
#if STM32_MSIPLL_ENABLED == TRUE
                        | RCC_CR_MSIPLLEN
#endif
#if STM32_MSI_ENABLED == TRUE
                        | RCC_CR_MSION
#endif
                          ,
  .rcc_cfgr             = STM32_MCO1PRE | STM32_MCO1SEL |
                          STM32_MCO2PRE | STM32_MCO2SEL |
                          STM32_PPRE    | STM32_HPRE    |
                          STM32_SW,
  .rcc_pllcfgr          = STM32_PLLR    | STM32_PLLREN  |
                          STM32_PLLQ    | STM32_PLLQEN  |
                          STM32_PLLP    | STM32_PLLPEN  |
                          STM32_PLLN    | STM32_PLLM    |
                          STM32_PLLSRC,
  .rcc_crrcr            = 0U
#if STM32_HSI48_ENABLED == TRUE
                        | RCC_CRRCR_HSI48ON
#endif
                        ,
  .flash_acr            = (STM32_FLASH_ACR & ~FLASH_ACR_LATENCY_Msk) |
                          STM32_FLASHBITS | FLASH_ACR_RESVD10
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Dynamic clock points for this device.
 * @note    This array is pre-initialized with the defaults value because
 *          clock_init() (called in early_init()) cannot initialize this
 *          at runtime, successive DATA/BSS segment initialization would
 *          overwrite it.
 */
static halfreq_t clock_points[CLK_ARRAY_SIZE] = {
  [CLK_SYSCLK]          = STM32_SYSCLK,
  [CLK_HSI16]           = STM32_HSI16CLK,
  [CLK_HSI48]           = STM32_HSI48CLK,
  [CLK_HSE]             = STM32_HSECLK,
  [CLK_MSI]             = STM32_MSICLK,
  [CLK_PLLP]            = STM32_PLL_P_CLKOUT,
  [CLK_PLLQ]            = STM32_PLL_Q_CLKOUT,
  [CLK_PLLR]            = STM32_PLL_R_CLKOUT,
  [CLK_HCLK]            = STM32_HCLK,
  [CLK_PCLK]            = STM32_PCLK,
  [CLK_PCLKTIM]         = STM32_TIMPCLK,
  [CLK_MCO1]            = STM32_MCO1CLK,
  [CLK_MCO2]            = STM32_MCO2CLK
};

/**
 * @brief   Type of a structure representing system limits.
 */
typedef struct {
  halfreq_t             sysclk_max;
  halfreq_t             pllin_max;
  halfreq_t             pllin_min;
  halfreq_t             pllvco_max;
  halfreq_t             pllvco_min;
  halfreq_t             pllp_max;
  halfreq_t             pllp_min;
  halfreq_t             pllq_max;
  halfreq_t             pllq_min;
  halfreq_t             pllr_max;
  halfreq_t             pllr_min;
  halfreq_t             flash_thresholds[STM32_WS_THRESHOLDS];
} system_limits_t;

/**
 * @brief   System limits for range 1.
 */
static const system_limits_t vos_range1 = {
  .sysclk_max           = STM32_RANGE1_SYSCLK_MAX,
  .pllin_max            = STM32_RANGE1_PLLIN_MAX,
  .pllin_min            = STM32_RANGE1_PLLIN_MIN,
  .pllvco_max           = STM32_RANGE1_PLLVCO_MAX,
  .pllvco_min           = STM32_RANGE1_PLLVCO_MIN,
  .pllp_max             = STM32_RANGE1_PLLP_MAX,
  .pllp_min             = STM32_RANGE1_PLLP_MIN,
  .pllq_max             = STM32_RANGE1_PLLQ_MAX,
  .pllq_min             = STM32_RANGE1_PLLQ_MIN,
  .pllr_max             = STM32_RANGE1_PLLR_MAX,
  .pllr_min             = STM32_RANGE1_PLLR_MIN,
  .flash_thresholds     = {STM32_RANGE1_0WS_THRESHOLD, STM32_RANGE1_1WS_THRESHOLD,
                           STM32_RANGE1_2WS_THRESHOLD}
};

/**
 * @brief   System limits for range 2.
 */
static const system_limits_t vos_range2 = {
  .sysclk_max           = STM32_RANGE2_SYSCLK_MAX,
  .pllin_max            = STM32_RANGE2_PLLIN_MAX,
  .pllin_min            = STM32_RANGE2_PLLIN_MIN,
  .pllvco_max           = STM32_RANGE2_PLLVCO_MAX,
  .pllvco_min           = STM32_RANGE2_PLLVCO_MIN,
  .pllp_max             = STM32_RANGE2_PLLP_MAX,
  .pllp_min             = STM32_RANGE2_PLLP_MIN,
  .pllq_max             = STM32_RANGE2_PLLQ_MAX,
  .pllq_min             = STM32_RANGE2_PLLQ_MIN,
  .pllr_max             = STM32_RANGE2_PLLR_MAX,
  .pllr_min             = STM32_RANGE2_PLLR_MIN,
  .flash_thresholds     = {STM32_RANGE2_0WS_THRESHOLD, STM32_RANGE2_1WS_THRESHOLD,
                           STM32_RANGE2_2WS_THRESHOLD}
};

#if 0
/**
 * @brief   System limits for LPR.
 */
static const system_limits_t vos_range2 = {
  .sysclk_max           = STM32_LPR_SYSCLK_MAX,
  .pllin_max            = STM32_LPR_PLLIN_MAX,
  .pllin_min            = STM32_LPR_PLLIN_MIN,
  .pllvco_max           = STM32_LPR_PLLVCO_MAX,
  .pllvco_min           = STM32_LPR_PLLVCO_MIN,
  .pllp_max             = STM32_LPR_PLLP_MAX,
  .pllp_min             = STM32_LPR_PLLP_MIN,
  .pllq_max             = STM32_LPR_PLLQ_MAX,
  .pllq_min             = STM32_LPR_PLLQ_MIN,
  .pllr_max             = STM32_LPR_PLLR_MAX,
  .pllr_min             = STM32_LPR_PLLR_MIN,
  .flash_thresholds     = {STM32_LPR_0WS_THRESHOLD, STM32_LPR_1WS_THRESHOLD,
                           STM32_LPR_2WS_THRESHOLD}
};
#endif
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#include "stm32_bd.inc"

/**
 * @brief   Configures the SYSCFG unit.
 */
static void hal_lld_set_static_syscfg(void) {

  /* SYSCFG clock enabled.*/
  rccEnableAPBR2(RCC_APBENR2_SYSCFGEN, false);

  halRegMaskedWrite32X(&SYSCFG->CFGR1, ~SYSCFG_CFGR1_MEM_MODE_Msk, STM32_SYSCFG_CFGR1, true);
}

/**
 * @brief   Configures the PWR unit.
 * @note    CR is not initialized inside this function except for DBP.
 */
static void hal_lld_set_static_pwr(void) {

  /* PWR clock enabled.*/
  rccEnablePWRInterface(false);

  /* Enable write access to Backup domain.*/
  halRegWrite32X(&PWR->CR1, STM32_PWR_CR1_RESET | PWR_CR1_DBP, true);

  /* Static settings for PWR registers.*/
  halRegWrite32X(&PWR->CR2,   STM32_PWR_CR2,   true);
  halRegWrite32X(&PWR->CR3,   STM32_PWR_CR3,   true);
  halRegWrite32X(&PWR->CR4,   STM32_PWR_CR4,   true);
  halRegWrite32X(&PWR->PUCRA, STM32_PWR_PUCRA, true);
  halRegWrite32X(&PWR->PDCRA, STM32_PWR_PDCRA, true);
  halRegWrite32X(&PWR->PDCRB, STM32_PWR_PDCRB, true);
  halRegWrite32X(&PWR->PUCRB, STM32_PWR_PUCRB, true);
  halRegWrite32X(&PWR->PUCRC, STM32_PWR_PUCRC, true);
  halRegWrite32X(&PWR->PDCRC, STM32_PWR_PDCRC, true);
  halRegWrite32X(&PWR->PUCRD, STM32_PWR_PUCRD, true);
  halRegWrite32X(&PWR->PDCRD, STM32_PWR_PDCRD, true);
}

/**
 * @brief   Initializes static muxes and dividers.
 */
static void hal_lld_set_static_clocks(void) {

  /* CCIPR registers initialization, note.*/
  halRegWrite32X(&RCC->CCIPR,
                 STM32_ADCSEL     | STM32_CLK48SEL    |
                 STM32_TIM15SEL   | STM32_TIM1SEL     |
                 STM32_LPTIM3SEL  | STM32_LPTIM2SEL   |
                 STM32_LPTIM1SEL  | STM32_I2C3SEL     |
                 STM32_I2C1SEL    | STM32_LPUART1SEL  |
                 STM32_LPUART2SEL | STM32_LPUART3SEL  |
                 STM32_USART2SEL  | STM32_USART1SEL,
                 true);
}

/**
 * @brief   Switches to a different clock configuration.
 *
 * @param[in] ccp       pointer to clock a @p halclkcfg_t structure
 * @return              The clock switch result.
 * @retval false        if the clock switch succeeded
 * @retval true         if the clock switch failed
 *
 * @notapi
 */
static bool hal_lld_clock_configure(const halclkcfg_t *ccp) {
  uint32_t wtmask;

  /* Setting flash ACR to the safest value while the clock tree is
     reconfigured. we don't know the current clock settings.*/
  halRegWrite32X(&FLASH->ACR,
                 FLASH_ACR_DBG_SWEN | FLASH_ACR_RESVD10 | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_2WS,
                 true);

  /* Disabling low power run mode if activated, not touching current
     VOS range yet.*/
  halRegClear32X(&PWR->CR1, PWR_CR1_LPR, false);
  if (halRegWaitAllClear32X(&PWR->SR2, PWR_SR2_REGLPF,
                            STM32_REGULATORS_TRANSITION_TIME, NULL)) {
    return true;
  }

  /* Resetting and restarting MSI without touching the other settings, it
     is required during the (re)configuration.*/
  halRegMaskedWrite32X(&RCC->CR,
                       RCC_CR_MSIRANGE_Msk | RCC_CR_MSION_Msk,
                       RCC_CR_MSIRANGE_4M | RCC_CR_MSION,
                       true);
  if (halRegWaitAllSet32X(&RCC->CR, RCC_CR_MSIRDY,
                          STM32_HSI_HSE_MSI_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* Switching to MSI.*/
  halRegWrite32X(&RCC->CFGR, STM32_RCC_CFGR_RESET, true);
  if (halRegWaitMatch32X(&RCC->CFGR,
                         RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_MSI,
                         STM32_SYSCLK_SWITCH_TIME,
                         NULL)) {
    return true;
  }

  /* Resetting clocks-related settings, this includes MSI.*/
  halRegWrite32X(&RCC->CR, STM32_RCC_CR_RESET, true);
  halRegWrite32X(&RCC->PLLCFGR, STM32_RCC_PLLCFGR_RESET, true);

  /* Post-reset voltage scaling enforcing.*/
  halRegWrite32X(&PWR->CR1, STM32_PWR_CR1_RESET, true);
  if (halRegWaitAllClear32X(&PWR->SR2, PWR_SR2_VOSF,
                            STM32_REGULATORS_TRANSITION_TIME, NULL)) {
    return true;
  }

  /* Enabling all required oscillators at same time, MSI enforced active,
     PLL not enabled yet.*/
  halRegWrite32X(&RCC->CR,
                  (ccp->rcc_cr | RCC_CR_MSION) & ~RCC_CR_PLLON,
                  true);

  /* Starting also HSI48 if required, waiting for it to become stable first
     because it is the fastest one.*/
  halRegWrite32X(&RCC->CRRCR, ccp->rcc_crrcr, true);
  if ((ccp->rcc_crrcr & RCC_CRRCR_HSI48ON) != 0U) {
    if (halRegWaitAllSet32X(&RCC->CRRCR,
                            RCC_CRRCR_HSI48RDY,
                            STM32_HSI48_STARTUP_TIME,
                            NULL)) {
      return true;
    }
  }

  /* Adding to the "wait mask" the status bits of other enabled oscillators.*/
  wtmask = RCC_CR_MSIRDY;                  /* Known to be ready already.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    wtmask |= RCC_CR_HSERDY;
  }
  if ((ccp->rcc_cr & RCC_CR_HSION) != 0U) {
    wtmask |= RCC_CR_HSIRDY;
  }
  if (halRegWaitAllSet32X(&RCC->CR,
                          wtmask,
                          STM32_HSI_HSE_MSI_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* Final programmable voltage scaling configuration. */
  halRegWrite32X(&PWR->CR1, ccp->pwr_cr1 | PWR_CR1_DBP, true);
  if (halRegWaitAllClear32X(&PWR->SR2, PWR_SR2_VOSF,
                            STM32_REGULATORS_TRANSITION_TIME, NULL)) {
    return true;
  }

  /* Enabling also PLLs if required by the configuration else skipping.*/
  if ((ccp->rcc_cr & RCC_CR_PLLON) != 0U) {
    halRegWrite32X(&RCC->PLLCFGR, ccp->rcc_pllcfgr, true);
    halRegWrite32X(&RCC->CR, ccp->rcc_cr | RCC_CR_MSION, true);
    if (halRegWaitAllSet32X(&RCC->CR,
                            RCC_CR_PLLRDY,
                            STM32_PLL_STARTUP_TIME,
                            NULL)) {
      return true;
    }
  }

  /* Final RCC CFGR settings (prescalers, MCO, etc).*/
  halRegWrite32X(&RCC->CFGR, ccp->rcc_cfgr, true);

  /* Final flash ACR settings according to the target configuration.*/
  halRegWrite32X(&FLASH->ACR, ccp->flash_acr | FLASH_ACR_RESVD10, true);

  /* Waiting for the requested SYSCLK source to become active. */
  if (halRegWaitMatch32X(&RCC->CFGR,
                         RCC_CFGR_SWS_Msk, (ccp->rcc_cfgr & RCC_CFGR_SW_Msk) << RCC_CFGR_SWS_Pos,
                         STM32_SYSCLK_SWITCH_TIME,
                         NULL)) {
    return true;
  }

  /* Final RCC_CR value, MSIS could go off at this point if it is not part
     of the mask.*/
  halRegWrite32X(&RCC->CR, ccp->rcc_cr, true);

  return false;
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
  static const uint32_t msirange[12] = {100000U, 200000U, 400000U,
                                        800000U, 1000000U, 2000000U,
                                        4000000U, 8000000U, 16000000U,
                                        24000000U, 32000000U, 48000000U};
  static const uint32_t msipllrange[12] = {98304U, 196608U, 393216U,
                                           786432U, 1016000U, 1999000U,
                                           3998000U, 7995000U, 15991000U,
                                           23986000U, 32014000U, 48005000U};
  static const uint32_t hprediv[16] = {1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U,
                                       2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
  static const uint32_t pprediv[16] = {1U, 1U, 1U, 1U, 2U, 4U, 8U, 16U};
  const system_limits_t *slp;
  halfreq_t hsi16clk = 0U, hsi48clk = 0U, hseclk = 0U, msiclk = 0U, pllselclk;
  halfreq_t pllpclk = 0U, pllqclk = 0U, pllrclk = 0U;
  halfreq_t sysclk, hclk, pclk, pclktim, mco1clk, mco2clk;
  uint32_t mcodiv, flashws, msiidx;

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

  /* HSE clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    hseclk = STM32_HSECLK;
  }

  /* HSI16 clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSION) != 0U) {
    hsi16clk = STM32_HSI16CLK;
  }

  /* HSI48 clock.*/
  if ((ccp->rcc_crrcr & RCC_CRRCR_HSI48ON) != 0U) {
    hsi48clk = STM32_HSI48CLK;
  }

  /* MSI clock.*/
  msiidx = (ccp->rcc_cr & RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos;
  if (msiidx >= 12) {
    return true;
  }
  if ((ccp->rcc_cr & RCC_CR_MSIPLLEN) != 0U) {
    msiclk = msipllrange[msiidx];
  }
  else {
    msiclk = msirange[msiidx];
  }

  /* PLL MUX clock.*/
  switch (ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLSRC_Msk) {
  case RCC_PLLCFGR_PLLSRC_MSI:
    pllselclk = msiclk;
    break;
  case RCC_PLLCFGR_PLLSRC_HSI16:
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
    if ((pllndiv < STM32_PLLN_VALUE_MIN) ||
        (pllndiv > STM32_PLLN_VALUE_MAX)) {
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
  case RCC_CFGR_SW_MSI:
    sysclk = msiclk;
    break;
  case RCC_CFGR_SW_HSI16:
    sysclk = hsi16clk;
    break;
  case RCC_CFGR_SW_HSE:
    sysclk = hseclk;
    break;
  case RCC_CFGR_SW_PLLR:
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
  switch (ccp->rcc_cfgr & RCC_CFGR_MCO1SEL_Msk) {
  case RCC_CFGR_MCO1SEL_SYSCLK:
    mco1clk = sysclk;
    break;
  case RCC_CFGR_MCO1SEL_MSI:
    mco1clk = msiclk;
    break;
  case RCC_CFGR_MCO1SEL_HSI16:
    mco1clk = hsi16clk;
    break;
  case RCC_CFGR_MCO1SEL_HSE:
    mco1clk = hseclk;
    break;
  case RCC_CFGR_MCO1SEL_PLLRCLK:
    mco1clk = pllrclk;
    break;
  case RCC_CFGR_MCO1SEL_LSI:
    mco1clk = STM32_LSICLK;
    break;
  case RCC_CFGR_MCO1SEL_LSE:
    mco1clk = STM32_LSECLK;
    break;
  case RCC_CFGR_MCO1SEL_HSI48:
    mco1clk = hsi48clk;
    break;
  case RCC_CFGR_MCO1SEL_RTCCLK:
    mco1clk = STM32_RTCCLK;
    break;
  case RCC_CFGR_MCO1SEL_RTCWKP:
    mco1clk = 0U; /* TODO: should come from RTC.*/
    break;
  default:
    mco1clk = 0U;
  }
  mcodiv = 1U << ((ccp->rcc_cfgr & RCC_CFGR_MCO1PRE_Msk) >> RCC_CFGR_MCO1PRE_Pos);
  if (mcodiv > 1024U) {
    return true;
  }
  mco1clk /= mcodiv;

  /* MCO2 clock.*/
  switch (ccp->rcc_cfgr & RCC_CFGR_MCO2SEL_Msk) {
  case RCC_CFGR_MCO2SEL_SYSCLK:
    mco2clk = sysclk;
    break;
  case RCC_CFGR_MCO2SEL_MSI:
    mco2clk = msiclk;
    break;
  case RCC_CFGR_MCO2SEL_HSI16:
    mco2clk = hsi16clk;
    break;
  case RCC_CFGR_MCO2SEL_HSE:
    mco2clk = hseclk;
    break;
  case RCC_CFGR_MCO2SEL_PLLRCLK:
    mco2clk = pllrclk;
    break;
  case RCC_CFGR_MCO2SEL_LSI:
    mco2clk = STM32_LSICLK;
    break;
  case RCC_CFGR_MCO2SEL_LSE:
    mco2clk = STM32_LSECLK;
    break;
  case RCC_CFGR_MCO2SEL_HSI48:
    mco2clk = hsi48clk;
    break;
  case RCC_CFGR_MCO2SEL_RTCCLK:
    mco2clk = STM32_RTCCLK;
    break;
  case RCC_CFGR_MCO2SEL_RTCWKP:
    mco2clk = 0U; /* TODO: should come from RTC.*/
    break;
  default:
    mco2clk = 0U;
  }
  mcodiv = 1U << ((ccp->rcc_cfgr & RCC_CFGR_MCO2PRE_Msk) >> RCC_CFGR_MCO2PRE_Pos);
  if (mcodiv > 1024U) {
    return true;
  }
  mco2clk /= mcodiv;

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
  clock_points[CLK_HSI16]     = hsi16clk;
  clock_points[CLK_HSI48]     = hsi48clk;
  clock_points[CLK_HSE]       = hseclk;
  clock_points[CLK_MSI]       = msiclk;
  clock_points[CLK_PLLP]      = pllpclk;
  clock_points[CLK_PLLQ]      = pllqclk;
  clock_points[CLK_PLLR]      = pllrclk;
  clock_points[CLK_HCLK]      = hclk;
  clock_points[CLK_PCLK]      = pclk;
  clock_points[CLK_PCLKTIM]   = pclktim;
  clock_points[CLK_MCO1]      = mco1clk;
  clock_points[CLK_MCO2]      = mco2clk;

  return false;
}
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

/**
 * @brief   STM32U0xx clocks and PLL initialization.
 * @note    This function should be invoked just after the system reset in
 *          order to accelerate boot.
 *
 * @special
 */
void stm32_clock_init(void) {
  int32_t div;

  /* Enabling TIM7 for timeout handling.*/
  rccResetTIM7();
  rccEnableTIM7(false);
  /* Clamp divider to avoid underflow on low TIMPCLK values. */
  div = ((int32_t)STM32_TIMPCLK / 1000000) - 1;
  if (div < 0) {
    div = 0;
  }
  halRegWrite32X(&TIM7->PSC, (uint32_t)div, true);
//  halRegWrite32X(&TIM7->ARR, 0xFFFFU, true);
  halRegWrite32X(&TIM7->EGR, TIM_EGR_UG, false);
  halRegWrite32X(&TIM7->CR1, TIM_CR1_CEN, true);

#if !STM32_NO_INIT
  /* Reset of all peripherals.
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB(~0);
  rccResetAPBR1(~0);
  rccResetAPBR2(~0);

  /* RTC APB clock enable.*/
#if (HAL_USE_RTC == TRUE) && defined(RCC_APB3ENR_RTCAPBEN)
  rccEnableAPB3(RCC_APB3ENR_RTCAPBEN, true);
#endif

  /* Static SYSCFG configurations.*/
  hal_lld_set_static_syscfg();

  /* Static PWR configurations.*/
  hal_lld_set_static_pwr();

  /* Backup domain reset if required.*/
  bd_reset();

  /* Static oscillators setup.*/
  lse_init();
  lsi_init();

  /* Static clocks setup (dividers, CCIPR selections).*/
  hal_lld_set_static_clocks();

  /* Selecting the default clock configuration. */
  halSftFailOnError(hal_lld_clock_configure(&hal_clkcfg_default), "clkinit");

  /* Backup domain initializations.*/
  bd_init();
#endif /* STM32_NO_INIT */
}

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
  int32_t div;

  if (hal_lld_clock_check_tree(ccp)) {
    return true;
  }

  if (hal_lld_clock_configure(ccp)) {
    return true;
  }

  /* Updating timeout counter clock, contemplating the case where the clock
     source frequency becomes lower than 1MHz.*/
  div = ((int)hal_lld_get_clock_point(CLK_PCLKTIM) / 1000000) - 1;
  if (div < 0) {
    div = 0;
  }
  halRegWrite32X(&TIM7->PSC, (uint32_t)div, true);
  halRegWrite32X(&TIM7->EGR, TIM_EGR_UG, false);

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
