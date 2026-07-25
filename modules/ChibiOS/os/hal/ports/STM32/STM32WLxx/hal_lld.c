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
 * @file    STM32WLxx/hal_lld.c
 * @brief   STM32WLxx HAL subsystem low level driver source.
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
#define STM32_WS_THRESHOLDS             3

/**
 * @brief   FLASH_ACR reset value.
 */
#define STM32_FLASH_ACR_RESET           0x00000600UL

/**
 * @brief   RCC_CR reset value.
 */
#define STM32_RCC_CR_RESET              0x00000061UL

/**
 * @brief   PWR CR bits safe for fast switch.
 */
#define STM32_PWR_CR1_SAFE_ONLY_MASK    (PWR_CR1_LPR             |          \
                                         PWR_CR1_FPDS            |          \
                                         PWR_CR1_SUBGHZSPINSSSEL |          \
                                         PWR_CR1_LPMS_Msk)

/**
 * @brief   MSI range array size.
 */
#define STM32_MSIRANGE_ARRAY_SIZE       12

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_stm32wlxx.h.
 */
uint32_t SystemCoreClock = STM32_HCLK;

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Post-reset clock configuration.
 */
const halclkcfg_t hal_clkcfg_reset = {
  .pwr_cr1              = PWR_CR1_VOS_0,
  .pwr_cr2              = 0U,
  .rcc_cr               = RCC_CR_MSIRANGE_6 | RCC_CR_MSION,
  .rcc_cfgr             = RCC_CFGR_PPRE2F | RCC_CFGR_PPRE1F | RCC_CFGR_HPREF,
  .rcc_extcfgr          = 0U,
  .rcc_pllcfgr          = 0U,
  .flash_acr            = FLASH_ACR_DCEN | FLASH_ACR_ICEN
};

/**
 * @brief   Default clock configuration.
 */
const halclkcfg_t hal_clkcfg_default = {
  .pwr_cr1              = STM32_VOS | PWR_CR1_DBP,
  .pwr_cr2              = STM32_PWR_CR2,
  .rcc_cr               = 0U
#if STM32_MSIPLL_ENABLED
                        | STM32_MSIRANGE | RCC_CR_MSIPLLEN | RCC_CR_MSION
#else
                        | STM32_MSIRANGE |                   RCC_CR_MSION
#endif
#if STM32_HSI16_ENABLED
                        | RCC_CR_HSIKERON | RCC_CR_HSION
#endif
#if STM32_HSE32_ENABLED
                        | RCC_CR_HSEON
#endif
#if STM32_ACTIVATE_PLL
                        | RCC_CR_PLLON
#endif
                        ,
  .rcc_cfgr             = STM32_MCOPRE | STM32_MCOSEL |
                          STM32_PPRE2  | STM32_PPRE1  |
                          STM32_HPRE   | STM32_SW,
  .rcc_extcfgr          = STM32_SHDHPRE | STM32_C2HPRE,
  .rcc_pllcfgr          = STM32_PLLR   | STM32_PLLREN |
                          STM32_PLLQ   | STM32_PLLQEN |
                          STM32_PLLP   | STM32_PLLPEN |
                          STM32_PLLN   | STM32_PLLM   |
                          STM32_PLLSRC,
  .flash_acr            = FLASH_ACR_DCEN   | FLASH_ACR_ICEN |
                          FLASH_ACR_PRFTEN | STM32_FLASHBITS
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
  STM32_SYSCLK,
  STM32_PLL_P_CLKOUT,
  STM32_PLL_Q_CLKOUT,
  STM32_PLL_R_CLKOUT,
  STM32_HCLK,
  STM32_PCLK1,
  STM32_TIMP1CLK,
  STM32_HCLK2,
  STM32_PCLK2,
  STM32_TIMP2CLK,
  STM32_HCLK3,
  STM32_MCOCLK
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
 * @brief   System limits for VOS RANGE1.
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
  .flash_thresholds     = {STM32_VOS1_0WS_THRESHOLD,
                           STM32_VOS1_1WS_THRESHOLD,
                           STM32_VOS1_2WS_THRESHOLD}
};

/**
 * @brief   System limits for VOS RANGE2.
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
  .flash_thresholds     = {STM32_VOS2_0WS_THRESHOLD,
                           STM32_VOS2_1WS_THRESHOLD,
                           STM32_VOS2_2WS_THRESHOLD}
};
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Safe setting of flash ACR register.
 *
 * @param[in] acr       value for the ACR register
 */
__STATIC_INLINE void flash_set_acr(uint32_t acr) {

  FLASH->ACR = acr;
  while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != (acr & FLASH_ACR_LATENCY_Msk)) {
    /* Waiting for flash wait states setup.*/
  }
}

/**
 * @brief   Configures the PWR unit.
 * @note    CR1, CR2 and CR5 are not initialized inside this function.
 */
__STATIC_INLINE void hal_lld_set_static_pwr(void) {

  /* Static PWR configurations.*/
  PWR->CR3 = STM32_PWR_CR3;
  PWR->CR4 = STM32_PWR_CR4;
  PWR->PUCRA = STM32_PWR_PUCRA;
  PWR->PDCRA = STM32_PWR_PDCRA;
  PWR->PUCRB = STM32_PWR_PUCRB;
  PWR->PDCRB = STM32_PWR_PDCRB;
  PWR->PUCRC = STM32_PWR_PUCRC;
  PWR->PDCRC = STM32_PWR_PDCRC;
  PWR->PUCRH = STM32_PWR_PUCRH;
  PWR->PDCRH = STM32_PWR_PDCRH;
}

/**
 * @brief   Initializes static muxes and dividers.
 */
__STATIC_INLINE void hal_lld_set_static_clocks(void) {

  uint32_t ccipr;

  /* Clock-related settings (dividers, MCO etc).*/
  RCC->CFGR = STM32_MCOPRE | STM32_MCOSEL | STM32_STOPWUCK |
              STM32_PPRE2  | STM32_PPRE1  | STM32_HPRE;
  RCC->EXTCFGR = STM32_SHDHPRE | STM32_C2HPRE;

  /* CCIPR register initialization, note, must take care of the _OFF
     pseudo settings.*/
  ccipr = STM32_RNGSEL    | STM32_ADCSEL    | STM32_LPTIM3SEL  |
          STM32_LPTIM2SEL | STM32_LPTIM1SEL | STM32_I2C3SEL    |
          STM32_I2C2SEL   | STM32_I2C1SEL   | STM32_LPUART1SEL |
          STM32_SPI2S2SEL | STM32_USART2SEL | STM32_USART1SEL;

  RCC->CCIPR = ccipr;
}

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
static bool hal_lld_check_pll(const system_limits_t *slp,
                              uint32_t cfgr,
                              halfreq_t selclk,
                              halfreq_t *pclkp,
                              halfreq_t *qclkp,
                              halfreq_t *rclkp) {

  uint32_t mdiv, ndiv, pdiv, qdiv, rdiv;
  halfreq_t vcoclk, pclk = 0U, qclk = 0U, rclk = 0U;

  /* PLL M divider.*/
  mdiv = ((cfgr & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos) + 1U;

  /* PLL N divider.*/
  ndiv = (cfgr & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
  if (ndiv < STM32_PLLN_VALUE_MIN) {
      return true;
  }

  /* PLL VCO frequency.*/
  vcoclk = (selclk / (halfreq_t)mdiv) * (halfreq_t)ndiv;

  if ((vcoclk < slp->pllvco_min) || (vcoclk > slp->pllvco_max)) {
    return true;
  }

  /* PLL P output frequency.*/
  pdiv = ((cfgr & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos) + 1;

  if ((pdiv < STM32_PLLP_VALUE_MIN) || (pdiv > STM32_PLLP_VALUE_MAX)) {
    return true;
  }

  if ((cfgr & RCC_PLLCFGR_PLLPEN) != 0U) {
    pclk = vcoclk / pdiv;

    if ((pclk < slp->pllp_min) || (pclk > slp->pllp_max)) {
      return true;
    }
  }

  /* PLL Q output frequency.*/
  qdiv = ((cfgr & RCC_PLLCFGR_PLLQ_Msk) >> RCC_PLLCFGR_PLLQ_Pos) + 1;

  if (qdiv < STM32_PLLQ_VALUE_MIN) {
    return true;
  }

  if ((cfgr & RCC_PLLCFGR_PLLQEN) != 0U) {
    qclk = vcoclk / qdiv;

    if ((qclk < slp->pllq_min) || (qclk > slp->pllq_max)) {
      return true;
    }
  }

  /* PLL R output frequency.*/
  rdiv = ((cfgr & RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos) + 1;

  if (rdiv < STM32_PLLR_VALUE_MIN) {
    return true;
  }

  if ((cfgr & RCC_PLLCFGR_PLLREN) != 0U) {
    rclk = vcoclk / rdiv;

    if ((rclk < slp->pllr_min) || (rclk > slp->pllr_max)) {
      return true;
    }
  }

  *pclkp = pclk;
  *qclkp = qclk;
  *rclkp = rclk;

  return false;
}

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

  static const uint32_t msirange[STM32_MSIRANGE_ARRAY_SIZE] =
                                       {100000U, 200000U, 400000U,
                                        800000U, 1000000U, 2000000U,
                                        4000000U, 8000000U, 16000000U,
                                        24000000U, 32000000U, 48000000U};
  static const uint32_t hprediv[16] = {1U, 3U, 5U, 1U, 1U, 6U, 10U, 32U,
                                       2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
  static const uint32_t pprediv[16] = {1U, 1U, 1U, 1U, 2U, 4U, 8U, 16U};
  const system_limits_t *slp;

  halfreq_t hsi16clk = 0U, hseclk = 0U, msiclk = 0U, pllselclk;
  halfreq_t pllpclk = 0U, pllqclk = 0U, pllrclk = 0U;
  halfreq_t sysclk, hclk, pclk1, pclk2, pclk1tim, pclk2tim, hclk2, hclk3, mcoclk;
  uint32_t mcodiv;
  uint32_t msiidx, flashws;

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

  /* MSI clock.*/
  msiidx = (uint8_t)((ccp->rcc_cr & RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos);
  if (msiidx >= STM32_MSIRANGE_ARRAY_SIZE) {
    return true;
  }
  msiclk = msirange[msiidx];

  /* HSI16 clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSION) != 0U) {
    hsi16clk = STM32_HSI16CLK;
  }

  /* HSE32 clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    hseclk = STM32_HSECLK;
  }

  /* PLL MUX clock.*/
  switch (ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLSRC_Msk) {
  case STM32_PLLSRC_HSI16:
    pllselclk = hsi16clk;
    break;
  case STM32_PLLSRC_HSE:
    pllselclk = hseclk;
    break;
  case STM32_PLLSRC_MSI:
    pllselclk = msiclk;
    break;
  default:
    pllselclk = 0U;
  }

  /* PLL outputs.*/
  if ((ccp->rcc_cr & RCC_CR_PLLON) != 0U) {
    if (hal_lld_check_pll(slp, ccp->rcc_pllcfgr, pllselclk,
                          &pllpclk, &pllqclk, &pllrclk)) {
      return true;
    }
  }

  /* SYSCLK frequency.*/
  switch (ccp->rcc_cfgr & RCC_CFGR_SW_Msk) {
  case STM32_SW_HSI16:
    sysclk = hsi16clk;
    break;
  case STM32_SW_HSE:
    sysclk = hseclk;
    break;
  case STM32_SW_PLL:
    sysclk = pllrclk;
    break;
  case STM32_SW_MSI:
    sysclk = msiclk;
    break;
  default:
    sysclk = 0U;
  }

  if ((sysclk > slp->sysclk_max) || sysclk == 0U) {
    return true;
  }

  /* LPRUN sysclk check.*/
  if (((ccp->pwr_cr1 & PWR_CR1_LPR_Msk) != 0U) && (sysclk > STM32_LPRUN_SYSCLK_MAX)) {
    return true;
  }

  /* HCLK frequency.*/
  hclk = sysclk / hprediv[(ccp->rcc_cfgr & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos];

  /* PPRE1 frequency.*/
  pclk1 = hclk / pprediv[(ccp->rcc_cfgr & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos];
  if ((ccp->rcc_cfgr & RCC_CFGR_PPRE1_Msk) < RCC_CFGR_PPRE1_2) {
    pclk1tim = pclk1;
  }
  else {
    pclk1tim = pclk1 * 2U;
  }

  /* PPRE2 frequency.*/
  pclk2 = hclk / pprediv[(ccp->rcc_cfgr & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos];
  if ((ccp->rcc_cfgr & RCC_CFGR_PPRE1_Msk) < RCC_CFGR_PPRE2_2) {
    pclk2tim = pclk2;
  }
  else {
    pclk2tim = pclk2 * 2U;
  }

  /* HCLK2 frequency.*/
  hclk2 = sysclk / hprediv[(ccp->rcc_extcfgr & RCC_EXTCFGR_C2HPRE_Msk) >> RCC_EXTCFGR_C2HPRE_Pos];

  /* HCLK3 frequency.*/
  hclk3 = sysclk / hprediv[(ccp->rcc_extcfgr & RCC_EXTCFGR_SHDHPRE_Msk) >> RCC_EXTCFGR_SHDHPRE_Pos];

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
  case STM32_MCOSEL_HSE32:
    mcoclk = STM32_HSE32CLK;
    break;
  case STM32_MCOSEL_PLLRCLK:
    mcoclk = pllrclk;
    break;
  case STM32_MCOSEL_PLLPCLK:
    mcoclk = pllpclk;
    break;
  case STM32_MCOSEL_PLLQCLK:
    mcoclk = pllqclk;
    break;
  case STM32_MCOSEL_LSI:
    mcoclk = STM32_LSICLK;
    break;
  case STM32_MCOSEL_LSE:
    mcoclk = STM32_LSECLK;
    break;
  case STM32_MCOSEL_MSI:
    mcoclk = msiclk;
    break;
  default:
    mcoclk = 0U;
  }

  mcodiv = 1U << ((ccp->rcc_cfgr & RCC_CFGR_MCOPRE_Msk) >> RCC_CFGR_MCOPRE_Pos);

  if (mcodiv > 16U) {
    return true;
  }
  mcoclk /= mcodiv;

  /* Flash settings.*/
  flashws = ((ccp->flash_acr & FLASH_ACR_LATENCY_Msk) >> FLASH_ACR_LATENCY_Pos);

  if (flashws >= STM32_WS_THRESHOLDS) {
    return true;
  }

  if (hclk3 > slp->flash_thresholds[flashws]) {
    return true;
  }

  /* Writing out results.*/
  clock_points[CLK_SYSCLK]   = sysclk;
  clock_points[CLK_PLLPCLK]  = pllpclk;
  clock_points[CLK_PLLQCLK]  = pllqclk;
  clock_points[CLK_PLLRCLK]  = pllrclk;
  clock_points[CLK_HCLK]     = hclk;
  clock_points[CLK_PCLK1]    = pclk1;
  clock_points[CLK_PCLK1TIM] = pclk1tim;
  clock_points[CLK_HCLK2]    = hclk2;
  clock_points[CLK_PCLK2]    = pclk2;
  clock_points[CLK_PCLK2TIM] = pclk2tim;
  clock_points[CLK_HCLK3]    = hclk3;
  clock_points[CLK_MCO]      = mcoclk;

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

  /* If the clock source is not MSI then we switch to MSI and reset some
     other relevant registers to their default value.*/
  if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI) {
    msi_reset();

    /* Resetting flash ACR settings to the default value.*/
    flash_set_acr(STM32_FLASH_ACR_RESET);

    /* Resetting all other clock sources and PLLs.*/
    RCC->CR = STM32_RCC_CR_RESET;
    while ((RCC->CR & (RCC_CR_HSIRDY | RCC_CR_HSERDY)) != 0U) {
      /* Waiting for oscillators to shut down.*/
    }

  }

  /* HSE32 setup, if required, before starting the PLL.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    hse32_enable();
  }

  /* HSI setup.*/
  if ((ccp->rcc_cr & RCC_CR_HSION) != 0U) {
    hsi16_enable();
  }

  /* PLL setup.*/
  RCC->PLLCFGR = ccp->rcc_pllcfgr;

  /* HSI, HSE32, PLL enabled if specified.*/
  RCC->CR = ccp->rcc_cr;

  /* PLL activation polling if required.*/
  if ((ccp->rcc_cr & RCC_CR_PLLON) != 0U) {
    pll_wait_lock();
  }

  /* MCO and bus dividers first.*/
  RCC->CFGR = (RCC->CFGR & RCC_CFGR_SW_Msk) | (ccp->rcc_cfgr & ~RCC_CFGR_SW_Msk);

  /* Final flash ACR settings.*/
  flash_set_acr(ccp->flash_acr);

  /* Final PWR modes.*/
  PWR->CR1 = ccp->pwr_cr1;
  PWR->CR2 = ccp->pwr_cr2;

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
  RCC->EXTCFGR = ccp->rcc_extcfgr;
  while ((RCC->CFGR & RCC_CFGR_SWS) != ((ccp->rcc_cfgr & RCC_CFGR_SW_Msk) << RCC_CFGR_SWS_Pos)) {
    /* Waiting for clock switch.*/
  }

  /* If MSI is not in configuration then it is finally shut down.*/
  if ((ccp->rcc_cr & RCC_CR_MSION) == 0U) {
    msi_disable();
  }

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

  /* IRQ subsystem initialization.*/
  irqInit();
}

/**
 * @brief   STM32WLxx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {

#if !STM32_NO_INIT

  /* Reset of all peripherals.
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB1(~0UL);
  rccResetAHB2(~STM32_GPIO_EN_MASK);
  /* Reset all except FLASH.*/
  rccResetAHB3(RCC_AHB3RSTR_PKARST | RCC_AHB3RSTR_AESRST |
               RCC_AHB3RSTR_RNGRST | RCC_AHB3RSTR_HSEMRST);
  rccResetAPB1R1(~0UL);
  rccResetAPB1R2(~0UL);
  rccResetAPB2(~0UL);

  /* RTC clock enable.*/
#if HAL_USE_RTC
  rccEnableAPB1R1(RCC_APB1ENR1_RTCAPBEN, true);
#endif

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT)

  /* Static PWR initializations.*/
  hal_lld_set_static_pwr();

  /* Backup domain made accessible.*/
  PWR->CR1 |= PWR_CR1_DBP;

  /* Backup domain reset.*/
  bd_reset();

  /* Static clocks setup.*/
  lse_init();
  lsi_init();

  /* MSISRANGE setup.*/
  RCC->CR |= RCC_CR_MSIRGSEL;
  RCC->CSR = (RCC->CSR & ~RCC_CSR_MSISRANGE_Msk) | STM32_MSISRANGE;

  /* Static clocks setup.*/
  hal_lld_set_static_clocks();

  /* Selecting the default clock/power/flash configuration.*/
  if (hal_lld_clock_raw_config(&hal_clkcfg_default)) {
    osalSysHalt("clkswc");
  }

  /* Backup domain initializations.*/
  bd_init();
#else /* !defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */
  /* Flash setup for selected MSI speed setting.*/
  flash_set_acr(FLASH_ACR_DCEN   | FLASH_ACR_ICEN |
                FLASH_ACR_PRFTEN |STM32_FLASHBITS);

  /* Static PWR initializations.*/
  hal_lld_set_static_pwr();

  /* Core voltage setup, backup domain access enabled and left open.*/
  PWR->CR1 = STM32_VOS | PWR_CR1_DBP;
  while ((PWR->SR2 & PWR_SR2_VOSF) != 0) {
    /* Wait until regulator is stable.*/
  }

  /* MSI clock reset.*/
  msi_reset();

  /* Backup domain reset.*/
  bd_reset();

  /* Clocks setup.*/
  lse_init();
  lsi_init();
  msi_init();
  hsi16_init();
  hse32_init();

  /* Backup domain initializations.*/
  bd_init();

  /* PLLs activation, if required.*/
  pll_init();

  /* Static clocks setup.*/
  hal_lld_set_static_clocks();

  /* Set flash WS's according HCLK3.*/
  flash_set_acr((FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | STM32_FLASHBITS);

  /* Switching to the configured SYSCLK source if it is different from MSI.*/
#if (STM32_SW != STM32_SW_MSI)
  RCC->CFGR |= STM32_SW;        /* Switches on the selected clock source.   */
  /* Wait until SYSCLK is stable.*/
  while ((RCC->CFGR & RCC_CFGR_SWS) != (STM32_SW << 2))
    ;
#endif
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

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

  if (hal_lld_clock_check_tree(ccp)) {
    return true;
  }

  if (hal_lld_clock_raw_config(ccp)) {
    return true;
  }

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
