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
 * @file    STM32U3xx/hal_lld.c
 * @brief   STM32U3xx HAL subsystem low level driver source.
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
 * @name    Registers reset values
 * @{
 */
#define STM32_PWR_VOSR_RESET            0x00020002U
#define STM32_FLASH_ACR_RESET           0x00000001U
#define STM32_RCC_CR_RESET              0x0000001DU
#define STM32_RCC_ICSCR1_RESET          0xB4000000U
#define STM32_RCC_CFGR1_RESET           0U
#define STM32_RCC_CFGR2_RESET           0U
#define STM32_RCC_CFGR3_RESET           0U
#define STM32_RCC_CFGR4_RESET           0U
/** @} */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_stm32u3xx.h.
 */
uint32_t SystemCoreClock;

/**
 * @brief   Post-reset clock configuration.
 */
const halclkcfg_t hal_clkcfg_reset = {
  .pwr_vosr             = STM32_PWR_VOSR_RESET,
  .rcc_cr               = STM32_RCC_CR_RESET,
  .rcc_icscr1           = STM32_RCC_ICSCR1_RESET,
  .rcc_cfgr1            = STM32_RCC_CFGR1_RESET,
  .rcc_cfgr2            = STM32_RCC_CFGR2_RESET,
  .rcc_cfgr3            = STM32_RCC_CFGR3_RESET,
  .rcc_cfgr4            = STM32_RCC_CFGR4_RESET,
  .flash_acr            = STM32_FLASH_ACR_RESET
};

/**
 * @brief   Default clock configuration.
 * @note    This is the configuration defined in mcuconf.h.
 */
const halclkcfg_t hal_clkcfg_default = {
  .pwr_vosr             = STM32_PWR_VOSR
#if STM32_BOOSTER_ENABLED == TRUE
                        | PWR_VOSR_BOOSTEN
#endif
                          ,
  .rcc_cr               = STM32_MSIPLL0FAST | STM32_MSIPLL1FAST
                        | STM32_MSIPLL0EN   | STM32_MSIPLL1EN
#if STM32_HSE_ENABLED == TRUE
                        | RCC_CR_HSEON
#endif
#if defined(STM32_HSE_BYPASS)
                        | RCC_CR_HSEBYP
#endif
#if STM32_HSI48_ENABLED == TRUE
                        | RCC_CR_HSI48ON
#endif
#if STM32_HSI16_ENABLED == TRUE
                        | RCC_CR_HSION | RCC_CR_HSIKERON
#endif
#if STM32_ACTIVATE_MSIS == TRUE
                        | RCC_CR_MSISON
#endif
#if STM32_ACTIVATE_MSIK == TRUE
                        | RCC_CR_MSIKON | RCC_CR_MSIKERON
#endif
                          ,
  .rcc_icscr1           = STM32_MSISSEL     | STM32_MSISDIV     |
                          STM32_MSIKSEL     | STM32_MSIKDIV     |
                          STM32_MSIPLL1N    | STM32_MSIBIAS     |
                          STM32_MSIPLL0SEL  | STM32_MSIPLL1SEL  |
                          STM32_MSIHSINDIV  |
                          RCC_ICSCR1_MSIRGSEL_ICSCR1, /* Note, enforced.*/
  .rcc_cfgr1            = STM32_MCO2PRE     | STM32_MCO2SEL     |
                          STM32_MCO1PRE     | STM32_MCO1SEL     |
                          STM32_STOPKERWUCK | STM32_STOPWUCK,
  .rcc_cfgr2            = STM32_PPRE2       | STM32_PPRE1       |
                          STM32_HPRE,
  .rcc_cfgr3            = STM32_PPRE3,
  .rcc_cfgr4            = 0U
#if STM32_BOOSTER_ENABLED == TRUE
                        | STM32_BOOSTDIV    | STM32_BOOSTSEL
#endif
                          ,
  .flash_acr            = (STM32_FLASH_ACR & ~FLASH_ACR_LATENCY_Msk) |
                          STM32_FLASHBITS,
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
#if STM32_HSI16_ENABLED
  [CLK_HSI16]           = STM32_HSI16CLK,
#else
  [CLK_HSI16]           = 0U,
#endif
#if STM32_HSI48_ENABLED
  [CLK_HSI48]           = STM32_HSI48CLK,
#else
  [CLK_HSI48]           = 0U,
#endif
#if STM32_HSE_ENABLED
  [CLK_HSE]             = STM32_HSECLK,
#else
  [CLK_HSE]             = 0U,
#endif
#if STM32_ACTIVATE_MSIS
  [CLK_MSIS]            = STM32_MSISCLK,
#else
  [CLK_MSIS]            = 0U,
#endif
#if STM32_ACTIVATE_MSIK
  [CLK_MSIK]            = STM32_MSIKCLK,
#else
  [CLK_MSIK]            = 0U,
#endif
  [CLK_SYSCLK]          = STM32_SYSCLK,
  [CLK_HCLK]            = STM32_HCLK,
  [CLK_PCLK1]           = STM32_PCLK1,
  [CLK_PCLK1TIM]        = STM32_TIMP1CLK,
  [CLK_PCLK2]           = STM32_PCLK2,
  [CLK_PCLK2TIM]        = STM32_TIMP2CLK,
  [CLK_PCLK3]           = STM32_PCLK3,
  [CLK_MCO1]            = STM32_MCO1CLK,
  [CLK_MCO2]            = STM32_MCO2CLK
};

/**
 * @brief   Type of a structure representing system limits.
 */
typedef struct {
  halfreq_t             sysclk_max;
  halfreq_t             flash_thresholds[STM32_WS_THRESHOLDS];
} system_limits_t;

/**
 * @brief   System limits for range 1.
 */
static const system_limits_t vos_range1 = {
  .sysclk_max           = STM32_RANGE1_SYSCLK_MAX,
  .flash_thresholds     = {STM32_RANGE1_0WS_THRESHOLD, STM32_RANGE1_1WS_THRESHOLD,
                           STM32_RANGE1_2WS_THRESHOLD, STM32_RANGE1_3WS_THRESHOLD,
                           STM32_RANGE1_4WS_THRESHOLD}
};

/**
 * @brief   System limits for range 2.
 */
static const system_limits_t vos_range2 = {
  .sysclk_max           = STM32_RANGE2_SYSCLK_MAX,
  .flash_thresholds     = {STM32_RANGE2_0WS_THRESHOLD, STM32_RANGE2_1WS_THRESHOLD,
                           STM32_RANGE2_2WS_THRESHOLD, STM32_RANGE2_3WS_THRESHOLD,
                           STM32_RANGE2_4WS_THRESHOLD}
};
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#include "stm32_bd.inc"

__STATIC_INLINE void hal_lld_set_coreclock(halfreq_t coreclock) {

  SystemCoreClock = (uint32_t)coreclock;
}

/**
 * @brief   Configures the PWR unit.
 * @note    CR1, CR2, VOSR are not initialized inside this function.
 */
__STATIC_INLINE void hal_lld_set_static_pwr(void) {

  /* PWR clock enabled.*/
  rccEnablePWRInterface(false);

  /* Enable write access to Backup domain.*/
  halRegWrite32X(&PWR->DBPR, PWR_DBPR_DBP, true);
  halRegWrite32X(&PWR->BDCR, STM32_PWR_BDCR, true);

  /* Static settings for PWR registers.*/
  halRegWrite32X(&PWR->CR3,      STM32_PWR_CR3,      true);
  halRegWrite32X(&PWR->SVMCR,    STM32_PWR_SVMCR,    true);
  halRegWrite32X(&PWR->WUCR1,    STM32_PWR_WUCR1,    true);
  halRegWrite32X(&PWR->WUCR2,    STM32_PWR_WUCR2,    true);
  halRegWrite32X(&PWR->WUCR3,    STM32_PWR_WUCR3,    true);
  halRegWrite32X(&PWR->PUCRA,    STM32_PWR_PUCRA,    true);
  halRegWrite32X(&PWR->PDCRA,    STM32_PWR_PDCRA,    true);
  halRegWrite32X(&PWR->PDCRB,    STM32_PWR_PDCRB,    true);
  halRegWrite32X(&PWR->PUCRB,    STM32_PWR_PUCRB,    true);
  halRegWrite32X(&PWR->PUCRC,    STM32_PWR_PUCRC,    true);
  halRegWrite32X(&PWR->PUCRD,    STM32_PWR_PUCRD,    true);
  halRegWrite32X(&PWR->PUCRE,    STM32_PWR_PUCRE,    true);
  halRegWrite32X(&PWR->PDCRC,    STM32_PWR_PDCRC,    true);
  halRegWrite32X(&PWR->PDCRD,    STM32_PWR_PDCRD,    true);
  halRegWrite32X(&PWR->PDCRE,    STM32_PWR_PDCRE,    true);
  halRegWrite32X(&PWR->PUCRG,    STM32_PWR_PUCRG,    true);
  halRegWrite32X(&PWR->PDCRG,    STM32_PWR_PDCRG,    true);
  halRegWrite32X(&PWR->PUCRH,    STM32_PWR_PUCRH,    true);
  halRegWrite32X(&PWR->PDCRH,    STM32_PWR_PDCRH,    true);
  halRegWrite32X(&PWR->APCR,     STM32_PWR_APCR,     true);
  halRegWrite32X(&PWR->I3CPUCR1, STM32_PWR_I3CPUCR1, true);
  halRegWrite32X(&PWR->I3CPUCR2, STM32_PWR_I3CPUCR2, true);

  /* Security/privilege settings for PWR.*/
  halRegWrite32X(&PWR->SECCFGR,  STM32_PWR_SECCFGR,  true);
  halRegWrite32X(&PWR->PRIVCFGR, STM32_PWR_PRIVCFGR, true);
}

/**
 * @brief   Initializes static muxes and dividers.
 */
__STATIC_INLINE void hal_lld_set_static_clocks(void) {

  /* CCIPR registers initialization, note.*/
  halRegWrite32X(&RCC->CCIPR1,
                 STM32_TIMICSEL    | STM32_USB1SEL     |
                 STM32_ICLKSEL     |
#if STM32_FDCAN1SEL != RCC_CCIPR1_FDCAN1SEL_IGNORE
                 STM32_FDCAN1SEL   |
#endif
                 STM32_SYSTICKSEL  | STM32_SPI1SEL     |
                 STM32_LPTIM2SEL   | STM32_SPI2SEL     |
                 STM32_I3C2SEL     | STM32_I2C2SEL     |
                 STM32_I2C1SEL     | STM32_I3C1SEL     |
                 STM32_UART5SEL    | STM32_UART4SEL    |
                 STM32_USART3SEL   | STM32_USART1SEL,
                 true);
  halRegWrite32X(&RCC->CCIPR2,
                 STM32_OCTOSPISEL  |
#if STM32_DAC1SHSEL != RCC_CCIPR2_DAC1SHSEL_IGNORE
                 STM32_DAC1SHSEL   |
#endif
                 STM32_ADCDACSEL   | STM32_ADCDACPRE   |
#if STM32_RNGSEL != RCC_CCIPR2_RNGSEL_IGNORE
                 STM32_RNGSEL      |
#endif
                 STM32_SAI1SEL     | STM32_SPI3SEL     |
                 STM32_ADF1SEL,
                 true);
  halRegWrite32X(&RCC->CCIPR3,
                 STM32_LPTIM1SEL   | STM32_LPTIM34SEL  |
                 STM32_I2C3SEL     | STM32_LPUART1SEL,
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
  halRegWrite32X(&FLASH->ACR, FLASH_ACR_LATENCY_4WS, true);

  /* MSIS must be active before performing the reconfiguration, MSI could
     be restarting so waiting for the ready bits is required.*/
  halRegWrite32X(&RCC->ICSCR1, STM32_RCC_ICSCR1_RESET, true);
  halRegWrite32X(&RCC->CR,     STM32_RCC_CR_RESET, true);
  if (halRegWaitAllSet32X(&RCC->CR,
                          RCC_CR_MSISRDY | RCC_CR_MSIKRDY,
                          STM32_OSCILLATORS_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* Resetting clock-related settings.*/
  halRegWrite32X(&RCC->CFGR1, STM32_RCC_CFGR1_RESET, true);
  if (halRegWaitMatch32X(&RCC->CFGR1,
                         RCC_CFGR1_SWS_Msk, RCC_CFGR1_SWS_MSIS,
                         STM32_SYSCLK_SWITCH_TIME,
                         NULL)) {
    return true;
  }
  halRegWrite32X(&RCC->CFGR2, STM32_RCC_CFGR2_RESET, true);
  halRegWrite32X(&RCC->CFGR3, STM32_RCC_CFGR3_RESET, true);
  halRegWrite32X(&RCC->CFGR4, STM32_RCC_CFGR4_RESET, true);
  halRegWrite32X(&PWR->VOSR,  STM32_PWR_VOSR_RESET, true);

  /* MSIRC1/4 as post-reset clock.*/
  hal_lld_set_coreclock(6000000U);

  /* Enabling all required oscillators at same time, MSIS enforced active,
     PLLs not enabled yet because we are running in "reset" mode.*/
  halRegWrite32X(&RCC->CR,
                 (ccp->rcc_cr | RCC_CR_MSISON) & ~(RCC_CR_MSIPLL0EN | RCC_CR_MSIPLL1EN),
                 true);

  /* Adding to the "wait mask" the status bits of all enabled oscillators.*/
  wtmask = RCC_CR_MSISRDY | RCC_CR_MSIKRDY;     /* Known to be ready already.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    wtmask |= RCC_CR_HSERDY;
  }
  if ((ccp->rcc_cr & RCC_CR_HSI48ON) != 0U) {
    wtmask |= RCC_CR_HSI48RDY;
  }
  if ((ccp->rcc_cr & RCC_CR_HSION) != 0U) {
    wtmask |= RCC_CR_HSIRDY;
  }
  if (halRegWaitAllSet32X(&RCC->CR,
                          wtmask,
                          STM32_OSCILLATORS_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* Programmable voltage scaling configuration, this is done at the end
     because the booster clock must be ready (see RCC_CFGR4) before enabling
     the booster. */
  halRegWrite32X(&RCC->CFGR4, ccp->rcc_cfgr4, true);
  halRegWrite32X(&PWR->VOSR,  ccp->pwr_vosr,  true);
  wtmask = ccp->pwr_vosr << 16;
  if (halRegWaitAllSet32X(&PWR->VOSR,
                          wtmask,
                          STM32_OSCILLATORS_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* MSI configuration (sources, dividers, bias). */
  halRegWrite32X(&RCC->ICSCR1, ccp->rcc_icscr1 | RCC_ICSCR1_MSIRGSEL_ICSCR1, true);

  /* Enabling also PLLs if required by the configuration.*/
  halRegWrite32X(&RCC->CR, ccp->rcc_cr | RCC_CR_MSISON, true);
  wtmask = 0U;
  if ((ccp->rcc_cr & RCC_CR_MSIPLL0EN) != 0U) {
    wtmask |= RCC_CR_MSIPLL0RDY;
  }
  if ((ccp->rcc_cr & RCC_CR_MSIPLL1EN) != 0U) {
    wtmask |= RCC_CR_MSIPLL1RDY;
  }
  if (halRegWaitAllSet32X(&RCC->CR,
                          wtmask,
                          STM32_MSIPLL_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* Final RCC CFGR settings (prescalers, MCO, STOP wake-up sources, booster).*/
  halRegWrite32X(&RCC->CFGR1, ccp->rcc_cfgr1, true);
  halRegWrite32X(&RCC->CFGR2, ccp->rcc_cfgr2, true);
  halRegWrite32X(&RCC->CFGR3, ccp->rcc_cfgr3, true);

  /* Final flash ACR settings according to the target configuration.*/
  halRegWrite32X(&FLASH->ACR, ccp->flash_acr, true);

  /* Waiting for the requested SYSCLK source to become active. */
  if (halRegWaitMatch32X(&RCC->CFGR1,
                         RCC_CFGR1_SWS_Msk, (ccp->rcc_cfgr1 & RCC_CFGR1_SW_Msk) << RCC_CFGR1_SWS_Pos,
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
  static const uint32_t hprediv[16] = {1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U,
                                       2U, 4U, 8U, 16U, 64U, 128U, 256U, 512U};
  static const uint32_t pprediv[16] = {1U, 1U, 1U, 1U, 2U, 4U, 8U, 16U};
  const system_limits_t *slp;
  uint32_t n, flashws;
  halfreq_t hsi16clk = 0U, hsi48clk = 0U, hseclk = 0U;
  halfreq_t msirc0clk = 0U, msirc1clk = 0U, msisclk = 0U, msikclk = 0U;
  halfreq_t sysclk, hclk, pclk1, pclk2, pclk3, pclk1tim, pclk2tim, mco1clk, mco2clk;

  /* System limits based on desired VOS settings.*/
  switch (ccp->pwr_vosr & PWR_VOSR_RANGE_Msk) {
  case PWR_VOSR_RANGE1:
    slp = &vos_range1;
    break;
  case PWR_VOSR_RANGE2:
    slp = &vos_range2;
    break;
  default:
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
  if ((ccp->rcc_cr & RCC_CR_HSI48ON) != 0U) {
    hsi48clk = STM32_HSI48CLK;
  }

  /* MSIRC0 base clock depending on MSIPLL0 mode. */
  if ((ccp->rcc_cr & RCC_CR_MSIPLL0EN) == 0U) {
    msirc0clk = 96000000U;
  }
  else {
    uint32_t sel0 = (ccp->rcc_icscr1 & RCC_ICSCR1_MSIPLL0SEL_Msk) >> RCC_ICSCR1_MSIPLL0SEL_Pos;

    switch (sel0) {
    case 0U: /* LSE reference */
      msirc0clk = 96010000U;
      break;
    case 1U: /* HSE reference */
      msirc0clk = 96000000U;
      break;
    default:
      return true;
    }
  }

  /* MSIRC1 base clock depending on MSIPLL1 mode. */
  if ((ccp->rcc_cr & RCC_CR_MSIPLL1EN) == 0U) {
    msirc1clk = 24000000U;
  }
  else {
    uint32_t sel1 = (ccp->rcc_icscr1 & RCC_ICSCR1_MSIPLL1SEL_Msk) >> RCC_ICSCR1_MSIPLL1SEL_Pos;
    uint32_t n1   = (ccp->rcc_icscr1 & RCC_ICSCR1_MSIPLL1N_Msk)   >> RCC_ICSCR1_MSIPLL1N_Pos;

    switch (n1) {
    case 0U:
      if (sel1 == 0U) {
        msirc1clk = 23986000U;
      }
      else if (sel1 == 1U) {
        msirc1clk = 24016000U;
      }
      else {
        return true;
      }
      break;
    case 2U:
      if (sel1 == 0U) {
        msirc1clk = 22577000U;
      }
      else if (sel1 == 1U) {
        msirc1clk = 22581000U;
      }
      else {
        return true;
      }
      break;
    case 3U:
      if (sel1 == 0U) {
        msirc1clk = 24576000U;
      }
      else if (sel1 == 1U) {
        msirc1clk = 24577000U;
      }
      else {
        return true;
      }
      break;
    default:
      return true;
    }
  }

  /* MSIS clock.*/
  if ((ccp->rcc_cr & RCC_CR_MSISON) != 0U) {
    halfreq_t msis_src;
    uint32_t msis_div;

    if ((ccp->rcc_icscr1 & RCC_ICSCR1_MSISSEL_Msk) == RCC_ICSCR1_MSISSEL_MSIRC0) {
      msis_src = msirc0clk;
    }
    else {
      msis_src = msirc1clk;
    }

    msis_div = (ccp->rcc_icscr1 & RCC_ICSCR1_MSISDIV_Msk) >> RCC_ICSCR1_MSISDIV_Pos;
    msisclk = msis_src >> msis_div;
  }

  /* MSIK clock.*/
  if ((ccp->rcc_cr & RCC_CR_MSIKON) != 0U) {
    halfreq_t msik_src;
    uint32_t msik_div;

    if ((ccp->rcc_icscr1 & RCC_ICSCR1_MSIKSEL_Msk) == RCC_ICSCR1_MSIKSEL_MSIRC0) {
      msik_src = msirc0clk;
    }
    else {
      msik_src = msirc1clk;
    }

    msik_div = (ccp->rcc_icscr1 & RCC_ICSCR1_MSIKDIV_Msk) >> RCC_ICSCR1_MSIKDIV_Pos;
    msikclk = msik_src >> msik_div;
  }

  /* SYSCLK frequency.*/
  switch (ccp->rcc_cfgr1 & RCC_CFGR1_SW_Msk) {
  case RCC_CFGR1_SW_MSIS:
    sysclk = msisclk;
    break;
  case RCC_CFGR1_SW_HSI16:
    sysclk = hsi16clk;
    break;
  case RCC_CFGR1_SW_HSE:
    sysclk = hseclk;
    break;
  default:
    sysclk = 0U;
  }

  /* Checking SYSCLK limit.*/
  if ((sysclk == 0U) || (sysclk > slp->sysclk_max)) {
    return true;
  }

  /* HCLK frequency.*/
  hclk = sysclk / hprediv[(ccp->rcc_cfgr2 & STM32_HPRE_MASK) >> STM32_HPRE_POS];

  /* PPRE1 frequency.*/
  n = pprediv[(ccp->rcc_cfgr2 & STM32_PPRE1_MASK) >> STM32_PPRE1_POS];
  pclk1 = hclk / n;
  if (n < 2) {
    pclk1tim = pclk1;
  }
  else {
    pclk1tim = pclk1 * 2U;
  }

  /* PPRE2 frequency.*/
  n = pprediv[(ccp->rcc_cfgr2 & STM32_PPRE2_MASK) >> STM32_PPRE2_POS];
  pclk2 = hclk / n;
  if (n < 2) {
    pclk2tim = pclk2;
  }
  else {
    pclk2tim = pclk2 * 2U;
  }

  /* PPRE3 frequency.*/
  n = pprediv[(ccp->rcc_cfgr3 & STM32_PPRE3_MASK) >> STM32_PPRE3_POS];
  pclk3 = hclk / n;

  /* MCO1 clock.*/
  switch (ccp->rcc_cfgr1 & RCC_CFGR1_MCOSEL_Msk) {
  case RCC_CFGR1_MCO1SEL_SYSCLK:
    mco1clk = sysclk;
    break;
  case RCC_CFGR1_MCO1SEL_MSIS:
    mco1clk = msisclk;
    break;
  case RCC_CFGR1_MCO1SEL_HSI16:
    mco1clk = hsi16clk;
    break;
  case RCC_CFGR1_MCO1SEL_HSE:
    mco1clk = hseclk;
    break;
  case RCC_CFGR1_MCO1SEL_LSI:
    mco1clk = STM32_LSICLK;
    break;
  case RCC_CFGR1_MCO1SEL_LSE:
    mco1clk = STM32_LSECLK;
    break;
  case RCC_CFGR1_MCO1SEL_HSI48:
    mco1clk = STM32_HSI48CLK;
    break;
  case RCC_CFGR1_MCO1SEL_MSIK:
    mco1clk = msikclk;
    break;
  default:
    mco1clk = 0U;
  }
  n = (ccp->rcc_cfgr1 & RCC_CFGR1_MCOPRE_Msk) >> RCC_CFGR1_MCOPRE_Pos;
  mco1clk /= 1U << n;

  /* MCO2 clock.*/
  switch (ccp->rcc_cfgr1 & RCC_CFGR1_MCO2SEL_Msk) {
  case RCC_CFGR1_MCO2SEL_SYSCLK:
    mco2clk = sysclk;
    break;
  case RCC_CFGR1_MCO2SEL_MSIS:
    mco2clk = msisclk;
    break;
  case RCC_CFGR1_MCO2SEL_HSI16:
    mco2clk = hsi16clk;
    break;
  case RCC_CFGR1_MCO2SEL_HSE:
    mco2clk = hseclk;
    break;
  case RCC_CFGR1_MCO2SEL_LSI:
    mco2clk = STM32_LSICLK;
    break;
  case RCC_CFGR1_MCO2SEL_LSE:
    mco2clk = STM32_LSECLK;
    break;
  case RCC_CFGR1_MCO2SEL_HSI48:
    mco2clk = STM32_HSI48CLK;
    break;
  case RCC_CFGR1_MCO2SEL_MSIK:
    mco2clk = msikclk;
    break;
  default:
    mco2clk = 0U;
  }
  n = (ccp->rcc_cfgr1 & RCC_CFGR1_MCO2PRE_Msk) >> RCC_CFGR1_MCO2PRE_Pos;
  mco2clk /= 1U << n;

  /* Flash settings.*/
  flashws = ((ccp->flash_acr & FLASH_ACR_LATENCY_Msk) >> FLASH_ACR_LATENCY_Pos);
  if (flashws >= STM32_WS_THRESHOLDS) {
    return true;
  }
  if (hclk > slp->flash_thresholds[flashws]) {
    return true;
  }

  /* Writing out results.*/
  clock_points[CLK_HSI16]    = hsi16clk;
  clock_points[CLK_HSI48]    = hsi48clk;
  clock_points[CLK_HSE]      = hseclk;
  clock_points[CLK_MSIS]     = msisclk;
  clock_points[CLK_MSIK]     = msikclk;
  clock_points[CLK_SYSCLK]   = sysclk;
  clock_points[CLK_HCLK]     = hclk;
  clock_points[CLK_PCLK1]    = pclk1;
  clock_points[CLK_PCLK1TIM] = pclk1tim;
  clock_points[CLK_PCLK2]    = pclk2;
  clock_points[CLK_PCLK2TIM] = pclk2tim;
  clock_points[CLK_PCLK3]    = pclk3;
  clock_points[CLK_MCO1]     = mco1clk;
  clock_points[CLK_MCO2]     = mco2clk;

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

  /* Frequency after applying the default configuration or ->assumed<- set
     by the bootloader in case of NO_INIT.*/
  hal_lld_set_coreclock(STM32_HCLK);

  /* DMA subsystems initialization.*/
#if defined(STM32_DMA3_REQUIRED)
  dma3Init();
#endif

  /* NVIC initialization.*/
  nvicInit();

  /* IRQ subsystem initialization.*/
  irqInit();
}

/**
 * @brief   Clocks initialization.
 * @note    This function is invoked early by the startup files, non-automatic
 *          variables are not initialized.
 *
 * @special
 */
void stm32_clock_init(void) {

  /* DWT cycles counter enabled, used for timeouts.*/
  halRegSet32X(&DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk, true);

#if !STM32_NO_INIT
  /* Assuming MSIRC1/4 as post-reset clock.*/
  hal_lld_set_coreclock(6000000U);

  /* Reset of all peripherals.
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB1R1(~0);
  rccResetAHB1R2(~0);
  rccResetAHB2R1(~STM32_GPIO_EN_MASK);
  rccResetAHB2R2(~0);
  rccResetAPB1R1(~0);
  rccResetAPB1R2(~0);
  rccResetAPB2(~0);
  rccResetAPB3(~0);

  /* RTC APB clock enable.*/
#if (HAL_USE_RTC == TRUE) && defined(RCC_APB3ENR_RTCAPBEN)
  rccEnableAPB3(RCC_APB3ENR_RTCAPBEN, true);
#endif

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

  /* Cache enable.*/
  icache_init();
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

  if (hal_lld_clock_configure(ccp)) {
    return true;
  }

  /* Updating the current system clock setting value.*/
  hal_lld_set_coreclock(hal_lld_get_clock_point(CLK_HCLK));

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
