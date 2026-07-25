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
 * @file    STM32G4xx_ALT/hal_lld.c
 * @brief   STM32G4xx ALT HAL subsystem low level driver source.
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
#define STM32_FLASH_ACR_RESET           (FLASH_ACR_DBG_SWEN |               \
                                         FLASH_ACR_DCEN     |               \
                                         FLASH_ACR_ICEN     |               \
                                         FLASH_ACR_LATENCY_0WS)
#define STM32_PWR_CR1_RESET             (PWR_CR1_VOS_0)
#define STM32_RCC_CR_RESET              (RCC_CR_HSION)
#define STM32_RCC_CFGR_RESET            0x00000005U
#define STM32_RCC_CRRCR_RESET           0x00000000U
/** @} */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    Must be kept updated during clock initializtions because timeouts
 *          calculation depends on this variable.
 * @note    It is declared in system_stm32g4xx.h.
 */
uint32_t SystemCoreClock;

/**
 * @brief   Post-reset clock configuration.
 */
const halclkcfg_t hal_clkcfg_reset = {
  .pwr_cr1              = PWR_CR1_VOS_0,
  .pwr_cr2              = 0U,
  .pwr_cr5              = PWR_CR5_R1MODE,
  .rcc_cr               = RCC_CR_HSIKERON | RCC_CR_HSION,
  .rcc_cfgr             = STM32_RCC_CFGR_RESET,
  .rcc_pllcfgr          = 0U,
  .rcc_crrcr            = 0U,
  .flash_acr            = STM32_FLASH_ACR_RESET
};

/**
 * @brief   Default clock configuration.
 */
const halclkcfg_t hal_clkcfg_default = {
  .pwr_cr1              = STM32_VOS | PWR_CR1_DBP,
  .pwr_cr2              = STM32_PWR_CR2,
  .pwr_cr5              = STM32_CR5BITS,
  .rcc_cr               = 0U
#if STM32_HSI16_ENABLED
                        | RCC_CR_HSIKERON | RCC_CR_HSION
#endif
#if STM32_HSE_ENABLED
                        | RCC_CR_HSEON
#if defined(STM32_HSE_BYPASS)
                        | RCC_CR_HSEBYP
#endif
#endif
#if STM32_ACTIVATE_PLL
                        | RCC_CR_PLLON
#endif
                        ,
  .rcc_cfgr             = STM32_MCOPRE  | STM32_MCOSEL |
                          STM32_PPRE2   | STM32_PPRE1  |
                          STM32_HPRE    | STM32_SW,
  .rcc_pllcfgr          = STM32_PLLPDIV | STM32_PLLR   |
                          STM32_PLLREN  | STM32_PLLQ   |
                          STM32_PLLQEN  | STM32_PLLP   |
                          STM32_PLLPEN  | STM32_PLLN   |
                          STM32_PLLM    | STM32_PLLSRC,
#if STM32_HSI48_ENABLED
  .rcc_crrcr            = RCC_CRRCR_HSI48ON,
#else
  .rcc_crrcr            = 0U,
#endif
  .flash_acr            = FLASH_ACR_DBG_SWEN | FLASH_ACR_DCEN   |
                          FLASH_ACR_ICEN     | FLASH_ACR_PRFTEN |
                          STM32_FLASHBITS
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if defined(HAL_LLD_USE_CLOCK_MANAGEMENT) || defined(__DOXYGEN__)
/**
 * @brief   Dynamic clock points for this device.
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
  [CLK_SYSCLK]          = STM32_SYSCLK,
  [CLK_PLLPCLK]         = STM32_PLL_P_CLKOUT,
  [CLK_PLLQCLK]         = STM32_PLL_Q_CLKOUT,
  [CLK_PLLRCLK]         = STM32_PLL_R_CLKOUT,
  [CLK_HCLK]            = STM32_HCLK,
  [CLK_PCLK1]           = STM32_PCLK1,
  [CLK_PCLK1TIM]        = STM32_TIMP1CLK,
  [CLK_PCLK2]           = STM32_PCLK2,
  [CLK_PCLK2TIM]        = STM32_TIMP2CLK,
  [CLK_MCO]             = STM32_MCOCLK,
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
 * @brief   System limits for VOS range 1 with boost.
 */
static const system_limits_t vos_range1_boost = {
  .sysclk_max           = STM32_BOOST_SYSCLK_MAX,
  .pllin_max            = STM32_BOOST_PLLIN_MAX,
  .pllin_min            = STM32_BOOST_PLLIN_MIN,
  .pllvco_max           = STM32_BOOST_PLLVCO_MAX,
  .pllvco_min           = STM32_BOOST_PLLVCO_MIN,
  .pllp_max             = STM32_BOOST_PLLP_MAX,
  .pllp_min             = STM32_BOOST_PLLP_MIN,
  .pllq_max             = STM32_BOOST_PLLQ_MAX,
  .pllq_min             = STM32_BOOST_PLLQ_MIN,
  .pllr_max             = STM32_BOOST_PLLR_MAX,
  .pllr_min             = STM32_BOOST_PLLR_MIN,
  .flash_thresholds     = {STM32_BOOST_0WS_THRESHOLD, STM32_BOOST_1WS_THRESHOLD,
                           STM32_BOOST_2WS_THRESHOLD, STM32_BOOST_3WS_THRESHOLD,
                           STM32_BOOST_4WS_THRESHOLD}
};

/**
 * @brief   System limits for VOS range 1 without boost.
 */
static const system_limits_t vos_range1_noboost = {
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

__STATIC_INLINE void hal_lld_set_coreclock(halfreq_t coreclock) {

  SystemCoreClock = (uint32_t)coreclock;
}

/**
 * @brief   Configures the PWR unit.
 * @note    CR1, CR2 and CR5 are not initialized inside this function.
 */
__STATIC_INLINE void hal_lld_set_static_pwr(void) {

  /* PWR clock enabled.*/
  rccEnablePWRInterface(false);

  /* Static PWR configurations.*/
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
  RCC->CFGR   = STM32_MCOPRE | STM32_MCOSEL | STM32_PPRE2 | STM32_PPRE1 |
                STM32_HPRE;

  /* CCIPR registers initialization, note.*/
  RCC->CCIPR  = STM32_ADC345SEL  | STM32_ADC12SEL   | STM32_CLK48SEL   |
                STM32_FDCANSEL   | STM32_I2S23SEL   | STM32_SAI1SEL    |
                STM32_LPTIM1SEL  | STM32_I2C3SEL    | STM32_I2C2SEL    |
                STM32_I2C1SEL    | STM32_LPUART1SEL | STM32_UART5SEL   |
                STM32_UART4SEL   | STM32_USART3SEL  | STM32_USART2SEL  |
                STM32_USART1SEL;
  RCC->CCIPR2 = STM32_QSPISEL    | STM32_I2C4SEL;
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
  uint32_t cr, wtmask;

  /* Restoring default PWR settings related clocks and sleep modes.*/
  halRegWrite32X(&PWR->CR1, STM32_PWR_CR1_RESET, true);
  if (halRegWaitAllClear32X(&PWR->SR2,
                            PWR_SR2_VOSF | PWR_SR2_REGLPF,
                            STM32_REGULATORS_TRANSITION_TIME,
                            NULL)) {
    return true;
  }

  /* Making sure HSI16 is activated and in use.*/
  halRegSet32X(&RCC->CR, RCC_CR_HSION | RCC_CR_HSIKERON, true);
  if (halRegWaitAllSet32X(&RCC->CR, RCC_CR_HSIRDY,
                          STM32_HSI_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* Switching to HSI16.*/
  halRegMaskedWrite32X(&RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI, true);
  if (halRegWaitMatch32X(&RCC->CFGR,
                         RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_HSI,
                         STM32_SYSCLK_SWITCH_TIME,
                         NULL)) {
    return true;
  }

  /* Updating the current system clock setting value.*/
  hal_lld_set_coreclock(STM32_HSI16CLK);

  /* Finally resetting FLASH_ACR, CR, CFGR and CRRCR.*/
  halRegWrite32X(&RCC->CR, STM32_RCC_CR_RESET, true);
  halRegWrite32X(&RCC->CFGR, STM32_RCC_CFGR_RESET, true);
  halRegWrite32X(&RCC->CRRCR, STM32_RCC_CRRCR_RESET, true);
  halRegWrite32X(&FLASH->ACR, STM32_FLASH_ACR_RESET, true);

  /* Waiting for HSE and PLL to stop.*/
  if (halRegWaitAllClear32X(&RCC->CR, RCC_CR_HSERDY | RCC_CR_PLLRDY,
                            STM32_OSCILLATORS_STARTUP_TIME,
                            NULL)) {
    return true;
  }

  /* Waiting for HSI48 to stop.*/
  if (halRegWaitAllClear32X(&RCC->CRRCR, RCC_CRRCR_HSI48RDY,
                            STM32_OSCILLATORS_STARTUP_TIME,
                            NULL)) {
    return true;
  }

  /* Enabling all required oscillators, keeping HSI16 enabled and
     PLL disabled.*/
  cr  = ccp->rcc_cr;
  cr |= RCC_CR_HSION;
  cr &= ~RCC_CR_PLLON;
  halRegWrite32X(&RCC->CR, cr, true);

  /* Adding to the "wait mask" the status bits of enabled oscillators.*/
  wtmask = RCC_CR_HSIRDY;
  if ((cr & RCC_CR_HSEON) != 0U) {
    wtmask |= RCC_CR_HSERDY;
  }
  if (halRegWaitAllSet32X(&RCC->CR, wtmask,
                          STM32_OSCILLATORS_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* HSI48 setup, if required.*/
  if ((ccp->rcc_crrcr & RCC_CRRCR_HSI48ON) != 0U) {
    halRegWrite32X(&RCC->CRRCR, ccp->rcc_crrcr, true);
    if (halRegWaitAllSet32X(&RCC->CRRCR, RCC_CRRCR_HSI48RDY,
                            STM32_HSI48_STARTUP_TIME,
                            NULL)) {
      return true;
    }
  }

  /* PLL setup and activation.*/
  if ((ccp->rcc_cr & RCC_CR_PLLON) != 0U) {
    halRegWrite32X(&RCC->PLLCFGR, ccp->rcc_pllcfgr, true);
    halRegWrite32X(&RCC->CR, ccp->rcc_cr, true);
    if (halRegWaitAllSet32X(&RCC->CR, RCC_CR_PLLRDY,
                            STM32_PLL_STARTUP_TIME,
                            NULL)) {
      return true;
    }
  }

  /* MCO and bus dividers first, SW left untouched for now.*/
  halRegWrite32X(&RCC->CFGR,
                 (RCC->CFGR & RCC_CFGR_SW_Msk) |
                 (ccp->rcc_cfgr & ~RCC_CFGR_SW_Msk),
                 true);

  /* Final flash ACR settings.*/
  halRegWrite32X(&FLASH->ACR, ccp->flash_acr, true);

  /* Final PWR modes.*/
  halRegWrite32X(&PWR->CR1, ccp->pwr_cr1, true);
  halRegWrite32X(&PWR->CR2, ccp->pwr_cr2, true);
  halRegWrite32X(&PWR->CR5, ccp->pwr_cr5, true);

  if (halRegWaitAllClear32X(&PWR->SR2, PWR_SR2_VOSF,
                            STM32_REGULATORS_TRANSITION_TIME,
                            NULL)) {
    return true;
  }

  /* Waiting for the correct regulator state.*/
  if ((ccp->pwr_cr1 & PWR_CR1_LPR) == 0U) {
    /* Main mode selected.*/
    if (halRegWaitAllClear32X(&PWR->SR2, PWR_SR2_REGLPF,
                              STM32_REGULATORS_TRANSITION_TIME,
                              NULL)) {
      return true;
    }
  }
  else {
    /* Low power mode selected.*/
    if (halRegWaitAllSet32X(&PWR->SR2, PWR_SR2_REGLPF,
                            STM32_REGULATORS_TRANSITION_TIME,
                            NULL)) {
      return true;
    }
  }

  /* Switching to the final clock source.*/
  halRegWrite32X(&RCC->CFGR, ccp->rcc_cfgr, true);
  if (halRegWaitMatch32X(&RCC->CFGR,
                         RCC_CFGR_SWS_Msk,
                         (ccp->rcc_cfgr & RCC_CFGR_SW_Msk) << RCC_CFGR_SWS_Pos,
                         STM32_SYSCLK_SWITCH_TIME,
                         NULL)) {
    return true;
  }

  /* If HSI16 is not in configuration then it is finally shut down.*/
  if ((ccp->rcc_cr & RCC_CR_HSION) == 0U) {
    halRegClear32X(&RCC->CR, RCC_CR_HSION, true);
  }

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
  halfreq_t hsi16clk = 0U, hsi48clk = 0U, hseclk = 0U, pllselclk;
  halfreq_t pllpclk = 0U, pllqclk = 0U, pllrclk = 0U;
  halfreq_t sysclk, hclk, pclk1, pclk2, pclk1tim, pclk2tim, mcoclk;
  uint32_t mcodiv, flashws;

  /* System limits based on desired VOS settings.*/
  if ((ccp->pwr_cr1 & PWR_CR1_VOS_Msk) == PWR_CR1_VOS_1) {
    if ((ccp->pwr_cr5 & PWR_CR5_R1MODE) != 0U) {
      return true;
    }
    slp = &vos_range2;
  }
  else if ((ccp->pwr_cr1 & PWR_CR1_VOS_Msk) == PWR_CR1_VOS_0) {
    if ((ccp->pwr_cr5 & PWR_CR5_R1MODE) != 0U) {
      slp = &vos_range1_boost;
    }
    else {
      slp = &vos_range1_noboost;
    }
  }
  else {
    return true;
  }

  /* HSI16 clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSION) != 0U) {
    hsi16clk = STM32_HSI16CLK;
  }

  /* HSI48 clock after divider.*/
  if ((ccp->rcc_crrcr & RCC_CRRCR_HSI48ON) != 0U) {
    hsi48clk = STM32_HSI48CLK;
  }

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
    pllpdiv = (ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLPDIV_Msk) >> RCC_PLLCFGR_PLLPDIV_Pos;
    if (pllpdiv == 1U) {
      return true;
    }
    if (pllpdiv == 0U) {
      if ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLP) == 0U) {
        pllpdiv = 7U;
      }
      else {
        pllpdiv = 17U;
      }
    }
    if ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLPEN) != 0U) {
      pllpclk = pllvcoclk / pllpdiv;

      if ((pllpclk < slp->pllp_min) || (pllpclk > slp->pllp_max)) {
        return true;
      }
    }

    /* PLL Q output frequency.*/
    pllqdiv = 2U + (2U * (ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLQ_Msk) >> RCC_PLLCFGR_PLLQ_Pos);
    if ((ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLQEN) != 0U) {
      pllqclk = pllvcoclk / pllqdiv;

      if ((pllqclk < slp->pllq_min) || (pllqclk > slp->pllq_max)) {
        return true;
      }
    }

    /* PLL R output frequency.*/
    pllrdiv = 2U + (2U * (ccp->rcc_pllcfgr & RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos);
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
    sysclk = hsi16clk;
    break;
  case RCC_CFGR_SW_HSE:
    sysclk = hseclk;
    break;
  case RCC_CFGR_SW_PLL:
    sysclk = pllrclk;
    break;
  default:
    sysclk = 0U;
  }

  if ((sysclk == 0U) || (sysclk > slp->sysclk_max)) {
    return true;
  }

  /* HCLK frequency.*/
  hclk = sysclk / hprediv[(ccp->rcc_cfgr & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos];

  /* PPRE1 frequency.*/
  pclk1 = hclk / pprediv[(ccp->rcc_cfgr & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos];
  if ((ccp->rcc_cfgr & RCC_CFGR_PPRE1_Msk) < RCC_CFGR_PPRE1_DIV2) {
    pclk1tim = pclk1;
  }
  else {
    pclk1tim = pclk1 * 2U;
  }

  /* PPRE2 frequency.*/
  pclk2 = hclk / pprediv[(ccp->rcc_cfgr & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos];
  if ((ccp->rcc_cfgr & RCC_CFGR_PPRE2_Msk) < RCC_CFGR_PPRE2_DIV2) {
    pclk2tim = pclk2;
  }
  else {
    pclk2tim = pclk2 * 2U;
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
  case STM32_MCOSEL_HSI48:
    mcoclk = STM32_HSI48CLK;
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
  if (hclk > slp->flash_thresholds[flashws]) {
    return true;
  }

  /* Writing out results.*/
  clock_points[CLK_HSI16]    = hsi16clk;
  clock_points[CLK_HSI48]    = hsi48clk;
  clock_points[CLK_HSE]      = hseclk;
  clock_points[CLK_SYSCLK]   = sysclk;
  clock_points[CLK_PLLPCLK]  = pllpclk;
  clock_points[CLK_PLLQCLK]  = pllqclk;
  clock_points[CLK_PLLRCLK]  = pllrclk;
  clock_points[CLK_HCLK]     = hclk;
  clock_points[CLK_PCLK1]    = pclk1;
  clock_points[CLK_PCLK1TIM] = pclk1tim;
  clock_points[CLK_PCLK2]    = pclk2;
  clock_points[CLK_PCLK2TIM] = pclk2tim;
  clock_points[CLK_MCO]      = mcoclk;

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
#if defined(STM32_DMA_REQUIRED)
  dmaInit();
#endif

  /* NVIC initialization.*/
  nvicInit();

  /* IRQ subsystem initialization.*/
  irqInit();
}

/**
 * @brief   STM32G4xx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {

  /* DWT cycles counter enabled, used for timeouts.*/
  halRegSet32X(&CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk, true);
  halRegSet32X(&DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk, true);

#if !STM32_NO_INIT
  /* Assuming HSI16 as initial clock.*/
  hal_lld_set_coreclock(STM32_HSI16CLK);

  /* Reset of all peripherals.
     Note, GPIOs are not reset because initialized before this point in
     board files.*/
  rccResetAHB1(~0);
  rccResetAHB2(~STM32_GPIO_EN_MASK);
  rccResetAHB3(~0);
  rccResetAPB1R1(~0);
  rccResetAPB1R2(~0);
  rccResetAPB2(~0);

  /* SYSCFG clock enabled here because it is a multi-functional unit shared
     among multiple drivers.*/
  rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, false);

  /* RTC APB clock enable.*/
#if (HAL_USE_RTC == TRUE) && defined(RCC_APB1ENR1_RTCAPBEN)
  rccEnableAPB1R1(RCC_APB1ENR1_RTCAPBEN, true);
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

  /* Recalculating all clock points and performing coarse configuration
     validity checks.*/
  if (hal_lld_clock_check_tree(ccp)) {
    return true;
  }

  /* Attempting configuration switch.*/
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
