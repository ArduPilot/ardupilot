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
 * @file    STM32C0xx+/hal_lld.c
 * @brief   STM32C0xx+ HAL subsystem low level driver source.
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
#define STM32_WS_THRESHOLDS                 2

/**
 * @name    Registers reset values
 * @{
 */
#define STM32_FLASH_ACR_RESET               0x00040600U
#define STM32_RCC_CR_RESET                  0x00001540U
#define STM32_RCC_CFGR_RESET                0x00000000U
/** @} */

/**
 * @name    Stabilization times
 * @{
 */
#define STM32_RELAXED_TIMEOUT_FACTOR        5U
#define STM32_HSI_STARTUP_TIME              (4U * STM32_RELAXED_TIMEOUT_FACTOR)
#define STM32_OSCILLATORS_STARTUP_TIME      (2000U * STM32_RELAXED_TIMEOUT_FACTOR)
#define STM32_SYSCLK_SWITCH_TIME            (50U * STM32_RELAXED_TIMEOUT_FACTOR)
/** @} */

/* Reserved bit to be kept at 1, ST mysteries.*/
#define FLASH_ACR_RESVD10                   (1U << 10)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_stm32c0xx.h.
 */
uint32_t SystemCoreClock = STM32_HCLK;

/**
 * @brief   Post-reset clock configuration.
 */
const halclkcfg_t hal_clkcfg_reset = {
  .rcc_cr               = STM32_RCC_CR_RESET,
  .rcc_cfgr             = STM32_RCC_CFGR_RESET,
  .flash_acr            = STM32_FLASH_ACR_RESET
};

/**
 * @brief   Default clock configuration.
 * @note    This is the configuration defined in mcuconf.h.
 */
const halclkcfg_t hal_clkcfg_default = {
  .rcc_cr               = 0U
#if STM32_HSIUSB48_ENABLED == TRUE
                        | RCC_CR_HSIUSB48ON
#endif
#if defined(STM32_HSE_BYPASS)
                        | RCC_CR_HSEBYP
#endif
#if STM32_HSE_ENABLED == TRUE
                        | RCC_CR_HSEON
#endif
#if STM32_HSI48_ENABLED == TRUE
                        | RCC_CR_HSION  | RCC_CR_HSIKERON
#endif
                        | STM32_HSIDIV  | STM32_HSIKERDIV | STM32_SYSDIV,
  .rcc_cfgr             = STM32_MCOPRE  | STM32_MCOSEL  |
                          STM32_MCO2PRE | STM32_MCO2SEL |
                          STM32_PPRE    | STM32_HPRE    |
                          STM32_SW,
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
  [CLK_HSE]             = STM32_HSECLK,
  [CLK_HSISYS]          = STM32_HSISYSCLK,
  [CLK_HSIKER]          = STM32_HSIKERCLK,
#if defined(RCC_CR_HSIUSB48ON)
  [CLK_HSIUSB48]        = STM32_HSIUSB48CLK,
#endif
  [CLK_HCLK]            = STM32_HCLK,
  [CLK_PCLK]            = STM32_PCLK,
  [CLK_PCLKTIM]         = STM32_TIMPCLK,
  [CLK_MCO1]            = STM32_MCOCLK,
  [CLK_MCO2]            = STM32_MCO2CLK
};
#endif /* defined(HAL_LLD_USE_CLOCK_MANAGEMENT) */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#include "stm32_bd_v3.inc"

/**
 * @brief   Configures the SYSCFG unit.
 */
static void hal_lld_set_static_syscfg(void) {

  /* SYSCFG clock enabled.*/
  rccEnableAPBR2(RCC_APBENR2_SYSCFGEN, false);

  halRegWrite32X(&SYSCFG->CFGR1, STM32_SYSCFG_CFGR1, true);
  halRegWrite32X(&SYSCFG->CFGR2, STM32_SYSCFG_CFGR2, true);
  halRegWrite32X(&SYSCFG->CFGR3, STM32_SYSCFG_CFGR3, true);
}

/**
 * @brief   Configures the PWR unit.
 * @note    CR1 is not initialized inside this function.
 */
static void hal_lld_set_static_pwr(void) {

  /* PWR clock enabled.*/
  rccEnablePWRInterface(false);

  /* Static PWR configurations.*/
#if defined(PWR_CR2_PVM_VDDIO2_Pos)
  halRegWrite32X(&PWR->CR2,   STM32_PWR_CR2,   true);
#endif
  halRegWrite32X(&PWR->CR3,   STM32_PWR_CR3,   true);
  halRegWrite32X(&PWR->CR4,   STM32_PWR_CR4,   true);
  halRegWrite32X(&PWR->PUCRA, STM32_PWR_PUCRA, true);
  halRegWrite32X(&PWR->PDCRA, STM32_PWR_PDCRA, true);
  halRegWrite32X(&PWR->PDCRB, STM32_PWR_PDCRB, true);
  halRegWrite32X(&PWR->PUCRB, STM32_PWR_PUCRB, true);
  halRegWrite32X(&PWR->PUCRC, STM32_PWR_PUCRC, true);
  halRegWrite32X(&PWR->PDCRC, STM32_PWR_PDCRC, true);
#if STM32_HAS_GPIOD
  halRegWrite32X(&PWR->PUCRD, STM32_PWR_PUCRD, true);
  halRegWrite32X(&PWR->PDCRD, STM32_PWR_PDCRD, true);
#endif
#if STM32_HAS_GPIOE
  halRegWrite32X(&PWR->PUCRE, STM32_PWR_PUCRE, true);
  halRegWrite32X(&PWR->PDCRE, STM32_PWR_PDCRE, true);
#endif
#if STM32_HAS_GPIOF
  halRegWrite32X(&PWR->PUCRF, STM32_PWR_PUCRF, true);
  halRegWrite32X(&PWR->PDCRF, STM32_PWR_PDCRF, true);
#endif
#if STM32_HAS_GPIOG
  halRegWrite32X(&PWR->PUCRG, STM32_PWR_PUCRG, true);
  halRegWrite32X(&PWR->PDCRG, STM32_PWR_PDCRG, true);
#endif
#if STM32_HAS_GPIOH
  halRegWrite32X(&PWR->PUCRH, STM32_PWR_PUCRH, true);
  halRegWrite32X(&PWR->PDCRH, STM32_PWR_PDCRH, true);
#endif
#if STM32_HAS_GPIOI
  halRegWrite32X(&PWR->PUCRI, STM32_PWR_PUCRI, true);
  halRegWrite32X(&PWR->PDCRI, STM32_PWR_PDCRI, true);
#endif
}

/**
 * @brief   Initializes static muxes and dividers.
 */
static void hal_lld_set_static_clocks(void) {

  /* CCIPR register initialization.*/
  halRegWrite32X(&RCC->CCIPR,
                 STM32_ADCSEL | STM32_I2S1SEL | STM32_I2C1SEL | STM32_FDCAN1SEL | STM32_USART1SEL,
                 true);

#if STM32_HAS_USB1
  /* CCIPR2 register initialization.*/
  halRegWrite32X(&RCC->CCIPR2, STM32_USBSEL, true);
#endif
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

  /* Using default clock settings (HSION) during transition.*/
  halRegWrite32X(&RCC->CR, STM32_RCC_CR_RESET, true);
  if (halRegWaitAllSet32X(&RCC->CR, RCC_CR_HSIRDY,
                          STM32_HSI_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* Making sure to run from HSI.*/
  halRegWrite32X(&RCC->CFGR, STM32_RCC_CFGR_RESET, true);
  if (halRegWaitMatch32X(&RCC->CFGR,
                         RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_HSI,
                         STM32_SYSCLK_SWITCH_TIME,
                         NULL)) {
    return true;
  }

  /* Enabling all required oscillators at same time, HSI enforced active
     because running from it.*/
  halRegWrite32X(&RCC->CR, ccp->rcc_cr | RCC_CR_HSION, true);

  /* Adding to the "wait mask" the status bits of other enabled oscillators,
     waiting for all oscillators to become ready.*/
  wtmask = RCC_CR_HSIRDY;                  /* Known to be ready already.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    wtmask |= RCC_CR_HSERDY;
  }
#if defined(RCC_CR_HSIUSB48ON)
  if ((ccp->rcc_cr & RCC_CR_HSIUSB48ON) != 0U) {
    wtmask |= RCC_CR_HSIUSB48RDY;
  }
#endif
  if (halRegWaitAllSet32X(&RCC->CR,
                          wtmask,
                          STM32_OSCILLATORS_STARTUP_TIME,
                          NULL)) {
    return true;
  }

  /* Final RCC CFGR settings (prescalers, MCO, etc).*/
  halRegWrite32X(&RCC->CFGR, ccp->rcc_cfgr, true);

  /* Final flash ACR settings according to the target configuration.*/
  halRegWrite32X(&FLASH->ACR, ccp->flash_acr | FLASH_ACR_RESVD10, true);

  /* Waiting for the requested SYSCLK source to become active (SWS == SW). */
  if (halRegWaitMatch32X(&RCC->CFGR,
                         RCC_CFGR_SWS_Msk, (ccp->rcc_cfgr & RCC_CFGR_SW_Msk) << RCC_CFGR_SWS_Pos,
                         STM32_SYSCLK_SWITCH_TIME,
                         NULL)) {
    return true;
  }

  /* Final RCC_CR value, HSI could go off at this point if it is not part
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
  static const halfreq_t flash_thresholds[STM32_WS_THRESHOLDS] = {STM32_0WS_THRESHOLD, STM32_1WS_THRESHOLD};
  halfreq_t hsi48clk = 0U, hseclk = 0U, hsisysclk = 0U, hsikerclk = 0U;
#if defined(RCC_CR_HSIUSB48ON)
  halfreq_t hsiusb48clk = 0U;
#endif
  halfreq_t sysclk, hclk, pclk, pclktim, mco1clk, mco2clk;
  uint32_t mcodiv, flashws;

  /* HSE clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSEON) != 0U) {
    hseclk = STM32_HSECLK;
  }

  /* HSI48 clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSION) != 0U) {
    hsi48clk = STM32_HSI48CLK;
    hsisysclk = hsi48clk / (1U << ((ccp->rcc_cr & RCC_CR_HSIDIV_Msk) >> RCC_CR_HSIDIV_Pos));
    hsikerclk = hsi48clk / (((ccp->rcc_cr & RCC_CR_HSIKERDIV_Msk) >> RCC_CR_HSIKERDIV_Pos) + 1U);
  }

#if defined(RCC_CR_HSIUSB48ON)
  /* HSIUSB48 clock.*/
  if ((ccp->rcc_cr & RCC_CR_HSIUSB48ON) != 0U) {
    hsiusb48clk = STM32_HSIUSB48CLK;
  }
#endif

  /* SYSCLK frequency.*/
  switch (ccp->rcc_cfgr & RCC_CFGR_SW_Msk) {
  case RCC_CFGR_SW_HSISYS:
    sysclk = hsisysclk;
    break;
  case RCC_CFGR_SW_HSE:
    sysclk = hseclk;
    break;
#if defined(RCC_CR_HSIUSB48ON)
  case RCC_CFGR_SW_HSIUSB48:
    sysclk = hsiusb48clk;
    break;
#endif
  case RCC_CFGR_SW_LSI:
    sysclk = STM32_LSICLK;
    break;
  case RCC_CFGR_SW_LSE:
    sysclk = STM32_LSECLK;
    break;
  default:
    sysclk = 0U;
  }

  if ((sysclk == 0U) || (sysclk > STM32_SYSCLK_MAX)) {
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
  case RCC_CFGR_MCOSEL_SYSCLK:
    mco1clk = sysclk;
    break;
  case RCC_CFGR_MCOSEL_HSI48:
    mco1clk = hsi48clk;
    break;
  case RCC_CFGR_MCOSEL_HSE:
    mco1clk = hseclk;
    break;
  case RCC_CFGR_MCOSEL_LSI:
    mco1clk = STM32_LSICLK;
    break;
  case RCC_CFGR_MCOSEL_LSE:
    mco1clk = STM32_LSECLK;
    break;
#if defined(RCC_CR_HSIUSB48ON)
  case RCC_CFGR_MCOSEL_HSIUSB48:
    mco1clk = hsiusb48clk;
    break;
#endif
  default:
    mco1clk = 0U;
  }
  mcodiv = 1U << ((ccp->rcc_cfgr & RCC_CFGR_MCOPRE_Msk) >> RCC_CFGR_MCOPRE_Pos);
#if defined(RCC_CFGR_MCOSEL_3)
  if (mcodiv > 1024U) {
#else
  if (mcodiv > 128U) {
#endif
    return true;
  }
  mco1clk /= mcodiv;

  /* MCO2 clock.*/
  switch (ccp->rcc_cfgr & RCC_CFGR_MCO2SEL_Msk) {
  case RCC_CFGR_MCO2SEL_SYSCLK:
    mco2clk = sysclk;
    break;
  case RCC_CFGR_MCO2SEL_HSI48:
    mco2clk = hsi48clk;
    break;
  case RCC_CFGR_MCO2SEL_HSE:
    mco2clk = hseclk;
    break;
  case RCC_CFGR_MCO2SEL_LSI:
    mco2clk = STM32_LSICLK;
    break;
  case RCC_CFGR_MCO2SEL_LSE:
    mco2clk = STM32_LSECLK;
    break;
#if defined(RCC_CR_HSIUSB48ON)
  case RCC_CFGR_MCO2SEL_HSIUSB48:
    mco2clk = hsiusb48clk;
    break;
#endif
  default:
    mco2clk = 0U;
  }
  mcodiv = 1U << ((ccp->rcc_cfgr & RCC_CFGR_MCO2PRE_Msk) >> RCC_CFGR_MCO2PRE_Pos);
#if defined(RCC_CFGR_MCO2SEL_3)
  if (mcodiv > 1024U) {
#else
  if (mcodiv > 128U) {
#endif
    return true;
  }
  mco2clk /= mcodiv;

  /* Flash settings.*/
  flashws = ((ccp->flash_acr & FLASH_ACR_LATENCY_Msk) >> FLASH_ACR_LATENCY_Pos);
  if (flashws >= STM32_WS_THRESHOLDS) {
    return true;
  }
  if (hclk > flash_thresholds[flashws]) {
    return true;
  }

  /* Writing out results.*/
  clock_points[CLK_SYSCLK]    = sysclk;
  clock_points[CLK_HSE]       = hseclk;
  clock_points[CLK_HSISYS]    = hsisysclk;
  clock_points[CLK_HSIKER]    = hsikerclk;
#if defined(RCC_CR_HSIUSB48ON)
  clock_points[CLK_HSIUSB48]  = hsiusb48clk;
#endif
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
 * @brief   STM32G0xx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void stm32_clock_init(void) {
  int32_t div;

  /* Enabling TIM17 for timeout handling.*/
  rccResetTIM17();
  rccEnableTIM17(false);
  /* Clamp divider to avoid underflow on low TIMPCLK values. */
  div = ((int32_t)STM32_TIMPCLK / 1000000) - 1;
  if (div < 0) {
    div = 0;
  }
  halRegWrite32X(&TIM17->PSC, (uint32_t)div, true);
//  halRegWrite32X(&TIM7->ARR, 0xFFFFU, true);
  halRegWrite32X(&TIM17->EGR, TIM_EGR_UG, false);
  halRegWrite32X(&TIM17->CR1, TIM_CR1_CEN, true);

#if !STM32_NO_INIT
  /* Reset of all peripherals.*/
  rccResetAHB(~0);
  rccResetAPBR1(~0);
  rccResetAPBR2(~0);

  /* RTC APB clock enable.*/
#if (HAL_USE_RTC == TRUE) && defined(RCC_APBENR1_RTCAPBEN)
  rccEnableAPBR1(RCC_APBENR1_RTCAPBEN, false);
#endif

  /* Static SYSCFG configurations.*/
  hal_lld_set_static_syscfg();

  /* Static PWR configurations.*/
  hal_lld_set_static_pwr();

  /* Backup domain reset.*/
  bd_reset();

  /* Clocks setup.*/
  lse_init();
  lsi_init();

  /* Backup domain initializations.*/
  bd_init();

  /* Static clocks setup.*/
  hal_lld_set_static_clocks();

  /* Selecting the default clock configuration. */
  halSftFailOnError(hal_lld_clock_configure(&hal_clkcfg_default), "clkinit");

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
  halRegWrite32X(&TIM17->PSC, (uint32_t)div, true);
  halRegWrite32X(&TIM17->EGR, TIM_EGR_UG, false);

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
