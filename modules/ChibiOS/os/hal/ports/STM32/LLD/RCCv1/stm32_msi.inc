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
 * @file    RCCv1/stm32_msi.inc
 * @brief   Shared MSI clock handler.
 *
 * @addtogroup STM32_MSI_HANDLER
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @name    RCC_CR register bits definitions (legacy)
 * @{
 */
#define STM32_MSIRANGE_MASK                 (15U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_100K                 (0U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_200K                 (1U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_400K                 (2U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_800K                 (3U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_1M                   (4U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_2M                   (5U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_4M                   (6U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_8M                   (7U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_16M                  (8U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_24M                  (9U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_32M                  (10U << RCC_CR_MSIRANGE_Pos)
#define STM32_MSIRANGE_48M                  (11U << RCC_CR_MSIRANGE_Pos)
/** @} */

/**
 * @name    RCC_CR register helpers
 * @{
 */
#define RCC_CR_MSIRANGE_VALUE(n)            ((n) << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_100K                RCC_CR_MSIRANGE_VALUE(0U)
#define RCC_CR_MSIRANGE_200K                RCC_CR_MSIRANGE_VALUE(1U)
#define RCC_CR_MSIRANGE_400K                RCC_CR_MSIRANGE_VALUE(2U)
#define RCC_CR_MSIRANGE_800K                RCC_CR_MSIRANGE_VALUE(3U)
#define RCC_CR_MSIRANGE_1M                  RCC_CR_MSIRANGE_VALUE(4U)
#define RCC_CR_MSIRANGE_2M                  RCC_CR_MSIRANGE_VALUE(5U)
#define RCC_CR_MSIRANGE_4M                  RCC_CR_MSIRANGE_VALUE(6U)
#define RCC_CR_MSIRANGE_8M                  RCC_CR_MSIRANGE_VALUE(7U)
#define RCC_CR_MSIRANGE_16M                 RCC_CR_MSIRANGE_VALUE(8U)
#define RCC_CR_MSIRANGE_24M                 RCC_CR_MSIRANGE_VALUE(9U)
#define RCC_CR_MSIRANGE_32M                 RCC_CR_MSIRANGE_VALUE(10U)
#define RCC_CR_MSIRANGE_48M                 RCC_CR_MSIRANGE_VALUE(11U)
/** @} */

/* ST headers inconsistencies.*/
#if !defined(RCC_CSR_MSISRANGE_Pos) && defined(RCC_CSR_MSISTBYRG_Pos)
#define RCC_CSR_MSISRANGE_Pos   RCC_CSR_MSISTBYRG_Pos
#endif

/**
 * @name    RCC_CSR register bits definitions (legacy)
 * @{
 */
#define STM32_MSISRANGE_MASK                (15U << RCC_CSR_MSISRANGE_Pos)
#define STM32_MSISRANGE_1M                  (4U << RCC_CSR_MSISRANGE_Pos)
#define STM32_MSISRANGE_2M                  (5U << RCC_CSR_MSISRANGE_Pos)
#define STM32_MSISRANGE_4M                  (6U << RCC_CSR_MSISRANGE_Pos)
#define STM32_MSISRANGE_8M                  (7U << RCC_CSR_MSISRANGE_Pos)
/** @} */

/**
 * @name    RCC_CSR register helpers
 * @{
 */
#define RCC_CSR_MSISRANGE_VALUE(n)          ((n) << RCC_CSR_MSISRANGE_Pos)
#define RCC_CSR_MSISRANGE_1M                RCC_CSR_MSISRANGE_VALUE(4U)
#define RCC_CSR_MSISRANGE_2M                RCC_CSR_MSISRANGE_VALUE(5U)
#define RCC_CSR_MSISRANGE_4M                RCC_CSR_MSISRANGE_VALUE(6U)
#define RCC_CSR_MSISRANGE_8M                RCC_CSR_MSISRANGE_VALUE(7U)
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks for robustness.*/
#if !defined(STM32_RCC_HAS_MSI)
#error "STM32_RCC_HAS_MSI not defined in stm32_registry.h"
#endif

/* Checks on configurations.*/
#if !defined(STM32_MSIPLL_ENABLED)
#error "STM32_MSIPLL_ENABLED not defined in mcuconf.h"
#endif

#if !defined(STM32_MSIRANGE)
#error "STM32_MSIRANGE not defined in mcuconf.h"
#endif

#if !defined(STM32_MSISRANGE)
#error "STM32_MSISRANGE not defined in mcuconf.h"
#endif

#if !defined(STM32_LSE_ENABLED)
#error "STM32_LSE_ENABLED not defined in mcuconf.h"
#endif

#if STM32_MSIPLL_ENABLED && !STM32_LSE_ENABLED
#error "STM32_MSIPLL_ENABLED requires LSE"
#endif

/**
 * @brief   MSI frequency.
 */
#if STM32_MSIPLL_ENABLED == TRUE
  #if STM32_MSIRANGE == STM32_MSIRANGE_100K
    #define STM32_MSICLK              98304U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_200K
    #define STM32_MSICLK              196608U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_400K
    #define STM32_MSICLK              393216U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_800K
    #define STM32_MSICLK              786432U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_1M
    #define STM32_MSICLK              1016000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_2M
    #define STM32_MSICLK              1999000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_4M
    #define STM32_MSICLK              3998000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_8M
    #define STM32_MSICLK              7995000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_16M
    #define STM32_MSICLK              15991000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_24M
    #define STM32_MSICLK              23986000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_32M
    #define STM32_MSICLK              32014000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_48M
    #define STM32_MSICLK              48005000U

  #else
    #error "invalid STM32_MSIRANGE value specified"
  #endif

/**
 * @brief   MSIS frequency.
 */
  #if STM32_MSISRANGE == STM32_MSISRANGE_1M
    #define STM32_MSISCLK             1016000U

  #elif STM32_MSISRANGE == STM32_MSISRANGE_2M
    #define STM32_MSISCLK             1999000U

  #elif STM32_MSISRANGE == STM32_MSISRANGE_4M
    #define STM32_MSISCLK             3998000U

  #elif STM32_MSISRANGE == STM32_MSISRANGE_8M
    #define STM32_MSISCLK             7995000U

  #else
    #error "invalid STM32_MSISRANGE value specified"
  #endif

#else /* STM32_MSIPLL_ENABLED == FALSE */
  #if STM32_MSIRANGE == STM32_MSIRANGE_100K
    #define STM32_MSICLK              100000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_200K
    #define STM32_MSICLK              200000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_400K
    #define STM32_MSICLK              400000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_800K
    #define STM32_MSICLK              800000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_1M
    #define STM32_MSICLK              1000000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_2M
    #define STM32_MSICLK              2000000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_4M
    #define STM32_MSICLK              4000000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_8M
    #define STM32_MSICLK              8000000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_16M
    #define STM32_MSICLK              16000000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_24M
    #define STM32_MSICLK              24000000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_32M
    #define STM32_MSICLK              32000000U

  #elif STM32_MSIRANGE == STM32_MSIRANGE_48M
    #define STM32_MSICLK              48000000U

  #else
    #error "invalid STM32_MSIRANGE value specified"
  #endif

  #if STM32_MSISRANGE == STM32_MSISRANGE_1M
    #define STM32_MSISCLK             1000000U

  #elif STM32_MSISRANGE == STM32_MSISRANGE_2M
    #define STM32_MSISCLK             2000000U

  #elif STM32_MSISRANGE == STM32_MSISRANGE_4M
    #define STM32_MSISCLK             4000000U

  #elif STM32_MSISRANGE == STM32_MSISRANGE_8M
    #define STM32_MSISCLK             8000000U

  #else
    #error "invalid STM32_MSISRANGE value specified"
  #endif
#endif /* STM32_MSIPLL_ENABLED == FALSE */

/* Some headers do not have this definition.*/
#if !defined(RCC_CFGR_SWS_MSI)
#define RCC_CFGR_SWS_MSI            0U
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE void msi_enable(void) {

  RCC->CR |= RCC_CR_MSION;
  while ((RCC->CR & RCC_CR_MSIRDY) == 0U) {
    /* Wait until MSI is stable.*/
  }
}

__STATIC_INLINE void msi_disable(void) {

  RCC->CR &= ~RCC_CR_MSION;
}

__STATIC_INLINE void msi_reset(void) {

  /* Resetting MSI defaults.
     Note from RM0432: MSIRANGE can be modified when MSI is OFF (MSION=0)
     or when MSI is ready (MSIRDY=1). MSIRANGE must NOT be modified when
     MSI is ON and NOT ready (MSION=1 and MSIRDY=0).*/
  RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE_Msk) | RCC_CR_MSIRANGE_6;

  /* Making sure MSI is active and ready.*/
  msi_enable();

  /* Clocking from MSI, in case MSI was not the default source.*/
  RCC->CFGR = RCC_CFGR_SW_MSI;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI) {
    /* Wait until MSI is selected.*/
  }
}

__STATIC_INLINE void msi_init(void) {
  uint32_t cr, csr;

  /* Initial clocks setup and wait for MSI stabilization, the MSI clock is
     always enabled because it is the fall back clock when PLL the fails.
     Trim fields are not altered from reset values.*/

  /* MSIRANGE can be set only when MSI is OFF or READY, it is ready after
     reset.*/
#if STM32_MSIPLL_ENABLED
  cr = STM32_MSIRANGE | RCC_CR_MSIPLLEN | RCC_CR_MSION;
#else
  cr = STM32_MSIRANGE | RCC_CR_MSION;
#endif
  RCC->CR = cr;
  while ((RCC->CR & RCC_CR_MSIRDY) == 0U) {
    /* Wait until MSI is stable.*/
  }

  /* Clocking from MSI, in case MSI was not the default source.*/
  RCC->CFGR = 0U;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
    ;                                       /* Wait until MSI is selected.  */

  /* Updating MSISRANGE value. MSISRANGE can be set only when MSIRGSEL is high.
     This range is used exiting the Standby mode until MSIRGSEL is set.*/
  cr      |= RCC_CR_MSIRGSEL;
  RCC->CR  = cr;

  csr      = RCC->CSR;
  csr     &= ~STM32_MSISRANGE_MASK;
  csr     |= STM32_MSISRANGE;
  RCC->CSR = csr;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/** @} */
