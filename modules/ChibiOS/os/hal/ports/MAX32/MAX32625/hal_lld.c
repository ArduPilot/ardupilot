/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/hal_lld.c
 * @brief   ADUCM36x HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Disable watchdog at startup.
 */
#define HAL_CFG_DISABLE_WDG                 TRUE

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CMSIS system core clock variable.
 * @note    It is declared in system_ADuCM36x.h.
 */
uint32_t SystemCoreClock = MAX32_SYSCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

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

  /* IRQ subsystem initialization.*/
  irqInit();
}

/**
 * @brief   ADUCM clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void max32_clock_init(void) {

#if !MAX32_NO_INIT

  /* Changing the clock source. */
  MXC_CLKMAN->clk_ctrl &= ~MXC_F_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT;
  MXC_CLKMAN->clk_ctrl |= MAX32_SYS_SRC;

  /* Applying the appropriate divider for the main core. */
  MXC_CLKMAN->sys_clk_ctrl_0_cm4 = MAX32_CM4_CLKMAN;

#endif /* !MAX32_NO_INIT */
}
/** @} */
