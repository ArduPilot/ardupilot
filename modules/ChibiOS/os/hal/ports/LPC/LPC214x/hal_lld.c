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
 * @file    LPC214x/hal_lld.c
 * @brief   LPC214x HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*
 * Non-vectored IRQs handler, the default action can be overridden by
 * redefining the @p LPC214x_NON_VECTORED_IRQ_HOOK() hook macro.
 */
static CH_IRQ_HANDLER(irq_handler) {

  CH_IRQ_PROLOGUE();

  LPC214x_NON_VECTORED_IRQ_HOOK();

  VICVectAddr = 0;
  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  vic_init();
  VICDefVectAddr = (IOREG32)irq_handler;

}

/**
 * @brief   LPC214x clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function must be invoked only after the system reset.
 *
 * @special
 */
void lpc214x_clock_init(void) {

  /*
   * All peripherals clock disabled by default in order to save power.
   */
  PCONP = PCRTC | PCTIM0;

  /*
   * MAM setup.
   */
  MAMTIM = 0x3;                 /* 3 cycles for flash accesses. */
  MAMCR  = 0x2;                 /* MAM fully enabled. */

  /*
   * PLL setup for Fosc=12MHz and CCLK=48MHz.
   * P=2 M=3.
   */
  PLL *pll = PLL0Base;
  pll->PLL_CFG  = 0x23;         /* P and M values. */
  pll->PLL_CON  = 0x1;          /* Enables the PLL 0. */
  pll->PLL_FEED = 0xAA;
  pll->PLL_FEED = 0x55;
  while (!(pll->PLL_STAT & 0x400))
    ;                           /* Wait for PLL lock. */

  pll->PLL_CON  = 0x3;          /* Connects the PLL. */
  pll->PLL_FEED = 0xAA;
  pll->PLL_FEED = 0x55;

  /*
   * VPB setup.
   * PCLK = CCLK / 4.
   */
  VPBDIV = VPD_D4;
}

/** @} */
