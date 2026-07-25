/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC564Axx/hal_lld.c
 * @brief   SPC564Axx HAL subsystem low level driver source.
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

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {
  uint32_t n;

  /* The SRAM is parked on the load/store port, for some unknown reason it
     is defaulted on the instructions port and this kills performance.*/
  XBAR.SGPCR2.B.PARK = 1;               /* RAM slave on load/store port.*/

  /* The DMA priority is placed above the CPU priority in order to not
     starve I/O activities while the CPU is executing tight loops (FLASH
     and SRAM slave ports only).*/
#if !defined(_SPC564A70_)
  XBAR.MPR0.R = 0x34000021;             /* Flash slave port priorities:
                                            eDMA (4):              0 (highest)
                                            Core Instructions (0): 1
                                            Core Data (1):         2
                                            EBI (7):               3
                                            Flexray (6):           4        */
  XBAR.MPR2.R = 0x34000021;             /* SRAM slave port priorities:
                                            eDMA (4):              0 (highest)
                                            Core Instructions (0): 1
                                            Core Data (1):         2
                                            EBI (7):               3
                                            FlexRay (6):           4        */
#else /* defined(_SPC564A70_) */
  XBAR.MPR0.R = 0x03000021;             /* Flash slave port priorities:
                                            eDMA (4):              0 (highest)
                                            Core Instructions (0): 1
                                            Core Data (1):         2
                                            Flexray (6):           3        */
  XBAR.MPR2.R = 0x03000021;             /* SRAM slave port priorities:
                                            eDMA (4):              0 (highest)
                                            Core Instructions (0): 1
                                            Core Data (1):         2
                                            FlexRay (6):           3        */
#endif /* defined(_SPC564A70_) */

  /* Decrementer timer initialized for system tick use, note, it is
     initialized here because in the OSAL layer the system clock frequency
     is not yet known.*/
  n = SPC5_SYSCLK / OSAL_ST_FREQUENCY;
  asm volatile ("mtspr   22, %[n]           \t\n"   /* Init. DEC register.  */
                "mtspr   54, %[n]           \t\n"   /* Init. DECAR register.*/
                "e_lis   %%r3, 0x0440       \t\n"   /* DIE ARE bits.        */
                "mtspr   340, %%r3"                 /* TCR register.        */
                : : [n] "r" (n) : "r3");

  /* TB counter enabled for debug and measurements.*/
  asm volatile ("e_li    %%r3, 0x4000       \t\n"   /* TBEN bit.            */
                "mtspr   1008, %%r3"                /* HID0 register.       */
                : : : "r3");

  /* eMIOS initialization.*/
  EMIOS.MCR.R = (1U << 26) | SPC5_EMIOS_GPRE;       /* GPREN and GPRE.      */

  /* EDMA initialization.*/
  edmaInit();
}

/**
 * @brief   SPC563 clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h and
 *          @p hal_lld.h
 * @note    This function must be invoked only after the system reset.
 *
 * @special
 */
void spc_clock_init(void) {

#if !SPC5_NO_INIT
  /* PLL activation.*/
  FMPLL.ESYNCR1.B.EMODE     = 1;                    /* Enhanced mode on.    */
  FMPLL.ESYNCR1.B.CLKCFG   &= 1;                    /* Bypass mode, PLL off.*/
#if !SPC5_CLK_BYPASS
  FMPLL.ESYNCR1.B.CLKCFG   |= 2;                    /* PLL on.              */
  FMPLL.ESYNCR1.B.EPREDIV   = SPC5_CLK_PREDIV;
  FMPLL.ESYNCR1.B.EMFD      = SPC5_CLK_MFD;
  FMPLL.ESYNCR2.B.ERFD      = SPC5_CLK_RFD;
  while (!FMPLL.SYNSR.B.LOCK)
    ;
  FMPLL.ESYNCR1.B.CLKCFG   |= 4;                    /* Clock from the PLL.  */
#endif /* !SPC5_CLK_BYPASS */

  /* Setting up RAM/Flash wait states and the prefetching bits.*/
  ECSM.MUDCR.R = SPC5_RAM_WS;
  FLASH_A.BIUCR.R  = SPC5_FLASH_BIUCR | SPC5_FLASH_WS;
  FLASH_A.BIUCR2.R = 0;
#if !defined(_SPC564A70_)
  /* The second controller is only present in Andorra 3M or 4M.*/
  FLASH_B.BIUCR.R  = SPC5_FLASH_BIUCR | SPC5_FLASH_WS;
  FLASH_B.BIUCR2.R = 0;
#endif /* !defined(_SPC564A70_) */
#endif /* !SPC5_NO_INIT */
}

/** @} */
