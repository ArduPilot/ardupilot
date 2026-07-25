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
 * @file    SPC56ECxx/hal_lld.c
 * @brief   SPC56ECxx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include <string.h>

#include "hal.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

typedef void (*storefunc_t)(volatile uint32_t *p, uint32_t w);

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*
 * Special function to be copied in RAM.
 */
static void do_word_store(volatile uint32_t *p, uint32_t w) {

  *p = w;
}

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

  /* The system is switched to the RUN0 mode, the default for normal
     operations.*/
  if (halSPCSetRunMode(SPC5_RUNMODE_RUN0) == OSAL_FAILED) {
    SPC5_CLOCK_FAILURE_HOOK();
  }

  /* Decrementer timer initialized for system tick use, note, it is
     initialized here because in the OSAL layer the system clock frequency
     is not yet known.*/
  n = halSPCGetSystemClock() / OSAL_ST_FREQUENCY;
  port_write_spr(22, n);                            /* Init. DEC register.  */
  port_write_spr(54, n);                            /* Init. DECAR register.*/
  n = 0x04400000;                                   /* DIE ARE bits.        */
  port_write_spr(340, n);                           /* TCR register.        */

  /* TB counter enabled for debug and measurements.*/
  n = 0x4000;                                       /* TBEN bit.            */
  port_write_spr(1008, n);                          /* HID0 register.       */

  /* EDMA initialization.*/
  edmaInit();
}

/**
 * @brief   SPC56ECxx clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h and
 *          @p hal_lld.h
 * @note    This function must be invoked only after the system reset.
 *
 * @special
 */
void spc_clock_init(void) {
#if !SPC5_NO_INIT
  uint32_t reg;
  uint32_t store_word[8];
#endif

  /* Waiting for IRC stabilization before attempting anything else.*/
  while (!ME.GS.B.S_FIRC)
    ;

#if !SPC5_NO_INIT
  /* Copies the store function in RAM, this is required in order to perform
     some flash-related operations.*/
  memcpy(store_word, do_word_store, sizeof(store_word));

#if SPC5_DISABLE_WATCHDOG
  /* SWT disabled.*/
  SWT.SR.R = 0xC520;
  SWT.SR.R = 0xD928;
  SWT.CR.R = 0xFF00000A;
#endif

  /* SSCM initialization. Setting up the most restrictive handling of
     invalid accesses to peripherals.*/
  SSCM.ERROR.R = 3;                             /* PAE and RAE bits.        */

  /* RGM errors clearing.*/
  RGM.FES.R         = 0xFFFF;
  RGM.DES.R         = 0xFFFF;

  /* Oscillators dividers setup.*/
  CGM.FIRC_CTL.B.RCDIV   = SPC5_IRCDIV_VALUE - 1;
  CGM.FXOSC_CTL.B.OSCDIV = SPC5_XOSCDIV_VALUE - 1;

  /* The system must be in DRUN mode on entry, if this is not the case then
     it is considered a serious anomaly.*/
  if (ME.GS.B.S_CURRENTMODE != SPC5_RUNMODE_DRUN) {
    SPC5_CLOCK_FAILURE_HOOK();
  }

#if defined(SPC5_OSC_BYPASS)
  /* If the board is equipped with an oscillator instead of a xtal then the
     bypass must be activated.*/
  CGM.OSC_CTL.B.OSCBYP = TRUE;
#endif /* SPC5_OSC_BYPASS */

  /* Setting the various dividers and source selectors.*/
  CGM.SC_DC0.R      = SPC5_CGM_SC_DC0;
  CGM.SC_DC1.R      = SPC5_CGM_SC_DC1;
  CGM.SC_DC2.R      = SPC5_CGM_SC_DC2;
  CGM.Z0_DCR.R      = SPC5_CGM_Z0_DCR;
  CGM.FEC_DCR.R     = SPC5_CGM_FEC_DCR;
  CGM.FLASH_DCR.R   = SPC5_CGM_FLASH_DCR;

  /* Selecting the external oscillator as source for the FMPLL, note that on
     older silicons, the settings are exchanged, a macro switch is provided.*/
#if SPC56ECXX_FMPLL_CLOCK_ERRATA_WORKAROUND == TRUE
  CGM.AC0_SC.R      = 0x01000000;
#else
  CGM.AC0_SC.R      = 0x00000000;               /* TODO: Add a setting.     */
#endif

  /* Initialization of the FMPLLs settings.*/
  CGM.FMPLL_CR.R = SPC5_FMPLL0_ODF |
                   ((SPC5_FMPLL0_IDF_VALUE - 1) << 26) |
                   (SPC5_FMPLL0_NDIV_VALUE << 16);
  CGM.FMPLL_MR.R = 0;                           /* TODO: Add a setting.     */

  /* Run modes initialization.*/
  ME.IS.R           = 8;                        /* Resetting I_ICONF status.*/
  ME.MER.R          = SPC5_ME_ME_BITS;          /* Enabled run modes.       */
  ME.TEST.R         = SPC5_ME_TEST_MC_BITS;     /* TEST run mode.           */
  ME.SAFE.R         = SPC5_ME_SAFE_MC_BITS;     /* SAFE run mode.           */
  ME.DRUN.R         = SPC5_ME_DRUN_MC_BITS;     /* DRUN run mode.           */
  ME.RUN[0].R       = SPC5_ME_RUN0_MC_BITS;     /* RUN0 run mode.           */
  ME.RUN[1].R       = SPC5_ME_RUN1_MC_BITS;     /* RUN1 run mode.           */
  ME.RUN[2].R       = SPC5_ME_RUN2_MC_BITS;     /* RUN2 run mode.           */
  ME.RUN[3].R       = SPC5_ME_RUN3_MC_BITS;     /* RUN0 run mode.           */
  ME.HALT.R         = SPC5_ME_HALT0_MC_BITS;    /* HALT0 run mode.          */
  ME.STOP.R         = SPC5_ME_STOP0_MC_BITS;    /* STOP0 run mode.          */
  ME.STANDBY.R      = SPC5_ME_STANDBY0_MC_BITS; /* STANDBY0 run mode.       */
  if (ME.IS.B.I_ICONF) {
    /* Configuration rejected.*/
    SPC5_CLOCK_FAILURE_HOOK();
  }

  /* Peripherals run and low power modes initialization.*/
  ME.RUNPC[0].R     = SPC5_ME_RUN_PC0_BITS;
  ME.RUNPC[1].R     = SPC5_ME_RUN_PC1_BITS;
  ME.RUNPC[2].R     = SPC5_ME_RUN_PC2_BITS;
  ME.RUNPC[3].R     = SPC5_ME_RUN_PC3_BITS;
  ME.RUNPC[4].R     = SPC5_ME_RUN_PC4_BITS;
  ME.RUNPC[5].R     = SPC5_ME_RUN_PC5_BITS;
  ME.RUNPC[6].R     = SPC5_ME_RUN_PC6_BITS;
  ME.RUNPC[7].R     = SPC5_ME_RUN_PC7_BITS;
  ME.LPPC[0].R      = SPC5_ME_LP_PC0_BITS;
  ME.LPPC[1].R      = SPC5_ME_LP_PC1_BITS;
  ME.LPPC[2].R      = SPC5_ME_LP_PC2_BITS;
  ME.LPPC[3].R      = SPC5_ME_LP_PC3_BITS;
  ME.LPPC[4].R      = SPC5_ME_LP_PC4_BITS;
  ME.LPPC[5].R      = SPC5_ME_LP_PC5_BITS;
  ME.LPPC[6].R      = SPC5_ME_LP_PC6_BITS;
  ME.LPPC[7].R      = SPC5_ME_LP_PC7_BITS;

  /* CFLASH settings calculated for a maximum clock of 120MHz.*/
  reg = (CFLASH_0.PFCR0.R & 0x073EFFFF) | 0x280A0000;
  ((storefunc_t)store_word)(&CFLASH_0.PFCR0.R, reg);
  reg = (CFLASH_0.PFCR1.R & 0x073EFFFF) | 0x681A0000;
  ((storefunc_t)store_word)(&CFLASH_0.PFCR1.R, reg);

  /* SRAM settings, 1 wait state.*/
  ECSM.MUDCR.B.RAM_WS = 1;

  /* Switches again to DRUN mode (current mode) in order to update the
     settings.*/
  if (halSPCSetRunMode(SPC5_RUNMODE_DRUN) == OSAL_FAILED) {
    SPC5_CLOCK_FAILURE_HOOK();
  }
#endif /* !SPC5_NO_INIT */
}

/**
 * @brief   Switches the system to the specified run mode.
 *
 * @param[in] mode      one of the possible run modes
 *
 * @return              The operation status.
 * @retval OSAL_SUCCESS   if the switch operation has been completed.
 * @retval OSAL_FAILED    if the switch operation failed.
 */
bool halSPCSetRunMode(spc5_runmode_t mode) {

  /* Clearing status register bits I_IMODE(4) and I_IMTC(1).*/
  ME.IS.R = 5;

  /* Starts a transition process.*/
  ME.MCTL.R = SPC5_ME_MCTL_MODE(mode) | SPC5_ME_MCTL_KEY;
  ME.MCTL.R = SPC5_ME_MCTL_MODE(mode) | SPC5_ME_MCTL_KEY_INV;

  /* Waits for the mode switch or an error condition.*/
  while (TRUE) {
    uint32_t r = ME.IS.R;
    if (r & 1)
      return OSAL_SUCCESS;
    if (r & 4)
      return OSAL_FAILED;
  }
}

/**
 * @brief   Changes the clock mode of a peripheral.
 *
 * @param[in] n         index of the @p PCTL register
 * @param[in] pctl      new value for the @p PCTL register
 *
 * @notapi
 */
void halSPCSetPeripheralClockMode(uint32_t n, uint32_t pctl) {
  uint32_t mode;

  ME.PCTL[n].R = pctl;
  mode = ME.MCTL.B.TARGET_MODE;
  ME.MCTL.R = SPC5_ME_MCTL_MODE(mode) | SPC5_ME_MCTL_KEY;
  ME.MCTL.R = SPC5_ME_MCTL_MODE(mode) | SPC5_ME_MCTL_KEY_INV;
}

#if !SPC5_NO_INIT || defined(__DOXYGEN__)
/**
 * @brief   Returns the system clock under the current run mode.
 *
 * @return              The system clock in Hertz.
 */
uint32_t halSPCGetSystemClock(void) {
  uint32_t sysclk;

  sysclk = ME.GS.B.S_SYSCLK;
  switch (sysclk) {
  case SPC5_ME_GS_SYSCLK_IRC:
    return SPC5_IRC_CLK;
  case SPC5_ME_GS_SYSCLK_DIVIRC:
    return SPC5_IRC_CLK / SPC5_IRCDIV_VALUE;
  case SPC5_ME_GS_SYSCLK_XOSC:
    return SPC5_XOSC_CLK / SPC5_XOSCDIV_VALUE;
  case SPC5_ME_GS_SYSCLK_DIVXOSC:
    return SPC5_XOSC_CLK;
  case SPC5_ME_GS_SYSCLK_FMPLL0:
    return SPC5_FMPLL0_CLK;
  default:
    return 0;
  }
}
#endif /* !SPC5_NO_INIT */

/** @} */
