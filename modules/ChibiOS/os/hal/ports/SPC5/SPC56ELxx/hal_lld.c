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
 * @file    SPC56ELxx/hal_lld.c
 * @brief   SPC56ELxx HAL subsystem low level driver source.
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

  /* The system is switched to the RUN0 mode, the default for normal
     operations.*/
  if (halSPCSetRunMode(SPC5_RUNMODE_RUN0) == OSAL_FAILED) {
    SPC5_CLOCK_FAILURE_HOOK();
  }

  /* Decrementer timer initialized for system tick use, note, it is
     initialized here because in the OSAL layer the system clock frequency
     is not yet known.*/
  n = halSPCGetSystemClock() / OSAL_ST_FREQUENCY;
  asm volatile ("mtspr   22, %[n]           \t\n"   /* Init. DEC register.  */
                "mtspr   54, %[n]           \t\n"   /* Init. DECAR register.*/
                "e_lis   %%r3, 0x0440       \t\n"   /* DIE ARE bits.        */
                "mtspr   340, %%r3"                 /* TCR register.        */
                : : [n] "r" (n) : "r3");

  /* TB counter enabled for debug and measurements.*/
  asm volatile ("e_li    %%r3, 0x4000       \t\n"   /* TBEN bit.            */
                "mtspr   1008, %%r3"                /* HID0 register.       */
                : : : "r3");

  /* EDMA initialization.*/
  edmaInit();
}

/**
 * @brief   SPC56ELxx early initialization.
 * @note    All the involved constants come from the file @p board.h and
 *          @p hal_lld.h
 * @note    This function must be invoked only after the system reset.
 *
 * @special
 */
void spc_clock_init(void) {

  /* Waiting for IRC stabilization before attempting anything else.*/
  while (!ME.GS.B.S_IRCOSC)
    ;

#if !SPC5_NO_INIT

#if SPC5_DISABLE_WATCHDOG
  /* SWT disabled.*/
  SWT.SR.R = 0xC520;
  SWT.SR.R = 0xD928;
  SWT.CR.R = 0xFF00000A;
#endif

  /* Enabling peripheral bridges to allow any operation.*/
  AIPS.MPROT.R      = 0x77777777;
  AIPS.PACR0_7.R    = 0;
  AIPS.PACR8_15.R   = 0;
  AIPS.PACR16_23.R  = 0;
  AIPS.PACR24_31.R  = 0;
  AIPS.OPACR0_7.R   = 0;
  AIPS.OPACR8_15.R  = 0;
  AIPS.OPACR16_23.R = 0;
  AIPS.OPACR24_31.R = 0;
  AIPS.OPACR32_39.R = 0;
  AIPS.OPACR40_47.R = 0;
  AIPS.OPACR48_55.R = 0;
  AIPS.OPACR56_63.R = 0;
  AIPS.OPACR64_71.R = 0;
  AIPS.OPACR72_79.R = 0;
  AIPS.OPACR80_87.R = 0;
  AIPS.OPACR88_95.R = 0;

  /* SSCM initialization. Setting up the most restrictive handling of
     invalid accesses to peripherals.*/
  SSCM.ERROR.R = 3;                             /* PAE and RAE bits.        */

  /* FCCU CF errors clearing.*/
  FCCU.CFK.R        = 0x618B7A50;
  FCCU.CFS[0].R     = 0xFFFFFFFF;
  while (FCCU.CTRL.B.OPS != 3)
    ;
  FCCU.CFK.R        = 0x618B7A50;
  FCCU.CFS[1].R     = 0xFFFFFFFF;
  while (FCCU.CTRL.B.OPS != 3)
    ;

  /* FCCU NCF errors clearing.*/
  FCCU.NCFK.R       = 0xAB3498FE;
  FCCU.NCFS[0].R    = 0xFFFFFFFF;
  while (FCCU.CTRL.B.OPS != 3)
    ;

  /* RGM errors clearing.*/
  RGM.FES.R         = 0xFFFF;
  RGM.DES.R         = 0xFFFF;

  /* The system must be in DRUN mode on entry, if this is not the case then
     it is considered a serious anomaly.*/
  if (ME.GS.B.S_CURRENT_MODE != SPC5_RUNMODE_DRUN) {
    SPC5_CLOCK_FAILURE_HOOK();
  }

#if defined(SPC5_OSC_BYPASS)
  /* If the board is equipped with an oscillator instead of a crystal then the
     bypass must be activated.*/
  CGM.OSC_CTL.B.OSCBYP = TRUE;
#endif /* SPC5_OSC_BYPASS */

  /* Setting the various dividers and source selectors.*/
  CGM.SC_DC0.R    = SPC5_CGM_SC_DC0;
  CGM.AC0_DC0_3.R = SPC5_CGM_AC0_DC0 | SPC5_CGM_AC0_DC1;
  CGM.AC0_SC.R    = SPC5_AUX0CLK_SRC;
  CGM.AC1_DC0_3.R = SPC5_CGM_AC1_DC0;
  CGM.AC1_SC.R    = SPC5_AUX1CLK_SRC;
  CGM.AC2_DC0_3.R = SPC5_CGM_AC2_DC0;
  CGM.AC2_SC.R    = SPC5_AUX2CLK_SRC;
  CGM.AC3_SC.R    = SPC5_FMPLL0_CLK_SRC;
  CGM.AC4_SC.R    = SPC5_FMPLL1_CLK_SRC;

  /* Enables the XOSC in order to check its functionality before proceeding
     with the initialization.*/
  ME.DRUN.R = SPC5_ME_MC_SYSCLK_IRC | SPC5_ME_MC_IRCON | SPC5_ME_MC_XOSC0ON |           \
              SPC5_ME_MC_FLAON_NORMAL | SPC5_ME_MC_MVRON;
  if (halSPCSetRunMode(SPC5_RUNMODE_DRUN) == OSAL_FAILED) {
    SPC5_CLOCK_FAILURE_HOOK();
  }

  /* Initialization of the FMPLLs settings.
     TODO: Add settings for the MR registers.*/
  CGM.FMPLL[0].CR.R = SPC5_FMPLL0_ODF |
                      ((SPC5_FMPLL0_IDF_VALUE - 1) << 26) |
                      (SPC5_FMPLL0_NDIV_VALUE << 16);
  CGM.FMPLL[0].MR.R = 0;
  CGM.FMPLL[1].CR.R = SPC5_FMPLL1_ODF |
                      ((SPC5_FMPLL1_IDF_VALUE - 1) << 26) |
                      (SPC5_FMPLL1_NDIV_VALUE << 16);
  CGM.FMPLL[1].MR.R = 0;

  /* Run modes initialization, note writes to the MC registers are verified
     by a protection mechanism, the operation success is verified at the
     end of the sequence.*/
  ME.IS.R           = 8;                        /* Resetting I_ICONF status.*/
  ME.MER.R          = SPC5_ME_ME_BITS;          /* Enabled run modes.       */
  ME.SAFE.R         = SPC5_ME_SAFE_MC_BITS;     /* SAFE run mode.           */
  ME.DRUN.R         = SPC5_ME_DRUN_MC_BITS;     /* DRUN run mode.           */
  ME.RUN[0].R       = SPC5_ME_RUN0_MC_BITS;     /* RUN0 run mode.           */
  ME.RUN[1].R       = SPC5_ME_RUN1_MC_BITS;     /* RUN1 run mode.           */
  ME.RUN[2].R       = SPC5_ME_RUN2_MC_BITS;     /* RUN2 run mode.           */
  ME.RUN[3].R       = SPC5_ME_RUN3_MC_BITS;     /* RUN0 run mode.           */
  ME.HALT0.R        = SPC5_ME_HALT0_MC_BITS;    /* HALT0 run mode.          */
  ME.STOP0.R        = SPC5_ME_STOP0_MC_BITS;    /* STOP0 run mode.          */
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

  /* CFLASH settings initialized for a maximum clock of 120MHz.*/
  CFLASH.PFCR0.B.B02_APC  = 3;
  CFLASH.PFCR0.B.B02_WWSC = 3;
  CFLASH.PFCR0.B.B02_RWSC = 3;

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
  case SPC5_ME_GS_SYSCLK_XOSC:
    return SPC5_XOSC_CLK;
  case SPC5_ME_GS_SYSCLK_FMPLL0:
    return SPC5_FMPLL0_CLK;
  default:
    return 0;
  }
}
#endif /* !SPC5_NO_INIT */

/** @} */
