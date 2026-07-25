/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    ARMv8-M-ML/chcore.c
 * @brief   ARMv8-M MainLine port code.
 *
 * @addtogroup ARMV8M_ML_CORE
 * @{
 */

#include <string.h>

#include "ch.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module interrupt handlers.                                                */
/*===========================================================================*/

#if (CORTEX_SIMPLIFIED_PRIORITY == FALSE) || defined(__DOXYGEN__)
/**
 * @brief   SVC vector.
 * @details The SVC vector is used for exception mode re-entering after a
 *          context switch and, optionally, for system calls.
 * @note    The SVC vector is only used in advanced kernel mode.
 */
/*lint -save -e9075 [8.4] All symbols are invoked from asm context.*/
void SVC_Handler(void) {
/*lint -restore*/
  uint32_t psp = __get_PSP();

  {
    /* From privileged mode, it is used for context discarding in the
       preemption code.*/

    /* Unstacking procedure, discarding the current exception context and
       positioning the stack to point to the real one.*/
    psp += sizeof (struct port_extctx);

#if CORTEX_USE_FPU == TRUE
    /* Enforcing unstacking of the FP part of the context.*/
    FPU->FPCCR &= ~FPU_FPCCR_LSPACT_Msk;
#endif

    /* Restoring real position of the original stack frame.*/
    __set_PSP(psp);

    /* Restoring the normal interrupts status.*/
    port_unlock_from_isr();
  }
}
#endif /* CORTEX_SIMPLIFIED_PRIORITY == FALSE */

#if (CORTEX_SIMPLIFIED_PRIORITY == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PendSV vector.
 * @details The PendSV vector is used for exception mode re-entering after a
 *          context switch.
 * @note    The PendSV vector is only used in compact kernel mode.
 */
/*lint -save -e9075 [8.4] All symbols are invoked from asm context.*/
void PendSV_Handler(void) {
/*lint -restore*/
  uint32_t psp = __get_PSP();

#if CORTEX_USE_FPU
  /* Enforcing unstacking of the FP part of the context.*/
  FPU->FPCCR &= ~FPU_FPCCR_LSPACT_Msk;
#endif

  /* Discarding the current exception context and positioning the stack to
     point to the real one.*/
  psp += sizeof (struct port_extctx);

  /* Restoring real position of the original stack frame.*/
  __set_PSP(psp);
}
#endif /* CORTEX_SIMPLIFIED_PRIORITY == TRUE */

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Port-related initialization code.
 *
 * @param[in, out] oip  pointer to the @p os_instance_t structure
 *
 * @notapi
 */
void port_init(os_instance_t *oip) {

  (void)oip;

  /* Starting in a known IRQ configuration.*/
  __set_BASEPRI(CORTEX_BASEPRI_DISABLED);
  __enable_irq();

  /* Initializing priority grouping.*/
  NVIC_SetPriorityGrouping(CORTEX_PRIGROUP_INIT);

  /* DWT cycle counter enable, note, the M7 requires DWT unlocking.*/
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//  DWT->LAR = 0xC5ACCE55U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Initialization of the system vectors used by the port.*/
#if CORTEX_SIMPLIFIED_PRIORITY == FALSE
  NVIC_SetPriority(SVCall_IRQn, CORTEX_PRIORITY_SVCALL);
#endif
  NVIC_SetPriority(PendSV_IRQn, CORTEX_PRIORITY_PENDSV);
}

/**
 * @brief   Exception exit redirection to @p __port_switch_from_isr().
 */
void __port_irq_epilogue(void) {

  port_lock_from_isr();
  if ((SCB->ICSR & SCB_ICSR_RETTOBASE_Msk) != 0U) {
    struct port_extctx *ectxp;
    uint32_t s_psp;

#if CORTEX_USE_FPU == TRUE
    /* Enforcing a lazy FPU state save by accessing the FPCSR register.*/
    (void) __get_FPSCR();
#endif

    s_psp = __get_PSP();

    /* Adding an artificial exception return context, there is no need to
       populate it fully.*/
    s_psp -= sizeof (struct port_extctx);

    /* The port_extctx structure is pointed by the S-PSP register.*/
    ectxp = (struct port_extctx *)s_psp;

    /* Setting up a fake XPSR register value.*/
    ectxp->xpsr = 0x01000000U;
#if CORTEX_USE_FPU == TRUE
    ectxp->fpscr = FPU->FPDSCR;
#endif

    /* Writing back the modified S-PSP value.*/
    __set_PSP(s_psp);

    /* The exit sequence is different depending on if a preemption is
       required or not.*/
    if (chSchIsPreemptionRequired()) {
      /* Preemption is required we need to enforce a context switch.*/
      ectxp->pc = (uint32_t)__port_switch_from_isr;
    }
    else {
      /* Preemption not required, we just need to exit the exception
         atomically.*/
      ectxp->pc = (uint32_t)__port_exit_from_isr;
    }

    /* Note, returning without unlocking is intentional, this is done in
       order to keep the rest of the context switch atomic.*/
    return;
  }
  port_unlock_from_isr();
}

/** @} */
