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
 * @file    ARMv6-M-RP2/chcore.c
 * @brief   ARMv6-M-RP2 port code.
 *
 * @addtogroup ARMV6M_RP2_CORE
 * @{
 */

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

/**
 * @brief   Remote replica of @p chSysHalt().
 * @note    The difference is that it does not change the system state and
 *          does not call the port hook again.
 */
void port_local_halt(void) {
  const char *reason = "remote panic";

  port_disable();

  /* Logging the event.*/
  __trace_halt("remote panic");

  /* Pointing to the passed message.*/
  currcore->dbg.panic_msg = reason;

  /* Halt hook code, usually empty.*/
  CH_CFG_SYSTEM_HALT_HOOK(reason);

  /* Harmless infinite loop.*/
  while (true) {
  }
}

/*===========================================================================*/
/* Module interrupt handlers.                                                */
/*===========================================================================*/

#if (CORTEX_ALTERNATE_SWITCH == FALSE) || defined(__DOXYGEN__)
/**
 * @brief   NMI vector.
 * @details The NMI vector is used for exception mode re-entering after a
 *          context switch.
 */
/*lint -save -e9075 [8.4] All symbols are invoked from asm context.*/
void NMI_Handler(void) {
/*lint -restore*/

  /* The port_extctx structure is pointed by the PSP register.*/
  struct port_extctx *ctxp = (struct port_extctx *)__get_PSP();

  /* Discarding the current exception context and positioning the stack to
     point to the real one.*/
  ctxp++;

  /* Writing back the modified PSP value.*/
  __set_PSP((uint32_t)ctxp);

  /* Restoring the normal interrupts status, releasing the spinlock.*/
  port_unlock_from_isr();
}
#endif /* !CORTEX_ALTERNATE_SWITCH */

#if (CORTEX_ALTERNATE_SWITCH == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PendSV vector.
 * @details The PendSV vector is used for exception mode re-entering after a
 *          context switch.
 */
/*lint -save -e9075 [8.4] All symbols are invoked from asm context.*/
void PendSV_Handler(void) {
/*lint -restore*/

  /* The port_extctx structure is pointed by the PSP register.*/
  struct port_extctx *ctxp = (struct port_extctx *)__get_PSP();

  /* Discarding the current exception context and positioning the stack to
     point to the real one.*/
  ctxp++;

  /* Writing back the modified PSP value.*/
  __set_PSP((uint32_t)ctxp);

#if CH_CFG_SMP_MODE== TRUE
   /* Interrupts have been re-enabled in the ASM part but the spinlock is
      still taken, releasing it.*/
   port_spinlock_release();
#endif
}
#endif /* CORTEX_ALTERNATE_SWITCH */

#if (CH_CFG_SMP_MODE== TRUE) || defined(__DOXYGEN__)
/**
 * @brief   FIFO interrupt handler for core 0.
 *
 * @isr
 */
CH_IRQ_HANDLER(Vector7C) {

  CH_IRQ_PROLOGUE();

  /* Error flags cleared and ignored.*/
  SIO->FIFO_ST = SIO_FIFO_ST_ROE | SIO_FIFO_ST_WOF;

  /* Read FIFO is fully emptied.*/
  while ((SIO->FIFO_ST & SIO_FIFO_ST_VLD) != 0U) {
    uint32_t message = SIO->FIFO_RD;
#if defined(PORT_HANDLE_FIFO_MESSAGE)
    if (message != PORT_FIFO_RESCHEDULE_MESSAGE) {
      PORT_HANDLE_FIFO_MESSAGE(1U, message);
    }
#else
    (void)message;
#endif
  }

  /* In case the other core is in WFE.*/
  __SEV();

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   FIFO interrupt handler for core 1.
 *
 * @isr
 */
CH_IRQ_HANDLER(Vector80) {

  CH_IRQ_PROLOGUE();

  /* Error flags cleared and ignored.*/
  SIO->FIFO_ST = SIO_FIFO_ST_ROE | SIO_FIFO_ST_WOF;

  /* Read FIFO is fully emptied.*/
  while ((SIO->FIFO_ST & SIO_FIFO_ST_VLD) != 0U) {
    uint32_t message = SIO->FIFO_RD;
    if (message == PORT_FIFO_PANIC_MESSAGE) {
      port_local_halt();
    }
#if defined(PORT_HANDLE_FIFO_MESSAGE)
    if (message != PORT_FIFO_RESCHEDULE_MESSAGE) {
      PORT_HANDLE_FIFO_MESSAGE(0U, message);
    }
#endif
  }

  /* In case the other core is in WFE.*/
  __SEV();

  CH_IRQ_EPILOGUE();
}
#endif /* CH_CFG_SMP_MODE== TRUE */

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

#if CH_CFG_ST_TIMEDELTA > 0
  /* Activating timer for this instance.*/
  port_timer_enable(oip);
#endif

#if CH_CFG_SMP_MODE== TRUE
  /* FIFO handlers for each core.*/
  SIO->FIFO_ST = SIO_FIFO_ST_ROE | SIO_FIFO_ST_WOF;
  if (oip->core_id == 0U) {
    NVIC_SetPriority(15, CORTEX_MINIMUM_PRIORITY);
    NVIC_EnableIRQ(15);
  }
  else if (oip->core_id == 1U) {
    NVIC_SetPriority(16, CORTEX_MINIMUM_PRIORITY);
    NVIC_EnableIRQ(16);
  }
  else {
    chDbgAssert(false, "unexpected core id");
  }
#endif

#if CORTEX_ALTERNATE_SWITCH == TRUE
  /* Initializing PendSV for the current core, it is only required in
     the alternate switch mode.*/
  NVIC_SetPriority(PendSV_IRQn, CORTEX_PRIORITY_PENDSV);
#endif
}

/**
 * @brief   IRQ epilogue code.
 *
 * @param[in] lr        value of the @p LR register on ISR entry
 */
void __port_irq_epilogue(uint32_t lr) {

  if (lr != 0xFFFFFFF1U) {
    struct port_extctx *ectxp;

    /* Entering system critical zone.*/
    port_lock_from_isr();

    /* The extctx structure is pointed by the PSP register.*/
    ectxp = (struct port_extctx *)__get_PSP();

    /* Adding an artificial exception return context, there is no need to
       populate it fully.*/
    ectxp--;

    /* Writing back the modified PSP value.*/
    __set_PSP((uint32_t)ectxp);

    /* Setting up a fake XPSR register value.*/
    ectxp->xpsr = 0x01000000U;

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
  }
}

#if (CH_CFG_SMP_MODE== TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Takes the kernel spinlock.
 */
void __port_spinlock_take(void) {

  port_spinlock_take();
}

/**
 * @brief   Releases the kernel spinlock.
 */
void __port_spinlock_release(void) {

  port_spinlock_release();
}
#endif /* CH_CFG_SMP_MODE== TRUE */

/** @} */
