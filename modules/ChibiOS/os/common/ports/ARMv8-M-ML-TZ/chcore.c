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
 * @file    ARMv8-M-ML-TZ/chcore.c
 * @brief   ARMv8-M MainLine port code.
 *
 * @addtogroup ARMV8M_ML_TZ_CORE
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

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

#if (PORT_KERNEL_MODE == PORT_KERNEL_MODE_HOST)  || defined(__DOXYGEN__)
/**
 * @brief   Delayed reschedule workaround function.
 * @note    This function is called when the PendSV handler cannot reschedule
 *          immediately, this can happen if it preempted a non-secure
 *          exception with lower priority. The strategy is to re-trigger
 *          PendSV-S after a small delay so it can reschedule.
 */
void port_delay_reschedule(void) {

}
#endif

/**
 * @brief   Tail ISR context switch code.
 *
 * @return              The thread being switched-in.
 */
thread_t *port_schedule_next(void) {
  thread_t *ntp;

  chSysLock();

  /* TODO statistics, tracing etc */
  ntp = chSchSelectFirst();

  chSysUnlock();

  return ntp;
}

/**
 * @brief   Port-related initialization code.
 *
 * @param[in, out] oip  pointer to the @p os_instance_t structure
 *
 * @notapi
 */
void port_init(os_instance_t *oip) {

  (void)oip;

#if PORT_KERNEL_MODE == PORT_KERNEL_MODE_HOST
  {
    /* Enabling PRIS in order to have two separate priority ranges for
       secure and non-secure states.*/
    uint32_t aircr  = SCB->AIRCR;
    aircr &= ~(uint32_t)SCB_AIRCR_VECTKEY_Msk;
    aircr |= (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_PRIS_Msk);
    SCB->AIRCR =  aircr;

    /* Secure threads keep PendSV-NS masked, this is a secure thread.*/
    __TZ_set_BASEPRI_NS((uint32_t)CORTEX_PRIO_MASK(CORTEX_MINIMUM_PRIORITY));
  }
#endif

  /* Starting in a known IRQ configuration.*/
  port_suspend();

  /* Initializing priority grouping.*/
  NVIC_SetPriorityGrouping(CORTEX_PRIGROUP_INIT);

  /* DWT cycle counter enable.*/
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Initialization of the system vectors used by the port.*/
  NVIC_SetPriority(SVCall_IRQn, CORTEX_PRIORITY_SVCALL);
  NVIC_SetPriority(PendSV_IRQn, CORTEX_PRIORITY_PENDSV);
}

/** @} */
