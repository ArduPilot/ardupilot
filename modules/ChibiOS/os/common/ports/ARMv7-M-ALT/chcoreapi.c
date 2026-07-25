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
 * @file    ARMv7-M-ALT/chcoreapi.c
 * @brief   ARMv7-M (alternate) specific API.
 *
 * @addtogroup ARMV7M_ALT_COREAPI
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

#if (PORT_USE_SYSCALL == TRUE) || defined(__DOXYGEN__)
static THD_FUNCTION(unprivileged_handler, arg) {
  thread_t *utp = chThdGetSelfX();

  (void)arg;

  /* Storing the current PSP position in the thread context, this position
     will be used for system calls processing,*/
  utp->ctx.syscall.s_psp = __get_PSP();

  /* Jump with no return to the context saved at "u_psp". */
  asm volatile ("svc     #1");
  chSysHalt("svc");
}
#endif

/*===========================================================================*/
/* Module interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

#if (PORT_USE_SYSCALL == TRUE) || defined(__DOXYGEN__)
thread_t *chThdCreateUnprivileged(const unprivileged_thread_descriptor_t *utdp) {
  thread_t *utp;
  uint32_t u_psp;
  struct port_extctx *ectxp;

  /* Creating a thread on the unprivileged handler.*/
  thread_descriptor_t td = {
    .name       = utdp->name,
    .wbase      = utdp->wbase,
    .wend       = utdp->wend,
    .prio       = utdp->prio,
    .funcp      = unprivileged_handler,
    .arg        = NULL,
#if CH_CFG_SMP_MODE != FALSE
    .instance   = NULL
#endif
  };
  utp = chThdCreateSuspended(&td);

  /* Persistent thread parameter.*/
  utp->ctx.syscall.p = utdp->arg;

#if PORT_SWITCHED_REGIONS_NUMBER > 0
  /* Regions for the unprivileged thread, will be set up on switch-in.*/
  for (unsigned i = 0U; i < PORT_SWITCHED_REGIONS_NUMBER; i++) {
    utp->ctx.regions[i].rasr = utdp->regions[i].rasr;
    utp->ctx.regions[i].rbar = utdp->regions[i].rbar;
  }
#endif

  /* Creating entry frame.*/
  u_psp = utdp->u_psp - sizeof (struct port_extctx);
  utp->ctx.syscall.u_psp = u_psp;
  ectxp = (struct port_extctx *)u_psp;

  /* Initializing the unprivileged mode entry context, clearing
     all registers.*/
  memset((void *)ectxp, 0, sizeof (struct port_extctx));
  ectxp->pc    = utdp->u_pc;
  ectxp->xpsr  = 0x01000000U;
#if CORTEX_USE_FPU == TRUE
  ectxp->fpscr = __get_FPSCR();
#endif

  /* Starting the thread.*/
  return chThdStart(utp);
}
#endif /* PORT_USE_SYSCALL == TRUE */

/** @} */
