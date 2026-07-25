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
 * @file    rt/src/chtrace.c
 * @brief   Tracer code.
 *
 * @addtogroup trace
 * @details System events tracing service.
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

#if (CH_DBG_TRACE_MASK != CH_DBG_TRACE_MASK_DISABLED) || defined(__DOXYGEN__)
/**
 * @brief   Writes a time stamp and increases the trace buffer pointer.
 *
 * @notapi
 */
NOINLINE static void trace_next(os_instance_t *oip) {

  oip->trace_buffer.ptr->time    = chVTGetSystemTimeX();
#if PORT_SUPPORTS_RT == TRUE
  oip->trace_buffer.ptr->rtstamp = chSysGetRealtimeCounterX();
#else
  oip->trace_buffer.ptr->rtstamp = (rtcnt_t)0;
#endif

  /* Trace hook, useful in order to interface debug tools.*/
  CH_CFG_TRACE_HOOK(oip->trace_buffer.ptr);

  if (++oip->trace_buffer.ptr >= &oip->trace_buffer.buffer[CH_DBG_TRACE_BUFFER_SIZE]) {
    oip->trace_buffer.ptr = &oip->trace_buffer.buffer[0];
  }
}
#endif

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

#if (CH_DBG_TRACE_MASK != CH_DBG_TRACE_MASK_DISABLED) || defined(__DOXYGEN__)
/**
 * @brief   Circular trace buffer initialization.
 * @note    Internal use only.
 *
 * @param[out] tbp      pointer to the @p trace_buffer_t structure
 *
 * @notapi
 */
void __trace_object_init(trace_buffer_t *tbp) {
  unsigned i;

  tbp->suspended = (uint16_t)~CH_DBG_TRACE_MASK;
  tbp->size      = CH_DBG_TRACE_BUFFER_SIZE;
  tbp->ptr       = &tbp->buffer[0];
  for (i = 0U; i < (unsigned)CH_DBG_TRACE_BUFFER_SIZE; i++) {
    tbp->buffer[i].type = CH_TRACE_TYPE_UNUSED;
  }
}

/**
 * @brief   Inserts in the circular debug trace buffer a ready record.
 *
 * @param[in] tp        the thread that just become ready
 * @param[in] msg       the thread ready message
 *
 * @notapi
 */
void __trace_ready(thread_t *tp, msg_t msg) {
  os_instance_t *oip = currcore;

  if ((oip->trace_buffer.suspended & CH_DBG_TRACE_MASK_READY) == 0U) {
    oip->trace_buffer.ptr->type        = CH_TRACE_TYPE_READY;
    oip->trace_buffer.ptr->state       = (uint8_t)tp->state;
    oip->trace_buffer.ptr->u.rdy.tp    = tp;
    oip->trace_buffer.ptr->u.rdy.msg   = msg;
    trace_next(oip);
  }
}

/**
 * @brief   Inserts in the circular debug trace buffer a context switch record.
 *
 * @param[in] ntp       the thread being switched in
 * @param[in] otp       the thread being switched out
 *
 * @notapi
 */
void __trace_switch(thread_t *ntp, thread_t *otp) {
  os_instance_t *oip = currcore;

  if ((oip->trace_buffer.suspended & CH_DBG_TRACE_MASK_SWITCH) == 0U) {
    oip->trace_buffer.ptr->type        = CH_TRACE_TYPE_SWITCH;
    oip->trace_buffer.ptr->state       = (uint8_t)otp->state;
    oip->trace_buffer.ptr->u.sw.ntp    = ntp;
    oip->trace_buffer.ptr->u.sw.wtobjp = otp->u.wtobjp;
    trace_next(oip);
  }
}

/**
 * @brief   Inserts in the circular debug trace buffer an ISR-enter record.
 *
 * @param[in] isr       name of the isr
 *
 * @notapi
 */
void __trace_isr_enter(const char *isr) {
  os_instance_t *oip = currcore;

  if ((oip->trace_buffer.suspended & CH_DBG_TRACE_MASK_ISR) == 0U) {
    port_lock_from_isr();
    oip->trace_buffer.ptr->type        = CH_TRACE_TYPE_ISR_ENTER;
    oip->trace_buffer.ptr->state       = 0U;
    oip->trace_buffer.ptr->u.isr.name  = isr;
    trace_next(oip);
    port_unlock_from_isr();
  }
}

/**
 * @brief   Inserts in the circular debug trace buffer an ISR-leave record.
 *
 * @param[in] isr       name of the isr
 *
 * @notapi
 */
void __trace_isr_leave(const char *isr) {
  os_instance_t *oip = currcore;

  if ((oip->trace_buffer.suspended & CH_DBG_TRACE_MASK_ISR) == 0U) {
    port_lock_from_isr();
    oip->trace_buffer.ptr->type        = CH_TRACE_TYPE_ISR_LEAVE;
    oip->trace_buffer.ptr->state       = 0U;
    oip->trace_buffer.ptr->u.isr.name  = isr;
    trace_next(oip);
    port_unlock_from_isr();
  }
}

/**
 * @brief   Inserts in the circular debug trace buffer an halt record.
 *
 * @param[in] reason    the halt error string
 *
 * @notapi
 */
void __trace_halt(const char *reason) {
  os_instance_t *oip = currcore;

  if ((oip->trace_buffer.suspended & CH_DBG_TRACE_MASK_HALT) == 0U) {
    oip->trace_buffer.ptr->type          = CH_TRACE_TYPE_HALT;
    oip->trace_buffer.ptr->state         = 0;
    oip->trace_buffer.ptr->u.halt.reason = reason;
    trace_next(oip);
  }
}

/**
 * @brief   Adds an user trace record to the trace buffer.
 *
 * @param[in] up1       user parameter 1
 * @param[in] up2       user parameter 2
 *
 * @iclass
 */
void chTraceWriteI(void *up1, void *up2) {
  os_instance_t *oip = currcore;

  chDbgCheckClassI();

  if ((oip->trace_buffer.suspended & CH_DBG_TRACE_MASK_USER) == 0U) {
    oip->trace_buffer.ptr->type       = CH_TRACE_TYPE_USER;
    oip->trace_buffer.ptr->state      = 0;
    oip->trace_buffer.ptr->u.user.up1 = up1;
    oip->trace_buffer.ptr->u.user.up2 = up2;
    trace_next(oip);
  }
}

/**
 * @brief   Adds an user trace record to the trace buffer.
 *
 * @param[in] up1       user parameter 1
 * @param[in] up2       user parameter 2
 *
 * @api
 */
void chTraceWrite(void *up1, void *up2) {

  chSysLock();
  chTraceWriteI(up1, up2);
  chSysUnlock();
}

/**
 * @brief   Suspends one or more trace events.
 *
 * @param[in] mask      mask of the trace events to be suspended
 *
 * @iclass
 */
void chTraceSuspendI(uint16_t mask) {

  chDbgCheckClassI();

  currcore->trace_buffer.suspended |= mask;
}

/**
 * @brief   Suspends one or more trace events.
 *
 * @param[in] mask      mask of the trace events to be suspended
 *
 * @api
 */
void chTraceSuspend(uint16_t mask) {

  chSysLock();
  chTraceSuspendI(mask);
  chSysUnlock();
}

/**
 * @brief   Resumes one or more trace events.
 *
 * @param[in] mask      mask of the trace events to be resumed
 *
 * @iclass
 */
void chTraceResumeI(uint16_t mask) {

  chDbgCheckClassI();

  currcore->trace_buffer.suspended &= ~mask;
}

/**
 * @brief   Resumes one or more trace events.
 *
 * @param[in] mask      mask of the trace events to be resumed
 *
 * @api
 */
void chTraceResume(uint16_t mask) {

  chSysLock();
  chTraceResumeI(mask);
  chSysUnlock();
}
#endif /* CH_DBG_TRACE_MASK != CH_DBG_TRACE_MASK_DISABLED */

/** @} */
