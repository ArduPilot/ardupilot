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
/*
   Concepts and parts of this file have been contributed by Andre R.
 */

/**
 * @file    cmsis_os.c
 * @brief   CMSIS RTOS module code.
 *
 * @addtogroup CMSIS_OS
 * @{
 */

#include "cmsis_os.h"
#include <string.h>

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

int32_t cmsis_os_started;

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

static memory_pool_t sempool;
static semaphore_t semaphores[CMSIS_CFG_NUM_SEMAPHORES];

static memory_pool_t timpool;
static struct os_timer_cb timers[CMSIS_CFG_NUM_TIMERS];

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

__STATIC_INLINE sysinterval_t tmo(uint32_t millisec) {

  return millisec == osWaitForever ? TIME_INFINITE :
                                     (millisec == 0 ? TIME_IMMEDIATE :
                                                      TIME_MS2I(millisec));
}

/**
 * @brief   Virtual timers common callback.
 */
static void timer_cb(virtual_timer_t *vtp, void const *arg) {
  osTimerId timer_id = (osTimerId)arg;

  (void)vtp;

  timer_id->ptimer(timer_id->argument);
  if (timer_id->type == osTimerPeriodic) {
    chSysLockFromISR();
    chVTDoSetI(&timer_id->vt, TIME_MS2I(timer_id->millisec),
               (vtfunc_t)timer_cb, timer_id);
    chSysUnlockFromISR();
  }
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Kernel initialization.
 *
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 */
osStatus osKernelInitialize(void) {

  cmsis_os_started = 0;

  chSysInit();
  chThdSetPriority(HIGHPRIO);

  chPoolObjectInit(&sempool, sizeof(semaphore_t), chCoreAllocAlignedI);
  chPoolLoadArray(&sempool, semaphores, CMSIS_CFG_NUM_SEMAPHORES);

  chPoolObjectInit(&timpool, sizeof(struct os_timer_cb), chCoreAllocAlignedI);
  chPoolLoadArray(&timpool, timers, CMSIS_CFG_NUM_TIMERS);

  return osOK;
}

/**
 * @brief   Kernel start.
 *
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 */
osStatus osKernelStart(void) {

  cmsis_os_started = 1;

  chThdSetPriority(NORMALPRIO);

  return osOK;
}

/**
 * @brief   Creates a thread.
 *
 * @param[in] thread_def            the thread object declared with @p osThread
 * @param[in] argument              argument for the thread function
 * @return                          The thread identifier.
 * @retval NULL                     if the function failed.
 */
osThreadId osThreadCreate(const osThreadDef_t *thread_def, void *argument) {
  size_t size;

  size = thread_def->stacksize == 0 ? CMSIS_CFG_DEFAULT_STACK :
                                      thread_def->stacksize;
  return (osThreadId)chThdCreateFromHeap(0,
                                         THD_WORKING_AREA_SIZE(size),
                                         thread_def->name,
                                         NORMALPRIO+thread_def->tpriority,
                                         (tfunc_t)thread_def->pthread,
                                         argument);
}

/**
 * @brief   Thread termination.
 * @note    The thread is not really terminated but asked to terminate which
 *          is not compliant.
 *
 * @param[in] thread_id             a thread identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 */
osStatus osThreadTerminate(osThreadId thread_id) {

  if (thread_id == osThreadGetId()) {
    /* Note, no memory will be recovered unless a cleaner thread is
       implemented using the registry.*/
    chThdExit(0);
  }
  chThdTerminate(thread_id);
  chThdWait((thread_t *)thread_id);

  return osOK;
}

/**
 * @brief   Changes a thread priority.
 * @note    This can interfere with the priority inheritance mechanism.
 *
 * @param[in] thread_id             a thread identifier
 * @param[in] newprio               new priority level
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 */
osStatus osThreadSetPriority(osThreadId thread_id, osPriority newprio) {
  thread_t * tp = (thread_t *)thread_id;

  chSysLock();

  /* Changing priority.*/
#if CH_CFG_USE_MUTEXES
  if ((tp->hdr.pqueue.prio == tp->realprio) ||
      ((tprio_t)newprio > tp->hdr.pqueue.prio))
    tp->hdr.pqueue.prio = (tprio_t)newprio;
  tp->realprio = (tprio_t)newprio;
#else
  tp->hdr.pqueue.prio = (tprio_t)newprio;
#endif

  /* The following states need priority queues reordering.*/
  switch (tp->state) {
#if CH_CFG_USE_MUTEXES |                                                    \
    CH_CFG_USE_CONDVARS |                                                   \
    (CH_CFG_USE_SEMAPHORES && CH_CFG_USE_SEMAPHORES_PRIORITY) |             \
    (CH_CFG_USE_MESSAGES && CH_CFG_USE_MESSAGES_PRIORITY)
#if CH_CFG_USE_MUTEXES
  case CH_STATE_WTMTX:
#endif
#if CH_CFG_USE_CONDVARS
  case CH_STATE_WTCOND:
#endif
#if CH_CFG_USE_SEMAPHORES && CH_CFG_USE_SEMAPHORES_PRIORITY
  case CH_STATE_WTSEM:
#endif
#if CH_CFG_USE_MESSAGES && CH_CFG_USE_MESSAGES_PRIORITY
  case CH_STATE_SNDMSGQ:
#endif
    /* Re-enqueues tp with its new priority on the queue.*/
    ch_sch_prio_insert((ch_queue_t *)tp->u.wtobjp,
                       ch_queue_dequeue(&tp->hdr.queue));
    break;
#endif
  case CH_STATE_READY:
#if CH_DBG_ENABLE_ASSERTS
    /* Prevents an assertion in chSchReadyI().*/
    tp->state = CH_STATE_CURRENT;
#endif
    /* Re-enqueues tp with its new priority on the ready list.*/
    chSchReadyI((thread_t *)ch_queue_dequeue(&tp->hdr.queue));
    break;
  }

  /* Rescheduling.*/
  chSchRescheduleS();

  chSysUnlock();

  return osOK;
}

/**
 * @brief   Creates a one-shot or periodic timer.
 * @details The timer is in stopped state until it is started with
 *          @p osTimerStart.
 *
 * @param[in] timer_def             the timer object declared with @p osTimer
 * @param[in] type                  @p osTimerOnce or @p osTimerPeriodic
 * @param[in] argument              argument for the timer callback function
 * @return                          The timer identifier.
 * @retval NULL                     if the function failed.
 */
osTimerId osTimerCreate(const osTimerDef_t *timer_def,
                        os_timer_type type,
                        void *argument) {

  if ((timer_def == NULL) || port_is_isr_context()) {
    return NULL;
  }

  osTimerId timer = chPoolAlloc(&timpool);
  chVTObjectInit(&timer->vt);
  timer->ptimer = timer_def->ptimer;
  timer->type = type;
  timer->argument = argument;

  return timer;
}

/**
 * @brief   Starts or restarts a timer.
 *
 * @param[in] timer_id              a timer identifier
 * @param[in] millisec              time delay value of the timer
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorParameter         if some parameter is @p NULL.
 * @retval osErrorISR               if the function has been called from ISR
 *                                  context.
 * @retval osErrorValue             if a parameter has an invalid value.
 */
osStatus osTimerStart(osTimerId timer_id, uint32_t millisec) {

  if (timer_id == NULL) {
    return osErrorParameter;
  }

  if (port_is_isr_context()) {
    return osErrorISR;
  }

  if ((millisec == 0) || (millisec == osWaitForever)) {
    return osErrorValue;
  }

  timer_id->millisec = millisec;
  chVTSet(&timer_id->vt, TIME_MS2I(millisec), (vtfunc_t)timer_cb, timer_id);

  return osOK;
}

/**
 * @brief   Stops a timer.
 *
 * @param[in] timer_id              a timer identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorParameter         if some parameter is @p NULL.
 * @retval osErrorISR               if the function has been called from ISR
 *                                  context.
 * @retval osErrorResource          if the object is in an invalid state.
 */
osStatus osTimerStop(osTimerId timer_id) {

  if (timer_id == NULL) {
    return osErrorParameter;
  }

  if (port_is_isr_context()) {
    return osErrorISR;
  }

  if (chVTIsArmed(&timer_id->vt) == false) {
    return osErrorResource;
  }

  chVTReset(&timer_id->vt);

  return osOK;
}

/**
 * @brief   Deletes the timer object.
 *
 * @param[in] thread_id             a timer identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorParameter         if some parameter is @p NULL.
 * @retval osErrorISR               if the function has been called from ISR
 *                                  context.
 */
osStatus osTimerDelete(osTimerId timer_id) {

  if (timer_id == NULL) {
    return osErrorParameter;
  };

  if (port_is_isr_context()) {
    return osErrorISR;
  }

  chVTReset(&timer_id->vt);
  chPoolFree(&timpool, (void *)timer_id);

  return osOK;
}

/**
 * @brief   Sends signals.
 *
 * @param[in] thread_id             a thread identifier
 * @param[in] signals               signals to be added to the thread
 * @return                          The previous signals mask.
 */
int32_t osSignalSet(osThreadId thread_id, int32_t signals) {
  int32_t oldsignals;

  syssts_t sts = chSysGetStatusAndLockX();

  oldsignals = (int32_t)thread_id->epending;
  chEvtSignalI((thread_t *)thread_id, (eventmask_t)signals);

  chSysRestoreStatusX(sts);

  return oldsignals;
}

/**
 * @brief   Clears signals.
 *
 * @param[in] thread_id             a thread identifier
 * @param[in] signals               signals to be cleared from the thread
 * @return                          The signals mask.
 */
int32_t osSignalClear(osThreadId thread_id, int32_t signals) {
  eventmask_t m;

  syssts_t sts = chSysGetStatusAndLockX();

  m = thread_id->epending & (eventmask_t)signals;
  thread_id->epending &= ~(eventmask_t)signals;

  chSysRestoreStatusX(sts);

  return (int32_t)m;
}

/**
 * @brief   Waits for signals.
 *
 * @param[in] signals               signals to waited for
 * @return                          An @p osEvent structure.
 */
osEvent osSignalWait(int32_t signals, uint32_t millisec) {
  osEvent event;

  if (signals == 0)
    event.value.signals = (uint32_t)chEvtWaitAnyTimeout(ALL_EVENTS,
                                                        tmo(millisec));
  else
    event.value.signals = (uint32_t)chEvtWaitAllTimeout((eventmask_t)signals,
                                                        tmo(millisec));

  /* Type of event.*/
  if (event.value.signals == 0)
    event.status = osEventTimeout;
  else
    event.status = osEventSignal;

  return event;
}

/**
 * @brief   Creates a semaphore.
 * @note    @p semaphore_def is not used in this implementation.
 * @note    Can involve memory allocation.
 *
 * @param[in] semaphore_def         the semaphore object declared with
 *                                  @p osSemaphore
 * @param[in] count                 the initial semaphore value
 * @return                          The semaphore identifier.
 * @retval NULL                     if the function failed.
 */
osSemaphoreId osSemaphoreCreate(const osSemaphoreDef_t *semaphore_def,
                                int32_t count) {

  (void)semaphore_def;

  semaphore_t *sem = chPoolAlloc(&sempool);
  chSemObjectInit(sem, (cnt_t)count);
  return sem;
}

/**
 * @brief   Waits on a semaphore.
 *
 * @param[in] semaphore_id          a semaphore identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorParameter         if some parameter is @p NULL.
 * @retval osErrorISR               if the function has been called from ISR
 *                                  context.
 * @retval osErrorResource          if the object is in an invalid state.
 * @retval osErrorValue             if a parameter has an invalid value.
 * @retval osErrorTimeoutResource   if the function timed out.
 */
int32_t osSemaphoreWait(osSemaphoreId semaphore_id, uint32_t millisec) {
/* TODO it is not consistent with specification.*/
  msg_t msg = chSemWaitTimeout((semaphore_t *)semaphore_id, tmo(millisec));
  switch (msg) {
  case MSG_OK:
    return osOK;
  case MSG_TIMEOUT:
    return osErrorTimeoutResource;
  }
  return osErrorResource;
}

/**
 * @brief   Releases a semaphore.
 *
 * @param[in] semaphore_id          a semaphore identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 */
osStatus osSemaphoreRelease(osSemaphoreId semaphore_id) {

  syssts_t sts = chSysGetStatusAndLockX();
  chSemSignalI((semaphore_t *)semaphore_id);
  chSysRestoreStatusX(sts);

  return osOK;
}

/**
 * @brief   Deletes a semaphore.
 * @note    After deletion there could be references in the system to a
 *          non-existent semaphore.
 *
 * @param[in] semaphore_id          a semaphore identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 */
osStatus osSemaphoreDelete(osSemaphoreId semaphore_id) {

  chSemReset((semaphore_t *)semaphore_id, 0);
  chPoolFree(&sempool, (void *)semaphore_id);

  return osOK;
}

/**
 * @brief   Creates a mutex.
 * @note    @p mutex_def is not used.
 * @note    Can involve memory allocation.
 *
 * @param[in] mutex_def             the mutex object declared with @p osMutex
 * @return                          The mutex identifier.
 * @retval NULL                     if the function failed.
 */
osMutexId osMutexCreate(const osMutexDef_t *mutex_def) {

  (void)mutex_def;

  binary_semaphore_t *mtx = chPoolAlloc(&sempool);
  chBSemObjectInit(mtx, false);
  return mtx;
}

/**
 * @brief   Waits on a mutex.
 *
 * @param[in] mutex_id              a mutex identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorResource          if the object is in an invalid state.
 * @retval osErrorTimeoutResource   if the function timed out.
 */
osStatus osMutexWait(osMutexId mutex_id, uint32_t millisec) {

  msg_t msg = chBSemWaitTimeout((binary_semaphore_t *)mutex_id, tmo(millisec));
  switch (msg) {
  case MSG_OK:
    return osOK;
  case MSG_TIMEOUT:
    return osErrorTimeoutResource;
  }
  return osErrorResource;
}

/**
 * @brief   Releases a mutex.
 *
 * @param[in] mutex_id              a mutex identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 */
osStatus osMutexRelease(osMutexId mutex_id) {

  syssts_t sts = chSysGetStatusAndLockX();
  chBSemSignalI((binary_semaphore_t *)mutex_id);
  chSysRestoreStatusX(sts);

  return osOK;
}

/**
 * @brief   Deletes a mutex.
 * @note    After deletion there could be references in the system to a
 *          non-existent semaphore.
 *
 * @param[in] mutex_id              a mutex identifier
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 */
osStatus osMutexDelete(osMutexId mutex_id) {

  chSemReset((semaphore_t *)mutex_id, 0);
  chPoolFree(&sempool, (void *)mutex_id);

  return osOK;
}

/**
 * @brief   Creates a memory pool.
 * @note    The pool is not really created because it is allocated statically,
 *          this function just re-initializes it.
 *
 * @param[in] pool_def              the pool object declared with @p osPool
 * @return                          The pool identifier.
 * @retval NULL                     if the function failed.
 */
osPoolId osPoolCreate(const osPoolDef_t *pool_def) {

  if ((pool_def == NULL) || port_is_isr_context()) {
    return NULL;
  }

  chPoolObjectInit(pool_def->pool, (size_t)pool_def->item_sz, NULL);
  chPoolLoadArray(pool_def->pool, pool_def->items, (size_t)pool_def->pool_sz);

  return (osPoolId)pool_def->pool;
}

/**
 * @brief   Allocates a memory block from the memory pool.
 *
 * @param[in] pool_id               a pool identifier
 * @return                          The pointer to the allocated memory block.
 * @retval NULL                     if the function failed.
 */
void *osPoolAlloc(osPoolId pool_id) {
  void *object;

  if (pool_id == NULL) {
    return NULL;
  }

  syssts_t sts = chSysGetStatusAndLockX();
  object = chPoolAllocI((memory_pool_t *)pool_id);
  chSysRestoreStatusX(sts);

  return object;
}

/**
 * @brief   Allocates a memory block from the memory pool with clearing.
 *
 * @param[in] pool_id               a pool identifier
 * @return                          The pointer to the allocated memory block.
 * @retval NULL                     if the function failed.
 */
void *osPoolCAlloc(osPoolId pool_id) {
  void *object;

  if (pool_id == NULL) {
    return NULL;
  }

  syssts_t sts = chSysGetStatusAndLockX();
  object = chPoolAllocI((memory_pool_t *)pool_id);
  chSysRestoreStatusX(sts);

  if (object != NULL) {
    memset(object, 0, pool_id->object_size);
  }

  return object;
}

/**
 * @brief   Returns a memory block to a memory pool.
 *
 * @param[in] pool_id               a pool identifier
 * @param[in] block                 pointer to an allocated memory block
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorParameter         if some parameter is @p NULL.
 */
osStatus osPoolFree(osPoolId pool_id, void *block) {

  if (pool_id == NULL) {
    return osErrorParameter;
  }

  syssts_t sts = chSysGetStatusAndLockX();
  chPoolFreeI((memory_pool_t *)pool_id, block);
  chSysRestoreStatusX(sts);

  return osOK;
}

/**
 * @brief   Creates a message queue.
 * @note    The queue is not really created because it is allocated statically,
 *          this function just re-initializes it.
 *
 * @param[in] queue_def             the message queue object declared with
 *                                  @p osMessageQDef
 * @param[in] thread_id             a thread identifier
 * @return                          The message queue identifier.
 * @retval NULL                     if the function failed.
 */
osMessageQId osMessageCreate(const osMessageQDef_t *queue_def,
                             osThreadId thread_id) {

  /* Ignoring this parameter for now.*/
  (void)thread_id;

  if (queue_def->item_sz > sizeof (msg_t)) {
    return NULL;
  }

  chMBObjectInit(queue_def->mailbox,
                 queue_def->items,
                 (size_t)queue_def->queue_sz);

  return (osMessageQId) queue_def->mailbox;
}

/**
 * @brief   Sends a message to the queue.
 *
 * @param[in] queue_id              a message queue identifier
 * @param[in] millisec              the timeout value
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorParameter         if some parameter is @p NULL.
 * @retval osErrorISR               if the function has been called from ISR
 *                                  context.
 * @retval osErrorResource          if the object is in an invalid state.
 * @retval osErrorValue             if a parameter has an invalid value.
 * @retval osErrorTimeoutResource   if the function timed out.
 */
osStatus osMessagePut(osMessageQId queue_id,
                      uint32_t info,
                      uint32_t millisec) {
  msg_t msg;

  if (port_is_isr_context()) {

    /* Waiting makes no sense in ISRs so any value except "immediate"
       makes no sense.*/
    if (millisec != 0)
      return osErrorValue;

    chSysLockFromISR();
    msg = chMBPostI((mailbox_t *)queue_id, (msg_t)info);
    chSysUnlockFromISR();
  }
  else
    msg = chMBPostTimeout((mailbox_t *)queue_id, (msg_t)info, tmo(millisec));

  return msg == MSG_OK ? osOK : osErrorTimeoutResource;
}

/**
 * @brief   Waits for a message from the queue.
 *
 * @param[in] queue_id              a message queue identifier
 * @param[in] millisec              the timeout value
 * @return                          An @p osEvent structure.
 */
osEvent osMessageGet(osMessageQId queue_id, uint32_t millisec) {
  msg_t msg;
  osEvent event = {
    .status = osErrorOS,
    .value = {
      .v = 0U
    },
    .def = {
      .mail_id = NULL
    }
  };

  event.def.message_id = queue_id;

  if (port_is_isr_context()) {

    /* Waiting makes no sense in ISRs so any value except "immediate"
       makes no sense.*/
    if (millisec != 0) {
      event.status = osErrorValue;
      return event;
    }

    chSysLockFromISR();
    msg = chMBFetchI((mailbox_t *)queue_id, (msg_t*)&event.value.v);
    chSysUnlockFromISR();
  }
  else {
    msg = chMBFetchTimeout((mailbox_t *)queue_id, (msg_t*)&event.value.v, tmo(millisec));
  }

  /* Returned event type.*/
  event.status = msg == MSG_OK ? osEventMessage : osEventTimeout;
  return event;
}

/**
 * @brief   Creates a mail queue.
 *
 * @param[in] queue_def             the mail object declared with @p osMailQDef
 * @param[in] thread_id             a thread identifier
 * @return                          The thread identifier.
 * @retval NULL                     if the function failed.
 */
osMailQId osMailCreate(const osMailQDef_t *mail_def, osThreadId thread_id) {

  /* Ignoring this parameter for now.*/
  (void)thread_id;

  if ((mail_def == NULL) || port_is_isr_context()) {
    return NULL;
  }

  chDbgCheck(mail_def != NULL);

  /* Messages queue initialization.*/
  chFifoObjectInit(mail_def->fifo,
                   (size_t)mail_def->item_sz,
                   (size_t)mail_def->queue_sz,
                   mail_def->objbuf,
                   mail_def->msgbuf);

  return mail_def->fifo;
}

/**
 * @brief   Allocates a mail object.
 *
 * @param[in] queue_id              a mail queue identifier
 * @param[in] millisec              the timeout value
 * @return                          The pointer to the allocated memory block.
 * @retval NULL                     if the function failed.
 */
void *osMailAlloc(osMailQId queue_id, uint32_t millisec) {
  void *mail;

  if ((queue_id == NULL) ||
    (port_is_isr_context() && (millisec > 0))) {
    return NULL;
  }

  if (port_is_isr_context()) {
    chSysLockFromISR();
    mail = chFifoTakeObjectI(queue_id);
    chSysUnlockFromISR();
  }
  else {
    mail = chFifoTakeObjectTimeout(queue_id, tmo(millisec));
  }

  return mail;
}

/**
 * @brief   Allocates a mail object with clearing.
 *
 * @param[in] queue_id              a mail queue identifier
 * @param[in] millisec              the timeout value
 * @return                          The pointer to the allocated memory block.
 * @retval NULL                     if the function failed.
 */
void *osMailCAlloc(osMailQId queue_id, uint32_t millisec) {
  void *mail;

  if ((queue_id == NULL) || (port_is_isr_context() && (millisec > 0))) {
    return NULL;
  }

  if (port_is_isr_context()) {
    chSysLockFromISR();
    mail = chFifoTakeObjectI(queue_id);
    chSysUnlockFromISR();
  }
  else {
    mail = chFifoTakeObjectTimeout(queue_id, tmo(millisec));
  }

  if (mail != NULL) {
    memset(mail, 0, queue_id->free.pool.object_size);
  }

  return mail;
}

/**
 * @brief   Sends a mail object.
 *
 * @param[in] queue_id              a mail queue identifier
 * @param[in] mail                  memory block previously returned by
 *                                  @p osMailAlloc() or @p osMailCAlloc()
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorValue             if mail is @p NULL.
 * @retval osErrorParameter         if some parameter is @p NULL.
 */
osStatus osMailPut(osMailQId queue_id, void *mail) {

  if (queue_id == NULL) {
    return osErrorParameter;
  }

  if (mail == NULL) {
    return osErrorValue;
  }

  chDbgCheck((queue_id != NULL) && (mail != NULL));

  if (port_is_isr_context()) {
    /* Waiting makes no sense in ISRs so any value except "immediate"
       makes no sense.*/
    chSysLockFromISR();
    chFifoSendObjectI(queue_id, mail);
    chSysUnlockFromISR();
  }
  else {
    chFifoSendObject(queue_id, mail);
  }

  return MSG_OK;
}

/**
 * @brief   Waits for an incoming mail object.
 *
 * @param[in] queue_id              a mail queue identifier
 * @param[in] millisec              the timeout value
 * @return                          An @p osEvent structure.
 */
osEvent osMailGet(osMailQId queue_id, uint32_t millisec) {
  msg_t msg;
  osEvent event = {
    .status = osErrorOS,
    .value = {
      .p = NULL
    },
    .def = {
      .mail_id = NULL
    }
  };
  
  if (queue_id == NULL) {
    event.status = osErrorParameter;
    return event;
  }

  chDbgCheck(queue_id != NULL);

  event.def.mail_id = queue_id;

  if (port_is_isr_context()) {
    /* Waiting makes no sense in ISRs so any value except "immediate"
       makes no sense.*/
    if (millisec != 0) {
      event.status = osErrorValue;
      return event;
    }

    chSysLockFromISR();
    msg = chFifoReceiveObjectI(queue_id, &event.value.p);
    chSysUnlockFromISR();
  }
  else {
    msg = chFifoReceiveObjectTimeout(queue_id, &event.value.p, tmo(millisec));
  }

  /* Returned event type.*/
  if ((millisec == 0) && (msg == MSG_TIMEOUT)) {
    event.status = osOK;
  } else {
    event.status = msg == MSG_OK ? osEventMail : osEventTimeout;
  }

  return event;
}

/**
 * @brief   Frees the mail object space after receiving it.
 *
 * @param[in] queue_id              a mail queue identifier
 * @param[in] mail                  pointer to the memory block returned
 *                                  by @p osMailGet.
 * @return                          The function execution status.
 * @retval osOK                     if the function succeeded.
 * @retval osErrorValue             if mail is @p NULL.
 * @retval osErrorParameter         if the value to the parameter queue_id is @p NULL.
 */
osStatus osMailFree(osMailQId queue_id, void *mail)
{
  if (queue_id == NULL) {
    return osErrorParameter;
  }

  if (mail == NULL) {
    return osErrorValue;
  }

  chDbgCheck((queue_id != NULL) && (mail != NULL));    

  syssts_t sts = chSysGetStatusAndLockX();
  chFifoReturnObjectI(queue_id, mail);
  chSysRestoreStatusX(sts);

  return osOK;
}
/** @} */
