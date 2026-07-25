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
 * @file    cmsis_os.h
 * @brief   CMSIS RTOS module macros and structures.
 *
 * @addtogroup CMSIS_OS
 * @{
 */

#ifndef CMSIS_OS_H
#define CMSIS_OS_H

#include "ch.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @brief   API version.
 */
#define osCMSIS                     0x10002

/**
 * @brief   Kernel version.
 */
#define osKernelSystemId            "KERNEL V1.00"

/**
 * @brief   ChibiOS/RT version encoded for CMSIS.
 */
#define osCMSIS_KERNEL              ((CH_KERNEL_MAJOR << 16) |              \
                                     (CH_KERNEL_MINOR << 8) |               \
                                     (CH_KERNEL_PATCH))

/**
 * @name    CMSIS Capabilities
 * @{
 */
#define osFeature_MainThread        1       
#define osFeature_Pool              1
#define osFeature_MailQ             1
#define osFeature_MessageQ          1
#define osFeature_Signals           24
#define osFeature_Semaphore         ((1U << 31) - 1U)
#define osFeature_Wait              0
#define osFeature_SysTick           1
/**< @} */

/**
 * @brief   Wait forever specification for timeouts.
 */
#define osWaitForever               ((uint32_t)-1)

/**
 * @brief   System tick frequency.
 */
#define osKernelSysTickFrequency    CH_CFG_ST_FREQUENCY

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Number of pre-allocated static semaphores/mutexes.
 */
#if !defined(CMSIS_CFG_DEFAULT_STACK)
#define CMSIS_CFG_DEFAULT_STACK     256
#endif

/**
 * @brief   Number of pre-allocated static semaphores/mutexes.
 */
#if !defined(CMSIS_CFG_NUM_SEMAPHORES)
#define CMSIS_CFG_NUM_SEMAPHORES    4
#endif

/**
 * @brief   Number of pre-allocated static timers.
 */
#if !defined(CMSIS_CFG_NUM_TIMERS)
#define CMSIS_CFG_NUM_TIMERS        4
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !CH_CFG_USE_MEMPOOLS
#error "CMSIS RTOS requires CH_CFG_USE_MEMPOOLS"
#endif

#if !CH_CFG_USE_EVENTS
#error "CMSIS RTOS requires CH_CFG_USE_EVENTS"
#endif

#if !CH_CFG_USE_EVENTS_TIMEOUT
#error "CMSIS RTOS requires CH_CFG_USE_EVENTS_TIMEOUT"
#endif

#if !CH_CFG_USE_SEMAPHORES
#error "CMSIS RTOS requires CH_CFG_USE_SEMAPHORES"
#endif

#if !CH_CFG_USE_OBJ_FIFOS
#error "CMSIS RTOS requires CH_CFG_USE_OBJ_FIFOS"
#endif

#if !CH_CFG_USE_DYNAMIC
#error "CMSIS RTOS requires CH_CFG_USE_DYNAMIC"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of priority levels.
 */
typedef enum {
  osPriorityIdle            = -3,
  osPriorityLow             = -2,
  osPriorityBelowNormal     = -1,
  osPriorityNormal          = 0,
  osPriorityAboveNormal     = +1,
  osPriorityHigh            = +2,
  osPriorityRealtime        = +3,
  osPriorityError           = 0x84
} osPriority;

/**
 * @brief   Type of error codes.
 */
typedef enum {
  osOK                      = 0,
  osEventSignal             = 0x08,
  osEventMessage            = 0x10,
  osEventMail               = 0x20,
  osEventTimeout            = 0x40,
  osErrorParameter          = 0x80,
  osErrorResource           = 0x81,
  osErrorTimeoutResource    = 0xC1,
  osErrorISR                = 0x82,
  osErrorISRRecursive       = 0x83,
  osErrorPriority           = 0x84,
  osErrorNoMemory           = 0x85,
  osErrorValue              = 0x86,
  osErrorOS                 = 0xFF,
  os_status_reserved        = 0x7FFFFFFF
} osStatus;

/**
 * @brief   Type of a timer mode.
 */
typedef enum {
  osTimerOnce               = 0,
  osTimerPeriodic           = 1
} os_timer_type;

/**
 * @brief   Type of thread functions.
 */
typedef void (*os_pthread) (void const *argument);

/**
 * @brief   Type of timer callback.
 */
typedef void (*os_ptimer) (void const *argument);

/**
 * @brief   Type of pointer to thread control block.
 */
typedef thread_t *osThreadId;

/**
 * @brief   Type of pointer to timer control block.
 */
typedef struct os_timer_cb {
  virtual_timer_t           vt;
  os_timer_type             type;
  os_ptimer                 ptimer;
  void                      *argument;
  uint32_t                  millisec;
} *osTimerId;

/**
 * @brief   Type of pointer to mutex control block.
 */
typedef binary_semaphore_t *osMutexId;

/**
 * @brief   Type of pointer to semaphore control block.
 */
typedef semaphore_t *osSemaphoreId;

/**
 * @brief   Type of pointer to memory pool control block.
 */
typedef memory_pool_t *osPoolId;

/**
 * @brief   Type of pointer to message queue control block.
 */
typedef mailbox_t *osMessageQId;

/**
 * @brief   Type of pointer to mail queue control block.
 */
typedef objects_fifo_t *osMailQId;

/**
 * @brief   Type of an event.
 */
typedef struct {
  osStatus                  status;
  union {
    uint32_t                v;
    void                    *p;
    int32_t                 signals;
  } value;
  union {
    osMailQId               mail_id;
    osMessageQId            message_id;
  } def;
} osEvent;

/**
 * @brief   Type of a thread definition block.
 */
typedef struct os_thread_def {
  os_pthread                pthread;
  osPriority                tpriority;
  uint32_t                  stacksize;
  const char                *name;
} osThreadDef_t;

/**
 * @brief   Type of a timer definition block.
 */
typedef struct os_timer_def {
  os_ptimer                 ptimer;
} osTimerDef_t;

/**
 * @brief   Type of a mutex definition block.
 */
typedef struct os_mutex_def {
  uint32_t                  dummy;
} osMutexDef_t;

/**
 * @brief   Type of a semaphore definition block.
 */
typedef struct os_semaphore_def {
  uint32_t                  dummy;
} osSemaphoreDef_t;

/**
 * @brief   Type of a memory pool definition block.
 */
typedef struct os_pool_def {
  uint32_t                  pool_sz;
  uint32_t                  item_sz;
  memory_pool_t             *pool;
  void                      *items;
} osPoolDef_t;

/**
 * @brief   Type of a message queue definition block.
 */
typedef struct os_messageQ_def {
  uint32_t                  queue_sz;
  uint32_t                  item_sz;
  mailbox_t                 *mailbox;
  void                      *items;
} osMessageQDef_t;

/**
 * @brief   Type of a mail queue definition block.
 */
typedef struct os_mailQ_def {
  uint32_t                  queue_sz;
  uint32_t                  item_sz;
  objects_fifo_t            *fifo;
  msg_t                     *msgbuf;
  void                      *objbuf;
} osMailQDef_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Convert a microseconds value to a RTOS kernel system timer value.
 */
#define osKernelSysTickMicroSec(microsec) (((uint64_t)microsec *            \
                                            (osKernelSysTickFrequency)) /   \
                                           1000000)

/**
 * @brief   Create a Thread definition.
 */
#if defined(osObjectsExternal)
#define osThreadDef(thd, priority, stacksz, name)                           \
  extern const osThreadDef_t os_thread_def_##thd
#else
#define osThreadDef(thd, priority, stacksz, name)                           \
const osThreadDef_t os_thread_def_##thd = {                                 \
  (thd),                                                                    \
  (priority),                                                               \
  (stacksz),                                                                \
  (name)                                                                    \
}
#endif

/**
 * @brief   Access a Thread definition.
 */
#define osThread(name)  &os_thread_def_##name

/**
 * @brief   Define a Timer object.
 */
#if defined(osObjectsExternal)
#define osTimerDef(name, function)                                          \
  extern const osTimerDef_t os_timer_def_##name
#else
#define osTimerDef(name, function)                                          \
const osTimerDef_t os_timer_def_##name = {                                  \
  (function)                                                                \
}
#endif

/**
 * @brief   Access a Timer definition.
 */
#define osTimer(name) &os_timer_def_##name

/**
 * @brief   Define a Mutex.
 */
#if defined(osObjectsExternal)
#define osMutexDef(name) extern const osMutexDef_t os_mutex_def_##name
#else
#define osMutexDef(name) const osMutexDef_t os_mutex_def_##name = {0}
#endif

/**
 * @brief   Access a Mutex definition.
 */
#define osMutex(name) &os_mutex_def_##name

/**
 * @brief   Define a Semaphore.
 */
#if defined(osObjectsExternal)
#define osSemaphoreDef(name)                                                \
  extern const osSemaphoreDef_t os_semaphore_def_##name
#else                            // define the object
#define osSemaphoreDef(name)                                                \
  const osSemaphoreDef_t os_semaphore_def_##name = {0}
#endif

/**
 * @brief   Access a Semaphore definition.
 */
#define osSemaphore(name) &os_semaphore_def_##name

/**
 * @brief   Define a Memory Pool.
 */
#if defined(osObjectsExternal)
#define osPoolDef(name, no, type)                                           \
  extern const osPoolDef_t os_pool_def_##name
#else
#define osPoolDef(name, no, type)                                           \
static const type os_pool_buf_##name[no];                                   \
static memory_pool_t os_pool_obj_##name;                                    \
const osPoolDef_t os_pool_def_##name = {                                    \
  (no),                                                                     \
  sizeof (type),                                                            \
  (void *)&os_pool_obj_##name,                                              \
  (void *)&os_pool_buf_##name[0]                                            \
}
#endif

/**
 * @brief   Access a Memory Pool definition.
 */
#define osPool(name) &os_pool_def_##name

/**
 * @brief   Define a Message Queue.
 */
#if defined(osObjectsExternal)
#define osMessageQDef(name, queue_sz, type)                                 \
  extern const osMessageQDef_t os_messageQ_def_##name
#else
#define osMessageQDef(name, queue_sz, type)                                 \
static msg_t os_messageQ_buf_##name[queue_sz];                              \
static mailbox_t os_messageQ_obj_##name;                                    \
const osMessageQDef_t os_messageQ_def_##name = {                            \
  (queue_sz),                                                               \
  sizeof (type),                                                            \
  (mailbox_t*)&os_messageQ_obj_##name,                                      \
  (void *)&os_messageQ_buf_##name[0]                                        \
}
#endif

/**
 * @brief   Access a Message Queue definition.
 */
#define osMessageQ(name) &os_messageQ_def_##name

/**
 * @brief   Define a Mail Queue.
 */
#if defined(osObjectsExternal)
#define osMailQDef(name, queue_sz, type)                \
  extern const osMailQDef_t os_mailQ_def_##name         
#else
#define osMailQDef(name, queue_sz, type)                \
  static msg_t os_mailQ_mb_buf_##name[queue_sz];        \
  static type os_mailQ_pool_buf_##name[queue_sz];       \
  static objects_fifo_t os_mailQ_fifo_##name;           \
  const osMailQDef_t os_mailQ_def_##name = {            \
    (queue_sz),                                         \
    sizeof (type),                                      \
    &os_mailQ_fifo_##name,                              \
    (msg_t*)&os_mailQ_mb_buf_##name[0],                 \
    (void *)&os_mailQ_pool_buf_##name[0]                \
  }
#endif

/**
 * @brief   Access a Mail Queue definition.
 */
#define osMailQ(name) &os_mailQ_def_##name

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern int32_t cmsis_os_started;

#ifdef __cplusplus
extern "C" {
#endif
  osStatus osKernelInitialize(void);
  osStatus osKernelStart(void);
  osThreadId osThreadCreate(const osThreadDef_t *thread_def, void *argument);
  osStatus osThreadTerminate(osThreadId thread_id);
  osStatus osThreadSetPriority(osThreadId thread_id, osPriority newprio);
  /*osEvent osWait(uint32_t millisec);*/
  osTimerId osTimerCreate(const osTimerDef_t *timer_def,
                          os_timer_type type,
                          void *argument);
  osStatus osTimerStart(osTimerId timer_id, uint32_t millisec);
  osStatus osTimerStop(osTimerId timer_id);
  osStatus osTimerDelete(osTimerId timer_id);
  int32_t osSignalSet(osThreadId thread_id, int32_t signals);
  int32_t osSignalClear(osThreadId thread_id, int32_t signals);
  osEvent osSignalWait(int32_t signals, uint32_t millisec);
  osSemaphoreId osSemaphoreCreate(const osSemaphoreDef_t *semaphore_def,
                                  int32_t count);
  int32_t osSemaphoreWait(osSemaphoreId semaphore_id, uint32_t millisec);
  osStatus osSemaphoreRelease(osSemaphoreId semaphore_id);
  osStatus osSemaphoreDelete(osSemaphoreId semaphore_id);
  osMutexId osMutexCreate(const osMutexDef_t *mutex_def);
  osStatus osMutexWait(osMutexId mutex_id, uint32_t millisec);
  osStatus osMutexRelease(osMutexId mutex_id);
  osStatus osMutexDelete(osMutexId mutex_id);
  osPoolId osPoolCreate(const osPoolDef_t *pool_def);
  void *osPoolAlloc(osPoolId pool_id);
  void *osPoolCAlloc(osPoolId pool_id);
  osStatus osPoolFree(osPoolId pool_id, void *block);
  osMessageQId osMessageCreate(const osMessageQDef_t *queue_def,
                               osThreadId thread_id);
  osStatus osMessagePut(osMessageQId queue_id,
                        uint32_t info,
                        uint32_t millisec);
  osEvent osMessageGet(osMessageQId queue_id,
                       uint32_t millisec);
  osMailQId osMailCreate(const osMailQDef_t *mail_def,
                         osThreadId thread_id);
  void *osMailAlloc(osMailQId queue_id, uint32_t millisec);
  void *osMailCAlloc(osMailQId queue_id, uint32_t millisec);
  osStatus osMailPut(osMailQId queue_id, void *mail);
  osEvent osMailGet(osMailQId queue_id, uint32_t millisec);
  osStatus osMailFree(osMailQId queue_id, void *mail);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   To be or not to be.
 */
static inline int32_t osKernelRunning(void) {

  return cmsis_os_started;
}

/**
 * @brief   System ticks since start.
 */
static inline uint32_t osKernelSysTick(void) {

  return (uint32_t)chVTGetSystemTimeX();
}

/**
 * @brief   Returns the current thread.
 */
static inline osThreadId osThreadGetId(void) {

  return (osThreadId)chThdGetSelfX();
}

/**
 * @brief   Thread time slice yield.
 */
static inline osStatus osThreadYield(void) {

  chThdYield();

  return osOK;
}

/**
 * @brief   Returns priority of a thread.
 */
static inline osPriority osThreadGetPriority(osThreadId thread_id) {

  return (osPriority)(NORMALPRIO - thread_id->hdr.pqueue.prio);
}

/**
 * @brief   Thread delay in milliseconds.
 */
static inline osStatus osDelay(uint32_t millisec) {

  chThdSleepMilliseconds(millisec);

  return osOK;
}

#endif /* CMSIS_OS_H */

/** @} */
