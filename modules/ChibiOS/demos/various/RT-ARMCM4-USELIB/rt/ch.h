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

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "cmparams.h"

#define CH_H 
#define __CHIBIOS_RT__ 
#define CH_KERNEL_STABLE 1
#define CH_KERNEL_VERSION "7.0.6"
#define CH_KERNEL_MAJOR 7
#define CH_KERNEL_MINOR 0
#define CH_KERNEL_PATCH 6
#define FALSE 0
#define TRUE 1
#define CHLICENSE_H 
#define CH_FEATURES_BASIC 0
#define CH_FEATURES_INTERMEDIATE 1
#define CH_FEATURES_FULL 2
#define CH_DEPLOY_UNLIMITED -1
#define CH_DEPLOY_NONE 0
#define CH_LICENSE_GPL 0
#define CH_LICENSE_GPL_EXCEPTION 1
#define CH_LICENSE_COMMERCIAL_FREE 2
#define CH_LICENSE_COMMERCIAL_DEV_1000 3
#define CH_LICENSE_COMMERCIAL_DEV_5000 4
#define CH_LICENSE_COMMERCIAL_FULL 5
#define CH_LICENSE_COMMERCIAL_RUNTIME 6
#define CH_LICENSE_PARTNER 7
#define CHVERSION_H 
#define __CHIBIOS__ 
#define CH_VERSION_STABLE 1
#define CH_VERSION "2021.11.5"
#define CH_VERSION_YEAR 21
#define CH_VERSION_MONTH 11
#define CH_VERSION_PATCH 5
#define CH_VERSION_NICKNAME "Agropoli"
#define CH_VERSION_DATE (((CH_VERSION_YEAR + 2000) * 100) + CH_VERSION_MONTH)
#define CHCUSTOMER_H 
#define CH_CUSTOMER_ID_STRING "Santa, North Pole"
#define CH_CUSTOMER_ID_CODE "xxxx-yyyy"
#define CH_CUSTOMER_LICENSE_EOS_DATE 209912
#define CH_CUSTOMER_LICENSE_VERSION_YEAR 99
#define CH_CUSTOMER_LICENSE_VERSION_MONTH 12
#define CH_LICENSE CH_LICENSE_GPL
#define CH_CUSTOMER_LIC_RT TRUE
#define CH_CUSTOMER_LIC_NIL TRUE
#define CH_CUSTOMER_LIC_OSLIB TRUE
#define CH_CUSTOMER_LIC_EX TRUE
#define CH_CUSTOMER_LIC_SB TRUE
#define CH_CUSTOMER_LIC_PORT_CM0 TRUE
#define CH_CUSTOMER_LIC_PORT_CM3 TRUE
#define CH_CUSTOMER_LIC_PORT_CM4 TRUE
#define CH_CUSTOMER_LIC_PORT_CM7 TRUE
#define CH_CUSTOMER_LIC_PORT_CM33 TRUE
#define CH_CUSTOMER_LIC_PORT_ARM79 TRUE
#define CH_CUSTOMER_LIC_PORT_E200Z0 TRUE
#define CH_CUSTOMER_LIC_PORT_E200Z2 TRUE
#define CH_CUSTOMER_LIC_PORT_E200Z3 TRUE
#define CH_CUSTOMER_LIC_PORT_E200Z4 TRUE
#define CH_CUSTOMER_LICENSE_VERSION_DATE (((CH_CUSTOMER_LICENSE_VERSION_YEAR + 2000) * 100) + CH_CUSTOMER_LICENSE_VERSION_MONTH)
#define CH_LICENSE_TYPE_STRING "GNU General Public License 3 (GPL3)"
#define CH_LICENSE_ID_STRING "N/A"
#define CH_LICENSE_ID_CODE "N/A"
#define CH_LICENSE_MODIFIABLE_CODE TRUE
#define CH_LICENSE_FEATURES CH_FEATURES_FULL
#define CH_LICENSE_MAX_DEPLOY CH_DEPLOY_UNLIMITED
#define CHCONF_H 
#define _CHIBIOS_RT_CONF_ 
#define _CHIBIOS_RT_CONF_VER_7_0_ 
#define CH_CFG_SMP_MODE FALSE
#define CH_CFG_ST_RESOLUTION 32
#define CH_CFG_ST_FREQUENCY 1000
#define CH_CFG_INTERVALS_SIZE 32
#define CH_CFG_TIME_TYPES_SIZE 32
#define CH_CFG_ST_TIMEDELTA 0
#define CH_CFG_TIME_QUANTUM 0
#define CH_CFG_NO_IDLE_THREAD FALSE
#define CH_CFG_OPTIMIZE_SPEED TRUE
#define CH_CFG_USE_TM TRUE
#define CH_CFG_USE_TIMESTAMP TRUE
#define CH_CFG_USE_REGISTRY TRUE
#define CH_CFG_USE_WAITEXIT TRUE
#define CH_CFG_USE_SEMAPHORES TRUE
#define CH_CFG_USE_SEMAPHORES_PRIORITY FALSE
#define CH_CFG_USE_MUTEXES TRUE
#define CH_CFG_USE_MUTEXES_RECURSIVE FALSE
#define CH_CFG_USE_CONDVARS TRUE
#define CH_CFG_USE_CONDVARS_TIMEOUT TRUE
#define CH_CFG_USE_EVENTS TRUE
#define CH_CFG_USE_EVENTS_TIMEOUT TRUE
#define CH_CFG_USE_MESSAGES TRUE
#define CH_CFG_USE_MESSAGES_PRIORITY FALSE
#define CH_CFG_USE_DYNAMIC TRUE
#define CH_CFG_USE_MAILBOXES TRUE
#define CH_CFG_USE_MEMCORE TRUE
#define CH_CFG_MEMCORE_SIZE 0
#define CH_CFG_USE_HEAP TRUE
#define CH_CFG_USE_MEMPOOLS TRUE
#define CH_CFG_USE_OBJ_FIFOS TRUE
#define CH_CFG_USE_PIPES TRUE
#define CH_CFG_USE_OBJ_CACHES TRUE
#define CH_CFG_USE_DELEGATES TRUE
#define CH_CFG_USE_JOBS TRUE
#define CH_CFG_USE_FACTORY TRUE
#define CH_CFG_FACTORY_MAX_NAMES_LENGTH 8
#define CH_CFG_FACTORY_OBJECTS_REGISTRY TRUE
#define CH_CFG_FACTORY_GENERIC_BUFFERS TRUE
#define CH_CFG_FACTORY_SEMAPHORES TRUE
#define CH_CFG_FACTORY_MAILBOXES TRUE
#define CH_CFG_FACTORY_OBJ_FIFOS TRUE
#define CH_CFG_FACTORY_PIPES TRUE
#define CH_DBG_STATISTICS FALSE
#define CH_DBG_SYSTEM_STATE_CHECK FALSE
#define CH_DBG_ENABLE_CHECKS FALSE
#define CH_DBG_ENABLE_ASSERTS FALSE
#define CH_DBG_TRACE_MASK CH_DBG_TRACE_MASK_DISABLED
#define CH_DBG_TRACE_BUFFER_SIZE 128
#define CH_DBG_ENABLE_STACK_CHECK FALSE
#define CH_DBG_FILL_THREADS FALSE
#define CH_DBG_THREADS_PROFILING FALSE
#define CH_CFG_SYSTEM_EXTRA_FIELDS 
#define CH_CFG_SYSTEM_INIT_HOOK() { }
#define CH_CFG_OS_INSTANCE_EXTRA_FIELDS 
#define CH_CFG_OS_INSTANCE_INIT_HOOK(oip) { }
#define CH_CFG_THREAD_EXTRA_FIELDS 
#define CH_CFG_THREAD_INIT_HOOK(tp) { }
#define CH_CFG_THREAD_EXIT_HOOK(tp) { }
#define CH_CFG_CONTEXT_SWITCH_HOOK(ntp,otp) { }
#define CH_CFG_IRQ_PROLOGUE_HOOK() { }
#define CH_CFG_IRQ_EPILOGUE_HOOK() { }
#define CH_CFG_IDLE_ENTER_HOOK() { }
#define CH_CFG_IDLE_LEAVE_HOOK() { }
#define CH_CFG_IDLE_LOOP_HOOK() { }
#define CH_CFG_SYSTEM_TICK_HOOK() { }
#define CH_CFG_SYSTEM_HALT_HOOK(reason) { }
#define CH_CFG_TRACE_HOOK(tep) { }
#define CH_CFG_RUNTIME_FAULTS_HOOK(mask) { }
#define CHCHECKS_H 
#define CHRESTRICTIONS_H 
#define CHEARLY_H 
#define CHTYPES_H 
#define CCPORTAB_H 
#define CC_SECTION(s) __attribute__((section(s)))
#define CC_WEAK __attribute__((weak))
#define CC_USED __attribute__((used))
#define CC_ALIGN_DATA(n) __attribute__((aligned(n)))
#define CC_ALIGN_CODE(n) __attribute__((aligned(n)))
#define CC_PACK __attribute__((packed))
#define CC_NO_INLINE __attribute__((noinline))
#define CC_FORCE_INLINE __attribute__((always_inline))
#define CC_NO_RETURN __attribute__((noreturn))
#define CC_ROMCONST const
#define CC_LIKELY(x) __builtin_expect(!!(x), 1)
#define CC_UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PORT_ARCH_SIZEOF_DATA_PTR 4
#define PORT_ARCH_SIZEOF_CODE_PTR 4
#define PORT_ARCH_REGISTERS_WIDTH 32
#define PORT_ARCH_REVERSE_ORDER 1
typedef uint32_t port_rtcnt_t;
typedef uint64_t port_rttime_t;
typedef uint32_t port_syssts_t;
typedef uint64_t port_stkalign_t;
#define PORT_DOES_NOT_PROVIDE_TYPES 
#define ROMCONST CC_ROMCONST
#define NOINLINE CC_NO_INLINE
#define ALIGNED_VAR(n) CC_ALIGN_DATA(n)
#define PACKED_VAR CC_PACK
#define SIZEOF_PTR PORT_ARCH_SIZEOF_DATA_PTR
#define PORT_LIKELY(x) CC_LIKELY(x)
#define PORT_UNLIKELY(x) CC_UNLIKELY(x)
typedef port_rtcnt_t rtcnt_t;
typedef port_rttime_t rttime_t;
typedef port_syssts_t syssts_t;
typedef port_stkalign_t stkalign_t;
typedef uint8_t tmode_t;
typedef uint8_t tstate_t;
typedef uint8_t trefs_t;
typedef uint8_t tslices_t;
typedef uint32_t tprio_t;
typedef int32_t msg_t;
typedef int32_t eventid_t;
typedef uint32_t eventmask_t;
typedef uint32_t eventflags_t;
typedef int32_t cnt_t;
typedef uint32_t ucnt_t;
typedef unsigned core_id_t;
typedef struct ch_thread thread_t;
typedef struct ch_os_instance os_instance_t;
#define __CH_STRINGIFY(a) #a
#define __CH_OFFSETOF(st,m) ((size_t)((char *)&((st *)0)->m - (char *)0))
#define __CH_USED(x) (void)(x)
#define likely(x) PORT_LIKELY(x)
#define unlikely(x) PORT_UNLIKELY(x)
#define threadref(p) ((thread_t *)(void *)(p))
  void chSysHalt(const char *reason);
#define CHRFCU_H 
#define CH_RFCU_VT_INSUFFICIENT_DELTA 1U
#define CH_RFCU_VT_SKIPPED_DEADLINE 2U
#define CH_RFCU_ALL_FAULTS ((rfcu_mask_t)-1)
typedef uint32_t rfcu_mask_t;
typedef struct ch_rfcu {
  rfcu_mask_t mask;
} rfcu_t;
  void chRFCUCollectFaultsI(rfcu_mask_t mask);
  rfcu_mask_t chRFCUGetAndClearFaultsI(rfcu_mask_t mask);
static inline void __rfcu_object_init(rfcu_t *rfcup) {
  rfcup->mask = (rfcu_mask_t)0;
}
#define CHDEBUG_H 
#define CH_DBG_STACK_FILL_VALUE 0x55
typedef struct ch_system_debug {
  const char * volatile panic_msg;
} system_debug_t;
#define __dbg_check_disable() 
#define __dbg_check_suspend() 
#define __dbg_check_enable() 
#define __dbg_check_lock() 
#define __dbg_check_unlock() 
#define __dbg_check_lock_from_isr() 
#define __dbg_check_unlock_from_isr() 
#define __dbg_check_enter_isr() 
#define __dbg_check_leave_isr() 
#define chDbgCheckClassI() 
#define chDbgCheckClassS() 
#define chDbgCheck(c) do { if (CH_DBG_ENABLE_CHECKS != FALSE) { if (unlikely(!(c))) { chSysHalt(__func__); } } } while (false)
#define chDbgAssert(c,r) do { if (CH_DBG_ENABLE_ASSERTS != FALSE) { if (unlikely(!(c))) { chSysHalt(__func__); } } } while (false)
static inline void __dbg_object_init(system_debug_t *sdp) {
  sdp->panic_msg = NULL;
}
#define CHTIME_H 
#define TIME_IMMEDIATE ((sysinterval_t)0)
#define TIME_INFINITE ((sysinterval_t)-1)
#define TIME_MAX_INTERVAL ((sysinterval_t)-2)
#define TIME_MAX_SYSTIME ((systime_t)-1)
typedef uint32_t systime_t;
typedef uint32_t sysinterval_t;
typedef uint64_t systimestamp_t;
typedef uint32_t time_secs_t;
typedef uint32_t time_msecs_t;
typedef uint32_t time_usecs_t;
typedef uint64_t time_conv_t;
#define TIME_S2I(secs) ((sysinterval_t)((time_conv_t)(secs) * (time_conv_t)CH_CFG_ST_FREQUENCY))
#define TIME_MS2I(msecs) ((sysinterval_t)((((time_conv_t)(msecs) * (time_conv_t)CH_CFG_ST_FREQUENCY) + (time_conv_t)999) / (time_conv_t)1000))
#define TIME_US2I(usecs) ((sysinterval_t)((((time_conv_t)(usecs) * (time_conv_t)CH_CFG_ST_FREQUENCY) + (time_conv_t)999999) / (time_conv_t)1000000))
#define TIME_I2S(interval) (time_secs_t)(((time_conv_t)(interval) + (time_conv_t)CH_CFG_ST_FREQUENCY - (time_conv_t)1) / (time_conv_t)CH_CFG_ST_FREQUENCY)
#define TIME_I2MS(interval) (time_msecs_t)((((time_conv_t)(interval) * (time_conv_t)1000) + (time_conv_t)CH_CFG_ST_FREQUENCY - (time_conv_t)1) / (time_conv_t)CH_CFG_ST_FREQUENCY)
#define TIME_I2US(interval) (time_usecs_t)((((time_conv_t)(interval) * (time_conv_t)1000000) + (time_conv_t)CH_CFG_ST_FREQUENCY - (time_conv_t)1) / (time_conv_t)CH_CFG_ST_FREQUENCY)
static inline sysinterval_t chTimeS2I(time_secs_t secs) {
  time_conv_t ticks;
  ticks = (time_conv_t)secs * (time_conv_t)CH_CFG_ST_FREQUENCY;
  chDbgAssert(ticks <= (time_conv_t)TIME_MAX_INTERVAL,
              "conversion overflow");
  return (sysinterval_t)ticks;
}
static inline sysinterval_t chTimeMS2I(time_msecs_t msec) {
  time_conv_t ticks;
  ticks = (((time_conv_t)msec * (time_conv_t)CH_CFG_ST_FREQUENCY) +
           (time_conv_t)999) / (time_conv_t)1000;
  chDbgAssert(ticks <= (time_conv_t)TIME_MAX_INTERVAL,
              "conversion overflow");
  return (sysinterval_t)ticks;
}
static inline sysinterval_t chTimeUS2I(time_usecs_t usec) {
  time_conv_t ticks;
  ticks = (((time_conv_t)usec * (time_conv_t)CH_CFG_ST_FREQUENCY) +
           (time_conv_t)999999) / (time_conv_t)1000000;
  chDbgAssert(ticks <= (time_conv_t)TIME_MAX_INTERVAL,
              "conversion overflow");
  return (sysinterval_t)ticks;
}
static inline time_secs_t chTimeI2S(sysinterval_t interval) {
  time_conv_t secs;
  secs = ((time_conv_t)interval +
          (time_conv_t)CH_CFG_ST_FREQUENCY -
          (time_conv_t)1) / (time_conv_t)CH_CFG_ST_FREQUENCY;
  chDbgAssert(secs < (time_conv_t)((time_secs_t)-1),
              "conversion overflow");
  return (time_secs_t)secs;
}
static inline time_msecs_t chTimeI2MS(sysinterval_t interval) {
  time_conv_t msecs;
  msecs = (((time_conv_t)interval * (time_conv_t)1000) +
           (time_conv_t)CH_CFG_ST_FREQUENCY - (time_conv_t)1) /
          (time_conv_t)CH_CFG_ST_FREQUENCY;
  chDbgAssert(msecs < (time_conv_t)((time_msecs_t)-1),
              "conversion overflow");
  return (time_msecs_t)msecs;
}
static inline time_usecs_t chTimeI2US(sysinterval_t interval) {
  time_conv_t usecs;
  usecs = (((time_conv_t)interval * (time_conv_t)1000000) +
           (time_conv_t)CH_CFG_ST_FREQUENCY - (time_conv_t)1) /
          (time_conv_t)CH_CFG_ST_FREQUENCY;
  chDbgAssert(usecs <= (time_conv_t)((time_usecs_t)-1),
              "conversion overflow");
  return (time_usecs_t)usecs;
}
static inline systime_t chTimeAddX(systime_t systime,
                                   sysinterval_t interval) {
  return systime + (systime_t)interval;
}
static inline sysinterval_t chTimeDiffX(systime_t start, systime_t end) {
  return (sysinterval_t)((systime_t)(end - start));
}
static inline bool chTimeIsInRangeX(systime_t time,
                                    systime_t start,
                                    systime_t end) {
  return (bool)((systime_t)((systime_t)time - (systime_t)start) <
                (systime_t)((systime_t)end - (systime_t)start));
}
static inline systimestamp_t chTimeStampAddX(systimestamp_t stamp,
                                             sysinterval_t interval) {
  return stamp + (systimestamp_t)interval;
}
static inline sysinterval_t chTimeStampDiffX(systimestamp_t start,
                                             systimestamp_t end) {
  systimestamp_t diff;
  diff = end - start;
  chDbgAssert(diff <= (systimestamp_t)((sysinterval_t)-1),
              "conversion overflow");
  return (sysinterval_t)diff;
}
static inline bool chTimeStampIsInRangeX(systimestamp_t stamp,
                                         systimestamp_t start,
                                         systimestamp_t end) {
  return (bool)((systimestamp_t)((systimestamp_t)stamp - (systimestamp_t)start) <
                (systimestamp_t)((systimestamp_t)end - (systimestamp_t)start));
}
#define CHLISTS_H 
typedef struct ch_list ch_list_t;
struct ch_list {
  ch_list_t *next;
};
typedef struct ch_queue ch_queue_t;
struct ch_queue {
  ch_queue_t *next;
  ch_queue_t *prev;
};
typedef struct ch_priority_queue ch_priority_queue_t;
struct ch_priority_queue {
  ch_priority_queue_t *next;
  ch_priority_queue_t *prev;
  tprio_t prio;
};
typedef struct ch_delta_list ch_delta_list_t;
struct ch_delta_list {
  ch_delta_list_t *next;
  ch_delta_list_t *prev;
  sysinterval_t delta;
};
#define __CH_QUEUE_DATA(name) {(ch_queue_t *)&name, (ch_queue_t *)&name}
#define CH_QUEUE_DECL(name) ch_queue_t name = __CH_QUEUE_DATA(name)
static inline void ch_list_init(ch_list_t *lp) {
  lp->next = lp;
}
static inline bool ch_list_isempty(ch_list_t *lp) {
  return (bool)(lp->next == lp);
}
static inline bool ch_list_notempty(ch_list_t *lp) {
  return (bool)(lp->next != lp);
}
static inline void ch_list_link(ch_list_t *lp, ch_list_t *p) {
  p->next = lp->next;
  lp->next = p;
}
static inline ch_list_t *ch_list_unlink(ch_list_t *lp) {
  ch_list_t *p = lp->next;
  lp->next = p->next;
  return p;
}
static inline void ch_queue_init(ch_queue_t *qp) {
  qp->next = qp;
  qp->prev = qp;
}
static inline bool ch_queue_isempty(const ch_queue_t *qp) {
  return (bool)(qp->next == qp);
}
static inline bool ch_queue_notempty(const ch_queue_t *qp) {
  return (bool)(qp->next != qp);
}
static inline void ch_queue_insert(ch_queue_t *qp, ch_queue_t *p) {
  p->next = qp;
  p->prev = qp->prev;
  p->prev->next = p;
  qp->prev = p;
}
static inline ch_queue_t *ch_queue_fifo_remove(ch_queue_t *qp) {
  ch_queue_t *p = qp->next;
  qp->next = p->next;
  qp->next->prev = qp;
  return p;
}
static inline ch_queue_t *ch_queue_lifo_remove(ch_queue_t *qp) {
  ch_queue_t *p = qp->prev;
  qp->prev = p->prev;
  qp->prev->next = qp;
  return p;
}
static inline ch_queue_t *ch_queue_dequeue(ch_queue_t *p) {
  p->prev->next = p->next;
  p->next->prev = p->prev;
  return p;
}
static inline void ch_pqueue_init(ch_priority_queue_t *pqp) {
  pqp->next = pqp;
  pqp->prev = pqp;
  pqp->prio = (tprio_t)0;
}
static inline ch_priority_queue_t *ch_pqueue_remove_highest(ch_priority_queue_t *pqp) {
  ch_priority_queue_t *p = pqp->next;
  pqp->next = p->next;
  pqp->next->prev = pqp;
  return p;
}
static inline ch_priority_queue_t *ch_pqueue_insert_behind(ch_priority_queue_t *pqp,
                                                           ch_priority_queue_t *p) {
  do {
    pqp = pqp->next;
  } while (unlikely(pqp->prio >= p->prio));
  p->next = pqp;
  p->prev = pqp->prev;
  p->prev->next = p;
  pqp->prev = p;
  return p;
}
static inline ch_priority_queue_t *ch_pqueue_insert_ahead(ch_priority_queue_t *pqp,
                                                          ch_priority_queue_t *p) {
  do {
    pqp = pqp->next;
  } while (unlikely(pqp->prio > p->prio));
  p->next = pqp;
  p->prev = pqp->prev;
  p->prev->next = p;
  pqp->prev = p;
  return p;
}
static inline void ch_dlist_init(ch_delta_list_t *dlhp) {
  dlhp->next = dlhp;
  dlhp->prev = dlhp;
  dlhp->delta = (sysinterval_t)-1;
}
static inline bool ch_dlist_isempty(ch_delta_list_t *dlhp) {
  return (bool)(dlhp == dlhp->next);
}
static inline bool ch_dlist_notempty(ch_delta_list_t *dlhp) {
  return (bool)(dlhp != dlhp->next);
}
static inline bool ch_dlist_islast(ch_delta_list_t *dlhp,
                                   ch_delta_list_t *dlp) {
  return (bool)(dlp->next == dlhp);
}
static inline bool ch_dlist_isfirst(ch_delta_list_t *dlhp,
                                    ch_delta_list_t *dlp) {
  return (bool)(dlhp->next == dlp);
}
static inline void ch_dlist_insert_after(ch_delta_list_t *dlhp,
                                         ch_delta_list_t *dlp,
                                         sysinterval_t delta) {
  dlp->delta = delta;
  dlp->prev = dlhp;
  dlp->next = dlp->prev->next;
  dlp->next->prev = dlp;
  dlhp->next = dlp;
}
static inline void ch_dlist_insert_before(ch_delta_list_t *dlhp,
                                          ch_delta_list_t *dlp,
                                          sysinterval_t delta) {
  dlp->delta = delta;
  dlp->next = dlhp;
  dlp->prev = dlp->next->prev;
  dlp->prev->next = dlp;
  dlhp->prev = dlp;
}
static inline void ch_dlist_insert(ch_delta_list_t *dlhp,
                                   ch_delta_list_t *dlep,
                                   sysinterval_t delta) {
  ch_delta_list_t *dlp;
  dlp = dlhp->next;
  while (likely(dlp->delta < delta)) {
    chDbgAssert(dlp != dlep, "element already in list");
    delta -= dlp->delta;
    dlp = dlp->next;
  }
  ch_dlist_insert_before(dlp, dlep, delta);
  dlp->delta -= delta;
  dlhp->delta = (sysinterval_t)-1;
}
static inline ch_delta_list_t *ch_dlist_remove_first(ch_delta_list_t *dlhp) {
  ch_delta_list_t *dlp = dlhp->next;
  dlhp->next = dlp->next;
  dlhp->next->prev = dlhp;
  return dlp;
}
static inline ch_delta_list_t *ch_dlist_dequeue(ch_delta_list_t *dlp) {
  dlp->prev->next = dlp->next;
  dlp->next->prev = dlp->prev;
  return dlp;
}
#define CHALIGN_H 
#define MEM_ALIGN_MASK(a) ((size_t)(a) - 1U)
#define MEM_ALIGN_PREV(p,a) ((size_t)(p) & ~MEM_ALIGN_MASK(a))
#define MEM_ALIGN_NEXT(p,a) MEM_ALIGN_PREV((size_t)(p) + MEM_ALIGN_MASK(a), (a))
#define MEM_IS_ALIGNED(p,a) (((size_t)(p) & MEM_ALIGN_MASK(a)) == 0U)
#define MEM_IS_VALID_ALIGNMENT(a) (((size_t)(a) != 0U) && (((size_t)(a) & ((size_t)(a) - 1U)) == 0U))
#define CHTRACE_H 
#define CH_TRACE_TYPE_UNUSED 0U
#define CH_TRACE_TYPE_READY 1U
#define CH_TRACE_TYPE_SWITCH 2U
#define CH_TRACE_TYPE_ISR_ENTER 3U
#define CH_TRACE_TYPE_ISR_LEAVE 4U
#define CH_TRACE_TYPE_HALT 5U
#define CH_TRACE_TYPE_USER 6U
#define CH_DBG_TRACE_MASK_DISABLED 255U
#define CH_DBG_TRACE_MASK_NONE 0U
#define CH_DBG_TRACE_MASK_READY 1U
#define CH_DBG_TRACE_MASK_SWITCH 2U
#define CH_DBG_TRACE_MASK_ISR 4U
#define CH_DBG_TRACE_MASK_HALT 8U
#define CH_DBG_TRACE_MASK_USER 16U
#define CH_DBG_TRACE_MASK_SLOW (CH_DBG_TRACE_MASK_READY | CH_DBG_TRACE_MASK_SWITCH | CH_DBG_TRACE_MASK_HALT | CH_DBG_TRACE_MASK_USER)
#define CH_DBG_TRACE_MASK_ALL (CH_DBG_TRACE_MASK_READY | CH_DBG_TRACE_MASK_SWITCH | CH_DBG_TRACE_MASK_ISR | CH_DBG_TRACE_MASK_HALT | CH_DBG_TRACE_MASK_USER)
#define __trace_ready(tp,msg) 
#define __trace_switch(ntp,otp) 
#define __trace_isr_enter(isr) 
#define __trace_isr_leave(isr) 
#define __trace_halt(reason) 
#define chDbgWriteTraceI(up1,up2) 
#define chDbgWriteTrace(up1,up2) 
#define CHPORT_H 
#define CHCORE_H 
#define MPU_H 
#define MPU_TYPE_SEPARATED (1U << 0U)
#define MPU_TYPE_DREGION(n) (((n) >> 8U) & 255U)
#define MPU_TYPE_IREGION(n) (((n) >> 16U) & 255U)
#define MPU_CTRL_ENABLE (1U << 0U)
#define MPU_CTRL_HFNMIENA (1U << 1U)
#define MPU_CTRL_PRIVDEFENA (1U << 2U)
#define MPU_RNR_REGION_MASK (255U << 0U)
#define MPU_RNR_REGION(n) ((n) << 0U)
#define MPU_RBAR_REGION_MASK (15U << 0U)
#define MPU_RBAR_REGION(n) ((n) << 0U)
#define MPU_RBAR_VALID (1U << 4U)
#define MPU_RBAR_ADDR_MASK 0xFFFFFFE0U
#define MPU_RBAR_ADDR(n) ((n) << 5U)
#define MPU_RASR_ENABLE (1U << 0U)
#define MPU_RASR_SIZE_MASK (31U << 1U)
#define MPU_RASR_SIZE(n) ((n) << 1U)
#define MPU_RASR_SIZE_32 MPU_RASR_SIZE(4U)
#define MPU_RASR_SIZE_64 MPU_RASR_SIZE(5U)
#define MPU_RASR_SIZE_128 MPU_RASR_SIZE(6U)
#define MPU_RASR_SIZE_256 MPU_RASR_SIZE(7U)
#define MPU_RASR_SIZE_512 MPU_RASR_SIZE(8U)
#define MPU_RASR_SIZE_1K MPU_RASR_SIZE(9U)
#define MPU_RASR_SIZE_2K MPU_RASR_SIZE(10U)
#define MPU_RASR_SIZE_4K MPU_RASR_SIZE(11U)
#define MPU_RASR_SIZE_8K MPU_RASR_SIZE(12U)
#define MPU_RASR_SIZE_16K MPU_RASR_SIZE(13U)
#define MPU_RASR_SIZE_32K MPU_RASR_SIZE(14U)
#define MPU_RASR_SIZE_64K MPU_RASR_SIZE(15U)
#define MPU_RASR_SIZE_128K MPU_RASR_SIZE(16U)
#define MPU_RASR_SIZE_256K MPU_RASR_SIZE(17U)
#define MPU_RASR_SIZE_512K MPU_RASR_SIZE(18U)
#define MPU_RASR_SIZE_1M MPU_RASR_SIZE(19U)
#define MPU_RASR_SIZE_2M MPU_RASR_SIZE(20U)
#define MPU_RASR_SIZE_4M MPU_RASR_SIZE(21U)
#define MPU_RASR_SIZE_8M MPU_RASR_SIZE(22U)
#define MPU_RASR_SIZE_16M MPU_RASR_SIZE(23U)
#define MPU_RASR_SIZE_32M MPU_RASR_SIZE(24U)
#define MPU_RASR_SIZE_64M MPU_RASR_SIZE(25U)
#define MPU_RASR_SIZE_128M MPU_RASR_SIZE(26U)
#define MPU_RASR_SIZE_256M MPU_RASR_SIZE(27U)
#define MPU_RASR_SIZE_512M MPU_RASR_SIZE(28U)
#define MPU_RASR_SIZE_1G MPU_RASR_SIZE(29U)
#define MPU_RASR_SIZE_2G MPU_RASR_SIZE(30U)
#define MPU_RASR_SIZE_4G MPU_RASR_SIZE(31U)
#define MPU_RASR_SRD_MASK (255U << 8U)
#define MPU_RASR_SRD(n) ((n) << 8U)
#define MPU_RASR_SRD_ALL (0U << 8U)
#define MPU_RASR_SRD_DISABLE_SUB0 (1U << 8U)
#define MPU_RASR_SRD_DISABLE_SUB1 (2U << 8U)
#define MPU_RASR_SRD_DISABLE_SUB2 (4U << 8U)
#define MPU_RASR_SRD_DISABLE_SUB3 (8U << 8U)
#define MPU_RASR_SRD_DISABLE_SUB4 (16U << 8U)
#define MPU_RASR_SRD_DISABLE_SUB5 (32U << 8U)
#define MPU_RASR_SRD_DISABLE_SUB6 (64U << 8U)
#define MPU_RASR_SRD_DISABLE_SUB7 (128U << 8U)
#define MPU_RASR_ATTR_B (1U << 16U)
#define MPU_RASR_ATTR_C (1U << 17U)
#define MPU_RASR_ATTR_S (1U << 18U)
#define MPU_RASR_ATTR_TEX_MASK (7U << 19U)
#define MPU_RASR_ATTR_TEX(n) ((n) << 19U)
#define MPU_RASR_ATTR_AP_MASK (7U << 24U)
#define MPU_RASR_ATTR_AP(n) ((n) << 24U)
#define MPU_RASR_ATTR_AP_NA_NA (0U << 24U)
#define MPU_RASR_ATTR_AP_RW_NA (1U << 24U)
#define MPU_RASR_ATTR_AP_RW_RO (2U << 24U)
#define MPU_RASR_ATTR_AP_RW_RW (3U << 24U)
#define MPU_RASR_ATTR_AP_RO_NA (5U << 24U)
#define MPU_RASR_ATTR_AP_RO_RO (6U << 24U)
#define MPU_RASR_ATTR_XN (1U << 28U)
#define MPU_RASR_ATTR_STRONGLY_ORDERED (MPU_RASR_ATTR_TEX(0))
#define MPU_RASR_ATTR_SHARED_DEVICE (MPU_RASR_ATTR_TEX(0) | MPU_RASR_ATTR_B)
#define MPU_RASR_ATTR_CACHEABLE_WT_NWA (MPU_RASR_ATTR_TEX(0) | MPU_RASR_ATTR_C)
#define MPU_RASR_ATTR_CACHEABLE_WB_NWA (MPU_RASR_ATTR_TEX(0) | MPU_RASR_ATTR_B | MPU_RASR_ATTR_C)
#define MPU_RASR_ATTR_NON_CACHEABLE (MPU_RASR_ATTR_TEX(1))
#define MPU_RASR_ATTR_CACHEABLE_WB_WA (MPU_RASR_ATTR_TEX(1) | MPU_RASR_ATTR_B | MPU_RASR_ATTR_C)
#define MPU_RASR_ATTR_NON_SHARED_DEVICE (MPU_RASR_ATTR_TEX(2))
#define MPU_REGION_0 0U
#define MPU_REGION_1 1U
#define MPU_REGION_2 2U
#define MPU_REGION_3 3U
#define MPU_REGION_4 4U
#define MPU_REGION_5 5U
#define MPU_REGION_6 6U
#define MPU_REGION_7 7U
#define mpuEnable(ctrl) { MPU->CTRL = ((uint32_t)ctrl) | MPU_CTRL_ENABLE; SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk; }
#define mpuDisable() { SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk; MPU->CTRL = 0; }
#define mpuConfigureRegion(region,addr,attribs) { MPU->RNR = ((uint32_t)region); MPU->RBAR = ((uint32_t)addr); MPU->RASR = ((uint32_t)attribs); }
#define mpuSetRegionAddress(region,addr) { MPU->RNR = ((uint32_t)region); MPU->RBAR = ((uint32_t)addr); }
#define PORT_COMPILER_NAME "GCC " __VERSION__
#define CORTEX_BASEPRI_DISABLED 0U
#define CORTEX_PRIORITY_LEVELS (1U << CORTEX_PRIORITY_BITS)
#define CORTEX_MINIMUM_PRIORITY (CORTEX_PRIORITY_LEVELS - 1U)
#define CORTEX_MAXIMUM_PRIORITY 0U
#define CORTEX_PRIORITY_SVCALL (CORTEX_MAXIMUM_PRIORITY + CORTEX_FAST_PRIORITIES)
#define CORTEX_PRIORITY_PENDSV CORTEX_MAX_KERNEL_PRIORITY
#define CORTEX_PRIO_MASK(n) ((n) << (8 - CORTEX_PRIORITY_BITS))
#define PORT_USE_SYSCALL FALSE
#define PORT_SWITCHED_REGIONS_NUMBER 0
#define PORT_ENABLE_GUARD_PAGES FALSE
#define PORT_USE_GUARD_MPU_REGION MPU_REGION_7
#define PORT_IDLE_THREAD_STACK_SIZE 64
#define PORT_INT_REQUIRED_STACK 64
#define CORTEX_ENABLE_WFI_IDLE FALSE
#define CORTEX_FAST_PRIORITIES 2U
#define CORTEX_SIMPLIFIED_PRIORITY FALSE
#define CORTEX_PRIGROUP_INIT (7 - CORTEX_PRIORITY_BITS)
#define PORT_SUPPORTS_RT TRUE
#define PORT_NATURAL_ALIGN sizeof (void *)
#define PORT_STACK_ALIGN sizeof (stkalign_t)
#define PORT_WORKING_AREA_ALIGN ((PORT_ENABLE_GUARD_PAGES == TRUE) ? 32U : PORT_STACK_ALIGN)
#define PORT_ARCHITECTURE_ARM 
  #define PORT_ARCHITECTURE_ARM_v7ME
  #define PORT_ARCHITECTURE_NAME "ARMv7E-M"
            #define PORT_CORE_VARIANT_NAME "Cortex-M4"
  #define PORT_INFO "Advanced kernel mode"
  #define CORTEX_MAX_KERNEL_PRIORITY (CORTEX_PRIORITY_SVCALL + 1U)
  #define CORTEX_MIN_KERNEL_PRIORITY (CORTEX_PRIORITY_LEVELS - 1U)
  #define CORTEX_BASEPRI_KERNEL CORTEX_PRIO_MASK(CORTEX_MAX_KERNEL_PRIORITY)
  #define PORT_GUARD_PAGE_SIZE 0U
struct port_extctx {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr_thd;
  uint32_t pc;
  uint32_t xpsr;
};
struct port_intctx {
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t lr;
};
struct port_context {
  struct port_intctx *sp;
};
#define PORT_IRQ_IS_VALID_PRIORITY(n) (((n) >= 0U) && ((n) < CORTEX_PRIORITY_LEVELS))
#define PORT_IRQ_IS_VALID_KERNEL_PRIORITY(n) (((n) >= CORTEX_MAX_KERNEL_PRIORITY) && ((n) <= CORTEX_MIN_KERNEL_PRIORITY))
#define PORT_THD_FUNCTION(tname,arg) void tname(void *arg)
  #define __PORT_SETUP_CONTEXT_SYSCALL(tp,wtop)
  #define __PORT_SETUP_CONTEXT_MPU(tp)
#define PORT_SETUP_CONTEXT(tp,wbase,wtop,pf,arg) do { (tp)->ctx.sp = (struct port_intctx *)(void *) ((uint8_t *)(wtop) - sizeof (struct port_intctx)); (tp)->ctx.sp->r4 = (uint32_t)(pf); (tp)->ctx.sp->r5 = (uint32_t)(arg); (tp)->ctx.sp->lr = (uint32_t)__port_thread_start; __PORT_SETUP_CONTEXT_MPU(tp); __PORT_SETUP_CONTEXT_SYSCALL(tp, wtop); } while (false)
#define PORT_WA_CTX_SIZE (sizeof (struct port_intctx) + sizeof (struct port_extctx) + sizeof (struct port_extctx))
#define PORT_WA_SIZE(n) ((size_t)PORT_GUARD_PAGE_SIZE + (size_t)PORT_WA_CTX_SIZE + (size_t)(n) + (size_t)PORT_INT_REQUIRED_STACK)
  #define PORT_WORKING_AREA(s,n) stkalign_t s[THD_WORKING_AREA_SIZE(n) / sizeof (stkalign_t)]
#define PORT_IRQ_PROLOGUE() 
#define PORT_IRQ_EPILOGUE() __port_irq_epilogue()
  #define PORT_IRQ_HANDLER(id) void id(void)
  #define PORT_FAST_IRQ_HANDLER(id) void id(void)
  #define port_switch(ntp,otp) __port_switch(ntp, otp)
#define port_get_lock_status() __port_get_irq_status()
#define port_is_locked(sts) !__port_irq_enabled(sts)
  void port_init(os_instance_t *oip);
  void __port_irq_epilogue(void);
  void __port_switch(thread_t *ntp, thread_t *otp);
  void __port_thread_start(void);
  void __port_switch_from_isr(void);
  void __port_exit_from_isr(void);
__STATIC_FORCEINLINE syssts_t __port_get_irq_status(void) {
  syssts_t sts;
  sts = (syssts_t)__get_BASEPRI();
  return sts;
}
__STATIC_FORCEINLINE bool __port_irq_enabled(syssts_t sts) {
  return sts == (syssts_t)CORTEX_BASEPRI_DISABLED;
}
__STATIC_FORCEINLINE bool port_is_isr_context(void) {
  return (bool)((__get_IPSR() & 0x1FFU) != 0U);
}
__STATIC_FORCEINLINE void port_lock(void) {
  __set_BASEPRI(CORTEX_BASEPRI_KERNEL);
}
__STATIC_FORCEINLINE void port_unlock(void) {
  __set_BASEPRI(CORTEX_BASEPRI_DISABLED);
}
__STATIC_FORCEINLINE void port_lock_from_isr(void) {
  port_lock();
}
__STATIC_FORCEINLINE void port_unlock_from_isr(void) {
  port_unlock();
}
__STATIC_FORCEINLINE void port_disable(void) {
  __disable_irq();
}
__STATIC_FORCEINLINE void port_suspend(void) {
  __set_BASEPRI(CORTEX_BASEPRI_KERNEL);
  __enable_irq();
}
__STATIC_FORCEINLINE void port_enable(void) {
  __set_BASEPRI(CORTEX_BASEPRI_DISABLED);
  __enable_irq();
}
__STATIC_FORCEINLINE void port_wait_for_interrupt(void) {
}
__STATIC_FORCEINLINE rtcnt_t port_rt_get_counter_value(void) {
  return DWT->CYCCNT;
}
#define PORT_CORES_NUMBER 1
#define CH_PORT_SUPPORTS_RECURSIVE_LOCKS TRUE
#define CHTM_H 
#define TM_CALIBRATION_LOOP 4U
typedef struct {
  rtcnt_t offset;
} tm_calibration_t;
typedef struct {
  rtcnt_t best;
  rtcnt_t worst;
  rtcnt_t last;
  ucnt_t n;
  rttime_t cumulative;
} time_measurement_t;
  void chTMObjectInit(time_measurement_t *tmp);
  NOINLINE void chTMStartMeasurementX(time_measurement_t *tmp);
  NOINLINE void chTMStopMeasurementX(time_measurement_t *tmp);
  NOINLINE void chTMChainMeasurementToX(time_measurement_t *tmp1,
                                        time_measurement_t *tmp2);
static inline void __tm_calibration_object_init(tm_calibration_t *tcp) {
  unsigned i;
  time_measurement_t tm;
  tcp->offset = (rtcnt_t)0;
  chTMObjectInit(&tm);
  i = TM_CALIBRATION_LOOP;
  do {
    chTMStartMeasurementX(&tm);
    chTMStopMeasurementX(&tm);
    i--;
  } while (i > 0U);
  tcp->offset = tm.best;
}
#define CHSTATS_H 
#define __stats_increase_irq() 
#define __stats_ctxswc(old,new) 
#define __stats_start_measure_crit_thd() 
#define __stats_stop_measure_crit_thd() 
#define __stats_start_measure_crit_isr() 
#define __stats_stop_measure_crit_isr() 
#define CHOBJECTS_H 
typedef enum {
  ch_sys_uninit = 0,
  ch_sys_initializing = 1,
  ch_sys_running = 2,
  ch_sys_halted = 3
} system_state_t;
typedef struct ch_virtual_timer virtual_timer_t;
typedef void (*vtfunc_t)(virtual_timer_t *vtp, void *p);
struct ch_virtual_timer {
  ch_delta_list_t dlist;
  vtfunc_t func;
  void *par;
  sysinterval_t reload;
};
typedef struct ch_virtual_timers_list {
  ch_delta_list_t dlist;
  volatile systime_t systime;
  volatile uint64_t laststamp;
} virtual_timers_list_t;
typedef struct ch_registry {
  ch_queue_t queue;
} registry_t;
typedef thread_t * thread_reference_t;
typedef struct ch_threads_queue {
  ch_queue_t queue;
} threads_queue_t;
struct ch_thread {
  union {
    ch_list_t list;
    ch_queue_t queue;
    ch_priority_queue_t pqueue;
  } hdr;
  struct port_context ctx;
  ch_queue_t rqueue;
  os_instance_t *owner;
  const char *name;
  stkalign_t *wabase;
  tstate_t state;
  tmode_t flags;
  trefs_t refs;
  union {
    msg_t rdymsg;
    msg_t exitcode;
    void *wtobjp;
    thread_reference_t *wttrp;
    struct ch_semaphore *wtsemp;
    struct ch_mutex *wtmtxp;
    eventmask_t ewmask;
  } u;
  ch_list_t waiting;
  ch_queue_t msgqueue;
  msg_t sentmsg;
  eventmask_t epending;
  struct ch_mutex *mtxlist;
  tprio_t realprio;
  void *mpool;
  CH_CFG_THREAD_EXTRA_FIELDS
};
typedef struct ch_ready_list {
  ch_priority_queue_t pqueue;
  thread_t *current;
} ready_list_t;
typedef struct ch_os_instance_config {
  const char *name;
  stkalign_t *mainthread_base;
  stkalign_t *mainthread_end;
  stkalign_t *idlethread_base;
  stkalign_t *idlethread_end;
} os_instance_config_t;
struct ch_os_instance {
  ready_list_t rlist;
  virtual_timers_list_t vtlist;
  registry_t reglist;
  core_id_t core_id;
  rfcu_t rfcu;
  const os_instance_config_t *config;
  thread_t mainthread;
  system_debug_t dbg;
  CH_CFG_OS_INSTANCE_EXTRA_FIELDS
};
typedef struct ch_system {
  system_state_t state;
  os_instance_t *instances[PORT_CORES_NUMBER];
  tm_calibration_t tmc;
  CH_CFG_SYSTEM_EXTRA_FIELDS
} ch_system_t;
#define CHSYS_H 
#define CH_INTEGRITY_RLIST 1U
#define CH_INTEGRITY_VTLIST 2U
#define CH_INTEGRITY_REGISTRY 4U
#define CH_INTEGRITY_PORT 8U
#define CH_SYS_CORE0_MEMORY 
#define CH_SYS_CORE1_MEMORY 
      #define currcore (&ch0)
#define CH_IRQ_IS_VALID_PRIORITY(prio) PORT_IRQ_IS_VALID_PRIORITY(prio)
#define CH_IRQ_IS_VALID_KERNEL_PRIORITY(prio) PORT_IRQ_IS_VALID_KERNEL_PRIORITY(prio)
#define CH_IRQ_PROLOGUE() PORT_IRQ_PROLOGUE(); CH_CFG_IRQ_PROLOGUE_HOOK(); __stats_increase_irq(); __trace_isr_enter(__func__); __dbg_check_enter_isr()
#define CH_IRQ_EPILOGUE() __dbg_check_leave_isr(); __trace_isr_leave(__func__); CH_CFG_IRQ_EPILOGUE_HOOK(); PORT_IRQ_EPILOGUE()
#define CH_IRQ_HANDLER(id) PORT_IRQ_HANDLER(id)
#define CH_FAST_IRQ_HANDLER(id) PORT_FAST_IRQ_HANDLER(id)
#define S2RTC(freq,sec) ((freq) * (sec))
#define MS2RTC(freq,msec) (rtcnt_t)((((freq) + 999UL) / 1000UL) * (msec))
#define US2RTC(freq,usec) (rtcnt_t)((((freq) + 999999UL) / 1000000UL) * (usec))
#define RTC2S(freq,n) ((((n) - 1UL) / (freq)) + 1UL)
#define RTC2MS(freq,n) ((((n) - 1UL) / ((freq) / 1000UL)) + 1UL)
#define RTC2US(freq,n) ((((n) - 1UL) / ((freq) / 1000000UL)) + 1UL)
#define chSysGetRealtimeCounterX() (rtcnt_t)port_rt_get_counter_value()
#define chSysSwitch(ntp,otp) { __trace_switch(ntp, otp); __stats_ctxswc(ntp, otp); CH_CFG_CONTEXT_SWITCH_HOOK(ntp, otp); port_switch(ntp, otp); }
extern ch_system_t ch_system;
extern const os_instance_config_t ch_core0_cfg;
extern os_instance_t ch0;
  void chSysWaitSystemState(system_state_t state);
  void chSysInit(void);
  thread_t *chSysGetIdleThreadX(void);
  bool chSysIntegrityCheckI(unsigned testmask);
  void chSysTimerHandlerI(void);
  syssts_t chSysGetStatusAndLockX(void);
  void chSysRestoreStatusX(syssts_t sts);
  bool chSysIsCounterWithinX(rtcnt_t cnt, rtcnt_t start, rtcnt_t end);
  void chSysPolledDelayX(rtcnt_t cycles);
static inline void chSysDisable(void) {
  port_disable();
  __dbg_check_disable();
}
static inline void chSysSuspend(void) {
  port_suspend();
  __dbg_check_suspend();
}
static inline void chSysEnable(void) {
  __dbg_check_enable();
  port_enable();
}
static inline void chSysLock(void) {
  port_lock();
  __stats_start_measure_crit_thd();
  __dbg_check_lock();
}
static inline void chSysUnlock(void) {
  __dbg_check_unlock();
  __stats_stop_measure_crit_thd();
  chDbgAssert((currcore->rlist.pqueue.next == &currcore->rlist.pqueue) ||
              (currcore->rlist.current->hdr.pqueue.prio >= currcore->rlist.pqueue.next->prio),
              "priority order violation");
  port_unlock();
}
static inline void chSysLockFromISR(void) {
  port_lock_from_isr();
  __stats_start_measure_crit_isr();
  __dbg_check_lock_from_isr();
}
static inline void chSysUnlockFromISR(void) {
  __dbg_check_unlock_from_isr();
  __stats_stop_measure_crit_isr();
  port_unlock_from_isr();
}
static inline void chSysUnconditionalLock(void) {
  if (!port_is_locked(port_get_lock_status())) {
    chSysLock();
  }
}
static inline void chSysUnconditionalUnlock(void) {
  if (port_is_locked(port_get_lock_status())) {
    chSysUnlock();
  }
}
#define CHINSTANCES_H 
#define __instance_get_currthread(oip) (oip)->rlist.current
#define __instance_set_currthread(oip,tp) (oip)->rlist.current = (tp)
  void chInstanceObjectInit(os_instance_t *oip,
                            const os_instance_config_t *oicp);
#define CHVT_H 
  void chVTDoSetI(virtual_timer_t *vtp, sysinterval_t delay,
                  vtfunc_t vtfunc, void *par);
  void chVTDoSetContinuousI(virtual_timer_t *vtp, sysinterval_t delay,
                            vtfunc_t vtfunc, void *par);
  void chVTDoResetI(virtual_timer_t *vtp);
  sysinterval_t chVTGetRemainingIntervalI(virtual_timer_t *vtp);
  void chVTDoTickI(void);
  systimestamp_t chVTGetTimeStampI(void);
  void chVTResetTimeStampI(void);
static inline void chVTObjectInit(virtual_timer_t *vtp) {
  vtp->dlist.next = NULL;
}
static inline systime_t chVTGetSystemTimeX(void) {
  return currcore->vtlist.systime;
}
static inline systime_t chVTGetSystemTime(void) {
  systime_t systime;
  chSysLock();
  systime = chVTGetSystemTimeX();
  chSysUnlock();
  return systime;
}
static inline sysinterval_t chVTTimeElapsedSinceX(systime_t start) {
  return chTimeDiffX(start, chVTGetSystemTimeX());
}
static inline bool chVTIsSystemTimeWithinX(systime_t start, systime_t end) {
  return chTimeIsInRangeX(chVTGetSystemTimeX(), start, end);
}
static inline bool chVTIsSystemTimeWithin(systime_t start, systime_t end) {
  return chTimeIsInRangeX(chVTGetSystemTime(), start, end);
}
static inline bool chVTGetTimersStateI(sysinterval_t *timep) {
  virtual_timers_list_t *vtlp = &currcore->vtlist;
  ch_delta_list_t *dlp = &vtlp->dlist;
  chDbgCheckClassI();
  if (dlp == dlp->next) {
    return false;
  }
  if (timep != NULL) {
    *timep = dlp->next->delta;
  }
  return true;
}
static inline bool chVTIsArmedI(const virtual_timer_t *vtp) {
  chDbgCheckClassI();
  return (bool)(vtp->dlist.next != NULL);
}
static inline bool chVTIsArmed(const virtual_timer_t *vtp) {
  bool b;
  chSysLock();
  b = chVTIsArmedI(vtp);
  chSysUnlock();
  return b;
}
static inline void chVTResetI(virtual_timer_t *vtp) {
  if (chVTIsArmedI(vtp)) {
    chVTDoResetI(vtp);
  }
}
static inline void chVTReset(virtual_timer_t *vtp) {
  chSysLock();
  chVTResetI(vtp);
  chSysUnlock();
}
static inline void chVTSetI(virtual_timer_t *vtp, sysinterval_t delay,
                            vtfunc_t vtfunc, void *par) {
  chVTResetI(vtp);
  chVTDoSetI(vtp, delay, vtfunc, par);
}
static inline void chVTSet(virtual_timer_t *vtp, sysinterval_t delay,
                           vtfunc_t vtfunc, void *par) {
  chSysLock();
  chVTSetI(vtp, delay, vtfunc, par);
  chSysUnlock();
}
static inline void chVTSetContinuousI(virtual_timer_t *vtp, sysinterval_t delay,
                                      vtfunc_t vtfunc, void *par) {
  chVTResetI(vtp);
  chVTDoSetContinuousI(vtp, delay, vtfunc, par);
}
static inline void chVTSetContinuous(virtual_timer_t *vtp, sysinterval_t delay,
                                     vtfunc_t vtfunc, void *par) {
  chSysLock();
  chVTSetContinuousI(vtp, delay, vtfunc, par);
  chSysUnlock();
}
static inline sysinterval_t chVTGetReloadIntervalX(virtual_timer_t *vtp) {
  return vtp->reload;
}
static inline void chVTSetReloadIntervalX(virtual_timer_t *vtp,
                                          sysinterval_t reload) {
  vtp->reload = reload;
}
static inline systimestamp_t chVTGetTimeStamp(void) {
  systimestamp_t stamp;
  chSysLock();
  stamp = chVTGetTimeStampI();
  chSysUnlock();
  return stamp;
}
static inline void chVTResetTimeStamp(void) {
  chSysLock();
  chVTResetTimeStampI();
  chSysUnlock();
}
static inline void __vt_object_init(virtual_timers_list_t *vtlp) {
  ch_dlist_init(&vtlp->dlist);
  vtlp->systime = (systime_t)0;
  vtlp->laststamp = (systimestamp_t)chVTGetSystemTimeX();
}
#define CHSCHD_H 
#define MSG_OK (msg_t)0
#define MSG_TIMEOUT (msg_t)-1
#define MSG_RESET (msg_t)-2
#define NOPRIO (tprio_t)0
#define IDLEPRIO (tprio_t)1
#define LOWPRIO (tprio_t)2
#define NORMALPRIO (tprio_t)128
#define HIGHPRIO (tprio_t)255
#define CH_STATE_READY (tstate_t)0
#define CH_STATE_CURRENT (tstate_t)1
#define CH_STATE_WTSTART (tstate_t)2
#define CH_STATE_SUSPENDED (tstate_t)3
#define CH_STATE_QUEUED (tstate_t)4
#define CH_STATE_WTSEM (tstate_t)5
#define CH_STATE_WTMTX (tstate_t)6
#define CH_STATE_WTCOND (tstate_t)7
#define CH_STATE_SLEEPING (tstate_t)8
#define CH_STATE_WTEXIT (tstate_t)9
#define CH_STATE_WTOREVT (tstate_t)10
#define CH_STATE_WTANDEVT (tstate_t)11
#define CH_STATE_SNDMSGQ (tstate_t)12
#define CH_STATE_SNDMSG (tstate_t)13
#define CH_STATE_WTMSG (tstate_t)14
#define CH_STATE_FINAL (tstate_t)15
#define CH_STATE_NAMES "READY", "CURRENT", "WTSTART", "SUSPENDED", "QUEUED", "WTSEM", "WTMTX", "WTCOND", "SLEEPING", "WTEXIT", "WTOREVT", "WTANDEVT", "SNDMSGQ", "SNDMSG", "WTMSG", "FINAL"
#define CH_FLAG_MODE_MASK (tmode_t)3U
#define CH_FLAG_MODE_STATIC (tmode_t)0U
#define CH_FLAG_MODE_HEAP (tmode_t)1U
#define CH_FLAG_MODE_MPOOL (tmode_t)2U
#define CH_FLAG_TERMINATE (tmode_t)4U
#define firstprio(rlp) ((rlp)->next->prio)
#define __sch_get_currthread() __instance_get_currthread(currcore)
  void chSchObjectInit(os_instance_t *oip,
                       const os_instance_config_t *oicp);
  thread_t *chSchReadyI(thread_t *tp);
  void chSchGoSleepS(tstate_t newstate);
  msg_t chSchGoSleepTimeoutS(tstate_t newstate, sysinterval_t timeout);
  void chSchWakeupS(thread_t *ntp, msg_t msg);
  void chSchRescheduleS(void);
  bool chSchIsPreemptionRequired(void);
  void chSchDoPreemption(void);
  void chSchPreemption(void);
  void chSchDoYieldS(void);
  thread_t *chSchSelectFirst(void);
static inline void ch_sch_prio_insert(ch_queue_t *qp, ch_queue_t *tp) {
  ch_queue_t *cp = qp;
  do {
    cp = cp->next;
  } while ((cp != qp) &&
           (threadref(cp)->hdr.pqueue.prio >= threadref(tp)->hdr.pqueue.prio));
  tp->next = cp;
  tp->prev = cp->prev;
  tp->prev->next = tp;
  cp->prev = tp;
}
#define CHTHREADS_H 
typedef void (*tfunc_t)(void *p);
typedef struct {
  const char *name;
  stkalign_t *wbase;
  stkalign_t *wend;
  tprio_t prio;
  tfunc_t funcp;
  void *arg;
} thread_descriptor_t;
#define __THREADS_QUEUE_DATA(name) {__CH_QUEUE_DATA(name)}
#define THREADS_QUEUE_DECL(name) threads_queue_t name = __THREADS_QUEUE_DATA(name)
#define THD_WORKING_AREA_SIZE(n) MEM_ALIGN_NEXT(sizeof(thread_t) + PORT_WA_SIZE(n), PORT_STACK_ALIGN)
#define THD_WORKING_AREA(s,n) PORT_WORKING_AREA(s, n)
#define THD_WORKING_AREA_BASE(s) ((stkalign_t *)(s))
#define THD_WORKING_AREA_END(s) (THD_WORKING_AREA_BASE(s) + (sizeof (s) / sizeof (stkalign_t)))
#define THD_FUNCTION(tname,arg) PORT_THD_FUNCTION(tname, arg)
#define THD_DESCRIPTOR(name,wbase,wend,prio,funcp,arg) { (name), (wbase), (wend), (prio), (funcp), (arg) }
#define THD_DESCRIPTOR_AFFINITY(name,wbase,wend,prio,funcp,arg,oip) { (name), (wbase), (wend), (prio), (funcp), (arg), (oip) }
#define chThdSleepSeconds(sec) chThdSleep(TIME_S2I(sec))
#define chThdSleepMilliseconds(msec) chThdSleep(TIME_MS2I(msec))
#define chThdSleepMicroseconds(usec) chThdSleep(TIME_US2I(usec))
   thread_t *__thd_object_init(os_instance_t *oip,
                               thread_t *tp,
                               const char *name,
                               tprio_t prio);
  thread_t *chThdCreateSuspendedI(const thread_descriptor_t *tdp);
  thread_t *chThdCreateSuspended(const thread_descriptor_t *tdp);
  thread_t *chThdCreateI(const thread_descriptor_t *tdp);
  thread_t *chThdCreate(const thread_descriptor_t *tdp);
  thread_t *chThdCreateStatic(void *wsp, size_t size,
                              tprio_t prio, tfunc_t pf, void *arg);
  thread_t *chThdStart(thread_t *tp);
  thread_t *chThdAddRef(thread_t *tp);
  void chThdRelease(thread_t *tp);
  void chThdExit(msg_t msg);
  void chThdExitS(msg_t msg);
  msg_t chThdWait(thread_t *tp);
  tprio_t chThdSetPriority(tprio_t newprio);
  void chThdTerminate(thread_t *tp);
  msg_t chThdSuspendS(thread_reference_t *trp);
  msg_t chThdSuspendTimeoutS(thread_reference_t *trp, sysinterval_t timeout);
  void chThdResumeI(thread_reference_t *trp, msg_t msg);
  void chThdResumeS(thread_reference_t *trp, msg_t msg);
  void chThdResume(thread_reference_t *trp, msg_t msg);
  msg_t chThdEnqueueTimeoutS(threads_queue_t *tqp, sysinterval_t timeout);
  void chThdDequeueNextI(threads_queue_t *tqp, msg_t msg);
  void chThdDequeueAllI(threads_queue_t *tqp, msg_t msg);
  void chThdSleep(sysinterval_t time);
  void chThdSleepUntil(systime_t time);
  systime_t chThdSleepUntilWindowed(systime_t prev, systime_t next);
  void chThdYield(void);
static inline thread_t *chThdGetSelfX(void) {
  return __sch_get_currthread();
}
static inline tprio_t chThdGetPriorityX(void) {
  return chThdGetSelfX()->hdr.pqueue.prio;
}
static inline stkalign_t *chThdGetWorkingAreaX(thread_t *tp) {
  return tp->wabase;
}
static inline bool chThdTerminatedX(thread_t *tp) {
  return (bool)(tp->state == CH_STATE_FINAL);
}
static inline bool chThdShouldTerminateX(void) {
  return (bool)((chThdGetSelfX()->flags & CH_FLAG_TERMINATE) != (tmode_t)0);
}
static inline thread_t *chThdStartI(thread_t *tp) {
  chDbgAssert(tp->state == CH_STATE_WTSTART, "wrong state");
  return chSchReadyI(tp);
}
static inline void chThdSleepS(sysinterval_t ticks) {
  chDbgCheck(ticks != TIME_IMMEDIATE);
  (void) chSchGoSleepTimeoutS(CH_STATE_SLEEPING, ticks);
}
static inline void chThdQueueObjectInit(threads_queue_t *tqp) {
  ch_queue_init(&tqp->queue);
}
static inline bool chThdQueueIsEmptyI(threads_queue_t *tqp) {
  chDbgCheckClassI();
  return ch_queue_isempty(&tqp->queue);
}
static inline void chThdDoDequeueNextI(threads_queue_t *tqp, msg_t msg) {
  thread_t *tp;
  chDbgAssert(ch_queue_notempty(&tqp->queue), "empty queue");
  tp = threadref(ch_queue_fifo_remove(&tqp->queue));
  chDbgAssert(tp->state == CH_STATE_QUEUED, "invalid state");
  tp->u.rdymsg = msg;
  (void) chSchReadyI(tp);
}
#define CHREGISTRY_H 
typedef struct {
  char identifier[4];
  uint8_t zero;
  uint8_t size;
  uint16_t version;
  uint8_t ptrsize;
  uint8_t timesize;
  uint8_t threadsize;
  uint8_t off_prio;
  uint8_t off_ctx;
  uint8_t off_newer;
  uint8_t off_older;
  uint8_t off_name;
  uint8_t off_stklimit;
  uint8_t off_state;
  uint8_t off_flags;
  uint8_t off_refs;
  uint8_t off_preempt;
  uint8_t off_time;
  uint8_t off_reserved[4];
  uint8_t intctxsize;
  uint8_t intervalsize;
  uint8_t instancesnum;
  uint8_t off_sys_state;
  uint8_t off_sys_instances;
  uint8_t off_sys_reglist;
  uint8_t off_sys_rfcu;
  uint8_t off_sys_reserved[4];
  uint8_t off_inst_rlist_current;
  uint8_t off_inst_rlist;
  uint8_t off_inst_vtlist;
  uint8_t off_inst_reglist;
  uint8_t off_inst_core_id;
  uint8_t off_inst_rfcu;
} chdebug_t;
#define REG_HEADER(oip) (&(oip)->reglist.queue)
#define REG_REMOVE(tp) (void) ch_queue_dequeue(&(tp)->rqueue)
#define REG_INSERT(oip,tp) ch_queue_insert(REG_HEADER(oip), &(tp)->rqueue)
  extern ROMCONST chdebug_t ch_debug;
  thread_t *chRegFirstThread(void);
  thread_t *chRegNextThread(thread_t *tp);
  thread_t *chRegFindThreadByName(const char *name);
  thread_t *chRegFindThreadByPointer(thread_t *tp);
  thread_t *chRegFindThreadByWorkingArea(stkalign_t *wa);
static inline void __reg_object_init(registry_t *rp) {
  ch_queue_init(&rp->queue);
}
static inline void chRegSetThreadName(const char *name) {
  __sch_get_currthread()->name = name;
}
static inline const char *chRegGetThreadNameX(thread_t *tp) {
  return tp->name;
}
static inline void chRegSetThreadNameX(thread_t *tp, const char *name) {
  tp->name = name;
}
#define CHSEM_H 
typedef struct ch_semaphore {
  ch_queue_t queue;
  cnt_t cnt;
} semaphore_t;
#define __SEMAPHORE_DATA(name,n) {__CH_QUEUE_DATA(name.queue), n}
#define SEMAPHORE_DECL(name,n) semaphore_t name = __SEMAPHORE_DATA(name, n)
  void chSemObjectInit(semaphore_t *sp, cnt_t n);
  void chSemResetWithMessage(semaphore_t *sp, cnt_t n, msg_t msg);
  void chSemResetWithMessageI(semaphore_t *sp, cnt_t n, msg_t msg);
  msg_t chSemWait(semaphore_t *sp);
  msg_t chSemWaitS(semaphore_t *sp);
  msg_t chSemWaitTimeout(semaphore_t *sp, sysinterval_t timeout);
  msg_t chSemWaitTimeoutS(semaphore_t *sp, sysinterval_t timeout);
  void chSemSignal(semaphore_t *sp);
  void chSemSignalI(semaphore_t *sp);
  void chSemAddCounterI(semaphore_t *sp, cnt_t n);
  msg_t chSemSignalWait(semaphore_t *sps, semaphore_t *spw);
static inline void chSemReset(semaphore_t *sp, cnt_t n) {
  chSemResetWithMessage(sp, n, MSG_RESET);
}
static inline void chSemResetI(semaphore_t *sp, cnt_t n) {
  chSemResetWithMessageI(sp, n, MSG_RESET);
}
static inline void chSemFastWaitI(semaphore_t *sp) {
  chDbgCheckClassI();
  sp->cnt--;
}
static inline void chSemFastSignalI(semaphore_t *sp) {
  chDbgCheckClassI();
  sp->cnt++;
}
static inline cnt_t chSemGetCounterI(const semaphore_t *sp) {
  chDbgCheckClassI();
  return sp->cnt;
}
#define CHMTX_H 
typedef struct ch_mutex mutex_t;
struct ch_mutex {
  ch_queue_t queue;
  thread_t *owner;
  mutex_t *next;
};
#define __MUTEX_DATA(name) {__CH_QUEUE_DATA(name.queue), NULL, NULL}
#define MUTEX_DECL(name) mutex_t name = __MUTEX_DATA(name)
  void chMtxObjectInit(mutex_t *mp);
  void chMtxLock(mutex_t *mp);
  void chMtxLockS(mutex_t *mp);
  bool chMtxTryLock(mutex_t *mp);
  bool chMtxTryLockS(mutex_t *mp);
  void chMtxUnlock(mutex_t *mp);
  void chMtxUnlockS(mutex_t *mp);
  void chMtxUnlockAll(void);
  void chMtxUnlockAllS(void);
static inline bool chMtxQueueNotEmptyS(mutex_t *mp) {
  chDbgCheckClassS();
  return ch_queue_notempty(&mp->queue);
}
static inline thread_t *chMtxGetOwnerI(mutex_t *mp) {
  chDbgCheckClassI();
  return mp->owner;
}
static inline mutex_t *chMtxGetNextMutexX(void) {
  return chThdGetSelfX()->mtxlist;
}
#define CHCOND_H 
typedef struct condition_variable {
  ch_queue_t queue;
} condition_variable_t;
#define __CONDVAR_DATA(name) {__CH_QUEUE_DATA(name.queue)}
#define CONDVAR_DECL(name) condition_variable_t name = __CONDVAR_DATA(name)
  void chCondObjectInit(condition_variable_t *cp);
  void chCondSignal(condition_variable_t *cp);
  void chCondSignalI(condition_variable_t *cp);
  void chCondBroadcast(condition_variable_t *cp);
  void chCondBroadcastI(condition_variable_t *cp);
  msg_t chCondWait(condition_variable_t *cp);
  msg_t chCondWaitS(condition_variable_t *cp);
  msg_t chCondWaitTimeout(condition_variable_t *cp, sysinterval_t timeout);
  msg_t chCondWaitTimeoutS(condition_variable_t *cp, sysinterval_t timeout);
#define CHEVENTS_H 
typedef struct event_listener event_listener_t;
struct event_listener {
  event_listener_t *next;
  thread_t *listener;
  eventmask_t events;
  eventflags_t flags;
  eventflags_t wflags;
};
typedef struct event_source {
  event_listener_t *next;
} event_source_t;
typedef void (*evhandler_t)(eventid_t id);
#define ALL_EVENTS ((eventmask_t)-1)
#define EVENT_MASK(eid) ((eventmask_t)1 << (eventmask_t)(eid))
#define __EVENTSOURCE_DATA(name) {(event_listener_t *)(&name)}
#define EVENTSOURCE_DECL(name) event_source_t name = __EVENTSOURCE_DATA(name)
  void chEvtRegisterMaskWithFlagsI(event_source_t *esp,
                                   event_listener_t *elp,
                                   eventmask_t events,
                                   eventflags_t wflags);
  void chEvtRegisterMaskWithFlags(event_source_t *esp,
                                  event_listener_t *elp,
                                  eventmask_t events,
                                  eventflags_t wflags);
  void chEvtUnregister(event_source_t *esp, event_listener_t *elp);
  eventmask_t chEvtGetAndClearEventsI(eventmask_t events);
  eventmask_t chEvtGetAndClearEvents(eventmask_t events);
  eventmask_t chEvtAddEvents(eventmask_t events);
  eventflags_t chEvtGetAndClearFlagsI(event_listener_t *elp);
  eventflags_t chEvtGetAndClearFlags(event_listener_t *elp);
  void chEvtSignal(thread_t *tp, eventmask_t events);
  void chEvtSignalI(thread_t *tp, eventmask_t events);
  void chEvtBroadcastFlags(event_source_t *esp, eventflags_t flags);
  void chEvtBroadcastFlagsI(event_source_t *esp, eventflags_t flags);
  void chEvtDispatch(const evhandler_t *handlers, eventmask_t events);
  eventmask_t chEvtWaitOne(eventmask_t events);
  eventmask_t chEvtWaitAny(eventmask_t events);
  eventmask_t chEvtWaitAll(eventmask_t events);
  eventmask_t chEvtWaitOneTimeout(eventmask_t events, sysinterval_t timeout);
  eventmask_t chEvtWaitAnyTimeout(eventmask_t events, sysinterval_t timeout);
  eventmask_t chEvtWaitAllTimeout(eventmask_t events, sysinterval_t timeout);
static inline void chEvtObjectInit(event_source_t *esp) {
  esp->next = (event_listener_t *)esp;
}
static inline void chEvtRegisterMask(event_source_t *esp,
                                     event_listener_t *elp,
                                     eventmask_t events) {
  chEvtRegisterMaskWithFlags(esp, elp, events, (eventflags_t)-1);
}
static inline void chEvtRegister(event_source_t *esp,
                                 event_listener_t *elp,
                                 eventid_t event) {
  chEvtRegisterMask(esp, elp, EVENT_MASK(event));
}
static inline bool chEvtIsListeningI(event_source_t *esp) {
  return (bool)(esp != (event_source_t *)esp->next);
}
static inline void chEvtBroadcast(event_source_t *esp) {
  chEvtBroadcastFlags(esp, (eventflags_t)0);
}
static inline void chEvtBroadcastI(event_source_t *esp) {
  chEvtBroadcastFlagsI(esp, (eventflags_t)0);
}
static inline eventmask_t chEvtAddEventsI(eventmask_t events) {
  return __sch_get_currthread()->epending |= events;
}
static inline eventmask_t chEvtGetEventsX(void) {
  return __sch_get_currthread()->epending;
}
#define CHMSG_H 
#define __ch_msg_insert(qp,tp) ch_queue_insert(qp, &tp->hdr.queue)
  msg_t chMsgSend(thread_t *tp, msg_t msg);
  thread_t *chMsgWaitS(void);
  thread_t *chMsgWaitTimeoutS(sysinterval_t timeout);
  thread_t *chMsgPollS(void);
  void chMsgRelease(thread_t *tp, msg_t msg);
static inline thread_t *chMsgWait(void) {
  thread_t *tp;
  chSysLock();
  tp = chMsgWaitS();
  chSysUnlock();
  return tp;
}
static inline thread_t *chMsgWaitTimeout(sysinterval_t timeout) {
  thread_t *tp;
  chSysLock();
  tp = chMsgWaitTimeoutS(timeout);
  chSysUnlock();
  return tp;
}
static inline thread_t *chMsgPoll(void) {
  thread_t *tp;
  chSysLock();
  tp = chMsgPollS();
  chSysUnlock();
  return tp;
}
static inline bool chMsgIsPendingI(thread_t *tp) {
  chDbgCheckClassI();
  return (bool)(tp->msgqueue.next != &tp->msgqueue);
}
static inline msg_t chMsgGet(thread_t *tp) {
  chDbgAssert(tp->state == CH_STATE_SNDMSG, "invalid state");
  return tp->sentmsg;
}
static inline void chMsgReleaseS(thread_t *tp, msg_t msg) {
  chDbgCheckClassS();
  chSchWakeupS(tp, msg);
}
#define CHLIB_H 
#define __CHIBIOS_OSLIB__ 
#define CH_OSLIB_STABLE 1
#define CH_OSLIB_VERSION "1.3.3"
#define CH_OSLIB_MAJOR 1
#define CH_OSLIB_MINOR 3
#define CH_OSLIB_PATCH 3
#define CHBSEM_H 
typedef struct ch_binary_semaphore {
  semaphore_t sem;
} binary_semaphore_t;
#define __BSEMAPHORE_DATA(name,taken) {__SEMAPHORE_DATA(name.sem, ((taken) ? 0 : 1))}
#define BSEMAPHORE_DECL(name,taken) binary_semaphore_t name = __BSEMAPHORE_DATA(name, taken)
static inline void chBSemObjectInit(binary_semaphore_t *bsp, bool taken) {
  chSemObjectInit(&bsp->sem, taken ? (cnt_t)0 : (cnt_t)1);
}
static inline msg_t chBSemWait(binary_semaphore_t *bsp) {
  return chSemWait(&bsp->sem);
}
static inline msg_t chBSemWaitS(binary_semaphore_t *bsp) {
  chDbgCheckClassS();
  return chSemWaitS(&bsp->sem);
}
static inline msg_t chBSemWaitTimeoutS(binary_semaphore_t *bsp,
                                       sysinterval_t timeout) {
  chDbgCheckClassS();
  return chSemWaitTimeoutS(&bsp->sem, timeout);
}
static inline msg_t chBSemWaitTimeout(binary_semaphore_t *bsp,
                                      sysinterval_t timeout) {
  return chSemWaitTimeout(&bsp->sem, timeout);
}
static inline void chBSemResetI(binary_semaphore_t *bsp, bool taken) {
  chDbgCheckClassI();
  chSemResetI(&bsp->sem, taken ? (cnt_t)0 : (cnt_t)1);
}
static inline void chBSemReset(binary_semaphore_t *bsp, bool taken) {
  chSemReset(&bsp->sem, taken ? (cnt_t)0 : (cnt_t)1);
}
static inline void chBSemSignalI(binary_semaphore_t *bsp) {
  chDbgCheckClassI();
  if (bsp->sem.cnt < (cnt_t)1) {
    chSemSignalI(&bsp->sem);
  }
}
static inline void chBSemSignal(binary_semaphore_t *bsp) {
  chSysLock();
  chBSemSignalI(bsp);
  chSchRescheduleS();
  chSysUnlock();
}
static inline bool chBSemGetStateI(const binary_semaphore_t *bsp) {
  chDbgCheckClassI();
  return (bsp->sem.cnt > (cnt_t)0) ? false : true;
}
#define CHMBOXES_H 
typedef struct {
  msg_t *buffer;
  msg_t *top;
  msg_t *wrptr;
  msg_t *rdptr;
  size_t cnt;
  bool reset;
  threads_queue_t qw;
  threads_queue_t qr;
} mailbox_t;
#define __MAILBOX_DATA(name,buffer,size) { (msg_t *)(buffer), (msg_t *)(buffer) + size, (msg_t *)(buffer), (msg_t *)(buffer), (size_t)0, false, __THREADS_QUEUE_DATA(name.qw), __THREADS_QUEUE_DATA(name.qr), }
#define MAILBOX_DECL(name,buffer,size) mailbox_t name = __MAILBOX_DATA(name, buffer, size)
  void chMBObjectInit(mailbox_t *mbp, msg_t *buf, size_t n);
  void chMBReset(mailbox_t *mbp);
  void chMBResetI(mailbox_t *mbp);
  msg_t chMBPostTimeout(mailbox_t *mbp, msg_t msg, sysinterval_t timeout);
  msg_t chMBPostTimeoutS(mailbox_t *mbp, msg_t msg, sysinterval_t timeout);
  msg_t chMBPostI(mailbox_t *mbp, msg_t msg);
  msg_t chMBPostAheadTimeout(mailbox_t *mbp, msg_t msg, sysinterval_t timeout);
  msg_t chMBPostAheadTimeoutS(mailbox_t *mbp, msg_t msg, sysinterval_t timeout);
  msg_t chMBPostAheadI(mailbox_t *mbp, msg_t msg);
  msg_t chMBFetchTimeout(mailbox_t *mbp, msg_t *msgp, sysinterval_t timeout);
  msg_t chMBFetchTimeoutS(mailbox_t *mbp, msg_t *msgp, sysinterval_t timeout);
  msg_t chMBFetchI(mailbox_t *mbp, msg_t *msgp);
static inline size_t chMBGetSizeI(const mailbox_t *mbp) {
  return (size_t)(mbp->top - mbp->buffer);
}
static inline size_t chMBGetUsedCountI(const mailbox_t *mbp) {
  chDbgCheckClassI();
  return mbp->cnt;
}
static inline size_t chMBGetFreeCountI(const mailbox_t *mbp) {
  chDbgCheckClassI();
  return chMBGetSizeI(mbp) - chMBGetUsedCountI(mbp);
}
static inline msg_t chMBPeekI(const mailbox_t *mbp) {
  chDbgCheckClassI();
  return *mbp->rdptr;
}
static inline void chMBResumeX(mailbox_t *mbp) {
  mbp->reset = false;
}
#define CHMEMCORE_H 
typedef void *(*memgetfunc_t)(size_t size, unsigned align);
typedef void *(*memgetfunc2_t)(size_t size, unsigned align, size_t offset);
typedef struct {
  uint8_t *basemem;
  uint8_t *topmem;
} memcore_t;
#define chCoreAllocAlignedWithOffsetI chCoreAllocFromTopI
#define chCoreAllocAlignedWithOffset chCoreAllocFromTop
extern memcore_t ch_memcore;
  void __core_init(void);
  void *chCoreAllocFromBaseI(size_t size, unsigned align, size_t offset);
  void *chCoreAllocFromTopI(size_t size, unsigned align, size_t offset);
  void *chCoreAllocFromBase(size_t size, unsigned align, size_t offset);
  void *chCoreAllocFromTop(size_t size, unsigned align, size_t offset);
  size_t chCoreGetStatusX(void);
static inline void *chCoreAllocAlignedI(size_t size, unsigned align) {
  return chCoreAllocAlignedWithOffsetI(size, align, 0U);
}
static inline void *chCoreAllocAligned(size_t size, unsigned align) {
  return chCoreAllocAlignedWithOffset(size, align, 0U);
}
static inline void *chCoreAllocI(size_t size) {
  return chCoreAllocAlignedWithOffsetI(size, PORT_NATURAL_ALIGN, 0U);
}
static inline void *chCoreAlloc(size_t size) {
  return chCoreAllocAlignedWithOffset(size, PORT_NATURAL_ALIGN, 0U);
}
#define CHMEMHEAPS_H 
#define CH_HEAP_ALIGNMENT 8U
typedef struct memory_heap memory_heap_t;
typedef union heap_header heap_header_t;
union heap_header {
  struct {
    heap_header_t *next;
    size_t pages;
  } free;
  struct {
    memory_heap_t *heap;
    size_t size;
  } used;
};
struct memory_heap {
  memgetfunc2_t provider;
  heap_header_t header;
  mutex_t mtx;
};
#define CH_HEAP_AREA(name,size) ALIGNED_VAR(CH_HEAP_ALIGNMENT) uint8_t name[MEM_ALIGN_NEXT((size), CH_HEAP_ALIGNMENT)]
  void __heap_init(void);
  void chHeapObjectInit(memory_heap_t *heapp, void *buf, size_t size);
  void *chHeapAllocAligned(memory_heap_t *heapp, size_t size, unsigned align);
  void chHeapFree(void *p);
  size_t chHeapStatus(memory_heap_t *heapp, size_t *totalp, size_t *largestp);
static inline void *chHeapAlloc(memory_heap_t *heapp, size_t size) {
  return chHeapAllocAligned(heapp, size, CH_HEAP_ALIGNMENT);
}
static inline size_t chHeapGetSize(const void *p) {
  return ((heap_header_t *)p - 1U)->used.size;
}
#define CHMEMPOOLS_H 
struct pool_header {
  struct pool_header *next;
};
typedef struct {
  struct pool_header *next;
  size_t object_size;
  unsigned align;
  memgetfunc_t provider;
} memory_pool_t;
typedef struct {
  semaphore_t sem;
  memory_pool_t pool;
} guarded_memory_pool_t;
#define __MEMORYPOOL_DATA(name,size,align,provider) {NULL, size, align, provider}
#define MEMORYPOOL_DECL(name,size,align,provider) memory_pool_t name = __MEMORYPOOL_DATA(name, size, align, provider)
#define __GUARDEDMEMORYPOOL_DATA(name,size,align) { __SEMAPHORE_DATA(name.sem, (cnt_t)0), __MEMORYPOOL_DATA(NULL, size, align, NULL) }
#define GUARDEDMEMORYPOOL_DECL(name,size,align) guarded_memory_pool_t name = __GUARDEDMEMORYPOOL_DATA(name, size, align)
  void chPoolObjectInitAligned(memory_pool_t *mp, size_t size,
                               unsigned align, memgetfunc_t provider);
  void chPoolLoadArray(memory_pool_t *mp, void *p, size_t n);
  void *chPoolAllocI(memory_pool_t *mp);
  void *chPoolAlloc(memory_pool_t *mp);
  void chPoolFreeI(memory_pool_t *mp, void *objp);
  void chPoolFree(memory_pool_t *mp, void *objp);
  void chGuardedPoolObjectInitAligned(guarded_memory_pool_t *gmp,
                                      size_t size,
                                      unsigned align);
  void chGuardedPoolLoadArray(guarded_memory_pool_t *gmp, void *p, size_t n);
  void *chGuardedPoolAllocTimeoutS(guarded_memory_pool_t *gmp,
                                   sysinterval_t timeout);
  void *chGuardedPoolAllocTimeout(guarded_memory_pool_t *gmp,
                                  sysinterval_t timeout);
  void chGuardedPoolFree(guarded_memory_pool_t *gmp, void *objp);
static inline void chPoolObjectInit(memory_pool_t *mp,
                                    size_t size,
                                    memgetfunc_t provider) {
  chPoolObjectInitAligned(mp, size, PORT_NATURAL_ALIGN, provider);
}
static inline void chPoolAdd(memory_pool_t *mp, void *objp) {
  chPoolFree(mp, objp);
}
static inline void chPoolAddI(memory_pool_t *mp, void *objp) {
  chPoolFreeI(mp, objp);
}
static inline void chGuardedPoolObjectInit(guarded_memory_pool_t *gmp,
                                           size_t size) {
  chGuardedPoolObjectInitAligned(gmp, size, PORT_NATURAL_ALIGN);
}
static inline cnt_t chGuardedPoolGetCounterI(guarded_memory_pool_t *gmp) {
  return chSemGetCounterI(&gmp->sem);
}
static inline void *chGuardedPoolAllocI(guarded_memory_pool_t *gmp) {
  void *p;
  if (chSemGetCounterI(&gmp->sem) > (cnt_t)0) {
    chSemFastWaitI(&gmp->sem);
    p = chPoolAllocI(&gmp->pool);
  }
  else {
    p = NULL;
  }
  return p;
}
static inline void chGuardedPoolFreeI(guarded_memory_pool_t *gmp, void *objp) {
  chPoolFreeI(&gmp->pool, objp);
  chSemSignalI(&gmp->sem);
}
static inline void chGuardedPoolFreeS(guarded_memory_pool_t *gmp, void *objp) {
  chGuardedPoolFreeI(gmp, objp);
  chSchRescheduleS();
}
static inline void chGuardedPoolAdd(guarded_memory_pool_t *gmp, void *objp) {
  chGuardedPoolFree(gmp, objp);
}
static inline void chGuardedPoolAddI(guarded_memory_pool_t *gmp, void *objp) {
  chGuardedPoolFreeI(gmp, objp);
}
static inline void chGuardedPoolAddS(guarded_memory_pool_t *gmp, void *objp) {
  chGuardedPoolFreeS(gmp, objp);
}
#define CHOBJFIFOS_H 
typedef struct ch_objects_fifo {
  guarded_memory_pool_t free;
  mailbox_t mbx;
} objects_fifo_t;
static inline void chFifoObjectInitAligned(objects_fifo_t *ofp, size_t objsize,
                                           size_t objn, unsigned objalign,
                                           void *objbuf, msg_t *msgbuf) {
  chDbgCheck((objsize >= objalign) && ((objsize % objalign) == 0U));
  chGuardedPoolObjectInitAligned(&ofp->free, objsize, objalign);
  chGuardedPoolLoadArray(&ofp->free, objbuf, objn);
  chMBObjectInit(&ofp->mbx, msgbuf, objn);
}
static inline void chFifoObjectInit(objects_fifo_t *ofp, size_t objsize,
                                    size_t objn, void *objbuf,
                                    msg_t *msgbuf) {
  chFifoObjectInitAligned(ofp, objsize, objn,
                          PORT_NATURAL_ALIGN,
                          objbuf, msgbuf);
}
static inline void *chFifoTakeObjectI(objects_fifo_t *ofp) {
  return chGuardedPoolAllocI(&ofp->free);
}
static inline void *chFifoTakeObjectTimeoutS(objects_fifo_t *ofp,
                                             sysinterval_t timeout) {
  return chGuardedPoolAllocTimeoutS(&ofp->free, timeout);
}
static inline void *chFifoTakeObjectTimeout(objects_fifo_t *ofp,
                                            sysinterval_t timeout) {
  return chGuardedPoolAllocTimeout(&ofp->free, timeout);
}
static inline void chFifoReturnObjectI(objects_fifo_t *ofp,
                                       void *objp) {
  chGuardedPoolFreeI(&ofp->free, objp);
}
static inline void chFifoReturnObjectS(objects_fifo_t *ofp,
                                       void *objp) {
  chGuardedPoolFreeS(&ofp->free, objp);
}
static inline void chFifoReturnObject(objects_fifo_t *ofp,
                                      void *objp) {
  chGuardedPoolFree(&ofp->free, objp);
}
static inline void chFifoSendObjectI(objects_fifo_t *ofp,
                                     void *objp) {
  msg_t msg;
  msg = chMBPostI(&ofp->mbx, (msg_t)objp);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chFifoSendObjectS(objects_fifo_t *ofp,
                                     void *objp) {
  msg_t msg;
  msg = chMBPostTimeoutS(&ofp->mbx, (msg_t)objp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chFifoSendObject(objects_fifo_t *ofp, void *objp) {
  msg_t msg;
  msg = chMBPostTimeout(&ofp->mbx, (msg_t)objp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chFifoSendObjectAheadI(objects_fifo_t *ofp,
                                          void *objp) {
  msg_t msg;
  msg = chMBPostAheadI(&ofp->mbx, (msg_t)objp);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chFifoSendObjectAheadS(objects_fifo_t *ofp,
                                          void *objp) {
  msg_t msg;
  msg = chMBPostAheadTimeoutS(&ofp->mbx, (msg_t)objp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chFifoSendObjectAhead(objects_fifo_t *ofp, void *objp) {
  msg_t msg;
  msg = chMBPostAheadTimeout(&ofp->mbx, (msg_t)objp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline msg_t chFifoReceiveObjectI(objects_fifo_t *ofp,
                                         void **objpp) {
  return chMBFetchI(&ofp->mbx, (msg_t *)objpp);
}
static inline msg_t chFifoReceiveObjectTimeoutS(objects_fifo_t *ofp,
                                                void **objpp,
                                                sysinterval_t timeout) {
  return chMBFetchTimeoutS(&ofp->mbx, (msg_t *)objpp, timeout);
}
static inline msg_t chFifoReceiveObjectTimeout(objects_fifo_t *ofp,
                                               void **objpp,
                                               sysinterval_t timeout) {
  return chMBFetchTimeout(&ofp->mbx, (msg_t *)objpp, timeout);
}
#define CHPIPES_H 
typedef struct {
  uint8_t *buffer;
  uint8_t *top;
  uint8_t *wrptr;
  uint8_t *rdptr;
  size_t cnt;
  bool reset;
  thread_reference_t wtr;
  thread_reference_t rtr;
  mutex_t cmtx;
  mutex_t wmtx;
  mutex_t rmtx;
} pipe_t;
#define __PIPE_DATA(name,buffer,size) { (uint8_t *)(buffer), (uint8_t *)(buffer) + size, (uint8_t *)(buffer), (uint8_t *)(buffer), (size_t)0, false, NULL, NULL, __MUTEX_DATA(name.cmtx), __MUTEX_DATA(name.wmtx), __MUTEX_DATA(name.rmtx), }
#define PIPE_DECL(name,buffer,size) pipe_t name = __PIPE_DATA(name, buffer, size)
  void chPipeObjectInit(pipe_t *pp, uint8_t *buf, size_t n);
  void chPipeReset(pipe_t *pp);
  size_t chPipeWriteTimeout(pipe_t *pp, const uint8_t *bp,
                            size_t n, sysinterval_t timeout);
  size_t chPipeReadTimeout(pipe_t *pp, uint8_t *bp,
                           size_t n, sysinterval_t timeout);
static inline size_t chPipeGetSize(const pipe_t *pp) {
  return (size_t)(pp->top - pp->buffer);
}
static inline size_t chPipeGetUsedCount(const pipe_t *pp) {
  return pp->cnt;
}
static inline size_t chPipeGetFreeCount(const pipe_t *pp) {
  return chPipeGetSize(pp) - chPipeGetUsedCount(pp);
}
static inline void chPipeResume(pipe_t *pp) {
  pp->reset = false;
}
#define CHOBJCACHES_H 
#define OC_FLAG_INLRU 0x00000001U
#define OC_FLAG_INHASH 0x00000002U
#define OC_FLAG_SHARED 0x00000004U
#define OC_FLAG_NOTSYNC 0x00000008U
#define OC_FLAG_LAZYWRITE 0x00000010U
#define OC_FLAG_FORGET 0x00000020U
typedef uint32_t oc_flags_t;
typedef struct ch_oc_hash_header oc_hash_header_t;
typedef struct ch_oc_lru_header oc_lru_header_t;
typedef struct ch_oc_object oc_object_t;
typedef struct ch_objects_cache objects_cache_t;
typedef bool (*oc_readf_t)(objects_cache_t *ocp,
                           oc_object_t *objp,
                           bool async);
typedef bool (*oc_writef_t)(objects_cache_t *ocp,
                            oc_object_t *objp,
                            bool async);
struct ch_oc_hash_header {
  oc_object_t *hash_next;
  oc_object_t *hash_prev;
};
struct ch_oc_lru_header {
  oc_object_t *hash_next;
  oc_object_t *hash_prev;
  oc_object_t *lru_next;
  oc_object_t *lru_prev;
};
struct ch_oc_object {
  oc_object_t *hash_next;
  oc_object_t *hash_prev;
  oc_object_t *lru_next;
  oc_object_t *lru_prev;
  uint32_t obj_group;
  uint32_t obj_key;
  semaphore_t obj_sem;
  oc_flags_t obj_flags;
  void *dptr;
};
struct ch_objects_cache {
  ucnt_t hashn;
  oc_hash_header_t *hashp;
  ucnt_t objn;
  size_t objsz;
  void *objvp;
  oc_lru_header_t lru;
  semaphore_t lru_sem;
  oc_readf_t readf;
  oc_writef_t writef;
};
  void chCacheObjectInit(objects_cache_t *ocp,
                         ucnt_t hashn,
                         oc_hash_header_t *hashp,
                         ucnt_t objn,
                         size_t objsz,
                         void *objvp,
                         oc_readf_t readf,
                         oc_writef_t writef);
  oc_object_t *chCacheGetObject(objects_cache_t *ocp,
                                uint32_t group,
                                uint32_t key);
  void chCacheReleaseObjectI(objects_cache_t *ocp,
                             oc_object_t *objp);
  bool chCacheReadObject(objects_cache_t *ocp,
                         oc_object_t *objp,
                         bool async);
  bool chCacheWriteObject(objects_cache_t *ocp,
                          oc_object_t *objp,
                          bool async);
static inline void chCacheReleaseObject(objects_cache_t *ocp,
                                        oc_object_t *objp) {
  chSysLock();
  chCacheReleaseObjectI(ocp, objp);
  chSchRescheduleS();
  chSysUnlock();
}
#define CHDELEGATES_H 
typedef msg_t (*delegate_veneer_t)(va_list *argsp);
typedef msg_t (*delegate_fn0_t)(void);
typedef msg_t (*delegate_fn1_t)(msg_t p1);
typedef msg_t (*delegate_fn2_t)(msg_t p1, msg_t p2);
typedef msg_t (*delegate_fn3_t)(msg_t p1, msg_t p2, msg_t p3);
typedef msg_t (*delegate_fn4_t)(msg_t p1, msg_t p2, msg_t p3, msg_t p4);
  msg_t __ch_delegate_fn0(va_list *argsp);
  msg_t __ch_delegate_fn1(va_list *argsp);
  msg_t __ch_delegate_fn2(va_list *argsp);
  msg_t __ch_delegate_fn3(va_list *argsp);
  msg_t __ch_delegate_fn4(va_list *argsp);
  void chDelegateDispatch(void);
  msg_t chDelegateDispatchTimeout(sysinterval_t timeout);
  msg_t chDelegateCallVeneer(thread_t *tp, delegate_veneer_t veneer, ...);
static inline msg_t chDelegateCallDirect0(thread_t *tp, delegate_fn0_t func) {
  return chDelegateCallVeneer(tp, __ch_delegate_fn0, func);
}
static inline msg_t chDelegateCallDirect1(thread_t *tp, delegate_fn1_t func,
                                          msg_t p1) {
  return chDelegateCallVeneer(tp, __ch_delegate_fn1, func, p1);
}
static inline msg_t chDelegateCallDirect2(thread_t *tp, delegate_fn2_t func,
                                          msg_t p1, msg_t p2) {
  return chDelegateCallVeneer(tp, __ch_delegate_fn2, func, p1, p2);
}
static inline msg_t chDelegateCallDirect3(thread_t *tp, delegate_fn3_t func,
                                          msg_t p1, msg_t p2, msg_t p3) {
  return chDelegateCallVeneer(tp, __ch_delegate_fn3, func, p1, p2, p3);
}
static inline msg_t chDelegateCallDirect4(thread_t *tp, delegate_fn4_t func,
                                          msg_t p1, msg_t p2, msg_t p3,
                                          msg_t p4) {
  return chDelegateCallVeneer(tp, __ch_delegate_fn4, func, p1, p2, p3, p4);
}
#define CHJOBS_H 
#define MSG_JOB_NULL ((msg_t)-3)
typedef struct ch_jobs_queue {
  guarded_memory_pool_t free;
  mailbox_t mbx;
} jobs_queue_t;
typedef void (*job_function_t)(void *arg);
typedef struct ch_job_descriptor {
  job_function_t jobfunc;
  void *jobarg;
} job_descriptor_t;
static inline void chJobObjectInit(jobs_queue_t *jqp,
                                   size_t jobsn,
                                   job_descriptor_t *jobsbuf,
                                   msg_t *msgbuf) {
  chDbgCheck((jqp != NULL) && (jobsn > 0U) && (jobsbuf != NULL) && (msgbuf != NULL));
  chGuardedPoolObjectInit(&jqp->free, sizeof (job_descriptor_t));
  chGuardedPoolLoadArray(&jqp->free, (void *)jobsbuf, jobsn);
  chMBObjectInit(&jqp->mbx, msgbuf, jobsn);
}
static inline job_descriptor_t *chJobGet(jobs_queue_t *jqp) {
  return (job_descriptor_t *)chGuardedPoolAllocTimeout(&jqp->free, TIME_INFINITE);
}
static inline job_descriptor_t *chJobGetI(jobs_queue_t *jqp) {
  return (job_descriptor_t *)chGuardedPoolAllocI(&jqp->free);
}
static inline job_descriptor_t *chJobGetTimeoutS(jobs_queue_t *jqp,
                                                 sysinterval_t timeout) {
  return (job_descriptor_t *)chGuardedPoolAllocTimeoutS(&jqp->free, timeout);
}
static inline job_descriptor_t *chJobGetTimeout(jobs_queue_t *jqp,
                                                sysinterval_t timeout) {
  return (job_descriptor_t *)chGuardedPoolAllocTimeout(&jqp->free, timeout);
}
static inline void chJobPostI(jobs_queue_t *jqp, job_descriptor_t *jp) {
  msg_t msg;
  chDbgCheck(jp != NULL);
  msg = chMBPostI(&jqp->mbx, (msg_t)jp);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chJobPostS(jobs_queue_t *jqp, job_descriptor_t *jp) {
  msg_t msg;
  chDbgCheck(jp != NULL);
  msg = chMBPostTimeoutS(&jqp->mbx, (msg_t)jp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chJobPost(jobs_queue_t *jqp, job_descriptor_t *jp) {
  msg_t msg;
  chDbgCheck(jp != NULL);
  msg = chMBPostTimeout(&jqp->mbx, (msg_t)jp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chJobPostAheadI(jobs_queue_t *jqp, job_descriptor_t *jp) {
  msg_t msg;
  chDbgCheck(jp != NULL);
  msg = chMBPostAheadI(&jqp->mbx, (msg_t)jp);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chJobPostAheadS(jobs_queue_t *jqp, job_descriptor_t *jp) {
  msg_t msg;
  chDbgCheck(jp != NULL);
  msg = chMBPostAheadTimeoutS(&jqp->mbx, (msg_t)jp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline void chJobPostAhead(jobs_queue_t *jqp, job_descriptor_t *jp) {
  msg_t msg;
  chDbgCheck(jp != NULL);
  msg = chMBPostAheadTimeout(&jqp->mbx, (msg_t)jp, TIME_IMMEDIATE);
  chDbgAssert(msg == MSG_OK, "post failed");
}
static inline msg_t chJobDispatch(jobs_queue_t *jqp) {
  msg_t msg, jmsg;
  msg = chMBFetchTimeout(&jqp->mbx, &jmsg, TIME_INFINITE);
  if (msg == MSG_OK) {
    job_descriptor_t *jp = (job_descriptor_t *)jmsg;
    chDbgAssert(jp != NULL, "is NULL");
    if (jp->jobfunc != NULL) {
      jp->jobfunc(jp->jobarg);
    }
    else {
      msg = MSG_JOB_NULL;
    }
    chGuardedPoolFree(&jqp->free, (void *)jp);
  }
  return msg;
}
static inline msg_t chJobDispatchTimeout(jobs_queue_t *jqp,
                                         sysinterval_t timeout) {
  msg_t msg, jmsg;
  msg = chMBFetchTimeout(&jqp->mbx, &jmsg, timeout);
  if (msg == MSG_OK) {
    job_descriptor_t *jp = (job_descriptor_t *)jmsg;
    chDbgAssert(jp != NULL, "is NULL");
    if (jp->jobfunc != NULL) {
      jp->jobfunc(jp->jobarg);
    }
    else {
      msg = MSG_JOB_NULL;
    }
    chGuardedPoolFree(&jqp->free, (void *)jp);
  }
  return msg;
}
#define CHFACTORY_H 
#define CH_FACTORY_REQUIRES_POOLS ((CH_CFG_FACTORY_OBJECTS_REGISTRY == TRUE) || (CH_CFG_FACTORY_SEMAPHORES == TRUE))
#define CH_FACTORY_REQUIRES_HEAP ((CH_CFG_FACTORY_GENERIC_BUFFERS == TRUE) || (CH_CFG_FACTORY_MAILBOXES == TRUE) || (CH_CFG_FACTORY_OBJ_FIFOS == TRUE) || (CH_CFG_FACTORY_PIPES == TRUE))
typedef struct ch_dyn_element {
  struct ch_dyn_element *next;
  ucnt_t refs;
  char name[CH_CFG_FACTORY_MAX_NAMES_LENGTH];
} dyn_element_t;
typedef struct ch_dyn_list {
    dyn_element_t *next;
} dyn_list_t;
typedef struct ch_registered_static_object {
  dyn_element_t element;
  void *objp;
} registered_object_t;
typedef struct ch_dyn_object {
  dyn_element_t element;
} dyn_buffer_t;
typedef struct ch_dyn_semaphore {
  dyn_element_t element;
  semaphore_t sem;
} dyn_semaphore_t;
typedef struct ch_dyn_mailbox {
  dyn_element_t element;
  mailbox_t mbx;
} dyn_mailbox_t;
typedef struct ch_dyn_objects_fifo {
  dyn_element_t element;
  objects_fifo_t fifo;
} dyn_objects_fifo_t;
typedef struct ch_dyn_pipe {
  dyn_element_t element;
  pipe_t pipe;
} dyn_pipe_t;
typedef struct ch_objects_factory {
  mutex_t mtx;
  dyn_list_t obj_list;
  memory_pool_t obj_pool;
  dyn_list_t buf_list;
  dyn_list_t sem_list;
  memory_pool_t sem_pool;
  dyn_list_t mbx_list;
  dyn_list_t fifo_list;
  dyn_list_t pipe_list;
} objects_factory_t;
extern objects_factory_t ch_factory;
  void __factory_init(void);
  dyn_element_t *chFactoryDuplicateReference(dyn_element_t *dep);
  registered_object_t *chFactoryRegisterObject(const char *name,
                                               void *objp);
  registered_object_t *chFactoryFindObject(const char *name);
  registered_object_t *chFactoryFindObjectByPointer(void *objp);
  void chFactoryReleaseObject(registered_object_t *rop);
  dyn_buffer_t *chFactoryCreateBuffer(const char *name, size_t size);
  dyn_buffer_t *chFactoryFindBuffer(const char *name);
  void chFactoryReleaseBuffer(dyn_buffer_t *dbp);
  dyn_semaphore_t *chFactoryCreateSemaphore(const char *name, cnt_t n);
  dyn_semaphore_t *chFactoryFindSemaphore(const char *name);
  void chFactoryReleaseSemaphore(dyn_semaphore_t *dsp);
  dyn_mailbox_t *chFactoryCreateMailbox(const char *name, size_t n);
  dyn_mailbox_t *chFactoryFindMailbox(const char *name);
  void chFactoryReleaseMailbox(dyn_mailbox_t *dmp);
  dyn_objects_fifo_t *chFactoryCreateObjectsFIFO(const char *name,
                                                 size_t objsize,
                                                 size_t objn,
                                                 unsigned objalign);
  dyn_objects_fifo_t *chFactoryFindObjectsFIFO(const char *name);
  void chFactoryReleaseObjectsFIFO(dyn_objects_fifo_t *dofp);
  dyn_pipe_t *chFactoryCreatePipe(const char *name, size_t size);
  dyn_pipe_t *chFactoryFindPipe(const char *name);
  void chFactoryReleasePipe(dyn_pipe_t *dpp);
static inline void *chFactoryGetObject(registered_object_t *rop) {
  return rop->objp;
}
static inline size_t chFactoryGetBufferSize(dyn_buffer_t *dbp) {
  return chHeapGetSize(dbp) - sizeof (dyn_element_t);
}
static inline uint8_t *chFactoryGetBuffer(dyn_buffer_t *dbp) {
  return (uint8_t *)(dbp + 1);
}
static inline semaphore_t *chFactoryGetSemaphore(dyn_semaphore_t *dsp) {
  return &dsp->sem;
}
static inline mailbox_t *chFactoryGetMailbox(dyn_mailbox_t *dmp) {
  return &dmp->mbx;
}
static inline objects_fifo_t *chFactoryGetObjectsFIFO(dyn_objects_fifo_t *dofp) {
  return &dofp->fifo;
}
static inline pipe_t *chFactoryGetPipe(dyn_pipe_t *dpp) {
  return &dpp->pipe;
}
static inline void __oslib_init(void) {
  __core_init();
  __heap_init();
  __factory_init();
}
#define CHDYNAMIC_H 
  thread_t *chThdCreateFromHeap(memory_heap_t *heapp, size_t size,
                                const char *name, tprio_t prio,
                                tfunc_t pf, void *arg);
  thread_t *chThdCreateFromMemoryPool(memory_pool_t *mp, const char *name,
                                      tprio_t prio, tfunc_t pf, void *arg);
