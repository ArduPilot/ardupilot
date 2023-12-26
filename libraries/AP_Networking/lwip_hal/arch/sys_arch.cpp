/*
  port of lwip to ArduPilot AP_HAL
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>

#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include <lwipopts.h>

extern "C" {
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/opt.h"
#include "lwip/stats.h"
#include "lwip/tcpip.h"
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <semaphore.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "hal.h"
#include "../../../libraries/AP_HAL_ChibiOS/hwdef/common/stm32_util.h"
#endif

extern const AP_HAL::HAL &hal;

unsigned int
lwip_port_rand(void)
{
    return (u32_t)rand();
}

static HAL_Semaphore lwprot_mutex;
static HAL_Semaphore tcpip_mutex;

struct sys_mbox_msg {
    struct sys_mbox_msg *next;
    void *msg;
};

#define SYS_MBOX_SIZE 128

struct sys_mbox {
    int first, last;
    void *msgs[SYS_MBOX_SIZE];
    struct sys_sem *not_empty;
    struct sys_sem *not_full;
    struct sys_sem *mutex;
    int wait_send;
};

struct sys_sem {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    sem_t sem;
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    semaphore_t sem;
#else
#error "Need sys_sem implementation"
#endif
};

static struct sys_sem *sys_sem_new_internal(u8_t count);
static void sys_sem_free_internal(struct sys_sem *sem);

/* Threads */

struct thread_wrapper_data {
    lwip_thread_fn function;
    void *arg;
};

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
static void *
thread_wrapper(void *arg)
{
    auto *thread_data = (struct thread_wrapper_data *)arg;
    thread_data->function(thread_data->arg);
    return NULL;
}
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
static void
thread_wrapper(void *arg)
{
    auto *thread_data = (struct thread_wrapper_data *)arg;
    thread_data->function(thread_data->arg);
}
#endif

sys_thread_t
sys_thread_new(const char *name, lwip_thread_fn function, void *arg, int stacksize, int prio)
{
    sys_thread_t ret = nullptr;
    struct thread_wrapper_data *thread_data;

    thread_data = new thread_wrapper_data;
    thread_data->arg = arg;
    thread_data->function = function;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    pthread_t t;
    if (pthread_create(&t, NULL, thread_wrapper, thread_data) == 0) {
        pthread_setname_np(t, name);
        ret = (void*)t;
    }
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    ret = thread_create_alloc(THD_WORKING_AREA_SIZE(stacksize+1024),
                              name,
                              prio+60, // need to use HAL thread call
                              thread_wrapper,
                              thread_data);
#endif
    if (ret == nullptr) {
        AP_HAL::panic("Failed to create thread %s", name);
    }
    return ret;
}

void sys_lock_tcpip_core(void)
{
    if (hal.scheduler != nullptr) {
        tcpip_mutex.take_blocking();
    }
}

void sys_unlock_tcpip_core(void)
{
    if (hal.scheduler != nullptr) {
        tcpip_mutex.give();
    }
}

void sys_mark_tcpip_thread(void)
{
}

void sys_check_core_locking(void)
{
    /* Embedded systems should check we are NOT in an interrupt
     * context here */
}

/*-----------------------------------------------------------------------------------*/
/* Mailbox */
err_t
sys_mbox_new(struct sys_mbox **mb, int size)
{
    struct sys_mbox *mbox;
    LWIP_UNUSED_ARG(size);

    mbox = new sys_mbox;
    if (mbox == NULL) {
        return ERR_MEM;
    }
    mbox->first = mbox->last = 0;
    mbox->not_empty = sys_sem_new_internal(0);
    mbox->not_full = sys_sem_new_internal(0);
    mbox->mutex = sys_sem_new_internal(1);
    mbox->wait_send = 0;

    *mb = mbox;
    return ERR_OK;
}

void
sys_mbox_free(struct sys_mbox **mb)
{
    if ((mb != NULL) && (*mb != SYS_MBOX_NULL)) {
        struct sys_mbox *mbox = *mb;
        sys_arch_sem_wait(&mbox->mutex, 0);

        sys_sem_free_internal(mbox->not_empty);
        sys_sem_free_internal(mbox->not_full);
        sys_sem_free_internal(mbox->mutex);
        mbox->not_empty = mbox->not_full = mbox->mutex = NULL;
        /*  LWIP_DEBUGF("sys_mbox_free: mbox 0x%lx\n", mbox); */
        delete mbox;
    }
}

err_t
sys_mbox_trypost(struct sys_mbox **mb, void *msg)
{
    u8_t first;
    struct sys_mbox *mbox;
    LWIP_ASSERT("invalid mbox", (mb != NULL) && (*mb != NULL));
    mbox = *mb;

    sys_arch_sem_wait(&mbox->mutex, 0);

    LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_trypost: mbox %p msg %p\n",
                            (void *)mbox, (void *)msg));

    if ((mbox->last + 1) >= (mbox->first + SYS_MBOX_SIZE)) {
        sys_sem_signal(&mbox->mutex);
        return ERR_MEM;
    }

    mbox->msgs[mbox->last % SYS_MBOX_SIZE] = msg;

    if (mbox->last == mbox->first) {
        first = 1;
    } else {
        first = 0;
    }

    mbox->last++;

    if (first) {
        sys_sem_signal(&mbox->not_empty);
    }

    sys_sem_signal(&mbox->mutex);

    return ERR_OK;
}

void
sys_mbox_post(struct sys_mbox **mb, void *msg)
{
    u8_t first;
    struct sys_mbox *mbox;
    LWIP_ASSERT("invalid mbox", (mb != NULL) && (*mb != NULL));
    mbox = *mb;

    sys_arch_sem_wait(&mbox->mutex, 0);

    LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_post: mbox %p msg %p\n", (void *)mbox, (void *)msg));

    while ((mbox->last + 1) >= (mbox->first + SYS_MBOX_SIZE)) {
        mbox->wait_send++;
        sys_sem_signal(&mbox->mutex);
        sys_arch_sem_wait(&mbox->not_full, 0);
        sys_arch_sem_wait(&mbox->mutex, 0);
        mbox->wait_send--;
    }

    mbox->msgs[mbox->last % SYS_MBOX_SIZE] = msg;

    if (mbox->last == mbox->first) {
        first = 1;
    } else {
        first = 0;
    }

    mbox->last++;

    if (first) {
        sys_sem_signal(&mbox->not_empty);
    }

    sys_sem_signal(&mbox->mutex);
}

u32_t
sys_arch_mbox_tryfetch(struct sys_mbox **mb, void **msg)
{
    struct sys_mbox *mbox = *mb;

    sys_arch_sem_wait(&mbox->mutex, 0);

    if (mbox->first == mbox->last) {
        sys_sem_signal(&mbox->mutex);
        return SYS_MBOX_EMPTY;
    }

    if (msg != NULL) {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_tryfetch: mbox %p msg %p\n", (void *)mbox, *msg));
        *msg = mbox->msgs[mbox->first % SYS_MBOX_SIZE];
    } else {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_tryfetch: mbox %p, null msg\n", (void *)mbox));
    }

    mbox->first++;

    if (mbox->wait_send) {
        sys_sem_signal(&mbox->not_full);
    }

    sys_sem_signal(&mbox->mutex);

    return 0;
}

u32_t
sys_arch_mbox_fetch(struct sys_mbox **mb, void **msg, u32_t timeout)
{
    struct sys_mbox *mbox;
    LWIP_ASSERT("invalid mbox", (mb != NULL) && (*mb != NULL));
    mbox = *mb;

    /* The mutex lock is quick so we don't bother with the timeout
       stuff here. */
    sys_arch_sem_wait(&mbox->mutex, 0);

    while (mbox->first == mbox->last) {
        sys_sem_signal(&mbox->mutex);

        /* We block while waiting for a mail to arrive in the mailbox. We
           must be prepared to timeout. */
        if (timeout != 0) {
            u32_t time_needed = sys_arch_sem_wait(&mbox->not_empty, timeout);

            if (time_needed == SYS_ARCH_TIMEOUT) {
                return SYS_ARCH_TIMEOUT;
            }
        } else {
            sys_arch_sem_wait(&mbox->not_empty, 0);
        }

        sys_arch_sem_wait(&mbox->mutex, 0);
    }

    if (msg != NULL) {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_fetch: mbox %p msg %p\n", (void *)mbox, *msg));
        *msg = mbox->msgs[mbox->first % SYS_MBOX_SIZE];
    } else {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_fetch: mbox %p, null msg\n", (void *)mbox));
    }

    mbox->first++;

    if (mbox->wait_send) {
        sys_sem_signal(&mbox->not_full);
    }

    sys_sem_signal(&mbox->mutex);

    return 0;
}

/*-----------------------------------------------------------------------------------*/
/* Semaphore */
static struct sys_sem *
sys_sem_new_internal(u8_t count)
{
    auto *ret = new sys_sem;
    if (ret != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        sem_init(&ret->sem, 0, count);
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        chSemObjectInit(&ret->sem, (cnt_t)count);
#endif
    }
    return ret;
}

err_t
sys_sem_new(struct sys_sem **sem, u8_t count)
{
    *sem = sys_sem_new_internal(count);
    if (*sem == NULL) {
        return ERR_MEM;
    }
    return ERR_OK;
}

u32_t
sys_arch_sem_wait(struct sys_sem **s, u32_t timeout_ms)
{
    struct sys_sem *sem = *s;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    if (timeout_ms == 0) {
        sem_wait(&sem->sem);
        return 0;
    }
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        return SYS_ARCH_TIMEOUT;
    }
    ts.tv_sec += timeout_ms/1000UL;
    ts.tv_nsec += (timeout_ms % 1000U) * 1000000UL;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }
    auto ret = sem_timedwait(&sem->sem, &ts);
    if (ret != 0) {
        return SYS_ARCH_TIMEOUT;
    }
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    chSysLock();
    sysinterval_t tmo = timeout_ms > 0 ? MIN(TIME_MAX_INTERVAL, TIME_MS2I((time_msecs_t)timeout_ms)) : TIME_INFINITE;
    if (chSemWaitTimeoutS(&sem->sem, tmo) != MSG_OK) {
        chSysUnlock();
        return SYS_ARCH_TIMEOUT;
    }
    chSysUnlock();
#endif
    return 0;
}

void
sys_sem_signal(struct sys_sem **s)
{
    struct sys_sem *sem = *s;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    sem_post(&sem->sem);
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    chSemSignal(&sem->sem);
#endif
}

static void
sys_sem_free_internal(struct sys_sem *sem)
{
    delete sem;
}

void
sys_sem_free(struct sys_sem **sem)
{
    if ((sem != NULL) && (*sem != SYS_SEM_NULL)) {
        sys_sem_free_internal(*sem);
    }
}

/*-----------------------------------------------------------------------------------*/
/* Mutex */
/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
err_t
sys_mutex_new(struct sys_mutex **mutex)
{
    auto *sem = new HAL_Semaphore;
    if (sem == nullptr) {
        return ERR_MEM;
    }
    *mutex = (struct sys_mutex *)sem;
    return ERR_OK;
}

/** Lock a mutex
 * @param mutex the mutex to lock */
void
sys_mutex_lock(struct sys_mutex **mutex)
{
    auto *sem = (HAL_Semaphore *)*mutex;
    if (hal.scheduler != nullptr) {
        sem->take_blocking();
    }
}

/** Unlock a mutex
 * @param mutex the mutex to unlock */
void
sys_mutex_unlock(struct sys_mutex **mutex)
{
    auto *sem = (HAL_Semaphore *)*mutex;
    if (hal.scheduler != nullptr) {
        sem->give();
    }
}

/** Delete a mutex
 * @param mutex the mutex to delete */
void
sys_mutex_free(struct sys_mutex **mutex)
{
    auto *sem = (HAL_Semaphore *)*mutex;
    delete sem;
}


/*-----------------------------------------------------------------------------------*/
/* Time */
u32_t
sys_now(void)
{
    return AP_HAL::millis();
}

u32_t
sys_jiffies(void)
{
    return AP_HAL::micros();
}

/*-----------------------------------------------------------------------------------*/
/* Init */

void
sys_init(void)
{
}

/*-----------------------------------------------------------------------------------*/
/* Critical section */
/** sys_prot_t sys_arch_protect(void)

This optional function does a "fast" critical region protection and returns
the previous protection level. This function is only called during very short
critical regions. An embedded system which supports ISR-based drivers might
want to implement this function by disabling interrupts. Task-based systems
might want to implement this by using a mutex or disabling tasking. This
function should support recursive calls from the same task or interrupt. In
other words, sys_arch_protect() could be called while already protected. In
that case the return value indicates that it is already protected.

sys_arch_protect() is only required if your port is supporting an operating
system.
*/
sys_prot_t
sys_arch_protect(void)
{
    if (hal.scheduler != nullptr) {
        lwprot_mutex.take_blocking();
    }
    return 0;
}

/** void sys_arch_unprotect(sys_prot_t pval)

This optional function does a "fast" set of critical region protection to the
value specified by pval. See the documentation for sys_arch_protect() for
more information. This function is only required if your port is supporting
an operating system.
*/
void
sys_arch_unprotect(sys_prot_t pval)
{
    LWIP_UNUSED_ARG(pval);
    if (hal.scheduler != nullptr) {
        lwprot_mutex.give();
    }
}

