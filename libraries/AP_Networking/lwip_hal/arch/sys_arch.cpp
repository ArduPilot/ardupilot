/*
  port of lwip to ArduPilot AP_HAL
  This is partly based on ChibiOS/os/various/lwip_bindings
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Networking/AP_Networking_Config.h>

#if AP_NETWORKING_NEED_LWIP
#include <AP_HAL/Semaphores.h>
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#endif

#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
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
    HAL_BinarySemaphore not_empty;
    HAL_BinarySemaphore not_full;
    HAL_BinarySemaphore mutex;
    int wait_send;
};

class ThreadWrapper {
public:
    ThreadWrapper(lwip_thread_fn fn, void *_arg) :
        function(fn),
        arg(_arg)
        {}
    bool create(const char *name, int stacksize, int prio) {
#ifdef HAL_BOOTLOADER_BUILD
        return thread_create_alloc(MAX(stacksize,2048), name, 60, function, arg);
#else
        return hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&ThreadWrapper::run, void), name, MAX(stacksize,2048), AP_HAL::Scheduler::PRIORITY_NET, prio);
#endif
    }
private:
    void run(void) {
        function(arg);
    }
    lwip_thread_fn function;
    void *arg;
};

sys_thread_t
sys_thread_new(const char *name, lwip_thread_fn function, void *arg, int stacksize, int prio)
{
    auto *thread_data = new ThreadWrapper(function, arg);
    if (!thread_data->create(name, stacksize, prio)) {
        AP_HAL::panic("lwip: Failed to start thread %s", name);
    }
    return (sys_thread_t)thread_data;
}

void sys_lock_tcpip_core(void)
{
    tcpip_mutex.take_blocking();
}

void sys_unlock_tcpip_core(void)
{
    tcpip_mutex.give();
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
    mbox->mutex.signal();
    mbox->wait_send = 0;

    *mb = mbox;
    return ERR_OK;
}

void
sys_mbox_free(struct sys_mbox **mb)
{
    if ((mb != NULL) && (*mb != SYS_MBOX_NULL)) {
        struct sys_mbox *mbox = *mb;
        mbox->mutex.wait_blocking();
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

    mbox->mutex.wait_blocking();

    LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_trypost: mbox %p msg %p\n",
                            (void *)mbox, (void *)msg));

    if ((mbox->last + 1) >= (mbox->first + SYS_MBOX_SIZE)) {
        mbox->mutex.signal();
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
        mbox->not_empty.signal();
    }

    mbox->mutex.signal();

    return ERR_OK;
}

void
sys_mbox_post(struct sys_mbox **mb, void *msg)
{
    u8_t first;
    struct sys_mbox *mbox;
    LWIP_ASSERT("invalid mbox", (mb != NULL) && (*mb != NULL));
    mbox = *mb;

    mbox->mutex.wait_blocking();

    LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_post: mbox %p msg %p\n", (void *)mbox, (void *)msg));

    while ((mbox->last + 1) >= (mbox->first + SYS_MBOX_SIZE)) {
        mbox->wait_send++;
        mbox->mutex.signal();
        mbox->not_full.wait_blocking();
        mbox->mutex.wait_blocking();
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
        mbox->not_empty.signal();
    }

    mbox->mutex.signal();
}

u32_t
sys_arch_mbox_tryfetch(struct sys_mbox **mb, void **msg)
{
    struct sys_mbox *mbox = *mb;

    mbox->mutex.wait_blocking();

    if (mbox->first == mbox->last) {
        mbox->mutex.signal();
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
        mbox->not_full.signal();
    }

    mbox->mutex.signal();

    return 0;
}

u32_t
sys_arch_mbox_fetch(struct sys_mbox **mb, void **msg, u32_t timeout_ms)
{
    struct sys_mbox *mbox;
    LWIP_ASSERT("invalid mbox", (mb != NULL) && (*mb != NULL));
    mbox = *mb;

    mbox->mutex.wait_blocking();

    while (mbox->first == mbox->last) {
        mbox->mutex.signal();

        /* We block while waiting for a mail to arrive in the mailbox. We
           must be prepared to timeout. */
        if (timeout_ms != 0) {
            if (!mbox->not_empty.wait(timeout_ms*1000U)) {
                return SYS_ARCH_TIMEOUT;
            }
        } else {
            mbox->not_empty.wait_blocking();
        }

        mbox->mutex.wait_blocking();
    }

    if (msg != NULL) {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_fetch: mbox %p msg %p\n", (void *)mbox, *msg));
        *msg = mbox->msgs[mbox->first % SYS_MBOX_SIZE];
    } else {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_fetch: mbox %p, null msg\n", (void *)mbox));
    }

    mbox->first++;

    if (mbox->wait_send) {
        mbox->not_full.signal();
    }

    mbox->mutex.signal();

    return 0;
}

err_t
sys_sem_new(sys_sem_t *sem, u8_t count)
{
    *sem = (sys_sem_t)new HAL_BinarySemaphore(count);
    if (*sem == NULL) {
        return ERR_MEM;
    }
    return ERR_OK;
}

u32_t
sys_arch_sem_wait(sys_sem_t *s, u32_t timeout_ms)
{
    HAL_BinarySemaphore *sem = (HAL_BinarySemaphore *)*s;
    if (timeout_ms == 0) {
        return sem->wait_blocking()?0:SYS_ARCH_TIMEOUT;
    }
    return sem->wait(timeout_ms*1000U)?0:SYS_ARCH_TIMEOUT;
}

void
sys_sem_signal(sys_sem_t *s)
{
    HAL_BinarySemaphore *sem = (HAL_BinarySemaphore *)*s;
    sem->signal();
}

void
sys_sem_free(sys_sem_t *sem)
{
    delete ((HAL_BinarySemaphore *)*sem);
}

/*-----------------------------------------------------------------------------------*/
/* Mutex */
/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
err_t
sys_mutex_new(sys_mutex_t *mutex)
{
    *mutex = (sys_mutex_t)new HAL_Semaphore;
    if (*mutex == nullptr) {
        return ERR_MEM;
    }
    return ERR_OK;
}

/** Lock a mutex
 * @param mutex the mutex to lock */
void
sys_mutex_lock(sys_mutex_t *mutex)
{
    ((HAL_Semaphore*)*mutex)->take_blocking();
}

/** Unlock a mutex
 * @param mutex the mutex to unlock */
void
sys_mutex_unlock(sys_mutex_t *mutex)
{
    ((HAL_Semaphore*)*mutex)->give();
}

/** Delete a mutex
 * @param mutex the mutex to delete */
void
sys_mutex_free(sys_mutex_t *mutex)
{
    delete (HAL_Semaphore*)*mutex;
}

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

void
sys_init(void)
{
}

sys_prot_t
sys_arch_protect(void)
{
    lwprot_mutex.take_blocking();
    return 0;
}

void
sys_arch_unprotect(sys_prot_t pval)
{
    LWIP_UNUSED_ARG(pval);
    lwprot_mutex.give();
}

#endif // AP_NETWORKING_NEED_LWIP

