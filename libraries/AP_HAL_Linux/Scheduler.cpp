#include "Scheduler.h"

#include <algorithm>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "RCInput.h"
#include "SPIUARTDriver.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "Util.h"

using namespace Linux;

extern const AP_HAL::HAL& hal;

#define APM_LINUX_MAX_PRIORITY          20
#define APM_LINUX_TIMER_PRIORITY        15
#define APM_LINUX_UART_PRIORITY         14
#define APM_LINUX_RCIN_PRIORITY         13
#define APM_LINUX_MAIN_PRIORITY         12
#define APM_LINUX_IO_PRIORITY           10
#define APM_LINUX_SCRIPTING_PRIORITY     1

#define APM_LINUX_TIMER_RATE            1000
#define APM_LINUX_UART_RATE             100
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO ||    \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
#define APM_LINUX_RCIN_RATE             500
#define APM_LINUX_IO_RATE               50
#else
#define APM_LINUX_RCIN_RATE             100
#define APM_LINUX_IO_RATE               50
#endif

#define SCHED_THREAD(name_, UPPER_NAME_)                        \
    {                                                           \
        .name = "ap-" #name_,                                   \
        .thread = &_##name_##_thread,                           \
        .policy = SCHED_FIFO,                                   \
        .prio = APM_LINUX_##UPPER_NAME_##_PRIORITY,             \
        .rate = APM_LINUX_##UPPER_NAME_##_RATE,                 \
    }

Scheduler::Scheduler()
{ }


void Scheduler::init_realtime()
{
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    // we don't run Replay in real-time...
    return;
#endif
#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    // we opportunistically run examples/tools in realtime
    if (geteuid() != 0) {
        fprintf(stderr, "WARNING: not running as root. Will not use realtime scheduling\n");
        return;
    }
#endif

    mlockall(MCL_CURRENT|MCL_FUTURE);

    struct sched_param param = { .sched_priority = APM_LINUX_MAIN_PRIORITY };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == -1) {
        AP_HAL::panic("Scheduler: failed to set scheduling parameters: %s",
                      strerror(errno));
    }
}

void Scheduler::init()
{
    int ret;
    const struct sched_table {
        const char *name;
        SchedulerThread *thread;
        int policy;
        int prio;
        uint32_t rate;
    } sched_table[] = {
        SCHED_THREAD(timer, TIMER),
        SCHED_THREAD(uart, UART),
        SCHED_THREAD(rcin, RCIN),
        SCHED_THREAD(io, IO),
    };

    _main_ctx = pthread_self();

    init_realtime();

    /* set barrier to N + 1 threads: worker threads + main */
    unsigned n_threads = ARRAY_SIZE(sched_table) + 1;
    ret = pthread_barrier_init(&_initialized_barrier, nullptr, n_threads);
    if (ret) {
        AP_HAL::panic("Scheduler: Failed to initialise barrier object: %s",
                      strerror(ret));
    }

    for (size_t i = 0; i < ARRAY_SIZE(sched_table); i++) {
        const struct sched_table *t = &sched_table[i];

        t->thread->set_rate(t->rate);
        t->thread->set_stack_size(1024 * 1024);
        t->thread->start(t->name, t->policy, t->prio);
    }

#if defined(DEBUG_STACK) && DEBUG_STACK
    register_timer_process(FUNCTOR_BIND_MEMBER(&Scheduler::_debug_stack, void));
#endif
}

void Scheduler::_debug_stack()
{
    uint64_t now = AP_HAL::millis64();

    if (now - _last_stack_debug_msec > 5000) {
        fprintf(stderr, "Stack Usage:\n"
                "\ttimer = %zu\n"
                "\tio    = %zu\n"
                "\trcin  = %zu\n"
                "\tuart  = %zu\n",
                _timer_thread.get_stack_usage(),
                _io_thread.get_stack_usage(),
                _rcin_thread.get_stack_usage(),
                _uart_thread.get_stack_usage());
        _last_stack_debug_msec = now;
    }
}

void Scheduler::microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void Scheduler::delay(uint16_t ms)
{
    if (_stopped_clock_usec) {
        return;
    }

    uint64_t start = AP_HAL::millis64();

    while ((AP_HAL::millis64() - start) < ms) {
        // this yields the CPU to other apps
        microsleep(1000);
        if (in_main_thread() && _min_delay_cb_ms <= ms) {
            call_delay_cb();
        }
    }
}

void Scheduler::delay_microseconds(uint16_t us)
{
    if (_stopped_clock_usec) {
        return;
    }
    microsleep(us);
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs >= LINUX_SCHEDULER_MAX_TIMER_PROCS) {
        hal.console->printf("Out of timer processes\n");
        return;
    }

    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < LINUX_SCHEDULER_MAX_IO_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::_timer_task()
{
    int i;

    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    // now call the timer based drivers
    for (i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_run_io(void)
{
    _io_semaphore.take_blocking();

    // now call the IO based drivers
    for (int i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }

    _io_semaphore.give();
}

/*
  run timers for all UARTs
 */
void Scheduler::_run_uarts()
{
    // process any pending serial bytes
    for (uint8_t i=0;i<hal.num_serial; i++) {
        hal.serial(i)->_timer_tick();
    }
}

void Scheduler::_rcin_task()
{
    RCInput::from(hal.rcin)->_timer_tick();
}

void Scheduler::_uart_task()
{
    _run_uarts();
}

void Scheduler::_io_task()
{
    // process any pending storage writes
    hal.storage->_timer_tick();

    // run registered IO processes
    _run_io();
}

bool Scheduler::in_main_thread() const
{
    return pthread_equal(pthread_self(), _main_ctx);
}

void Scheduler::_wait_all_threads()
{
    int r = pthread_barrier_wait(&_initialized_barrier);
    if (r == PTHREAD_BARRIER_SERIAL_THREAD) {
        pthread_barrier_destroy(&_initialized_barrier);
    }
}

void Scheduler::set_system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::set_system_initialized called more than once");
    }

    _initialized = true;

    _wait_all_threads();
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    exit(1);
}

#if APM_BUILD_TYPE(APM_BUILD_Replay)
void Scheduler::stop_clock(uint64_t time_usec)
{
    if (time_usec < _stopped_clock_usec) {
        ::fprintf(stderr, "Warning: setting time backwards from (%" PRIu64 ") to (%" PRIu64 ")\n", _stopped_clock_usec, time_usec);
        return;
    }

    _stopped_clock_usec = time_usec;
    _run_io();
}
#else
void Scheduler::stop_clock(uint64_t time_usec)
{
    // stop_clock() is not called outside of Replay, but we can't
    // guard it in the header because of the vehicle-dependent-library
    // checks in waf.
}
#endif

bool Scheduler::SchedulerThread::_run()
{
    _sched._wait_all_threads();

    return PeriodicThread::_run();
}

void Scheduler::teardown()
{
    _timer_thread.stop();
    _io_thread.stop();
    _rcin_thread.stop();
    _uart_thread.stop();

    _timer_thread.join();
    _io_thread.join();
    _rcin_thread.join();
    _uart_thread.join();
}

// calculates an integer to be used as the priority for a newly-created thread
uint8_t Scheduler::calculate_thread_priority(priority_base base, int8_t priority) const
{
    uint8_t thread_priority = APM_LINUX_IO_PRIORITY;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, APM_LINUX_MAIN_PRIORITY},
        { PRIORITY_MAIN, APM_LINUX_MAIN_PRIORITY},
        { PRIORITY_SPI, AP_LINUX_SENSORS_SCHED_PRIO},
        { PRIORITY_I2C, AP_LINUX_SENSORS_SCHED_PRIO},
        { PRIORITY_CAN, APM_LINUX_TIMER_PRIORITY},
        { PRIORITY_TIMER, APM_LINUX_TIMER_PRIORITY},
        { PRIORITY_RCIN, APM_LINUX_RCIN_PRIORITY},
        { PRIORITY_IO, APM_LINUX_IO_PRIORITY},
        { PRIORITY_UART, APM_LINUX_UART_PRIORITY},
        { PRIORITY_STORAGE, APM_LINUX_IO_PRIORITY},
        { PRIORITY_SCRIPTING, APM_LINUX_SCRIPTING_PRIORITY},
    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
            thread_priority = constrain_int16(priority_map[i].p + priority, 1, APM_LINUX_MAX_PRIORITY);
            break;
        }
    }

    return thread_priority;
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
    Thread *thread = new Thread{(Thread::task_t)proc};
    if (!thread) {
        return false;
    }

    const uint8_t thread_priority = calculate_thread_priority(base, priority);

    // Add 256k to HAL-independent requested stack size
    thread->set_stack_size(256 * 1024 + stack_size);

    /*
     * We should probably store the thread handlers and join() when exiting,
     * but let's the thread manage itself for now.
     */
    thread->set_auto_free(true);

    if (!thread->start(name, SCHED_FIFO, thread_priority)) {
        delete thread;
        return false;
    }

    return true;
}
