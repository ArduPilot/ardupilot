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

#include "RCInput.h"
#include "RPIOUARTDriver.h"
#include "SPIUARTDriver.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "Util.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT
#include <rpcmem.h>
#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>
#include <AP_HAL_Linux/qflight/qflight_buffer.h>
#endif

using namespace Linux;

extern const AP_HAL::HAL& hal;

#define APM_LINUX_TIMER_PRIORITY        15
#define APM_LINUX_UART_PRIORITY         14
#define APM_LINUX_RCIN_PRIORITY         13
#define APM_LINUX_MAIN_PRIORITY         12
#define APM_LINUX_TONEALARM_PRIORITY    11
#define APM_LINUX_IO_PRIORITY           10

#define APM_LINUX_TIMER_RATE            1000
#define APM_LINUX_UART_RATE             100
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO ||    \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI
#define APM_LINUX_RCIN_RATE             2000
#define APM_LINUX_TONEALARM_RATE        100
#define APM_LINUX_IO_RATE               50
#else
#define APM_LINUX_RCIN_RATE             100
#define APM_LINUX_TONEALARM_RATE        100
#define APM_LINUX_IO_RATE               50
#endif

#define SCHED_THREAD(name_, UPPER_NAME_)                        \
    {                                                           \
        .name = "sched-" #name_,                                \
        .thread = &_##name_##_thread,                           \
        .policy = SCHED_FIFO,                                   \
        .prio = APM_LINUX_##UPPER_NAME_##_PRIORITY,             \
        .rate = APM_LINUX_##UPPER_NAME_##_RATE,                 \
    }

Scheduler::Scheduler()
{ }

void Scheduler::init()
{
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
        SCHED_THREAD(tonealarm, TONEALARM),
        SCHED_THREAD(io, IO),
    };

    mlockall(MCL_CURRENT|MCL_FUTURE);

    if (geteuid() != 0) {
        printf("WARNING: running as non-root. Will not use realtime scheduling\n");
    }

    struct sched_param param = { .sched_priority = APM_LINUX_MAIN_PRIORITY };
    sched_setscheduler(0, SCHED_FIFO, &param);

    /* set barrier to N + 1 threads: worker threads + main */
    unsigned n_threads = ARRAY_SIZE(sched_table) + 1;
    pthread_barrier_init(&_initialized_barrier, nullptr, n_threads);

    for (size_t i = 0; i < ARRAY_SIZE(sched_table); i++) {
        const struct sched_table *t = &sched_table[i];

        t->thread->set_rate(t->rate);
        t->thread->set_stack_size(256 * 1024);
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
                "\tuart  = %zu\n"
                "\ttone  = %zu\n",
                _timer_thread.get_stack_usage(),
                _io_thread.get_stack_usage(),
                _rcin_thread.get_stack_usage(),
                _uart_thread.get_stack_usage(),
                _tonealarm_thread.get_stack_usage());
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
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
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

void Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                             uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < LINUX_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

bool Scheduler::register_timer_process(AP_HAL::MemberProc proc,
                                       uint8_t freq_div)
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    if (freq_div > 1) {
        return _register_timesliced_proc(proc, freq_div);
    }
    /* fallback to normal timer process */
#endif
    register_timer_process(proc);
    return false;
}

bool Scheduler::_register_timesliced_proc(AP_HAL::MemberProc proc,
                                          uint8_t freq_div)
{
    unsigned int i, j;
    uint8_t distance, min_distance, best_distance;
    uint8_t best_timeslot;

    if (_num_timesliced_procs > LINUX_SCHEDULER_MAX_TIMESLICED_PROCS) {
        hal.console->printf("Out of timesliced processes\n");
        return false;
    }

    /* if max_freq_div increases, update the timeslots accordingly */
    if (freq_div > _max_freq_div) {
        for (i = 0; i < _num_timesliced_procs; i++) {
            _timesliced_proc[i].timeslot =  _timesliced_proc[i].timeslot
                                            / _max_freq_div * freq_div;
        }
        _max_freq_div = freq_div;
    }

    best_distance = 0;
    best_timeslot = 0;

    /* Look for the timeslot that maximizes the min distance with other timeslots */
    for (i = 0; i < _max_freq_div; i++) {
        min_distance = _max_freq_div;
        for (j = 0; j < _num_timesliced_procs; j++) {
            distance = std::min(i - _timesliced_proc[j].timeslot,
                            _max_freq_div + _timesliced_proc[j].timeslot - i);
            if (distance < min_distance) {
                min_distance = distance;
                if (min_distance == 0) {
                    break;
                }
            }
        }
        if (min_distance > best_distance) {
            best_distance = min_distance;
            best_timeslot = i;
        }
    }

    _timesliced_proc[_num_timesliced_procs].proc = proc;
    _timesliced_proc[_num_timesliced_procs].timeslot = best_timeslot;
    _timesliced_proc[_num_timesliced_procs].freq_div = freq_div;
    _num_timesliced_procs++;
    return true;
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

void Scheduler::suspend_timer_procs()
{
    if (!_timer_semaphore.take(0)) {
        printf("Failed to take timer semaphore\n");
    }
}

void Scheduler::resume_timer_procs()
{
    _timer_semaphore.give();
}

void Scheduler::_timer_task()
{
    int i;

    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_semaphore.take(0)) {
        printf("Failed to take timer semaphore in %s\n", __PRETTY_FUNCTION__);
    }
    // now call the timer based drivers
    for (i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
    //SPI UART use SPI
    if (!((RPIOUARTDriver *)hal.uartC)->isExternal()) {
        ((RPIOUARTDriver *)hal.uartC)->_timer_tick();
    }
#endif

    for (i = 0; i < _num_timesliced_procs; i++) {
        if ((_timeslices_count + _timesliced_proc[i].timeslot)
            % _timesliced_proc[i].freq_div == 0) {
            _timesliced_proc[i].proc();
        }
    }

    if (_max_freq_div != 0) {
        _timeslices_count++;
        if (_timeslices_count == _max_freq_div) {
            _timeslices_count = 0;
        }
    }

    _timer_semaphore.give();

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe();
    }

    _in_timer_proc = false;

#if HAL_LINUX_UARTS_ON_TIMER_THREAD
    /*
       some boards require that UART calls happen on the same
       thread as other calls of the same time. This impacts the
       QFLIGHT calls where UART output is an RPC call to the DSPs
       */
    _run_uarts();
    RCInput::from(hal.rcin)->_timer_tick();
#endif
}

void Scheduler::_run_io(void)
{
    if (!_io_semaphore.take(0)) {
        return;
    }

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
    UARTDriver::from(hal.uartA)->_timer_tick();
    UARTDriver::from(hal.uartB)->_timer_tick();
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
    //SPI UART not use SPI
    if (RPIOUARTDriver::from(hal.uartC)->isExternal()) {
        RPIOUARTDriver::from(hal.uartC)->_timer_tick();
    }
#else
    UARTDriver::from(hal.uartC)->_timer_tick();
#endif
    UARTDriver::from(hal.uartE)->_timer_tick();
    UARTDriver::from(hal.uartF)->_timer_tick();
}

void Scheduler::_rcin_task()
{
#if !HAL_LINUX_UARTS_ON_TIMER_THREAD
    RCInput::from(hal.rcin)->_timer_tick();
#endif
}

void Scheduler::_uart_task()
{
#if !HAL_LINUX_UARTS_ON_TIMER_THREAD
    _run_uarts();
#endif
}

void Scheduler::_tonealarm_task()
{
    // process tone command
    Util::from(hal.util)->_toneAlarm_timer_tick();
}

void Scheduler::_io_task()
{
    // process any pending storage writes
    Storage::from(hal.storage)->_timer_tick();

    // run registered IO processes
    _run_io();
}

bool Scheduler::in_timerprocess()
{
    return _in_timer_proc;
}

void Scheduler::_wait_all_threads()
{
    int r = pthread_barrier_wait(&_initialized_barrier);
    if (r == PTHREAD_BARRIER_SERIAL_THREAD) {
        pthread_barrier_destroy(&_initialized_barrier);
    }
}

void Scheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }

    _initialized = true;

    _wait_all_threads();
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    exit(1);
}

void Scheduler::stop_clock(uint64_t time_usec)
{
    if (time_usec >= _stopped_clock_usec) {
        _stopped_clock_usec = time_usec;
        _run_io();
    }
}

bool Scheduler::SchedulerThread::_run()
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT
    if (_sched._timer_thread.is_current_thread()) {
        /* make rpcmem initialization on timer thread */
        printf("Initialising rpcmem\n");
        rpcmem_init();
    }
#endif

    _sched._wait_all_threads();

    return PeriodicThread::_run();
}
