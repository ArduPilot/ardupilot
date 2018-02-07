#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_HAL_PX4.h"
#include "Scheduler.h"

#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>
#include <drivers/drv_hrt.h>
#include <nuttx/arch.h>
#include <systemlib/systemlib.h>
#include <pthread.h>
#include <poll.h>

#include "UARTDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "RCOutput.h"
#include "RCInput.h"

#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#if HAL_WITH_UAVCAN
#include "CAN.h"
#include <AP_UAVCAN/AP_UAVCAN.h>
#endif

using namespace PX4;

extern const AP_HAL::HAL& hal;

extern bool _px4_thread_should_exit;

PX4Scheduler::PX4Scheduler() :
    _perf_timers(perf_alloc(PC_ELAPSED, "APM_timers")),
    _perf_io_timers(perf_alloc(PC_ELAPSED, "APM_IO_timers")),
    _perf_storage_timer(perf_alloc(PC_ELAPSED, "APM_storage_timers")),
    _perf_delay(perf_alloc(PC_ELAPSED, "APM_delay"))
{}

void PX4Scheduler::init()
{
    _main_task_pid = getpid();

    // setup the timer thread - this will call tasks at 1kHz
    pthread_attr_t thread_attr;
    struct sched_param param;

    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 2048);

    param.sched_priority = APM_TIMER_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_timer_thread_ctx, &thread_attr, &PX4Scheduler::_timer_thread, this);

    // the UART thread runs at a medium priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 2048);

    param.sched_priority = APM_UART_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_uart_thread_ctx, &thread_attr, &PX4Scheduler::_uart_thread, this);

    // the IO thread runs at lower priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 2048);

    param.sched_priority = APM_IO_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_io_thread_ctx, &thread_attr, &PX4Scheduler::_io_thread, this);

    // the storage thread runs at just above IO priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 1024);

    param.sched_priority = APM_STORAGE_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_storage_thread_ctx, &thread_attr, &PX4Scheduler::_storage_thread, this);
}

void PX4Scheduler::create_uavcan_thread()
{
#if HAL_WITH_UAVCAN
    pthread_attr_t thread_attr;
    struct sched_param param;

     //the UAVCAN thread runs at medium priority
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 8192);

    param.sched_priority = APM_CAN_PRIORITY;
    (void) pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        if (hal.can_mgr[i] != nullptr) {
            if (hal.can_mgr[i]->get_UAVCAN() != nullptr) {
                _uavcan_thread_arg *arg = new _uavcan_thread_arg;
                arg->sched = this;
                arg->uavcan_number = i;

                pthread_create(&_uavcan_thread_ctx, &thread_attr, &PX4Scheduler::_uavcan_thread, arg);
            }
        }
    }
#endif
}

/**
   delay for a specified number of microseconds using a semaphore wait
 */
void PX4Scheduler::delay_microseconds_semaphore(uint16_t usec)
{
    sem_t wait_semaphore;
    struct hrt_call wait_call;
    sem_init(&wait_semaphore, 0, 0);
    memset(&wait_call, 0, sizeof(wait_call));
    hrt_call_after(&wait_call, usec, (hrt_callout)sem_post, &wait_semaphore);
    sem_wait(&wait_semaphore);
}

void PX4Scheduler::delay_microseconds(uint16_t usec)
{
    perf_begin(_perf_delay);
    delay_microseconds_semaphore(usec);
    perf_end(_perf_delay);
}

/*
  wrapper around sem_post that boosts main thread priority
 */
static void sem_post_boost(sem_t *sem)
{
    hal_px4_set_priority(APM_MAIN_PRIORITY_BOOST);
    sem_post(sem);
}

/*
  return the main thread to normal priority
 */
static void set_normal_priority(void *sem)
{
    hal_px4_set_priority(APM_MAIN_PRIORITY);
}

/*
  a variant of delay_microseconds that boosts priority to
  APM_MAIN_PRIORITY_BOOST for APM_MAIN_PRIORITY_BOOST_USEC
  microseconds when the time completes. This significantly improves
  the regularity of timing of the main loop as it takes
 */
void PX4Scheduler::delay_microseconds_boost(uint16_t usec)
{
    sem_t wait_semaphore;
    static struct hrt_call wait_call;
    sem_init(&wait_semaphore, 0, 0);
    hrt_call_after(&wait_call, usec, (hrt_callout)sem_post_boost, &wait_semaphore);
    sem_wait(&wait_semaphore);
    hrt_call_after(&wait_call, APM_MAIN_PRIORITY_BOOST_USEC, (hrt_callout)set_normal_priority, nullptr);
}

void PX4Scheduler::delay(uint16_t ms)
{
    if (!in_main_thread()) {
        ::printf("ERROR: delay() from timer process\n");
        return;
    }
    perf_begin(_perf_delay);
    uint64_t start = AP_HAL::micros64();

    while ((AP_HAL::micros64() - start)/1000 < ms &&
           !_px4_thread_should_exit) {
        delay_microseconds_semaphore(1000);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
    perf_end(_perf_delay);
    if (_px4_thread_should_exit) {
        exit(1);
    }
}

void PX4Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                            uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void PX4Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < PX4_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void PX4Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < PX4_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void PX4Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void PX4Scheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}

void PX4Scheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

void PX4Scheduler::reboot(bool hold_in_bootloader)
{
    // disarm motors to ensure they are off during a bootloader upload
    hal.rcout->force_safety_on();
    hal.rcout->force_safety_no_wait();

    // delay to ensure the async force_saftey operation completes
    delay(500);

    px4_systemreset(hold_in_bootloader);
}

void PX4Scheduler::_run_timers(bool called_from_timer_thread)
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i]) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    // process analog input
    ((PX4AnalogIn *)hal.analogin)->_timer_tick();

    _in_timer_proc = false;
}

extern bool px4_ran_overtime;

void *PX4Scheduler::_timer_thread(void *arg)
{
    PX4Scheduler *sched = (PX4Scheduler *)arg;
    uint32_t last_ran_overtime = 0;

    pthread_setname_np(pthread_self(), "apm_timer");

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }
    while (!_px4_thread_should_exit) {
        sched->delay_microseconds_semaphore(1000);

        // run registered timers
        perf_begin(sched->_perf_timers);
        sched->_run_timers(true);
        perf_end(sched->_perf_timers);

        // process any pending RC output requests
        hal.rcout->timer_tick();

        // process any pending RC input requests
        ((PX4RCInput *)hal.rcin)->_timer_tick();

        if (px4_ran_overtime && AP_HAL::millis() - last_ran_overtime > 2000) {
            last_ran_overtime = AP_HAL::millis();
#if 0
            printf("Overtime in task %d\n", (int)AP_Scheduler::current_task);
            hal.console->printf("Overtime in task %d\n", (int)AP_Scheduler::current_task);
#endif
        }
    }
    return nullptr;
}

void PX4Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i]) {
                _io_proc[i]();
            }
        }
    }

    _in_io_proc = false;
}

void *PX4Scheduler::_uart_thread(void *arg)
{
    PX4Scheduler *sched = (PX4Scheduler *)arg;

    pthread_setname_np(pthread_self(), "apm_uart");

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }
    while (!_px4_thread_should_exit) {
        sched->delay_microseconds_semaphore(1000);

        // process any pending serial bytes
        hal.uartA->_timer_tick();
        hal.uartB->_timer_tick();
        hal.uartC->_timer_tick();
        hal.uartD->_timer_tick();
        hal.uartE->_timer_tick();
        hal.uartF->_timer_tick();
    }
    return nullptr;
}

void *PX4Scheduler::_io_thread(void *arg)
{
    PX4Scheduler *sched = (PX4Scheduler *)arg;

    pthread_setname_np(pthread_self(), "apm_io");

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }
    while (!_px4_thread_should_exit) {
        sched->delay_microseconds_semaphore(1000);

        // run registered IO processes
        perf_begin(sched->_perf_io_timers);
        sched->_run_io();
        perf_end(sched->_perf_io_timers);
    }
    return nullptr;
}

void *PX4Scheduler::_storage_thread(void *arg)
{
    PX4Scheduler *sched = (PX4Scheduler *)arg;

    pthread_setname_np(pthread_self(), "apm_storage");

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }
    while (!_px4_thread_should_exit) {
        sched->delay_microseconds_semaphore(10000);

        // process any pending storage writes
        perf_begin(sched->_perf_storage_timer);
        hal.storage->_timer_tick();
        perf_end(sched->_perf_storage_timer);
    }
    return nullptr;
}

#if HAL_WITH_UAVCAN
void *PX4Scheduler::_uavcan_thread(void *arg)
{
    PX4Scheduler *sched = ((_uavcan_thread_arg *) arg)->sched;
    uint8_t uavcan_number = ((_uavcan_thread_arg *) arg)->uavcan_number;

    char name[15];
    snprintf(name, sizeof(name), "apm_uavcan:%u", uavcan_number);
    pthread_setname_np(pthread_self(), name);

    while (!sched->_hal_initialized) {
        poll(nullptr, 0, 1);
    }

    while (!_px4_thread_should_exit) {
        if (((PX4CANManager *)hal.can_mgr[uavcan_number])->is_initialized()) {
            if (((PX4CANManager *)hal.can_mgr[uavcan_number])->get_UAVCAN() != nullptr) {
                (((PX4CANManager *)hal.can_mgr[uavcan_number])->get_UAVCAN())->do_cyclic();
            } else {
                sched->delay_microseconds_semaphore(10000);
            }
        } else {
            sched->delay_microseconds_semaphore(10000);
        }
    }

    return nullptr;
}
#endif

bool PX4Scheduler::in_main_thread() const
{
    return getpid() == _main_task_pid;
}

void PX4Scheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                      "more than once");
    }
    _initialized = true;
}

#endif
