#include "AP_HAL_ESP32/Scheduler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

bool Scheduler::_initialized = true;

Scheduler::Scheduler()
{
    _initialized = false;
}

void Scheduler::init()
{
    _main_task_handle = xTaskGetCurrentTaskHandle();
    vTaskPrioritySet(_main_task_handle, 20);
    xTaskCreate(_timer_thread, "TIMER_THREAD", TIMER_SS, this, TIMER_PRIO, &_timer_task_handle);
    xTaskCreate(_rcin_thread, "RCIN_THREAD", RCIN_SS, this, RCIN_PRIO, &_rcin_task_handle);
    xTaskCreate(_uart_thread, "UART_THREAD", UART_SS, this, UART_PRIO, &_uart_task_handle);
    xTaskCreate(_io_thread, "IO_THREAD", IO_SS, this, IO_PRIO, &_io_task_handle);
    xTaskCreate(_storage_thread, "STORAGE_THREAD", STORAGE_SS, this, STORAGE_PRIO, &_storage_task_handle);
}

void Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

void Scheduler::delay_microseconds(uint16_t us)
{
    vTaskDelay(us/(portTICK_PERIOD_MS * 1000));
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }
    if (_num_timer_procs >= ESP32_SCHEDULER_MAX_TIMER_PROCS) {
        printf("Out of timer processes\n");
        return;
    }
    _timer_sem.take_blocking();
    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
    _timer_sem.give();
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < ESP32_SCHEDULER_MAX_IO_PROCS) {
        _io_sem.take_blocking();
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
        _io_sem.give();
    } else {
        printf("Out of IO processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    printf("Restarting now...\n");
    esp_restart();
}

bool Scheduler::in_main_thread() const
{
    return _main_task_handle == xTaskGetCurrentTaskHandle();
}

void Scheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }

    _initialized = true;
}

void Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        sched->_run_timers();
        // process any pending RC output requests
        //hal.rcout->timer_tick();
        //analog in
        //((AnalogIn *)hal.analogin)->_timer_tick();
    }
}

void Scheduler::_run_timers()
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    int num_procs = 0;

    _timer_sem.take_blocking();
    num_procs = _num_timer_procs;
    _timer_sem.give();

    // now call the timer based drivers
    for (int i = 0; i < num_procs; i++) {
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

void Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(2500);
        //((RCInput *)hal.rcin)->_timer_tick();
    }
}

void Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }

    _in_io_proc = true;

    int num_procs = 0;
    _io_sem.take_blocking();
    num_procs = _num_io_procs;
    _io_sem.give();
    // now call the IO based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }
    _in_io_proc = false;
}

void Scheduler::_io_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        // run registered IO processes
        sched->_run_io();
    }
}

void Scheduler::_storage_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (sched->_initialized) {
        sched->delay_microseconds(10000);
    }
    while (true) {
        sched->delay_microseconds(10000);
        // process any pending storage writes
        hal.storage->_timer_tick();
    }
}

void Scheduler::_uart_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(20000);
    }
    while (true) {
        sched->delay_microseconds(1000);
        hal.uartA->_timer_tick();
        hal.uartB->_timer_tick();
        hal.uartC->_timer_tick();
        hal.uartD->_timer_tick();
    }
}
