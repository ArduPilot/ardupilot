
#include "Scheduler.h"
#include <unistd.h>
#include <sys/time.h>

using namespace Linux;

extern const AP_HAL::HAL& hal;

LinuxScheduler::LinuxScheduler()
{}

void LinuxScheduler::init(void* machtnichts)
{
    gettimeofday(&_sketch_start_time, NULL);
}

void LinuxScheduler::delay(uint16_t ms)
{
    usleep(ms * 1000);
}

uint32_t LinuxScheduler::millis() 
{
    struct timeval tp;
    gettimeofday(&tp,NULL);
    return 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
                  (_sketch_start_time.tv_sec +
                   (_sketch_start_time.tv_usec*1.0e-6)));
}

uint32_t LinuxScheduler::micros() 
{
    struct timeval tp;
    gettimeofday(&tp,NULL);
    return 1.0e6*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
                  (_sketch_start_time.tv_sec +
                   (_sketch_start_time.tv_usec*1.0e-6)));
}

void LinuxScheduler::delay_microseconds(uint16_t us)
{
    usleep(us);
}

void LinuxScheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void LinuxScheduler::register_timer_process(AP_HAL::TimedProc k)
{}

void LinuxScheduler::register_io_process(AP_HAL::TimedProc k)
{}

void LinuxScheduler::register_timer_failsafe(AP_HAL::TimedProc,
            uint32_t period_us)
{}

void LinuxScheduler::suspend_timer_procs()
{}

void LinuxScheduler::resume_timer_procs()
{}

bool LinuxScheduler::in_timerprocess() {
    return false;
}

void LinuxScheduler::begin_atomic()
{}

void LinuxScheduler::end_atomic()
{}

bool LinuxScheduler::system_initializing() {
    return false;
}

void LinuxScheduler::system_initialized()
{}

void LinuxScheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

void LinuxScheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}
