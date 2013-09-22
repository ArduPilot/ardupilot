
#ifndef __AP_HAL_LINUX_SCHEDULER_H__
#define __AP_HAL_LINUX_SCHEDULER_H__

#include <AP_HAL_Linux.h>
#include <sys/time.h>

class Linux::LinuxScheduler : public AP_HAL::Scheduler {
public:
    LinuxScheduler();
    void     init(void* machtnichts);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::TimedProc);
    void     register_io_process(AP_HAL::TimedProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::TimedProc,
                uint32_t period_us);

    void     begin_atomic();
    void     end_atomic();

    bool     system_initializing();
    void     system_initialized();

    void     panic(const prog_char_t *errormsg);
    void     reboot(bool hold_in_bootloader);

private:
    struct timeval _sketch_start_time;    
};

#endif // __AP_HAL_LINUX_SCHEDULER_H__
