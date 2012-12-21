
#ifndef __AP_HAL_EMPTY_SCHEDULER_H__
#define __AP_HAL_EMPTY_SCHEDULER_H__

#include <AP_HAL_Empty.h>

class Empty::EmptyScheduler : public AP_HAL::Scheduler {
public:
    EmptyScheduler();
    void     init(void* machtnichts);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);
    void     register_timer_process(AP_HAL::TimedProc);
    void     register_timer_failsafe(AP_HAL::TimedProc,
                uint32_t period_us);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    void     begin_atomic();
    void     end_atomic();

    void     panic(const prog_char_t *errormsg);
    void     reboot();

};

#endif // __AP_HAL_EMPTY_SCHEDULER_H__
