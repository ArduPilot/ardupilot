
#ifndef __AP_HAL_AVR_SCHEDULER_H__
#define __AP_HAL_AVR_SCHEDULER_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::ArduinoScheduler : public AP_HAL::Scheduler {
public:
    ArduinoScheduler() {}
    /* AP_HAL::Scheduler methods */
    void     init();
    void     delay(uint32_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(void(*cb)(void));
};

#endif // __AP_HAL_AVR_SCHEDULER_H__

