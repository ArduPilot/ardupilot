
#ifndef __AP_HAL_AVR_SCHEDULER_H__
#define __AP_HAL_AVR_SCHEDULER_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::ArduinoScheduler : public AP_HAL::Scheduler {
public:
    ArduinoScheduler() {}
    /* AP_HAL::Scheduler methods */
    void          init();
    void          delay(unsigned long ms);
    unsigned long millis();
    unsigned long micros();
    void          delayMicroseconds(unsigned int us);
};

#endif // __AP_HAL_AVR_SCHEDULER_H__

