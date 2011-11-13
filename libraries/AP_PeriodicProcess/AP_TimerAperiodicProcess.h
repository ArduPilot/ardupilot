
#ifndef __AP_TIMER_APERIODIC_PROCESS_H__
#define __AP_TIMER_APERIODIC_PROCESS_H__

#include <stdint.h>

#include "../Arduino_Mega_ISR_Registry/Arduino_Mega_ISR_Registry.h"
#include "AP_TimerProcess.h"

class AP_TimerAperiodicProcess : public AP_TimerProcess
{
    public:
        void init( Arduino_Mega_ISR_Registry * isr_reg );
        static void run(void);
    private:
        static uint8_t _timer_offset;
};

#endif // __AP_TIMER_APERIODIC_PROCESS_H__
