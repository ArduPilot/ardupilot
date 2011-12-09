
#ifndef __AP_TIMERPROCESS_H__
#define __AP_TIMERPROCESS_H__

#include "PeriodicProcess.h"
#include "../Arduino_Mega_ISR_Registry/Arduino_Mega_ISR_Registry.h"

/* XXX this value is a total guess, will look up. */
#define TIMERPROCESS_PER_DEFAULT (256)

#define AP_TIMERPROCESS_MAX_PROCS 3

class AP_TimerProcess : public AP_PeriodicProcess
{
    public:
        AP_TimerProcess(int period = TIMERPROCESS_PER_DEFAULT);
        void init( Arduino_Mega_ISR_Registry * isr_reg );
        void register_process(void (* proc)(void));
        static void run(void);
    protected:
        static int _period;
        static ap_procedure _proc[AP_TIMERPROCESS_MAX_PROCS];
        static int _pidx;
};

#endif // __AP_TIMERPROCESS_H__
