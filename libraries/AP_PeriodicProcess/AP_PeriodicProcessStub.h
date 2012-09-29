
#ifndef __AP_PERIODIC_PROCESS_STUB_H__
#define __AP_PERIODIC_PROCESS_STUB_H__

#include "PeriodicProcess.h"
#include "../Arduino_Mega_ISR_Registry/Arduino_Mega_ISR_Registry.h"


class AP_PeriodicProcessStub : public AP_PeriodicProcess
{
public:
    AP_PeriodicProcessStub(uint8_t period = 0);
    void                    init( Arduino_Mega_ISR_Registry * isr_reg );
    void                    register_process(ap_procedure proc);
    void                    set_failsafe(ap_procedure proc);
    bool                    queue_process(ap_procedure proc);   // queue process to run as soon as possible after any currently running ap_processes complete.  returns true if it ran immediately
    void                    suspend_timer(void);
    void                    resume_timer(void);
    bool                    running();
    static void             run(void);
protected:
    static uint8_t          _period;
    static void             (*_proc)(void);
    static void             (*_failsafe)(void);
    static void             (*_queued_proc)(void);
    static bool             _suspended;
};

#endif
