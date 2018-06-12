/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*

(c) 2017 night_ghost@ykoctpa.ru

*/

#pragma GCC optimize ("O2")

#include <AP_HAL_F4Light/AP_HAL_F4Light.h>
#include "Semaphores.h"
#include "Scheduler.h"

using namespace F4Light;

extern const AP_HAL::HAL& hal;

#ifdef SEM_PROF 
uint64_t Semaphore::sem_time=0;    
#endif

// Constructor
Semaphore::Semaphore()
    : _taken(false)
    , _task(NULL)
    , _is_waiting(false)
{}


bool Semaphore::give() {
    if(Scheduler::in_interrupt()) { // SVC from interrupt will cause HardFault, but we need to give 
        bool v=_is_waiting;                 //   bus semaphores from IO_Complete ISR.
        bool ret=svc_give();                // This is atomic and don't breaks anything
        if(v) Scheduler::context_switch_isr(); // if anyone waits for this semaphore then reschedule tasks after interrupt
        return ret;
    }
    return _give(); 
}

bool Semaphore::take_nonblocking() {       
    return _take_nonblocking(); 
}

bool Semaphore::take(uint32_t timeout_ms) {
    uint32_t now=Scheduler::_micros();
    uint32_t dt = timeout_ms*1000;
    bool ret;
    do {     // task switching can be asyncronous but we can't return to caller before take semaphore

        if(Scheduler::in_interrupt()) { // SVC from interrupt will cause HardFault
            ret = svc_take_nonblocking();       //   but this can be called from failsafe_check which executed in ISR context
        } else {
            ret = _take_from_mainloop(timeout_ms);
        }
        if(ret) break;
    }while(Scheduler::_micros()-now < dt || timeout_ms==HAL_SEMAPHORE_BLOCK_FOREVER);
    
    return ret;
}


// realization
bool NAKED Semaphore::_give() {
    asm volatile("svc 1 \r\n"
                 "bx lr \r\n");
}

bool NAKED Semaphore::_take_from_mainloop(uint32_t timeout_ms) {
    asm volatile("svc 2 \r\n"
                 "bx lr \r\n");

}

bool NAKED Semaphore::_take_nonblocking() {
    asm volatile("svc 3 \r\n"
                 "bx lr \r\n");
}


#ifdef SEM_DEBUG
void Semaphore::save_log(enum Sem_OP op, bool result){
    Sem_Log *lp = sem_log[sem_log_ptr++];
    if(sem_log_ptr >= SEM_LOG_SIZE) sem_log_ptr=0;
    
    lp.time=Scheduler::_micros();
    lp.sem  = this;
    lp.task = Scheduler::get_current_task_isr();
    lp.op = op;
    lp.result=result;
    
#endif

// this functions called only at SVC level so serialized by hardware and don't needs to disable interrupts

bool Semaphore::svc_give() {
    _is_waiting=false;
    if (_taken) {
        _taken = false;
        _task = NULL;
#ifdef SEM_DEBUG
        save_log(Sem_Give, true);
#endif
        return true;
    }
#ifdef SEM_DEBUG
        save_log(Sem_Give, false);
#endif
    return false;
}

bool Semaphore::svc_take_nonblocking() {
    void *me = Scheduler::get_current_task_isr();
    if (!_taken) {
        _taken = true;
        _task = me;     // remember task which owns semaphore 
#ifdef SEM_DEBUG
        save_log(Sem_Take_Nonblocking, true);
#endif
        return true;
    }

    if(_task == me){     // the current task already owns this semaphore
#ifdef SEM_DEBUG
        save_log(Sem_Take_Nonblocking, true);
#endif
        return true; 
    }
    _is_waiting=true;
#ifdef SEM_DEBUG
    save_log(Sem_Take_Nonblocking, false);
#endif
    return false;
}

bool Semaphore::svc_take(uint32_t timeout_ms) {
    void *me = Scheduler::get_current_task_isr();
    if (!_taken) {
        _taken = true;
        _task = me; // remember task which owns semaphore 
#ifdef SEM_DEBUG
        save_log(Sem_Take, true);
#endif
        return true;
    }
    if(_task == me){     // the current task already owns this semaphore
#ifdef SEM_DEBUG
        save_log(Sem_Take, true);
#endif
        return true; 
    }
    _is_waiting=true;
#ifdef SEM_DEBUG
    save_log(Sem_Take, false);
#endif
    return false;
}

