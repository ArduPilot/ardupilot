
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_F4Light_Namespace.h"
#include <exti.h>
#include "Config.h"

#ifdef SEM_DEBUG
#define SEM_LOG_SIZE 500
enum Sem_OP {
    Sem_Give,
    Sem_Take,
    Sem_Take_Nonblocking
};

class F4Light::Semaphore;

typedef struct SEM_LOG {
    uint32_t time;
    F4Light::Semaphore *sem;
    void * task;
    enum Sem_OP op;
    bool result;
} Sem_Log;

Sem_Log sem_log[SEM_LOG_SIZE];

#endif


class F4Light::Semaphore : public AP_HAL::Semaphore {
public: // interface
    Semaphore();
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();

//[ functions that called by scheduler only in SVC level so need not to disable interrupts
    bool svc_give();
    bool svc_take(uint32_t timeout_ms);
    bool svc_take_nonblocking();
    inline void *get_owner() { return _task; }       // task that owns this semaphore
    inline bool is_taken()   { return _taken; }
    inline bool is_waiting() { return _is_waiting; } // does anyone want this semaphore when it was busy?
//]

#ifdef SEM_DEBUG
    static Sem_Log  sem_log[SEM_LOG_SIZE];
    static uint16_t sem_log_ptr;
#endif

protected:
    bool _take_from_mainloop(uint32_t timeout_ms);
    bool _take_nonblocking();
    bool _give();

    volatile bool _taken;
    void * _task; // owner
    bool _is_waiting;
};


