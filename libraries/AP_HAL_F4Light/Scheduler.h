#pragma once

//#pragma GCC optimize ("O2")

#include <AP_HAL/AP_HAL.h>


#include "AP_HAL_F4Light_Namespace.h"
#include "handler.h"
#include "Config.h"

#include "Semaphores.h"
#include "GPIO.h"

#include <delay.h>
#include <systick.h>
#include <boards.h>
#include <timer.h>
//#include <setjmp.h>

#define F4Light_SCHEDULER_MAX_IO_PROCS 10


#define MAIN_PRIORITY  100  // priority for main task
#define DRIVER_PRIORITY 98  // priority for drivers, speed of main will be 1/4 of this
#define IO_PRIORITY    115  // main task has 100 so IO tasks will use 1/16 of CPU

#define SHED_FREQ 10000   // timer's freq in Hz
#define TIMER_PERIOD 100  // task timeslice period in uS


#define MAIN_STACK_SIZE     4096U+1024U // measured use of stack is only 1.5K - but it grows up to 3K when using FatFs, also this includes 1K stack for ISR
#define IO_STACK_SIZE       4096U   // IO_tasks stack size - io_thread can do work with filesystem, stack overflows if 2K
#define DEFAULT_STACK_SIZE  1024U   // Default tasks stack size 
#define SMALL_TASK_STACK    1024U   // small stack for sensors
#define STACK_MAX          65536U

#if 1
 #define EnterCriticalSection __set_BASEPRI(SVC_INT_PRIORITY << (8 - __NVIC_PRIO_BITS))
 #define LeaveCriticalSection __set_BASEPRI(0)
#else
 #define EnterCriticalSection noInterrupts()
 #define LeaveCriticalSection interrupts()
#endif


/*
 * Task run-time structure (Task control block AKA TCB)
 */
struct task_t {
        const uint8_t* sp;              // Task stack pointer, should be first to access from context switcher
        task_t* next;                   // Next task (double linked list)
        task_t* prev;                   // Previous task
        Handler handle;                 // loop() in Revo_handler - to allow to change task, called via revo_call_handler
        const uint8_t* stack;           // Task stack bottom
        uint8_t id;                     // id of task
        uint8_t priority;               // base priority of task
        uint8_t curr_prio;              // current priority of task, usually higher than priority
        bool active;                    // task still not ended
        bool f_yield;                   // task gives its quant voluntary (to not call it again)
        uint32_t ttw;                   // time to wait - for delays and IO
        uint32_t t_yield;               // time when task loose control
        uint32_t period;                // if set then task will start on time basis
        uint32_t time_start;            // start time of task (for periodic tasks)
        F4Light::Semaphore *sem;       // task should start after owning this semaphore
        F4Light::Semaphore *sem_wait;  // task is waiting this semaphore
        uint32_t sem_time;              // max time to wait semaphore
        uint32_t sem_start_wait;        // time when waiting starts (can use t_yield but stays for clarity)
#if defined(MTASK_PROF)
        uint32_t start;         // microseconds of timeslice start
        uint32_t in_isr;        // time in ISR when task runs
        uint32_t def_ttw;       // default TTW - not as hard as period
        uint8_t sw_type;        // type of task switch
        uint64_t time;          // full time
        uint32_t max_time;      //  maximal execution time of task - to show
        uint32_t count;         // call count to calc mean
        uint32_t work_time;     // max time of full task
        uint32_t sem_max_wait;  // max time of semaphore waiting
        uint32_t quants;        // count of ticks
        uint32_t quants_time;   // sum of quatn's times
        uint32_t t_paused;      // time task was paused on IO
        uint32_t count_paused;  // count task was paused on IO
        uint32_t max_paused;    // max time task was paused on IO
        uint32_t max_c_paused;  // count task was paused on IO
        uint32_t stack_free;    // free stack
#endif
        uint32_t guard; // stack guard to check TCB corruption
};

extern "C" {
    extern unsigned _estack; // defined by link script
    extern uint32_t us_ticks;
    extern void *_sdata;
    extern void *_edata;
    extern void *_sccm;  // start of CCM
    extern void *_eccm;  // end of CCM vars

    void revo_call_handler(Handler hh, uint32_t arg); // universal caller for all type handlers - memberProc and Proc

    extern voidFuncPtr boardEmergencyHandler; // will be called on any fault or panic() before halt
    void PendSV_Handler();
    void SVC_Handler();
    void getNextTask();

    void switchContext();
    void __do_context_switch();
    void hal_try_kill_task_or_reboot(uint8_t n);
    void hal_go_next_task();
    void hal_stop_multitask();

    extern task_t *s_running; // running task 
    extern task_t *next_task; // task to run next

    extern caddr_t stack_bottom; // for SBRK check
    
    bool hal_is_armed();
    
// publish to low-level functions
    void hal_yield(uint16_t ttw);
    void hal_delay(uint16_t t);
    void hal_delay_microseconds(uint16_t t);
    uint32_t hal_micros();
    void hal_isr_time(uint32_t t);
    
// task management for USB MSC mode
    void hal_set_task_active(void * handle); 
    void hal_context_switch_isr();
    void * hal_register_task(voidFuncPtr task, uint32_t stack);
    void hal_set_task_priority(void * handle, uint8_t prio);
    
    void enqueue_flash_erase(uint32_t from, uint32_t to);
}


#define RAMEND ((size_t)&_estack)



#ifdef SHED_DEBUG
typedef struct RevoSchedLog {
    uint32_t start;
    uint32_t end;
    uint32_t ttw;
    uint32_t time_start;    
    uint32_t quant;
    uint32_t in_isr;
    task_t  *want_tail;
    uint8_t  task_id;
    uint8_t  prio;
    uint8_t  active;
    uint8_t  sw_type;
} revo_sched_log;

#define SHED_DEBUG_SIZE 512
#endif

enum Revo_IO_Flags {
    IO_PERIODIC= 0,
    IO_ONCE    = 1,
};

typedef struct REVO_IO {
    Handler h;
    Revo_IO_Flags flags;
} Revo_IO;

class F4Light::Scheduler : public AP_HAL::Scheduler {
public:

    typedef struct IO_COMPLETION {
        Handler handler;
        bool request; 
#ifdef SHED_PROF
        uint64_t time;
        uint32_t count;
        uint32_t max_time;
#endif
    } IO_Completion;



    Scheduler();
    void     init();
    inline void     delay(uint16_t ms) { _delay(ms); } // uses internal static methods
    inline void     delay_microseconds(uint16_t us) { _delay_microseconds(us); }
    inline void     delay_microseconds_boost(uint16_t us) override { _delay_microseconds_boost(us); }
    
    inline   uint32_t millis() {    return AP_HAL::millis(); } 
    inline   uint32_t micros() {    return _micros(); }
    
    inline void register_timer_process(AP_HAL::MemberProc proc) { _register_timer_process(proc, 1000); }

    void     register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms) override;
    static void  _register_io_process(Handler h, Revo_IO_Flags flags);
    void          register_io_process(AP_HAL::MemberProc proc) { Revo_handler h = { .mp=proc }; _register_io_process(h.h, IO_PERIODIC); }


    static inline void     _register_timer_process(AP_HAL::MemberProc proc, uint32_t period) {
        Revo_handler r = { .mp=proc };

        _register_timer_task(period, r.h, NULL);
    }

    inline bool in_timerprocess() {   return false; } // we don't calls anything in ISR

    void inline register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) { _failsafe = failsafe; }

    void     system_initialized();

    static void _reboot(bool hold_in_bootloader);
    void        reboot(bool hold_in_bootloader);

    inline bool in_main_thread() const override { return _in_main_thread(); }

// drivers are not the best place for its own sheduler so let do it here
    static inline AP_HAL::Device::PeriodicHandle register_timer_task(uint32_t period_us, AP_HAL::Device::PeriodicCb proc, F4Light::Semaphore *sem) {
        Revo_handler r = { .pcb=proc };
        return _register_timer_task(period_us, r.h, sem);
    }

    static void _delay(uint16_t ms);
    static void _delay_microseconds(uint16_t us);
    static void _delay_microseconds_boost(uint16_t us);

    static void _delay_us_ny(uint16_t us); // no yield delay

    static inline  uint32_t _millis() {    return systick_uptime(); } //systick_uptime returns 64-bit time
    static inline  uint64_t _millis64() {  return systick_uptime(); }

    static inline  uint32_t _micros() {    return timer_get_count32(TIMER5); }
    static         uint64_t _micros64(); 

    
    static bool           adjust_timer_task(AP_HAL::Device::PeriodicHandle h, uint32_t period_us);
    static bool           unregister_timer_task(AP_HAL::Device::PeriodicHandle h);
    void                  loop();      // to add ability to print out scheduler's stats in main thread

    static inline bool in_interrupt(){ return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) /* || (__get_BASEPRI()) */; }


//{ this functions do a preemptive multitask and inspired by Arduino-Scheduler (Mikael Patel), and scmrtos
    
  /**
   * Initiate scheduler and main task with given stack size. Should
   * be called before start of any tasks if the main task requires a
   * stack size other than the default size. Returns true if
   * successful otherwise false.
   * @param[in] stackSize in bytes.
   * @return bool.
   */
    static inline bool adjust_stack(size_t stackSize) {  s_top = stackSize; return true; } 
    
  /**
   * Start a task with given function and stack size. Should be
   * called from main task. The functions are executed by the
   * task. The taskLoop function is repeatedly called. Returns 
   * not-NULL if successful otherwise NULL (no memory for new task).
   * @param[in] taskLoop function to call.
   * @param[in] stackSize in bytes.
   * @return address of TCB.
   */
  static void * _start_task(Handler h,  size_t stackSize);

  static inline void * start_task(voidFuncPtr taskLoop, size_t stackSize = DEFAULT_STACK_SIZE){
        Revo_handler r = { .vp=taskLoop };
        return _start_task(r.h, stackSize);
  }
  static inline void * start_task(AP_HAL::MemberProc proc,  size_t stackSize = DEFAULT_STACK_SIZE){
        Revo_handler r = { .mp=proc };
        return _start_task(r.h, stackSize);
  }
  // not used - tasks are never stopped
  static void stop_task(void * h);

  
// functions to alter task's properties
//[ this functions called only at task start
  static void set_task_period(void *h, uint32_t period);       // task will be auto-activated by this period

  static inline void set_task_priority(void *h, uint8_t prio){ // priority is a relative speed of task
    task_t *task = (task_t *)h;

    task->curr_prio= prio;
    task->priority = prio;
  }

  // task wants to run only with this semaphore owned
  static inline void set_task_semaphore(void *h, F4Light::Semaphore *sem){ // taskLoop function will be called owning this semaphore
    task_t *task = (task_t *)h;

    task->sem = sem;
  }
//]


// this functions are atomic so don't need to disable interrupts
  static inline void *get_current_task() { // get task handler or 0 if called from ISR
    if(in_interrupt()) return NULL;
    return s_running; 
  }
  static inline void *get_current_task_isr() { // get current task handler even if called from ISR
    return s_running; 
  }
  static inline void set_task_active(void *h) {   // tasks are created in stopped state
    task_t * task = (task_t*)h; 
    task->active=true; 
}

    // do context switch after return from interrupt
    static inline void context_switch_isr(){    timer_generate_update(TIMER14);    }


#if defined(MTASK_PROF)
    static void inline task_pause(uint32_t t) {   // called from task when it starts transfer
        s_running->ttw=t;
        s_running->sem_start_wait=_micros();
        s_running->count_paused++;
    }                    
    static void inline task_resume(void *h) {   // called from IO_Complete ISR to resume task
#if defined(USE_MPU)
        mpu_disable();      // we need access to write
#endif
        task_t * task = (task_t*)h; 
        task->ttw=0;  
        task->active=true;
        _forced_task = task; // force it. Thus we exclude loop to select task
        context_switch_isr();
        uint32_t dt= _micros() - task->sem_start_wait;
        task->t_paused += dt;
    } 
#else
    static void inline task_pause(uint16_t t) {   s_running->ttw=t;  }  // called from task when it starts IO transfer
    static void inline task_resume(void *h)   {    // called from IO_Complete ISR to resume task, and will get 1st quant 100%
#if defined(USE_MPU)
        mpu_disable();      // we need access to write
#endif
        task_t * task = (task_t*)h; // called from ISR so don't need disabling interrupts when writes to TCB
        task->ttw=0;  
        task->active=true;
        _forced_task = task; // force it, to not spent time for  search by priority
        context_switch_isr();
    } 
#endif
//]  

/*
    task scheduler. Gives task ready to run with highest priority
*/
    static task_t *get_next_task(); 

/*
 * finish current tick and schedule new task excluding this
 */
    static void yield(uint16_t ttw=0); // optional time to wait
  
  /**
   * Return current task stack size.
   * @return bytes
   */
    static size_t task_stack();
  
    // check from what task it called
    static inline bool _in_main_thread() { return s_running == &s_main; }

    // resume task that called delay_boost()
    static void resume_boost(){
        if(boost_task) {
            task_t *task = (task_t *) boost_task;
            boost_task=NULL;
            
            
            if(task->ttw){ // task now in wait 
#if defined(USE_MPU)
                mpu_disable();      // we need access to write
#endif
                uint32_t now =  _micros();
                uint32_t timeFromLast = now - task->t_yield;     // time since that moment
                if(task->ttw<=100 || timeFromLast > task->ttw*3/2){       // gone 2/3 of time?
                    task->ttw=0;        // stop waiting  
                    task->active=true;
                    _forced_task = task; // force it
                }
            } else {
                _forced_task = task; // just force it
            }
        }
    }

    static inline void plan_context_switch(){
        need_switch_task = true; // require context switch
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; // PENDSVSET    
    }

    static void SVC_Handler(uint32_t * svc_args); // many functions called via SVC for hardware serialization
    
    static void _try_kill_task_or_reboot(uint8_t n); // exception occures in armed state - try to kill current task
    static void _go_next_task();
    static void _stop_multitask();

    static volatile bool need_switch_task;   // should be public for access from C code
//}


//{ IO completion routines, allows to move out time consuming parts from ISR
 #define MAX_IO_COMPLETION 8
    
    typedef voidFuncPtr ioc_proc;

    static uint8_t register_io_completion(Handler handle);

    static inline uint8_t register_io_completion(ioc_proc cb) {
        Revo_handler r = { .vp=cb };
        return register_io_completion(r.h);
    }
    static inline uint8_t register_io_completion(AP_HAL::MemberProc proc) {
        Revo_handler r = { .mp=proc };
        return register_io_completion(r.h);
    }

    static inline void do_io_completion(uint8_t id){ // schedule selected IO completion
        if(id) { 
            io_completion[id-1].request = true;
            timer_generate_update(TIMER13);
        } 
    }

    static void exec_io_completion();

    static volatile bool need_io_completion;
//}


    // helpers
    static inline Handler get_handler(AP_HAL::MemberProc proc){
        Revo_handler h = { .mp = proc };
        return h.h;
    }
    static inline Handler get_handler(AP_HAL::Proc proc){
        Revo_handler h = { .hp = proc };
        return h.h;
    }
        
    static inline void setEmergencyHandler(voidFuncPtr handler) { boardEmergencyHandler = handler; }


#ifdef MPU_DEBUG
    static inline void MPU_buffer_overflow(){ MPU_overflow_cnt++; } 
    static inline void MPU_restarted() {      MPU_restart_cnt++; }
    static inline void MPU_stats(uint16_t count, uint32_t time) {
        if(count>MPU_count) {
            MPU_count=count;
            MPU_Time=time;
        }
    }
#endif

    static inline void arming_state_changed(bool v){ if(!v && on_disarm_handler) revo_call_handler(on_disarm_handler, 0); }
    static inline void register_on_disarm(Handler h){ on_disarm_handler=h; }

    static void start_stats_task(); // it interferes with CONNECT_COM and CONNECT_ESC so should be started last

protected:

//{ multitask
    // executor for task's handler, never called but used when task context formed
    static void do_task(task_t * task);

    // gves first deleted task or NULL - not used because tasks are never finished
    static task_t* get_empty_task();
/*
   * Initiate a task with the given functions and stack. When control
   * is yield to the task then the loop function is repeatedly called.
   * @param[in] h     task handler (may be NULL)
   * @param[in] stack top reference.
*/
    static void *init_task(uint64_t h, const uint8_t* stack);

    static uint32_t fill_task(task_t &tp);      // prepares task's TCB
    static void enqueue_task(task_t &tp);       // add new task to run queue
    static void dequeue_task(task_t &tp);       // remove task from run queue

    // plan context switch
    static void switch_task();
    static void _switch_task();

    static task_t s_main;   // main task TCB
    static size_t s_top;    // Task stack allocation top. 
    static uint16_t task_n; // counter of tasks

    static task_t *_idle_task;   // remember TCB of idle task
    static task_t *_forced_task; // task activated from ISR so should be called without prioritization
    static void *boost_task;     // task that called delay_boost()
    
    static void check_stack(uint32_t sp);
 
#define await(cond) while(!(cond)) yield()
  
//} end of multitask
    
private:
    static AP_HAL::Device::PeriodicHandle _register_timer_task(uint32_t period_us, Handler proc, F4Light::Semaphore *sem);

    static void * _delay_cb_handle;
    static bool _initialized;

    // ISR functions
    static void _timer_isr_event(uint32_t v /*TIM_TypeDef *tim */);
    static void _timer5_ovf(uint32_t v /*TIM_TypeDef *tim */ );
    static void _tail_timer_event(uint32_t v /*TIM_TypeDef *tim */);
    static void _ioc_timer_event(uint32_t v /*TIM_TypeDef *tim */);
    static void _delay_timer_event(uint32_t v /*TIM_TypeDef *tim */);
    
    static void _run_timer_procs(bool called_from_isr);

    static uint32_t timer5_ovf_cnt; // high part of 64-bit time
    
    static AP_HAL::Proc _failsafe; // periodically called from ISR

    static Revo_IO _io_proc[F4Light_SCHEDULER_MAX_IO_PROCS]; // low priority tasks for IO thread
    static void _run_io(void);
    static uint8_t _num_io_proc;
    static bool _in_io_proc;

    static Handler on_disarm_handler;

    static void _print_stats();

    static uint32_t lowest_stack;
    
    static struct IO_COMPLETION io_completion[MAX_IO_COMPLETION];
    static uint8_t num_io_completion;

    
#ifdef SHED_PROF
    static uint64_t shed_time;
    static uint64_t task_time;
    static bool flag_10s;
    
    static uint64_t delay_time;
    static uint64_t delay_int_time;
    static uint32_t max_loop_time;
    
    static void _set_10s_flag();
    static uint64_t ioc_time;
    static uint64_t sleep_time;
    static uint32_t max_delay_err;


    static uint32_t tick_micros;    // max exec time
    static uint32_t tick_count;     // number of calls
    static uint64_t tick_fulltime;  // full consumed time to calc mean

#endif

#ifdef MTASK_PROF
    static uint32_t max_wfe_time;
    static uint32_t tsched_count;
    static uint32_t tsched_sw_count;
    static uint32_t tsched_count_y;
    static uint32_t tsched_sw_count_y;
    static uint32_t tsched_count_t;
    static uint32_t tsched_sw_count_t;


 #ifdef SHED_DEBUG
    static revo_sched_log logbuf[SHED_DEBUG_SIZE];
    static uint16_t sched_log_ptr;
 #endif
#endif




#ifdef MPU_DEBUG
    static uint32_t MPU_overflow_cnt;
    static uint32_t MPU_restart_cnt;
    static uint32_t MPU_count;
    static uint32_t MPU_Time;
#endif
    
};

void revo_call_handler(Handler h, uint32_t arg);


