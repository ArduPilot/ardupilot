/*
(c) 2017 night_ghost@ykoctpa.ru
*/

#pragma GCC optimize ("O2")


#include "Scheduler.h"

#include <stdio.h>
#include <AP_HAL_F4Light/AP_HAL_F4Light.h>


#include "Semaphores.h"
#include "I2CDevice.h"

#include <timer.h>


#include <AP_Notify/AP_Notify.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param_Helper/AP_Param_Helper.h>

#include "RCInput.h"
#include <systick.h>
#include "GPIO.h"
#include <usb.h>


using namespace F4Light;
extern const AP_HAL::HAL& hal;


AP_HAL::Proc  Scheduler::_failsafe  IN_CCM= NULL;

Revo_IO    Scheduler::_io_proc[F4Light_SCHEDULER_MAX_IO_PROCS] IN_CCM;
uint8_t    Scheduler::_num_io_proc IN_CCM=0;

void *   Scheduler::_delay_cb_handle IN_CCM=0;

uint32_t Scheduler::timer5_ovf_cnt IN_CCM=0;

bool Scheduler::_initialized IN_CCM=false;

Handler Scheduler::on_disarm_handler IN_CCM;
task_t * Scheduler::_idle_task IN_CCM;

void *Scheduler::boost_task;

static void loc_ret(){}

#define STACK_GUARD 0x60a4d51aL

// Reference running task
task_t* s_running IN_CCM;
task_t* next_task IN_CCM;
// Main task and run queue
task_t Scheduler::s_main = { 0 }; // NOT in CCM to can't be corrupted by stack
uint16_t Scheduler::task_n=0;


struct Scheduler::IO_COMPLETION Scheduler::io_completion[MAX_IO_COMPLETION] IN_CCM;

uint8_t Scheduler::num_io_completion  IN_CCM= 0;


// Initial top stack for task allocation
size_t Scheduler::s_top IN_CCM; //  = MAIN_STACK_SIZE; - CCM not initialized!


#ifdef SHED_PROF
uint64_t Scheduler::shed_time = 0;
bool     Scheduler::flag_10s = false;
uint64_t Scheduler::task_time IN_CCM = 0;
uint64_t Scheduler::delay_time IN_CCM = 0;
uint64_t Scheduler::delay_int_time IN_CCM = 0;
uint32_t Scheduler::max_loop_time IN_CCM =0;
uint64_t Scheduler::ioc_time IN_CCM =0;
uint64_t Scheduler::sleep_time IN_CCM =0;
uint32_t Scheduler::max_delay_err=0;

uint32_t Scheduler::tick_micros IN_CCM;    // max exec time
uint32_t Scheduler::tick_count IN_CCM;     // number of calls
uint64_t Scheduler::tick_fulltime IN_CCM;  // full consumed time to calc mean
#endif


#ifdef MTASK_PROF
 uint32_t Scheduler::max_wfe_time IN_CCM =0;
 uint32_t Scheduler::tsched_count IN_CCM;
 uint32_t Scheduler::tsched_sw_count IN_CCM;
 uint32_t Scheduler::tsched_count_y IN_CCM;
 uint32_t Scheduler::tsched_sw_count_y IN_CCM;
 uint32_t Scheduler::tsched_count_t IN_CCM;
 uint32_t Scheduler::tsched_sw_count_t IN_CCM;
 #ifdef SHED_DEBUG
  revo_sched_log Scheduler::logbuf[SHED_DEBUG_SIZE] IN_CCM;
  uint16_t Scheduler::sched_log_ptr;
 #endif

uint32_t Scheduler::lowest_stack = (uint32_t)-1;
#endif


bool Scheduler::_in_io_proc IN_CCM =0;
#ifdef MPU_DEBUG
uint32_t Scheduler::MPU_overflow_cnt IN_CCM;
uint32_t Scheduler::MPU_restart_cnt IN_CCM;
uint32_t Scheduler::MPU_count IN_CCM;
uint32_t Scheduler::MPU_Time IN_CCM;
#endif
volatile bool Scheduler::need_switch_task IN_CCM;
task_t *Scheduler::_forced_task IN_CCM;


Scheduler::Scheduler()
{

    s_running = &s_main;         //  CCM don't initialized! - Reference running task
    next_task = &s_main;         
    s_top = MAIN_STACK_SIZE;     // Initial top stack for task allocation

// init main task
    memset(&s_main, 0, sizeof(s_main));

    Revo_handler h = { .vp=loc_ret }; // fake handler to not 0

    s_main.next = &s_main; // linked list
    s_main.prev = &s_main;
    s_main.priority = MAIN_PRIORITY; // base priority
    s_main.active = true;  // not paused
    s_main.handle = h.h;        // to not 0
    s_main.guard = STACK_GUARD; // to check corruption of TCB by stack overflow

#ifdef MTASK_PROF
    s_main.stack_free = (uint32_t) -1;
#endif

    _forced_task = NULL;
}


// to do when nothing to do 
static void idle_task(){
    while(1){
        __WFE(); 
        // see RM090 12.2.3
        TIMER6->regs->SR &= TIMER_SR_UIF;    // reset pending bit
        NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); // timer6 as event generator - reset IRQ
        
        Scheduler::yield(0);
    }
}

void Scheduler::init()
{
    if(in_interrupt()){ // some interrupt caused restart at ISR level
        AP_HAL::panic("HAL initialization on ISR level=0x%x", (uint8_t)(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk));
    }
    memset(_io_proc,      0, sizeof(_io_proc) );
    memset(io_completion, 0, sizeof(io_completion) );


    // The PendSV exception is always enabled so disable interrupts
    // to prevent it from occurring while being configured 
    noInterrupts();

    NVIC_SetPriority(PendSV_IRQn,  PENDSV_INT_PRIORITY);      // lowest priority so all IRQs can't be switced
    NVIC_SetPriority(SVCall_IRQn,  SVC_INT_PRIORITY);         // priority 14 - the same as Timer7 ISR
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_INT_PRIORITY);     // priority 5 - less thah fast device IO ISRs but higher than USB


    // Ensure the effect of the priority change occurs before 
    // clearing PRIMASK to ensure that future PendSV exceptions 
    // are taken at the new priority 
    asm volatile("dsb \n");
    asm volatile("isb \n");

    interrupts();


    CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk)); //we don't need deep sleep
    SET_BIT(  SCB->SCR, ((uint32_t)SCB_SCR_SEVONPEND_Msk)); //we need Event on each interrupt

/*[ DEBUG
    SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk; // disable imprecise exceptions
//]*/

    timer_foreach(timer_reset); // timer_reset(dev) moved out from configTimeBase so reset by hands


    {// timeslice timer, not SYSTICK because we need to restart it by hands
        uint32_t period    = (2000000UL / SHED_FREQ) - 1; 
                // dev    period   freq, kHz
        configTimeBase(TIMER7, period, 2000);       //2MHz 0.5us ticks
        Revo_handler h = { .isr = _timer_isr_event };
        timer_attach_interrupt(TIMER7, TIMER_UPDATE_INTERRUPT, h.h , SVC_INT_PRIORITY); // almost lowest priority, higher than Pend_SW to schedule task switch
        TIMER7->regs->CR1 |=  TIMER_CR1_URS;  // interrupt only by overflow, not by update
        timer_resume(TIMER7);
    }

    {// timer5 - 32-bit general timer, unused for other needs
     // so we can read micros32() directly from its counter and micros64() from counter and overflows
        configTimeBase(TIMER5, 0, 1000);       //1MHz 1us ticks
        timer_set_count(TIMER5,(1000000/SHED_FREQ)/2); // to not interfere with TIMER7
        Revo_handler h = { .isr = _timer5_ovf };
        timer_attach_interrupt(TIMER5, TIMER_UPDATE_INTERRUPT, h.h, MPU_INT_PRIORITY); // high priority
        timer_resume(TIMER5);
    }

    {     // only Timer6 from spare timers has personal NVIC line - TIM6_DAC_IRQn
        uint32_t freq = configTimeBase(TIMER6, 0, 20000);     // 20MHz - we here don't know real freq so can't set period
        timer_set_reload(TIMER6, freq / 1000000);             // period to generate 1uS requests
        timer_enable_irq(TIMER6, TIMER_UPDATE_INTERRUPT);     // enable interrupt requests from timer but not enable them in NVIC - will be events
        timer_resume(TIMER6);
    }

    { // timer to generate more precise delays via quant termination
                // dev    period   freq, kHz
        configTimeBase(TIMER14, 0, 1000);       //1MHz 1us ticks
        Revo_handler h = { .isr = _tail_timer_event };
        timer_attach_interrupt(TIMER14, TIMER_UPDATE_INTERRUPT, h.h , SVC_INT_PRIORITY); // priority 14 - the same as Timer7 and SVC
        TIMER14->regs->CR1 &= ~(TIMER_CR1_ARPE | TIMER_CR1_URS); // not buffered preload, interrupt by overflow or by UG set
    }


    { // timer to generate interrupt for driver's IO_Completion
                // dev    period   freq, kHz
        configTimeBase(TIMER13, 0, 1000);       //1MHz 1us ticks
        Revo_handler h = { .isr = _ioc_timer_event };
        timer_attach_interrupt(TIMER13, TIMER_UPDATE_INTERRUPT, h.h , IOC_INT_PRIORITY); // priority 12
        TIMER13->regs->CR1 &= ~(TIMER_CR1_ARPE | TIMER_CR1_URS); // not buffered preload, interrupt by overflow or by UG set
    }

    void *task = _start_task((uint32_t)idle_task, 256); // only for one context
    set_task_priority(task, 255);                       // lowest possible, to fill delay()
    _idle_task=(task_t *)task;
    set_task_active(task);                              // tasks are created paused so run it
}



// it can't be started on init() because should be stopped in later_init()
void Scheduler::start_stats_task(){
#ifdef DEBUG_BUILD
// show stats output each 10 seconds
    Revo_handler h = { .vp = _set_10s_flag };
    void *task = _register_timer_task(10000000, h.h, NULL);
    set_task_priority(task, IO_PRIORITY+1); // lower than IO_thread
#endif

// task list is filled. so now we can do a trick -
//    dequeue_task(_idle_task); // exclude idle task from task queue, it will be used by direct link.
                              //  its own .next still shows to next task so no problems will. This works but...

}

void Scheduler::_delay(uint16_t ms)
{
    uint32_t start = _micros();
#ifdef SHED_PROF
    uint32_t t=start;
#endif

    uint32_t dt = ms * 1000;
    uint32_t now;

    while((now=_micros()) - start < dt) {
        if (hal.scheduler->_min_delay_cb_ms <= ms) { // MAVlink callback uses 5ms
            hal.scheduler->call_delay_cb();
            yield(1000 - (_micros() - now)); // to not stop MAVlink callback
        } else {
            yield(dt); // for full time
        }
    }


#ifdef SHED_PROF
    uint32_t us=_micros()-t;
    delay_time     +=us;

#endif
}

// also see resume_boost()
// this used from InertialSensor only
void Scheduler::_delay_microseconds_boost(uint16_t us){
    boost_task=get_current_task();

#ifdef SHED_PROF
    uint32_t t = _micros(); 
#endif

    yield(us); // yield raises priority by 6 so task will be high-priority for 1st time

#ifdef SHED_PROF
    uint32_t r_us=_micros()-t; // real time
    delay_time     +=r_us;
#endif
    boost_task=NULL;
}

#define NO_YIELD_TIME 8 // uS

void Scheduler::_delay_microseconds(uint16_t us)
{

#ifdef SHED_PROF
    uint32_t t = _micros(); 
#endif
    
    uint16_t no_yield_t;    // guard time for main process
    no_yield_t=NO_YIELD_TIME;

    if(us > no_yield_t){
        yield(us);

 #ifdef SHED_PROF
        uint32_t r_us=_micros()-t; // real time
        delay_time     +=r_us;
 #endif

    }else{
        _delay_us_ny(us);
    }

}

void Scheduler::_delay_us_ny(uint16_t us){ // precise no yield delay
    uint32_t rtime = stopwatch_getticks(); // get start ticks first

    uint32_t dt = us_ticks * us;  // delay time in ticks
    
    while ((stopwatch_getticks() - rtime) < dt) {
        // __WFE(); -- not helps very much
    }    

#ifdef SHED_PROF
    delay_time     +=us;
    
#endif

}

void Scheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms)
{
    static bool init_done=false;
    if(!init_done){     // small hack to load HAL parameters in needed time

        ((HAL_F4Light&) hal).lateInit();
        
        init_done=true;
    }

    AP_HAL::Scheduler::register_delay_callback(proc, min_time_ms);


/* 
1 - it should run in delay() only 
2 - it should be removed after init done
    if(proc) {
        _delay_cb_handle = start_task(proc);
    } else {
        stop_task(_delay_cb_handle);
    }
*/
}


void Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    // now call the IO based drivers. TODO: per-task stats
    for (int i = 0; i < _num_io_proc; i++) {
        if (_io_proc[i].h) {
            revo_call_handler(_io_proc[i].h,0);
            if(_io_proc[i].flags == IO_ONCE){
                _io_proc[i].h = 0;
            }
        }
    }

    _in_io_proc = false;
}


void Scheduler::_register_io_process(Handler h, Revo_IO_Flags flags)
{
    if(_num_io_proc>=F4Light_SCHEDULER_MAX_IO_PROCS) return;

    if(_num_io_proc==0){
        void *task = start_task(_run_io, IO_STACK_SIZE);
        set_task_period(task, 1000); 
        set_task_priority(task, IO_PRIORITY); 
    }

    uint8_t i;
    for(i=0; i<_num_io_proc; i++){ // find free slots
        if(_io_proc[i].h == 0) {  // found
            _io_proc[i].h = h; 
            _io_proc[i].flags = flags;
            return;
        }
    }

    i=_num_io_proc++;
    _io_proc[i].h = h;
    _io_proc[i].flags = flags;
}


#ifdef MTASK_PROF
void Scheduler::check_stack(uint32_t sp) { // check for stack usage
    
//    uint32_t * stack = (uint32_t *)sp;

    // Stack frame contains:
    // r0, r1, r2, r3, r12, r14, the return address and xPSR
    // - Stacked R0  = stack[0]
    // - Stacked R1  = stack[1]
    // - Stacked R2  = stack[2]
    // - Stacked R3  = stack[3]
    // - Stacked R12 = stack[4]
    // - Stacked LR  = stack[5]
    // - Stacked PC  = stack[6]
    // - Stacked xPSR= stack[7]
        
    if(ADDRESS_IN_CCM(sp)){
        if(sp<lowest_stack){ lowest_stack=sp; }
    }
}
#endif


void Scheduler::_run_timer_procs(bool called_from_isr) {

    // and the failsafe, if one is setted 
    if (_failsafe) {
        static uint32_t last_failsafe=0;
        uint32_t t=_millis();
        if(t-last_failsafe>10){
            last_failsafe = t+50; // 50ms = 20Hz
            _failsafe();
        }
    }

}

void Scheduler::_timer_isr_event(uint32_t v  /* TIM_TypeDef *tim */) {
#ifdef MTASK_PROF
    uint32_t sp; 
    // Get stack pointer
    asm volatile ("MRS     %0, PSP\n\t"  : "=rm" (sp));

    check_stack(sp);
#endif
    static uint32_t last_timer_procs=0;

    uint32_t now = _micros();

    if(now - last_timer_procs >= 1000) {
        last_timer_procs = now;
        _run_timer_procs(true);
    }


#ifndef MTASK_PROF
    _switch_task();
#else
    
    if(task_n==0 || need_switch_task) return;        // if there no tasks or already planned
    
    next_task = get_next_task();
    tsched_count++;

    if(next_task != s_running) { // if we should switch task
        s_running->sw_type=0;
        tsched_sw_count++;
        plan_context_switch();     // plan context switch after return from ISR
    }
#endif
}

void Scheduler::_timer5_ovf(uint32_t v /* TIM_TypeDef *tim */) {
    timer5_ovf_cnt++;
}

uint64_t Scheduler::_micros64() {
#pragma pack(push, 1)
    union {
        uint64_t t;
        uint32_t w[2];
    } now;
#pragma pack(pop)

    noInterrupts();
    now.w[0] = _micros();
    now.w[1] = timer5_ovf_cnt;
    interrupts();
    return now.t;
}


void Scheduler::system_initialized()
{
#ifndef I_KNOW_WHAT_I_DO
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }
#endif
    _initialized = true;
    
    board_set_rtc_register(0,RTC_SIGNATURE_REG); // clear bootloader flag after init done
}


void Scheduler::_reboot(bool hold_in_bootloader) {

    if(hold_in_bootloader) {
#if 1
        if(is_bare_metal() || hal_param_helper->_boot_dfu) { // bare metal build without bootloader of parameter set

            board_set_rtc_register(DFU_RTC_SIGNATURE, RTC_SIGNATURE_REG);

        } else
#endif
            board_set_rtc_register(BOOT_RTC_SIGNATURE, RTC_SIGNATURE_REG);
    }

    _delay(100);

    NVIC_SystemReset();

    _delay(1000);
}


void Scheduler::reboot(bool hold_in_bootloader) {
    
    hal.console->println("GOING DOWN FOR A REBOOT\r\n");
    _reboot(hold_in_bootloader);
}



#ifdef DEBUG_BUILD

extern "C" {
    extern  void *__brkval;
    extern  void *__brkval_ccm;
}


void Scheduler::_print_stats(){
    static int cnt=0;

    if(flag_10s) {
        flag_10s=false;

#if defined(USE_MPU)
        mpu_disable();      // we need access to all tasks
#endif

        
        uint32_t t=_millis();
        const int Kf=100;
        
        switch(cnt++) {
        
        case 0:{
#ifdef SHED_PROF
            float eff= (task_time)/(float)(task_time+shed_time);

            static float shed_eff=0;

            if(is_zero(shed_eff)) shed_eff = eff;
            else              shed_eff = shed_eff*(1 - 1/Kf) + eff*(1/Kf);

            printf("\nSched stats: uptime %lds\n  %% of full time: %5.2f  Efficiency %5.3f max loop time %ld\n", t/1000, (task_time/10.0)/t /* in percent*/ , shed_eff, max_loop_time);
            printf("delay times: in main %5.2f including in timer %5.2f",         (delay_time/10.0)/t, (delay_int_time/10.0)/t);
            max_loop_time=0;

#ifdef ISR_PROF
            printf("\nISR time %5.2f max %5.2f", (isr_time/10.0/(float)us_ticks)/t, max_isr_time/(float)us_ticks );
            max_isr_time=0;
#endif
#ifdef MPU_DEBUG
            printf("MPU overflows: %ld restarts %ld max samples %ld time %ld\n", MPU_overflow_cnt, MPU_restart_cnt, MPU_count, MPU_Time); MPU_overflow_cnt=0; MPU_restart_cnt=0; MPU_count=0; MPU_Time=0;
#endif

            printf("\nPPM max buffer size: %d\n", RCInput::max_num_pulses); RCInput::max_num_pulses=0;
#endif
                        
        } break;

        case 1:{
#ifdef SHED_PROF 
#endif
    
            }break;
            
        case 2:{
#ifdef MTASK_PROF    
            task_t* ptr = &s_main;

            uint32_t  fc=tsched_count+tsched_count_y+tsched_count_t;
            printf("\nsched time: by timer %5.2f%% sw %5.2f%% in yield %5.2f%% sw %5.2f%% in tails %5.2f%% sw %5.2f%%\n", 100.0*tsched_count/fc, 100.0 * tsched_sw_count/tsched_count, 100.0*tsched_count_y/fc,100.0 * tsched_sw_count_y/tsched_count_y, 100.0*tsched_count_t/fc, 100.0 * tsched_sw_count_t/tsched_count_t);
        
            do {
                printf("task %d (0x%015llx) time: %7.2f%% mean %8.1fuS max %5lduS full %7lduS wait sem. %6lduS free stack 0x%lx\n",  
                          ptr->id, ptr->handle, 100.0 * ptr->time/1000.0 / t, 
                                                              (float)ptr->time / ptr->count, 
                                                                       ptr->max_time, 
                                                                                  ptr->work_time,       ptr->sem_max_wait, ptr->stack_free);
        
                ptr->max_time=0; // reset times
                ptr->work_time=0;
                ptr->sem_max_wait=0;
                ptr->quants=0;
                ptr->quants_time=0;
                ptr->max_paused=0;
                
                ptr = ptr->next;
            } while(ptr != &s_main);
#endif
            }break;

        case 3: {
            uint8_t n = I2CDevice::get_dev_count();
            printf("\nI2C stats\n");
    
            for(uint8_t i=0; i<n; i++){
                I2CDevice * d = I2CDevice::get_device(i);
                if(d){
                    printf("bus %d addr %x errors %ld last error=%d state=%d\n",d->get_bus(), d->get_addr(), d->get_error_count(), d->get_last_error(), d->get_last_error_state());   
                }
            }
            }break;

        case 4: {
            uint32_t heap_ptr = (uint32_t)__brkval; // upper bound of sbrk()
            uint32_t bottom   = (uint32_t)&_sdata;
            
            // 48K after boot 72K while logging on
            printf("\nMemory used: static %ldk full %ldk\n",((uint32_t)&_edata-bottom+1023)/1024, (heap_ptr-bottom+1023)/1024);
            printf("Free stack: %ldk\n",(lowest_stack - (uint32_t)&_eccm)/1024);
            printf("CCM use: %ldk\n",((uint32_t)__brkval_ccm - (uint32_t)&_sccm)/1024);

            } break;
        
        case 5: {
            printf("\nIO completion %7.3f%%\n",  ioc_time/1000.0/t*100);
            uint64_t iot=0;
            for(uint8_t i=0; i<num_io_completion; i++){
                struct IO_COMPLETION &io = io_completion[i];
                
                if(io.handler) {
                    if(io.count){
                        printf("task %llx time %7.3f%% mean %7.3fuS max %lduS\n", io.handler,  100.0 * io.time / t / 1000, (float)io.time/io.count, io.max_time);
                        io.max_time=0; 
                        iot+=io.time;
                    }
                }    
            }
            if(ioc_time)
                printf("IO completion effectiveness=%7.3f%%\n",  100.0 * iot/ioc_time);
        
            }break;
            
        case 6:
        default:
            cnt=0;
            break;
        }
    }
}

void Scheduler::_set_10s_flag(){
    flag_10s=true;
    _print_stats();
}
#endif

/*
[    common implementation of all Device.PeriodicCallback;
*/

AP_HAL::Device::PeriodicHandle Scheduler::_register_timer_task(uint32_t period_us, Handler proc, F4Light::Semaphore *sem){

    // all drivers will runs at individual IO tasks
    void *task = _start_task(proc, SMALL_TASK_STACK);
    if(task){
        set_task_priority(task, DRIVER_PRIORITY);
        set_task_semaphore(task, sem);
        set_task_period(task, period_us); // setting of period allows task to run
    }
    return (AP_HAL::Device::PeriodicHandle)task;
}




bool Scheduler::adjust_timer_task(AP_HAL::Device::PeriodicHandle h, uint32_t period_us)
{
 #pragma GCC diagnostic push
 #pragma GCC diagnostic ignored "-Wcast-align" // yes I know

    task_t *p = (task_t *)h;
 #pragma GCC diagnostic pop
    p->period = period_us;
    return true;
}

bool Scheduler::unregister_timer_task(AP_HAL::Device::PeriodicHandle h)
{
 #pragma GCC diagnostic push
 #pragma GCC diagnostic ignored "-Wcast-align"
    task_t *p = (task_t *)h;
 #pragma GCC diagnostic pop

    noInterrupts(); // 64-bits should be 
    p->handle=0L;
    interrupts();
    return true;
}
// ]


//[ -------- preemptive multitasking --------


#if 0 // once started tasks are never ended

task_t* Scheduler::get_empty_task(){
    task_t* ptr = &s_main;

    do {
        if(ptr->handler == NULL)  return ptr;
        
        ptr = ptr->next;
    } while(ptr != &s_main);

    return NULL;
}

#endif

void Scheduler::stop_task(void *h){
    if(h) {
        task_t *tp = (task_t *)h ;
        noInterrupts();    
        tp->handle = 0;
        interrupts();
    }
}


// task's executor, which calls user's function having semaphore
void Scheduler::do_task(task_t *task) {
    while(1){
        uint32_t t=0;
        if(task->handle && task->active) { // Task Switch occures asyncronously so we should wait until task becomes active again
            task->time_start=_micros();
            if(task->sem) {// if task requires a semaphore - block on it
                if(!task->sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                    yield(0);   // can't be
                    continue;
                }
                revo_call_handler(task->handle, task->id); 
                task->sem->give(); // give semaphore when task finished
            } else {
                revo_call_handler(task->handle, task->id); 
            }
#ifdef MTASK_PROF
            t = _micros()-task->time_start; // execution time
            if(t > task->work_time)  task->work_time=t;
            if(task->t_paused > task->max_paused) {
                task->max_paused = task->t_paused;
            }
            if(task->count_paused>task->max_c_paused) {
                task->max_c_paused = task->count_paused;
            }
            task->count_paused=0;
            task->t_paused=0;
#endif
            task->active=false;                    // turn off active, to know when task is started again. last! or can never give semaphore
            task->curr_prio = task->priority - 6;  // just activated task will have a highest priority for one quant
        }
        yield(0);        // give up quant remainder
    } // endless loop
}

void Scheduler::enqueue_task(task_t &tp) { // add new task to run queue, starting main task
    tp.next = &s_main;  // prepare for insert task into linked list
    tp.prev = s_main.prev;
    tp.id = ++task_n;   // counter - new task is created

    noInterrupts();     // we will break linked list so do it in critical section
    s_main.prev->next = &tp;
    s_main.prev = &tp;
    interrupts();       // now TCB is ready to task scheduler
}

void Scheduler::dequeue_task(task_t &tp) { // remove task from run queue
    noInterrupts();     // we will break linked list so do it in critical section
    tp.prev->next = tp.next;
    tp.next->prev = tp.prev;
    interrupts();       // done
}


// Create task descriptor
uint32_t Scheduler::fill_task(task_t &tp){
    memset(&tp,0,sizeof(tp));

    // fill required fields
    tp.priority  = MAIN_PRIORITY;  // default priority equal to main task
    tp.curr_prio = MAIN_PRIORITY;  // current priority the same
#ifdef MTASK_PROF
    tp.start=_micros(); 
    tp.stack_free = (uint32_t) -1;
#endif
    tp.guard = STACK_GUARD;

    return (uint32_t)&tp;
}

// create task descriptor and context
void * Scheduler::init_task(Handler handler, const uint8_t* stack){
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align" // yes I know

//  task_t *task = (task_t *)((uint32_t)(stack-sizeof(task_t)) & 0xFFFFFFFCUL); // control block below memory top, 4-byte alignment
    task_t *task = (task_t *)((uint32_t)(stack-sizeof(task_t)) & 0xFFFFFFE0UL); // control block below memory top, 32-byte alignment for MMU page
#pragma GCC diagnostic pop

    fill_task(*task);  // fill task descriptor
    task->stack  = stack;

    /*
     * ARM Architecture Procedure Call Standard [AAPCS] requires 8-byte stack alignment.
     * This means that we must get top of stack aligned _after_ context "pushing", at
     * interrupt entry.
     */
    uint32_t *sp =(uint32_t *) (((uint32_t)task - 4) & 0xFFFFFFF8UL); // below TCB

// HW frame
    *(--sp)  = 0x01000000UL;          // xPSR
    //           61000000
    *(--sp)  = ((uint32_t)do_task);   // PC Entry Point - task executor
    *(--sp)  = ((uint32_t)do_task)|1; // LR the same, with thumb bit set
    sp -= 4;                          // emulate "push R12,R3,R2,R1"
    *(--sp)  = (uint32_t)task;        // emulate "push r0"
// SW frame, context saved as  "STMDB     R0!, {R4-R11, LR}"
    *(--sp)  = 0xFFFFFFFDUL;          // emulate "push lr" =exc_return: Return to Thread mode, floating-point context inactive, execution uses PSP after return.
#if 0    
    asm volatile (
        "MOV       R0, %0       \n\t"
        "STMDB     R0!, {R4-R11}\n\t"  : "+rm" (sp) ); // push real registers - they can be global register variables
        "MOV       %0,R0        \n\t"
#else
    sp -= 8;                        // emulate "push R4-R11"
#endif
    task->sp=(uint8_t *)sp;           // set stack pointer of task

    // task is not active so we need not to disable interrupts
    task->handle = handler; // save handler to TCB

    return (void *)task;
}



// create a paused task
void * NOINLINE Scheduler::_start_task(Handler handle, size_t stackSize)
{
    // Check called from main task
    if (!_in_main_thread() ) return NULL; 
    
    if(in_interrupt()) {
        AP_HAL::panic("start_task called from ISR 0x%x", (uint8_t)(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk));
    }

#if defined(USE_MPU)
    mpu_disable();      // we need access to new tasks TCB which can be overlapped by guard page
#endif

    // Adjust stack size with size of task context
    stackSize += sizeof(task_t)+8; // for alignment

    if (s_main.stack == NULL) {       // first call, initialize all task subsystem
        s_main.stack = (const uint8_t*)RAMEND - s_top; // remember bottom of stack of main task on first call
    }

    const uint8_t *sp=(const uint8_t*)s_main.prev->stack; // top of stack for new task    

    task_t *task=(task_t *) init_task(handle, sp);    // give stack top as parameter, will correct later
    sp-= stackSize;               // calc stack bottom
    task->stack = sp;             // correct to bottom of stack
    stack_bottom = (caddr_t)sp;   // and remember for memory allocator
    s_top += stackSize;           // adjust used size at stack top

    enqueue_task(*task);          // task is ready, now we can add new task to run queue
                                  //  task will not be executed because .active==0

    return (void *)task;    // return address of task descriptor as task handle
}

// task should run periodically, period in uS. this will be high-priority task
void Scheduler::set_task_period(void *h, uint32_t period){
    task_t *task = (task_t *)h;

    task->active = false; // will be first started after 'period'
    task->time_start  = _micros();
    task->period = period;
}


#ifdef SHED_DEBUG
static uint16_t next_log_ptr(uint16_t sched_log_ptr){
    uint16_t lp = sched_log_ptr+ 1;
    if(lp >= SHED_DEBUG_SIZE) lp=0;
    return lp;
}
#endif

// exception occures in armed state - try to kill current task, or reboot if this is main task
void Scheduler::_try_kill_task_or_reboot(uint8_t n){
    task_t  *me = s_running; // current task
    uint8_t tmp = task_n;

    if(tmp==0 || me->id == 0) { // no tasks yet or in main task
        board_set_rtc_register(FORCE_APP_RTC_SIGNATURE, RTC_SIGNATURE_REG); // force bootloader to not wait
        _reboot(false);
    }
    stop_task(me); // exclude task from planning
    task_n = 0;    // printf() can call yield while we now between live and death

    printf("\nTaks %d killed by exception %d!\n",me->id, n);

    task_n = tmp;
    next_task = get_next_task();
}

void Scheduler::_go_next_task() {
    plan_context_switch();    

    while(1);
}

void Scheduler::_stop_multitask(){
    task_n = 0;
}


// this function called only from SVC Level ISRs so there is no need to be reentrant
task_t *Scheduler::get_next_task(){
    task_t *me = s_running; // current task
    task_t *task=_idle_task; // task to switch to, idle_task by default


    uint32_t timeFromLast=0;
    uint32_t remains = 0;

    uint32_t partial_quant=(uint32_t)-1;
    task_t *want_tail = NULL;

    uint32_t now =  _micros();
    me->t_yield = now;

#if defined(USE_MPU)
    mpu_disable();      // we need access to all tasks
#endif

    { // isolate dt
#if defined(MTASK_PROF) 
        uint32_t dt =  now - me->start;       // time in task
        if(dt >= me->in_isr) dt -= me->in_isr;  // minus time in interrupts
        else                 dt=0;

        me->time+=dt;                           // calculate sum
        me->quants_time+=dt;
#endif


#ifdef MTASK_PROF
        if(dt > me->max_time) {
            me->max_time = dt; // maximum to show
        }

 #ifdef SHED_DEBUG
        {
            revo_sched_log &lp = logbuf[sched_log_ptr];
            lp.end = now;
            lp.task_id=me->id;
            lp.ttw = me->ttw;
            lp.in_isr = me->in_isr;
            lp.sw_type=me->sw_type;
            sched_log_ptr = next_log_ptr(sched_log_ptr);
            ZeroIt(logbuf[sched_log_ptr]); // clear next
        }
 #endif
#endif
    }

    if(_forced_task) {
        task = _forced_task;
        _forced_task = NULL;
    } else {

        task_t *ptr = me; // starting from current task
        bool was_yield=false;

        while(true) { // lets try to find task to switch to
            ptr = ptr->next; // Next task in run queue will continue

#if !defined(USE_MPU) || 1 
            if(ptr->guard != STACK_GUARD){ // check for TCB is not damaged
                printf("PANIC: stack guard spoiled in process %d (from %d)\n", task->id, me->id);
                dequeue_task(*ptr); // исключить задачу из планирования
                goto skip_task;    // skip this tasks
            }
#endif

            if(!ptr->handle) goto skip_task; // skip finished tasks

            if(ptr->f_yield) { // task wants to give one quant
                ptr->f_yield = false;
                was_yield = true;
                goto skip_task; // skip this tasks
            }

        
            if(ptr->sem_wait) { // task want a semaphore
                if(ptr->sem_wait->is_taken()) { // task blocked on semaphore
                    task_t *own =(task_t *)ptr->sem_wait->get_owner();
                    if(own != ptr) { // owner is another task?
                        uint32_t dt = now - ptr->sem_start_wait;   // time since start waiting
                        if(ptr->sem_time == HAL_SEMAPHORE_BLOCK_FOREVER || dt < ptr->sem_time) {
                            if(own->curr_prio > ptr->curr_prio) {
                                own->curr_prio=ptr->curr_prio;
                            }
                            goto skip_task; 
                        }
                    }       
                } 
                ptr->sem_wait=NULL; // clear semaphore after release
#ifdef MTASK_PROF
                uint32_t st=now-ptr->sem_start_wait;
                if(st>ptr->sem_max_wait) ptr->sem_max_wait=st; // time of semaphore waiting
#endif            
            }
            
        
            if(!ptr->active){ // non-active, is it periodic?
                if(ptr->period){                 
                    timeFromLast = now - ptr->time_start; // time from last run
                    if( timeFromLast < ptr->period) {     //   is less than task's period?
                        remains = ptr->period - timeFromLast;
                        if(remains>4) {
                            if(remains<partial_quant && ptr->curr_prio <= want_tail->curr_prio) { // exclude low-prio tasks
                                partial_quant=remains; // minimal time remains to next task
                                want_tail = ptr;
                            }
                            goto skip_task; 
                        }// else execute task slightly before
                    }
                } else { // non-active non-periodic tasks with manual activation
                    goto skip_task; // should be skipped
                }
                ptr->active=true; // selected task to run, even if it will lose quant by priority

            } else { // обычный тайм слайс

                if(ptr->ttw){// task wants to wait 
                    timeFromLast = now - ptr->t_yield;     // time since that moment
                    if(timeFromLast < ptr->ttw){           // still less than ttw ?
                        remains = ptr->ttw - timeFromLast; // remaining time to wait
                        if(remains>4) { // context switch time
                            if(remains<partial_quant && ptr->curr_prio <= want_tail->curr_prio) {
                                partial_quant=remains;
                                want_tail = ptr;
                            }
                            goto skip_task; 
                        }// else execute task slightly before
                    }
                } 
            }


            if(ptr->curr_prio <= task->curr_prio){ // select the most priority task, round-robin for equal priorities
                // task loose tick
                if(task->priority != 255) { // not for idle task
                    if(task->curr_prio>1)  task->curr_prio--;      // increase priority if task loose tick
// as a result of the rising priority of the waiting task, we do not completely stop the low priority tasks, but only slow them down
// execution, forcing to skip the number of ticks equal to the priority difference. As a result, the low priority task is performed at a lower speed,
// which can be adjusted by changing the priority difference                    
                }
                task = ptr; // winner
            } else { // ptr loose a chance - increase priority
                if(ptr->priority != 255) { // not for idle task
                    if(ptr->curr_prio>1)  ptr->curr_prio--;
                }    
            }
            
skip_task:
        // we should do this check after EACH task so can't use "continue" which skips ALL loop. 
        // And we can't move this to begin of loop because then interrupted task does not participate in the comparison of priorities
            if(ptr == me) {  // 'me' is the task that works now, so full loop - now we have most-priority task so let it run!
                if(was_yield && task == _idle_task) { // task wants to yield() but there is no other tasks
                    was_yield=false;    // reset flag and loop again
                } else {
                    break;  // task found
                }
            }
        }
    }

    // task to run is selected

 #ifdef SHED_DEBUG
    revo_sched_log &lp = logbuf[sched_log_ptr];
    lp.start = now;
    lp.task_id=task->id;
    lp.prio = task->curr_prio;
    lp.active = task->active;
    lp.time_start = task->time_start;
    lp.quant = partial_quant;
    lp.want_tail = want_tail;
    ZeroIt(logbuf[next_log_ptr(sched_log_ptr)]); // clear next
 #endif

    task->curr_prio=task->priority; // reset current priority to default value
    task->ttw=0;        // time to wait is over
#if defined(MTASK_PROF) 
    task->start = now;  // task startup time
    task->in_isr=0; // reset ISR time
    task->count++;     // full count
    task->quants++;    // one-start count

    uint32_t sz =  s_running->sp - s_running->stack;
    if(sz< s_running->stack_free) s_running->stack_free = sz;

#endif

    if(want_tail && want_tail->curr_prio <= task->curr_prio) { // we have a high-prio task that want to be started next in the middle of tick
        if(partial_quant < TIMER_PERIOD-10) { // if time less than tick
            timer_set_count(TIMER14, 0);
            timer_set_reload(TIMER14, partial_quant+2); // +2 to guarantee
            timer_resume(TIMER14);
        }
    }

    return task;
}



/*
    interrupt to reduce timeslice quant
*/
void Scheduler::_tail_timer_event(uint32_t v /*TIM_TypeDef *tim */){
    timer_pause(TIMER14); // stop tail timer
    timer_generate_update(TIMER7); // tick is over
#ifndef MTASK_PROF
    _switch_task();
#else

    if(need_switch_task || task_n==0) return; // already scheduled context switch

    next_task = get_next_task();

    tsched_count_t++;

    if(next_task != s_running) { // if we should switch task
        s_running->sw_type=1;
        tsched_sw_count_t++;
        plan_context_switch();  
    }
#endif
}


void Scheduler::yield(uint16_t ttw) // time to wait 
{
    if(task_n==0 || in_interrupt()) { // SVC causes HardFault if in interrupt so just nothing to do
 #ifdef USE_WFE
        if(ttw) {__WFE(); }
 #endif
        return;
    }

// if yield() called with a time, then task don't want to run all this time so exclude it from time sliceing
    if(ttw) { // ttw cleared on sleep exit so always 0 if not set specially 
        s_running->ttw=ttw;   // if switching tasks occurs between writing and calling svc, we just add an extra tick
    }
    asm volatile("svc 0");
}


/**
   * Return current task stack size.
   * @return bytes
 */
size_t Scheduler::task_stack(){
  unsigned char marker;
  return (&marker - s_running->stack);
}





// register IO completion routine
uint8_t Scheduler::register_io_completion(Handler handler){

    if(num_io_completion < MAX_IO_COMPLETION){
        io_completion[num_io_completion].handler=handler; // no need to disable interrupts because we increment counter later
        io_completion[num_io_completion].request=false;
        return ++num_io_completion;
    }
    return 0;
}

void Scheduler::_ioc_timer_event(uint32_t v){ // isr at low priority to do all IO completion routines
    bool do_it = false;

#ifdef SHED_PROF
    uint32_t full_t=_micros();
#endif

    do {
        do_it = false;
        for(uint8_t i=0; i<num_io_completion; i++){
            IO_Completion &io = io_completion[i];
            
            if(io.request) {
                io.request=false; // ASAP - it can be set again in interrupt. 
                // we don't need to disable interrupts because all drivers has own queue and can survive a skipping of one request
                if(io.handler){
                    do_it=true;

#ifdef SHED_PROF
                    uint32_t t = _micros();
#endif
                    revo_call_handler(io.handler,i); // call it
#ifdef SHED_PROF
                    t = _micros() - t;
                    io.time += t;
                    io.count++;
                    if(t>io.max_time) io.max_time=t;
#endif
                }
            }
        }
    } while(do_it);

#ifdef SHED_PROF
    full_t=_micros() - full_t;
    ioc_time += full_t;
#endif

#ifdef MTASK_PROF
// To exclude the interruption time from the task's time, which it interrupted
    s_running->in_isr += full_t;
#endif

}

#pragma GCC optimize ("O2") // should ALWAYS be -O2 for tail recursion optimization in PendSV_Handler

/* how to configure and schedule a PendSV exception
from http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0395b/CIHJHFJD.html
*/

void PendSV_Handler(){
    Scheduler::need_switch_task=false;
#if defined(USE_MPU)
    // set the guard page for the next task
    mpu_configure_region(MPU_REGION_0,                              
                        (uint32_t)(next_task->stack) & ~31, // end of stack in the guard page
                         MPU_RASR_ATTR_AP_RO_RO | MPU_RASR_ATTR_NON_CACHEABLE | MPU_RASR_SIZE_32);  // disable write access
    
    mpu_enable(MPU_CTRL_PRIVDEFENA); // enable default memory map
#endif 
    __do_context_switch();
}

void SVC_Handler(){
    uint32_t * svc_args;
    
// SVC can't be used from any interrupt so this is only for reliability
    asm volatile (
        "TST lr, #4     \n"  
        "ite eq         \n"
        "MRSEQ %0, MSP  \n"  
        "MRSNE %0, PSP  \n"  
        : "=rm" (svc_args) );

    Scheduler::SVC_Handler(svc_args);
}


// svc executes on same priority as Timer7 ISR so there is no need to prevent interrupts
void Scheduler::SVC_Handler(uint32_t * svc_args){
    //    * Stack contains:    * r0, r1, r2, r3, r12, r14, the return address and xPSR      
    unsigned int svc_number = ((char *)svc_args[6])[-2];    

    bool ret;
    switch(svc_number)    {        
    case 0:            // Handle SVC 00 - yield()
        timer_generate_update(TIMER7); // tick is over
        timer_pause(TIMER14);

        if(s_running->priority!=255){ // not for idle task or low-priority tasks
            if(s_running->priority<IO_PRIORITY && s_running->ttw){ // the task voluntarily gave up its quant and wants delay, so that at the end of the delay it will have the high priority
                s_running->curr_prio = s_running->priority - 6;
            } else {
                s_running->f_yield = true;      // to guarantee that quant will not return even if there is no high priority tasks
            }
        }
        switch_task();
        break;        

    case 1:{            // Handle SVC 01 - semaphore give(semaphore) returns bool
            Semaphore * sem = (F4Light::Semaphore *)svc_args[0];
            bool v=sem->is_waiting();            
            svc_args[0] = sem->svc_give();
            if(v) switch_task(); // switch context to waiting task if any
        }
        break;

    case 2:{            // Handle SVC 02 - semaphore take(semaphore, time) returns bool
            Semaphore * sem = (F4Light::Semaphore *)svc_args[0];
            uint32_t timeout_ms = svc_args[1];
            svc_args[0] = ret = sem->svc_take(timeout_ms);
            if(!ret)  {                                 // if failed - switch context to pause waiting task
                task_t *own = (task_t *)sem->get_owner();
                task_t *curr_task = s_running;
                curr_task->sem_wait = sem;             // semaphore
                curr_task->sem_start_wait = _micros(); // time when waiting starts
                if(timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER){
                    curr_task->sem_time = timeout_ms;
                } else {
                    curr_task->sem_time = timeout_ms*1000;      // time to wait semaphore
                }
                //Increase the priority of the semaphore's owner up to the priority of the current task
                if(own->priority >= curr_task->priority) own->curr_prio = curr_task->priority-1;
                switch_task(); 
            }
        }
        break;

    case 3:{            // Handle SVC 03 - semaphore take_nonblocking(semaphore) returns bool
            Semaphore * sem = (F4Light::Semaphore *)svc_args[0];
            svc_args[0] = ret = sem->svc_take_nonblocking();
            if(!ret)  { // if failed - switch context to give tick to semaphore's owner
                task_t *own = (task_t *)sem->get_owner();
                task_t *curr_task = s_running;
                //Increase the priority of the semaphore's owner up to the priority of the current task
                if(own->priority >= curr_task->priority) own->curr_prio = curr_task->priority-1;

                switch_task(); 
            }
        }
        break;
        
    case 4: // whats more we can do via SVC?

    default:                // Unknown SVC - just ignore
        break;    
    }
}

// prepare task switch and plan it if needed. This function called only on ISR level 14
void Scheduler::switch_task(){
    timer_pause(TIMER14);          // we will recalculate scheduling
    timer_generate_update(TIMER7); // tick is over
    _switch_task();
}

void Scheduler::_switch_task(){
    if(need_switch_task || task_n==0) return; // already scheduled context switch

    next_task = get_next_task(); // 2.5uS mean full time

#ifdef MTASK_PROF
    tsched_count_y++;
#endif

    if(next_task != s_running) { // if we should switch task
#ifdef MTASK_PROF
        s_running->sw_type=2;
        tsched_sw_count_y++;
#endif
        plan_context_switch();   
    }
#ifdef MTASK_PROF
      else if(next_task == _idle_task){ // the same idle task
        tsched_count_y--; // don't count loops in idle task
    }
#endif
}

////////////////////////////////////
/*
union Revo_handler { // blood, bowels, assembler :) transform functors into a unified view for calling from C
    voidFuncPtr vp;
    AP_HAL::MemberProc mp;          this is C not C ++, so we can not declare the support of functors explicitly, and are forced to pass
    uint64_t h; // treat as handle             <-- as 64-bit integer
    uint32_t w[2]; // words, to check. if this is a functor then the high is the address of the flash and the lower one is the address in RAM.
                                       if this is a function pointer then lower word is an address in flash and high is 0
};
*/

void revo_call_handler(uint64_t hh, uint32_t arg){
    Revo_handler h = { .h = hh };

    if(ADDRESS_IN_FLASH(h.w[0])){
        (h.isr)(arg);
    } else if(ADDRESS_IN_FLASH(h.w[1])) {
        (h.mpa)(arg);
    }
}

void hal_yield(uint16_t ttw){ Scheduler::yield(ttw); }
void hal_delay(uint16_t t){   Scheduler::_delay(t); }
void hal_delay_microseconds(uint16_t t){ Scheduler::_delay_microseconds(t);}

uint32_t hal_micros() { return Scheduler::_micros(); }
void hal_isr_time(uint32_t t) { s_running->in_isr += t; }

// task management for USB and another C code
void hal_set_task_active(void * h) { Scheduler::set_task_active(h); } 
void hal_context_switch_isr() { Scheduler::context_switch_isr(); }
void hal_set_task_priority(void * h, uint8_t prio) {Scheduler::set_task_priority(h, prio); }
void * hal_register_task(voidFuncPtr task, uint32_t stack) {
    Revo_handler r = { .vp=task };
    return Scheduler::_start_task(r.h, stack);
}

bool hal_is_armed() { return hal.util->get_soft_armed();  }
void hal_try_kill_task_or_reboot(uint8_t n) { Scheduler::_try_kill_task_or_reboot(n); }
void hal_go_next_task() { Scheduler::_go_next_task(); }
void hal_stop_multitask() { Scheduler::_stop_multitask(); }
