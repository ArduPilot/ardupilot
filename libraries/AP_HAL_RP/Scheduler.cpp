#include "AP_HAL_RP/Scheduler.h"
#include "AP_Math/AP_Math.h"
#include <cstdlib>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"

using namespace RP;

extern const AP_HAL::HAL& hal;

Scheduler::Scheduler() :
    _num_timer_procs(0),
    _num_io_procs(0),
    _failsafe_proc(nullptr),
    _main_task_handle(nullptr),
    _timer_task_handle(nullptr),
    _io_task_handle(nullptr),
    _uart_task_handle(nullptr),
    _callbacks(nullptr),
    _initialized(false) 
{
    // Initialize MemberProc arrays with zeros (nullptr)
    for (uint8_t i = 0; i < RP2350_SCHEDULER_MAX_TIMER_PROCS; i++) {
        _timer_proc[i] = nullptr;
        _io_proc[i] = nullptr;
    }    
}

void Scheduler::init() {
#ifdef SCHEDDEBUG
    DEV_PRINTF("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif

    // Create the main task (Main Loop)
    // High priority for flight stability
    if (xTaskCreate(_main_task, "Ardu_Main", MAIN_SS, this, MAIN_PRIO , &_main_task_handle) == pdPASS) {
        DEV_PRINTF("OK created task _main_task\n");
    }
    else {
        DEV_PRINTF("FAILED to create task _main_task\n");
    }
    // Bind the main task to Core 0 (SMP Affinity)
    // In FreeRTOS SMP mask (1 << 0) means work only on Core 0
    #define CORE_0 (1 << 0)
    #define CORE_1 (1 << 1)
    vTaskCoreAffinitySet(_main_task_handle, CORE_0);

    // Create a timer task on Core 0 for maximum accuracy
    if (xTaskCreate(_timer_task, "Ardu_Timer", TIMER_SS, this, TIMER_PRIO, &_timer_task_handle) == pdPASS) {
        DEV_PRINTF("OK created task _timer_task\n");
    }
    else {
        DEV_PRINTF("FAILED to create task _timer_task\n");
    }
    vTaskCoreAffinitySet(_timer_task_handle, CORE_0);

    // Create the uart task on Core 0
    if (xTaskCreate(_uart_task, "Ardu_UART", UART_SS, this, UART_PRIO, &_uart_task_handle) == pdPASS) {
        DEV_PRINTF("OK created task _uart_task\n");
    }
    else {
        DEV_PRINTF("FAILED to create task _uart_task\n");
    }
    vTaskCoreAffinitySet(_uart_task_handle, CORE_0);

    // Create an IO task on Core 1
    if (xTaskCreate(_io_task, "Ardu_IO", IO_SS, this, IO_PRIO, &_io_task_handle) == pdPASS) {
        DEV_PRINTF("OK created task _io_task\n");
    }
    else {
        DEV_PRINTF("FAILED to create task _io_task\n");
    }
    vTaskCoreAffinitySet(_io_task_handle, CORE_1);

#if 0
    // Create a task for storage on Core 1
    xTaskCreate(_storage_task, "Ardu_Storage", STORAGE_SS, this, STORAGE_PRIO, &_storage_task_handle);
    vTaskCoreAffinitySet(_storage_task_handle, CORE_1);
#endif
}

void Scheduler::_main_task(void *pvParameters) {
    Scheduler *sched = (Scheduler *)pvParameters;

#ifdef SCHEDDEBUG
    DEV_PRINTF("Free RAM: %u\n", (unsigned int)xPortGetFreeHeapSize());
#endif

#if defined(HAL_RCOUT_DRIVER_ENABLED) && HAL_RCOUT_DRIVER_ENABLED == 1
    hal.rcout->init();
#endif
    
    // Call the setup() method of your firmware/test
    sched->_callbacks->setup();
    sched->set_system_initialized();

#ifdef SCHEDDEBUG
    DEV_PRINTF("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif

    for (;;) {
        sched->_callbacks->loop();
        // Yield the processor if the loop completed too quickly
        taskYIELD();
    }
}

void Scheduler::_timer_task(void *pvParameters) {
    // Here is the logic for calling functions registered via register_timer_process
    Scheduler *sched = (Scheduler *)pvParameters;
    for (;;) {
        // Start all timer processes
        for (uint8_t i = 0; i < sched->_num_timer_procs; i++) {
            if (sched->_timer_proc[i]) {
                sched->_timer_proc[i]();
            }
        }

        // Call failsafe (check communication, stop motors)
        if (sched->_failsafe_proc) {
            sched->_failsafe_proc();
        }

        // Typically 1kHz or as configured in ArduPilot
        sched->delay(1);
    }
}

void Scheduler::_io_task(void *pvParameters) {
    Scheduler *sched = (Scheduler *)pvParameters;
    for (;;) {
        for (uint8_t i = 0; i < sched->_num_io_procs; i++) {
            if (sched->_io_proc[i]) {
                sched->_io_proc[i]();
            }
        }
        // IO processes can run less frequently, e.g. 100 Hz
        sched->delay(10);
    }
}

void Scheduler::_uart_task(void *pvParameters) {
    Scheduler *sched = (Scheduler *)pvParameters;

    while (!sched->_initialized) {
        sched->delay(10);
    }

    while (true) {
        for (uint8_t i = 0; i < hal.num_serial; i++) {
            if (hal.serial(i)) {
                hal.serial(i)->_timer_tick();
            }
        }
        if (hal.console) {
            hal.console->_timer_tick();
        }
        sched->delay(1);
    }
}

void Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);
    vTaskDelete(NULL);
}

uint8_t Scheduler::calculate_thread_priority(priority_base base, int8_t priority) const
{
    uint8_t thread_priority = IO_PRIO;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, IO_PRIO},
        { PRIORITY_MAIN, MAIN_PRIO},
        { PRIORITY_SPI, SPI_PRIORITY},
        { PRIORITY_I2C, I2C_PRIORITY},
        { PRIORITY_CAN, IO_PRIO},
        { PRIORITY_TIMER, TIMER_PRIO},
        { PRIORITY_RCOUT, RCOUT_PRIO},
        { PRIORITY_LED, IO_PRIO},
        { PRIORITY_RCIN, RCIN_PRIO},
        { PRIORITY_IO, IO_PRIO},
        { PRIORITY_UART, UART_PRIO},
        { PRIORITY_STORAGE, STORAGE_PRIO},
        { PRIORITY_SCRIPTING, UART_PRIO},
        { PRIORITY_NET, IO_PRIO},
    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
            thread_priority = constrain_int16(priority_map[i].p + priority, MIN_PRIO, MAX_PRIO);
            break;
        }
    }
    return thread_priority;
}

bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name,
    uint32_t requested_stack_size, priority_base base, int8_t priority) {

    // take a copy of the MemberProc, it is freed after thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)calloc(1, sizeof(proc));
    if (!tproc) {
        return false;
    }
    *tproc = proc;

    const uint8_t thread_priority = calculate_thread_priority(base, priority);

    // chibios has a 'thread working area', we just another 1k.
    #define EXTRA_THREAD_SPACE 1024
    uint32_t actual_stack_size = requested_stack_size+EXTRA_THREAD_SPACE;

    TaskHandle_t xhandle;
    BaseType_t xReturned = xTaskCreate(
        thread_create_trampoline, name, actual_stack_size, tproc, thread_priority, &xhandle);
    if (xReturned != pdPASS) {
        free(tproc);
        return false;
    }
    return true;
}

void Scheduler::delay(uint16_t ms) {
    if (_initialized) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        sleep_ms(ms); // SDK function until RTOS started
    }
}

void Scheduler::delay_microseconds(uint16_t us) {
    // For microseconds we use SDK busy-wait,
    // because RTOS context switching takes longer
    sleep_us(us);
}

bool Scheduler::in_main_thread() const {
    return xTaskGetCurrentTaskHandle() == _main_task_handle;
}

void Scheduler::set_system_initialized() {
    _initialized = true;
}

bool Scheduler::is_system_initialized() {
    return _initialized;
}

void Scheduler::reboot(bool hold_in_bootloader) {
    if (hold_in_bootloader) {
        reset_usb_boot(0, 0);
    } else {
        watchdog_reboot(0, 0, 0);
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc) {
    if (_num_timer_procs < RP2350_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs++] = proc;
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc) {
    if (_num_io_procs < RP2350_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs++] = proc;
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc proc, uint32_t period_us) {
    // period_us is usually ignored because failsafe is called together with timers
    _failsafe_proc = proc;
}

void Scheduler::start() {
    vTaskStartScheduler();
    while(1);
}
