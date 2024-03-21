/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_HAL_ESP32/Scheduler.h"
#include "AP_HAL_ESP32/RCInput.h"
#include "AP_HAL_ESP32/AnalogIn.h"
#include "AP_Math/AP_Math.h"
#include "SdCard.h"
#include "Profile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/rtc_wdt.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <stdio.h>
#include <AP_Common/ExpandingString.h>


//#define SCHEDULERDEBUG 1

using namespace ESP32;

extern const AP_HAL::HAL& hal;

bool Scheduler::_initialized = true;
TaskHandle_t idle_0 = NULL;
TaskHandle_t idle_1 = NULL;

Scheduler::Scheduler()
{
    _initialized = false;
}

void disableCore0WDT()
{
     idle_0 = xTaskGetIdleTaskHandleForCPU(0);
    if (idle_0 == NULL || esp_task_wdt_delete(idle_0) != ESP_OK) {
        //print("Failed to remove Core 0 IDLE task from WDT");
    }
}

void disableCore1WDT()
{
     idle_1 = xTaskGetIdleTaskHandleForCPU(1);
    if (idle_1 == NULL || esp_task_wdt_delete(idle_1) != ESP_OK) {
        //print("Failed to remove Core 1 IDLE task from WDT");
    }
}
void enableCore0WDT()
{
    if (idle_0 != NULL && esp_task_wdt_add(idle_0) != ESP_OK) {
        //print("Failed to add Core 0 IDLE task to WDT");
    }
}
void enableCore1WDT()
{
    if (idle_1 != NULL && esp_task_wdt_add(idle_1) != ESP_OK) {
        //print("Failed to add Core 1 IDLE task to WDT");
    }
}

void Scheduler::init()
{

#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif

    // disable wd while booting, as things like mounting the sd-card in the io thread can take a while, especially if there isn't hardware attached.
    disableCore0WDT(); //FASTCPU
    disableCore1WDT(); //SLOWCPU


    hal.console->printf("%s:%d running with CONFIG_FREERTOS_HZ=%d\n", __PRETTY_FUNCTION__, __LINE__,CONFIG_FREERTOS_HZ);

    // keep main tasks that need speed on CPU 0
    // pin potentially slow stuff to CPU 1, as we have disabled the WDT on that core.
    #define FASTCPU 0
    #define SLOWCPU 1

    // pin main thread to Core 0, and we'll also pin other heavy-tasks to core 1, like wifi-related.
    if (xTaskCreatePinnedToCore(_main_thread, "APM_MAIN", Scheduler::MAIN_SS, this, Scheduler::MAIN_PRIO, &_main_task_handle,FASTCPU) != pdPASS) {
    //if (xTaskCreate(_main_thread, "APM_MAIN", Scheduler::MAIN_SS, this, Scheduler::MAIN_PRIO, &_main_task_handle) != pdPASS) {
        hal.console->printf("FAILED to create task _main_thread on FASTCPU\n");
    } else {
    	hal.console->printf("OK created task _main_thread on FASTCPU\n");
    }

    if (xTaskCreatePinnedToCore(_timer_thread, "APM_TIMER", TIMER_SS, this, TIMER_PRIO, &_timer_task_handle,FASTCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _timer_thread on FASTCPU\n");
    } else {
    	hal.console->printf("OK created task _timer_thread on FASTCPU\n");
    }	

    if (xTaskCreatePinnedToCore(_rcout_thread, "APM_RCOUT", RCOUT_SS, this, RCOUT_PRIO, &_rcout_task_handle,SLOWCPU) != pdPASS) {
       hal.console->printf("FAILED to create task _rcout_thread on SLOWCPU\n");
    } else {
       hal.console->printf("OK created task _rcout_thread on SLOWCPU\n");
    }

    if (xTaskCreatePinnedToCore(_rcin_thread, "APM_RCIN", RCIN_SS, this, RCIN_PRIO, &_rcin_task_handle,SLOWCPU) != pdPASS) {
       hal.console->printf("FAILED to create task _rcin_thread on SLOWCPU\n");
    } else {
       hal.console->printf("OK created task _rcin_thread on SLOWCPU\n");
    }

    // pin this thread to Core 1 as it keeps all teh uart/s feed data, and we need that quick.
    if (xTaskCreatePinnedToCore(_uart_thread, "APM_UART", UART_SS, this, UART_PRIO, &_uart_task_handle,FASTCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _uart_thread on FASTCPU\n");
    } else {
    	hal.console->printf("OK created task _uart_thread on FASTCPU\n");
    }	  

    // we put thos on the SLOW core as it mounts the sd card, and that often isn't conencted.
    if (xTaskCreatePinnedToCore(_io_thread, "SchedulerIO:APM_IO", IO_SS, this, IO_PRIO, &_io_task_handle,SLOWCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _io_thread on SLOWCPU\n");
    } else {
        hal.console->printf("OK created task _io_thread on SLOWCPU\n");
    }	 

    if (xTaskCreatePinnedToCore(_storage_thread, "APM_STORAGE", STORAGE_SS, this, STORAGE_PRIO, &_storage_task_handle,SLOWCPU) != pdPASS) { //no actual flash writes without this, storage kinda appears to work, but does an erase on every boot and params don't persist over reset etc.
        hal.console->printf("FAILED to create task _storage_thread\n");
    } else {
    	hal.console->printf("OK created task _storage_thread\n");
    }

    //   xTaskCreatePinnedToCore(_print_profile, "APM_PROFILE", IO_SS, this, IO_PRIO, nullptr,SLOWCPU);

    hal.console->printf("OK Sched Init, enabling WD\n");
    enableCore0WDT();   //FASTCPU
    //enableCore1WDT();   //we don't enable WD on SLOWCPU right now.

}

template <typename T>
void executor(T oui)
{
    oui();
}

void Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t requested_stack_size, priority_base base, int8_t priority)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif

    // take a copy of the MemberProc, it is freed after thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)calloc(1, sizeof(proc));
    if (!tproc) {
        return false;
    }
    *tproc = proc;

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
        { PRIORITY_RCIN, RCIN_PRIO},
        { PRIORITY_IO, IO_PRIO},
        { PRIORITY_UART, UART_PRIO},
        { PRIORITY_NET, WIFI_PRIO1},
        { PRIORITY_STORAGE, STORAGE_PRIO},
        { PRIORITY_SCRIPTING, UART_PRIO},
    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
#ifdef SCHEDDEBUG
            printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
            thread_priority = constrain_int16(priority_map[i].p + priority, 1, 25);
            break;
        }
    }
    // chibios has a 'thread working area', we just another 1k.
    #define EXTRA_THREAD_SPACE 1024
    uint32_t actual_stack_size = requested_stack_size+EXTRA_THREAD_SPACE;

    tskTaskControlBlock* xhandle;
    BaseType_t xReturned = xTaskCreate(thread_create_trampoline, name, actual_stack_size, tproc, thread_priority, &xhandle);
    if (xReturned != pdPASS) {
        free(tproc);
        return false;
    }
    return true;
}

void Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

void Scheduler::delay_microseconds(uint16_t us)
{
    if (in_main_thread() && us < 100) {
        ets_delay_us(us);
    } else { // Minimum delay for FreeRTOS is 1ms
        uint32_t tick = portTICK_PERIOD_MS * 1000;

        vTaskDelay((us+tick-1)/tick);
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }
    if (_num_timer_procs >= ESP32_SCHEDULER_MAX_TIMER_PROCS) {
        printf("Out of timer processes\n");
        return;
    }
    _timer_sem.take_blocking();
    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
    _timer_sem.give();
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _io_sem.take_blocking();
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            _io_sem.give();
            return;
        }
    }
    if (_num_io_procs < ESP32_SCHEDULER_MAX_IO_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        printf("Out of IO processes\n");
    }
    _io_sem.give();
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    printf("Restarting now...\n");
    hal.rcout->force_safety_on();
    unmount_sdcard();
    esp_restart();
}

bool Scheduler::in_main_thread() const
{
    return _main_task_handle == xTaskGetCurrentTaskHandle();
}

void Scheduler::set_system_initialized()
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }

    _initialized = true;
}

bool Scheduler::is_system_initialized()
{
    return _initialized;
}

void Scheduler::_timer_thread(void *arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;

#if HAL_INS_DEFAULT != HAL_INS_NONE
    // wait to ensure INS system inits unless using HAL_INS_NONE
    while (!_initialized) {
        sched->delay_microseconds(1000);
    }
#endif

#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        sched->_run_timers();
        //analog in
#ifndef HAL_DISABLE_ADC_DRIVER
        ((AnalogIn*)hal.analogin)->_timer_tick();
#endif
    }
}

void Scheduler::_rcout_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(1000);
    }

    while (true) {
        sched->delay_microseconds(4000);
        // process any pending RC output requests
        hal.rcout->timer_tick();
    }
}

void Scheduler::_run_timers()
{
#ifdef SCHEDULERDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_in_timer_proc) {
        return;
    }
#ifdef SCHEDULERDEBUG
    printf("%s:%d _in_timer_proc \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _in_timer_proc = true;

    int num_procs = 0;

    _timer_sem.take_blocking();
    num_procs = _num_timer_procs;
    _timer_sem.give();

    // now call the timer based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(20000);
    }
    hal.rcin->init();
    while (true) {
        sched->delay_microseconds(1000);
        ((RCInput *)hal.rcin)->_timer_tick();
    }
}

void Scheduler::_run_io(void)
{
#ifdef SCHEDULERDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_in_io_proc) {
        return;
    }
#ifdef SCHEDULERDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _in_io_proc = true;

    int num_procs = 0;
    _io_sem.take_blocking();
    num_procs = _num_io_procs;
    _io_sem.give();
    // now call the IO based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }
    _in_io_proc = false;
}

void Scheduler::_io_thread(void* arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    mount_sdcard();
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(1000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint32_t last_sd_start_ms = AP_HAL::millis();
    while (true) {
        sched->delay_microseconds(1000);
        // run registered IO processes
        sched->_run_io();

        if (!hal.util->get_soft_armed()) {
            // if sdcard hasn't mounted then retry it every 3s in the IO
            // thread when disarmed
            uint32_t now = AP_HAL::millis();
            if (now - last_sd_start_ms > 3000) {
                last_sd_start_ms = now;
                sdcard_retry();
            }
        }
    }
}


void Scheduler::_storage_thread(void* arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(10000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        // process any pending storage writes
        hal.storage->_timer_tick();
        //print_profile();
    }
}

void Scheduler::_print_profile(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(10000);
    }

    while (true) {
        sched->delay(10000);
        print_profile();
    }

}

void Scheduler::_uart_thread(void *arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(2000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        for (uint8_t i=0; i<hal.num_serial; i++) {
            hal.serial(i)->_timer_tick();
        }
        hal.console->_timer_tick();
    }
}


// get the active main loop rate
uint16_t Scheduler::get_loop_rate_hz(void)
{
    if (_active_loop_rate_hz == 0) {
        _active_loop_rate_hz = _loop_rate_hz;
    }
    return _active_loop_rate_hz;
}


#define STATS_TICKS         pdMS_TO_TICKS(1000)

// once every 60 seconds, print some stats...
void Scheduler::print_stats(void)
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 60000) {
        char buffer[1024];
        vTaskGetRunTimeStats(buffer);
        printf("\n\n%s\n", buffer);
        heap_caps_print_heap_info(0);
        last_run = AP_HAL::millis64();

        print_real_time_stats(STATS_TICKS);
        printf("Real time stats obtained\n");

        ExpandingString *sstr = new ExpandingString;
        AP::scheduler().task_info(*sstr);
        printf("\n\n%s\n", sstr->get_string());
    }

    // printf("loop_rate_hz: %d",get_loop_rate_hz());
}

#define ARRAY_SIZE_OFFSET   5   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

/**
 * @brief   Function to print the CPU usage of tasks over a given duration.
 *
 * This function will measure and print the CPU usage of tasks over a specified
 * number of ticks (i.e. real time stats). This is implemented by simply calling
 * uxTaskGetSystemState() twice separated by a delay, then calculating the
 * differences of task run times before and after the delay.
 *
 * @note    If any tasks are added or removed during the delay, the stats of
 *          those tasks will not be printed.
 * @note    This function should be called from a high priority task to minimize
 *          inaccuracies with delays.
 * @note    When running in dual core mode, each core will correspond to 50% of
 *          the run time.
 *
 * @param   xTicksToWait    Period of stats measurement
 *
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_NO_MEM        Insufficient memory to allocated internal arrays
 *  - ESP_ERR_INVALID_SIZE  Insufficient array size for uxTaskGetSystemState. Trying increasing ARRAY_SIZE_OFFSET
 *  - ESP_ERR_INVALID_STATE Delay duration too short
 */
 // from :
 // https://github.com/espressif/esp-idf/blob/56123c52aaa08f1b53350c7af30c91320b352ef4/examples/system/freertos/real_time_stats/main/real_time_stats_example_main.c#L160
void Scheduler::print_real_time_stats(TickType_t xTicksToWait)
{
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    uint32_t start_run_time, end_run_time;
    //esp_err_t ret;

    //Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = (TaskStatus_t*)malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
        printf("ESP_ERR_NO_MEM\n");
        free(start_array);
        free(end_array);
        return;
    }
    //Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        printf("ESP_ERR_INVALID_SIZE\n");
        free(start_array);
        free(end_array);
        return;
    }

    vTaskDelay(xTicksToWait);

    //Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    end_array = (TaskStatus_t*)malloc(sizeof(TaskStatus_t) * end_array_size);
    if (end_array == NULL) {
        printf("ESP_ERR_NO_MEM\n");
        free(start_array);
        free(end_array);
        return;
    }
    //Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
        printf("ESP_ERR_INVALID_SIZE\n");
        free(start_array);
        free(end_array);
        return;
    }

    //Calculate total_elapsed_time in units of run time stats clock period.
    uint32_t total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        printf("ESP_ERR_INVALID_STATE\n");
        free(start_array);
        free(end_array);
        return;
    }

    printf("| Task | Run Time | Percentage\n");
    //Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {
        int k = -1;
        for (int j = 0; j < end_array_size; j++) {
            if (start_array[i].xHandle == end_array[j].xHandle) {
                k = j;
                //Mark that task have been matched by overwriting their handles
                start_array[i].xHandle = NULL;
                end_array[j].xHandle = NULL;
                break;
            }
        }
        //Check if matching task found
        if (k >= 0) {
            uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
            uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);
            printf("| %s | %" PRIu32 " | %" PRIu32 "%%\n", start_array[i].pcTaskName, task_elapsed_time, percentage_time);
        }
    }

    //Print unmatched tasks
    for (int i = 0; i < start_array_size; i++) {
        if (start_array[i].xHandle != NULL) {
            printf("| %s | Deleted\n", start_array[i].pcTaskName);
        }
    }
    for (int i = 0; i < end_array_size; i++) {
        if (end_array[i].xHandle != NULL) {
            printf("| %s | Created\n", end_array[i].pcTaskName);
        }
    }
    //ret = ESP_OK;

//exit:    //Common return path
    free(start_array);
    free(end_array);
    //return ret;
}

// Run every 10s
void Scheduler::print_main_loop_rate(void)
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 10000) {
        last_run = AP_HAL::millis64();
        // null pointer in here...
        //const float actual_loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
        //const uint16_t expected_loop_rate = AP::scheduler().get_loop_rate_hz();
        //hal.console->printf("loop_rate: actual: %uHz, expected: %uHz\n",
    }
}

void IRAM_ATTR Scheduler::_main_thread(void *arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;

#ifndef HAL_DISABLE_ADC_DRIVER
    hal.analogin->init();
#endif
    hal.rcout->init();

    sched->callbacks->setup();

    sched->set_system_initialized();

#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->callbacks->loop();
        sched->delay_microseconds(250);

        // run stats periodically
        sched->print_stats();
        sched->print_main_loop_rate();
    }
}

