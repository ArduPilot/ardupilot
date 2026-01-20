#pragma once

#include "AP_HAL_RP.h"
#include "AP_HAL_RP_Namespace.h"
#include <atomic>

#include "FreeRTOS.h"
#include "task.h"

#define RP2350_SCHEDULER_MAX_TIMER_PROCS 10

class RP::Scheduler : public AP_HAL::Scheduler {
public:
    Scheduler();
    void     init() override;
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;

    void     set_system_initialized() override;
    bool     is_system_initialized() override;

    void     reboot(bool hold_in_bootloader) override;

    bool     in_main_thread() const override;

    // Priorities (the higher the number, the higher the priority)
    static const int MAX_PRIO     = configMAX_PRIORITIES - 1;
    static const int MAIN_PRIO    = MAX_PRIO - 1;   // Main loop (Core 0)
    static const int TIMER_PRIO   = MAX_PRIO;       // Timers (must be higher than MAIN for accuracy)
    static const int UART_PRIO    = MAX_PRIO - 2;   // UART (fast output, but below the timer)
    static const int SPI_PRIORITY = MAX_PRIO - 3;   // Fast sensors (IMU)
    static const int RCOUT_PRIO   = MAX_PRIO - 15;  // PWM output
    static const int RCIN_PRIO    = MAX_PRIO - 15;  // Signal input
    static const int I2C_PRIORITY = MAX_PRIO - 17;  // I2C is usually slower than SPI
    static const int IO_PRIO      = MAX_PRIO - 20;  // Background operations
    static const int STORAGE_PRIO = MAX_PRIO - 21;  // Flash recording (lowest, may be long)

    // Stack sizes (in words, 1 word = 4 bytes on RP2350)
    // Values increased by 20% because Cortex-M33 with FPU needs more space when switching context
    static const int MAIN_SS      = 2048; // 8 KB
    static const int TIMER_SS     = 1024; // 4 KB
    static const int UART_SS      = 1024; // 4 KB
    static const int DEVICE_SS    = 1536; // 6 KB (for SPI/I2C buses)
    static const int RCIN_SS      = 768;  // 3 KB
    static const int RCOUT_SS     = 512;  // 2 KB
    static const int IO_SS        = 1024; // 4 KB
    static const int STORAGE_SS   = 1024; // 4 KB

private:
    static void _main_task(void *pvParameters);
    static void _timer_task(void *pvParameters);
    static void _io_task(void *pvParameters);

    AP_HAL::MemberProc _timer_proc[RP2350_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;

    AP_HAL::MemberProc _io_proc[RP2350_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;

    AP_HAL::Proc _failsafe_proc;

    TaskHandle_t _main_task_handle;
    TaskHandle_t _timer_task_handle;
    TaskHandle_t _io_task_handle;

    std::atomic<bool> _initialized;
};
