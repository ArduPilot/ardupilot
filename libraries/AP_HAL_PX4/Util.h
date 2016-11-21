#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_PX4_Namespace.h"
#include "Semaphores.h"

class PX4::NSHShellStream : public AP_HAL::Stream {
public:
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);
    int16_t read() override;
    uint32_t available() override;
    uint32_t txspace() override;
private:
    int shell_stdin = -1;
    int shell_stdout = -1;
    pthread_t shell_thread_ctx;

    struct {
        int in = -1;
        int out = -1;
    } child;
    bool showed_memory_warning = false;
    bool showed_armed_warning = false;

    void start_shell(void);
    static void shell_thread(void *arg);
};

class PX4::PX4Util : public AP_HAL::Util {
public:
    PX4Util(void);
    bool run_debug_shell(AP_HAL::BetterStream *stream);

    enum safety_state safety_switch_state(void);

    /*
      set system clock in UTC microseconds
     */
    void set_system_clock(uint64_t time_utc_usec);

    /*
      get system identifier (STM32 serial number)
     */
    bool get_system_id(char buf[40]);

    uint32_t available_memory(void) override;

    /*
      return a stream for access to nsh shell
     */
    AP_HAL::Stream *get_shell_stream() { return &_shell_stream; }
    perf_counter_t perf_alloc(perf_counter_type t, const char *name) override;
    void perf_begin(perf_counter_t ) override;
    void perf_end(perf_counter_t) override;
    void perf_count(perf_counter_t) override;
    
    // create a new semaphore
    AP_HAL::Semaphore *new_semaphore(void) override { return new PX4::Semaphore; }

    void set_imu_temp(float current) override;
    void set_imu_target_temp(int8_t *target) override;
    
private:
    int _safety_handle;
    PX4::NSHShellStream _shell_stream;

    struct {
        int8_t *target;
        float integrator;
        uint16_t count;
        float sum;
        uint32_t last_update_ms;
        int fd = -1;
    } _heater;
};
