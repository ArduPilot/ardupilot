
#ifndef __AP_HAL_PX4_UTIL_H__
#define __AP_HAL_PX4_UTIL_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_PX4_Namespace.h"

class PX4::NSHShellStream : public AP_HAL::Stream {
public:
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);
    int16_t read();
    int16_t available();
    int16_t txspace();
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
    void shell_thread(void);
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

    uint16_t available_memory(void);

    /*
      return a stream for access to nsh shell
     */
    AP_HAL::Stream *get_shell_stream() { return &_shell_stream; }

private:
    int _safety_handle;
    PX4::NSHShellStream _shell_stream;
};

#endif // __AP_HAL_PX4_UTIL_H__
