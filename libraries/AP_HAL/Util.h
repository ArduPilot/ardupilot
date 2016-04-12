#pragma once

#include <stdarg.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Util {
public:
    int snprintf(char* str, size_t size,
                 const char *format, ...);

    int vsnprintf(char* str, size_t size,
                  const char *format, va_list ap);

    void set_soft_armed(const bool b) { soft_armed = b; }
    bool get_soft_armed() const { return soft_armed; }

    void set_capabilities(uint64_t cap) { capabilities |= cap; }
    void clear_capabilities(uint64_t cap) { capabilities &= ~(cap); }
    uint64_t get_capabilities() const { return capabilities; }

    virtual const char* get_custom_log_directory() { return NULL; } 
    virtual const char* get_custom_terrain_directory() const { return NULL;  }

    // get path to custom defaults file for AP_Param
    virtual const char* get_custom_defaults_file() const {
        return HAL_PARAM_DEFAULTS_PATH;
    }
    
    // run a debug shall on the given stream if possible. This is used
    // to support dropping into a debug shell to run firmware upgrade
    // commands
    virtual bool run_debug_shell(AP_HAL::BetterStream *stream) = 0;

    enum safety_state {
        SAFETY_NONE, SAFETY_DISARMED, SAFETY_ARMED
    };

    /*
      return state of safety switch, if applicable
     */
    virtual enum safety_state safety_switch_state(void) { return SAFETY_NONE; }

    /*
      set system clock in UTC microseconds
     */
    virtual void set_system_clock(uint64_t time_utc_usec) {}

    /*
      get system clock in UTC milliseconds
     */
    uint64_t get_system_clock_ms() const;

    /*
      get system time in UTC hours, minutes, seconds and milliseconds
     */
    void get_system_clock_utc(int32_t &hour, int32_t &min, int32_t &sec, int32_t &ms) const;

    uint32_t get_time_utc(int32_t hour, int32_t min, int32_t sec, int32_t ms) const;

    /*
      get system identifier (eg. serial number)
      return false if a system identifier is not available

      Buf should be filled with a printable string and must be null
      terminated
     */
    virtual bool get_system_id(char buf[40]) { return false; }

    /**
       how much free memory do we have in bytes. If unknown return 4096
     */
    virtual uint32_t available_memory(void) { return 4096; }

    /**
       return commandline arguments, if available
     */
    virtual void commandline_arguments(uint8_t &argc, char * const *&argv) { argc = 0; }
    
    /*
        ToneAlarm Driver
    */
    virtual bool toneAlarm_init() { return false;}
    virtual void toneAlarm_set_tune(uint8_t tune) {}
    virtual void _toneAlarm_timer_tick() {}

    /*
      return a stream for access to a system shell, if available
     */
    virtual AP_HAL::Stream *get_shell_stream() { return NULL; }

    /* Support for an imu heating system */
    virtual void set_imu_temp(float current) {}

    /*
      performance counter calls - wrapper around original PX4 interface
     */
    enum perf_counter_type {
	PC_COUNT,		/**< count the number of times an event occurs */
	PC_ELAPSED,		/**< measure the time elapsed performing an event */
	PC_INTERVAL		/**< measure the interval between instances of an event */
    };
    typedef void *perf_counter_t;
    virtual perf_counter_t perf_alloc(perf_counter_type t, const char *name) { return NULL; }
    virtual void perf_begin(perf_counter_t h) {}
    virtual void perf_end(perf_counter_t h) {}
    virtual void perf_count(perf_counter_t h) {}

    // create a new semaphore
    virtual Semaphore *new_semaphore(void) { return nullptr; }
    
protected:
    // we start soft_armed false, so that actuators don't send any
    // values until the vehicle code has fully started
    bool soft_armed = false;
    uint64_t capabilities = 0;
};
