#pragma once

#include <stdarg.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Util {
public:
    int snprintf(char* str, size_t size,
                 const char *format, ...);

    int vsnprintf(char* str, size_t size,
                  const char *format, va_list ap);

    void set_soft_armed(const bool b);
    bool get_soft_armed() const { return soft_armed; }

    // return true if the reason for the reboot was a watchdog reset
    virtual bool was_watchdog_reset() const { return false; }

    // return true if safety was off and this was a watchdog reset
    bool was_watchdog_safety_off() const {
        return was_watchdog_reset() && persistent_data.safety_state == SAFETY_ARMED;
    }

    // return true if this is a watchdog reset boot and we were armed
    bool was_watchdog_armed() const { return was_watchdog_reset() && persistent_data.armed; }

    virtual const char* get_custom_log_directory() const { return nullptr; }
    virtual const char* get_custom_terrain_directory() const { return nullptr;  }
    virtual const char *get_custom_storage_directory() const { return nullptr;  }

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
      persistent data structure. This data is restored on boot if
      there has been a watchdog reset.  The data in this structure
      should only be read if was_watchdog_reset() is true
      Note that on STM32 this structure is limited to 76 bytes
     */
    struct PersistentData {
        float roll_rad, pitch_rad, yaw_rad; // attitude
        int32_t home_lat, home_lon, home_alt_cm; // home position
        bool armed; // true if vehicle was armed
        enum safety_state safety_state;
        uint32_t internal_errors;
        uint32_t internal_error_count;
        uint16_t waypoint_num;
        int8_t scheduler_task;
        uint16_t last_mavlink_msgid;
        uint16_t last_mavlink_cmd;
        uint16_t semaphore_line;
        uint32_t spi_count;
        uint32_t i2c_count;
    };
    struct PersistentData persistent_data;

    /*
      return state of safety switch, if applicable
     */
    virtual enum safety_state safety_switch_state(void) { return SAFETY_NONE; }

    /*
      set HW RTC in UTC microseconds
     */
    virtual void set_hw_rtc(uint64_t time_utc_usec);

    /*
      get system clock in UTC microseconds
     */
    virtual uint64_t get_hw_rtc() const;

    // overwrite bootloader (probably with one from ROMFS)
    virtual bool flash_bootloader() { return false; }

    /*
      get system identifier (eg. serial number)
      return false if a system identifier is not available

      Buf should be filled with a printable string and must be null
      terminated
     */
    virtual bool get_system_id(char buf[40]) { return false; }
    virtual bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) { return false; }

    /**
       return commandline arguments, if available
     */
    virtual void commandline_arguments(uint8_t &argc, char * const *&argv) { argc = 0; }

    /*
        ToneAlarm Driver
    */
    virtual bool toneAlarm_init() { return false;}
    virtual void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) {}

    /*
      return a stream for access to a system shell, if available
     */
    virtual AP_HAL::BetterStream *get_shell_stream() { return nullptr; }

    /* Support for an imu heating system */
    virtual void set_imu_temp(float current) {}

    /* Support for an imu heating system */
    virtual void set_imu_target_temp(int8_t *target) {}
    
    /*
      performance counter calls - wrapper around original PX4 interface
     */
    enum perf_counter_type {
        PC_COUNT,        /**< count the number of times an event occurs */
        PC_ELAPSED,      /**< measure the time elapsed performing an event */
        PC_INTERVAL      /**< measure the interval between instances of an event */
    };
    typedef void *perf_counter_t;
    virtual perf_counter_t perf_alloc(perf_counter_type t, const char *name) { return nullptr; }
    virtual void perf_begin(perf_counter_t h) {}
    virtual void perf_end(perf_counter_t h) {}
    virtual void perf_count(perf_counter_t h) {}

    // allocate and free DMA-capable memory if possible. Otherwise return normal memory
    enum Memory_Type {
        MEM_DMA_SAFE,
        MEM_FAST
    };
    virtual void *malloc_type(size_t size, Memory_Type mem_type) { return calloc(1, size); }
    virtual void free_type(void *ptr, size_t size, Memory_Type mem_type) { return free(ptr); }

#ifdef ENABLE_HEAP
    // heap functions, note that a heap once alloc'd cannot be dealloc'd
    virtual void *allocate_heap_memory(size_t size) = 0;
    virtual void *heap_realloc(void *heap, void *ptr, size_t new_size) = 0;
#endif // ENABLE_HEAP

    /**
       how much free memory do we have in bytes. If unknown return 4096
     */
    virtual uint32_t available_memory(void) { return 4096; }

    /*
      initialise (or re-initialise) filesystem storage
     */
    virtual bool fs_init(void) { return false; }

protected:
    // we start soft_armed false, so that actuators don't send any
    // values until the vehicle code has fully started
    bool soft_armed = false;
};
