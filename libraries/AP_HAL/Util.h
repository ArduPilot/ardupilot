#pragma once

#include <stdarg.h>
#include <AP_Common/AP_Common.h> // for FMT_PRINTF
#include "AP_HAL_Namespace.h"

class ExpandingString;

class AP_HAL::Util {
public:
    int snprintf(char* str, size_t size,
                 const char *format, ...) FMT_PRINTF(4, 5);

    int vsnprintf(char* str, size_t size,
                  const char *format, va_list ap);

    virtual void set_soft_armed(const bool b);
    bool get_soft_armed() const { return soft_armed; }

    // return the time that the armed state last changed
    uint32_t get_last_armed_change() const { return last_armed_change_ms; };

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

    // set command line parameters to the eeprom on start
    virtual void set_cmdline_parameters() {};

    // run a debug shall on the given stream if possible. This is used
    // to support dropping into a debug shell to run firmware upgrade
    // commands
    virtual bool run_debug_shell(AP_HAL::BetterStream *stream) = 0;

    enum safety_state : uint8_t {
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
        uint32_t fault_addr;
        uint32_t fault_icsr;
        uint32_t fault_lr;
        uint32_t internal_errors;
        uint16_t internal_error_count;
        uint16_t internal_error_last_line;
        uint32_t spi_count;
        uint32_t i2c_count;
        uint32_t i2c_isr_count;
        uint16_t waypoint_num;
        uint16_t last_mavlink_msgid;
        uint16_t last_mavlink_cmd;
        uint16_t semaphore_line;
        uint16_t fault_line;
        uint8_t fault_type;
        uint8_t fault_thd_prio;
        char thread_name4[4];
        int8_t scheduler_task;
        bool armed; // true if vehicle was armed
        enum safety_state safety_state;
    };
    struct PersistentData persistent_data;
    // last_persistent_data is only filled in if we've suffered a watchdog reset
    struct PersistentData last_persistent_data;

    /*
      return state of safety switch, if applicable
     */
    virtual enum safety_state safety_switch_state(void) { return SAFETY_NONE; }

    /*
      set HW RTC in UTC microseconds
     */
    virtual void set_hw_rtc(uint64_t time_utc_usec) = 0;

    /*
      get system clock in UTC microseconds
     */
    virtual uint64_t get_hw_rtc() const = 0;

    enum class FlashBootloader {
        OK=0,
        NO_CHANGE=1,
        FAIL=2,
        NOT_AVAILABLE=3,
    };

    // overwrite bootloader (probably with one from ROMFS)
    virtual FlashBootloader flash_bootloader() { return FlashBootloader::NOT_AVAILABLE; }

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

    virtual bool toneAlarm_init(uint8_t types) { return false;}
    virtual void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) {}

    /*
      return a stream for access to a system shell, if available
     */
    virtual AP_HAL::BetterStream *get_shell_stream() { return nullptr; }

    /* Support for an imu heating system */
    virtual void set_imu_temp(float current) {}

    /* Support for an imu heating system */
    virtual void set_imu_target_temp(int8_t *target) {}
    
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
#if USE_LIBC_REALLOC
    virtual void *std_realloc(void *ptr, size_t new_size) { return realloc(ptr, new_size); }
#else
    virtual void *std_realloc(void *ptr, size_t new_size) = 0;
#endif // USE_LIBC_REALLOC
#endif // ENABLE_HEAP


    /**
       how much free memory do we have in bytes. If unknown return 4096
     */
    virtual uint32_t available_memory(void) { return 4096; }

    // attempt to trap the processor, presumably to enter an attached debugger
    virtual bool trap() const { return false; }

    // request information on running threads
    virtual void thread_info(ExpandingString &str) {}

    // request information on dma contention
    virtual void dma_info(ExpandingString &str) {}

    // request information on memory allocation
    virtual void mem_info(ExpandingString &str) {}

    // load persistent parameters from bootloader sector
    virtual bool load_persistent_params(ExpandingString &str) const { return false; }

#if HAL_UART_STATS_ENABLED
    // request information on uart I/O
    virtual void uart_info(ExpandingString &str) {}
#endif
    // request information on timer frequencies
    virtual void timer_info(ExpandingString &str) {}

    // generate Random values
    virtual bool get_random_vals(uint8_t* data, size_t size) { return false; }

    // generate Random values, will block until enough entropy is available
    virtual bool get_true_random_vals(uint8_t* data, size_t size, uint32_t timeout_us) { return false; }

    // log info on stack usage
    virtual void log_stack_info(void) {}

#if AP_CRASHDUMP_ENABLED
    virtual size_t last_crash_dump_size() const { return 0; }
    virtual void* last_crash_dump_ptr() const { return nullptr; }
#endif

protected:
    // we start soft_armed false, so that actuators don't send any
    // values until the vehicle code has fully started
    bool soft_armed = false;
    uint32_t last_armed_change_ms;
};
