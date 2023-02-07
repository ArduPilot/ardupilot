#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "Heat.h"
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#include "ToneAlarm_Disco.h"
#endif
#include "ToneAlarm.h"
#include "Semaphores.h"

namespace Linux {

enum hw_type {
    UTIL_HARDWARE_RPI1 = 0,
    UTIL_HARDWARE_RPI2,
    UTIL_HARDWARE_RPI4,
    UTIL_HARDWARE_BEBOP,
    UTIL_HARDWARE_BEBOP2,
    UTIL_HARDWARE_DISCO,
    UTIL_NUM_HARDWARES,
};

class Util : public AP_HAL::Util {
public:
    static Util *from(AP_HAL::Util *util) {
        return static_cast<Util*>(util);
    }

    void init(int argc, char *const *argv);
    bool run_debug_shell(AP_HAL::BetterStream *stream) override { return false; }

    /**
       return commandline arguments, if available
     */
    void commandline_arguments(uint8_t &argc, char * const *&argv) override;

    /*
      get/set system clock in UTC microseconds
     */
    void set_hw_rtc(uint64_t time_utc_usec) override;
    uint64_t get_hw_rtc() const override;

    const char *get_custom_log_directory() const override final { return custom_log_directory; }
    const char *get_custom_terrain_directory() const override final { return custom_terrain_directory; }
    const char *get_custom_storage_directory() const override final { return custom_storage_directory; }

    void set_custom_log_directory(const char *_custom_log_directory) { custom_log_directory = _custom_log_directory; }
    void set_custom_terrain_directory(const char *_custom_terrain_directory) { custom_terrain_directory = _custom_terrain_directory; }
    void set_custom_storage_directory(const char *_custom_storage_directory) {
        custom_storage_directory = _custom_storage_directory;
    }
    void set_custom_defaults_path(const char *_custom_defaults) {
        custom_defaults = _custom_defaults;
    }

    // get path to custom defaults file for AP_Param
    const char* get_custom_defaults_file() const override final {
        return custom_defaults;
    }

    /* Parse cpu set in the form 0; 0,2; or 0-2 */
    bool parse_cpu_set(const char *s, cpu_set_t *cpu_set) const;

    bool is_chardev_node(const char *path);
    void set_imu_temp(float current) override;
    void set_imu_target_temp(int8_t *target) override;

    uint32_t available_memory(void) override;

    bool get_system_id(char buf[50]) override;
    bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) override;

#ifdef ENABLE_HEAP
    // heap functions, note that a heap once alloc'd cannot be dealloc'd
    virtual void *allocate_heap_memory(size_t size) override;
    virtual void *heap_realloc(void *h, void *ptr, size_t old_size, size_t new_size) override;
#endif // ENABLE_HEAP
    
    /*
     * Write a string as specified by @fmt to the file in @path. Note this
     * should not be used on hot path since it will open, write and close the
     * file for each call.
     */
    int write_file(const char *path, const char *fmt, ...) FMT_PRINTF(3, 4);

    /*
     * Read a string as specified by @fmt from the file in @path. Note this
     * should not be used on hot path since it will open, read and close the
     * file for each call.
     */
    int read_file(const char *path, const char *fmt, ...) FMT_SCANF(3, 4);

    int get_hw_arm32();

    bool toneAlarm_init(uint8_t types) override { return _toneAlarm.init(); }
    void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) override {
        _toneAlarm.set_buzzer_tone(frequency, volume, duration_ms);
    }

    // fills data with random values of requested size
    bool get_random_vals(uint8_t* data, size_t size) override;

private:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
    static ToneAlarm_Disco _toneAlarm;
#else
    static ToneAlarm _toneAlarm;
#endif
    int saved_argc;
    Heat *_heat;
    char *const *saved_argv;
    const char *custom_log_directory = nullptr;
    const char *custom_terrain_directory = nullptr;
    const char *custom_storage_directory = nullptr;
    const char *custom_defaults = HAL_PARAM_DEFAULTS_PATH;
    static const char *_hw_names[UTIL_NUM_HARDWARES];

#ifdef ENABLE_HEAP
    struct heap_allocation_header {
        size_t allocation_size; // size of allocated block, not including this header
    };

    struct heap {
      size_t max_heap_size;
      size_t current_heap_usage;
    };
#endif // ENABLE_HEAP

};

}
