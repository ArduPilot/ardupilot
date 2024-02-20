#pragma once

#include "AP_BoardConfig_config.h"
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_RTC/AP_RTC.h>
#include <AC_PID/AC_PI.h>

#if HAL_RCINPUT_WITH_AP_RADIO
#include <AP_Radio/AP_Radio.h>
#endif

extern "C" typedef int (*main_fn_t)(int argc, char **);

class AP_BoardConfig {
public:
    AP_BoardConfig();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_BoardConfig);

    // singleton support
    static AP_BoardConfig *get_singleton(void) {
        return _singleton;
    }
    
    void init(void);
    void init_safety(void);

    static const struct AP_Param::GroupInfo var_info[];

    // notify user of a fatal startup error related to available sensors. 
    static void config_error(const char *reason, ...) FMT_PRINTF(1, 2) NORETURN;

    // notify user of a non-fatal startup error related to allocation failures.
    static void allocation_error(const char *reason, ...) FMT_PRINTF(1, 2) NORETURN;

    // permit other libraries (in particular, GCS_MAVLink) to detect
    // that we're never going to boot properly:
    static bool in_config_error(void) { return _in_error_loop; }

    // valid types for BRD_TYPE: these values need to be in sync with the
    // values from the param description
    enum px4_board_type {
        BOARD_TYPE_UNKNOWN = -1,
        PX4_BOARD_AUTO     = 0,
        PX4_BOARD_PX4V1    = 1,
        PX4_BOARD_PIXHAWK  = 2,
        PX4_BOARD_PIXHAWK2 = 3,
        PX4_BOARD_PIXRACER = 4,
        PX4_BOARD_PHMINI   = 5,
        PX4_BOARD_PH2SLIM  = 6,
        PX4_BOARD_AEROFC   = 13,
        PX4_BOARD_PIXHAWK_PRO = 14,
        PX4_BOARD_AUAV21   = 20,
        PX4_BOARD_PCNC1    = 21,
        PX4_BOARD_MINDPXV2 = 22,
        PX4_BOARD_SP01     = 23,
        PX4_BOARD_FMUV5    = 24,
        VRX_BOARD_BRAIN51  = 30,
        VRX_BOARD_BRAIN52  = 32,
        VRX_BOARD_BRAIN52E = 33,
        VRX_BOARD_UBRAIN51 = 34,
        VRX_BOARD_UBRAIN52 = 35,
        VRX_BOARD_CORE10   = 36,
        VRX_BOARD_BRAIN54  = 38,
        PX4_BOARD_FMUV6    = 39,
        FMUV6_BOARD_HOLYBRO_6X = 40,
        FMUV6_BOARD_CUAV_6X = 41,
        FMUV6_BOARD_HOLYBRO_6X_REV6 = 42,
        FMUV6_BOARD_HOLYBRO_6X_45686 = 43,
        PX4_BOARD_OLDDRIVERS = 100,
    };

    // set default value for BRD_SAFETY_MASK
    void set_default_safety_ignore_mask(uint32_t mask);

    static enum px4_board_type get_board_type(void) {
#if AP_FEATURE_BOARD_DETECT
        return px4_configured_board;
#else
        return BOARD_TYPE_UNKNOWN;
#endif
    }

    // ask if IOMCU is enabled. This is a uint8_t to allow
    // developer debugging by setting BRD_IO_ENABLE=100 to avoid the
    // crc check of IO firmware on startup
    static uint8_t io_enabled(void) {
#if HAL_WITH_IO_MCU
        return _singleton?uint8_t(_singleton->state.io_enable.get()):0;
#else
        return 0;
#endif
    }

    static bool io_dshot(void) {
#if HAL_WITH_IO_MCU_DSHOT
        return io_enabled() && _singleton?_singleton->state.io_dshot.get():false;
#else
        return false;
#endif
    }

    // get alternative config selection
    uint8_t get_alt_config(void) {
        return uint8_t(_alt_config.get());
    }

    enum board_safety_button_option {
        BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_OFF= (1 << 0),
        BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON=  (1 << 1),
        BOARD_SAFETY_OPTION_BUTTON_ACTIVE_ARMED=      (1 << 2),
        BOARD_SAFETY_OPTION_SAFETY_ON_DISARM=         (1 << 3),
    };

    // return safety button options. Bits are in enum board_safety_button_option
    uint16_t get_safety_button_options(void) const {
        return uint16_t(state.safety_option.get());
    }

    // return the value of BRD_SAFETY_MASK
    uint16_t get_safety_mask(void) const {
        return uint32_t(state.ignore_safety_channels.get());
    }

    uint32_t get_serial_number() const {
        return (uint32_t)vehicleSerialNumber.get();
    }

#if HAL_HAVE_BOARD_VOLTAGE
    // get minimum board voltage
    static float get_minimum_board_voltage(void) {
        return _singleton?_singleton->_vbus_min.get():0;
    }
#endif

#if HAL_HAVE_SERVO_VOLTAGE
    // get minimum servo voltage
    static float get_minimum_servo_voltage(void) {
        return _singleton?_singleton->_vservo_min.get():0;
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static uint8_t get_sdcard_slowdown(void) {
        return _singleton?_singleton->_sdcard_slowdown.get():0;
    }
#endif

    enum board_options {
        BOARD_OPTION_WATCHDOG = (1 << 0),
        DISABLE_FTP = (1<<1),
        ALLOW_SET_INTERNAL_PARM = (1<<2),
        BOARD_OPTION_DEBUG_ENABLE = (1<<3),
        UNLOCK_FLASH = (1<<4),
        WRITE_PROTECT_FLASH = (1<<5),
        WRITE_PROTECT_BOOTLOADER = (1<<6),
        SKIP_BOARD_VALIDATION = (1<<7),
        DISABLE_ARMING_GPIO = (1<<8)
    };

    //return true if arming gpio output is disabled
    static bool arming_gpio_disabled(void) {
        return _singleton?(_singleton->_options & DISABLE_ARMING_GPIO)!=0:1;
    }
    
#ifndef HAL_ARM_GPIO_POL_INVERT
#define HAL_ARM_GPIO_POL_INVERT 0
#endif

    // return true if ftp is disabled
    static bool ftp_disabled(void) {
        return _singleton?(_singleton->_options & DISABLE_FTP)!=0:1;
    }

    // return true if watchdog enabled
    static bool watchdog_enabled(void) {
        return _singleton?(_singleton->_options & BOARD_OPTION_WATCHDOG)!=0:HAL_WATCHDOG_ENABLED_DEFAULT;
    }

    // return true if flash should be unlocked
    static bool unlock_flash(void) {
        return _singleton && (_singleton->_options & UNLOCK_FLASH) != 0;
    }

    // return true if flash should be write protected
    static bool protect_flash(void) {
        return _singleton && (_singleton->_options & WRITE_PROTECT_FLASH) != 0;
    }

    // return true if bootloader should be write protected
    static bool protect_bootloader(void) {
        return _singleton && (_singleton->_options & WRITE_PROTECT_BOOTLOADER) != 0;
    }

    // return true if we allow setting of internal parameters (for developers)
    static bool allow_set_internal_parameters(void) {
        return _singleton?(_singleton->_options & ALLOW_SET_INTERNAL_PARM)!=0:false;
    }
    
    // handle press of safety button. Return true if safety state
    // should be toggled
    bool safety_button_handle_pressed(uint8_t press_count);

#if HAL_HAVE_IMU_HEATER
    void set_imu_temp(float current_temp_c);

    // heater duty cycle is as a percentage (0 to 100)
    float get_heater_duty_cycle(void) const {
        return heater.output;
    }

    // getters for current temperature and min arming temperature, return false if heater disabled
    bool get_board_heater_temperature(float &temperature) const;
    bool get_board_heater_arming_temperature(int8_t &temperature) const;
#endif

#if AP_SDCARD_STORAGE_ENABLED
    // return number of kb of mission storage to use on microSD
    static uint16_t get_sdcard_mission_kb(void) {
        return _singleton? _singleton->sdcard_storage.mission_kb.get() : 0;
    }

    // return number of kb of fence storage to use on microSD
    static uint16_t get_sdcard_fence_kb(void) {
        return _singleton? _singleton->sdcard_storage.fence_kb.get() : 0;
    }
#endif

private:
    static AP_BoardConfig *_singleton;
    
    AP_Int32 vehicleSerialNumber;

    struct {
        AP_Int8 safety_enable;
        AP_Int16 safety_option;
        AP_Int32 ignore_safety_channels;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        AP_Int8 ser_rtscts[6];
        AP_Int8 sbus_out_rate;
#endif
        AP_Int8 board_type;
        AP_Int8 io_enable;
        AP_Int8 io_dshot;
    } state;

#if AP_SDCARD_STORAGE_ENABLED
    struct {
        AP_Int16 mission_kb;
        AP_Int16 fence_kb;
    } sdcard_storage;
#endif

#if AP_FEATURE_BOARD_DETECT
    static enum px4_board_type px4_configured_board;

    void board_setup_drivers(void);
    bool spi_check_register(const char *devname, uint8_t regnum, uint8_t value, uint8_t read_flag = 0x80);
    bool spi_check_register_inv2(const char *devname, uint8_t regnum, uint8_t value, uint8_t read_flag = 0x80);
    void validate_board_type(void);
    void board_autodetect(void);
    void detect_fmuv6_variant(void);
    bool check_ms5611(const char* devname);

#endif // AP_FEATURE_BOARD_DETECT

    void board_init_safety(void);
    void board_init_debug(void);

    void board_setup_uart(void);
    void board_setup_sbus(void);
    void board_setup(void);

    // common method to throw errors
    static void throw_error(const char *err_str, const char *fmt, va_list arg) NORETURN;

    static bool _in_error_loop;

#if HAL_HAVE_IMU_HEATER
    struct {
        AC_PI pi_controller;
        AP_Int8 imu_target_temperature;
        uint32_t last_update_ms;
        uint16_t count;
        float sum;
        float output;
        uint32_t last_log_ms;
        float temperature;
        AP_Int8 imu_arming_temperature_margin_low;
    } heater;
#endif

#if HAL_RCINPUT_WITH_AP_RADIO
    // direct attached radio
    AP_Radio _radio;
#endif

#if AP_RTC_ENABLED
    // real-time-clock; private because access is via the singleton
    AP_RTC rtc;
#endif

#if HAL_HAVE_BOARD_VOLTAGE
    AP_Float _vbus_min;
#endif

#if HAL_HAVE_SERVO_VOLTAGE
    AP_Float _vservo_min;
#endif

    AP_Int8 _pwm_volt_sel;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    AP_Int8 _sdcard_slowdown;
#endif

    AP_Int16 _boot_delay_ms;

    AP_Int32 _options;

    AP_Int8  _alt_config;
};

namespace AP {
    AP_BoardConfig *boardConfig(void);
};
