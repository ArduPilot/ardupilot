#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_RTC/AP_RTC.h>
#include <AC_PID/AC_PI.h>

#ifndef AP_FEATURE_BOARD_DETECT
#if defined(HAL_CHIBIOS_ARCH_FMUV3) || defined(HAL_CHIBIOS_ARCH_FMUV4) || defined(HAL_CHIBIOS_ARCH_FMUV5) || defined(HAL_CHIBIOS_ARCH_MINDPXV2) || defined(HAL_CHIBIOS_ARCH_FMUV4PRO) || defined(HAL_CHIBIOS_ARCH_BRAINV51) || defined(HAL_CHIBIOS_ARCH_BRAINV52) || defined(HAL_CHIBIOS_ARCH_UBRAINV51) || defined(HAL_CHIBIOS_ARCH_COREV10) || defined(HAL_CHIBIOS_ARCH_BRAINV54)
#define AP_FEATURE_BOARD_DETECT 1
#else
#define AP_FEATURE_BOARD_DETECT 0
#endif
#endif

#ifndef AP_FEATURE_RTSCTS
#define AP_FEATURE_RTSCTS 0
#endif

#ifndef AP_FEATURE_SBUS_OUT
#define AP_FEATURE_SBUS_OUT 0
#endif

#if HAL_RCINPUT_WITH_AP_RADIO
#include <AP_Radio/AP_Radio.h>
#endif

#ifndef HAL_WATCHDOG_ENABLED_DEFAULT
#define HAL_WATCHDOG_ENABLED_DEFAULT false
#endif

extern "C" typedef int (*main_fn_t)(int argc, char **);

class AP_BoardConfig {
public:
    AP_BoardConfig() {
        _singleton = this;
        AP_Param::setup_object_defaults(this, var_info);
    };

    /* Do not allow copies */
    AP_BoardConfig(const AP_BoardConfig &other) = delete;
    AP_BoardConfig &operator=(const AP_BoardConfig&) = delete;

    // singleton support
    static AP_BoardConfig *get_singleton(void) {
        return _singleton;
    }
    
    void init(void);
    void init_safety(void);

    static const struct AP_Param::GroupInfo var_info[];

    // notify user of a fatal startup error related to available sensors. 
    static void config_error(const char *reason, ...);

    // permit other libraries (in particular, GCS_MAVLink) to detect
    // that we're never going to boot properly:
    static bool in_config_error(void) { return _in_sensor_config_error; }

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
        PX4_BOARD_OLDDRIVERS = 100,
        PX4_BOARD_FMUV6    = 39,
    };

    // set default value for BRD_SAFETY_MASK
    void set_default_safety_ignore_mask(uint16_t mask);

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

    // get number of PWM outputs enabled on FMU
    static uint8_t get_pwm_count(void) {
        return _singleton?_singleton->pwm_count.get():8;
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
    uint16_t get_safety_button_options(void) {
        return uint16_t(state.safety_option.get());
    }

    // return the value of BRD_SAFETY_MASK
    uint16_t get_safety_mask(void) const {
#if AP_FEATURE_BOARD_DETECT || defined(AP_FEATURE_BRD_PWM_COUNT_PARAM)
        return uint16_t(state.ignore_safety_channels.get());
#else
        return 0;
#endif
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
    };

    // return true if watchdog enabled
    static bool watchdog_enabled(void) {
        return _singleton?(_singleton->_options & BOARD_OPTION_WATCHDOG)!=0:HAL_WATCHDOG_ENABLED_DEFAULT;
    }

    // handle press of safety button. Return true if safety state
    // should be toggled
    bool safety_button_handle_pressed(uint8_t press_count);

#if HAL_HAVE_IMU_HEATER
    void set_imu_temp(float current_temp_c);
#endif

private:
    static AP_BoardConfig *_singleton;
    
    AP_Int16 vehicleSerialNumber;
    AP_Int8 pwm_count;

    struct {
        AP_Int8 safety_enable;
        AP_Int16 safety_option;
        AP_Int32 ignore_safety_channels;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        AP_Int8 ser1_rtscts;
        AP_Int8 ser2_rtscts;
        AP_Int8 sbus_out_rate;
#endif
        AP_Int8 board_type;
        AP_Int8 io_enable;
    } state;

#if AP_FEATURE_BOARD_DETECT
    static enum px4_board_type px4_configured_board;

    void board_setup_drivers(void);
    bool spi_check_register(const char *devname, uint8_t regnum, uint8_t value, uint8_t read_flag = 0x80);
    void validate_board_type(void);
    void board_autodetect(void);

#endif // AP_FEATURE_BOARD_DETECT

    void board_init_safety(void);

    void board_setup_uart(void);
    void board_setup_sbus(void);
    void board_setup(void);

    static bool _in_sensor_config_error;

#if HAL_HAVE_IMU_HEATER
    struct {
        AP_Int8 imu_target_temperature;
        uint32_t last_update_ms;
        AC_PI pi_controller{200, 0.3, 70};
        uint16_t count;
        float sum;
        float output;
        uint32_t last_log_ms;
    } heater;
#endif

#if HAL_RCINPUT_WITH_AP_RADIO
    // direct attached radio
    AP_Radio _radio;
#endif
    
    // real-time-clock; private because access is via the singleton
    AP_RTC rtc;

#if HAL_HAVE_BOARD_VOLTAGE
    AP_Float _vbus_min;
#endif

#if HAL_HAVE_SERVO_VOLTAGE
    AP_Float _vservo_min;
#endif

#ifdef HAL_GPIO_PWM_VOLT_PIN
    AP_Int8 _pwm_volt_sel;
#endif

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
