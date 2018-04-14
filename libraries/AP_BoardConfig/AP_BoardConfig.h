#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#if defined(HAL_NEEDS_PARAM_HELPER)
#include <AP_Param_Helper/AP_Param_Helper.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN || defined(HAL_CHIBIOS_ARCH_FMUV3) || defined(HAL_CHIBIOS_ARCH_FMUV4) || defined(HAL_CHIBIOS_ARCH_MINDPXV2)
#define AP_FEATURE_BOARD_DETECT 1
#define AP_FEATURE_SAFETY_BUTTON 1
#else
#define AP_FEATURE_BOARD_DETECT 0
#define AP_FEATURE_SAFETY_BUTTON 0
#endif

#ifndef AP_FEATURE_RTSCTS
#define AP_FEATURE_RTSCTS 0
#endif

#ifndef AP_FEATURE_RTSCTS
#define AP_FEATURE_RTSCTS 0
#endif

#ifndef AP_FEATURE_SBUS_OUT
#define AP_FEATURE_SBUS_OUT 0
#endif

#ifdef HAL_RCINPUT_WITH_AP_RADIO
#include <AP_Radio/AP_Radio.h>
#endif

extern "C" typedef int (*main_fn_t)(int argc, char **);

class AP_BoardConfig {
public:
    AP_BoardConfig() {
        instance = this;
        AP_Param::setup_object_defaults(this, var_info);
    };

    /* Do not allow copies */
    AP_BoardConfig(const AP_BoardConfig &other) = delete;
    AP_BoardConfig &operator=(const AP_BoardConfig&) = delete;

    // singleton support
    AP_BoardConfig *get_instance(void) {
        return instance;
    }
    
    void init(void);
    void init_safety(void);

    static const struct AP_Param::GroupInfo var_info[];

    // notify user of a fatal startup error related to available sensors. 
    static void sensor_config_error(const char *reason);

    // permit other libraries (in particular, GCS_MAVLink) to detect
    // that we're never going to boot properly:
    static bool in_sensor_config_error(void) { return _in_sensor_config_error; }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // public method to start a driver
    static bool px4_start_driver(main_fn_t main_function, const char *name, const char *arguments);
#endif

#if AP_FEATURE_BOARD_DETECT

    // valid types for BRD_TYPE: these values need to be in sync with the
    // values from the param description
    enum px4_board_type {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN || CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
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
        VRX_BOARD_BRAIN51  = 30,
        VRX_BOARD_BRAIN52  = 32,
        VRX_BOARD_BRAIN52E = 33,
        VRX_BOARD_UBRAIN51 = 34,
        VRX_BOARD_UBRAIN52 = 35,
        VRX_BOARD_CORE10   = 36,
        VRX_BOARD_BRAIN54  = 38,
        PX4_BOARD_OLDDRIVERS = 100,
#endif
    };
#endif // AP_FEATURE_BOARD_DETECT

    // set default value for BRD_SAFETY_MASK
    void set_default_safety_ignore_mask(uint16_t mask);

#if AP_FEATURE_BOARD_DETECT
    static enum px4_board_type get_board_type(void) {
        return px4_configured_board;
    }
#endif

    // ask if IOMCU is enabled
    static bool io_enabled(void) {
#if AP_FEATURE_BOARD_DETECT
        return instance?instance->state.io_enable.get():false;
#else
        return false;
#endif
    }

    // get number of PWM outputs enabled on FMU
    static uint8_t get_pwm_count(void) {
#if AP_FEATURE_BOARD_DETECT
        return instance?instance->state.pwm_count.get():4;
#else
        return 0;
#endif
    }
    
private:
    static AP_BoardConfig *instance;
    
    AP_Int16 vehicleSerialNumber;

#if AP_FEATURE_BOARD_DETECT
    struct {
        AP_Int8 pwm_count;
        AP_Int8 safety_enable;
        AP_Int32 ignore_safety_channels;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        AP_Int8 ser1_rtscts;
        AP_Int8 ser2_rtscts;
        AP_Int8 sbus_out_rate;
#endif
        AP_Int8 board_type;
        AP_Int8 io_enable;
    } state;

    static enum px4_board_type px4_configured_board;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    void px4_setup_pwm(void);
    void px4_setup_safety_mask(void);
    void px4_tone_alarm(const char *tone_string);
    void px4_setup_px4io(void);
    void px4_setup_peripherals(void);
#endif
    

    void board_init_safety(void);
    void board_setup_safety_mask(void);
    void board_setup_drivers(void);
    bool spi_check_register(const char *devname, uint8_t regnum, uint8_t value, uint8_t read_flag = 0x80);
    void validate_board_type(void);
    void board_autodetect(void);

#endif // AP_FEATURE_BOARD_DETECT

    void board_setup_uart(void);
    void board_setup_sbus(void);
    void board_setup(void);

    static bool _in_sensor_config_error;

    // target temperarure for IMU in Celsius, or -1 to disable
    AP_Int8 _imu_target_temperature;

#ifdef HAL_RCINPUT_WITH_AP_RADIO
    // direct attached radio
    AP_Radio _radio;
#endif
    
#if defined(HAL_NEEDS_PARAM_HELPER)
    // HAL specific parameters
    AP_Param_Helper param_helper{false};
#endif
};
