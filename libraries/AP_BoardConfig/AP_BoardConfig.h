#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <sys/ioctl.h>

extern "C" typedef int (*main_fn_t)(int argc, char **);

class AP_BoardConfig {
public:
    // constructor
    AP_BoardConfig(void)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

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

    // valid types for BRD_TYPE: these values need to be in sync with the
    // values from the param description
    enum px4_board_type {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
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
        PX4_BOARD_OLDDRIVERS = 100,
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        VRX_BOARD_BRAIN51  = 7,
        VRX_BOARD_BRAIN52  = 8,
        VRX_BOARD_UBRAIN51 = 9,
        VRX_BOARD_UBRAIN52 = 10,
        VRX_BOARD_CORE10   = 11,
        VRX_BOARD_BRAIN54  = 12,
#endif
    };
#endif

    // set default value for BRD_SAFETY_MASK
    void set_default_safety_ignore_mask(uint16_t mask);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    static enum px4_board_type get_board_type(void) {
        return px4_configured_board;
    }
#endif

private:
    AP_Int16 vehicleSerialNumber;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    struct {
        AP_Int8 pwm_count;
        AP_Int8 safety_enable;
        AP_Int32 ignore_safety_channels;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        AP_Int8 ser1_rtscts;
        AP_Int8 ser2_rtscts;
        AP_Int8 sbus_out_rate;
#endif
        AP_Int8 board_type;
        AP_Int8 io_enable;
    } px4;

    static enum px4_board_type px4_configured_board;

    void px4_drivers_start(void);
    void px4_setup(void);
    void px4_setup_pwm(void);
    void px4_init_safety(void);
    void px4_setup_safety_mask(void);
    void px4_setup_uart(void);
    void px4_setup_sbus(void);
    void px4_setup_drivers(void);
    void px4_setup_peripherals(void);
    void px4_setup_px4io(void);
    void px4_tone_alarm(const char *tone_string);
    bool spi_check_register(const char *devname, uint8_t regnum, uint8_t value, uint8_t read_flag = 0x80);
    void validate_board_type(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    void px4_autodetect(void);
#endif

#endif // HAL_BOARD_PX4 || HAL_BOARD_VRBRAIN

    static bool _in_sensor_config_error;

    // target temperarure for IMU in Celsius, or -1 to disable
    AP_Int8 _imu_target_temperature;
};
