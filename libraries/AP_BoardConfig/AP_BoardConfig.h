/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <sys/ioctl.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && !defined(CONFIG_ARCH_BOARD_PX4FMU_V1)

#define _UAVCAN_IOCBASE             (0x4000)                        // IOCTL base for module UAVCAN
#define _UAVCAN_IOC(_n)             (_IOC(_UAVCAN_IOCBASE, _n))

#define UAVCAN_IOCG_NODEID_INPROGRESS  _UAVCAN_IOC(1)               // query if node identification is in progress
#define UAVCAN_IOCS_HARDPOINT_SET      _UAVCAN_IOC(10)              // control hardpoint (e.g. OpenGrab EPM)

#define UAVCAN_NODE_FILE            "/dev/uavcan/esc"   // design flaw of uavcan driver, this should be /dev/uavcan/node one day

#endif

extern "C" typedef int (*main_fn_t)(int argc, char **);

class AP_BoardConfig
{
public:
    // constructor
    AP_BoardConfig(void)
    {
		AP_Param::setup_object_defaults(this, var_info);
    };

    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // public method to start a driver
    static bool px4_start_driver(main_fn_t main_function, const char *name, const char *arguments);

    // valid types for BRD_TYPE
    enum px4_board_type {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        PX4_BOARD_AUTO     = 0,
        PX4_BOARD_PX4V1    = 1,
        PX4_BOARD_PIXHAWK  = 2,
        PX4_BOARD_PIXHAWK2 = 3,
        PX4_BOARD_PIXRACER = 4,
        PX4_BOARD_PHMINI   = 5,
        PX4_BOARD_PH2SLIM  = 6,
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
        
private:
    AP_Int16 vehicleSerialNumber;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    struct {
        AP_Int8 pwm_count;
        AP_Int8 safety_enable;
        AP_Int32 ignore_safety_channels;
#ifndef CONFIG_ARCH_BOARD_PX4FMU_V1
        AP_Int8 can_enable;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        AP_Int8 ser1_rtscts;
        AP_Int8 ser2_rtscts;
        AP_Int8 sbus_out_rate;
#endif
        AP_Int8 board_type;
    } px4;
    void px4_drivers_start(void);
    void px4_setup(void);
    void px4_setup_pwm(void);
    void px4_setup_safety(void);
    void px4_setup_uart(void);
    void px4_setup_sbus(void);
    void px4_setup_canbus(void);
    void px4_setup_drivers(void);
    void px4_sensor_error(const char *reason);
    
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    void px4_start_common_sensors(void);
    void px4_start_fmuv1_sensors(void);
    void px4_start_fmuv2_sensors(void);
    void px4_start_fmuv4_sensors(void);
    void px4_start_pixhawk2slim_sensors(void);
    void px4_start_phmini_sensors(void);
    void px4_start_optional_sensors(void);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    void vrx_start_common_sensors(void);
    void vrx_start_brain51_sensors(void);
    void vrx_start_brain52_sensors(void);
    void vrx_start_ubrain51_sensors(void);
    void vrx_start_ubrain52_sensors(void);
    void vrx_start_core10_sensors(void);
    void vrx_start_brain54_sensors(void);
    void vrx_start_optional_sensors(void);
#endif

#endif // HAL_BOARD_PX4 || HAL_BOARD_VRBRAIN

    // target temperarure for IMU in Celsius, or -1 to disable
    AP_Int8 _imu_target_temperature;
};
