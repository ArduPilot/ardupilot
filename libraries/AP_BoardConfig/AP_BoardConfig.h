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

private:
    AP_Int16 vehicleSerialNumber;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    AP_Int8 _pwm_count;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    AP_Int8 _ser1_rtscts;
    AP_Int8 _ser2_rtscts;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    AP_Int8 _safety_enable;
    AP_Int32 _ignore_safety_channels;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    AP_Int8 _sbus_out_rate;
#ifndef CONFIG_ARCH_BOARD_PX4FMU_V1
    AP_Int8 _can_enable;
#endif
#endif

    // target temperarure for IMU in Celsius, or -1 to disable
    AP_Int8 _imu_target_temperature;
};
