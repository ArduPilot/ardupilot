/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   AP_BoardConfig - board specific configuration
 */


#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_BoardConfig.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
# define BOARD_SAFETY_ENABLE_DEFAULT 1
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
#define BOARD_PWM_COUNT_DEFAULT 2
#define BOARD_SER1_RTSCTS_DEFAULT 0 // no flow control on UART5 on FMUv1
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
#define BOARD_PWM_COUNT_DEFAULT 6
#define BOARD_SER1_RTSCTS_DEFAULT 2
#else // V2
#define BOARD_PWM_COUNT_DEFAULT 4
#define BOARD_SER1_RTSCTS_DEFAULT 2
#endif
#define BOARD_TYPE_DEFAULT PX4_BOARD_AUTO
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
# define BOARD_SAFETY_ENABLE_DEFAULT 0
# define BOARD_PWM_COUNT_DEFAULT 8
# if defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
#  define BOARD_TYPE_DEFAULT VRX_BOARD_BRAIN51
# elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52)
#  define BOARD_TYPE_DEFAULT VRX_BOARD_BRAIN52
# elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
#  define BOARD_TYPE_DEFAULT VRX_BOARD_UBRAIN51
# elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
#  define BOARD_TYPE_DEFAULT VRX_BOARD_UBRAIN52
# elif defined(CONFIG_ARCH_BOARD_VRCORE_V10)
#  define BOARD_TYPE_DEFAULT VRX_BOARD_CORE10
# elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V54)
#  define BOARD_TYPE_DEFAULT VRX_BOARD_BRAIN54
# endif
#endif

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_BoardConfig::var_info[] = {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Param: PWM_COUNT
    // @DisplayName: Auxiliary pin config
    // @Description: Control assigning of FMU pins to PWM output, timer capture and GPIO. All unassigned pins can be used for GPIO
    // @Values: 0:No PWMs,2:Two PWMs,4:Four PWMs,6:Six PWMs,7:Three PWMs and One Capture
    // @RebootRequired: True
    AP_GROUPINFO("PWM_COUNT",    0, AP_BoardConfig, px4.pwm_count, BOARD_PWM_COUNT_DEFAULT),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Param: SER1_RTSCTS
    // @DisplayName: Serial 1 flow control
    // @Description: Enable flow control on serial 1 (telemetry 1) on Pixhawk. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    AP_GROUPINFO("SER1_RTSCTS",    1, AP_BoardConfig, px4.ser1_rtscts, BOARD_SER1_RTSCTS_DEFAULT),

    // @Param: SER2_RTSCTS
    // @DisplayName: Serial 2 flow control
    // @Description: Enable flow control on serial 2 (telemetry 2) on Pixhawk and PX4. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    AP_GROUPINFO("SER2_RTSCTS",    2, AP_BoardConfig, px4.ser2_rtscts, 2),
#endif
    
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Param: SAFETYENABLE
    // @DisplayName: Enable use of safety arming switch
    // @Description: This controls the default state of the safety switch at startup. When set to 1 the safety switch will start in the safe state (flashing) at boot. When set to zero the safety switch will start in the unsafe state (solid) at startup. Note that if a safety switch is fitted the user can still control the safety state after startup using the switch. The safety state can also be controlled in software using a MAVLink message.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    AP_GROUPINFO("SAFETYENABLE",   3, AP_BoardConfig, px4.safety_enable, BOARD_SAFETY_ENABLE_DEFAULT),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Param: SBUS_OUT
    // @DisplayName:  SBUS output rate
    // @Description: This sets the SBUS output frame rate in Hz
    // @Values: 0:Disabled,1:50Hz,2:75Hz,3:100Hz,4:150Hz,5:200Hz,6:250Hz,7:300Hz
    // @RebootRequired: True
    AP_GROUPINFO("SBUS_OUT",   4, AP_BoardConfig, px4.sbus_out_rate, 0),
#endif

    // @Param: SERIAL_NUM
    // @DisplayName: User-defined serial number
    // @Description: User-defined serial number of this vehicle, it can be any arbitrary number you want and has no effect on the autopilot
    // @Range: -32767 32768
    // @User: Standard
    AP_GROUPINFO("SERIAL_NUM", 5, AP_BoardConfig, vehicleSerialNumber, 0),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && !defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    // @Param: CAN_ENABLE
    // @DisplayName:  Enable use of UAVCAN devices
    // @Description: Enabling this option on a Pixhawk enables UAVCAN devices. Note that this uses about 25k of memory
    // @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO("CAN_ENABLE", 6, AP_BoardConfig, px4.can_enable, 0),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Param: SAFETY_MASK
    // @DisplayName: Channels to which ignore the safety switch state
    // @Description: A bitmask which controls what channels can move while the safety switch has not been pressed
    // @Values: 0:Disabled,1:Enabled
    // @Bitmask: 0:Ch1,1:Ch2,2:Ch3,3:Ch4,4:Ch5,5:Ch6,6:Ch7,7:Ch8
    // @RebootRequired: True
    AP_GROUPINFO("SAFETY_MASK", 7, AP_BoardConfig, px4.ignore_safety_channels, 0),
#endif

#if HAL_HAVE_IMU_HEATER
    // @Param: IMU_TARGTEMP
    // @DisplayName: Target IMU temperature
    // @Description: This sets the target IMU temperature for boards with controllable IMU heating units. A value of -1 disables heating.
    // @Range: -1 80
    // @Units: degreesC
    AP_GROUPINFO("IMU_TARGTEMP", 8, AP_BoardConfig, _imu_target_temperature, HAL_IMU_TEMP_DEFAULT),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Param: TYPE
    // @DisplayName: Board type
    // @Description: This allows selection of a PX4 or VRBRAIN board type. If set to zero then the board type is auto-detected (PX4)
    // @Values: 0:AUTO,1:PX4V1,2:Pixhawk,3:Pixhawk2,4:Pixracer,5:PixhawkMini,6:Pixhawk2Slim,7:VRBrain 5.1,8:VRBrain 5.2,9:VR Micro Brain 5.1,10:VR Micro Brain 5.2,11:VRBrain Core 1.0,12:VRBrain 5.4
    // @RebootRequired: True
    AP_GROUPINFO("TYPE", 9, AP_BoardConfig, px4.board_type, BOARD_TYPE_DEFAULT),
#endif
    
    AP_GROUPEND
};

void AP_BoardConfig::init()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    px4_setup();
#endif
    
#if HAL_HAVE_IMU_HEATER
    // let the HAL know the target temperature. We pass a pointer as
    // we want the user to be able to change the parameter without
    // rebooting
    hal.util->set_imu_target_temp((int8_t *)&_imu_target_temperature);
#endif
}
