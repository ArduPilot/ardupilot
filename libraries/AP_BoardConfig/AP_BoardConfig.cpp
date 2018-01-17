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
#include <GCS_MAVLink/GCS.h>
#include "AP_BoardConfig.h"
#include <stdio.h>

#if HAL_WITH_UAVCAN
#include <AP_UAVCAN/AP_UAVCAN.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/CAN.h>
#endif
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
# define BOARD_SAFETY_ENABLE_DEFAULT 1
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
#define BOARD_PWM_COUNT_DEFAULT 2
#define BOARD_SER1_RTSCTS_DEFAULT 0 // no flow control on UART5 on FMUv1
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
#define BOARD_PWM_COUNT_DEFAULT 6
#define BOARD_SER1_RTSCTS_DEFAULT 2
#elif defined(CONFIG_ARCH_BOARD_AEROFC_V1)
#define BOARD_PWM_COUNT_DEFAULT 0
#define BOARD_SER1_RTSCTS_DEFAULT 0
# undef BOARD_SAFETY_ENABLE_DEFAULT
# define BOARD_SAFETY_ENABLE_DEFAULT 0
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

#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
# define BOARD_SAFETY_ENABLE_DEFAULT 1
# define BOARD_PWM_COUNT_DEFAULT 6
# define BOARD_SER1_RTSCTS_DEFAULT 2
# define BOARD_TYPE_DEFAULT PX4_BOARD_AUTO
#endif

#ifndef HAL_IMU_TEMP_DEFAULT
#define HAL_IMU_TEMP_DEFAULT       -1 // disabled
#endif

extern const AP_HAL::HAL& hal;
AP_BoardConfig *AP_BoardConfig::instance;

// table of user settable parameters
const AP_Param::GroupInfo AP_BoardConfig::var_info[] = {
#if AP_FEATURE_BOARD_DETECT
    // @Param: PWM_COUNT
    // @DisplayName: Auxiliary pin config
    // @Description: Control assigning of FMU pins to PWM output, timer capture and GPIO. All unassigned pins can be used for GPIO
    // @Values: 0:No PWMs,2:Two PWMs,4:Four PWMs,6:Six PWMs,7:Three PWMs and One Capture
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PWM_COUNT",    0, AP_BoardConfig, state.pwm_count, BOARD_PWM_COUNT_DEFAULT),
#endif

#if AP_FEATURE_RTSCTS
    // @Param: SER1_RTSCTS
    // @DisplayName: Serial 1 flow control
    // @Description: Enable flow control on serial 1 (telemetry 1) on Pixhawk. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SER1_RTSCTS",    1, AP_BoardConfig, state.ser1_rtscts, BOARD_SER1_RTSCTS_DEFAULT),

    // @Param: SER2_RTSCTS
    // @DisplayName: Serial 2 flow control
    // @Description: Enable flow control on serial 2 (telemetry 2) on Pixhawk and STATE. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SER2_RTSCTS",    2, AP_BoardConfig, state.ser2_rtscts, 2),
#endif

#if AP_FEATURE_SAFETY_BUTTON
    // @Param: SAFETYENABLE
    // @DisplayName: Enable use of safety arming switch
    // @Description: This controls the default state of the safety switch at startup. When set to 1 the safety switch will start in the safe state (flashing) at boot. When set to zero the safety switch will start in the unsafe state (solid) at startup. Note that if a safety switch is fitted the user can still control the safety state after startup using the switch. The safety state can also be controlled in software using a MAVLink message.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("SAFETYENABLE",   3, AP_BoardConfig, state.safety_enable, BOARD_SAFETY_ENABLE_DEFAULT),
#endif

#if AP_FEATURE_SBUS_OUT
    // @Param: SBUS_OUT
    // @DisplayName:  SBUS output rate
    // @Description: This sets the SBUS output frame rate in Hz
    // @Values: 0:Disabled,1:50Hz,2:75Hz,3:100Hz,4:150Hz,5:200Hz,6:250Hz,7:300Hz
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SBUS_OUT",   4, AP_BoardConfig, state.sbus_out_rate, 0),
#endif

    // @Param: SERIAL_NUM
    // @DisplayName: User-defined serial number
    // @Description: User-defined serial number of this vehicle, it can be any arbitrary number you want and has no effect on the autopilot
    // @Range: -32768 32767
    // @User: Standard
    AP_GROUPINFO("SERIAL_NUM", 5, AP_BoardConfig, vehicleSerialNumber, 0),

#if AP_FEATURE_SAFETY_BUTTON
    // @Param: SAFETY_MASK
    // @DisplayName: Channels to which ignore the safety switch state
    // @Description: A bitmask which controls what channels can move while the safety switch has not been pressed
    // @Values: 0:Disabled,1:Enabled
    // @Bitmask: 0:Ch1,1:Ch2,2:Ch3,3:Ch4,4:Ch5,5:Ch6,6:Ch7,7:Ch8,8:Ch9,9:Ch10,10:Ch11,11:Ch12,12:Ch13,13:Ch14
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SAFETY_MASK", 7, AP_BoardConfig, state.ignore_safety_channels, 0),
#endif

#if HAL_HAVE_IMU_HEATER
    // @Param: IMU_TARGTEMP
    // @DisplayName: Target IMU temperature
    // @Description: This sets the target IMU temperature for boards with controllable IMU heating units. DO NOT SET -1 on The Cube. A value of -1 sets PH1 behaviour 
    // @Range: -1 80
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("IMU_TARGTEMP", 8, AP_BoardConfig, _imu_target_temperature, HAL_IMU_TEMP_DEFAULT),
#endif

#if AP_FEATURE_BOARD_DETECT
    // @Param: TYPE
    // @DisplayName: Board type
    // @Description: This allows selection of a PX4 or VRBRAIN board type. If set to zero then the board type is auto-detected (PX4)
    // @Values: 0:AUTO,1:PX4V1,2:Pixhawk,3:Cube/Pixhawk2,4:Pixracer,5:PixhawkMini,6:Pixhawk2Slim,7:VRBrain 5.1,8:VRBrain 5.2,9:VR Micro Brain 5.1,10:VR Micro Brain 5.2,11:VRBrain Core 1.0,12:VRBrain 5.4,13:Intel Aero FC,20:AUAV2.1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TYPE", 9, AP_BoardConfig, state.board_type, BOARD_TYPE_DEFAULT),
#endif

#if AP_FEATURE_BOARD_DETECT
#if HAL_PX4_HAVE_PX4IO || HAL_WITH_IO_MCU
    // @Param: IO_ENABLE
    // @DisplayName: Enable IO co-processor
    // @Description: This allows for the IO co-processor on FMUv1 and FMUv2 to be disabled
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("IO_ENABLE", 10, AP_BoardConfig, state.io_enable, 1),
#endif
#endif

    // ID number 11 reserved for AP_Radio (pending PR)
    
    AP_GROUPEND
};

void AP_BoardConfig::init()
{
    board_setup();

#if HAL_HAVE_IMU_HEATER
    // let the HAL know the target temperature. We pass a pointer as
    // we want the user to be able to change the parameter without
    // rebooting
    hal.util->set_imu_target_temp((int8_t *)&_imu_target_temperature);
#endif
}

// set default value for BRD_SAFETY_MASK
void AP_BoardConfig::set_default_safety_ignore_mask(uint16_t mask)
{
#if AP_FEATURE_SAFETY_BUTTON
    state.ignore_safety_channels.set_default(mask);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    px4_setup_safety_mask();
#endif
#endif
}

void AP_BoardConfig::init_safety()
{
#if AP_FEATURE_SAFETY_BUTTON
    board_init_safety();
#endif
}

/*
  notify user of a fatal startup error related to available sensors. 
*/
bool AP_BoardConfig::_in_sensor_config_error;

void AP_BoardConfig::sensor_config_error(const char *reason)
{
    _in_sensor_config_error = true;
    /*
      to give the user the opportunity to connect to USB we keep
      repeating the error.  The mavlink delay callback is initialised
      before this, so the user can change parameters (and in
      particular BRD_TYPE if needed)
    */
    while (true) {
        printf("Sensor failure: %s\n", reason);
        gcs().send_text(MAV_SEVERITY_ERROR, "Check BRD_TYPE: %s", reason);
        hal.scheduler->delay(3000);
    }
}
