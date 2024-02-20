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

#include "AP_BoardConfig.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include <stdio.h>

#ifndef BOARD_TYPE_DEFAULT
#define BOARD_TYPE_DEFAULT PX4_BOARD_AUTO
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#ifndef BOARD_SER1_RTSCTS_DEFAULT
# define BOARD_SER1_RTSCTS_DEFAULT 2
#endif
#ifndef BOARD_TYPE_DEFAULT
# define BOARD_TYPE_DEFAULT PX4_BOARD_AUTO
#endif
#endif

#ifndef BOARD_SAFETY_ENABLE_DEFAULT
#if defined(HAL_GPIO_PIN_SAFETY_IN)
  // have safety startup enabled if we have a safety pin
  # define BOARD_SAFETY_ENABLE_DEFAULT 1
#elif defined(HAL_WITH_IO_MCU)
  // if we have an IOMCU then enable by default
  # define BOARD_SAFETY_ENABLE_DEFAULT HAL_WITH_IO_MCU
#else
  # define BOARD_SAFETY_ENABLE_DEFAULT 0
#endif
#endif

#ifndef HAL_IMU_TEMP_DEFAULT
#define HAL_IMU_TEMP_DEFAULT       -1 // disabled
#endif

#ifndef HAL_IMU_TEMP_MARGIN_LOW_DEFAULT
#define HAL_IMU_TEMP_MARGIN_LOW_DEFAULT 0 // disabled
#endif

#ifndef BOARD_SAFETY_OPTION_DEFAULT
#  define BOARD_SAFETY_OPTION_DEFAULT (BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_OFF|BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON)
#endif
#ifndef BOARD_SAFETY_ENABLE
#  define BOARD_SAFETY_ENABLE 1
#endif

#ifndef BOARD_CONFIG_BOARD_VOLTAGE_MIN
#define BOARD_CONFIG_BOARD_VOLTAGE_MIN 4.3f
#endif

#ifndef HAL_BRD_OPTIONS_DEFAULT
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN) && !APM_BUILD_TYPE(APM_BUILD_Replay)
#ifdef HAL_DEBUG_BUILD
#define HAL_BRD_OPTIONS_DEFAULT BOARD_OPTION_WATCHDOG | BOARD_OPTION_DEBUG_ENABLE
#else
#define HAL_BRD_OPTIONS_DEFAULT BOARD_OPTION_WATCHDOG
#endif
#else
#define HAL_BRD_OPTIONS_DEFAULT 0
#endif
#endif

#ifndef HAL_DEFAULT_BOOT_DELAY
#define HAL_DEFAULT_BOOT_DELAY 0
#endif

extern const AP_HAL::HAL& hal;
AP_BoardConfig *AP_BoardConfig::_singleton;

// constructor
AP_BoardConfig::AP_BoardConfig()
#if HAL_HAVE_IMU_HEATER
    // initialise heater PI controller. Note we do this in the cpp file
    // for ccache efficiency
    : heater{{HAL_IMUHEAT_P_DEFAULT, HAL_IMUHEAT_I_DEFAULT, 70},}
#endif
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
};

// table of user settable parameters
const AP_Param::GroupInfo AP_BoardConfig::var_info[] = {

    // index 0 was used by PWM_COUNT

#if AP_FEATURE_RTSCTS
#ifdef HAL_HAVE_RTSCTS_SERIAL1
    // @Param: SER1_RTSCTS
    // @DisplayName: Serial 1 flow control
    // @Description: Enable flow control on serial 1 (telemetry 1). You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SER1_RTSCTS",    1, AP_BoardConfig, state.ser_rtscts[1], BOARD_SER1_RTSCTS_DEFAULT),
#endif

#ifdef HAL_HAVE_RTSCTS_SERIAL2
    // @Param: SER2_RTSCTS
    // @DisplayName: Serial 2 flow control
    // @Description: Enable flow control on serial 2 (telemetry 2). You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SER2_RTSCTS",    2, AP_BoardConfig, state.ser_rtscts[2], 2),
#endif

#ifdef HAL_HAVE_RTSCTS_SERIAL3
    // @Param: SER3_RTSCTS
    // @DisplayName: Serial 3 flow control
    // @Description: Enable flow control on serial 3. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SER3_RTSCTS",    26, AP_BoardConfig, state.ser_rtscts[3], 2),
#endif

#ifdef HAL_HAVE_RTSCTS_SERIAL4
    // @Param: SER4_RTSCTS
    // @DisplayName: Serial 4 flow control
    // @Description: Enable flow control on serial 4. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SER4_RTSCTS",    27, AP_BoardConfig, state.ser_rtscts[4], 2),
#endif

#ifdef HAL_HAVE_RTSCTS_SERIAL5
    // @Param: SER5_RTSCTS
    // @DisplayName: Serial 5 flow control
    // @Description: Enable flow control on serial 5. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
    // @Values: 0:Disabled,1:Enabled,2:Auto
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SER5_RTSCTS",    25, AP_BoardConfig, state.ser_rtscts[5], 2),
#endif
#endif

    // @Param: SAFETY_DEFLT
    // @DisplayName: Sets default state of the safety switch
    // @Description: This controls the default state of the safety switch at startup. When set to 1 the safety switch will start in the safe state (flashing) at boot. When set to zero the safety switch will start in the unsafe state (solid) at startup. Note that if a safety switch is fitted the user can still control the safety state after startup using the switch. The safety state can also be controlled in software using a MAVLink message.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("SAFETY_DEFLT",   3, AP_BoardConfig, state.safety_enable, BOARD_SAFETY_ENABLE_DEFAULT),

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
    // @Range: -8388608 8388607
    // @User: Standard
    AP_GROUPINFO("SERIAL_NUM", 5, AP_BoardConfig, vehicleSerialNumber, 0),

    // @Param: SAFETY_MASK
    // @DisplayName: Outputs which ignore the safety switch state
    // @Description: A bitmask which controls what outputs can move while the safety switch has not been pressed
    // @Bitmask: 0:Output1,1:Output2,2:Output3,3:Output4,4:Output5,5:Output6,6:Output7,7:Output8,8:Output9,9:Output10,10:Output11,11:Output12,12:Output13,13:Output14
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SAFETY_MASK", 7, AP_BoardConfig, state.ignore_safety_channels, 0),

#if HAL_HAVE_IMU_HEATER
    // @Param: HEAT_TARG
    // @DisplayName: Board heater temperature target
    // @Description: Board heater target temperature for boards with controllable heating units. Set to -1 to disable the heater, please reboot after setting to -1.
    // @Range: -1 80
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("HEAT_TARG", 8, AP_BoardConfig, heater.imu_target_temperature, HAL_IMU_TEMP_DEFAULT),
#endif

#if AP_FEATURE_BOARD_DETECT
    // @Param: TYPE
    // @DisplayName: Board type
    // @Description: This allows selection of a PX4 or VRBRAIN board type. If set to zero then the board type is auto-detected (PX4)
    // @Values: 0:AUTO,1:PX4V1,2:Pixhawk,3:Cube/Pixhawk2,4:Pixracer,5:PixhawkMini,6:Pixhawk2Slim,13:Intel Aero FC,14:Pixhawk Pro,20:AUAV2.1,21:PCNC1,22:MINDPXV2,23:SP01,24:CUAVv5/FMUV5,30:VRX BRAIN51,32:VRX BRAIN52,33:VRX BRAIN52E,34:VRX UBRAIN51,35:VRX UBRAIN52,36:VRX CORE10,38:VRX BRAIN54,39:PX4 FMUV6,100:PX4 OLDDRIVERS
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("TYPE", 9, AP_BoardConfig, state.board_type, BOARD_TYPE_DEFAULT),
#endif

#if HAL_WITH_IO_MCU
    // @Param: IO_ENABLE
    // @DisplayName: Enable IO co-processor
    // @Description: This allows for the IO co-processor on boards with an IOMCU to be disabled. Setting to 2 will enable the IOMCU but not attempt to update firmware on startup
    // @Values: 0:Disabled,1:Enabled,2:EnableNoFWUpdate
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("IO_ENABLE", 10, AP_BoardConfig, state.io_enable, 1),
#endif

#if HAL_RCINPUT_WITH_AP_RADIO
    // @Group: RADIO
    // @Path: ../AP_Radio/AP_Radio.cpp
    AP_SUBGROUPINFO(_radio, "RADIO", 11, AP_BoardConfig, AP_Radio),
#endif

    // @Param: SAFETYOPTION
    // @DisplayName: Options for safety button behavior
    // @Description: This controls the activation of the safety button. It allows you to control if the safety button can be used for safety enable and/or disable, and whether the button is only active when disarmed
    // @Bitmask: 0:ActiveForSafetyDisable,1:ActiveForSafetyEnable,2:ActiveWhenArmed,3:Force safety on when the aircraft disarms
    // @User: Standard
    AP_GROUPINFO("SAFETYOPTION",   13, AP_BoardConfig, state.safety_option, BOARD_SAFETY_OPTION_DEFAULT),

#if AP_RTC_ENABLED
    // @Group: RTC
    // @Path: ../AP_RTC/AP_RTC.cpp
    AP_SUBGROUPINFO(rtc, "RTC", 14, AP_BoardConfig, AP_RTC),
#endif

#if HAL_HAVE_BOARD_VOLTAGE
    // @Param: VBUS_MIN
    // @DisplayName: Autopilot board voltage requirement
    // @Description: Minimum voltage on the autopilot power rail to allow the aircraft to arm. 0 to disable the check.
    // @Units: V
    // @Range: 4.0 5.5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VBUS_MIN",    15,     AP_BoardConfig,  _vbus_min,  BOARD_CONFIG_BOARD_VOLTAGE_MIN),

#endif

#if HAL_HAVE_SERVO_VOLTAGE
    // @Param: VSERVO_MIN
    // @DisplayName: Servo voltage requirement
    // @Description: Minimum voltage on the servo rail to allow the aircraft to arm. 0 to disable the check.
    // @Units: V
    // @Range: 3.3 12.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VSERVO_MIN",    16,     AP_BoardConfig, _vservo_min,  0),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // @Param: SD_SLOWDOWN
    // @DisplayName: microSD slowdown
    // @Description: This is a scaling factor to slow down microSD operation. It can be used on flight board and microSD card combinations where full speed is not reliable. For normal full speed operation a value of 0 should be used.
    // @Range: 0 32
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SD_SLOWDOWN",  17,     AP_BoardConfig, _sdcard_slowdown,  0),
#endif

#ifdef HAL_GPIO_PWM_VOLT_PIN
    // @Param: PWM_VOLT_SEL
    // @DisplayName: Set PWM Out Voltage
    // @Description: This sets the voltage max for PWM output pulses. 0 for 3.3V and 1 for 5V output. On boards with an IOMCU that support this parameter this option only affects the 8 main outputs, not the 6 auxiliary outputs. Using 5V output can help to reduce the impact of ESC noise interference corrupting signals to the ESCs.
    // @Values: 0:3.3V,1:5V
    // @User: Advanced
    AP_GROUPINFO("PWM_VOLT_SEL", 18, AP_BoardConfig, _pwm_volt_sel, 0),
#endif

    // @Param: OPTIONS
    // @DisplayName: Board options
    // @Description: Board specific option flags
    // @Bitmask: 0:Enable hardware watchdog, 1:Disable MAVftp, 2:Enable set of internal parameters, 3:Enable Debug Pins, 4:Unlock flash on reboot, 5:Write protect firmware flash on reboot, 6:Write protect bootloader flash on reboot, 7:Skip board validation, 8:Disable board arming gpio output change on arm/disarm
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 19, AP_BoardConfig, _options, HAL_BRD_OPTIONS_DEFAULT),

    // @Param: BOOT_DELAY
    // @DisplayName: Boot delay
    // @Description: This adds a delay in milliseconds to boot to ensure peripherals initialise fully
    // @Range: 0 10000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("BOOT_DELAY", 20, AP_BoardConfig, _boot_delay_ms, HAL_DEFAULT_BOOT_DELAY),

#if HAL_HAVE_IMU_HEATER
    // @Param: HEAT_P
    // @DisplayName: Board Heater P gain
    // @Description: Board Heater P gain
    // @Range: 1 500
    // @Increment: 1
    // @User: Advanced

    // @Param: HEAT_I
    // @DisplayName: Board Heater I gain
    // @Description: Board Heater integrator gain
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced

    // @Param: HEAT_IMAX
    // @DisplayName: Board Heater IMAX
    // @Description: Board Heater integrator maximum
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_SUBGROUPINFO(heater.pi_controller, "HEAT_",  21, AP_BoardConfig, AC_PI),
#endif

#ifdef HAL_PIN_ALT_CONFIG
    // @Param: ALT_CONFIG
    // @DisplayName: Alternative HW config
    // @Description: Select an alternative hardware configuration. A value of zero selects the default configuration for this board. Other values are board specific. Please see the documentation for your board for details on any alternative configuration values that may be available.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ALT_CONFIG", 22, AP_BoardConfig, _alt_config, 0),
#endif // HAL_PIN_ALT_CONFIG

#if HAL_HAVE_IMU_HEATER
    // @Param: HEAT_LOWMGN
    // @DisplayName: Board heater temp lower margin
    // @Description: Arming check will fail if temp is lower than this margin below BRD_HEAT_TARG. 0 disables the low temperature check
    // @Range: 0 20
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("HEAT_LOWMGN", 23, AP_BoardConfig, heater.imu_arming_temperature_margin_low, HAL_IMU_TEMP_MARGIN_LOW_DEFAULT),
#endif

#if AP_SDCARD_STORAGE_ENABLED
    // @Param: SD_MISSION
    // @DisplayName:  SDCard Mission size
    // @Description: This sets the amount of storage in kilobytes reserved on the microsd card in mission.stg for waypoint storage. Each waypoint uses 15 bytes.
    // @Range: 0 64
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SD_MISSION", 24, AP_BoardConfig, sdcard_storage.mission_kb, 0),

    // @Param: SD_FENCE
    // @DisplayName:  SDCard Fence size
    // @Description: This sets the amount of storage in kilobytes reserved on the microsd card in fence.stg for fence storage.
    // @Range: 0 64
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("SD_FENCE", 29, AP_BoardConfig, sdcard_storage.fence_kb, 0),
#endif

    // index 25 used by SER5_RTSCTS
    // index 26 used by SER3_RTSCTS
    // index 27 used by SER4_RTSCTS

    
#if HAL_WITH_IO_MCU_DSHOT
    // @Param: IO_DSHOT
    // @DisplayName: Load DShot FW on IO
    // @Description: This loads the DShot firmware on the IO co-processor
    // @Values: 0:StandardFW,1:DshotFW
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("IO_DSHOT", 28, AP_BoardConfig, state.io_dshot, 0),
#endif
    AP_GROUPEND
};

void AP_BoardConfig::init()
{
    // PARAMETER_CONVERSION - Added: APR-2022
    vehicleSerialNumber.convert_parameter_width(AP_PARAM_INT16);

    board_setup();

#if AP_RTC_ENABLED
    AP::rtc().set_utc_usec(hal.util->get_hw_rtc(), AP_RTC::SOURCE_HW);
#endif

    if (_boot_delay_ms > 0) {
        uint16_t delay_ms = uint16_t(_boot_delay_ms.get());
        if (hal.util->was_watchdog_armed() && delay_ms > 200) {
            // don't delay a long time on watchdog reset, the pilot
            // may be able to save the vehicle
            delay_ms = 200;
        }
        hal.scheduler->delay(delay_ms);
    }
    
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(USE_POSIX)
    uint8_t slowdown = constrain_int16(_sdcard_slowdown.get(), 0, 32);
    const uint8_t max_slowdown = 8;
    do {
        if (AP::FS().retry_mount()) {
            break;
        }
        slowdown++;
        hal.scheduler->delay(5);
    } while (slowdown < max_slowdown);
    if (slowdown < max_slowdown) {
        _sdcard_slowdown.set(slowdown);
    } else {
        printf("SDCard failed to start\n");
    }
#endif
}

// set default value for BRD_SAFETY_MASK
void AP_BoardConfig::set_default_safety_ignore_mask(uint32_t mask)
{
    state.ignore_safety_channels.set_default(mask);
}

void AP_BoardConfig::init_safety()
{
    board_init_safety();
    board_init_debug();
}

/*
  notify user of a fatal startup error related to available sensors. 
*/
bool AP_BoardConfig::_in_error_loop;

void AP_BoardConfig::throw_error(const char *err_type, const char *fmt, va_list arg)
{
    _in_error_loop = true;
    /*
      to give the user the opportunity to connect to USB we keep
      repeating the error.  The mavlink delay callback is initialised
      before this, so the user can change parameters (and in
      particular BRD_TYPE if needed)
    */
    uint32_t last_print_ms = 0;
    while (true) {
        uint32_t now = AP_HAL::millis();
        if (now - last_print_ms >= 5000) {
            last_print_ms = now;
            char printfmt[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+2];
            hal.util->snprintf(printfmt, sizeof(printfmt), "%s: %s\n", err_type, fmt);
            {
                va_list arg_copy;
                va_copy(arg_copy, arg);
                vprintf(printfmt, arg_copy);
                va_end(arg_copy);
            }
#if HAL_GCS_ENABLED
            hal.util->snprintf(printfmt, sizeof(printfmt), "%s: %s", err_type, fmt);
            {
                va_list arg_copy;
                va_copy(arg_copy, arg);
                gcs().send_textv(MAV_SEVERITY_CRITICAL, printfmt, arg_copy);
                va_end(arg_copy);
            }
#endif
        }
#if HAL_GCS_ENABLED
        gcs().update_receive();
        gcs().update_send();
#endif
        EXPECT_DELAY_MS(10);
        hal.scheduler->delay(5);
    }
}

void AP_BoardConfig::allocation_error(const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    char newfmt[64] {};
    snprintf(newfmt, sizeof(newfmt), "Unable to allocate %s", fmt);
    throw_error("Allocation Error", newfmt, arg_list);
    va_end(arg_list);
}

void AP_BoardConfig::config_error(const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    throw_error("Config Error", fmt, arg_list);
    va_end(arg_list);
}

/*
  handle logic for safety state button press. This should be called at
  10Hz when the button is pressed. The button can either be directly
  on a pin or on a UAVCAN device
  This function returns true if the safety state should be toggled
 */
bool AP_BoardConfig::safety_button_handle_pressed(uint8_t press_count)
{
    if (press_count != 10) {
        return false;
    }
    // get button options
    uint16_t safety_options = get_safety_button_options();
    if (!(safety_options & BOARD_SAFETY_OPTION_BUTTON_ACTIVE_ARMED) &&
        hal.util->get_soft_armed()) {
        return false;
    }
    AP_HAL::Util::safety_state safety_state = hal.util->safety_switch_state();
    if (safety_state == AP_HAL::Util::SAFETY_DISARMED &&
        !(safety_options & BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_OFF)) {
        return false;
    }
    if (safety_state == AP_HAL::Util::SAFETY_ARMED &&
        !(safety_options & BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON)) {
        return false;
    }
    return true;
}

namespace AP {
    AP_BoardConfig *boardConfig(void) {
        return AP_BoardConfig::get_singleton();
    }
};
