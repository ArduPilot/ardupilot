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
 *   AP_Airspeed.cpp - airspeed (pitot) driver
 */

#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "AP_Airspeed.h"

// Dummy the AP_Airspeed class to allow building Airspeed only for plane, rover, sub, and copter & heli 2MB boards
// This could be removed once the build system allows for APM_BUILD_TYPE in header files
#ifndef AP_AIRSPEED_DUMMY_METHODS_ENABLED
#define AP_AIRSPEED_DUMMY_METHODS_ENABLED ((APM_BUILD_COPTER_OR_HELI && BOARD_FLASH_SIZE <= 1024) || \
                                            APM_BUILD_TYPE(APM_BUILD_AntennaTracker) || APM_BUILD_TYPE(APM_BUILD_Blimp))
#endif

#if !AP_AIRSPEED_DUMMY_METHODS_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <utility>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_Airspeed_MS4525.h"
#include "AP_Airspeed_MS5525.h"
#include "AP_Airspeed_SDP3X.h"
#include "AP_Airspeed_DLVR.h"
#include "AP_Airspeed_analog.h"
#include "AP_Airspeed_ASP5033.h"
#include "AP_Airspeed_Backend.h"
#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include "AP_Airspeed_UAVCAN.h"
#endif
#if APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
#include "AP_Airspeed_NMEA.h"
#endif
#if HAL_MSP_AIRSPEED_ENABLED
#include "AP_Airspeed_MSP.h"
#endif
extern const AP_HAL::HAL &hal;

#ifdef HAL_AIRSPEED_TYPE_DEFAULT
 #define ARSPD_DEFAULT_TYPE HAL_AIRSPEED_TYPE_DEFAULT
 #ifndef ARSPD_DEFAULT_PIN
 #define ARSPD_DEFAULT_PIN 1
 #endif
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // The HAL_BOARD_SITL setting is required because of current probe process for MS4525 will
 // connect and find the SIM_DLVR sensors & fault as there is no way to tell them apart
 #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
  #define ARSPD_DEFAULT_TYPE TYPE_ANALOG
  #define ARSPD_DEFAULT_PIN 1
 #else
  #define ARSPD_DEFAULT_TYPE TYPE_I2C_MS4525
  #ifdef HAL_DEFAULT_AIRSPEED_PIN
      #define ARSPD_DEFAULT_PIN HAL_DEFAULT_AIRSPEED_PIN
  #else
     #define ARSPD_DEFAULT_PIN 15
  #endif
 #endif //CONFIG_HAL_BOARD
#else   // All Other Vehicle Types
 #define ARSPD_DEFAULT_TYPE TYPE_NONE
 #define ARSPD_DEFAULT_PIN 15
#endif

#ifndef HAL_AIRSPEED_BUS_DEFAULT
#define HAL_AIRSPEED_BUS_DEFAULT 1
#endif

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#define PSI_RANGE_DEFAULT 0.05
#endif

#ifndef PSI_RANGE_DEFAULT
#define PSI_RANGE_DEFAULT 1.0f
#endif

#define OPTIONS_DEFAULT AP_Airspeed::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE | AP_Airspeed::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE

// table of user settable parameters
const AP_Param::GroupInfo AP_Airspeed::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Airspeed type
    // @Description: Type of airspeed sensor
    // @Values: 0:None,1:I2C-MS4525D0,2:Analog,3:I2C-MS5525,4:I2C-MS5525 (0x76),5:I2C-MS5525 (0x77),6:I2C-SDP3X,7:I2C-DLVR-5in,8:DroneCAN,9:I2C-DLVR-10in,10:I2C-DLVR-20in,11:I2C-DLVR-30in,12:I2C-DLVR-60in,13:NMEA water speed,14:MSP,15:ASP5033
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_Airspeed, param[0].type, ARSPD_DEFAULT_TYPE, AP_PARAM_FLAG_ENABLE),     // NOTE: Index 0 is actually used as index 63 here

    // @Param: _DEVID
    // @DisplayName: Airspeed ID
    // @Description: Airspeed sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_DEVID", 24, AP_Airspeed, param[0].bus_id, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: _USE
    // @DisplayName: Airspeed use
    // @Description: Enables airspeed use for automatic throttle modes and replaces control from THR_TRIM. Continues to display and log airspeed if set to 0. Uses airspeed for control if set to 1. Only uses airspeed when throttle = 0 if set to 2 (useful for gliders with airspeed sensors behind propellers).
    // @Values: 0:DoNotUse,1:Use,2:UseWhenZeroThrottle
    // @User: Standard
    AP_GROUPINFO("_USE",    1, AP_Airspeed, param[0].use, 0),

    // @Param: _OFFSET
    // @DisplayName: Airspeed offset
    // @Description: Airspeed calibration offset
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_OFFSET", 2, AP_Airspeed, param[0].offset, 0),

    // @Param: _RATIO
    // @DisplayName: Airspeed ratio
    // @Description: Calibrates pitot tube pressure to velocity. Increasing this value will indicate a higher airspeed at any given dynamic pressure.
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_RATIO",  3, AP_Airspeed, param[0].ratio, 1.9936f),

    // @Param: _PIN
    // @DisplayName: Airspeed pin
    // @Description: The pin number that the airspeed sensor is connected to for analog sensors. Set to 15 on the Pixhawk for the analog airspeed port. 
    // @User: Advanced
    AP_GROUPINFO("_PIN",  4, AP_Airspeed, param[0].pin, ARSPD_DEFAULT_PIN),
#endif // HAL_BUILD_AP_PERIPH

#if AP_AIRSPEED_AUTOCAL_ENABLE
    // @Param: _AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration
    // @Description: Enables automatic adjustment of ARSPD_RATIO during a calibration flight based on estimation of ground speed and true airspeed. New ratio saved every 2 minutes if change is > 5%. Should not be left enabled.
    // @User: Advanced
    AP_GROUPINFO("_AUTOCAL",  5, AP_Airspeed, param[0].autocal, 0),
#endif

    // @Param: _TUBE_ORDER
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
    // @Values: 0:Normal,1:Swapped,2:Auto Detect
    AP_GROUPINFO("_TUBE_ORDER",  6, AP_Airspeed, param[0].tube_order, 2),

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: _SKIP_CAL
    // @DisplayName: Skip airspeed calibration on startup
    // @Description: This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("_SKIP_CAL",  7, AP_Airspeed, param[0].skip_cal, 0),
#endif // HAL_BUILD_AP_PERIPH

    // @Param: _PSI_RANGE
    // @DisplayName: The PSI range of the device
    // @Description: This parameter allows you to to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device
    // @User: Advanced
    AP_GROUPINFO("_PSI_RANGE",  8, AP_Airspeed, param[0].psi_range, PSI_RANGE_DEFAULT),

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: _BUS
    // @DisplayName: Airspeed I2C bus
    // @Description: Bus number of the I2C bus where the airspeed sensor is connected
    // @Values: 0:Bus0(internal),1:Bus1(external),2:Bus2(auxillary)
    // @User: Advanced
    AP_GROUPINFO("_BUS",  9, AP_Airspeed, param[0].bus, HAL_AIRSPEED_BUS_DEFAULT),
#endif // HAL_BUILD_AP_PERIPH

#if AIRSPEED_MAX_SENSORS > 1
    // @Param: _PRIMARY
    // @DisplayName: Primary airspeed sensor
    // @Description: This selects which airspeed sensor will be the primary if multiple sensors are found
    // @Values: 0:FirstSensor,1:2ndSensor
    // @User: Advanced
    AP_GROUPINFO("_PRIMARY", 10, AP_Airspeed, primary_sensor, 0),
#endif

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: _OPTIONS
    // @DisplayName: Airspeed options bitmask
    // @Description: Bitmask of options to use with airspeed. 0:Disable use based on airspeed/groundspeed mismatch (see ARSPD_WIND_MAX), 1:Automatically reenable use based on airspeed/groundspeed mismatch recovery (see ARSPD_WIND_MAX) 2:Disable voltage correction
    // @Bitmask: 0:SpeedMismatchDisable, 1:AllowSpeedMismatchRecovery, 2:DisableVoltageCorrection
    // @User: Advanced
    AP_GROUPINFO("_OPTIONS", 21, AP_Airspeed, _options, OPTIONS_DEFAULT),

    // @Param: _WIND_MAX
    // @DisplayName: Maximum airspeed and ground speed difference
    // @Description: If the difference between airspeed and ground speed is greater than this value the sensor will be marked unhealthy. Using ARSPD_OPTION this health value can be used to disable the sensor.
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("_WIND_MAX", 22, AP_Airspeed, _wind_max, 0),

    // @Param: _WIND_WARN
    // @DisplayName: Airspeed and ground speed difference that gives a warning
    // @Description: If the difference between airspeed and ground speed is greater than this value the sensor will issue a warning. If 0 ARSPD_WIND_MAX is used.
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("_WIND_WARN", 23, AP_Airspeed, _wind_warn, 0),
#endif

#if AIRSPEED_MAX_SENSORS > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Airspeed type
    // @Description: Type of 2nd airspeed sensor
    // @Values: 0:None,1:I2C-MS4525D0,2:Analog,3:I2C-MS5525,4:I2C-MS5525 (0x76),5:I2C-MS5525 (0x77),6:I2C-SDP3X,7:I2C-DLVR-5in,8:DroneCAN,9:I2C-DLVR-10in,10:I2C-DLVR-20in,11:I2C-DLVR-30in,12:I2C-DLVR-60in,13:NMEA water speed,14:MSP,15:ASP5033
    // @User: Standard
    AP_GROUPINFO_FLAGS("2_TYPE", 11, AP_Airspeed, param[1].type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: 2_USE
    // @DisplayName: Enable use of 2nd airspeed sensor
    // @Description: use airspeed for flight control. When set to 0 airspeed sensor can be logged and displayed on a GCS but won't be used for flight. When set to 1 it will be logged and used. When set to 2 it will be only used when the throttle is zero, which can be useful in gliders with airspeed sensors behind a propeller
    // @Values: 0:Don't Use,1:use,2:UseWhenZeroThrottle
    // @User: Standard
    AP_GROUPINFO("2_USE",    12, AP_Airspeed, param[1].use, 0),

    // @Param: 2_OFFSET
    // @DisplayName: Airspeed offset for 2nd airspeed sensor
    // @Description: Airspeed calibration offset
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("2_OFFSET", 13, AP_Airspeed, param[1].offset, 0),

    // @Param: 2_RATIO
    // @DisplayName: Airspeed ratio for 2nd airspeed sensor
    // @Description: Airspeed calibration ratio
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("2_RATIO",  14, AP_Airspeed, param[1].ratio, 2),

    // @Param: 2_PIN
    // @DisplayName: Airspeed pin for 2nd airspeed sensor
    // @Description: Pin number indicating location of analog airspeed sensors. Pixhawk/Cube if set to 15. 
    // @User: Advanced
    AP_GROUPINFO("2_PIN",  15, AP_Airspeed, param[1].pin, 0),

    // @Param: 2_AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration for 2nd airspeed sensor
    // @Description: If this is enabled then the autopilot will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%. This option should be enabled for a calibration flight then disabled again when calibration is complete. Leaving it enabled all the time is not recommended.
    // @User: Advanced
    AP_GROUPINFO("2_AUTOCAL",  16, AP_Airspeed, param[1].autocal, 0),

    // @Param: 2_TUBE_ORDR
    // @DisplayName: Control pitot tube order of 2nd airspeed sensor
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
    // @Values: 0:Normal,1:Swapped,2:Auto Detect
    AP_GROUPINFO("2_TUBE_ORDR",  17, AP_Airspeed, param[1].tube_order, 2),

    // @Param: 2_SKIP_CAL
    // @DisplayName: Skip airspeed calibration on startup for 2nd sensor
    // @Description: This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("2_SKIP_CAL",  18, AP_Airspeed, param[1].skip_cal, 0),

    // @Param: 2_PSI_RANGE
    // @DisplayName: The PSI range of the device for 2nd sensor
    // @Description: This parameter allows you to to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device
    // @User: Advanced
    AP_GROUPINFO("2_PSI_RANGE",  19, AP_Airspeed, param[1].psi_range, PSI_RANGE_DEFAULT),

    // @Param: 2_BUS
    // @DisplayName: Airspeed I2C bus for 2nd sensor
    // @Description: The bus number of the I2C bus to look for the sensor on
    // @Values: 0:Bus0(internal),1:Bus1(external),2:Bus2(auxillary)
    // @User: Advanced
    AP_GROUPINFO("2_BUS",  20, AP_Airspeed, param[1].bus, 1),

#if AIRSPEED_MAX_SENSORS > 1
    // @Param: 2_DEVID
    // @DisplayName: Airspeed2 ID
    // @Description: Airspeed2 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("2_DEVID", 25, AP_Airspeed, param[1].bus_id, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),
#endif
    
#endif // AIRSPEED_MAX_SENSORS

    // Note that 21, 22 and 23 are used above by the _OPTIONS, _WIND_MAX and _WIND_WARN parameters.  Do not use them!!

    // NOTE: Index 63 is used by AIRSPEED_TYPE, Do not use it!: AP_Param converts an index of 0 to 63 so that the index may be bit shifted
    AP_GROUPEND
};

/*
  this scaling factor converts from the old system where we used a
  0 to 4095 raw ADC value for 0-5V to the new system which gets the
  voltage in volts directly from the ADC driver
 */
#define SCALING_OLD_CALIBRATION 819 // 4095/5

AP_Airspeed::AP_Airspeed()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Airspeed must be singleton");
    }
    _singleton = this;
}

// macro for use by HAL_INS_PROBE_LIST
#define GET_I2C_DEVICE(bus, address) hal.i2c_mgr->get_device(bus, address)

bool AP_Airspeed::add_backend(AP_Airspeed_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_sensors >= AIRSPEED_MAX_SENSORS) {
        AP_HAL::panic("Too many airspeed drivers");
    }
    const uint8_t i = num_sensors;
    sensor[num_sensors++] = backend;
    if (!sensor[i]->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airspeed %u init failed", i+1);
        delete sensor[i];
        sensor[i] = nullptr;
    }
    return true;
}

/*
  macro to add a backend with check for too many sensors
  We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(backend) \
    do { add_backend(backend);     \
    if (num_sensors == AIRSPEED_MAX_SENSORS) { return; } \
    } while (0)

void AP_Airspeed::init()
{   
    if (sensor[0] != nullptr) {
        // already initialised
        return;
    }
    // cope with upgrade from old system
    if (param[0].pin.load() && param[0].pin.get() != 65) {
        param[0].type.set_default(TYPE_ANALOG);
    }

#ifndef HAL_BUILD_AP_PERIPH
    // Switch to dedicated WIND_MAX param
    // PARAMETER_CONVERSION - Added: Oct-2020
    const float ahrs_max_wind = AP::ahrs().get_max_wind();
    if (!_wind_max.configured() && is_positive(ahrs_max_wind)) {
        _wind_max.set_and_save(ahrs_max_wind);

        // Turn off _options to override the new default
        if (!_options.configured()) {
            _options.set_and_save(0);
        }
    }
#endif

#ifdef HAL_AIRSPEED_PROBE_LIST
    // load sensors via a list from hwdef.dat
    HAL_AIRSPEED_PROBE_LIST;
#else
    // look for sensors based on type parameters
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
#if AP_AIRSPEED_AUTOCAL_ENABLE
        state[i].calibration.init(param[i].ratio);
        state[i].last_saved_ratio = param[i].ratio;
#endif

        // Set the enable automatically to false and set the probability that the airspeed is healhy to start with
        state[i].failures.health_probability = 1.0f;

        switch ((enum airspeed_type)param[i].type.get()) {
        case TYPE_NONE:
            // nothing to do
            break;
        case TYPE_I2C_MS4525:
            sensor[i] = new AP_Airspeed_MS4525(*this, i);
            break;
        case TYPE_ANALOG:
            sensor[i] = new AP_Airspeed_Analog(*this, i);
            break;
        case TYPE_I2C_MS5525:
            sensor[i] = new AP_Airspeed_MS5525(*this, i, AP_Airspeed_MS5525::MS5525_ADDR_AUTO);
            break;
        case TYPE_I2C_MS5525_ADDRESS_1:
            sensor[i] = new AP_Airspeed_MS5525(*this, i, AP_Airspeed_MS5525::MS5525_ADDR_1);
            break;
        case TYPE_I2C_MS5525_ADDRESS_2:
            sensor[i] = new AP_Airspeed_MS5525(*this, i, AP_Airspeed_MS5525::MS5525_ADDR_2);
            break;
        case TYPE_I2C_SDP3X:
            sensor[i] = new AP_Airspeed_SDP3X(*this, i);
            break;
        case TYPE_I2C_DLVR_5IN:
#if !HAL_MINIMIZE_FEATURES
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 5);
#endif // !HAL_MINIMIZE_FEATURES
            break;
        case TYPE_I2C_DLVR_10IN:
#if !HAL_MINIMIZE_FEATURES
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 10);
#endif // !HAL_MINIMIZE_FEATURES
            break;
        case TYPE_I2C_DLVR_20IN:
#if !HAL_MINIMIZE_FEATURES
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 20);
#endif // !HAL_MINIMIZE_FEATURES
            break;
        case TYPE_I2C_DLVR_30IN:
#if !HAL_MINIMIZE_FEATURES
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 30);
#endif // !HAL_MINIMIZE_FEATURES
            break;
        case TYPE_I2C_DLVR_60IN:
#if !HAL_MINIMIZE_FEATURES
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 60);
#endif // !HAL_MINIMIZE_FEATURES
            break;
        case TYPE_I2C_ASP5033:
#if !HAL_MINIMIZE_FEATURES
            sensor[i] = new AP_Airspeed_ASP5033(*this, i);
#endif // !HAL_MINIMIZE_FEATURES
            break;
        case TYPE_UAVCAN:
#if HAL_ENABLE_LIBUAVCAN_DRIVERS
            sensor[i] = AP_Airspeed_UAVCAN::probe(*this, i);
#endif
            break;
        case TYPE_NMEA_WATER:
#if APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_TYPE(APM_BUILD_ArduSub) 
            sensor[i] = new AP_Airspeed_NMEA(*this, i);
#endif
            break;
        case TYPE_MSP:
#if HAL_MSP_AIRSPEED_ENABLED
            sensor[i] = new AP_Airspeed_MSP(*this, i, 0);
#endif
            break;
        }
        if (sensor[i] && !sensor[i]->init()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airspeed %u init failed", i + 1);
            delete sensor[i];
            sensor[i] = nullptr;
        }
        if (sensor[i] != nullptr) {
            num_sensors = i+1;
        }
    }
#endif // HAL_AIRSPEED_PROBE_LIST
}

// read the airspeed sensor
float AP_Airspeed::get_pressure(uint8_t i)
{
    if (!enabled(i)) {
        return 0;
    }
    float pressure = 0;
    if (sensor[i]) {
        state[i].healthy = sensor[i]->get_differential_pressure(pressure);
    }
    return pressure;
}

// get a temperature reading if possible
bool AP_Airspeed::get_temperature(uint8_t i, float &temperature)
{
    if (!enabled(i)) {
        return false;
    }
    if (sensor[i]) {
        return sensor[i]->get_temperature(temperature);
    }
    return false;
}

// calibrate the zero offset for the airspeed. This must be called at
// least once before the get_airspeed() interface can be used
void AP_Airspeed::calibrate(bool in_startup)
{
    if (hal.util->was_watchdog_reset()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Airspeed: skipping cal");
        return;
    }
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (!enabled(i)) {
            continue;
        }
        if (state[i].use_zero_offset) {
            param[i].offset.set(0);
            continue;
        }
        if (in_startup && param[i].skip_cal) {
            continue;
        }
        if (sensor[i] == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airspeed %u not initalized, cannot cal", i+1);
            continue;
        }
        state[i].cal.start_ms = AP_HAL::millis();
        state[i].cal.count = 0;
        state[i].cal.sum = 0;
        state[i].cal.read_count = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Airspeed %u calibration started", i+1);
    }
}

/*
  update async airspeed zero offset calibration
*/
void AP_Airspeed::update_calibration(uint8_t i, float raw_pressure)
{
    if (!enabled(i) || state[i].cal.start_ms == 0) {
        return;
    }
    
    // consider calibration complete when we have at least 15 samples
    // over at least 1 second
    if (AP_HAL::millis() - state[i].cal.start_ms >= 1000 &&
        state[i].cal.read_count > 15) {
        if (state[i].cal.count == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Airspeed %u unhealthy", i + 1);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Airspeed %u calibrated", i + 1);
            param[i].offset.set_and_save(state[i].cal.sum / state[i].cal.count);
        }
        state[i].cal.start_ms = 0;
        return;
    }
    // we discard the first 5 samples
    if (state[i].healthy && state[i].cal.read_count > 5) {
        state[i].cal.sum += raw_pressure;
        state[i].cal.count++;
    }
    state[i].cal.read_count++;
}

// read one airspeed sensor
void AP_Airspeed::read(uint8_t i)
{
    if (!enabled(i) || !sensor[i]) {
        return;
    }
    state[i].last_update_ms = AP_HAL::millis();

    // try and get a direct reading of airspeed
    if (sensor[i]->has_airspeed()) {
        state[i].healthy = sensor[i]->get_airspeed(state[i].airspeed);
        state[i].raw_airspeed = state[i].airspeed;  // for logging
        return;
    }

    bool prev_healthy = state[i].healthy;
    float raw_pressure = get_pressure(i);
    if (state[i].cal.start_ms != 0) {
        update_calibration(i, raw_pressure);
    }

    float airspeed_pressure = raw_pressure - param[i].offset;

    // remember raw pressure for logging
    state[i].corrected_pressure = airspeed_pressure;

    // filter before clamping positive
    if (!prev_healthy) {
        // if the previous state was not healthy then we should not
        // use an IIR filter, otherwise a bad reading will last for
        // some time after the sensor becomees healthy again
        state[i].filtered_pressure = airspeed_pressure;
    } else {
        state[i].filtered_pressure = 0.7f * state[i].filtered_pressure + 0.3f * airspeed_pressure;
    }

    /*
      we support different pitot tube setups so user can choose if
      they want to be able to detect pressure on the static port
     */
    switch ((enum pitot_tube_order)param[i].tube_order.get()) {
    case PITOT_TUBE_ORDER_NEGATIVE:
        state[i].last_pressure  = -airspeed_pressure;
        state[i].raw_airspeed   = sqrtf(MAX(-airspeed_pressure, 0) * param[i].ratio);
        state[i].airspeed       = sqrtf(MAX(-state[i].filtered_pressure, 0) * param[i].ratio);
        break;
    case PITOT_TUBE_ORDER_POSITIVE:
        state[i].last_pressure  = airspeed_pressure;
        state[i].raw_airspeed   = sqrtf(MAX(airspeed_pressure, 0) * param[i].ratio);
        state[i].airspeed       = sqrtf(MAX(state[i].filtered_pressure, 0) * param[i].ratio);
        break;
    case PITOT_TUBE_ORDER_AUTO:
    default:
        state[i].last_pressure  = fabsf(airspeed_pressure);
        state[i].raw_airspeed   = sqrtf(fabsf(airspeed_pressure) * param[i].ratio);
        state[i].airspeed       = sqrtf(fabsf(state[i].filtered_pressure) * param[i].ratio);
        break;
    }

    if (state[i].last_pressure < -32) {
        // we're reading more than about -8m/s. The user probably has
        // the ports the wrong way around
        state[i].healthy = false;
    }

}

// read all airspeed sensors
void AP_Airspeed::update()
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        read(i);
    }

#if HAL_GCS_ENABLED
    // debugging until we get MAVLink support for 2nd airspeed sensor
    if (enabled(1)) {
        gcs().send_named_float("AS2", get_airspeed(1));
    }
#endif

    // setup primary
    if (healthy(primary_sensor.get())) {
        primary = primary_sensor.get();
    } else {
        for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
            if (healthy(i)) {
                primary = i;
                break;
            }
        }
    }

    check_sensor_failures();

#if HAL_LOGGING_ENABLED
    if (_log_bit != (uint32_t)-1 && AP::logger().should_log(_log_bit)) {
        Log_Airspeed();
    }
#endif
}

#if HAL_MSP_AIRSPEED_ENABLED
/*
  handle MSP airspeed data
 */
void AP_Airspeed::handle_msp(const MSP::msp_airspeed_data_message_t &pkt)
{
    
    if (pkt.instance > 1) {
        return; //supporting 2 airspeed sensors at most
    }

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (sensor[i]) {
            sensor[i]->handle_msp(pkt);
        }
    }
}
#endif 

void AP_Airspeed::Log_Airspeed()
{
    const uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (!enabled(i)) {
            continue;
        }
        float temperature;
        if (!get_temperature(i, temperature)) {
            temperature = 0;
        }
        const struct log_ARSP pkt{
            LOG_PACKET_HEADER_INIT(LOG_ARSP_MSG),
            time_us       : now,
            instance      : i,
            airspeed      : get_raw_airspeed(i),
            diffpressure  : get_differential_pressure(i),
            temperature   : (int16_t)(temperature * 100.0f),
            rawpressure   : get_corrected_pressure(i),
            offset        : get_offset(i),
            use           : use(i),
            healthy       : healthy(i),
            health_prob   : get_health_probability(i),
            primary       : get_primary()
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

bool AP_Airspeed::use(uint8_t i) const
{
    if (_force_disable_use) {
        return false;
    }
    if (!enabled(i) || !param[i].use) {
        return false;
    }
#ifndef HAL_BUILD_AP_PERIPH
    if (param[i].use == 2 && !is_zero(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))) {
        // special case for gliders with airspeed sensors behind the
        // propeller. Allow airspeed to be disabled when throttle is
        // running
        return false;
    }
#endif
    return true;
}

/*
  return true if all enabled sensors are healthy
 */
bool AP_Airspeed::all_healthy(void) const
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (enabled(i) && !healthy(i)) {
            return false;
        }
    }
    return true;
}

#else  // build type is not appropriate; provide a dummy implementation:
const AP_Param::GroupInfo AP_Airspeed::var_info[] = { AP_GROUPEND };

void AP_Airspeed::update() {};
bool AP_Airspeed::get_temperature(uint8_t i, float &temperature) { return false; }
void AP_Airspeed::calibrate(bool in_startup) {}
bool AP_Airspeed::use(uint8_t i) const { return false; }

#if HAL_MSP_AIRSPEED_ENABLED
void AP_Airspeed::handle_msp(const MSP::msp_airspeed_data_message_t &pkt) {}
#endif

bool AP_Airspeed::all_healthy(void) const { return false; }
void AP_Airspeed::init(void) {};
AP_Airspeed::AP_Airspeed() {}

#endif // #if AP_AIRSPEED_DUMMY_METHODS_ENABLED

// singleton instance
AP_Airspeed *AP_Airspeed::_singleton;

namespace AP {

AP_Airspeed *airspeed()
{
    return AP_Airspeed::get_singleton();
}

};
