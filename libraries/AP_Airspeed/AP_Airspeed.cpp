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

#include "AP_Airspeed.h"

#include <AP_Vehicle/AP_Vehicle_Type.h>

// Dummy the AP_Airspeed class to allow building Airspeed only for plane, rover, sub, and copter & heli 2MB boards
// This could be removed once the build system allows for APM_BUILD_TYPE in header files
// Note that this is also defined in AP_Airspeed_Params.cpp
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
#include "AP_Airspeed_MS4525.h"
#include "AP_Airspeed_MS5525.h"
#include "AP_Airspeed_SDP3X.h"
#include "AP_Airspeed_DLVR.h"
#include "AP_Airspeed_analog.h"
#include "AP_Airspeed_ASP5033.h"
#include "AP_Airspeed_Backend.h"
#include "AP_Airspeed_DroneCAN.h"
#include "AP_Airspeed_NMEA.h"
#include "AP_Airspeed_MSP.h"
#include "AP_Airspeed_SITL.h"
extern const AP_HAL::HAL &hal;

#include <AP_Vehicle/AP_FixedWing.h>

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

#define OPTIONS_DEFAULT AP_Airspeed::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE | AP_Airspeed::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE | AP_Airspeed::OptionsMask::USE_EKF_CONSISTENCY

#define ENABLE_PARAMETER !(APM_BUILD_TYPE(APM_BUILD_ArduPlane) || defined(HAL_BUILD_AP_PERIPH))

// table of user settable parameters
const AP_Param::GroupInfo AP_Airspeed::var_info[] = {

#if ENABLE_PARAMETER
    // @Param: _ENABLE
    // @DisplayName: Airspeed Enable
    // @Description: Enable airspeed sensor support
    // @Values: 0:Disable, 1:Enable
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 30, AP_Airspeed, _enable, 0, AP_PARAM_FLAG_ENABLE),
#endif
    // slots 0-9 (and 63) were previously used by params before being refactored into AP_Airspeed_Params

    // @Param: _TUBE_ORDER
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
    // @Values: 0:Normal,1:Swapped,2:Auto Detect

    // tube order param had to be shortened so is not preserved in per group descriptions 

#if AIRSPEED_MAX_SENSORS > 1
    // @Param: _PRIMARY
    // @DisplayName: Primary airspeed sensor
    // @Description: This selects which airspeed sensor will be the primary if multiple sensors are found
    // @Values: 0:FirstSensor,1:2ndSensor
    // @User: Advanced
    AP_GROUPINFO("_PRIMARY", 10, AP_Airspeed, primary_sensor, 0),
#endif

    // 11-20 were previously used by second sensor params before being refactored into AP_Airspeed_Params

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: _OPTIONS
    // @DisplayName: Airspeed options bitmask
    // @Description: Bitmask of options to use with airspeed. 0:Disable use based on airspeed/groundspeed mismatch (see ARSPD_WIND_MAX), 1:Automatically reenable use based on airspeed/groundspeed mismatch recovery (see ARSPD_WIND_MAX) 2:Disable voltage correction, 3:Check that the airspeed is statistically consistent with the navigation EKF vehicle and wind velocity estimates using EKF3 (requires AHRS_EKF_TYPE = 3)
    // @Description{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle. Always set to 0.
    // @Bitmask: 0:SpeedMismatchDisable, 1:AllowSpeedMismatchRecovery, 2:DisableVoltageCorrection, 3:UseEkf3Consistency
    // @User: Advanced
    AP_GROUPINFO("_OPTIONS", 21, AP_Airspeed, _options, OPTIONS_DEFAULT),

    // @Param: _WIND_MAX
    // @DisplayName: Maximum airspeed and ground speed difference
    // @Description: If the difference between airspeed and ground speed is greater than this value the sensor will be marked unhealthy. Using ARSPD_OPTION this health value can be used to disable the sensor.
    // @Description{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle. Always set to 0.
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("_WIND_MAX", 22, AP_Airspeed, _wind_max, 0),

    // @Param: _WIND_WARN
    // @DisplayName: Airspeed and ground speed difference that gives a warning
    // @Description: If the difference between airspeed and ground speed is greater than this value the sensor will issue a warning. If 0 ARSPD_WIND_MAX is used.
    // @Description{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle. Always set to 0.
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("_WIND_WARN", 23, AP_Airspeed, _wind_warn, 0),

    // @Param: _WIND_GATE
    // @DisplayName: Re-enable Consistency Check Gate Size
    // @Description: Number of standard deviations applied to the re-enable EKF consistency check that is used when ARSPD_OPTIONS bit position 3 is set. Larger values will make the re-enabling of the airspeed sensor faster, but increase the likelihood of re-enabling a degraded sensor. The value can be tuned by using the ARSP.TR log message by setting ARSPD_WIND_GATE to a value that is higher than the value for ARSP.TR observed with a healthy airspeed sensor. Occasional transients in ARSP.TR above the value set by ARSPD_WIND_GATE can be tolerated provided they are less than 5 seconds in duration and less than 10% duty cycle.
    // @Description{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle.
    // @Range: 0.0 10.0
    // @User: Advanced
    AP_GROUPINFO("_WIND_GATE", 26, AP_Airspeed, _wind_gate, 5.0f),
    
    // @Param: _OFF_PCNT
    // @DisplayName: Maximum offset cal speed error 
    // @Description: The maximum percentage speed change in airspeed reports that is allowed due to offset changes between calibraions before a warning is issued. This potential speed error is in percent of ASPD_FBW_MIN. 0 disables. Helps warn of calibrations without pitot being covered.
    // @Range: 0.0 10.0
    // @Units: %
    // @User: Advanced
    AP_GROUPINFO_FRAME("_OFF_PCNT", 27, AP_Airspeed, max_speed_pcnt, 0, AP_PARAM_FRAME_PLANE),    

#endif

    // @Group: _
    // @Path: AP_Airspeed_Params.cpp
    AP_SUBGROUPINFO(param[0], "_", 28, AP_Airspeed, AP_Airspeed_Params),

#if AIRSPEED_MAX_SENSORS > 1
    // @Group: 2_
    // @Path: AP_Airspeed_Params.cpp
    AP_SUBGROUPINFO(param[1], "2_", 29, AP_Airspeed, AP_Airspeed_Params),
#endif

    // index 30 is used by enable at the top of the table

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

    // Setup defaults that only apply to first sensor
    param[0].type.set_default(ARSPD_DEFAULT_TYPE);
#ifndef HAL_BUILD_AP_PERIPH
    param[0].bus.set_default(HAL_AIRSPEED_BUS_DEFAULT);
    param[0].pin.set_default(ARSPD_DEFAULT_PIN);
#endif

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Airspeed must be singleton");
    }
    _singleton = this;
}

void AP_Airspeed::set_fixedwing_parameters(const AP_FixedWing *_fixed_wing_parameters)
{
    fixed_wing_parameters = _fixed_wing_parameters;
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


// convet params to per instance param table
// PARAMETER_CONVERSION - Added: Dec-2022
void AP_Airspeed::convert_per_instance()
{
    AP_Param::ConversionInfo info;
#ifndef HAL_BUILD_AP_PERIPH
    // Vehicle conversion
    if (!AP_Param::find_key_by_pointer(this, info.old_key)) {
        return;
    }

    static const struct convert_table {
        uint32_t element[2];
        ap_var_type type;
        const char* name;
    }  conversion_table[] = {
        { {4042, 714}, AP_PARAM_INT8, "TYPE" },      // ARSPD_TYPE, ARSPD2_TYPE
        { {74, 778}, AP_PARAM_INT8, "USE" },        // ARSPD_USE, ARSPD2_USE
        { {138, 842}, AP_PARAM_FLOAT, "OFFSET" },    // ARSPD_OFFSET, ARSPD2_OFFSET
        { {202, 906}, AP_PARAM_FLOAT, "RATIO" },     // ARSPD_RATIO, ARSPD2_RATIO
        { {266, 970}, AP_PARAM_INT8, "PIN" },        // ARSPD_PIN, ARSPD2_PIN
#if AP_AIRSPEED_AUTOCAL_ENABLE
        { {330, 1034}, AP_PARAM_INT8, "AUTOCAL" },    // ARSPD_AUTOCAL, ARSPD2_AUTOCAL
#endif
        { {394, 1098}, AP_PARAM_INT8, "TUBE_ORDR" },  // ARSPD_TUBE_ORDER, ARSPD2_TUBE_ORDR
        { {458, 1162}, AP_PARAM_INT8, "SKIP_CAL" },   // ARSPD_SKIP_CAL, ARSPD2_SKIP_CAL
        { {522, 1226}, AP_PARAM_FLOAT, "PSI_RANGE" }, // ARSPD_PSI_RANGE, ARSPD2_PSI_RANGE
        { {586, 1290}, AP_PARAM_INT8, "BUS" },        // ARSPD_BUS, ARSPD2_BUS
        { {1546, 1610}, AP_PARAM_INT32, "DEVID" },    // ARSPD_DEVID, ARSPD2_DEVID
    };

#else
    // Periph conversion
    if (!AP_Param::find_top_level_key_by_pointer(this, info.old_key)) {
        return;
    }
    const struct convert_table {
        uint32_t element[2];
        ap_var_type type;
        const char* name;
    }  conversion_table[] = {
        { {0, 11}, AP_PARAM_INT8, "TYPE" },      // ARSPD_TYPE, ARSPD2_TYPE
#if AP_AIRSPEED_AUTOCAL_ENABLE
        { {5, 16}, AP_PARAM_INT8, "AUTOCAL" },    // ARSPD_AUTOCAL, ARSPD2_AUTOCAL
#endif
        { {8, 19}, AP_PARAM_FLOAT, "PSI_RANGE" }, // ARSPD_PSI_RANGE, ARSPD2_PSI_RANGE
        { {24, 25}, AP_PARAM_INT32, "DEVID" },    // ARSPD_DEVID, ARSPD2_DEVID
    };
#endif

    char param_name[17] {};
    info.new_name = param_name;

    for (const auto & elem : conversion_table) {
        info.type = elem.type;
        for (uint8_t i=0; i < MIN(AIRSPEED_MAX_SENSORS,2); i++) {
            info.old_group_element = elem.element[i];
            if (i == 0) {
                hal.util->snprintf(param_name, sizeof(param_name), "ARSPD_%s",  elem.name);
            } else {
                hal.util->snprintf(param_name, sizeof(param_name), "ARSPD%i_%s", i+1,  elem.name);
            }
            AP_Param::convert_old_parameter(&info, 1.0, 0);
        }
    }
}

void AP_Airspeed::init()
{

    convert_per_instance();

#if ENABLE_PARAMETER
    // if either type is set then enable if not manually set
    if (!_enable.configured() && ((param[0].type.get() != TYPE_NONE) || (param[1].type.get() != TYPE_NONE))) {
        _enable.set_and_save(1);
    }

    // Check if enabled
    if (!lib_enabled()) {
        return;
    }
#endif

    if (enabled(0)) {
        allocate();
    }
}

void AP_Airspeed::allocate()
{
    if (sensor[0] != nullptr) {
        // already initialised, periph may call allocate several times to allow CAN detection
        return;
    }

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
#if AP_AIRSPEED_MS4525_ENABLED
            sensor[i] = new AP_Airspeed_MS4525(*this, i);
#endif
            break;
        case TYPE_SITL:
#if AP_AIRSPEED_SITL_ENABLED
            sensor[i] = new AP_Airspeed_SITL(*this, i);
#endif
            break;
        case TYPE_ANALOG:
#if AP_AIRSPEED_ANALOG_ENABLED
            sensor[i] = new AP_Airspeed_Analog(*this, i);
#endif
            break;
        case TYPE_I2C_MS5525:
#if AP_AIRSPEED_MS5525_ENABLED
            sensor[i] = new AP_Airspeed_MS5525(*this, i, AP_Airspeed_MS5525::MS5525_ADDR_AUTO);
#endif
            break;
        case TYPE_I2C_MS5525_ADDRESS_1:
#if AP_AIRSPEED_MS5525_ENABLED
            sensor[i] = new AP_Airspeed_MS5525(*this, i, AP_Airspeed_MS5525::MS5525_ADDR_1);
#endif
            break;
        case TYPE_I2C_MS5525_ADDRESS_2:
#if AP_AIRSPEED_MS5525_ENABLED
            sensor[i] = new AP_Airspeed_MS5525(*this, i, AP_Airspeed_MS5525::MS5525_ADDR_2);
#endif
            break;
        case TYPE_I2C_SDP3X:
#if AP_AIRSPEED_SDP3X_ENABLED
            sensor[i] = new AP_Airspeed_SDP3X(*this, i);
#endif
            break;
        case TYPE_I2C_DLVR_5IN:
#if AP_AIRSPEED_DLVR_ENABLED
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 5);
#endif
            break;
        case TYPE_I2C_DLVR_10IN:
#if AP_AIRSPEED_DLVR_ENABLED
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 10);
#endif
            break;
        case TYPE_I2C_DLVR_20IN:
#if AP_AIRSPEED_DLVR_ENABLED
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 20);
#endif
            break;
        case TYPE_I2C_DLVR_30IN:
#if AP_AIRSPEED_DLVR_ENABLED
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 30);
#endif
            break;
        case TYPE_I2C_DLVR_60IN:
#if AP_AIRSPEED_DLVR_ENABLED
            sensor[i] = new AP_Airspeed_DLVR(*this, i, 60);
#endif  // AP_AIRSPEED_DLVR_ENABLED
            break;
        case TYPE_I2C_ASP5033:
#if AP_AIRSPEED_ASP5033_ENABLED
            sensor[i] = new AP_Airspeed_ASP5033(*this, i);
#endif
            break;
        case TYPE_UAVCAN:
#if AP_AIRSPEED_DRONECAN_ENABLED
            sensor[i] = AP_Airspeed_DroneCAN::probe(*this, i, uint32_t(param[i].bus_id.get()));
#endif
            break;
        case TYPE_NMEA_WATER:
#if AP_AIRSPEED_NMEA_ENABLED
#if APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_TYPE(APM_BUILD_ArduSub) 
            sensor[i] = new AP_Airspeed_NMEA(*this, i);
#endif
#endif
            break;
        case TYPE_MSP:
#if AP_AIRSPEED_MSP_ENABLED
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

#if AP_AIRSPEED_DRONECAN_ENABLED
    // we need a 2nd pass for DroneCAN sensors so we can match order by DEVID
    // the 2nd pass accepts any devid
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (sensor[i] == nullptr && (enum airspeed_type)param[i].type.get() == TYPE_UAVCAN) {
            sensor[i] = AP_Airspeed_DroneCAN::probe(*this, i, 0);
            if (sensor[i] != nullptr) {
                num_sensors = i+1;
            }
        }
    }
#endif // AP_AIRSPEED_DRONECAN_ENABLED
#endif // HAL_AIRSPEED_PROBE_LIST

    // set DEVID to zero for any sensors not found. This allows backends to order
    // based on previous value of DEVID. This allows for swapping out sensors
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (sensor[i] == nullptr) {
            // note we use set() not set_and_save() to allow a sensor to be temporarily
            // removed for one boot without losing its slot
            param[i].bus_id.set(0);
        }
    }
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
#ifndef HAL_BUILD_AP_PERIPH
    if (!lib_enabled()) {
        return;
    }
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
        calibration_state[i] = CalibrationState::IN_PROGRESS;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Airspeed %u calibration started", i+1);
    }
#endif // HAL_BUILD_AP_PERIPH
}

/*
  update async airspeed zero offset calibration
*/
void AP_Airspeed::update_calibration(uint8_t i, float raw_pressure)
{
#ifndef HAL_BUILD_AP_PERIPH
    if (!enabled(i) || state[i].cal.start_ms == 0) {
        return;
    }
    
    // consider calibration complete when we have at least 15 samples
    // over at least 1 second
    if (AP_HAL::millis() - state[i].cal.start_ms >= 1000 &&
        state[i].cal.read_count > 15) {
        if (state[i].cal.count == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Airspeed %u unhealthy", i + 1);
            calibration_state[i] = CalibrationState::FAILED;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Airspeed %u calibrated", i + 1);
            float calibrated_offset = state[i].cal.sum / state[i].cal.count;
            // check if new offset differs too greatly from last calibration, indicating pitot uncovered in wind
            if (fixed_wing_parameters != nullptr) {
                float airspeed_min = fixed_wing_parameters->airspeed_min.get();
                // use percentage of ARSPD_FBW_MIN as criteria for max allowed change in offset
                float max_change = 0.5*(sq((1 + (max_speed_pcnt * 0.01))*airspeed_min) - sq(airspeed_min));
                if (max_speed_pcnt > 0 && (abs(calibrated_offset-param[i].offset) > max_change) && (abs(param[i].offset) > 0)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Arspd %d offset change large;cover and recal", i +1);
                }
            }
            param[i].offset.set_and_save(calibrated_offset);
            calibration_state[i] = CalibrationState::SUCCESS;
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
#endif // HAL_BUILD_AP_PERIPH
}

// get aggregate calibration state for the Airspeed library:
AP_Airspeed::CalibrationState AP_Airspeed::get_calibration_state() const
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        switch (calibration_state[i]) {
        case CalibrationState::SUCCESS:
        case CalibrationState::NOT_STARTED:
            continue;
        case CalibrationState::IN_PROGRESS:
            return CalibrationState::IN_PROGRESS;
        case CalibrationState::FAILED:
            return CalibrationState::FAILED;
        }
    }
    return CalibrationState::SUCCESS;
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

    float raw_pressure = get_pressure(i);
    float airspeed_pressure = raw_pressure - get_offset(i);

    // remember raw pressure for logging
    state[i].corrected_pressure = airspeed_pressure;

#ifndef HAL_BUILD_AP_PERIPH
    bool prev_healthy = state[i].healthy;
    if (state[i].cal.start_ms != 0) {
        update_calibration(i, raw_pressure);
    }

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
#endif // HAL_BUILD_AP_PERIPH
}

// read all airspeed sensors
void AP_Airspeed::update()
{
    if (!lib_enabled()) {
        return;
    }

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        read(i);
    }

#if HAL_GCS_ENABLED
    // debugging until we get MAVLink support for 2nd airspeed sensor
    if (enabled(1)) {
        gcs().send_named_float("AS2", get_airspeed(1));
    }
#endif

#if HAL_LOGGING_ENABLED
    const uint8_t old_primary = primary;
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
    if (primary != old_primary) {
        AP::logger().Write_Event(LogEvent::AIRSPEED_PRIMARY_CHANGED);
    }
    if (_log_bit != (uint32_t)-1 && AP::logger().should_log(_log_bit)) {
        Log_Airspeed();
    }
#endif
}

#if AP_AIRSPEED_MSP_ENABLED
/*
  handle MSP airspeed data
 */
void AP_Airspeed::handle_msp(const MSP::msp_airspeed_data_message_t &pkt)
{
    if (!lib_enabled()) {
        return;
    }

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

// @LoggerMessage: HYGR
// @Description: Hygrometer data
// @Field: TimeUS: Time since system startup
// @Field: Id: sensor ID
// @Field: Humidity: percentage humidity
// @Field: Temp: temperature in degrees C

void AP_Airspeed::Log_Airspeed()
{
    const uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (!enabled(i) || sensor[i] == nullptr) {
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
            test_ratio    : get_test_ratio(i),
            primary       : get_primary()
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));

#if AP_AIRSPEED_HYGROMETER_ENABLE
        struct {
            uint32_t sample_ms;
            float temperature;
            float humidity;
        } hygrometer;
        if (sensor[i]->get_hygrometer(hygrometer.sample_ms, hygrometer.temperature, hygrometer.humidity) &&
            hygrometer.sample_ms != state[i].last_hygrometer_log_ms) {
            AP::logger().WriteStreaming("HYGR",
                                        "TimeUS,Id,Humidity,Temp",
                                        "s#%O",
                                        "F---",
                                        "QBff",
                                        AP_HAL::micros64(),
                                        i,
                                        hygrometer.humidity,
                                        hygrometer.temperature);
            state[i].last_hygrometer_log_ms = hygrometer.sample_ms;
        }
#endif
    }
}

bool AP_Airspeed::use(uint8_t i) const
{
#ifndef HAL_BUILD_AP_PERIPH
    if (!lib_enabled()) {
        return false;
    }
    if (_force_disable_use) {
        return false;
    }
    if (!enabled(i) || !param[i].use) {
        return false;
    }
    if (param[i].use == 2 && !is_zero(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))) {
        // special case for gliders with airspeed sensors behind the
        // propeller. Allow airspeed to be disabled when throttle is
        // running
        return false;
    }
    return true;
#else
    return false;
#endif // HAL_BUILD_AP_PERIPH
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

bool AP_Airspeed::lib_enabled() const {
#if ENABLE_PARAMETER
    return _enable > 0;
#endif
    return true;
}

// return true if airspeed is enabled
bool AP_Airspeed::enabled(uint8_t i) const {
    if (!lib_enabled()) {
        return false;
    }
    if (i < AIRSPEED_MAX_SENSORS) {
        return param[i].type.get() != TYPE_NONE;
    }
    return false;
}

// return health status of sensor
bool AP_Airspeed::healthy(uint8_t i) const {
    bool ok = state[i].healthy && enabled(i) && sensor[i] != nullptr;
#ifndef HAL_BUILD_AP_PERIPH
    // sanity check the offset parameter.  Zero is permitted if we are skipping calibration.
    ok &= (fabsf(param[i].offset) > 0 || state[i].use_zero_offset || param[i].skip_cal);
#endif
    return ok;
}

// return the current airspeed in m/s
float AP_Airspeed::get_airspeed(uint8_t i) const {
    if (!enabled(i)) {
        // we can't have negative airspeed so sending an obviously invalid value
        return -1.0;
    }
    return state[i].airspeed;
}

// return the unfiltered airspeed in m/s
float AP_Airspeed::get_raw_airspeed(uint8_t i) const {
    if (!enabled(i)) {
        // we can't have negative airspeed so sending an obviously invalid value
        return -1.0;
    }
    return state[i].raw_airspeed;
}

// return the differential pressure in Pascal for the last airspeed reading
float AP_Airspeed::get_differential_pressure(uint8_t i) const {
    if (!enabled(i)) {
        return 0.0;
    }
    return state[i].last_pressure;
}

// return the current corrected pressure
float AP_Airspeed::get_corrected_pressure(uint8_t i) const {
    if (!enabled(i)) {
        return 0.0;
    }
    return state[i].corrected_pressure;
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
bool AP_Airspeed::get_hygrometer(uint8_t i, uint32_t &last_sample_ms, float &temperature, float &humidity) const
{
    if (!enabled(i) || sensor[i] == nullptr) {
        return false;
    }
    return sensor[i]->get_hygrometer(last_sample_ms, temperature, humidity);
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE

#else  // build type is not appropriate; provide a dummy implementation:
const AP_Param::GroupInfo AP_Airspeed::var_info[] = { AP_GROUPEND };

void AP_Airspeed::update() {};
bool AP_Airspeed::get_temperature(uint8_t i, float &temperature) { return false; }
void AP_Airspeed::calibrate(bool in_startup) {}
AP_Airspeed::CalibrationState AP_Airspeed::get_calibration_state() const { return CalibrationState::NOT_STARTED; }
bool AP_Airspeed::use(uint8_t i) const { return false; }
bool AP_Airspeed::enabled(uint8_t i) const { return false; }
bool AP_Airspeed::healthy(uint8_t i) const { return false; }
float AP_Airspeed::get_airspeed(uint8_t i) const { return 0.0; }
float AP_Airspeed::get_differential_pressure(uint8_t i) const { return 0.0; }

#if AP_AIRSPEED_MSP_ENABLED
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
