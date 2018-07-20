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
 *   APM_Airspeed.cpp - airspeed (pitot) driver
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <utility>
#include "AP_Airspeed.h"
#include "AP_Airspeed_MS4525.h"
#include "AP_Airspeed_MS5525.h"
#include "AP_Airspeed_SDP3X.h"
#include "AP_Airspeed_analog.h"
#include "AP_Airspeed_Backend.h"

extern const AP_HAL::HAL &hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #define ARSPD_DEFAULT_TYPE TYPE_ANALOG
 #define ARSPD_DEFAULT_PIN 1
#else
 #define ARSPD_DEFAULT_TYPE TYPE_I2C_MS4525
 #define ARSPD_DEFAULT_PIN 15
#endif

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#define PSI_RANGE_DEFAULT 0.05
#endif

#ifndef PSI_RANGE_DEFAULT
#define PSI_RANGE_DEFAULT 1.0f
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Airspeed::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Airspeed type
    // @Description: Type of airspeed sensor
    // @Values: 0:None,1:I2C-MS4525D0,2:Analog,3:I2C-MS5525,4:I2C-MS5525 (0x76),5:I2C-MS5525 (0x77),6:I2C-SDP3X
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_Airspeed, param[0].type, ARSPD_DEFAULT_TYPE, AP_PARAM_FLAG_ENABLE),

    // @Param: _USE
    // @DisplayName: Airspeed use
    // @Description: use airspeed for flight control. When set to 0 airspeed sensor can be logged and displayed on a GCS but won't be used for flight. When set to 1 it will be logged and used. When set to 2 it will be only used when the throttle is zero, which can be useful in gliders with airspeed sensors behind a propeller
    // @Values: 0:Don't Use,1:use,2:UseWhenZeroThrottle
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
    // @Description: Airspeed calibration ratio
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_RATIO",  3, AP_Airspeed, param[0].ratio, 1.9936f),

    // @Param: _PIN
    // @DisplayName: Airspeed pin
    // @Description: The pin number that the airspeed sensor is connected to for analog sensors. Set to 15 on the Pixhawk for the analog airspeed port. 
    // @User: Advanced
    AP_GROUPINFO("_PIN",  4, AP_Airspeed, param[0].pin, ARSPD_DEFAULT_PIN),

    // @Param: _AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration
    // @Description: If this is enabled then the APM will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%. This option should be enabled for a calibration flight then disabled again when calibration is complete. Leaving it enabled all the time is not recommended.
    // @User: Advanced
    AP_GROUPINFO("_AUTOCAL",  5, AP_Airspeed, param[0].autocal, 0),

    // @Param: _TUBE_ORDER
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the top connector on the sensor needs to be the dynamic pressure. If set to 1 then the bottom connector needs to be the dynamic pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft it receiving excessive pressure on the static port, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
    AP_GROUPINFO("_TUBE_ORDER",  6, AP_Airspeed, param[0].tube_order, 2),

    // @Param: _SKIP_CAL
    // @DisplayName: Skip airspeed calibration on startup
    // @Description: This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("_SKIP_CAL",  7, AP_Airspeed, param[0].skip_cal, 0),

    // @Param: _PSI_RANGE
    // @DisplayName: The PSI range of the device
    // @Description: This parameter allows you to to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device
    // @User: Advanced
    AP_GROUPINFO("_PSI_RANGE",  8, AP_Airspeed, param[0].psi_range, PSI_RANGE_DEFAULT),

    // @Param: _BUS
    // @DisplayName: Airspeed I2C bus
    // @Description: The bus number of the I2C bus to look for the sensor on
    // @Values: 0:Bus0(internal),1:Bus1(external),2:Bus2(auxillary)
    // @User: Advanced
    AP_GROUPINFO("_BUS",  9, AP_Airspeed, param[0].bus, 1),

    // @Param: _PRIMARY
    // @DisplayName: Primary airspeed sensor
    // @Description: This selects which airspeed sensor will be the primary if multiple sensors are found
    // @Values: 0:FirstSensor,1:2ndSensor
    // @User: Advanced
    AP_GROUPINFO("_PRIMARY", 10, AP_Airspeed, primary_sensor, 0),

    // @Param: 2_TYPE
    // @DisplayName: Second Airspeed type
    // @Description: Type of 2nd airspeed sensor
    // @Values: 0:None,1:I2C-MS4525D0,2:Analog,3:I2C-MS5525,4:I2C-MS5525 (0x76),5:I2C-MS5525 (0x77),6:I2C-SDP3X
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
    // @Description: The pin number that the airspeed sensor is connected to for analog sensors. Set to 15 on the Pixhawk for the analog airspeed port. 
    // @User: Advanced
    AP_GROUPINFO("2_PIN",  15, AP_Airspeed, param[1].pin, 0),

    // @Param: 2_AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration for 2nd airspeed sensor
    // @Description: If this is enabled then the APM will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%. This option should be enabled for a calibration flight then disabled again when calibration is complete. Leaving it enabled all the time is not recommended.
    // @User: Advanced
    AP_GROUPINFO("2_AUTOCAL",  16, AP_Airspeed, param[1].autocal, 0),

    // @Param: 2_TUBE_ORDR
    // @DisplayName: Control pitot tube order of 2nd airspeed sensor
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the top connector on the sensor needs to be the dynamic pressure. If set to 1 then the bottom connector needs to be the dynamic pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft it receiving excessive pressure on the static port, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
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
    
    AP_GROUPEND
};


AP_Airspeed::AP_Airspeed()
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        state[i].EAS2TAS = 1;
    }
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Airspeed must be singleton");
    }
    _singleton = this;
}


/*
  this scaling factor converts from the old system where we used a
  0 to 4095 raw ADC value for 0-5V to the new system which gets the
  voltage in volts directly from the ADC driver
 */
#define SCALING_OLD_CALIBRATION 819 // 4095/5

void AP_Airspeed::init()
{
    // cope with upgrade from old system
    if (param[0].pin.load() && param[0].pin.get() != 65) {
        param[0].type.set_default(TYPE_ANALOG);
    }

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        state[i].calibration.init(param[i].ratio);
        state[i].last_saved_ratio = param[i].ratio;

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
        }
        if (sensor[i] && !sensor[i]->init()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Airspeed[%u] init failed", i);
            delete sensor[i];
            sensor[i] = nullptr;
        }
    }
}

// read the airspeed sensor
float AP_Airspeed::get_pressure(uint8_t i)
{
    if (!enabled(i)) {
        return 0;
    }
    if (state[i].hil_set) {
        state[i].healthy = true;
        return state[i].hil_pressure;
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
        state[i].cal.start_ms = AP_HAL::millis();
        state[i].cal.count = 0;
        state[i].cal.sum = 0;
        state[i].cal.read_count = 0;
    }
    gcs().send_text(MAV_SEVERITY_INFO,"Airspeed calibration started");
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
            gcs().send_text(MAV_SEVERITY_INFO, "Airspeed[%u] sensor unhealthy", i);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Airspeed[%u] sensor calibrated", i);
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
    float airspeed_pressure;
    if (!enabled(i) || !sensor[i]) {
        return;
    }
    bool prev_healthy = state[i].healthy;
    float raw_pressure = get_pressure(i);
    if (state[i].cal.start_ms != 0) {
        update_calibration(i, raw_pressure);
    }
    
    airspeed_pressure = raw_pressure - param[i].offset;

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
        if (airspeed_pressure < -32) {
            // we're reading more than about -8m/s. The user probably has
            // the ports the wrong way around
            state[i].healthy = false;
        }
        break;
    case PITOT_TUBE_ORDER_AUTO:
    default:
        state[i].last_pressure  = fabsf(airspeed_pressure);
        state[i].raw_airspeed   = sqrtf(fabsf(airspeed_pressure) * param[i].ratio);
        state[i].airspeed       = sqrtf(fabsf(state[i].filtered_pressure) * param[i].ratio);
        break;
    }

    state[i].last_update_ms = AP_HAL::millis();
}

// read all airspeed sensors
void AP_Airspeed::read(void)
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        read(i);
    }

#if 1
    // debugging until we get MAVLink support for 2nd airspeed sensor
    if (enabled(1)) {
        gcs().send_named_float("AS2", get_airspeed(1));
    }
#endif

    // setup primary
    if (healthy(primary_sensor.get())) {
        primary = primary_sensor.get();
        return;
    }
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (healthy(i)) {
            primary = i;
            break;
        }
    }
}

void AP_Airspeed::setHIL(float airspeed, float diff_pressure, float temperature)
{
    state[0].raw_airspeed = airspeed;
    state[0].airspeed = airspeed;
    state[0].last_pressure = diff_pressure;
    state[0].last_update_ms = AP_HAL::millis();
    state[0].hil_pressure = diff_pressure;
    state[0].hil_set = true;
    state[0].healthy = true;
}

bool AP_Airspeed::use(uint8_t i) const
{
    if (!enabled(i) || !param[i].use) {
        return false;
    }
    if (param[i].use == 2 && SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) != 0) {
        // special case for gliders with airspeed sensors behind the
        // propeller. Allow airspeed to be disabled when throttle is
        // running
        return false;
    }
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

// singleton instance
AP_Airspeed *AP_Airspeed::_singleton;

