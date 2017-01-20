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

#include <AP_ADC/AP_ADC.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <utility>
#include "AP_Airspeed.h"
#include "AP_Airspeed_MS4525.h"
#include "AP_Airspeed_MS5525.h"
#include "AP_Airspeed_analog.h"

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

    // @Param: TYPE
    // @DisplayName: Airspeed type
    // @Description: Type of airspeed sensor
    // @Values: 0:None,1:I2C-MS4525D0,2:Analog,3:I2C-MS5525
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 0, AP_Airspeed, _type, ARSPD_DEFAULT_TYPE, AP_PARAM_FLAG_ENABLE),

    // @Param: USE
    // @DisplayName: Airspeed use
    // @Description: use airspeed for flight control
    // @Values: 1:Use,0:Don't Use
    // @User: Standard
    AP_GROUPINFO("USE",    1, AP_Airspeed, _use, 0),

    // @Param: OFFSET
    // @DisplayName: Airspeed offset
    // @Description: Airspeed calibration offset
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("OFFSET", 2, AP_Airspeed, _offset, 0),

    // @Param: RATIO
    // @DisplayName: Airspeed ratio
    // @Description: Airspeed calibration ratio
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("RATIO",  3, AP_Airspeed, _ratio, 1.9936f),

    // @Param: PIN
    // @DisplayName: Airspeed pin
    // @Description: The pin number that the airspeed sensor is connected to for analog sensors. Set to 15 on the Pixhawk for the analog airspeed port. 
    // @User: Advanced
    AP_GROUPINFO("PIN",  4, AP_Airspeed, _pin, ARSPD_DEFAULT_PIN),

    // @Param: AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration
    // @Description: If this is enabled then the APM will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%. This option should be enabled for a calibration flight then disabled again when calibration is complete. Leaving it enabled all the time is not recommended.
    // @User: Advanced
    AP_GROUPINFO("AUTOCAL",  5, AP_Airspeed, _autocal, 0),

    // @Param: TUBE_ORDER
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the top connector on the sensor needs to be the dynamic pressure. If set to 1 then the bottom connector needs to be the dynamic pressure. If set to 2 (the default) then the airspeed driver will accept either order. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft it receiving excessive pressure on the static port, which would otherwise be seen as a positive airspeed.
    // @User: Advanced
    AP_GROUPINFO("TUBE_ORDER",  6, AP_Airspeed, _tube_order, 2),

    // @Param: SKIP_CAL
    // @DisplayName: Skip airspeed calibration on startup
    // @Description: This parameter allows you to skip airspeed offset calibration on startup, instead using the offset from the last calibration. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("SKIP_CAL",  7, AP_Airspeed, _skip_cal, 0),

    // @Param: PSI_RANGE
    // @DisplayName: The PSI range of the device
    // @Description: This parameter allows you to to set the PSI (pounds per square inch) range for your sensor. You should not change this unless you examine the datasheet for your device
    // @User: Advanced
    AP_GROUPINFO("PSI_RANGE",  8, AP_Airspeed, _psi_range, PSI_RANGE_DEFAULT),

    // @Param: BUS
    // @DisplayName: Airspeed I2C bus
    // @Description: The bus number of the I2C bus to look for the sensor on
    // @Values: 0:Bus0,1:Bus1
    // @User: Advanced
    AP_GROUPINFO("BUS",  9, AP_Airspeed, _bus, 1),
    
    AP_GROUPEND
};


AP_Airspeed::AP_Airspeed()
    : _EAS2TAS(1.0f)
    , _calibration()
{
    AP_Param::setup_object_defaults(this, var_info);
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
    if (_pin.load() && _pin.get() != 65) {
        _type.set_default(TYPE_ANALOG);
    }

    _last_pressure = 0;
    _calibration.init(_ratio);
    _last_saved_ratio = _ratio;
    _counter = 0;

    switch ((enum airspeed_type)_type.get()) {
    case TYPE_NONE:
        // nothing to do
        break;
    case TYPE_I2C_MS4525:
        sensor = new AP_Airspeed_MS4525(*this);
        break;
    case TYPE_ANALOG:
        sensor = new AP_Airspeed_Analog(*this);
        break;
    case TYPE_I2C_MS5525:
        sensor = new AP_Airspeed_MS5525(*this);
        break;
    }
    if (sensor && !sensor->init()) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Airspeed init failed");
        delete sensor;
        sensor = nullptr;
    }
}

// read the airspeed sensor
float AP_Airspeed::get_pressure(void)
{
    if (!enabled()) {
        return 0;
    }
    if (_hil_set) {
        _healthy = true;
        return _hil_pressure;
    }
    float pressure = 0;
    if (sensor) {
        _healthy = sensor->get_differential_pressure(pressure);
    }
    return pressure;
}

// get a temperature reading if possible
bool AP_Airspeed::get_temperature(float &temperature)
{
    if (!enabled()) {
        return false;
    }
    if (sensor) {
        return sensor->get_temperature(temperature);
    }
    return false;
}

// calibrate the airspeed. This must be called at least once before
// the get_airspeed() interface can be used
void AP_Airspeed::calibrate(bool in_startup)
{
    if (!enabled()) {
        return;
    }
    if (in_startup && _skip_cal) {
        return;
    }
    _cal.start_ms = AP_HAL::millis();
    _cal.count = 0;
    _cal.sum = 0;
    _cal.read_count = 0;
}

/*
  update async airspeed calibration
*/
void AP_Airspeed::update_calibration(float raw_pressure)
{
    // consider calibration complete when we have at least 15 samples
    // over at least 1 second
    if (AP_HAL::millis() - _cal.start_ms >= 1000 &&
        _cal.read_count > 15) {
        if (_cal.count == 0) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Airspeed sensor unhealthy");
        } else {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Airspeed sensor calibrated");
            _offset.set_and_save(_cal.sum / _cal.count);
        }
        _cal.start_ms = 0;
        return;
    }
    // we discard the first 5 samples
    if (_healthy && _cal.read_count > 5) {
        _cal.sum += raw_pressure;
        _cal.count++;
    }
    _cal.read_count++;
}

// read the airspeed sensor
void AP_Airspeed::read(void)
{
    float airspeed_pressure;
    if (!enabled()) {
        return;
    }
    float raw_pressure = get_pressure();
    if (_cal.start_ms != 0) {
        update_calibration(raw_pressure);
    }
    
    airspeed_pressure = raw_pressure - _offset;

    // remember raw pressure for logging
    _corrected_pressure = airspeed_pressure;

    /*
      we support different pitot tube setups so used can choose if
      they want to be able to detect pressure on the static port
     */
    switch ((enum pitot_tube_order)_tube_order.get()) {
    case PITOT_TUBE_ORDER_NEGATIVE:
        airspeed_pressure = -airspeed_pressure;
        // no break
    case PITOT_TUBE_ORDER_POSITIVE:
        if (airspeed_pressure < -32) {
            // we're reading more than about -8m/s. The user probably has
            // the ports the wrong way around
            _healthy = false;
        }
        break;
    case PITOT_TUBE_ORDER_AUTO:
    default:
        airspeed_pressure = fabsf(airspeed_pressure);
        break;
    }
    airspeed_pressure       = MAX(airspeed_pressure, 0);
    _last_pressure          = airspeed_pressure;
    _raw_airspeed           = sqrtf(airspeed_pressure * _ratio);
    _airspeed               = 0.7f * _airspeed  +  0.3f * _raw_airspeed;
    _last_update_ms         = AP_HAL::millis();
}

void AP_Airspeed::setHIL(float airspeed, float diff_pressure, float temperature)
{
    _raw_airspeed = airspeed;
    _airspeed = airspeed;
    _last_pressure = diff_pressure;
    _last_update_ms = AP_HAL::millis();
    _hil_pressure = diff_pressure;
    _hil_set = true;
    _healthy = true;
}
