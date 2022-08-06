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

#include "AP_WindVane.h"

#include "AP_WindVane_Home.h"
#include "AP_WindVane_Analog.h"
#include "AP_WindVane_ModernDevice.h"
#include "AP_WindVane_Airspeed.h"
#include "AP_WindVane_RPM.h"
#include "AP_WindVane_SITL.h"
#include "AP_WindVane_NMEA.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_WindVane::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Wind Vane Type
    // @Description: Wind Vane type
    // @Values: 0:None,1:Heading when armed,2:RC input offset heading when armed,3:Analog,4:NMEA,10:SITL true,11:SITL apparent
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_WindVane, _direction_type, 0, AP_PARAM_FLAG_ENABLE),

    // 2 was RC_IN_NO

    // @Param: DIR_PIN
    // @DisplayName: Wind vane analog voltage pin for direction
    // @Description: Analog input pin to read as wind vane direction
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6/Pixhawk2 ADC,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("DIR_PIN", 3, AP_WindVane, _dir_analog_pin, WINDVANE_DEFAULT_PIN),

    // @Param: DIR_V_MIN
    // @DisplayName: Wind vane voltage minimum
    // @Description: Minimum voltage supplied by analog wind vane. When using pin 103, the maximum value of the parameter is 3.3V.
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("DIR_V_MIN", 4, AP_WindVane, _dir_analog_volt_min, 0.0f),

    // @Param: DIR_V_MAX
    // @DisplayName: Wind vane voltage maximum
    // @Description: Maximum voltage supplied by analog wind vane. When using pin 103, the maximum value of the parameter is 3.3V.
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("DIR_V_MAX", 5, AP_WindVane, _dir_analog_volt_max, 3.3f),

    // @Param: DIR_OFS
    // @DisplayName: Wind vane headwind offset
    // @Description: Angle offset when analog windvane is indicating a headwind, ie 0 degress relative to vehicle
    // @Units: deg
    // @Increment: 1
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("DIR_OFS", 6, AP_WindVane, _dir_analog_bearing_offset, 0.0f),

    // @Param: DIR_FILT
    // @DisplayName: apparent Wind vane direction low pass filter frequency
    // @Description: apparent Wind vane direction low pass filter frequency, a value of -1 disables filter
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("DIR_FILT", 7, AP_WindVane, _dir_filt_hz, 0.5f),

    // @Param: CAL
    // @DisplayName: Wind vane calibration start
    // @Description: Start wind vane calibration by setting this to 1 or 2
    // @Values: 0:None, 1:Calibrate direction, 2:Calibrate speed
    // @User: Standard
    AP_GROUPINFO("CAL", 8, AP_WindVane, _calibration, 0),

    // @Param: DIR_DZ
    // @DisplayName: Wind vane deadzone when using analog sensor
    // @Description: Wind vane deadzone when using analog sensor
    // @Units: deg
    // @Increment: 1
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("DIR_DZ", 9, AP_WindVane, _dir_analog_deadzone, 0),

    // @Param: SPEED_MIN
    // @DisplayName: Wind vane cut off wind speed
    // @Description: Wind vane direction will be ignored when apparent wind speeds are below this value (if wind speed sensor is present).  If the apparent wind is consistently below this value the vane will not work
    // @Units: m/s
    // @Increment: 0.1
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN", 10, AP_WindVane, _dir_speed_cutoff, 0),

    // @Param: SPEED_TYPE
    // @DisplayName: Wind speed sensor Type
    // @Description: Wind speed sensor type
    // @Values: 0:None,1:Airspeed library,2:Modern Devices Wind Sensor,3:RPM library,4:NMEA,10:SITL true,11:SITL apparent
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SPEED_TYPE", 11, AP_WindVane, _speed_sensor_type,  0),

    // @Param: SPEED_PIN
    // @DisplayName: Wind vane speed sensor analog pin
    // @Description: Wind speed analog speed input pin for Modern Devices Wind Sensor rev. p
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6/Pixhawk2 ADC,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("SPEED_PIN", 12, AP_WindVane, _speed_sensor_speed_pin,  WINDSPEED_DEFAULT_SPEED_PIN),

    // @Param: TEMP_PIN
    // @DisplayName: Wind vane speed sensor analog temp pin
    // @Description: Wind speed sensor analog temp input pin for Modern Devices Wind Sensor rev. p, set to -1 to diasble temp readings
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6/Pixhawk2 ADC,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("TEMP_PIN", 13, AP_WindVane, _speed_sensor_temp_pin,  WINDSPEED_DEFAULT_TEMP_PIN),

    // @Param: SPEED_OFS
    // @DisplayName: Wind speed sensor analog voltage offset
    // @Description: Wind sensor analog voltage offset at zero wind speed
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 3.3
    // @User: Standard
    AP_GROUPINFO("SPEED_OFS", 14, AP_WindVane, _speed_sensor_voltage_offset, WINDSPEED_DEFAULT_VOLT_OFFSET),

    // @Param: SPEED_FILT
    // @DisplayName: apparent wind speed low pass filter frequency
    // @Description: apparent Wind speed low pass filter frequency, a value of -1 disables filter
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("SPEED_FILT", 15, AP_WindVane, _speed_filt_hz, 0.5f),

    // @Param: TRUE_FILT
    // @DisplayName: True speed and direction low pass filter frequency
    // @Description: True speed and direction low pass filter frequency, a value of -1 disables filter
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("TRUE_FILT", 16, AP_WindVane, _true_filt_hz, 0.05f),

    AP_GROUPEND
};

// constructor
AP_WindVane::AP_WindVane()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton) {
        AP_HAL::panic("Too many Wind Vane sensors");
    }
#endif
    _singleton = this;
}

/*
 * Get the AP_WindVane singleton
 */
AP_WindVane *AP_WindVane::get_singleton()
{
    return _singleton;
}

// return true if wind vane is enabled
bool AP_WindVane::enabled() const
{
    return _direction_type != WINDVANE_NONE;
}

// return true if wind speed is enabled
bool AP_WindVane::wind_speed_enabled() const
{
    return (_speed_sensor_type != WINDSPEED_NONE);
}

// Initialize the Wind Vane object and prepare it for use
void AP_WindVane::init(const AP_SerialManager& serial_manager)
{
    // don't construct twice
    if (_direction_driver != nullptr || _speed_driver != nullptr ) {
        return;
    }

    // wind direction
    switch (_direction_type) {
        case WindVaneType::WINDVANE_NONE:
            // WindVane disabled
            return;
        case WindVaneType::WINDVANE_HOME_HEADING:
        case WindVaneType::WINDVANE_PWM_PIN:
            _direction_driver = new AP_WindVane_Home(*this);
            break;
        case WindVaneType::WINDVANE_ANALOG_PIN:
            _direction_driver = new AP_WindVane_Analog(*this);
            break;
        case WindVaneType::WINDVANE_SITL_TRUE:
        case WindVaneType::WINDVANE_SITL_APPARENT:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            _direction_driver = new AP_WindVane_SITL(*this);
#endif
            break;
        case WindVaneType::WINDVANE_NMEA:
            _direction_driver = new AP_WindVane_NMEA(*this);
            _direction_driver->init(serial_manager);
            break;
    }

    // wind speed
    switch (_speed_sensor_type) {
        case Speed_type::WINDSPEED_NONE:
            break;
        case Speed_type::WINDSPEED_AIRSPEED:
            _speed_driver = new AP_WindVane_Airspeed(*this);
            break;
        case Speed_type::WINDVANE_WIND_SENSOR_REV_P:
            _speed_driver = new AP_WindVane_ModernDevice(*this);
            break;
        case Speed_type::WINDSPEED_SITL_TRUE:
        case Speed_type::WINDSPEED_SITL_APPARENT:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            // single driver does both speed and direction
            if (_direction_type != _speed_sensor_type) {
                _speed_driver = new AP_WindVane_SITL(*this);
            } else {
                _speed_driver = _direction_driver;
            }
#endif
            break;
        case Speed_type::WINDSPEED_NMEA:
            // single driver does both speed and direction
            if (_direction_type != WindVaneType::WINDVANE_NMEA) {
                _speed_driver = new AP_WindVane_NMEA(*this);
                _speed_driver->init(serial_manager);
            } else {
                _speed_driver = _direction_driver;
            }
            break;
        case Speed_type::WINDSPEED_RPM:
            _speed_driver = new AP_WindVane_RPM(*this);
            break;
    }
}

// update wind vane, expected to be called at 20hz
void AP_WindVane::update()
{
    const bool have_speed = _speed_driver != nullptr;
    const bool have_direction = _direction_driver != nullptr;

    // exit immediately if not enabled
    if (!enabled() || (!have_speed && !have_direction)) {
        return;
    }

    // calibrate if booted and disarmed
    if (!hal.util->get_soft_armed()) {
        if (_calibration == 1 && have_direction) {
            _direction_driver->calibrate();
        } else if (_calibration == 2 && have_speed) {
            _speed_driver->calibrate();
        } else if (_calibration != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "WindVane: driver not found");
            _calibration.set_and_save(0);
        }
    } else if (_calibration != 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "WindVane: disarm for cal");
        _calibration.set_and_save(0);
    }

    // read apparent wind speed
    if (have_speed) {
        _speed_driver->update_speed();
    }

    // read apparent wind direction
    if (have_direction) {
        _direction_driver->update_direction();
    }

    if (have_speed && have_direction) {
        if (_speed_apparent >= _dir_speed_cutoff) {
            // calculate true wind speed and direction from apparent wind
            update_true_wind_speed_and_direction();
        } else {
            // assume true wind has not changed, calculate the apparent wind direction and speed
            update_apparent_wind_dir_from_true();
        }
    } else {
        // only have direction, can't do true wind calcs, set true direction to apparent + heading
        _direction_true_raw = wrap_PI(_direction_apparent_raw + AP::ahrs().yaw);
        _speed_true_raw = 0.0f;
    }

    /*
        Apply filters
        https://en.wikipedia.org/wiki/Mean_of_circular_quantities
    */
    float filtered_sin;
    float filtered_cos;

    // apparent speed
    if (is_positive(_speed_filt_hz)) {
        _speed_apparent_filt.set_cutoff_frequency(_speed_filt_hz);
        _speed_apparent = _speed_apparent_filt.apply(_speed_apparent_raw,0.05f);
    } else {
        _speed_apparent = _speed_apparent_raw;
    }

    // apparent direction
    if (is_positive(_dir_filt_hz)) {
        _direction_apparent_sin_filt.set_cutoff_frequency(_dir_filt_hz);
        _direction_apparent_cos_filt.set_cutoff_frequency(_dir_filt_hz);
        filtered_sin = _direction_apparent_sin_filt.apply(sinf(_direction_apparent_raw),0.05f);
        filtered_cos = _direction_apparent_cos_filt.apply(cosf(_direction_apparent_raw),0.05f);
        _direction_apparent = atan2f(filtered_sin, filtered_cos);
    } else {
        _direction_apparent = _direction_apparent_raw;
    }

    // apparent direction for tack threshold, hard coded freq
    filtered_sin = _tack_sin_filt.apply(sinf(_direction_apparent_raw),0.05f);
    filtered_cos = _tack_cos_filt.apply(cosf(_direction_apparent_raw),0.05f);
    _direction_tack = atan2f(filtered_sin, filtered_cos);
    _current_tack = is_negative(_direction_tack) ? Sailboat_Tack::TACK_PORT : Sailboat_Tack::TACK_STARBOARD;

    // true wind direction and speed, both at the same freq
    if (is_positive(_true_filt_hz)) {
        _speed_true_filt.set_cutoff_frequency(_true_filt_hz);
        _direction_true_sin_filt.set_cutoff_frequency(_true_filt_hz);
        _direction_true_cos_filt.set_cutoff_frequency(_true_filt_hz);

        _speed_true = _speed_true_filt.apply(_speed_true_raw,0.05f);
        filtered_sin = _direction_true_sin_filt.apply(sinf(_direction_true_raw),0.05f);
        filtered_cos = _direction_true_cos_filt.apply(cosf(_direction_true_raw),0.05f);
        _direction_true = atan2f(filtered_sin, filtered_cos);
    } else {
        _speed_true = _speed_true_raw;
        _direction_true = _direction_true_raw;
    }

// @LoggerMessage: WIND
// @Description: Windvane readings
// @Field: TimeUS: Time since system startup
// @Field: DrRaw: raw apparent wind direction direct from sensor, in body-frame
// @Field: DrApp: Apparent wind direction, in body-frame
// @Field: DrTru: True wind direction
// @Field: SpdRaw: raw wind speed direct from sensor
// @Field: SpdApp: Apparent wind Speed
// @Field: SpdTru: True wind speed
    AP::logger().WriteStreaming("WIND", "TimeUS,DrRaw,DrApp,DrTru,SpdRaw,SpdApp,SpdTru",
                        "sddhnnn", "F000000", "Qffffff",
                        AP_HAL::micros64(),
                        degrees(_direction_apparent_raw),
                        degrees(_direction_apparent),
                        degrees(_direction_true),
                        _speed_apparent_raw,
                        _speed_apparent,
                        _speed_true);

}

void AP_WindVane::record_home_heading()
{
    _home_heading = AP::ahrs().yaw;
}

// to start direction calibration from mavlink or other
bool AP_WindVane::start_direction_calibration()
{
    if (enabled() && (_calibration == 0)) {
        _calibration.set(1);
        return true;
    }
    return false;
}

// to start speed calibration from mavlink or other
bool AP_WindVane::start_speed_calibration()
{
    if (enabled() && (_calibration == 0)) {
        _calibration.set(2);
        return true;
    }
    return false;
}

// send mavlink wind message
void AP_WindVane::send_wind(mavlink_channel_t chan) const
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // send wind
    mavlink_msg_wind_send(
        chan,
        wrap_360(degrees(get_true_wind_direction_rad())),
        get_true_wind_speed(),
        0);

    // send apparent wind using named floats
    // TODO: create a dedicated MAVLink message
    gcs().send_named_float("AppWndSpd", get_apparent_wind_speed());
    gcs().send_named_float("AppWndDir", degrees(get_apparent_wind_direction_rad()));

}

// calculate true wind speed and direction from apparent wind
// https://en.wikipedia.org/wiki/Apparent_wind
void AP_WindVane::update_true_wind_speed_and_direction()
{
    // if no vehicle speed, can't do calcs
    Vector3f veh_velocity;
    if (!AP::ahrs().get_velocity_NED(veh_velocity)) {
        // if no vehicle speed use apparent speed and direction directly
        _direction_true_raw = _direction_apparent_raw;
        _speed_true_raw = _speed_apparent;
        return;
    }

    // convert apparent wind speed and direction to 2D vector in same frame as vehicle velocity
    const float wind_dir_180 = _direction_apparent_raw + AP::ahrs().yaw + radians(180);
    const Vector2f wind_apparent_vec(cosf(wind_dir_180) * _speed_apparent, sinf(wind_dir_180) * _speed_apparent);

    // add vehicle velocity
    Vector2f wind_true_vec = Vector2f(wind_apparent_vec.x + veh_velocity.x, wind_apparent_vec.y + veh_velocity.y);

    // calculate true speed and direction
    _direction_true_raw = wrap_PI(atan2f(wind_true_vec.y, wind_true_vec.x) - radians(180));
    _speed_true_raw = wind_true_vec.length();
}

// apparent wind is low, can't trust sensors, assume true wind has not changed and calculate apparent wind
void AP_WindVane::update_apparent_wind_dir_from_true()
{
    // if no vehicle speed, can't do calcs
    Vector3f veh_velocity;
    if (!AP::ahrs().get_velocity_NED(veh_velocity)) {
        return;
    }

    // convert true wind speed and direction to 2D vector in same frame as vehicle velocity
    const float wind_dir_180 = _direction_true + radians(180);
    const Vector2f wind_true_vec(cosf(wind_dir_180) * _speed_true, sinf(wind_dir_180) * _speed_true);

    // add vehicle velocity
    Vector2f wind_apparent_vec = Vector2f(wind_true_vec.x - veh_velocity.x, wind_true_vec.y - veh_velocity.y);

    // calculate apartment speed and direction
    _direction_apparent_raw = wrap_PI(atan2f(wind_apparent_vec.y, wind_apparent_vec.x) - radians(180) - AP::ahrs().yaw);
    _speed_apparent_raw = wind_apparent_vec.length();
}

AP_WindVane *AP_WindVane::_singleton = nullptr;

namespace AP {
    AP_WindVane *windvane()
    {
        return AP_WindVane::get_singleton();
    }
};
