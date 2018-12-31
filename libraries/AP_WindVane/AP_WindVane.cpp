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

#include <AP_WindVane/AP_WindVane.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <Filter/Filter.h>
#include <utility>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <board_config.h>
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

extern const AP_HAL::HAL& hal;

#define WINDVANE_DEFAULT_PIN 15                     // default wind vane sensor analog pin
#define WINDVANE_CALIBRATION_VOLT_DIFF_MIN  1.0f    // calibration routine's min voltage difference required for success
#define WINDSPEED_DEFAULT_SPEED_PIN 14              // default pin for reading speed from ModernDevice rev p wind sensor
#define WINDSPEED_DEFAULT_TEMP_PIN 13               // default pin for reading temperature from ModernDevice rev p wind sensor
#define WINDSPEED_DEFAULT_VOLT_OFFSET 1.346         // default voltage offset between speed and temp pins from ModernDevice rev p wind sensor

const AP_Param::GroupInfo AP_WindVane::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Wind Vane Type
    // @Description: Wind Vane type
    // @Values: 0:None,1:Heading when armed,2:RC input offset heading when armed,3:Analog
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_WindVane, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: RC_IN_NO
    // @DisplayName: Wind vane sensor RC Input Channel
    // @Description: RC Input Channel to use as wind angle value
    // @Range: 0 16
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RC_IN_NO", 2, AP_WindVane, _rc_in_no, 0),

    // @Param: DIR_PIN
    // @DisplayName: Wind vane analog voltage pin for direction
    // @Description: Analog input pin to read as wind vane direction
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6,15:Pixhawk2 ADC,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("DIR_PIN", 3, AP_WindVane, _dir_analog_pin, WINDVANE_DEFAULT_PIN),

    // @Param: DIR_V_MIN
    // @DisplayName: Wind vane voltage minimum
    // @Description: Minimum voltage supplied by analog wind vane
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("DIR_V_MIN", 4, AP_WindVane, _dir_analog_volt_min, 0.0f),

    // @Param: DIR_V_MAX
    // @DisplayName: Wind vane voltage maximum
    // @Description: Maximum voltage supplied by analog wind vane
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
    // @DisplayName: Wind vane direction low pass filter frequency
    // @Description: Wind vane direction low pass filter frequency, a value of -1 disables filter
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("DIR_FILT", 7, AP_WindVane, _dir_filt_hz, 0.5f),

    // @Param: CAL
    // @DisplayName: Wind vane calibration start
    // @Description: Start wind vane calibration by setting this to 1
    // @Values: 0:None, 1:Calibrate
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
    // @Values: 0:None,1:Airspeed library,2:Moden Devices Wind Sensor,3:SITL
    // @User: Standard
    AP_GROUPINFO("SPEED_TYPE", 11, AP_WindVane, _speed_sensor_type,  0),

    // @Param: SPEED_PIN
    // @DisplayName: Wind vane speed sensor analog pin
    // @Description: Wind speed analog speed input pin for Modern Devices Wind Sensor rev. p
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6,15:Pixhawk2 ADC,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("SPEED_PIN", 12, AP_WindVane, _speed_sensor_speed_pin,  WINDSPEED_DEFAULT_SPEED_PIN),

    // @Param: TEMP_PIN
    // @DisplayName: Wind vane speed sensor analog temp pin
    // @Description: Wind speed sensor analog temp input pin for Moden Devices Wind Sensor rev. p, set to -1 to diasble temp readings
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6,15:Pixhawk2 ADC,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6,103:Pixhawk SBUS
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
    // @DisplayName: Wind speed low pass filter frequency
    // @Description: Wind speed low pass filter frequency, a value of -1 disables filter
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("SPEED_FILT", 15, AP_WindVane, _speed_filt_hz, 0.5f),

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
    return (_type != WINDVANE_NONE);
}

// Initialize the Wind Vane object and prepare it for use
void AP_WindVane::init()
{
    // a pin for reading the Wind Vane voltage
    dir_analog_source = hal.analogin->channel(_dir_analog_pin);

    // pins for ModernDevice rev p wind sensor
    speed_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);
    speed_temp_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);

    // check that airspeed is enabled if it is selected as sensor type, if not revert to no wind speed sensor
    AP_Airspeed* airspeed = AP_Airspeed::get_singleton();
    if (_speed_sensor_type == Speed_type::WINDSPEED_AIRSPEED && (airspeed == nullptr || !airspeed->enabled())) {
        _speed_sensor_type.set(Speed_type::WINDSPEED_NONE);
    }
}

// update wind vane, expected to be called at 20hz
void AP_WindVane::update()
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // check for calibration
    calibrate();

    // read apparent wind speed
    update_apparent_wind_speed();

    // read apparent wind direction (relies on wind speed above)
    update_apparent_wind_direction();

    // calculate true wind speed and direction from apparent wind
    update_true_wind_speed_and_direction();
}

// get the apparent wind direction in radians, 0 = head to wind
float AP_WindVane::get_apparent_wind_direction_rad() const
{
    return wrap_PI(_direction_apparent_ef - AP::ahrs().yaw);
}

// record home heading for use as wind direction if no sensor is fitted
void AP_WindVane::record_home_heading()
{
    _home_heading = AP::ahrs().yaw;
}

bool AP_WindVane::start_calibration()
{
    if (enabled() && (_calibration == 0)) {
        _calibration = 1;
        return true;
    }
    return false;
}

// send mavlink wind message
void AP_WindVane::send_wind(mavlink_channel_t chan)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // send wind
    mavlink_msg_wind_send(
        chan,
        wrap_360(degrees(get_absolute_wind_direction_rad())),
        get_true_wind_speed(),
        0);
}

// read an analog port and calculate the wind direction in earth-frame in radians
// assumes voltage increases as wind vane moves clockwise
float AP_WindVane::read_analog_direction_ef()
{
    dir_analog_source->set_pin(_dir_analog_pin);
    _current_analog_voltage = dir_analog_source->voltage_average_ratiometric();

    const float voltage_ratio = linear_interpolate(0.0f, 1.0f, _current_analog_voltage, _dir_analog_volt_min, _dir_analog_volt_max);
    const float direction = (voltage_ratio * radians(360 - _dir_analog_deadzone)) + radians(_dir_analog_bearing_offset);

    return wrap_PI(direction + AP::ahrs().yaw);
}

// read rc input of apparent wind direction in earth-frame in radians
float AP_WindVane::read_PWM_direction_ef()
{
    RC_Channel *chan = rc().channel(_rc_in_no-1);
    if (chan == nullptr) {
        return 0.0f;
    }
    float direction = chan->norm_input() * radians(45);

    return wrap_PI(direction + _home_heading);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// read SITL's apparent wind direction in earth-frame in radians
float AP_WindVane::read_SITL_direction_ef()
{
    // temporarily store true speed and direction for easy access
    const float wind_speed = AP::sitl()->wind_speed_active;
    const float wind_dir_rad = radians(AP::sitl()->wind_direction_active);

    // Note than the SITL wind direction is defined as the direction the wind is travelling to
    // This is accounted for in these calculations

    // convert true wind speed and direction into a 2D vector
    Vector2f wind_vector_ef(cosf(wind_dir_rad) * wind_speed, sinf(wind_dir_rad) * wind_speed);

    // add vehicle speed to get apparent wind vector
    wind_vector_ef.x += AP::sitl()->state.speedN;
    wind_vector_ef.y += AP::sitl()->state.speedE;

    return atan2f(wind_vector_ef.y, wind_vector_ef.x);
}

// read the apparent wind speed in m/s from SITL
float AP_WindVane::read_wind_speed_SITL()
{
    // temporarily store true speed and direction for easy access
    const float wind_speed = AP::sitl()->wind_speed_active;
    const float wind_dir_rad = radians(AP::sitl()->wind_direction_active);

    // convert true wind speed and direction into a 2D vector
    Vector2f wind_vector_ef(cosf(wind_dir_rad) * wind_speed, sinf(wind_dir_rad) * wind_speed);

    // add vehicle speed to get apparent wind vector
    wind_vector_ef.x += AP::sitl()->state.speedN;
    wind_vector_ef.y += AP::sitl()->state.speedE;

    return wind_vector_ef.length();
}
#endif

// read wind speed from Modern Device rev p wind sensor
// https://moderndevice.com/news/calibrating-rev-p-wind-sensor-new-regression/
float AP_WindVane::read_wind_speed_ModernDevice()
{
    float analog_voltage = 0.0f;

    // only read temp pin if defined, sensor will do OK assuming constant temp
    float temp_ambient = 28.0f; // equations were generated at this temp in above data sheet
    if (is_positive(_speed_sensor_temp_pin)) {
        speed_temp_analog_source->set_pin(_speed_sensor_temp_pin);
        analog_voltage = speed_temp_analog_source->voltage_average();
        temp_ambient = (analog_voltage - 0.4f) / 0.0195f; // deg C
        // constrain to reasonable range to avoid deviating from calibration too much and potential divide by zero
        temp_ambient = constrain_float(temp_ambient, 10.0f, 40.0f);
    }

    speed_analog_source->set_pin(_speed_sensor_speed_pin);
    analog_voltage = speed_analog_source->voltage_average();

    // apply voltage offset and make sure not negative
    // by default the voltage offset is the number provide by the manufacturer
    analog_voltage = analog_voltage - _speed_sensor_voltage_offset;
    if (is_negative(analog_voltage)) {
        analog_voltage = 0.0f;
    }

    // simplified equation from data sheet, converted from mph to m/s
    return 24.254896f * powf((analog_voltage / powf(temp_ambient, 0.115157f)), 3.009364f);
}

// update the apparent wind speed
void AP_WindVane::update_apparent_wind_speed()
{
    float apparent_speed_in = 0.0f;

    switch (_speed_sensor_type) {
        case WINDSPEED_NONE:
            _speed_apparent = 0.0f;
            break;
        case WINDSPEED_AIRSPEED: {
            AP_Airspeed* airspeed = AP_Airspeed::get_singleton();
            if (airspeed != nullptr) {
                apparent_speed_in = airspeed->get_airspeed();
            }
            break;
        }
        case WINDVANE_WIND_SENSOR_REV_P:
            apparent_speed_in = read_wind_speed_ModernDevice();
            break;
        case WINDSPEED_SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            apparent_speed_in = read_wind_speed_SITL();
#endif
            break;
    }

    // apply low pass filter if enabled
    if (is_positive(_speed_filt_hz)) {
        _speed_filt.set_cutoff_frequency(_speed_filt_hz);
        _speed_apparent = _speed_filt.apply(apparent_speed_in, 0.02f);
    } else {
        _speed_apparent = apparent_speed_in;
    }
}

// calculate the apparent wind direction in radians, the wind comes from this direction, 0 = head to wind
// expected to be called at 20hz after apparent wind speed has been updated
void AP_WindVane::update_apparent_wind_direction()
{
    float apparent_angle_ef = 0.0f;

    switch (_type) {
        case WindVaneType::WINDVANE_HOME_HEADING:
            // this is a approximation as we are not considering boat speed and wind speed
            // do not filter home heading
            _direction_apparent_ef = _home_heading;
            return;
        case WindVaneType::WINDVANE_PWM_PIN:
            // this is a approximation as we are not considering boat speed and wind speed
            // do not filter pwm input from pilot
            _direction_apparent_ef = read_PWM_direction_ef();
            return;
        case WindVaneType::WINDVANE_ANALOG_PIN:
            apparent_angle_ef = read_analog_direction_ef();
            break;
        case WindVaneType::WINDVANE_SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            apparent_angle_ef = read_SITL_direction_ef();
#endif
            break;
    }

    // if not enough wind to move vane do not update the value
    if (_speed_apparent < _dir_speed_cutoff){
        return;
    }

    // apply low pass filter if enabled
    if (is_positive(_dir_filt_hz)) {
        _dir_sin_filt.set_cutoff_frequency(_dir_filt_hz);
        _dir_cos_filt.set_cutoff_frequency(_dir_filt_hz);
        // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
        const float filtered_sin = _dir_sin_filt.apply(sinf(apparent_angle_ef), 0.05f);
        const float filtered_cos = _dir_cos_filt.apply(cosf(apparent_angle_ef), 0.05f);
        _direction_apparent_ef = atan2f(filtered_sin, filtered_cos);
    } else {
        _direction_apparent_ef = apparent_angle_ef;
    }

    // make sure between -pi and pi
    _direction_apparent_ef = wrap_PI(_direction_apparent_ef);
}

// calculate true wind speed and direction from apparent wind
// https://en.wikipedia.org/wiki/Apparent_wind
void AP_WindVane::update_true_wind_speed_and_direction()
{
    if (_speed_sensor_type == Speed_type::WINDSPEED_NONE) {
        // no wind speed sensor, so can't do true wind calcs
        _direction_absolute = _direction_apparent_ef;
        _speed_true = 0.0f;
        return;
    }

    // if no vehicle speed, can't do calcs
    Vector3f veh_velocity;
    if (!AP::ahrs().get_velocity_NED(veh_velocity)) {
        // if no vehicle speed use apparent speed and direction directly
        _direction_absolute = _direction_apparent_ef;
        _speed_true = _speed_apparent;
        return;
    }

    // convert apparent wind speed and direction to 2D vector in same frame as vehicle velocity
    const float wind_dir_180 = wrap_PI(_direction_apparent_ef + radians(180));
    const Vector2f wind_apparent_vec(cosf(wind_dir_180) * _speed_apparent, sinf(wind_dir_180) * _speed_apparent);

    // add vehicle velocity
    Vector2f wind_true_vec = Vector2f(wind_apparent_vec.x + veh_velocity.x, wind_apparent_vec.y + veh_velocity.y);

    // calculate true speed and direction
    _direction_absolute = wrap_PI(atan2f(wind_true_vec.y, wind_true_vec.x) - radians(180));
    _speed_true = wind_true_vec.length();
}

// calibrate windvane
void AP_WindVane::calibrate()
{
    // exit immediately if armed or too soon after start
    if (hal.util->get_soft_armed()) {
        return;
    }

    // return if not calibrating
    if (_calibration == 0) {
        return;
    }

    switch (_type) {
        case WindVaneType::WINDVANE_HOME_HEADING:
        case WindVaneType::WINDVANE_PWM_PIN:
            gcs().send_text(MAV_SEVERITY_INFO, "WindVane: No cal required");
            _calibration.set_and_save(0);
            break;
        case WindVaneType::WINDVANE_ANALOG_PIN:
            // start calibration
            if (_cal_start_ms == 0) {
                _cal_start_ms = AP_HAL::millis();
                _cal_volt_max = _current_analog_voltage;
                _cal_volt_min = _current_analog_voltage;
                gcs().send_text(MAV_SEVERITY_INFO, "WindVane: Calibration started, rotate wind vane");
            }

            // record min and max voltage
            _cal_volt_max = MAX(_cal_volt_max, _current_analog_voltage);
            _cal_volt_min = MIN(_cal_volt_min, _current_analog_voltage);

            // calibrate for 30 seconds
            if ((AP_HAL::millis() - _cal_start_ms) > 30000) {
                // check for required voltage difference
                const float volt_diff = _cal_volt_max - _cal_volt_min;
                if (volt_diff >= WINDVANE_CALIBRATION_VOLT_DIFF_MIN) {
                    // save min and max voltage
                    _dir_analog_volt_max.set_and_save(_cal_volt_max);
                    _dir_analog_volt_min.set_and_save(_cal_volt_min);
                    _calibration.set_and_save(0);
                    gcs().send_text(MAV_SEVERITY_INFO, "WindVane: Calibration complete (volt min:%.1f max:%1.f)",
                            (double)_cal_volt_min,
                            (double)_cal_volt_max);
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "WindVane: Calibration failed (volt diff %.1f below %.1f)",
                            (double)volt_diff,
                            (double)WINDVANE_CALIBRATION_VOLT_DIFF_MIN);
                }
                _cal_start_ms = 0;
            }
            break;
    }
}

AP_WindVane *AP_WindVane::_singleton = nullptr;

namespace AP {
    AP_WindVane *windvane()
    {
        return AP_WindVane::get_singleton();
    }
};
