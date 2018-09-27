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

// by default use the airspeed pin for Vane
#define WINDVANE_DEFAULT_PIN 15
#define WINDVANE_CALIBRATION_VOLT_DIFF_MIN  1.0f    // calibration routine's min voltage difference required for success

const AP_Param::GroupInfo AP_WindVane::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Wind Vane Type
    // @Description: Wind Vane type
    // @Values: 0:None,1:Heading when armed,2:RC input offset heading when armed,3:Analog
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_WindVane, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: RC_IN_NO
    // @DisplayName: RC Input Channel to use as wind angle value
    // @Description: RC Input Channel to use as wind angle value
    // @Range: 0 16
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RC_IN_NO", 2, AP_WindVane, _rc_in_no, 0),

    // @Param: ANA_PIN
    // @DisplayName: Analog input
    // @Description: Analog input pin to read as Wind vane sensor pot
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6,15:Pixhawk2 ADC,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("ANA_PIN", 3, AP_WindVane, _analog_pin_no, WINDVANE_DEFAULT_PIN),

    // @Param: ANA_V_MIN
    // @DisplayName: Analog minumum voltage
    // @Description: Minimum analalog voltage read by windvane
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("ANA_V_MIN", 4, AP_WindVane, _analog_volt_min, 0.0f),

    // @Param: ANA_V_MAX
    // @DisplayName: Analog maximum voltage
    // @Description: Minimum analalog voltage read by windvane
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("ANA_V_MAX", 5, AP_WindVane, _analog_volt_max, 3.3f),

    // @Param: ANA_OF_HD
    // @DisplayName: Analog headwind offset
    // @Description: Angle offset when windvane is indicating a headwind, ie 0 degress relative to vehicle
    // @Units: deg
    // @Increment: 1
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("ANA_OF_HD", 6, AP_WindVane, _analog_head_bearing_offset, 0.0f),

    // @Param: VANE_FILT
    // @DisplayName: Wind vane low pass filter frequency
    // @Description: Wind vane low pass filter frequency, a value of -1 disables filter
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("VANE_FILT", 7, AP_WindVane, _vane_filt_hz, 0.5f),

    // @Param: CAL
    // @DisplayName: set to one to enter calibration on reboot
    // @Description: set to one to enter calibration on reboot
    // @Values: 0:None, 1:Calibrate
    // @User: Standard
    AP_GROUPINFO("CAL", 8, AP_WindVane, _calibration, 0),

    // @Param: ANA_DZ
    // @DisplayName: Analog potentiometer dead zone
    // @Description: Analog potentiometer mechanical dead zone
    // @Units: deg
    // @Increment: 1
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("ANA_DZ", 9, AP_WindVane, _analog_deadzone, 0),

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

AP_WindVane::~AP_WindVane()
{
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
    windvane_analog_source = hal.analogin->channel(_analog_pin_no);
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

    update_apparent_wind_direction();
}

// get the apparent wind direction in radians, 0 = head to wind
float AP_WindVane::get_apparent_wind_direction_rad() const
{
    return wrap_PI(_direction_apparent_ef - AP::ahrs().yaw);
}

// record home heading for use as wind direction if no sensor is fitted
void AP_WindVane::record_home_headng()
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

// read an analog port and calculate the wind direction in earth-frame in radians
// assumes voltage increases as wind vane moves clockwise
float AP_WindVane::read_analog_direction_ef()
{
    windvane_analog_source->set_pin(_analog_pin_no);
    _current_analog_voltage = windvane_analog_source->voltage_average_ratiometric();

    const float voltage_ratio = linear_interpolate(0.0f, 1.0f, _current_analog_voltage, _analog_volt_min, _analog_volt_max);
    const float direction = (voltage_ratio * radians(360 - _analog_deadzone)) + radians(_analog_head_bearing_offset);

    return wrap_PI(direction + AP::ahrs().yaw);
}

// read rc input of apparent wind direction in earth-frame in radians
float AP_WindVane::read_PWM_direction_ef()
{
    RC_Channel *ch = rc().channel(_rc_in_no-1);
    if (ch == nullptr) {
        return 0.0f;
    }
    float direction = ch->norm_input() * radians(45);

    return wrap_PI(direction + _home_heading);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// read SITL's apparent wind direction in earth-frame in radians
float AP_WindVane::read_SITL_direction_ef()
{
    // temporarily store true speed and direction for easy access
    const float wind_speed = AP::sitl()->wind_speed_active;
    const float wind_dir_rad = radians(AP::sitl()->wind_direction_active);

    // convert true wind speed and direction into a 2D vector
    Vector2f wind_vector_ef(sinf(wind_dir_rad) * wind_speed, cosf(wind_dir_rad) * wind_speed);

    // add vehicle speed to get apparent wind vector
    wind_vector_ef.x += AP::sitl()->state.speedE;
    wind_vector_ef.y += AP::sitl()->state.speedN;

    return atan2f(wind_vector_ef.x, wind_vector_ef.y);
}
#endif

// calculate the apparent wind direction in radians, the wind comes from this direction, 0 = head to wind
// expected to be called at 20hz
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

    // apply low pass filter if enabled
    if (is_positive(_vane_filt_hz)) {
        wind_sin_filt.set_cutoff_frequency(_vane_filt_hz);
        wind_cos_filt.set_cutoff_frequency(_vane_filt_hz);
        // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
        const float filtered_sin = wind_sin_filt.apply(sinf(apparent_angle_ef), 0.05f);
        const float filtered_cos = wind_cos_filt.apply(cosf(apparent_angle_ef), 0.05f);
        _direction_apparent_ef = atan2f(filtered_sin, filtered_cos);
    } else {
        _direction_apparent_ef = apparent_angle_ef;
    }

    // make sure between -pi and pi
    _direction_apparent_ef = wrap_PI(_direction_apparent_ef);
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
                    _analog_volt_max.set_and_save(_cal_volt_max);
                    _analog_volt_min.set_and_save(_cal_volt_min);
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
