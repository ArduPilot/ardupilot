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
#include <Filter/Filter.h>
#include <utility>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <board_config.h>
#endif

extern const AP_HAL::HAL& hal;

// by default use the airspeed pin
#define WINDVANE_DEFAULT_PIN 15

const AP_Param::GroupInfo AP_WindVane::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Wind Vane Type
    // @Description: Wind Vane type
    // @Values: 0:None,1:Heading when armed,2:RC input,3:Analog
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_WindVane, _type,  0, AP_PARAM_FLAG_ENABLE),

    // @Param: RC_IN_NO
    // @DisplayName: RC Input Channel to use as wind angle value
    // @Description: RC Input Channel to use as wind angle value
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("RC_IN_NO", 2, AP_WindVane, _rc_in_no,  0),

    // @Param: ANALOG_PIN
    // @DisplayName: Analog input
    // @Description: Analog input pin to read as Wind vane sensor pot
    // @User: Standard
    AP_GROUPINFO("ANA_PIN", 3, AP_WindVane, _analog_pin_no, WINDVANE_DEFAULT_PIN),

    // @Param: ANALOG_V_MIN
    // @DisplayName: Analog minumum voltage
    // @Description: Minimum analalog voltage read by windvane 
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("ANA_V_MIN", 4, AP_WindVane, _analog_volt_min, 0.0f),

    // @Param: ANALOG_V_MAX
    // @DisplayName: Analog maximum voltage
    // @Description: Minimum analalog voltage read by windvane 
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("ANA_V_MAX", 5, AP_WindVane, _analog_volt_max, 3.3f),

    // @Param: ANALOG_V_HEAD
    // @DisplayName: Analog headwind voltage
    // @Description: Voltage when windvane is indicating a headwind, ie 0 degress relative to vehicle
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("ANA_V_HEAD", 6, AP_WindVane, _analog_volt_head, 0.0f),

    // @Param: FILT
    // @DisplayName: Wind vane low pass filter frequency
    // @Description: Wind vane low pass filter frequency
    // @Units: hz
    // @User: Standard
    AP_GROUPINFO("FILT", 7, AP_WindVane, _filt_hz, 1.0f),

    AP_GROUPEND
};

// create a global instances of low pass filter
LowPassFilterFloat low_pass_filter_wind_sin =  LowPassFilterFloat(2.0f);
LowPassFilterFloat low_pass_filter_wind_cos =  LowPassFilterFloat(2.0f);

// Public
// ------

// constructor
AP_WindVane::AP_WindVane()
{       
    AP_Param::setup_object_defaults(this, var_info);
    if (_s_instance) {
        AP_HAL::panic("Too many Wind Vane sensors");
    }
    _s_instance = this;
}

// destructor
AP_WindVane::~AP_WindVane(void)
{       
}

/*
 * Get the AP_WindVane singleton
 */
AP_WindVane *AP_WindVane::get_instance()
{
    return _s_instance;
}

// return true if wind vane is enabled
bool AP_WindVane::enabled() const
{
    return (_type != WINDVANE_NONE);
}

// Initialize the Wind Vane object and prepare it for use
void AP_WindVane::init()
{
    // a pin for reading the Wind Vane voltage.
    windvane_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);    

    // set-up filter
    low_pass_filter_wind_sin.set_cutoff_frequency(_filt_hz);
    low_pass_filter_wind_cos.set_cutoff_frequency(_filt_hz);
}

// Caculate the apparent wind bearing in radians, the wind comes from this direciton, 0 = head to wind, called at 50hz
void AP_WindVane::update_apparent_wind()
{
    float apparent_angle_in = 0.0f;

    switch (_type) {
        case WindVaneType::WINDVANE_HOME_HEADING:
        case WindVaneType::WINDVANE_PWM_PIN:
        {
            // this is a approximation as we are not considering boat speed and wind speed
            // do not filter home heading and pwm type vanes
            _apparent_angle = wrap_PI(get_absolute_wind_direction_rad() - AP::ahrs().yaw);
            return;
        }
        case WindVaneType::WINDVANE_ANALOG_PIN:
        {
            apparent_angle_in = read_analog();
            break;
        }
    }

    // Try and spot a stuck vane, must have moved by atlest 2deg, probably some tuning to be done to the value
    if (fabsf(wrap_PI(_apparent_angle_last-apparent_angle_in)) < radians(2.0f)) {
        apparent_angle_in = _apparent_angle; // if its not moved just use current apparent angle
    } else {
        _apparent_angle_last = apparent_angle_in;
    }

    // apply low pass filter
    // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
    float filtered_sin = low_pass_filter_wind_sin.apply(sinf(apparent_angle_in), 0.02f); // 0.02s = 50hz
    float filtered_cos = low_pass_filter_wind_cos.apply(cosf(apparent_angle_in), 0.02f);
    _apparent_angle = atan2f(filtered_sin, filtered_cos);

    // make sure between -pi and pi
    _apparent_angle = wrap_PI(_apparent_angle);
}

// Return the apparent wind bearing in radians, the wind comes from this direciton, 0 = head to wind
float AP_WindVane::get_apparent_wind_direction_rad()
{
    return _apparent_angle;
}

// Return the absoute wind bearing in radians, the wind comes from this direciton, 0 = North
float AP_WindVane::get_absolute_wind_direction_rad()
{
    // default to 0
    float bearing = 0.0f;

    // PWM and home location directly read absolute bearing
    switch (_type) {
        case WindVaneType::WINDVANE_HOME_HEADING:
        {
            bearing =  _home_heading;
            return bearing;
        }
        case WindVaneType::WINDVANE_PWM_PIN:
        {
            bearing = read_PWM_bearing(); // read bearing from pwm and offset home bearing by that much
            bearing = wrap_2PI(bearing + _home_heading);
            return bearing;
        }
    }

    // convert from apparent
    bearing = apparent_to_absolute();

    return wrap_PI(bearing);
}

// record home heading for use as wind direction if no sensor is fitted
void AP_WindVane::record_home_headng()
{
    _home_heading = AP::ahrs().yaw;
}

// Private
// -------

// read the Wind Vane value from an analog pin
float AP_WindVane::read_analog()
{
    windvane_analog_source->set_pin(_analog_pin_no);
    float current_analog_voltage = windvane_analog_source->voltage_average();

    // calculate bearing from analog voltage
    // assumes voltage increases as wind vane moves clockwise, we could get round this with more complex code and calibration
    // not sure about where to write a calibration code, but we just need to rotate the vane a few times and record the min and max voltage and the set it to head to wind to record the offset

    float voltage_ratio = linear_interpolate(0.0f, 1.0f, current_analog_voltage, _analog_volt_min, _analog_volt_max);
    float bearing_offset = linear_interpolate(0.0f, 1.0f, _analog_volt_head, _analog_volt_min, _analog_volt_max);

    float bearing = (voltage_ratio + bearing_offset) * radians(360);

    return wrap_PI(bearing);
}

// read the bearing value from a PWM value on a RC channel (+- 45deg in radians)
float AP_WindVane::read_PWM_bearing()
{
    RC_Channel *ch = rc().channel(_rc_in_no-1);
    if (ch == nullptr) {
        return 0.0f;
    }
    float bearing = ch->norm_input() * radians(45);

    return wrap_PI(bearing);
}

// convert from apparent wind angle to true wind absolute angle
float AP_WindVane::apparent_to_absolute()
{
    // https://en.wikipedia.org/wiki/Apparent_wind
    float bearing = 0.0f;

    float heading =  AP::ahrs().yaw;
    float ground_speed = 0.0f; // AP::ahrs().groundspeed;
    float apparent_wind_speed = 0.0f; // read wind speed from sensor
 
    // Calculate true wind speed (possibly put this in another function somewhere)
    float true_wind_speed = sqrtf( powf(apparent_wind_speed,2)  + powf(ground_speed,2)  - 2 * apparent_wind_speed * ground_speed * cosf(_apparent_angle));

    if (is_zero(true_wind_speed)) { // There is no true wind, so return apparent angle, to avoid divide by zero
        bearing = _apparent_angle;
    } else if (_apparent_angle <= radians(180)) {
        bearing = acosf((apparent_wind_speed * cosf(_apparent_angle) - ground_speed) / true_wind_speed);
    } else {
        bearing = -acosf((apparent_wind_speed * cosf(_apparent_angle) - ground_speed) / true_wind_speed);
    }

    // make sure between -pi and pi
    bearing = wrap_2PI(heading + bearing);
    return bearing;
}

AP_WindVane *AP_WindVane::_s_instance = nullptr;

namespace AP {
    AP_WindVane *windvane()
    {
        return AP_WindVane::get_instance();
    }
};
