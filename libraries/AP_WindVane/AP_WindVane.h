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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_WindVane
{

public:

    enum WindVaneType {
        WINDVANE_NONE       = 0,
        WINDVANE_HOME_HEADING = 1,
        WINDVANE_PWM_PIN    = 2,
        WINDVANE_ANALOG_PIN = 3,
        WINDVANE_SITL       = 10
    };

    AP_WindVane();

    /* Do not allow copies */
    AP_WindVane(const AP_WindVane &other) = delete;
    AP_WindVane &operator=(const AP_WindVane&) = delete;

    // destructor
    ~AP_WindVane(void);

    static AP_WindVane *get_instance();

    // return true if wind vane is enabled
    bool enabled() const;

    // Initialize the Wind Vane object and prepare it for use
    void init();

    // update wind vane
    void update();

    // get the apparent wind direction in radians, 0 = head to wind
    float get_apparent_wind_direction_rad() const { return _direction_apparent; }

    // record home heading
    void record_home_headng();

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // read the bearing value from an analog pin - returns radians
    float read_analog();

    // read the bearing value from a PWM value on a RC channel - returns radians
    float read_PWM_bearing();

    // read the apparent wind direction in radians from SITL
    float read_direction_SITL();

    // update apparent wind direction
    void update_apparent_wind_direction();

    // calibrate
    void calibrate();

    // parameters
    AP_Int8 _type;                                  // type of windvane being used
    AP_Int8 _rc_in_no;                              // RC input channel to use
    AP_Int8 _analog_pin_no;                         // analog pin connected to sensor
    AP_Float _analog_volt_min;                      // minimum voltage read by windvane
    AP_Float _analog_volt_max;                      // maximum voltage read by windvane
    AP_Float _analog_head_bearing_offset;           // angle offset when windvane is indicating a headwind, ie 0 degress relative to vehicle
    AP_Float _vane_filt_hz;                         // vane Low pass filter frequency
    AP_Int8 _calibration;                           // enter calibration
    AP_Float _analog_deadzone;                      // analog pot deadzone in degrees

    static AP_WindVane *_s_instance;

    // wind direction variables
    float _home_heading;                            // heading in radians recorded when vehicle was armed
    float _direction_apparent;                      // wind's apparent direction in radians (0 = ahead of vehicle)
    float _current_analog_voltage;                  // wind direction's latest analog voltage reading

    // calibration variables
    uint32_t _cal_start_ms = 0;                     // calibration start time in milliseconds after boot
    float _cal_volt_max;                            // maximum observed voltage during calibration
    float _cal_volt_min;                            // minimum observed voltage during calibration

    // pin for reading analog voltage
    AP_HAL::AnalogSource *windvane_analog_source;
};

namespace AP {
    AP_WindVane *windvane();
};
