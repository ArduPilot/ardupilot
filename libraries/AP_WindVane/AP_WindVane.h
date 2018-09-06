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

    // get the apparent wind direction in radians
    float get_apparent_wind_direction_rad();

    // get the absolute wind direction in radians
    float get_absolute_wind_direction_rad();
    
    // Update apparent wind direction
    void update_apparent_wind();

    // record home heading
    void record_home_headng();

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // read the bearing value from an analog pin - returns radians
    float read_analog();

    // read the bearing value from a PWM value on a RC channel - returns radians
    float read_PWM_bearing();

    // Convert from apparent wind angle to true wind absolute angle
    float apparent_to_absolute();

    // parameters
    AP_Int8 _type;             // type of windvane being used
    AP_Int8 _rc_in_no;         // RC input channel to use
    AP_Int8 _analog_pin_no;    // analog pin connected to sensor
    AP_Float _analog_volt_min;          // minimum voltage read by windvane
    AP_Float _analog_volt_max;          // maximum voltage read by windvane
    AP_Float _analog_volt_head;         // voltage when windvane is indicating a headwind, ie 0 degress relative to vehicle
    AP_Float _filter_freq;              // Low pass filter frequency

    static AP_WindVane *_s_instance;
    float _home_heading;
    float _apparent_angle;
    float _apparent_angle_last; 

    // pin for reading analog voltage
    AP_HAL::AnalogSource *windvane_analog_source;
};

namespace AP {
    AP_WindVane *windvane();
};
