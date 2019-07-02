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
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_RPM/AP_RPM.h>

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

    static AP_WindVane *get_singleton();

    // return true if wind vane is enabled
    bool enabled() const;

    // Initialize the Wind Vane object and prepare it for use
    void init();

    // update wind vane
    void update();

    // get the apparent wind direction in body-frame in radians, 0 = head to wind
    float get_apparent_wind_direction_rad() const;

    // get the absolute wind direction in radians, 0 = wind coming from north
    float get_absolute_wind_direction_rad() const { return _direction_absolute; }

    // Return apparent wind speed
    float get_apparent_wind_speed() const { return _speed_apparent; }

    // Return true wind speed
    float get_true_wind_speed() const { return _speed_true; }

    // record home heading
    void record_home_heading();

    // start calibration routine
    bool start_calibration();

    // send mavlink wind message
    void send_wind(mavlink_channel_t chan);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // read an analog port and calculate the wind direction in earth-frame in radians
    float read_analog_direction_ef();

    // read rc input of apparent wind direction in earth-frame in radians
    float read_PWM_direction_ef();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // read SITL's apparent wind direction in earth-frame in radians
    float read_SITL_direction_ef();

    // read the apparent wind speed in m/s from SITL
    float read_wind_speed_SITL();
#endif

    // read wind speed from ModernDevice wind speed sensor rev p
    float read_wind_speed_ModernDevice();

    // update wind speed sensor
    void update_apparent_wind_speed();

    // update apparent wind direction
    void update_apparent_wind_direction();

    // calculate true wind speed and direction from apparent wind
    void update_true_wind_speed_and_direction();

    // calibrate
    void calibrate();

    // parameters
    AP_Int8 _type;                                  // type of windvane being used
    AP_Int8 _rc_in_no;                              // RC input channel to use
    AP_Int8 _dir_analog_pin;                        // analog pin connected to wind vane direction sensor
    AP_Float _dir_analog_volt_min;                  // minimum voltage read by windvane
    AP_Float _dir_analog_volt_max;                  // maximum voltage read by windvane
    AP_Float _dir_analog_bearing_offset;            // angle offset when windvane is indicating a headwind, ie 0 degress relative to vehicle
    AP_Float _dir_analog_deadzone;                  // analog pot deadzone in degrees
    AP_Float _dir_filt_hz;                          // vane Low pass filter frequency
    AP_Int8 _calibration;                           // enter calibration
    AP_Float _dir_speed_cutoff;                     // vane cutoff wind speed
    AP_Int8 _speed_sensor_type;                     // wind speed sensor type
    AP_Int8 _speed_sensor_speed_pin;                // speed sensor analog pin for reading speed
    AP_Float _speed_sensor_temp_pin;                // speed sensor analog pin for reading temp, -1 if disable
    AP_Float _speed_sensor_voltage_offset;          // analog speed zero wind voltage offset
    AP_Float _speed_filt_hz;                        // speed sensor low pass filter frequency

    static AP_WindVane *_singleton;

    // wind direction variables
    float _home_heading;                            // heading in radians recorded when vehicle was armed
    float _direction_apparent_ef;                   // wind's apparent direction in radians (0 = ahead of vehicle)
    float _direction_absolute;                      // wind's absolute direction in radians (0 = North)
    float _current_analog_voltage;                  // wind direction's latest analog voltage reading

    // wind speed variables
    float _speed_apparent;                          // wind's apparent speed in m/s
    float _speed_true;                              // wind's true estimated speed in m/s

    // calibration variables
    uint32_t _cal_start_ms = 0;                     // calibration start time in milliseconds after boot
    float _cal_volt_max;                            // maximum observed voltage during calibration
    float _cal_volt_min;                            // minimum observed voltage during calibration

    enum Speed_type {
        WINDSPEED_NONE               = 0,
        WINDSPEED_AIRSPEED           = 1,
        WINDVANE_WIND_SENSOR_REV_P   = 2,
        WINDSPEED_RPM                = 3,
        WINDSPEED_SITL               = 10
    };

    // pin for reading analog voltage
    AP_HAL::AnalogSource *dir_analog_source;
    AP_HAL::AnalogSource *speed_analog_source;
    AP_HAL::AnalogSource *speed_temp_analog_source;

    // low pass filters of direction and speed
    LowPassFilterFloat _dir_sin_filt = LowPassFilterFloat(2.0f);
    LowPassFilterFloat _dir_cos_filt = LowPassFilterFloat(2.0f);
    LowPassFilterFloat _speed_filt = LowPassFilterFloat(2.0f);
};

namespace AP {
    AP_WindVane *windvane();
};
