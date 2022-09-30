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

#include <AP_Param/AP_Param.h>
#include <Filter/Filter.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#ifndef WINDVANE_DEFAULT_PIN
#define WINDVANE_DEFAULT_PIN -1                     // default wind vane sensor analog pin
#endif
#ifndef WINDSPEED_DEFAULT_SPEED_PIN
#define WINDSPEED_DEFAULT_SPEED_PIN -1              // default pin for reading speed from ModernDevice rev p wind sensor
#endif
#ifndef WINDSPEED_DEFAULT_TEMP_PIN
#define WINDSPEED_DEFAULT_TEMP_PIN -1               // default pin for reading temperature from ModernDevice rev p wind sensor
#endif
#define WINDSPEED_DEFAULT_VOLT_OFFSET 1.346f        // default voltage offset between speed and temp pins from ModernDevice rev p wind sensor

class AP_WindVane_Backend;

class AP_WindVane
{
    friend class AP_WindVane_Backend;
    friend class AP_WindVane_Home;
    friend class AP_WindVane_Analog;
    friend class AP_WindVane_SITL;
    friend class AP_WindVane_ModernDevice;
    friend class AP_WindVane_Airspeed;
    friend class AP_WindVane_RPM;
    friend class AP_WindVane_NMEA;

public:
    AP_WindVane();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_WindVane);

    static AP_WindVane *get_singleton();

    // return true if wind vane is enabled
    bool enabled() const;

    // return true if wind speed is enabled
    bool wind_speed_enabled() const;

    // Initialize the Wind Vane object and prepare it for use
    void init(const class AP_SerialManager& serial_manager);

    // update wind vane
    void update();

    // get the apparent wind direction in body-frame in radians, 0 = head to wind
    float get_apparent_wind_direction_rad() const { return _direction_apparent; }

    // get the true wind direction in radians, 0 = wind coming from north
    float get_true_wind_direction_rad() const { return _direction_true; }

    // Return apparent wind speed
    float get_apparent_wind_speed() const { return _speed_apparent; }

    // Return true wind speed
    float get_true_wind_speed() const { return _speed_true; }

    // Return the apparent wind angle used to determin the current tack
    float get_tack_threshold_wind_dir_rad() const { return _direction_tack; }

    // enum defining current tack
    enum Sailboat_Tack {
        TACK_PORT,
        TACK_STARBOARD
    };

    // return the current tack
    Sailboat_Tack get_current_tack() const { return _current_tack; }

    // record home heading for use as wind direction if no sensor is fitted
    void record_home_heading();

    // start calibration routine
    bool start_direction_calibration();
    bool start_speed_calibration();

    // send mavlink wind message
    void send_wind(mavlink_channel_t chan) const;

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Int8 _direction_type;                        // type of windvane being used
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
    AP_Float _true_filt_hz;                         // true wind speed and direction low pass filter frequency

    AP_WindVane_Backend *_direction_driver;
    AP_WindVane_Backend *_speed_driver;

    // update wind speed sensor
    void update_apparent_wind_speed();

    // update apparent wind direction
    void update_apparent_wind_direction();

    // calculate true wind speed and direction from apparent wind
    void update_true_wind_speed_and_direction();

    // assume true wind has not changed and calculate apparent wind
    void update_apparent_wind_dir_from_true();

    // wind direction variables
    float _direction_apparent_raw;                  // wind's apparent direction in radians (0 = ahead of vehicle) in body frame
    float _direction_apparent;                      // wind's apparent direction in radians (0 = ahead of vehicle) in body frame - filtered
    float _direction_true_raw;                      // wind's true direction in radians (0 = North)
    float _direction_true;                          // wind's true direction in radians (0 = North) - filtered
    float _direction_tack;                          // filtered apparent wind used to determin the current tack
    LowPassFilterFloat _direction_apparent_sin_filt{2.0f};
    LowPassFilterFloat _direction_apparent_cos_filt{2.0f};
    LowPassFilterFloat _direction_true_sin_filt{2.0f};
    LowPassFilterFloat _direction_true_cos_filt{2.0f};
    LowPassFilterFloat _tack_sin_filt{0.1f};
    LowPassFilterFloat _tack_cos_filt{0.1f};

    // wind speed variables
    float _speed_apparent_raw;                      // wind's apparent speed in m/s
    float _speed_apparent;                          // wind's apparent speed in m/s - filtered
    float _speed_true_raw;                          // wind's true estimated speed in m/s
    float _speed_true;                              // wind's true estimated speed in m/s - filtered
    LowPassFilterFloat _speed_apparent_filt{2.0f};
    LowPassFilterFloat _speed_true_filt{2.0f};

    // current tack
    Sailboat_Tack _current_tack;

    // heading in radians recorded when vehicle was armed
    float _home_heading;

    enum WindVaneType {
        WINDVANE_NONE           = 0,
        WINDVANE_HOME_HEADING   = 1,
        WINDVANE_PWM_PIN        = 2,
        WINDVANE_ANALOG_PIN     = 3,
        WINDVANE_NMEA           = 4,
        WINDVANE_SITL_TRUE      = 10,
        WINDVANE_SITL_APPARENT  = 11,
    };

    enum Speed_type {
        WINDSPEED_NONE               = 0,
        WINDSPEED_AIRSPEED           = 1,
        WINDVANE_WIND_SENSOR_REV_P   = 2,
        WINDSPEED_RPM                = 3,
        WINDSPEED_NMEA               = 4,
        WINDSPEED_SITL_TRUE          = 10,
        WINDSPEED_SITL_APPARENT      = 11,
    };

    static AP_WindVane *_singleton;
};

namespace AP {
    AP_WindVane *windvane();
};
