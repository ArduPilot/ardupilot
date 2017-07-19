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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// Maximum number of WheelEncoder measurement instances available on this platform
#define WHEELENCODER_MAX_INSTANCES      2
#define WHEELENCODER_SCALING_DEFAULT    0.05f   // default scaling between sensor readings and millimeters

class AP_WheelEncoder_Backend; 
 
class AP_WheelEncoder
{
public:
    friend class AP_WheelEncoder_Backend;
    friend class AP_WheelEncoder_Quadrature;

    AP_WheelEncoder(void);

    // WheelEncoder driver types
    enum WheelEncoder_Type {
        WheelEncoder_TYPE_NONE          = 0,
        WheelEncoder_TYPE_QUADRATURE    = 1
    };

    // The WheelEncoder_State structure is filled in by the backend driver
    struct WheelEncoder_State {
        uint8_t                instance;        // the instance number of this WheelEncoder
        int32_t                distance_count;  // cumulative number of forward + backwards events received from wheel encoder
        float                  distance;        // total distance measured
        uint32_t               total_count;     // total number of successful readings from sensor (used for sensor quality calcs)
        uint32_t               error_count;     // total number of errors reading from sensor (used for sensor quality calcs)
        uint32_t               last_reading_ms; // time of last reading
    };

    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all sensors. Should be called from main loop
    void update(void);

    // return the number of wheel encoder sensor instances
    uint8_t num_sensors(void) const { return num_instances; }

    // return true if healthy
    bool healthy(uint8_t instance) const;

    // return true if the instance is enabled
    bool enabled(uint8_t instance) const;

    // get the position of the wheel associated with the wheel encoder
    Vector2f get_position(uint8_t instance) const;

    // get the total distance traveled in meters
    float get_distance(uint8_t instance) const;

    // get the total number of sensor reading from the encoder
    uint32_t get_total_count(uint8_t instance) const;

    // get the total number of errors reading from the encoder
    uint32_t get_error_count(uint8_t instance) const;

    // get the signal quality for a sensor (0 = extremely poor quality, 100 = extremely good quality)
    float get_signal_quality(uint8_t instance) const;

    // get the system time (in milliseconds) of the last update
    uint32_t get_last_reading_ms(uint8_t instance) const;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // parameters for each instance
    AP_Int8  _type[WHEELENCODER_MAX_INSTANCES];
    AP_Float _scaling[WHEELENCODER_MAX_INSTANCES];
    AP_Float _pos_x[WHEELENCODER_MAX_INSTANCES];
    AP_Float _pos_y[WHEELENCODER_MAX_INSTANCES];
    AP_Int8  _pina[WHEELENCODER_MAX_INSTANCES];
    AP_Int8  _pinb[WHEELENCODER_MAX_INSTANCES];

    WheelEncoder_State state[WHEELENCODER_MAX_INSTANCES];
    AP_WheelEncoder_Backend *drivers[WHEELENCODER_MAX_INSTANCES];
    uint8_t num_instances;
};
