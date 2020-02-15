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
#define WHEELENCODER_CPR_DEFAULT        3200    // default encoder counts per full revolution of the wheel
#define WHEELENCODER_RADIUS_DEFAULT     0.05f   // default wheel radius of 5cm (0.05m)

class AP_WheelEncoder_Backend; 
 
class AP_WheelEncoder
{
public:
    friend class AP_WheelEncoder_Backend;
    friend class AP_WheelEncoder_Quadrature;
    friend class AP_WheelEncoder_SITL_Quadrature;

    AP_WheelEncoder(void);

    /* Do not allow copies */
    AP_WheelEncoder(const AP_WheelEncoder &other) = delete;
    AP_WheelEncoder &operator=(const AP_WheelEncoder&) = delete;

    // WheelEncoder driver types
    enum WheelEncoder_Type : uint8_t {
        WheelEncoder_TYPE_NONE             =   0,
        WheelEncoder_TYPE_QUADRATURE       =   1,
        WheelEncoder_TYPE_SITL_QUADRATURE  =  10,
    };

    // The WheelEncoder_State structure is filled in by the backend driver
    struct WheelEncoder_State {
        uint8_t                instance;        // the instance number of this WheelEncoder
        int32_t                distance_count;  // cumulative number of forward + backwards events received from wheel encoder
        float                  distance;        // total distance measured in meters
        uint32_t               total_count;     // total number of successful readings from sensor (used for sensor quality calcs)
        uint32_t               error_count;     // total number of errors reading from sensor (used for sensor quality calcs)
        uint32_t               last_reading_ms; // time of last reading
        int32_t                dist_count_change; // distance count change during the last update (used to calculating rate)
        uint32_t               dt_ms;             // time change (in milliseconds) for the previous period (used to calculating rate)
    };

    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all sensors. Should be called from main loop
    void update(void);

    // log data to logger
    void Log_Write();

    // return the number of wheel encoder sensor instances
    uint8_t num_sensors(void) const { return num_instances; }

    // return true if healthy
    bool healthy(uint8_t instance) const;

    // return true if the instance is enabled
    bool enabled(uint8_t instance) const;

    // get the counts per revolution of the encoder
    uint16_t get_counts_per_revolution(uint8_t instance) const;

    // get the wheel radius in meters
    float get_wheel_radius(uint8_t instance) const;

    // return a 3D vector defining the position offset of the center of the wheel in meters relative to the body frame origin
    const Vector3f &get_pos_offset(uint8_t instance) const;

    // get total delta angle (in radians) measured by the wheel encoder
    float get_delta_angle(uint8_t instance) const;

    // get the total distance traveled in meters
    float get_distance(uint8_t instance) const;

    // get the instantaneous rate in radians/second
    float get_rate(uint8_t instance) const;

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
    AP_Int16 _counts_per_revolution[WHEELENCODER_MAX_INSTANCES];
    AP_Float _wheel_radius[WHEELENCODER_MAX_INSTANCES];
    AP_Vector3f _pos_offset[WHEELENCODER_MAX_INSTANCES];
    AP_Int8  _pina[WHEELENCODER_MAX_INSTANCES];
    AP_Int8  _pinb[WHEELENCODER_MAX_INSTANCES];

    WheelEncoder_State state[WHEELENCODER_MAX_INSTANCES];
    AP_WheelEncoder_Backend *drivers[WHEELENCODER_MAX_INSTANCES];
    uint8_t num_instances;
    Vector3f pos_offset_zero;   // allows returning position offsets of zero for invalid requests
};
