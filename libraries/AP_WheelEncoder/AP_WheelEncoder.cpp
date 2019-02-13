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

#include "AP_WheelEncoder.h"
#include "WheelEncoder_Quadrature.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_WheelEncoder::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: WheelEncoder type
    // @Description: What type of WheelEncoder is connected
    // @Values: 0:None,1:Quadrature
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_WheelEncoder, _type[0], 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _CPR
    // @DisplayName: WheelEncoder counts per revolution
    // @Description: WheelEncoder counts per full revolution of the wheel
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_CPR",     1, AP_WheelEncoder, _counts_per_revolution[0], WHEELENCODER_CPR_DEFAULT),

    // @Param: _RADIUS
    // @DisplayName: Wheel radius
    // @Description: Wheel radius
    // @Units: m
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_RADIUS",  2, AP_WheelEncoder, _wheel_radius[0], WHEELENCODER_RADIUS_DEFAULT),

    // @Param: _POS_X
    // @DisplayName: Wheel's X position offset
    // @Description: X position of the center of the wheel in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Increment: 0.01
    // @User: Standard

    // @Param: _POS_Y
    // @DisplayName: Wheel's Y position offset
    // @Description: Y position of the center of the wheel in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Increment: 0.01
    // @User: Standard

    // @Param: _POS_Z
    // @DisplayName: Wheel's Z position offset
    // @Description: Z position of the center of the wheel in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_POS",     3, AP_WheelEncoder, _pos_offset[0], 0.0f),

    // @Param: _PINA
    // @DisplayName: Input Pin A
    // @Description: Input Pin A
    // @Values: -1:Disabled,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6
    // @User: Standard
    AP_GROUPINFO("_PINA",    4, AP_WheelEncoder, _pina[0], 55),

    // @Param: _PINB
    // @DisplayName: Input Pin B
    // @Description: Input Pin B
    // @Values: -1:Disabled,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6
    // @User: Standard
    AP_GROUPINFO("_PINB",    5, AP_WheelEncoder, _pinb[0], 54),

#if WHEELENCODER_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second WheelEncoder type
    // @Description: What type of WheelEncoder sensor is connected
    // @Values: 0:None,1:Quadrature
    // @User: Standard
    AP_GROUPINFO("2_TYPE",   6, AP_WheelEncoder, _type[1], 0),

    // @Param: 2_CPR
    // @DisplayName: WheelEncoder 2 counts per revolution
    // @Description: WheelEncoder 2 counts per full revolution of the wheel
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_CPR",     7, AP_WheelEncoder, _counts_per_revolution[1], WHEELENCODER_CPR_DEFAULT),

    // @Param: 2_RADIUS
    // @DisplayName: Wheel2's radius
    // @Description: Wheel2's radius
    // @Units: m
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("2_RADIUS", 8, AP_WheelEncoder, _wheel_radius[1], WHEELENCODER_RADIUS_DEFAULT),

    // @Param: 2_POS_X
    // @DisplayName: Wheel2's X position offset
    // @Description: X position of the center of the second wheel in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Increment: 0.01
    // @User: Standard

    // @Param: 2_POS_Y
    // @DisplayName: Wheel2's Y position offset
    // @Description: Y position of the center of the second wheel in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Increment: 0.01
    // @User: Standard

    // @Param: 2_POS_Z
    // @DisplayName: Wheel2's Z position offset
    // @Description: Z position of the center of the second wheel in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("2_POS",    9, AP_WheelEncoder, _pos_offset[1], 0.0f),

    // @Param: 2_PINA
    // @DisplayName: Second Encoder Input Pin A
    // @Description: Second Encoder Input Pin A
    // @Values: -1:Disabled,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6
    // @User: Standard
    AP_GROUPINFO("2_PINA",   10, AP_WheelEncoder, _pina[1], 53),

    // @Param: 2_PINB
    // @DisplayName: Second Encoder Input Pin B
    // @Description: Second Encoder Input Pin B
    // @Values: -1:Disabled,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6
    // @User: Standard
    AP_GROUPINFO("2_PINB",   11, AP_WheelEncoder, _pinb[1], 52),
#endif

    AP_GROUPEND
};

AP_WheelEncoder::AP_WheelEncoder(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_WheelEncoder class.
void AP_WheelEncoder::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<WHEELENCODER_MAX_INSTANCES; i++) {
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        switch ((WheelEncoder_Type)_type[i].get()) {
        case WheelEncoder_TYPE_QUADRATURE:
            drivers[i] = new AP_WheelEncoder_Quadrature(*this, i, state[i]);
            break;
        case WheelEncoder_TYPE_NONE:
            break;
        }
#endif

        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;  // num_instances is a high-water-mark
        }
    }
}

// update WheelEncoder state for all instances. This should be called by main loop
void AP_WheelEncoder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr && _type[i] != WheelEncoder_TYPE_NONE) {
            drivers[i]->update();
        }
    }
}

// log wheel encoder information
void AP_WheelEncoder::Log_Write()
{
    // return immediately if no wheel encoders are enabled
    if (!enabled(0) && !enabled(1)) {
        return;
    }

    struct log_WheelEncoder pkt = {
        LOG_PACKET_HEADER_INIT(LOG_WHEELENCODER_MSG),
        time_us     : AP_HAL::micros64(),
        distance_0  : get_distance(0),
        quality_0   : (uint8_t)get_signal_quality(0),
        distance_1  : get_distance(1),
        quality_1   : (uint8_t)get_signal_quality(1),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// check if an instance is healthy
bool AP_WheelEncoder::healthy(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    return true;
}

// check if an instance is activated
bool AP_WheelEncoder::enabled(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    // if no sensor type is selected, the sensor is not activated.
    if (_type[instance] == WheelEncoder_TYPE_NONE) {
        return false;
    }
    return true;
}

// get the counts per revolution of the encoder
uint16_t AP_WheelEncoder::get_counts_per_revolution(uint8_t instance) const
{
    // for invalid instances return zero vector
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0;
    }
    return (uint16_t)_counts_per_revolution[instance];
}

// get the wheel radius in meters
float AP_WheelEncoder::get_wheel_radius(uint8_t instance) const
{
    // for invalid instances return zero vector
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0.0f;
    }
    return _wheel_radius[instance];
}

// return a 3D vector defining the position offset of the center of the wheel in meters relative to the body frame origin
const Vector3f &AP_WheelEncoder::get_pos_offset(uint8_t instance) const
{
    // for invalid instances return zero vector
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return pos_offset_zero;
    }
    return _pos_offset[instance];
}

// get total delta angle (in radians) measured by the wheel encoder
float AP_WheelEncoder::get_delta_angle(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0.0f;
    }
    // protect against divide by zero
    if (_counts_per_revolution[instance] == 0) {
        return 0.0f;
    }
    return M_2PI * state[instance].distance_count / _counts_per_revolution[instance];
}

// get the total distance traveled in meters
float AP_WheelEncoder::get_distance(uint8_t instance) const
{
    // for invalid instances return zero
    return get_delta_angle(instance) * _wheel_radius[instance];
}

// get the instantaneous rate in radians/second
float AP_WheelEncoder::get_rate(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0.0f;
    }

    // protect against divide by zero
    if ((state[instance].dt_ms == 0) || _counts_per_revolution[instance] == 0) {
        return 0;
    }

    // calculate delta_angle (in radians) per second
    return M_2PI * (state[instance].dist_count_change / ((float)_counts_per_revolution[instance])) / (state[instance].dt_ms * 1e-3f);
}

// get the total number of sensor reading from the encoder
uint32_t AP_WheelEncoder::get_total_count(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0;
    }
    return state[instance].total_count;
}

// get the total distance traveled in meters
uint32_t AP_WheelEncoder::get_error_count(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0;
    }
    return state[instance].error_count;
}

// get the signal quality for a sensor
float AP_WheelEncoder::get_signal_quality(uint8_t instance) const
{
    // protect against divide by zero
    if (state[instance].total_count == 0) {
        return 0.0f;
    }
    return constrain_float((1.0f - ((float)state[instance].error_count / (float)state[instance].total_count)) * 100.0f, 0.0f, 100.0f);
}

// get the system time (in milliseconds) of the last update
uint32_t AP_WheelEncoder::get_last_reading_ms(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0;
    }
    return state[instance].last_reading_ms;
}
