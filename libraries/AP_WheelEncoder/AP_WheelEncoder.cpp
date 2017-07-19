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

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_WheelEncoder::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: WheelEncoder type
    // @Description: What type of WheelEncoder is connected
    // @Values: 0:None,1:Quadrature
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, AP_WheelEncoder, _type[0], 0),

    // @Param: _SCALING
    // @DisplayName: WheelEncoder scaling
    // @Description: Scaling factor between sensor reading and measured distance in millimeters
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_SCALING", 1, AP_WheelEncoder, _scaling[0], WHEELENCODER_SCALING_DEFAULT),

    // @Param: _POS_X
    // @DisplayName: WheelEncoder X position
    // @Description: X position of the first wheel encoder in body frame. Positive X is forward of the origin
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_POS_X",   2, AP_WheelEncoder, _pos_x[0], 0.0f),

    // @Param: _POS_Y
    // @DisplayName: WheelEncoder Y position
    // @Description: Y position of the first wheel encoder accelerometer in body frame. Positive Y is to the right of the origin
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_POS_Y",   3, AP_WheelEncoder, _pos_y[0], 0.0f),

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

    // @Param: 2_SCALING
    // @DisplayName: WheelEncoder scaling
    // @Description: Scaling factor between sensor reading and measured distance in millimeters
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("2_SCALING",7, AP_WheelEncoder, _scaling[1], WHEELENCODER_SCALING_DEFAULT),

    // @Param: 2_POS_X
    // @DisplayName: WheelEncoder X position
    // @Description: X position of the first wheel encoder in body frame. Positive X is forward of the origin
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("2_POS_X",  8, AP_WheelEncoder, _pos_x[1], 0.0f),

    // @Param: _POS_Y
    // @DisplayName: WheelEncoder Y position
    // @Description: Y position of the first wheel encoder accelerometer in body frame. Positive Y is to the right of the origin
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("2_POS_Y",  9, AP_WheelEncoder, _pos_y[1], 0.0f),

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

AP_WheelEncoder::AP_WheelEncoder(void) :
    num_instances(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state, 0, sizeof(state));
    memset(drivers, 0, sizeof(drivers));
}

// initialise the AP_WheelEncoder class.
void AP_WheelEncoder::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<WHEELENCODER_MAX_INSTANCES; i++) {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4  || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        uint8_t type = _type[num_instances];
        uint8_t instance = num_instances;

        if (type == WheelEncoder_TYPE_QUADRATURE) {
            state[instance].instance = instance;
            drivers[instance] = new AP_WheelEncoder_Quadrature(*this, instance, state[instance]);
        }
#endif

        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
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

// get the total distance travelled in meters
Vector2f AP_WheelEncoder::get_position(uint8_t instance) const
{
    // for invalid instances return zero vector
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return Vector2f(0.0f, 0.0f);
    }
    return Vector2f(_pos_x[instance],_pos_y[instance]);
}


// get the total distance traveled in meters
float AP_WheelEncoder::get_distance(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= WHEELENCODER_MAX_INSTANCES) {
        return 0.0f;
    }
    return _scaling[instance] * state[instance].distance_count * 0.001f;
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
