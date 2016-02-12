/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
/*
  multicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  class to describe a motor position
 */
class Motor {
public:
    float angle;
    float yaw_factor;
    uint8_t servo;
    uint8_t display_order;

    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order) :
        servo(_servo), // what servo output drives this motor
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        display_order(_display_order) // order for clockwise display
    {}
};

/*
  class to describe a multicopter frame type
 */
class Frame {
public:
    const char *name;
    uint8_t num_motors;
    const Motor *motors;

    Frame(const char *_name,
          uint8_t _num_motors,
          const Motor *_motors) :
        name(_name),
        num_motors(_num_motors),
        motors(_motors) {}


    // find a frame by name
    static Frame *find_frame(const char *name);
    
    // initialise frame
    void init(float mass, float hover_throttle, float terminal_velocity, float terminal_rotation_rate);

    // calculate rotational and linear accelerations
    void calculate_forces(const Aircraft &aircraft,
                          const Aircraft::sitl_input &input,
                          Vector3f &rot_accel, Vector3f &body_accel);
    
    float terminal_velocity;
    float terminal_rotation_rate;
    float thrust_scale;
    float mass;
    uint8_t motor_offset;
};

/*
  a multicopter simulator
 */
class MultiCopter : public Aircraft {
public:
    MultiCopter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new MultiCopter(home_str, frame_str);
    }

protected:
    // calculate rotational and linear accelerations
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    Frame *frame;
};

} // namespace SITL
