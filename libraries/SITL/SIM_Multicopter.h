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

#ifndef _SIM_MULTICOPTER_H
#define _SIM_MULTICOPTER_H

#include "SIM_Aircraft.h"

/*
  class to describe a motor position
 */
class Motor {
public:
    float angle;
    bool clockwise;
    uint8_t servo;

    Motor(float _angle, bool _clockwise, uint8_t _servo) :
        angle(_angle), // angle in degrees from front
        clockwise(_clockwise), // clockwise == true, anti-clockwise == false
        servo(_servo) // what servo output drives this motor
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
};

/*
  a multicopter simulator
 */
class MultiCopter : public Aircraft
{
public:
    MultiCopter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new MultiCopter(home_str, frame_str);
    }

private:
    const Frame *frame;
    float hover_throttle; // 0..1
    float terminal_velocity; // m/s

    const float terminal_rotation_rate;
    float thrust_scale;
};


#endif // _SIM_MULTICOPTER_H
