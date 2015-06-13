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
  balloon simulator class
*/

#ifndef _SIM_BALLOON_H
#define _SIM_BALLOON_H

#include "SIM_Aircraft.h"

/*
  a balloon simulator
 */
class Balloon : public Aircraft
{
public:
    Balloon(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Balloon(home_str, frame_str);
    }

private:
    float terminal_rotation_rate = radians(100);
    float climb_rate = 20;
    float terminal_velocity = 40;
    float burst_altitude = 20000;
    bool burst = false;
    bool released = false;
};


#endif // _SIM_BALLOON_H
