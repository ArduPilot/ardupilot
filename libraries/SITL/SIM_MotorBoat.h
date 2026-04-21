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
  MotorBoat simulator class
*/

#pragma once

#include "SIM_Sailboat.h"

namespace SITL {

/*
  a MotorBoat simulator. We re-use SailBoat but force motor_connected and sail_area to 0
 */
class MotorBoat : public Sailboat {
public:
    MotorBoat(const char *frame_str) : Sailboat(frame_str) {
        motor_connected = true;
        sail_area = 0.0;
    }

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW MotorBoat(frame_str);
    }
};

} // namespace SITL
