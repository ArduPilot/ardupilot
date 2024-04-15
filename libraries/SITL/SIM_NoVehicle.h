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
  empty vehicle for example CI testing
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  empty vehicle
 */
class NoVehicle : public Aircraft {
public:
    NoVehicle(const char *frame_str) : Aircraft(frame_str) {}

    void update(const struct sitl_input &input) override {}

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new NoVehicle(frame_str);
    }
};

} // namespace SITL
