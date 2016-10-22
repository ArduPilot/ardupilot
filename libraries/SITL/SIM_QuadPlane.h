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
  simple plane simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Plane.h"
#include "SIM_Multicopter.h"

namespace SITL {

/*
  a very simple quadplane simulator
 */
class QuadPlane : public Plane {
public:
    QuadPlane(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new QuadPlane(home_str, frame_str);
    }
private:
    Frame *frame;
};

} // namespace SITL
