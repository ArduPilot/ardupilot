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
  simple buzzer simulation class
*/

#pragma once

#include <AP_Param/AP_Param.h>
#include "SITL_Input.h"

#include "stdint.h"

namespace SITL {

class Buzzer {
public:
    Buzzer();

    // update buzzer state
    void update(const struct sitl_input &input);

    static const struct AP_Param::GroupInfo var_info[];

    bool is_enabled() const {return static_cast<bool>(_enable);}

 private:

    AP_Int8  _enable;  // enable buzzer sim
    AP_Int8  _pin;
    bool was_on;

    uint32_t on_time;

    bool prep_done;
};

}
