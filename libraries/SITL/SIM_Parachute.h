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
  simple parachute simulation class
*/

#pragma once

#include "stdint.h"
#include <AP_Param/AP_Param.h>
#include "SITL_Input.h"
#include <AP_Math/AP_Math.h>

namespace SITL {

class Parachute {
public:
    Parachute() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // update parachute state
    void update(const struct sitl_input &input);

    Vector3f drag() const;

    static const struct AP_Param::GroupInfo var_info[];
    bool is_enabled() const {return static_cast<bool>(parachute_enable);}

 private:

    AP_Int8  parachute_enable;  // enable parachute sim
    AP_Int8  parachute_pin; // pin with pyrotechnics on

    const uint32_t report_interval = 1000000; // microseconds
    uint64_t last_report_us;

    uint32_t deployed_ms; // time parachute was deployed

    uint64_t last_update_us;

    bool should_report();
    bool zero_report_done = false;
};

}
