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
  simple sprayer simulation class
*/

#pragma once

#include "stdint.h"
#include <AP_Param/AP_Param.h>
#include "SITL_Input.h"

namespace SITL {

class Sprayer {
public:
    Sprayer() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // update sprayer state
    void update(const struct sitl_input &input);

    float payload_mass() const { return static_cast<float>(capacity); }; // kg; water, so kg=l

    static const struct AP_Param::GroupInfo var_info[];
    bool is_enabled() const {return static_cast<bool>(sprayer_enable);}

 private:

    AP_Int8  sprayer_enable;  // enable sprayer sim
    AP_Int8  sprayer_pump_pin;
    AP_Int8  sprayer_spin_pin;

    const uint32_t report_interval = 1000000; // microseconds
    uint64_t last_report_us;

    const float pump_max_rate = 0.01f; // litres/second
    const float pump_slew_rate = 20.0f; // percent/second
    float last_pump_output; // percentage

    const float spinner_max_rate = 3600.0f; // degrees/second
    const float spinner_slew_rate = 20.0f; // percent/second
    float last_spinner_output; // percentage

    double capacity = 0.25; // litres

    uint64_t last_update_us;

    bool should_report();
    bool zero_report_done = false;
};

}
