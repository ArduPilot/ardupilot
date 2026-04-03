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
  simple camera simulation class
*/

#pragma once

#include "stdint.h"
#include <AP_Param/AP_Param.h>
#include "SITL_Input.h"

namespace SITL {

class Camera {
public:
    Camera() {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // update camera state
    void update(const struct sitl_input &input);

    static const struct AP_Param::GroupInfo var_info[];

    bool is_enabled() const { return static_cast<bool>(_enable); }
    uint32_t image_count() const { return _image_count; }

private:
    AP_Int8 _enable;
    AP_Int8 _trigger_servo_pin;
    AP_Int8 _trigger_gpio_pin;
    AP_Int16 _trigger_pwm;

    bool _last_trigger_state;
    uint32_t _image_count;
};

}
