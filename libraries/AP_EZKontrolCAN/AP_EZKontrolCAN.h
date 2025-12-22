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

#pragma once

#include <AP_Param/AP_Param.h>

namespace AP_HAL {
    class CANIface;
}

class AP_EZKontrolCAN {
public:
    AP_EZKontrolCAN();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_EZKontrolCAN);

    static AP_EZKontrolCAN *get_singleton();

    void init();
    void set_targets(float left_norm, float right_norm, bool armed);
    void update();

    bool enabled() const;
    bool healthy() const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _enable;
    AP_Int8 _can_port;
    AP_Int16 _bitrate;
    AP_Int16 _addr_left;
    AP_Int16 _addr_right;
    AP_Int8 _mode;
    AP_Int16 _max_rpm;
    AP_Float _max_curr_a;
    AP_Int8 _debug;

    AP_HAL::CANIface *_can_iface;
    bool _can_inited;
    bool _healthy;

    float _left_target;
    float _right_target;
    bool _armed;
    uint32_t _last_target_ms;
    uint32_t _last_update_ms;
    uint32_t _last_debug_ms;

    static AP_EZKontrolCAN *_singleton;
};

namespace AP {
    AP_EZKontrolCAN *ezkontrol_can();
};
