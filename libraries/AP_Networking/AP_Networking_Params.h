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

class AP_Networking_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Networking_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_Params);

    // Feature types
    enum class Type : uint8_t {
        None                        = 0,
        SpeedTest                   = 1,
        LatencyTest                 = 2,
        Ping                        = 3,
    };

    AP_Enum<Type> type;                   // AP_Networking_Params::Type, 0=disabled, others see frontend enum TYPE
};

