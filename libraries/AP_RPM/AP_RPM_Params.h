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

class AP_RPM_Params {

public:
    // Constructor
    AP_RPM_Params(void);

    // parameters for each instance
    AP_Int8  type;
    AP_Int8  pin;
    AP_Float scaling;
    AP_Float maximum;
    AP_Float minimum;
    AP_Float quality_min;
    AP_Int32 esc_mask;

    static const struct AP_Param::GroupInfo var_info[];

};
