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

#include "AP_Frsky_Telem.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_Frsky_Telem;

class AP_Frsky_Parameters
{
    friend class AP_Frsky_SPort_Passthrough;
public:
    AP_Frsky_Parameters();

    // parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    // settable parameters
    AP_Int8 _uplink_id;
    AP_Int8 _dnlink_id;
    AP_Int8 _dnlink1_id;
    AP_Int8 _dnlink2_id;
};

#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
