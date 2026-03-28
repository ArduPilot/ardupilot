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
#include "AP_Doppler_Parameters.h"


const AP_Param::GroupInfo AP_Doppler_Parameters::var_info[] = {
   
    // @Param: OPTIONS
    // @DisplayName: Doppler Telemetry Options
    // @Description: A bitmask to set some Doppler Telemetry specific options
    // @Bitmask: 0:EnableAirspeedAndGroundspeed
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 1, AP_Doppler_Parameters, _options, 1),

    // @Param: MAV_EN
    // @DisplayName: DVL MAVLink upload enable
    // @Description: Enable custom DVL MAVLink message upload
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("MAV_EN", 2, AP_Doppler_Parameters, _mav_en, 0),

    // @Param: MAV_RATE
    // @DisplayName: DVL MAVLink upload rate
    // @Description: Unified DVL MAVLink upload rate in Hz
    // @Range: 1 50
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MAV_RATE", 3, AP_Doppler_Parameters, _mav_rate_hz, 5),

    AP_GROUPEND
};

AP_Doppler_Parameters::AP_Doppler_Parameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
