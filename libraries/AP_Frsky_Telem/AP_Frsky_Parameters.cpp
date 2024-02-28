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
#include "AP_Frsky_Parameters.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

const AP_Param::GroupInfo AP_Frsky_Parameters::var_info[] = {
    // @Param: UPLINK_ID
    // @DisplayName: Uplink sensor id
    // @Description: Change the uplink sensor id (SPort only)
    // @Values: -1:Disable,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18,19:19,20:20,21:21,22:22,23:23,24:24,25:25,26:26
    // @User: Advanced
    AP_GROUPINFO("UPLINK_ID",  1, AP_Frsky_Parameters, _uplink_id, 13),

    // @Param: DNLINK1_ID
    // @DisplayName: First downlink sensor id
    // @Description: Change the first extra downlink sensor id (SPort only)
    // @Values: -1:Disable,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18,19:19,20:20,21:21,22:22,23:23,24:24,25:25,26:26
    // @User: Advanced
    AP_GROUPINFO("DNLINK1_ID",  2, AP_Frsky_Parameters, _dnlink1_id, 20),

    // @Param: DNLINK2_ID
    // @DisplayName: Second downlink sensor id
    // @Description: Change the second extra downlink sensor id (SPort only)
    // @Values: -1:Disable,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18,19:19,20:20,21:21,22:22,23:23,24:24,25:25,26:26
    // @User: Advanced
    AP_GROUPINFO("DNLINK2_ID",  3, AP_Frsky_Parameters, _dnlink2_id, 7),

    // @Param: DNLINK_ID
    // @DisplayName: Default downlink sensor id
    // @Description: Change the default downlink sensor id (SPort only)
    // @Values: -1:Disable,7:7,8:8,9:9,10:10,11:11,12:12,13:13,14:14,15:15,16:16,17:17,18:18,19:19,20:20,21:21,22:22,23:23,24:24,25:25,26:26,27:27
    // @User: Advanced
    AP_GROUPINFO("DNLINK_ID",  4, AP_Frsky_Parameters, _dnlink_id, 27),

    // @Param: OPTIONS
    // @DisplayName: FRSky Telemetry Options
    // @Description: A bitmask to set some FRSky Telemetry specific options
    // @Bitmask: 0:EnableAirspeedAndGroundspeed
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 5, AP_Frsky_Parameters, _options, 0),
    AP_GROUPEND
};

AP_Frsky_Parameters::AP_Frsky_Parameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
