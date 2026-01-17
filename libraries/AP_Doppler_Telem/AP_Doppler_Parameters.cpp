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
    // @Param: ENABLE
    // @DisplayName: Doppler velocity fusion enable
    // @Description: Enable Doppler velocity fusion path.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_Doppler_Parameters, _enable, 0),

    // @Param: Q_MIN
    // @DisplayName: Doppler quality minimum
    // @Description: Minimum quality required to fuse Doppler velocity.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("Q_MIN", 2, AP_Doppler_Parameters, _q_min, 100),

    // @Param: VEL_ERR
    // @DisplayName: Doppler bottom-lock velocity error
    // @Description: 1-sigma velocity error for bottom-lock fusion.
    // @Units: m/s
    // @Range: 0.01 5.0
    // @User: Standard
    AP_GROUPINFO("VEL_ERR", 3, AP_Doppler_Parameters, _vel_err, 0.1f),

    // @Param: VEL_ERR_WTR
    // @DisplayName: Doppler water-track velocity error
    // @Description: 1-sigma velocity error for water-track fusion. Set to 0 to disable fusion.
    // @Units: m/s
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("VEL_ERR_WTR", 4, AP_Doppler_Parameters, _vel_err_wtr, 0.0f),

    // @Param: _POS_X
    // @DisplayName: Doppler sensor X position offset
    // @Description: Doppler sensor X position offset in body frame.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Doppler sensor Y position offset
    // @Description: Doppler sensor Y position offset in body frame.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Doppler sensor Z position offset
    // @Description: Doppler sensor Z position offset in body frame.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced
    AP_GROUPINFO("_POS", 5, AP_Doppler_Parameters, _pos_offset, 0.0f),


    // @Param: OPTIONS
    // @DisplayName: Doppler Telemetry Options
    // @Description: A bitmask to set some Doppler Telemetry specific options
    // @Bitmask: 0:EnableAirspeedAndGroundspeed
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 6, AP_Doppler_Parameters, _options, 0),
    AP_GROUPEND
};

AP_Doppler_Parameters::AP_Doppler_Parameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}

