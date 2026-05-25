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

    // @Param: SIM_EN
    // @DisplayName: DVL simulation enable
    // @Description: Enable simulated DVL MAVLink cache updates for link bring-up without hardware
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SIM_EN", 4, AP_Doppler_Parameters, _sim_en, 0),

    // @Param: FUSEMODE
    // @DisplayName: DVL EKF fusion mode
    // @Description: Controls how DVL velocity is provided to the EKF
    // @Values: 0:Disabled,1:BodyOdometry,2:ExternalNavVelocity
    // @User: Advanced
    AP_GROUPINFO("FUSEMODE", 5, AP_Doppler_Parameters, _fusion_mode, (int8_t)AP_Doppler_Parameters::FusionMode::BodyOdom),

    // @Param: ORIENT
    // @DisplayName: DVL sensor orientation
    // @Description: DVL sensor orientation relative to the vehicle body frame
    // @Values: 0:Forward, 2:Right, 4:Back, 6:Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 6, AP_Doppler_Parameters, _orientation, ROTATION_NONE),

    // @Param: POS_X
    // @DisplayName: DVL X position offset
    // @Description: X position of the DVL in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_Y
    // @DisplayName: DVL Y position offset
    // @Description: Y position of the DVL in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_Z
    // @DisplayName: DVL Z position offset
    // @Description: Z position of the DVL in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS", 7, AP_Doppler_Parameters, _pos_offset, 0.0f),

    // @Param: DELAY_MS
    // @DisplayName: DVL sensor delay
    // @Description: DVL sensor delay relative to inertial measurements
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("DELAY_MS", 8, AP_Doppler_Parameters, _delay_ms, 10),

    // @Param: USEWTR
    // @DisplayName: DVL water-track fusion enable
    // @Description: Allows water-track velocity to be used when bottom-track velocity is unavailable. Bottom-track is still preferred.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USEWTR", 9, AP_Doppler_Parameters, _use_water_track, 0),

    // @Param: MIN_QUAL
    // @DisplayName: DVL minimum fusion quality
    // @Description: Minimum DVL quality required before velocity is fused by the EKF
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("MIN_QUAL", 10, AP_Doppler_Parameters, _min_quality, 50.0f),

    // @Param: SIMVEL_X
    // @DisplayName: DVL simulated X velocity
    // @Description: Simulated DVL velocity in the DVL sensor X axis before ORIENT correction. Positive X is forward when ORIENT is Forward.
    // @Units: m/s
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: SIMVEL_Y
    // @DisplayName: DVL simulated Y velocity
    // @Description: Simulated DVL velocity in the DVL sensor Y axis before ORIENT correction. Positive Y is right when ORIENT is Forward.
    // @Units: m/s
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: SIMVEL_Z
    // @DisplayName: DVL simulated Z velocity
    // @Description: Simulated DVL velocity in the DVL sensor Z axis before ORIENT correction. Positive Z is down when ORIENT is Forward.
    // @Units: m/s
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("SIMVEL", 11, AP_Doppler_Parameters, _sim_velocity, 0.0f),

    // @Param: SIM_ALT
    // @DisplayName: DVL simulated altitude
    // @Description: Simulated bottom-track altitude reported by the internal DVL simulator
    // @Units: m
    // @Range: 0.1 100
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SIM_ALT", 12, AP_Doppler_Parameters, _sim_altitude_m, 2.0f),

    // @Param: SIM_QUAL
    // @DisplayName: DVL simulated quality
    // @Description: Simulated DVL bottom-track quality passed to the EKF body odometry path
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("SIM_QUAL", 13, AP_Doppler_Parameters, _sim_quality, 100.0f),

    // @Param: DEBUG
    // @DisplayName: DVL debug logging enable
    // @Description: Enable DVL driver debug text messages for startup command retries and parsed EPD6 data frames
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 14, AP_Doppler_Parameters, _debug, 0),

    AP_GROUPEND
};

AP_Doppler_Parameters::AP_Doppler_Parameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
