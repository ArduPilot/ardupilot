
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

#include "AP_GPS_config.h"

#if AP_GPS_ENABLED

#include "AP_GPS.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_GPS::Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: GPS type
    // @Description: GPS type
    // @Values: 0:None,1:AUTO,2:uBlox,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:DroneCAN,10:SBF,11:GSOF,13:ERB,14:MAV,15:NOVA,16:HemisphereNMEA,17:uBlox-MovingBaseline-Base,18:uBlox-MovingBaseline-Rover,19:MSP,20:AllyStar,21:ExternalAHRS,22:DroneCAN-MovingBaseline-Base,23:DroneCAN-MovingBaseline-Rover,24:UnicoreNMEA,25:UnicoreMovingBaselineNMEA,26:SBF-DualAntenna
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("TYPE",    1, AP_GPS::Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: GNSS_MODE
    // @DisplayName: GNSS system configuration
    // @Description: Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured)
    // @Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLONASS
    // @User: Advanced
    AP_GROUPINFO("GNSS_MODE", 2, AP_GPS::Params, gnss_mode, 0),

    // @Param: RATE_MS
    // @DisplayName: GPS update rate in milliseconds
    // @Description: Controls how often the GPS should provide a position update. Lowering below 5Hz(default) is not allowed. Raising the rate above 5Hz usually provides little benefit and for some GPS (eg Ublox M9N) can severely impact performance.
    // @Units: ms
    // @Values: 100:10Hz,125:8Hz,200:5Hz
    // @Range: 50 200
    // @User: Advanced
    AP_GROUPINFO("RATE_MS", 3, AP_GPS::Params, rate_ms, 200),

    // @Param: POS_X
    // @DisplayName: Antenna X position offset
    // @Description: X position of the first GPS antenna in body frame. Positive X is forward of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_Y
    // @DisplayName: Antenna Y position offset
    // @Description: Y position of the first GPS antenna in body frame. Positive Y is to the right of the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_Z
    // @DisplayName: Antenna Z position offset
    // @Description: Z position of the first GPS antenna in body frame. Positive Z is down from the origin. Use antenna phase centroid location if provided by the manufacturer.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS", 4, AP_GPS::Params, antenna_offset, 0.0f),

    // @Param: DELAY_MS
    // @DisplayName: GPS delay in milliseconds
    // @Description: Controls the amount of GPS  measurement delay that the autopilot compensates for. Set to zero to use the default delay for the detected GPS type.
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("DELAY_MS", 5, AP_GPS::Params, delay_ms, 0),

#if AP_GPS_SBF_ENABLED
    // @Param: COM_PORT
    // @DisplayName: GPS physical COM port
    // @Description: The physical COM port on the connected device, currently only applies to SBF and GSOF GPS
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @Values: 0:COM1(RS232) on GSOF, 1:COM2(TTL) on GSOF
    // @RebootRequired: True
    AP_GROUPINFO("COM_PORT", 6, AP_GPS::Params, com_port, HAL_GPS_COM_PORT_DEFAULT),
#endif

#if GPS_MOVING_BASELINE
    // @Group: MB_
    // @Path: MovingBase.cpp
    AP_SUBGROUPINFO(mb_params, "MB_", 7, AP_GPS::Params, MovingBase),
#endif

#if HAL_ENABLE_DRONECAN_DRIVERS
    // @Param: CAN_NODEID
    // @DisplayName: Detected CAN Node ID for GPS
    // @Description: GPS Node id for GPS.  Detected node unless CAN_OVRIDE is set
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("CAN_NODEID", 8, AP_GPS::Params, node_id, 0),

    // @Param: CAN_OVRIDE
    // @DisplayName: DroneCAN GPS NODE ID
    // @Description: GPS Node id for GPS. If 0 the gps will be automatically selected on a first-come-first-GPS basis.
    // @User: Advanced
    AP_GROUPINFO("CAN_OVRIDE", 9, AP_GPS::Params, override_node_id, 0),
#endif

    AP_GROUPEND
};

AP_GPS::Params::Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // AP_GPS_ENABLED
