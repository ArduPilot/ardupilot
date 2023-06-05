/*
 * This program is free software: you can redistribute it and/or modify
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

#include "AP_SerialManager.h"

#if AP_SERIAL_EXTENSION_ENABLED && AP_NETWORKING_ENABLED
const AP_Param::GroupInfo AP_SerialManager::SerialExtState::IP_Params::var_info[] = {
    // @Param: PROTOCOL
    // @DisplayName: IP Channel protocol selection
    // @Description: Control what protocol to use on the USART. 
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Rangefinder, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 13:Beacon, 14:Volz servo out, 15:SBus servo out, 16:ESC Telemetry, 17:Devo Telemetry, 18:OpticalFlow, 19:RobotisServo, 20:NMEA Output, 21:WindVane, 22:SLCAN, 23:RCIN, 24:EFI Serial, 25:LTM, 26:RunCam, 27:HottTelem, 28:Scripting, 29:Crossfire VTX, 30:Generator, 31:Winch, 32:MSP, 33:DJI FPV, 34:AirSpeed, 35:ADSB, 36:AHRS, 37:SmartAudio, 38:FETtecOneWire, 39:Torqeedo, 40:AIS, 41:CoDevESC, 42:DisplayPort, 43:MAVLink High Latency, 44:IRC Tramp
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("PROTOCOL",  1, AP_SerialManager::SerialExtState::IP_Params, protocol, SerialProtocol_MAVLink2),

    // @Param: PORT
    // @DisplayName: IP Channel port
    // @Description: Port number to use for the IP channel
    // @Range: 1 65535
    // @User: Standard
    AP_GROUPINFO("PORT", 2, AP_SerialManager::SerialExtState::IP_Params, port, 14560),

    // @Param: DST_IP0
    // @DisplayName: Destination IP address 0
    // @Description: First byte of the destination IP address
    // @Range: 0 255
    AP_GROUPINFO("DST_IP0", 3,  AP_SerialManager::SerialExtState::IP_Params,  ip[0],   255),
  
    // @Param: DST_IP1
    // @DisplayName: Destination IP address 1
    // @Description: Second byte of the destination IP address
    // @Range: 0 255
    AP_GROUPINFO("DST_IP1", 4,  AP_SerialManager::SerialExtState::IP_Params,  ip[1],   255),
  
    // @Param: DST_IP2
    // @DisplayName: Destination IP address 2
    // @Description: Third byte of the destination IP address
    // @Range: 0 255
    AP_GROUPINFO("DST_IP2", 5,  AP_SerialManager::SerialExtState::IP_Params,  ip[2],   255),
  
    // @Param: DST_IP3
    // @DisplayName: Destination IP address 3
    // @Description: Fourth byte of the destination IP address
    // @Range: 0 255
    AP_GROUPINFO("DST_IP3", 6,  AP_SerialManager::SerialExtState::IP_Params,  ip[3],   255),

    // @Param: PASSTHR
    // @DisplayName: Serial port to pass through
    // @Description: Serial port to pass through
    // @Range: 0 255
    AP_GROUPINFO("PASSTHR", 7, AP_SerialManager::SerialExtState::IP_Params, passthru, 0),

    AP_GROUPEND
};
#endif
