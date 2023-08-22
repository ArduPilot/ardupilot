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

#if HAL_NUM_CAN_IFACES && AP_SERIAL_EXTENSION_ENABLED
const AP_Param::GroupInfo AP_SerialManager::SerialExtState::DroneCAN_Params::var_info[] = {
    // @Param: PROTOCOL
    // @DisplayName: DroneCAN Channel protocol selection
    // @Description: Control what protocol to use on the USART. 
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Rangefinder, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 13:Beacon, 14:Volz servo out, 15:SBus servo out, 16:ESC Telemetry, 17:Devo Telemetry, 18:OpticalFlow, 19:RobotisServo, 20:NMEA Output, 21:WindVane, 22:SLCAN, 23:RCIN, 24:EFI Serial, 25:LTM, 26:RunCam, 27:HottTelem, 28:Scripting, 29:Crossfire VTX, 30:Generator, 31:Winch, 32:MSP, 33:DJI FPV, 34:AirSpeed, 35:ADSB, 36:AHRS, 37:SmartAudio, 38:FETtecOneWire, 39:Torqeedo, 40:AIS, 41:CoDevESC, 42:DisplayPort, 43:MAVLink High Latency, 44:IRC Tramp
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("PROTOCOL",  1, AP_SerialManager::SerialExtState::DroneCAN_Params, protocol, SerialProtocol_Passthru),

    // @Param: BAUD
    // @DisplayName: USART baud rate
    // @Description: The baud rate used on the USART port. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("BAUD",  2, AP_SerialManager::SerialExtState::DroneCAN_Params, baud, 115),

    // @Param: CHAN_ID
    // @DisplayName: Channel ID
    // @Description: The baud rate used on the USART port. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("CHAN_ID",  3, AP_SerialManager::SerialExtState::DroneCAN_Params, chan_id, 0),

    // @Param: OPTIONS
    // @DisplayName: USART options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:SwapTXRX, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OPTIONS",  4, AP_SerialManager::SerialExtState::DroneCAN_Params, options, 0),

    // @Param: BUF_US
    // @DisplayName: UAVCAN Tunnel Buffer Time
    // @Description: This is the buffer time for which the message is in the buffer before forwarding to the physical port. This allows data frames to be fully read before sending.
    // @Range: 0 1000000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BUF_US", 5, AP_SerialManager::SerialExtState::DroneCAN_Params, buffer_us, 1000),

    AP_GROUPEND
};
#endif
