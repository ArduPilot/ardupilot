/*
   Please contribute your ideas! See https://ardupilot.org/dev for details

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
/*
  SerialManager allows defining the protocol and baud rates for the available
  serial ports and provides helper functions so objects (like a gimbal) can
  find which serial port they should use
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_MSP/AP_MSP.h>
#include "AP_SerialManager.h"

extern const AP_HAL::HAL& hal;

#ifdef HAL_SERIAL2_PROTOCOL
#define SERIAL2_PROTOCOL HAL_SERIAL2_PROTOCOL
#else
#define SERIAL2_PROTOCOL SerialProtocol_MAVLink2
#endif

#ifndef HAL_SERIAL3_PROTOCOL
#define SERIAL3_PROTOCOL SerialProtocol_GPS
#else
#define SERIAL3_PROTOCOL HAL_SERIAL3_PROTOCOL
#endif

#ifndef HAL_SERIAL4_PROTOCOL
#define SERIAL4_PROTOCOL SerialProtocol_GPS
#else
#define SERIAL4_PROTOCOL HAL_SERIAL4_PROTOCOL
#endif

#ifdef HAL_SERIAL5_PROTOCOL
#define SERIAL5_PROTOCOL HAL_SERIAL5_PROTOCOL
#define SERIAL5_BAUD HAL_SERIAL5_BAUD
#else
#define SERIAL5_PROTOCOL SerialProtocol_None
#define SERIAL5_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000
#endif

#ifndef HAL_SERIAL6_PROTOCOL
#define SERIAL6_PROTOCOL SerialProtocol_None
#define SERIAL6_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000
#else
#define SERIAL6_PROTOCOL HAL_SERIAL6_PROTOCOL
#define SERIAL6_BAUD HAL_SERIAL6_BAUD
#endif

#ifndef HAL_SERIAL7_PROTOCOL
#define SERIAL7_PROTOCOL SerialProtocol_None
#define SERIAL7_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000
#else
#define SERIAL7_PROTOCOL HAL_SERIAL7_PROTOCOL
#define SERIAL7_BAUD HAL_SERIAL7_BAUD
#endif

#ifndef HAL_SERIAL8_PROTOCOL
#define SERIAL8_PROTOCOL SerialProtocol_None
#define SERIAL8_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000
#else
#define SERIAL8_PROTOCOL HAL_SERIAL8_PROTOCOL
#define SERIAL8_BAUD HAL_SERIAL8_BAUD
#endif

#ifndef HAL_SERIAL9_PROTOCOL
#define SERIAL9_PROTOCOL SerialProtocol_None
#define SERIAL9_BAUD AP_SERIALMANAGER_MAVLINK_BAUD/1000
#else
#define SERIAL9_PROTOCOL HAL_SERIAL9_PROTOCOL
#define SERIAL9_BAUD HAL_SERIAL9_BAUD
#endif

#ifdef HAL_BUILD_AP_PERIPH
/*
  AP_Periph doesn't include the SERIAL parameter tree, instead each
  supported serial device type has it's own parameter within AP_Periph
  for which port is used.
 */
#undef SERIAL0_PROTOCOL
#undef SERIAL1_PROTOCOL
#undef SERIAL2_PROTOCOL
#undef SERIAL3_PROTOCOL
#undef SERIAL4_PROTOCOL
#undef SERIAL5_PROTOCOL
#undef SERIAL6_PROTOCOL
#undef SERIAL7_PROTOCOL
#undef SERIAL8_PROTOCOL
#undef SERIAL9_PROTOCOL
#define SERIAL0_PROTOCOL SerialProtocol_None
#define SERIAL1_PROTOCOL SerialProtocol_None
#define SERIAL2_PROTOCOL SerialProtocol_None
#define SERIAL3_PROTOCOL SerialProtocol_None
#define SERIAL4_PROTOCOL SerialProtocol_None
#define SERIAL5_PROTOCOL SerialProtocol_None
#define SERIAL6_PROTOCOL SerialProtocol_None
#define SERIAL7_PROTOCOL SerialProtocol_None
#define SERIAL8_PROTOCOL SerialProtocol_None
#define SERIAL9_PROTOCOL SerialProtocol_None
#endif // HAL_BUILD_AP_PERIPH

const AP_Param::GroupInfo AP_SerialManager::var_info[] = {
#if SERIALMANAGER_NUM_PORTS > 0
    // @Param: 0_BAUD
    // @DisplayName: Serial0 baud rate
    // @Description: The baud rate used on the USB console. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("0_BAUD",  0, AP_SerialManager, state[0].baud, AP_SERIALMANAGER_CONSOLE_BAUD/1000),

    // @Param: 0_PROTOCOL
    // @DisplayName: Console protocol selection
    // @Description: Control what protocol to use on the console. 
    // @Values: 1:MAVlink1, 2:MAVLink2
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("0_PROTOCOL",  11, AP_SerialManager, state[0].protocol, SerialProtocol_MAVLink2),
#endif

#if SERIALMANAGER_NUM_PORTS > 1
    // @Param: 1_PROTOCOL
    // @DisplayName: Telem1 protocol selection
    // @Description: Control what protocol to use on the Telem1 port. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @Values: -1:None, 1:MAVLink1, 2:MAVLink2, 3:Frsky D, 4:Frsky SPort, 5:GPS, 7:Alexmos Gimbal Serial, 8:SToRM32 Gimbal Serial, 9:Rangefinder, 10:FrSky SPort Passthrough (OpenTX), 11:Lidar360, 13:Beacon, 14:Volz servo out, 15:SBus servo out, 16:ESC Telemetry, 17:Devo Telemetry, 18:OpticalFlow, 19:RobotisServo, 20:NMEA Output, 21:WindVane, 22:SLCAN, 23:RCIN, 24:EFI Serial, 25:LTM, 26:RunCam, 27:HottTelem, 28:Scripting, 29:Crossfire VTX, 30:Generator, 31:Winch, 32:MSP, 33:DJI FPV, 34:AirSpeed, 35:ADSB, 36:AHRS, 37:SmartAudio, 38:FETtecOneWire, 39:Torqeedo, 40:AIS, 41:CoDevESC, 42:DisplayPort, 43:MAVLink High Latency, 44:IRC Tramp
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("1_PROTOCOL",  1, AP_SerialManager, state[1].protocol, SerialProtocol_MAVLink2),

    // @Param: 1_BAUD
    // @DisplayName: Telem1 Baud Rate
    // @Description: The baud rate used on the Telem1 port. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("1_BAUD", 2, AP_SerialManager, state[1].baud, AP_SERIALMANAGER_MAVLINK_BAUD/1000),
#endif

#if SERIALMANAGER_NUM_PORTS > 2
    // @Param: 2_PROTOCOL
    // @DisplayName: Telemetry 2 protocol selection
    // @Description: Control what protocol to use on the Telem2 port. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @CopyValuesFrom: SERIAL1_PROTOCOL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("2_PROTOCOL",  3, AP_SerialManager, state[2].protocol, SERIAL2_PROTOCOL),

    // @Param: 2_BAUD
    // @DisplayName: Telemetry 2 Baud Rate
    // @Description: The baud rate of the Telem2 port. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @CopyValuesFrom: SERIAL1_BAUD
    // @User: Standard
    AP_GROUPINFO("2_BAUD", 4, AP_SerialManager, state[2].baud, AP_SERIALMANAGER_MAVLINK_BAUD/1000),
#endif

#if SERIALMANAGER_NUM_PORTS > 3
    // @Param: 3_PROTOCOL
    // @DisplayName: Serial 3 (GPS) protocol selection
    // @Description: Control what protocol Serial 3 (GPS) should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @CopyValuesFrom: SERIAL1_PROTOCOL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("3_PROTOCOL",  5, AP_SerialManager, state[3].protocol, SERIAL3_PROTOCOL),

    // @Param: 3_BAUD
    // @DisplayName: Serial 3 (GPS) Baud Rate
    // @Description: The baud rate used for the Serial 3 (GPS). Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @CopyValuesFrom: SERIAL1_BAUD
    // @User: Standard
    AP_GROUPINFO("3_BAUD", 6, AP_SerialManager, state[3].baud, AP_SERIALMANAGER_GPS_BAUD/1000),
#endif

#if SERIALMANAGER_NUM_PORTS > 4
    // @Param: 4_PROTOCOL
    // @DisplayName: Serial4 protocol selection
    // @Description: Control what protocol Serial4 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @CopyValuesFrom: SERIAL1_PROTOCOL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("4_PROTOCOL",  7, AP_SerialManager, state[4].protocol, SERIAL4_PROTOCOL),

    // @Param: 4_BAUD
    // @DisplayName: Serial 4 Baud Rate
    // @Description: The baud rate used for Serial4. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @CopyValuesFrom: SERIAL1_BAUD
    // @User: Standard
    AP_GROUPINFO("4_BAUD", 8, AP_SerialManager, state[4].baud, AP_SERIALMANAGER_GPS_BAUD/1000),
#endif

#if SERIALMANAGER_NUM_PORTS > 5
    // @Param: 5_PROTOCOL
    // @DisplayName: Serial5 protocol selection
    // @Description: Control what protocol Serial5 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @CopyValuesFrom: SERIAL1_PROTOCOL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("5_PROTOCOL",  9, AP_SerialManager, state[5].protocol, SERIAL5_PROTOCOL),

    // @Param: 5_BAUD
    // @DisplayName: Serial 5 Baud Rate
    // @Description: The baud rate used for Serial5. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @CopyValuesFrom: SERIAL1_BAUD
    // @User: Standard
    AP_GROUPINFO("5_BAUD", 10, AP_SerialManager, state[5].baud, SERIAL5_BAUD),
#endif

    // index 11 used by 0_PROTOCOL
        
#if SERIALMANAGER_NUM_PORTS > 6
    // @Param: 6_PROTOCOL
    // @DisplayName: Serial6 protocol selection
    // @Description: Control what protocol Serial6 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @CopyValuesFrom: SERIAL1_PROTOCOL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("6_PROTOCOL",  12, AP_SerialManager, state[6].protocol, SERIAL6_PROTOCOL),

    // @Param: 6_BAUD
    // @DisplayName: Serial 6 Baud Rate
    // @Description: The baud rate used for Serial6. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @CopyValuesFrom: SERIAL1_BAUD
    // @User: Standard
    AP_GROUPINFO("6_BAUD", 13, AP_SerialManager, state[6].baud, SERIAL6_BAUD),
#endif

#if SERIALMANAGER_NUM_PORTS > 1
    // @Param: 1_OPTIONS
    // @DisplayName: Telem1 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("1_OPTIONS",  14, AP_SerialManager, state[1].options, 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 2
    // @Param: 2_OPTIONS
    // @DisplayName: Telem2 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("2_OPTIONS",  15, AP_SerialManager, state[2].options, 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 3
    // @Param: 3_OPTIONS
    // @DisplayName: Serial3 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("3_OPTIONS",  16, AP_SerialManager, state[3].options, 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 4
    // @Param: 4_OPTIONS
    // @DisplayName: Serial4 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("4_OPTIONS",  17, AP_SerialManager, state[4].options, 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 5
    // @Param: 5_OPTIONS
    // @DisplayName: Serial5 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("5_OPTIONS",  18, AP_SerialManager, state[5].options, 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 6
    // @Param: 6_OPTIONS
    // @DisplayName: Serial6 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("6_OPTIONS",  19, AP_SerialManager, state[6].options, 0),
#endif

    // @Param: _PASS1
    // @DisplayName: Serial passthru first port
    // @Description: This sets one side of pass-through between two serial ports. Once both sides are set then all data received on either port will be passed to the other port
    // @Values: -1:Disabled,0:Serial0,1:Serial1,2:Serial2,3:Serial3,4:Serial4,5:Serial5,6:Serial6
    // @User: Advanced
    AP_GROUPINFO("_PASS1",  20, AP_SerialManager, passthru_port1, 0),

    // @Param: _PASS2
    // @DisplayName: Serial passthru second port
    // @Description: This sets one side of pass-through between two serial ports. Once both sides are set then all data received on either port will be passed to the other port
    // @Values: -1:Disabled,0:Serial0,1:Serial1,2:Serial2,3:Serial3,4:Serial4,5:Serial5,6:Serial6
    // @User: Advanced
    AP_GROUPINFO("_PASS2",  21, AP_SerialManager, passthru_port2, -1),

    // @Param: _PASSTIMO
    // @DisplayName: Serial passthru timeout
    // @Description: This sets a timeout for serial pass-through in seconds. When the pass-through is enabled by setting the SERIAL_PASS1 and SERIAL_PASS2 parameters then it remains in effect until no data comes from the first port for SERIAL_PASSTIMO seconds. This allows the port to revent to its normal usage (such as MAVLink connection to a GCS) when it is no longer needed. A value of 0 means no timeout.
    // @Range: 0 120
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("_PASSTIMO",  22, AP_SerialManager, passthru_timeout, 15),

#if SERIALMANAGER_NUM_PORTS > 7
    // @Param: 7_PROTOCOL
    // @DisplayName: Serial7 protocol selection
    // @Description: Control what protocol Serial7 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @CopyValuesFrom: SERIAL1_PROTOCOL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("7_PROTOCOL",  23, AP_SerialManager, state[7].protocol, SERIAL7_PROTOCOL),

    // @Param: 7_BAUD
    // @DisplayName: Serial 7 Baud Rate
    // @Description: The baud rate used for Serial7. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @CopyValuesFrom: SERIAL1_BAUD
    // @User: Standard
    AP_GROUPINFO("7_BAUD", 24, AP_SerialManager, state[7].baud, SERIAL7_BAUD),

    // @Param: 7_OPTIONS
    // @DisplayName: Serial7 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("7_OPTIONS",  25, AP_SerialManager, state[7].options, 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 8
    // @Param: 8_PROTOCOL
    // @DisplayName: Serial8 protocol selection
    // @Description: Control what protocol Serial8 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @CopyValuesFrom: SERIAL1_PROTOCOL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("8_PROTOCOL",  26, AP_SerialManager, state[8].protocol, SERIAL8_PROTOCOL),

    // @Param: 8_BAUD
    // @DisplayName: Serial 8 Baud Rate
    // @Description: The baud rate used for Serial8. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @CopyValuesFrom: SERIAL1_BAUD
    // @User: Standard
    AP_GROUPINFO("8_BAUD", 27, AP_SerialManager, state[8].baud, SERIAL8_BAUD),

    // @Param: 8_OPTIONS
    // @DisplayName: Serial8 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("8_OPTIONS",  28, AP_SerialManager, state[8].options, 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 9
    // @Param: 9_PROTOCOL
    // @DisplayName: Serial9 protocol selection
    // @Description: Control what protocol Serial9 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
    // @CopyValuesFrom: SERIAL1_PROTOCOL
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("9_PROTOCOL",  29, AP_SerialManager, state[9].protocol, SERIAL9_PROTOCOL),

    // @Param: 9_BAUD
    // @DisplayName: Serial 9 Baud Rate
    // @Description: The baud rate used for Serial8. Most stm32-based boards can support rates of up to 1500. If you setup a rate you cannot support and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @CopyValuesFrom: SERIAL1_BAUD
    // @User: Standard
    AP_GROUPINFO("9_BAUD", 30, AP_SerialManager, state[9].baud, SERIAL9_BAUD),

    // @Param: 9_OPTIONS
    // @DisplayName: Serial9 options
    // @Description: Control over UART options. The InvertRX option controls invert of the receive pin. The InvertTX option controls invert of the transmit pin. The HalfDuplex option controls half-duplex (onewire) mode, where both transmit and receive is done on the transmit wire.
    // @Bitmask: 0:InvertRX, 1:InvertTX, 2:HalfDuplex, 3:Swap, 4: RX_PullDown, 5: RX_PullUp, 6: TX_PullDown, 7: TX_PullUp, 8: RX_NoDMA, 9: TX_NoDMA, 10: Don't forward mavlink to/from, 11: DisableFIFO, 12: Ignore Streamrate
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("9_OPTIONS",  31, AP_SerialManager, state[9].options, 0),
#endif
    
    AP_GROUPEND
};

// singleton instance
AP_SerialManager *AP_SerialManager::_singleton;

// Constructor
AP_SerialManager::AP_SerialManager()
{
    _singleton = this;
    // setup parameter defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// init_console - initialise console at default baud rate
void AP_SerialManager::init_console()
{
    // initialise console immediately at default size and baud
#if SERIALMANAGER_NUM_PORTS > 0
    if (!init_console_done) {
        init_console_done = true;
        hal.serial(0)->begin(AP_SERIALMANAGER_CONSOLE_BAUD,
                             AP_SERIALMANAGER_CONSOLE_BUFSIZE_RX,
                             AP_SERIALMANAGER_CONSOLE_BUFSIZE_TX);
    }
#endif
}

// init - // init - initialise serial ports
void AP_SerialManager::init()
{
    // always reset passthru port2 on boot
    passthru_port2.set_and_save_ifchanged(-1);

#ifdef HAL_OTG1_CONFIG
    /*
      prevent users from changing USB protocol to other than
      MAVLink. This fixes an issue where users trying to get SLCAN
      change SERIAL0_PROTOCOL to 22 and find they can no longer connect
     */
    if (state[0].protocol != SerialProtocol_MAVLink &&
        state[0].protocol != SerialProtocol_MAVLink2) {
        state[0].protocol.set(SerialProtocol_MAVLink2);
    }
#endif

    init_console();

    // initialise serial ports
    for (uint8_t i=1; i<SERIALMANAGER_NUM_PORTS; i++) {
        auto *uart = hal.serial(i);

        if (uart != nullptr) {
            set_options(i);
            switch (state[i].protocol) {
                case SerialProtocol_None:
#if HAL_GCS_ENABLED
                    // disable RX and TX pins in case they are shared
                    // with another peripheral (eg. RCIN pin). We
                    // don't do this if GCS is not enabled as in that
                    // case we don't have serialmanager parameters and
                    // this would prevent AP_Periph from using a GPS
                    uart->disable_rxtx();
#endif
                    break;
                case SerialProtocol_Console:
                case SerialProtocol_MAVLink:
                case SerialProtocol_MAVLink2:
                case SerialProtocol_MAVLinkHL:
                    uart->begin(state[i].baudrate(),
                                         AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX,
                                         AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX);
                    break;
                case SerialProtocol_FrSky_D:
                    // Note baudrate is hardcoded to 9600
                    state[i].baud.set_and_default(AP_SERIALMANAGER_FRSKY_D_BAUD/1000); // update baud param in case user looks at it
                    // begin is handled by AP_Frsky_telem library
                    break;
                case SerialProtocol_FrSky_SPort:
                case SerialProtocol_FrSky_SPort_Passthrough:
                    // Note baudrate is hardcoded to 57600
                    state[i].baud.set_and_default(AP_SERIALMANAGER_FRSKY_SPORT_BAUD/1000); // update baud param in case user looks at it
                    // begin is handled by AP_Frsky_telem library
                    break;
                case SerialProtocol_GPS:
                case SerialProtocol_GPS2:
                    uart->begin(state[i].baudrate(),
                                         AP_SERIALMANAGER_GPS_BUFSIZE_RX,
                                         AP_SERIALMANAGER_GPS_BUFSIZE_TX);
                    break;
                case SerialProtocol_AlexMos:
                    // Note baudrate is hardcoded to 115200
                    state[i].baud.set_and_default(AP_SERIALMANAGER_ALEXMOS_BAUD / 1000);   // update baud param in case user looks at it
                    uart->begin(AP_SERIALMANAGER_ALEXMOS_BAUD,
                                         AP_SERIALMANAGER_ALEXMOS_BUFSIZE_RX,
                                         AP_SERIALMANAGER_ALEXMOS_BUFSIZE_TX);
                    break;
                case SerialProtocol_SToRM32:
                    // Note baudrate is hardcoded to 115200
                    state[i].baud.set_and_default(AP_SERIALMANAGER_SToRM32_BAUD / 1000);   // update baud param in case user looks at it
                    uart->begin(state[i].baudrate(),
                                         AP_SERIALMANAGER_SToRM32_BUFSIZE_RX,
                                         AP_SERIALMANAGER_SToRM32_BUFSIZE_TX);
                    break;
                case SerialProtocol_Aerotenna_USD1:
                    state[i].protocol.set_and_save(SerialProtocol_Rangefinder);
                    break;
                case SerialProtocol_Volz:
                                    // Note baudrate is hardcoded to 115200
                                    state[i].baud.set_and_default(AP_SERIALMANAGER_VOLZ_BAUD);   // update baud param in case user looks at it
                                    uart->begin(state[i].baudrate(),
                                    		AP_SERIALMANAGER_VOLZ_BUFSIZE_RX,
											AP_SERIALMANAGER_VOLZ_BUFSIZE_TX);
                                    uart->set_unbuffered_writes(true);
                                    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
                                    break;
                case SerialProtocol_Sbus1:
                    state[i].baud.set_and_default(AP_SERIALMANAGER_SBUS1_BAUD / 1000);   // update baud param in case user looks at it
                    uart->begin(state[i].baudrate(),
                                         AP_SERIALMANAGER_SBUS1_BUFSIZE_RX,
                                         AP_SERIALMANAGER_SBUS1_BUFSIZE_TX);
                    uart->configure_parity(2);    // enable even parity
                    uart->set_stop_bits(2);
                    uart->set_unbuffered_writes(true);
                    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
                    break;

                case SerialProtocol_ESCTelemetry:
                    // ESC telemetry protocol from BLHeli32 ESCs. Note that baudrate is hardcoded to 115200
                    state[i].baud.set_and_default(115200 / 1000);
                    uart->begin(state[i].baudrate(), 30, 30);
                    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
                    break;

                case SerialProtocol_Robotis:
                    uart->begin(state[i].baudrate(),
                                         AP_SERIALMANAGER_ROBOTIS_BUFSIZE_RX,
                                         AP_SERIALMANAGER_ROBOTIS_BUFSIZE_TX);
                    uart->set_unbuffered_writes(true);
                    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
                    break;

                case SerialProtocol_SLCAN:
                    uart->begin(state[i].baudrate(),
                                         AP_SERIALMANAGER_SLCAN_BUFSIZE_RX,
                                         AP_SERIALMANAGER_SLCAN_BUFSIZE_TX);
                    break;

#ifndef HAL_BUILD_AP_PERIPH
                case SerialProtocol_RCIN:
                    AP::RC().add_uart(uart);
                    break;
#endif
                    
                case SerialProtocol_EFI:
                    state[i].baud.set_default(AP_SERIALMANAGER_EFI_MS_BAUD);
                    uart->begin(state[i].baudrate(),
                                         AP_SERIALMANAGER_EFI_MS_BUFSIZE_RX,
                                         AP_SERIALMANAGER_EFI_MS_BUFSIZE_TX);
                    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
                    break;

                case SerialProtocol_Generator:
                    break;
#if HAL_MSP_ENABLED                    
                case SerialProtocol_MSP:
                case SerialProtocol_DJI_FPV:
                case SerialProtocol_MSP_DisplayPort:
                    // baudrate defaults to 115200
                    state[i].baud.set_default(AP_SERIALMANAGER_MSP_BAUD/1000);
                    uart->begin(state[i].baudrate(),
                                         AP_SERIALMANAGER_MSP_BUFSIZE_RX,
                                         AP_SERIALMANAGER_MSP_BUFSIZE_TX);
                    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
                    // Note init is handled by AP_MSP
                    break;
#endif
                default:
                    uart->begin(state[i].baudrate());
            }
        }
    }
}


const AP_SerialManager::UARTState *AP_SerialManager::find_protocol_instance(enum SerialProtocol protocol, uint8_t instance) const
{
    uint8_t found_instance = 0;

    // search for matching protocol
    for(uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (protocol_match(protocol, (enum SerialProtocol)state[i].protocol.get())) {
            if (found_instance == instance) {
                return &state[i];
            }
            found_instance++;
        }
    }

    // if we got this far we did not find the uart
    return nullptr;
}

// find_serial - searches available serial ports for the first instance that allows the given protocol
//  instance should be zero if searching for the first instance, 1 for the second, etc
//  returns uart on success, nullptr if a serial port cannot be found
AP_HAL::UARTDriver *AP_SerialManager::find_serial(enum SerialProtocol protocol, uint8_t instance) const
{
    const struct UARTState *_state = find_protocol_instance(protocol, instance);
    if (_state == nullptr) {
        return nullptr;
    }
    const uint8_t serial_idx = _state - &state[0];

    // set options before any user does begin()
    AP_HAL::UARTDriver *port = hal.serial(serial_idx);
    if (port) {
        port->set_options(_state->options);
    }
    return port;
}

// have_serial - return true if we have the given serial protocol configured
bool AP_SerialManager::have_serial(enum SerialProtocol protocol, uint8_t instance) const
{
    return find_protocol_instance(protocol, instance) != nullptr;
}

// find_baudrate - searches available serial ports for the first instance that allows the given protocol
//  instance should be zero if searching for the first instance, 1 for the second, etc
//  returns baudrate on success, 0 if a serial port cannot be found
uint32_t AP_SerialManager::find_baudrate(enum SerialProtocol protocol, uint8_t instance) const
{
    const struct UARTState *_state = find_protocol_instance(protocol, instance);
    if (_state == nullptr) {
        return 0;
    }
    return _state->baudrate();
}

// find_portnum - find port number (SERIALn index) for a protocol and instance, -1 for not found
int8_t AP_SerialManager::find_portnum(enum SerialProtocol protocol, uint8_t instance) const
{
    const struct UARTState *_state = find_protocol_instance(protocol, instance);
    if (_state == nullptr) {
        return -1;
    }
    return int8_t(_state - &state[0]);
}

// get_serial_by_id - gets serial by serial id
AP_HAL::UARTDriver *AP_SerialManager::get_serial_by_id(uint8_t id)
{
    if (id < SERIALMANAGER_NUM_PORTS) {
        return hal.serial(id);
    }
    return nullptr;
}

// set_blocking_writes_all - sets block_writes on or off for all serial channels
void AP_SerialManager::set_blocking_writes_all(bool blocking)
{
    // set block_writes for all initialised serial ports
    for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        auto *uart = hal.serial(i);
        if (uart != nullptr) {
            uart->set_blocking_writes(blocking);
        }
    }
}

/*
 *  map from a 16 bit EEPROM baud rate to a real baud rate.  For
 *  stm32-based boards we can do 1.5MBit, although 921600 is more
 *  reliable.
 */
uint32_t AP_SerialManager::map_baudrate(int32_t rate)
{
    if (rate <= 0) {
        rate = 57;
    }
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 100:  return 100000;
    case 111:  return 111100;
    case 115:  return 115200;
    case 230:  return 230400;
    case 256:  return 256000;
    case 460:  return 460800;
    case 500:  return 500000;
    case 921:  return 921600;
    case 1500:  return 1500000;
    }

    if (rate > 2000) {
        // assume it is a direct baudrate. This allows for users to
        // set an exact baudrate as long as it is over 2000 baud
        return (uint32_t)rate;
    }

    // otherwise allow any other kbaud rate
    return rate*1000;
}

// protocol_match - returns true if the protocols match
bool AP_SerialManager::protocol_match(enum SerialProtocol protocol1, enum SerialProtocol protocol2) const
{
    // check for obvious match
    if (protocol1 == protocol2) {
        return true;
    }

    // mavlink match
    if (((protocol1 == SerialProtocol_MAVLink) || (protocol1 == SerialProtocol_MAVLink2) || (protocol1 == SerialProtocol_MAVLinkHL)) &&
        ((protocol2 == SerialProtocol_MAVLink) || (protocol2 == SerialProtocol_MAVLink2) || (protocol2 == SerialProtocol_MAVLinkHL))) {
        return true;
    }

    // gps match
    if (((protocol1 == SerialProtocol_GPS) || (protocol1 == SerialProtocol_GPS2)) &&
        ((protocol2 == SerialProtocol_GPS) || (protocol2 == SerialProtocol_GPS2))) {
        return true;
    }

    return false;
}

// setup any special options
void AP_SerialManager::set_options(uint16_t i)
{
    struct UARTState &opt = state[i];
    // pass through to HAL
    if (!hal.serial(i)->set_options(opt.options)) {
        DEV_PRINTF("Unable to setup options for Serial%u\n", i);
    }
}

// get the passthru ports if enabled
bool AP_SerialManager::get_passthru(AP_HAL::UARTDriver *&port1, AP_HAL::UARTDriver *&port2, uint8_t &timeout_s,
                                    uint32_t &baud1, uint32_t &baud2) const
{
    if (passthru_port2 < 0 ||
        passthru_port2 >= SERIALMANAGER_NUM_PORTS ||
        passthru_port1 < 0 ||
        passthru_port1 >= SERIALMANAGER_NUM_PORTS) {
        return false;
    }
    port1 = hal.serial(passthru_port1);
    port2 = hal.serial(passthru_port2);
    baud1 = state[passthru_port1].baudrate();
    baud2 = state[passthru_port2].baudrate();
    timeout_s = MAX(passthru_timeout, 0);
    return true;
}

// disable passthru by settings SERIAL_PASS2 to -1
void AP_SerialManager::disable_passthru(void)
{
    passthru_port2.set_and_notify(-1);
}

// accessor for AP_Periph to set baudrate and type
void AP_SerialManager::set_protocol_and_baud(uint8_t sernum, enum SerialProtocol protocol, uint32_t baudrate)
{
    if (sernum < SERIALMANAGER_NUM_PORTS) {
        state[sernum].protocol.set(protocol);
        state[sernum].baud.set(baudrate);
    }
}


namespace AP {

AP_SerialManager &serialmanager()
{
    return *AP_SerialManager::get_singleton();
}

}
