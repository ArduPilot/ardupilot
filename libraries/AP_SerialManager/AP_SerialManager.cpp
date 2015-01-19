/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   Please contribute your ideas! See http://dev.ardupilot.com for details

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

#include <AP_HAL.h>
#include <AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_SerialManager::var_info[] PROGMEM = {
    // @Param: 0_BAUD
    // @DisplayName: Serial0 baud rate
    // @Description: The baud rate used on the USB console. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("0_BAUD",  0, AP_SerialManager, state[0].baud, AP_SERIALMANAGER_CONSOLE_BAUD/1000),

    // @Param: 1_PROTOCOL
    // @DisplayName: Telem1 protocol selection
    // @Description: Control what protocol to use on the Telem1 port
    // @Values: 1:GCS Mavlink, 2:Frsky D-PORT, 3:GPS
    // @User: Standard
    AP_GROUPINFO("1_PROTOCOL",  1, AP_SerialManager, state[1].protocol, SerialProtocol_MAVLink1),

    // @Param: 1_BAUD
    // @DisplayName: Telem1 Baud Rate
    // @Description: The baud rate used on the Telem1 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("1_BAUD", 2, AP_SerialManager, state[1].baud, AP_SERIALMANAGER_MAVLINK_BAUD/1000),

    // @Param: 2_PROTOCOL
    // @DisplayName: Telemetry 2 protocol selection
    // @Description: Control what protocol to use on the Telem2 port
    // @Values: 1:GCS Mavlink, 2:Frsky D-PORT, 3:GPS
    // @User: Standard
    AP_GROUPINFO("2_PROTOCOL",  3, AP_SerialManager, state[2].protocol, SerialProtocol_MAVLink2),

    // @Param: 2_BAUD
    // @DisplayName: Telemetry 2 Baud Rate
    // @Description: The baud rate of the Telem2 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("2_BAUD", 4, AP_SerialManager, state[2].baud, AP_SERIALMANAGER_MAVLINK_BAUD/1000),

    // @Param: 3_PROTOCOL
    // @DisplayName: Serial 3 (GPS) protocol selection
    // @Description: Control what protocol Serial 3 (GPS) should be used for
    // @Values: 1:GCS Mavlink, 2:Frsky D-PORT, 3:GPS
    // @User: Standard
    AP_GROUPINFO("3_PROTOCOL",  5, AP_SerialManager, state[3].protocol, SerialProtocol_GPS),

    // @Param: 3_BAUD
    // @DisplayName: Serial 3 (GPS) Baud Rate
    // @Description: The baud rate used for the Serial 3 (GPS). The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("3_BAUD", 6, AP_SerialManager, state[3].baud, AP_SERIALMANAGER_GPS_BAUD/1000),

    // @Param: 4_PROTOCOL
    // @DisplayName: Serial4 protocol selection
    // @Description: Control what protocol Serial4 port should be used for
    // @Values: 1:GCS Mavlink, 2:Frsky D-PORT, 3:GPS
    // @User: Standard
    AP_GROUPINFO("4_PROTOCOL",  7, AP_SerialManager, state[4].protocol, SerialProtocol_GPS2),

    // @Param: 4_BAUD
    // @DisplayName: Serial 4 Baud Rate
    // @Description: The baud rate used for Serial4. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,500:500000,921:921600,1500:1500000
    // @User: Standard
    AP_GROUPINFO("4_BAUD", 8, AP_SerialManager, state[4].baud, AP_SERIALMANAGER_GPS_BAUD/1000),

    AP_GROUPEND
};

// Constructor
AP_SerialManager::AP_SerialManager()
{
    // setup parameter defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// init_console - initialise console at default baud rate
void AP_SerialManager::init_console()
{
    // initialise console immediately at default size and baud
    state[0].protocol = SerialProtocol_Console; // protocol is unused for console
    state[0].uart = hal.uartA;  // serial0, uartA, always console
    state[0].rx_size = AP_SERIALMANAGER_CONSOLE_BUFSIZE_RX;
    state[0].tx_size = AP_SERIALMANAGER_CONSOLE_BUFSIZE_TX;
    state[0].uart->begin(AP_SERIALMANAGER_CONSOLE_BAUD, state[0].rx_size, state[0].tx_size);
}

// init - // init - initialise serial ports
void AP_SerialManager::init()
{
    // initialise pointers to serial ports
    state[1].uart = hal.uartC;  // serial1, uartC, normally telem1
    state[2].uart = hal.uartD;  // serial2, uartD, normally telem2
    state[3].uart = hal.uartB;  // serial3, uartB, normally 1st GPS
    state[4].uart = hal.uartE;  // serial4, uartE, normally 2nd GPS

    // initialise serial ports
    for (uint8_t i=1; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (state[i].uart != NULL) {
            switch (state[i].protocol) {
                case SerialProtocol_Console:
                    state[i].rx_size = AP_SERIALMANAGER_CONSOLE_BUFSIZE_RX;
                    state[i].tx_size = AP_SERIALMANAGER_CONSOLE_BUFSIZE_TX;
                    state[i].uart->begin(map_baudrate(state[i].baud), state[i].rx_size, state[i].tx_size);
                    break;
                case SerialProtocol_MAVLink1:
                case SerialProtocol_MAVLink2:
                    state[i].rx_size = AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX;
                    state[i].tx_size = AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX;
                    state[i].uart->begin(map_baudrate(state[i].baud), state[i].rx_size, state[i].tx_size);
                    break;
                case SerialProtocol_FRSky_DPort:
                    // Note baudrate is hardcoded to 57600
                    state[i].rx_size = AP_SERIALMANAGER_FRSKY_BUFSIZE_RX;
                    state[i].tx_size = AP_SERIALMANAGER_FRSKY_BUFSIZE_TX;
                    state[i].baud = AP_SERIALMANAGER_FRSKY_DPORT_BAUD/1000; // update baud param in case user looks at it
                    state[i].uart->begin(AP_SERIALMANAGER_FRSKY_DPORT_BAUD, state[i].rx_size, state[i].tx_size);
                    break;
                case SerialProtocol_FRSky_SPort:
                    // Note baudrate is hardcoded to 9600
                    state[i].rx_size = AP_SERIALMANAGER_FRSKY_BUFSIZE_RX;
                    state[i].tx_size = AP_SERIALMANAGER_FRSKY_BUFSIZE_TX;
                    state[i].baud = AP_SERIALMANAGER_FRSKY_SPORT_BAUD/1000; // update baud param in case user looks at it
                    state[i].uart->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, state[i].rx_size, state[i].tx_size);
                    break;
                case SerialProtocol_GPS:
                case SerialProtocol_GPS2:
                    state[i].rx_size = AP_SERIALMANAGER_GPS_BUFSIZE_RX;
                    state[i].tx_size = AP_SERIALMANAGER_GPS_BUFSIZE_TX;
                    state[i].uart->begin(map_baudrate(state[i].baud), state[i].rx_size, state[i].tx_size);
                    break;
                case SerialProtocol_AlexMos:
                    // Note baudrate is hardcoded to 115200
                    state[i].rx_size = AP_SERIALMANAGER_ALEXMOS_BUFSIZE_RX;
                    state[i].tx_size = AP_SERIALMANAGER_ALEXMOS_BUFSIZE_TX;
                    state[i].baud = AP_SERIALMANAGER_ALEXMOS_BAUD / 1000;   // update baud param in case user looks at it
                    state[i].uart->begin(AP_SERIALMANAGER_ALEXMOS_BAUD, state[i].rx_size, state[i].tx_size);
                    break;
            }
        }
    }
}

// find_serial - searches available serial ports for the first instance that allows the given protocol
//  returns true on success, false if a serial port cannot be found
//  result is updated with reference to the serial port
bool AP_SerialManager::find_serial(enum SerialProtocol protocol, serial_state& ret_state) const
{
    // search for matching protocol
    for(uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (state[i].protocol == protocol) {
            ret_state = state[i];
            return true;
        }
    }

    // if we got this far we did not find the uart
    return false;
}

// get_mavlink_channel - provides the mavlink channel associated with a given protocol
//  returns true if a channel is found, false if not
bool AP_SerialManager::get_mavlink_channel(enum SerialProtocol protocol, mavlink_channel_t &mav_chan) const
{
    switch (protocol) {
        case SerialProtocol_Console:
            mav_chan = MAVLINK_COMM_0;
            return true;
            break;
        case SerialProtocol_MAVLink1:
            mav_chan = MAVLINK_COMM_1;
            return true;
            break;
        case SerialProtocol_MAVLink2:
            mav_chan = MAVLINK_COMM_2;
            return true;
            break;
        default:
            return false;
            break;
    }

    // we should never reach here
    return false;
}

// set_blocking_writes_all - sets block_writes on or off for all serial channels
void AP_SerialManager::set_blocking_writes_all(bool blocking)
{
    // set block_writes for all initialised serial ports
    for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (state[i].uart != NULL) {
            state[i].uart->set_blocking_writes(blocking);
        }
    }
}

// set_console_baud - sets the console's baud rate to the rate specified by the protocol
//  used on APM2 to switch the console between the console baud rate (115200) and the SERIAL1 baud rate (user configurable)
void AP_SerialManager::set_console_baud(enum SerialProtocol protocol) const
{
    // find baud rate of this protocol
    for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS; i++) {
        if (state[i].protocol == protocol) {
            // set console's baud rate
            state[0].uart->begin(map_baudrate(state[i].baud));
        }
    }
}
