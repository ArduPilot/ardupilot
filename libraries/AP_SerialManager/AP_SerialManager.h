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

#ifndef _AP_SERIALMANAGER_
#define _AP_SERIALMANAGER_

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_HAL.h>
#include <GCS_MAVLink.h>

#define SERIALMANAGER_NUM_PORTS 5

 // console default baud rates and buffer sizes
#ifdef HAL_SERIAL0_BAUD_DEFAULT
# define AP_SERIALMANAGER_CONSOLE_BAUD          HAL_SERIAL0_BAUD_DEFAULT
#else
# define AP_SERIALMANAGER_CONSOLE_BAUD          115200
#endif
# define AP_SERIALMANAGER_CONSOLE_BUFSIZE_RX    128
# define AP_SERIALMANAGER_CONSOLE_BUFSIZE_TX    512

// mavlink default baud rates and buffer sizes
#define AP_SERIALMANAGER_MAVLINK_BAUD           57600
#define AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX     128
#define AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX     256

// mavlink default baud rates, use default buffer sizes
#define AP_SERIALMANAGER_FRSKY_DPORT_BAUD       9600
#define AP_SERIALMANAGER_FRSKY_SPORT_BAUD       57600
#define AP_SERIALMANAGER_FRSKY_BUFSIZE_RX       0
#define AP_SERIALMANAGER_FRSKY_BUFSIZE_TX       0

// GPS default baud rates and buffer sizes
// we need a 256 byte buffer for some GPS types (eg. UBLOX)
#define AP_SERIALMANAGER_GPS_BAUD               38400
#define AP_SERIALMANAGER_GPS_BUFSIZE_RX         256
#define AP_SERIALMANAGER_GPS_BUFSIZE_TX         16

// AlexMos Gimbal protocol default baud rates and buffer sizes
// we need a 256 byte buffer for some GPS types (eg. UBLOX)
#define AP_SERIALMANAGER_ALEXMOS_BAUD           115200
#define AP_SERIALMANAGER_ALEXMOS_BUFSIZE_RX     128
#define AP_SERIALMANAGER_ALEXMOS_BUFSIZE_TX     128

class AP_SerialManager {

public:

    enum SerialProtocol {
        SerialProtocol_Console = 0,
        SerialProtocol_MAVLink1 = 1,
        SerialProtocol_MAVLink2 = 2,
        SerialProtocol_FRSky_DPort = 3,
        SerialProtocol_FRSky_SPort = 4,
        SerialProtocol_GPS = 5,
        SerialProtocol_GPS2 = 6,
        SerialProtocol_AlexMos = 7
    };

    // array of uart info
    typedef struct {
        AP_Int8 protocol;
        AP_Int16 baud;
        uint16_t rx_size;
        uint16_t tx_size;
        AP_HAL::UARTDriver* uart;
    } serial_state;

    // Constructor
    AP_SerialManager();

    // init_console - initialise console at default baud rate
    void init_console();

    // init - initialise serial ports
    void init();

    // find_serial - searches available serial ports for the first instance that allows the given protocol
    //  returns true on success, false if a serial port cannot be found
    //  result is updated with reference to the serial port
    bool find_serial(enum SerialProtocol protocol, serial_state& ret_state) const;

    // get_mavlink_channel - provides the mavlink channel associated with a given protocol
    //  returns true if a channel is found, false if not
    bool get_mavlink_channel(enum SerialProtocol protocol, mavlink_channel_t &mav_chan) const;

    // set_blocking_writes_all - sets block_writes on or off for all serial channels
    void set_blocking_writes_all(bool blocking);

    // set_console_baud - sets the console's baud rate to the rate specified by the protocol
    //  used on APM2 to switch the console between the console baud rate (115200) and the SERIAL1 baud rate (user configurable)
    void set_console_baud(enum SerialProtocol protocol) const;

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // array of uart info
    serial_state state[SERIALMANAGER_NUM_PORTS];
};

#endif // _AP_SERIALMANAGER_
