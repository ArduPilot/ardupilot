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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "AP_SerialDevice_UART.h"

#ifdef HAL_UART_NUM_SERIAL_PORTS
#if HAL_UART_NUM_SERIAL_PORTS >= 4
#define SERIALMANAGER_NUM_PORTS HAL_UART_NUM_SERIAL_PORTS
#else
// we need a minimum of 4 to allow for a GPS due to the odd ordering
// of hal.uartB as SERIAL3
#define SERIALMANAGER_NUM_PORTS 4
#endif
#else
// assume max 8 ports
#define SERIALMANAGER_NUM_PORTS 8
#endif

/*
  array size for state[]. This needs to be at least
  SERIALMANAGER_NUM_PORTS, but we want it to be the same length on
  similar boards to get the ccache efficiency up. This wastes a small
  amount of memory, but makes a huge difference to the build times
 */
#if SERIALMANAGER_NUM_PORTS > 10 || SERIALMANAGER_NUM_PORTS < 5
#define SERIALMANAGER_MAX_PORTS SERIALMANAGER_NUM_PORTS
#else
#define SERIALMANAGER_MAX_PORTS 10
#endif


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

// FrSky default baud rates, use default buffer sizes
#define AP_SERIALMANAGER_FRSKY_D_BAUD           9600
#define AP_SERIALMANAGER_FRSKY_SPORT_BAUD       57600
#define AP_SERIALMANAGER_FRSKY_BUFSIZE_RX       0
#define AP_SERIALMANAGER_FRSKY_BUFSIZE_TX       0

// GPS default baud rates and buffer sizes
// we need a 256 byte buffer for some GPS types (eg. UBLOX)
#define AP_SERIALMANAGER_GPS_BAUD               38400
#define AP_SERIALMANAGER_GPS_BUFSIZE_RX         256
#define AP_SERIALMANAGER_GPS_BUFSIZE_TX         16

// AlexMos Gimbal protocol default baud rates and buffer sizes
#define AP_SERIALMANAGER_ALEXMOS_BAUD           115200
#define AP_SERIALMANAGER_ALEXMOS_BUFSIZE_RX     128
#define AP_SERIALMANAGER_ALEXMOS_BUFSIZE_TX     128

#define AP_SERIALMANAGER_SToRM32_BAUD           115200
#define AP_SERIALMANAGER_SToRM32_BUFSIZE_RX     128
#define AP_SERIALMANAGER_SToRM32_BUFSIZE_TX     128

#define AP_SERIALMANAGER_VOLZ_BAUD           115
#define AP_SERIALMANAGER_VOLZ_BUFSIZE_RX     128
#define AP_SERIALMANAGER_VOLZ_BUFSIZE_TX     128

#define AP_SERIALMANAGER_ROBOTIS_BUFSIZE_RX  128
#define AP_SERIALMANAGER_ROBOTIS_BUFSIZE_TX  128

// MegaSquirt EFI protocol
#define AP_SERIALMANAGER_EFI_MS_BAUD           115
#define AP_SERIALMANAGER_EFI_MS_BUFSIZE_RX     512
#define AP_SERIALMANAGER_EFI_MS_BUFSIZE_TX     16

// SBUS servo outputs
#define AP_SERIALMANAGER_SBUS1_BAUD           100000
#define AP_SERIALMANAGER_SBUS1_BUFSIZE_RX     16
#define AP_SERIALMANAGER_SBUS1_BUFSIZE_TX     32

#define AP_SERIALMANAGER_SLCAN_BAUD             115200
#define AP_SERIALMANAGER_SLCAN_BUFSIZE_RX       128
#define AP_SERIALMANAGER_SLCAN_BUFSIZE_TX       128

// MSP protocol default buffer sizes
#define AP_SERIALMANAGER_MSP_BUFSIZE_RX     128
#define AP_SERIALMANAGER_MSP_BUFSIZE_TX     256
#define AP_SERIALMANAGER_MSP_BAUD           115200

class AP_SerialManager {
public:
    AP_SerialManager();

    /* Do not allow copies */
    AP_SerialManager(const AP_SerialManager &other) = delete;
    AP_SerialManager &operator=(const AP_SerialManager&) = delete;

    // get singleton instance
    static AP_SerialManager *get_singleton(void) {
        return _singleton;
    }

    // init_console - initialise console at default baud rate
    void init_console();

    // init - initialise serial ports
    void init();

    AP_SerialDevice *get(uint8_t instance) const {
        if (instance < ARRAY_SIZE(serial_uart_device)) {
            return serial_uart_device[instance];
        }
        return nullptr;
    }

    // find_serial - searches available serial ports that allows the given protocol
    //  instance should be zero if searching for the first instance, 1 for the second, etc
    //  returns uart on success, nullptr if a serial port cannot be found
    AP_SerialDevice *find_serial(AP_SerialDevice::Protocol protocol, uint8_t instance) const;
    AP_SerialDevice_UART *find_serial_uart(AP_SerialDevice::Protocol protocol, uint8_t instance) const;

    // have_serial - return true if we have the corresponding serial protocol configured
    bool have_serial(enum SerialProtocol protocol, uint8_t instance) const;
    
    // find_baudrate - searches available serial ports for the first instance that allows the given protocol
    //  instance should be zero if searching for the first instance, 1 for the second, etc
    //  returns the baudrate of that protocol on success, 0 if a serial port cannot be found
    // uint32_t find_baudrate(AP_SerialDevice::Protocol protocol, uint8_t instance) const;

    // find_portnum - find port number (SERIALn index) for a protocol and instance, -1 for not found
    int8_t find_portnum(AP_SerialDevice::Protocol protocol, uint8_t instance) const;

    // set_blocking_writes_all - sets block_writes on or off for all serial channels
    void set_blocking_writes_all(bool blocking);

    // get the passthru ports if enabled
    bool get_passthru(AP_SerialDevice *&port1,
                      AP_SerialDevice *&port2,
                      uint8_t &timeout_s,
                      uint32_t &baud1,
                      uint32_t &baud2) const;

    // disable passthru by settings SERIAL_PASS2 to -1
    void disable_passthru(void);

    // get Serial Port
    AP_SerialDevice *get_serial_by_id(uint8_t id);

    // accessors for AP_Periph to set baudrate and type
    void set_protocol_and_baud(uint8_t sernum, AP_SerialDevice::Protocol protocol, uint32_t baudrate);

    static uint32_t map_baudrate(int32_t rate);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    class UARTState {
        friend class AP_SerialManager;
    public:
        bool option_enabled(uint16_t option) const {
            return (options & option) == option;
        }
        // returns a baudrate such as 9600.  May map from a special
        // parameter value like "57" to "57600":
        uint32_t baudrate() const {
            return AP_SerialManager::map_baudrate(baud);
        }
        AP_SerialManager::SerialProtocol get_protocol() const {
            return AP_SerialManager::SerialProtocol(protocol.get());
        }
    private:
        AP_Int32 baud;
        AP_Int16 options;
        AP_Int8 protocol;
    };

    // search through managed serial connections looking for the
    // instance-nth UART which is running protocol protocol.
    // protocol_match is used to determine equivalence of one protocol
    // to another, e.g. MAVLink2 is considered MAVLink1 for finding
    // mavlink1 protocol instances.
    const UARTState *find_protocol_instance(enum SerialProtocol protocol,
                                            uint8_t instance) const;

private:
    static AP_SerialManager *_singleton;

    AP_SerialDevice_UART::UARTState state[SERIALMANAGER_NUM_PORTS];

    // array of uart info
    AP_SerialDevice_UART *serial_uart_device[SERIALMANAGER_NUM_PORTS];

    // pass-through serial support
    AP_Int8 passthru_port1;
    AP_Int8 passthru_port2;
    AP_Int8 passthru_timeout;

    void init_serial_uart_devices();
    uint8_t count_protocol_instances(AP_SerialDevice::Protocol protocol) const;

    // search through managed serial connections looking for the
    // instance-nth UART which is running protocol protocol
    const AP_SerialDevice_UART::UARTState *find_protocol_instance(AP_SerialDevice::Protocol protocol,
                                      uint8_t instance) const;

    // protocol_match - returns true if the protocols match
    bool protocol_match(AP_SerialDevice::Protocol protocol1, AP_SerialDevice::Protocol protocol2) const;

    bool init_console_done;
};

namespace AP {
    AP_SerialManager &serialmanager();
};
