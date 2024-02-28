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

#include "AP_SerialManager_config.h"
#include <AP_Param/AP_Param.h>

class AP_SerialManager {
public:
    AP_SerialManager();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_SerialManager);

    enum SerialProtocol {
        SerialProtocol_None = -1,
        SerialProtocol_Console = 0, // unused
        SerialProtocol_MAVLink = 1,
        SerialProtocol_MAVLink2 = 2,                 // do not use - use MAVLink and provide instance of 1
        SerialProtocol_FrSky_D = 3,                  // FrSky D protocol (D-receivers)
        SerialProtocol_FrSky_SPort = 4,              // FrSky SPort protocol (X-receivers)
        SerialProtocol_GPS = 5,
        SerialProtocol_GPS2 = 6,                     // do not use - use GPS and provide instance of 1
        SerialProtocol_AlexMos = 7,
        SerialProtocol_Gimbal = 8,                   // SToRM32, Siyi custom serial protocols
        SerialProtocol_Rangefinder = 9,
        SerialProtocol_FrSky_SPort_Passthrough = 10, // FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
        SerialProtocol_Lidar360 = 11,                // Lightware SF40C, TeraRanger Tower or RPLidarA2
        SerialProtocol_Aerotenna_USD1      = 12, // USD1 support - deprecated, users should use Rangefinder
        SerialProtocol_Beacon = 13,
        SerialProtocol_Volz = 14,                    // Volz servo protocol
        SerialProtocol_Sbus1 = 15,
        SerialProtocol_ESCTelemetry = 16,
        SerialProtocol_Devo_Telem = 17,
        SerialProtocol_OpticalFlow = 18,
        SerialProtocol_Robotis = 19,
        SerialProtocol_NMEAOutput = 20,
        SerialProtocol_WindVane = 21,
        SerialProtocol_SLCAN = 22,
        SerialProtocol_RCIN = 23,
        SerialProtocol_EFI = 24,                   // EFI serial protocol
        SerialProtocol_LTM_Telem = 25,
        SerialProtocol_RunCam = 26,
        SerialProtocol_Hott = 27,
        SerialProtocol_Scripting = 28,
        SerialProtocol_CRSF = 29,
        SerialProtocol_Generator = 30,
        SerialProtocol_Winch = 31,
        SerialProtocol_MSP = 32,
        SerialProtocol_DJI_FPV = 33,
        SerialProtocol_AirSpeed = 34,
        SerialProtocol_ADSB = 35,
        SerialProtocol_AHRS = 36,
        SerialProtocol_SmartAudio = 37,
        SerialProtocol_FETtecOneWire = 38,
        SerialProtocol_Torqeedo = 39,
        SerialProtocol_AIS = 40,
        SerialProtocol_CoDevESC = 41,
        SerialProtocol_MSP_DisplayPort = 42,
        SerialProtocol_MAVLinkHL = 43,
        SerialProtocol_Tramp = 44,
        SerialProtocol_DDS_XRCE = 45,
        SerialProtocol_IMUOUT = 46,
        // Reserving Serial Protocol 47 for SerialProtocol_IQ
        SerialProtocol_PPP = 48,
        SerialProtocol_NumProtocols                    // must be the last value
    };

    // get singleton instance
    static AP_SerialManager *get_singleton(void) {
        return _singleton;
    }

    // init_console - initialise console at default baud rate
    void init_console();

    // init - initialise serial ports
    void init();

    // find_serial - searches available serial ports that allows the given protocol
    //  instance should be zero if searching for the first instance, 1 for the second, etc
    //  returns uart on success, nullptr if a serial port cannot be found
    // note that the SERIALn_OPTIONS are applied if the port is found
    AP_HAL::UARTDriver *find_serial(enum SerialProtocol protocol, uint8_t instance) const;

    // have_serial - return true if we have the corresponding serial protocol configured
    bool have_serial(enum SerialProtocol protocol, uint8_t instance) const;
    
    // find_baudrate - searches available serial ports for the first instance that allows the given protocol
    //  instance should be zero if searching for the first instance, 1 for the second, etc
    //  returns the baudrate of that protocol on success, 0 if a serial port cannot be found
    uint32_t find_baudrate(enum SerialProtocol protocol, uint8_t instance) const;

    // find_portnum - find port number (SERIALn index) for a protocol and instance, -1 for not found
    int8_t find_portnum(enum SerialProtocol protocol, uint8_t instance) const;

    // get the passthru ports if enabled
    bool get_passthru(AP_HAL::UARTDriver *&port1, AP_HAL::UARTDriver *&port2, uint8_t &timeout_s,
                      uint32_t &baud1, uint32_t &baud2);

    // disable passthru by settings SERIAL_PASS2 to -1
    void disable_passthru(void);

    // get Serial Port
    AP_HAL::UARTDriver *get_serial_by_id(uint8_t id);

    // accessors for AP_Periph to set baudrate and type
    void set_protocol_and_baud(uint8_t sernum, enum SerialProtocol protocol, uint32_t baudrate);

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
        AP_Int32 baud;
        AP_Int16 options;
        AP_Int8 protocol;

        // serial index number
        uint8_t idx;
    };

    // get a state from serial index
    const UARTState *get_state_by_id(uint8_t id) const;

    // search through managed serial connections looking for the
    // instance-nth UART which is running protocol protocol.
    // protocol_match is used to determine equivalence of one protocol
    // to another, e.g. MAVLink2 is considered MAVLink1 for finding
    // mavlink1 protocol instances.
    const UARTState *find_protocol_instance(enum SerialProtocol protocol,
                                            uint8_t instance) const;

#if AP_SERIALMANAGER_REGISTER_ENABLED
    /*
      a class for a externally registered port
      used by AP_Networking
     */
    class RegisteredPort : public AP_HAL::UARTDriver {
    public:
        RegisteredPort *next;
        UARTState state;
    };
    RegisteredPort *registered_ports;
    HAL_Semaphore port_sem;

    // register an externally managed port
    void register_port(RegisteredPort *port);

#endif // AP_SERIALMANAGER_REGISTER_ENABLED


private:
    static AP_SerialManager *_singleton;

    // array of uart info. See comment above about
    // SERIALMANAGER_MAX_PORTS
    UARTState state[SERIALMANAGER_MAX_PORTS];

    // pass-through serial support
    AP_Int8 passthru_port1;
    AP_Int8 passthru_port2;
    AP_Int8 passthru_timeout;

    // protocol_match - returns true if the protocols match
    bool protocol_match(enum SerialProtocol protocol1, enum SerialProtocol protocol2) const;

    // setup any special options
    void set_options(uint16_t i);

    bool init_console_done;
};

namespace AP {
    AP_SerialManager &serialmanager();
};
