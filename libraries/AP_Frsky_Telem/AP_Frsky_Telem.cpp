/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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
   FRSKY Telemetry library
*/

#include "AP_Frsky_Telem.h"

#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_Frsky_D.h"
#include "AP_Frsky_SPort.h"
#include "AP_Frsky_SPort_Passthrough.h"

extern const AP_HAL::HAL& hal;

AP_Frsky_Telem *AP_Frsky_Telem::singleton;

AP_Frsky_Telem::AP_Frsky_Telem()
{
    singleton = this;
}

AP_Frsky_Telem::~AP_Frsky_Telem(void)
{
    singleton = nullptr;
}

/*
 * init - perform required initialisation
 */
bool AP_Frsky_Telem::init(bool use_external_data)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    AP_HAL::UARTDriver *port;
    if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_D, 0))) {
        _backend = new AP_Frsky_D(port);
    } else if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort, 0))) {
        _backend = new AP_Frsky_SPort(port);
    } else if (use_external_data || (port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough, 0))) {
        _backend = new AP_Frsky_SPort_Passthrough(port, use_external_data);
    }

    if (_backend == nullptr) {
        return false;
    }

    if (!_backend->init()) {
        delete _backend;
        _backend = nullptr;
        return false;
    }

    return true;
}

bool AP_Frsky_Telem::_get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data)
{
    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_telem_data(frame, appid, data);
}

/*
  fetch Sport data for an external transport, such as FPort
 */
bool AP_Frsky_Telem::get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data)
{
    if (!singleton && !hal.util->get_soft_armed()) {
        // if telem data is requested when we are disarmed and don't
        // yet have a AP_Frsky_Telem object then try to allocate one
        new AP_Frsky_Telem();
        // initialize the passthrough scheduler
        if (singleton) {
            singleton->init(true);
        }
    }
    if (!singleton) {
        return false;
    }
    return singleton->_get_telem_data(frame, appid, data);
}

namespace AP {
    AP_Frsky_Telem *frsky_telem() {
        return AP_Frsky_Telem::get_singleton();
    }
};
