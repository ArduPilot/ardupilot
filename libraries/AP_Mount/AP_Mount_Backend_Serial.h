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

/*
  Mount driver backend class for serial drivers
  Mounts using custom serial protocols should be derived from this class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include "AP_Mount_Backend.h"

class AP_Mount_Backend_Serial : public AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Backend_Serial(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance, uint8_t serial_instance) :
        AP_Mount_Backend(frontend, params, instance),
        _serial_instance(serial_instance)
    {}

    // perform any required initialisation for this instance
    void init() override;

protected:

    // internal variables
    AP_HAL::UARTDriver *_uart;      // uart connected to gimbal
    uint8_t _serial_instance;       // this instance's serial instance number
    bool _initialised;              // true if uart has been initialised
};

#endif // HAL_MOUNT_ENABLED
