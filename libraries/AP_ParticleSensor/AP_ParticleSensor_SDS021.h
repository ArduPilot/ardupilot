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
#pragma once

#include "AP_ParticleSensor_Backend.h"
#include <AP_HAL/AP_HAL.h>

class AP_ParticleSensor_SDS021 : public AP_ParticleSensor_Backend {
public:

    AP_ParticleSensor_SDS021(AP_HAL::UARTDriver &port);

    virtual void update() override;

private:

    AP_HAL::UARTDriver &port;

    void yield_message();
    void handle_byte_read(uint8_t byte);

    enum class State {
        WantHeader,
        WantCommand,
        WantData1,
        WantData2,
        WantData3,
        WantData4,
        WantData5,
        WantData6,
        WantChecksum,
        WantTail,
    };
    State state = State::WantHeader;

    uint32_t bad_chars;
    uint32_t checksum_failures;

    class Reading
    {
    public:
        uint16_t pm25; //10ug/m^3
        uint16_t pm10; //10ug/m^3
    };

    Reading reading;
    uint8_t checksum;
};
