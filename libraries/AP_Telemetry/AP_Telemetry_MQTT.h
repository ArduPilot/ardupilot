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

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_Telemetry_Backend.h"

class AP_Telemetry_MQTT : public AP_Telemetry_Backend
{
public:

    AP_Telemetry_MQTT(AP_Telemetry &frontend, AP_HAL::UARTDriver* uart);

    // update - provide an opportunity to read/send telemetry
    void update() override;

private:

    uint32_t _last_send_ms;
};
