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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define AP_TELEMETRY_MAX_INSTANCES  1

class AP_Telemetry_Backend;

class AP_Telemetry {
public:

    friend class AP_Telemetry_Backend;
    friend class AP_Telemetry_MQTT;

    AP_Telemetry();

    // perform initialisation and create backends
    void init(const AP_SerialManager &serial_manager, const AP_AHRS &ahrs);

    // update - provide an opportunity to read/send telemetry
    void update();
    int recv_mavlink_message(mavlink_message_t *msg);

    // send text
    void send_text(const char *str);
    void send_text_fmt(const char *str, const char *fmt, ...) {}

protected:

    const AP_AHRS           *_ahrs;
    AP_Telemetry_Backend    *_drivers[AP_TELEMETRY_MAX_INSTANCES];
    uint8_t                 _num_instances;
};
