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

#include "AP_RPM_config.h"

#if AP_RPM_DRONECAN_ENABLED

#include "RPM_Backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_RPM_DroneCAN : public AP_RPM_Backend
{
public:
    AP_RPM_DroneCAN(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state);

    // Subscribe to incoming rpm messages
    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);

    // update state
    void update(void) override;

private:

    // Receive new CAN message
    static void handle_rpm(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_rpm_RPM &msg);

    // Temporay variables used to update main state in update call
    float rpm;
    uint32_t last_reading_ms;
    float signal_quality;

    // Static list of drivers
    static AP_RPM_DroneCAN *_drivers[RPM_MAX_INSTANCES];
    static uint8_t _driver_instance;
    static HAL_Semaphore _driver_sem;

};

#endif // AP_RPM_DRONECAN_ENABLED
