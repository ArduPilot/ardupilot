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

#include "AP_RPM_config.h"

#if AP_RPM_DRONECAN_ENABLED

#include "RPM_DroneCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

AP_RPM_DroneCAN* AP_RPM_DroneCAN::_drivers[];
uint8_t AP_RPM_DroneCAN::_driver_instance;
HAL_Semaphore AP_RPM_DroneCAN::_driver_sem;

AP_RPM_DroneCAN::AP_RPM_DroneCAN(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, instance, _state)
{
    // Register self in static driver list
    WITH_SEMAPHORE(_driver_sem);
    _drivers[_driver_instance] = this;
    _driver_instance++;
}

// Subscribe to incoming rpm messages
void AP_RPM_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_rpm, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("rpm_sub");
    }
}

// Receive new CAN message
void AP_RPM_DroneCAN::handle_rpm(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_rpm_RPM &msg)
{
    WITH_SEMAPHORE(_driver_sem);

    for (uint8_t i = 0; i < _driver_instance; i++) {
        if (_drivers[i] == nullptr) {
            continue;
        }
        // Find params for this instance
        const uint8_t instance = _drivers[i]->state.instance;
        const AP_RPM_Params& params = _drivers[i]->ap_rpm._params[instance];

        if (params.dronecan_sensor_id == msg.sensor_id) {
            // Driver loaded and looking for this ID, add reading
            _drivers[i]->last_reading_ms = AP_HAL::millis();
            _drivers[i]->rpm = msg.rpm * params.scaling;

            const bool heathy = (msg.flags & DRONECAN_SENSORS_RPM_RPM_FLAGS_UNHEALTHY) == 0;
            _drivers[i]->signal_quality = heathy ? 0.5 : 0.0;
        }
    }
}

void AP_RPM_DroneCAN::update(void)
{
    WITH_SEMAPHORE(_driver_sem);

    // Update state from temporay variables
    state.last_reading_ms = last_reading_ms;
    state.signal_quality = signal_quality;
    state.rate_rpm = rpm;

    // assume we get readings at at least 1Hz, otherwise reset quality to zero
    if ((AP_HAL::millis() - state.last_reading_ms) > 1000) {
        state.signal_quality = 0;
        state.rate_rpm = 0;
    }
}

#endif // AP_RPM_DRONECAN_ENABLED
