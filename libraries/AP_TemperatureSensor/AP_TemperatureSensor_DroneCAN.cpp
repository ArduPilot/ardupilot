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

#include "AP_TemperatureSensor_config.h"

#if AP_TEMPERATURE_SENSOR_DRONECAN_ENABLED

#include "AP_TemperatureSensor_DroneCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>

AP_TemperatureSensor_DroneCAN* AP_TemperatureSensor_DroneCAN::_drivers[];
uint8_t AP_TemperatureSensor_DroneCAN::_driver_instance;
HAL_Semaphore AP_TemperatureSensor_DroneCAN::_driver_sem;

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_TemperatureSensor_DroneCAN::var_info[] = {

    // @Param: MSG_ID
    // @DisplayName: Temperature sensor DroneCAN message ID
    // @Description: Sets the message device ID this backend listens for
    // @Range: 0 65535
    AP_GROUPINFO("MSG_ID", 1, AP_TemperatureSensor_DroneCAN, _ID, 0),

    AP_GROUPEND
};

AP_TemperatureSensor_DroneCAN::AP_TemperatureSensor_DroneCAN(AP_TemperatureSensor &front,
                                                             AP_TemperatureSensor::TemperatureSensor_State &state,
                                                             AP_TemperatureSensor_Params &params) :
    AP_TemperatureSensor_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;

    // Register self in static driver list
    WITH_SEMAPHORE(_driver_sem);
    _drivers[_driver_instance] = this;
    _driver_instance++;
}

// Subscript to incoming temperature messages
void AP_TemperatureSensor_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_temperature, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("temp_sub");
    }
}

void AP_TemperatureSensor_DroneCAN::handle_temperature(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_device_Temperature &msg)
{
    WITH_SEMAPHORE(_driver_sem);

    for (uint8_t i = 0; i < _driver_instance; i++) {
        if ((_drivers[i] != nullptr) && (_drivers[i]->_ID.get() == msg.device_id)) {
            // Driver loaded and looking for this ID, set temp
            _drivers[i]->set_temperature(KELVIN_TO_C(msg.temperature));
        }
    }
}

#endif // AP_TEMPERATURE_SENSOR_DRONECAN_ENABLED

