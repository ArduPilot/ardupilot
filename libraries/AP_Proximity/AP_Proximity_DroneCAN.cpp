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

#include "AP_Proximity_DroneCAN.h"

#if AP_PROXIMITY_DRONECAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;


ObjectBuffer_TS<AP_Proximity_DroneCAN::ObstacleItem> AP_Proximity_DroneCAN::items(50);

#define PROXIMITY_TIMEOUT_MS    500 // distance messages must arrive within this many milliseconds


//links the Proximity DroneCAN message to this backend
void AP_Proximity_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_measurement, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("measurement_sub");
    }
}

//Method to find the backend relating to the node id
AP_Proximity_DroneCAN* AP_Proximity_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t address, bool create_new)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }

    AP_Proximity *prx = AP::proximity();
    if (prx == nullptr) {
        return nullptr;
    }

    AP_Proximity_DroneCAN* driver = nullptr;
    //Scan through the proximity type params to find DroneCAN with matching address.
    for (uint8_t i = 0; i < PROXIMITY_MAX_INSTANCES; i++) {
        if ((AP_Proximity::Type)prx->params[i].type.get() == AP_Proximity::Type::DroneCAN &&
            prx->params[i].address == address) {
            driver = (AP_Proximity_DroneCAN*)prx->drivers[i];
        }
        //Double check if the driver was initialised as DroneCAN Type
        if (driver != nullptr && (driver->_backend_type == AP_Proximity::Type::DroneCAN)) {
            if (driver->_ap_dronecan == ap_dronecan &&
                driver->_node_id == node_id) {
                return driver;
            } else {
                //we found a possible duplicate addressed sensor
                //we return nothing in such scenario
                return nullptr;
            }
        }
    }

    if (create_new) {
        for (uint8_t i = 0; i < PROXIMITY_MAX_INSTANCES; i++) {
            if ((AP_Proximity::Type)prx->params[i].type.get() == AP_Proximity::Type::DroneCAN &&
                prx->params[i].address == address) {
                WITH_SEMAPHORE(prx->detect_sem);
                if (prx->drivers[i] != nullptr) {
                    //we probably initialised this driver as something else, reboot is required for setting
                    //it up as DroneCAN type
                    return nullptr;
                }
                prx->drivers[i] = new AP_Proximity_DroneCAN(*prx, prx->state[i], prx->params[i]);
                driver = (AP_Proximity_DroneCAN*)prx->drivers[i];
                if (driver == nullptr) {
                    break;
                }
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Prx[%u]: added DroneCAN node %u addr %u",
                                unsigned(i), unsigned(node_id), unsigned(address));

                if (is_zero(prx->params[i].max_m) && is_zero(prx->params[i].min_m)) {
                    // GCS reporting will be incorrect if min/max are not set
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Configure PRX%u_MIN and PRX%u_MAX",
                                unsigned(i), unsigned(i));
                }
                //Assign node id and respective dronecan driver, for identification
                if (driver->_ap_dronecan == nullptr) {
                    driver->_ap_dronecan = ap_dronecan;
                    driver->_node_id = node_id;
                    break;
                }
            }
        }
    }

    return driver;
}


// update the state of the sensor
void AP_Proximity_DroneCAN::update(void)
{
    // check for timeout and set health status
    if ((_last_update_ms == 0 || (AP_HAL::millis() - _last_update_ms > PROXIMITY_TIMEOUT_MS))) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(_status);
    }

    if (_status == AP_Proximity::Status::Good) {
        ObstacleItem object_item;
        WITH_SEMAPHORE(_sem);
        while (items.pop(object_item)) {
            const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(object_item.pitch_deg, object_item.yaw_deg);
            if (!is_zero(object_item.distance_m) && !ignore_reading(object_item.pitch_deg, object_item.yaw_deg, object_item.distance_m, false)) {
                // update boundary used for avoidance
                frontend.boundary.set_face_attributes(face, object_item.pitch_deg, object_item.yaw_deg, object_item.distance_m, state.instance);
                // update OA database
                database_push(object_item.pitch_deg, object_item.yaw_deg, object_item.distance_m);
            }
        }
    }
}

// get maximum and minimum distances (in meters)
float AP_Proximity_DroneCAN::distance_max() const
{
    if (is_zero(params.max_m)) {
        // GCS will not report correct correct value if max isn't set properly
        // This is a arbitrary value to prevent the above issue
        return 100.0f;
    }
    return params.max_m;
}

float AP_Proximity_DroneCAN::distance_min() const
{
    return params.min_m;
}

//Proximity message handler
void AP_Proximity_DroneCAN::handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_proximity_sensor_Proximity &msg)
{
    //fetch the matching DroneCAN driver, node id and sensor id backend instance
    AP_Proximity_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, msg.sensor_id, true);
    if (driver == nullptr) {
        return;
    }
    WITH_SEMAPHORE(driver->_sem);
    switch (msg.reading_type) {
        case ARDUPILOT_EQUIPMENT_PROXIMITY_SENSOR_PROXIMITY_READING_TYPE_GOOD: {
            //update the states in backend instance
            driver->_last_update_ms = AP_HAL::millis();
            driver->_status = AP_Proximity::Status::Good;
            const ObstacleItem item = {msg.yaw, msg.pitch, msg.distance};

            if (driver->items.space()) {
                // ignore reading if no place to put it in the queue
                driver->items.push(item);
            }
            break;
        }
        //Additional states supported by Proximity message
        case ARDUPILOT_EQUIPMENT_PROXIMITY_SENSOR_PROXIMITY_READING_TYPE_NOT_CONNECTED: {
            driver->_last_update_ms = AP_HAL::millis();
            driver->_status = AP_Proximity::Status::NotConnected;
            break;
        }
        case ARDUPILOT_EQUIPMENT_PROXIMITY_SENSOR_PROXIMITY_READING_TYPE_NO_DATA: {
            driver->_last_update_ms = AP_HAL::millis();
            driver->_status = AP_Proximity::Status::NoData;
            break;
        }
        default:
            break;
    }
}

#endif // AP_PROXIMITY_DRONECAN_ENABLED
