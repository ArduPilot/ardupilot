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

#include "AP_Compass_DroneCAN.h"

#if AP_COMPASS_DRONECAN_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "COMPASS"

AP_Compass_DroneCAN::DetectedModules AP_Compass_DroneCAN::_detected_modules[];
HAL_Semaphore AP_Compass_DroneCAN::_sem_registry;

AP_Compass_DroneCAN::AP_Compass_DroneCAN(AP_DroneCAN* ap_dronecan, uint32_t devid) :
    _devid(devid)
{
}

void AP_Compass_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_magnetic_field, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("mag_sub");
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_magnetic_field_2, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("mag2_sub");
    }
}

AP_Compass_Backend* AP_Compass_DroneCAN::probe(uint8_t index)
{
    AP_Compass_DroneCAN* driver = nullptr;
    if (!_detected_modules[index].driver && _detected_modules[index].ap_dronecan) {
        WITH_SEMAPHORE(_sem_registry);
        // Register new Compass mode to a backend
        driver = new AP_Compass_DroneCAN(_detected_modules[index].ap_dronecan, _detected_modules[index].devid);
        if (driver) {
            if (!driver->init()) {
                delete driver;
                return nullptr;
            }
            _detected_modules[index].driver = driver;
            AP::can().log_text(AP_CANManager::LOG_INFO,
                                LOG_TAG,
                                "Found Mag Node %d on Bus %d Sensor ID %d\n",
                                _detected_modules[index].node_id,
                                _detected_modules[index].ap_dronecan->get_driver_index(),
                                _detected_modules[index].sensor_id);
#if AP_TEST_DRONECAN_DRIVERS
            // Scroll through the registered compasses, and set the offsets
            if (driver->_compass.get_offsets(index).is_zero()) {
                driver->_compass.set_offsets(index, AP::sitl()->mag_ofs[index]);
            }

            // we want to simulate a calibrated compass by default, so set
            // scale to 1
            AP_Param::set_default_by_name("COMPASS_SCALE", 1);
            AP_Param::set_default_by_name("COMPASS_SCALE2", 1);
            AP_Param::set_default_by_name("COMPASS_SCALE3", 1);
            driver->save_dev_id(index);
            driver->set_rotation(index, ROTATION_NONE);

            // make first compass external
            driver->set_external(index, true);
#endif
        }
    }
    return driver;
}

bool AP_Compass_DroneCAN::init()
{
    // Adding 1 is necessary to allow backward compatibility, where this field was set as 1 by default
    if (!register_compass(_devid, _instance)) {
        return false;
    }

    set_dev_id(_instance, _devid);
    set_external(_instance, true);

    AP::can().log_text(AP_CANManager::LOG_INFO, LOG_TAG,  "AP_Compass_DroneCAN loaded\n\r");
    return true;
}

AP_Compass_DroneCAN* AP_Compass_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t sensor_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    for (uint8_t i=0; i<COMPASS_MAX_BACKEND; i++) {
        if (_detected_modules[i].driver &&
            _detected_modules[i].ap_dronecan == ap_dronecan &&
            _detected_modules[i].node_id == node_id &&
            _detected_modules[i].sensor_id == sensor_id) {
            return _detected_modules[i].driver;
        }
    }

    bool already_detected = false;
    // Check if there's an empty spot for possible registration
    for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
        if (_detected_modules[i].ap_dronecan == ap_dronecan && 
            _detected_modules[i].node_id == node_id &&
            _detected_modules[i].sensor_id == sensor_id) {
            // Already Detected
            already_detected = true;
            break;
        }
    }
    if (!already_detected) {
        for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
            if (nullptr == _detected_modules[i].ap_dronecan) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                _detected_modules[i].node_id = node_id;
                _detected_modules[i].sensor_id = sensor_id;
                _detected_modules[i].devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                 ap_dronecan->get_driver_index(),
                                                 node_id,
                                                 sensor_id + 1); // we use sensor_id as devtype
                break;
            }
        }
    }

    struct DetectedModules tempslot;
    // Sort based on the node_id, larger values first
    // we do this, so that we have repeatable compass
    // registration, especially in cases of extraneous
    // CAN compass is connected.
    for (uint8_t i = 1; i < COMPASS_MAX_BACKEND; i++) {
        for (uint8_t j = i; j > 0; j--) {
            if (_detected_modules[j].node_id > _detected_modules[j-1].node_id) {
                tempslot = _detected_modules[j];
                _detected_modules[j] = _detected_modules[j-1];
                _detected_modules[j-1] = tempslot;
            }
        }
    }
    return nullptr;
}

void AP_Compass_DroneCAN::handle_mag_msg(const Vector3f &mag)
{
    Vector3f raw_field = mag * 1000.0;

    accumulate_sample(raw_field, _instance);
}

void AP_Compass_DroneCAN::handle_magnetic_field(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength& msg)
{
    WITH_SEMAPHORE(_sem_registry);

    Vector3f mag_vector;
    AP_Compass_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, 0);
    if (driver != nullptr) {
        mag_vector[0] = msg.magnetic_field_ga[0];
        mag_vector[1] = msg.magnetic_field_ga[1];
        mag_vector[2] = msg.magnetic_field_ga[2];
        driver->handle_mag_msg(mag_vector);
    }
}

void AP_Compass_DroneCAN::handle_magnetic_field_2(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength2 &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    Vector3f mag_vector;
    uint8_t sensor_id = msg.sensor_id;
    AP_Compass_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, sensor_id);
    if (driver != nullptr) {
        mag_vector[0] = msg.magnetic_field_ga[0];
        mag_vector[1] = msg.magnetic_field_ga[1];
        mag_vector[2] = msg.magnetic_field_ga[2];
        driver->handle_mag_msg(mag_vector);
    }
}

void AP_Compass_DroneCAN::read(void)
{
    drain_accumulated_samples(_instance);
}
#endif  // AP_COMPASS_DRONECAN_ENABLED
