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

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Compass_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength2.hpp>

extern const AP_HAL::HAL& hal;

#define debug_mag_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)


// Frontend Registry Binders
UC_REGISTRY_BINDER(MagCb, uavcan::equipment::ahrs::MagneticFieldStrength);
UC_REGISTRY_BINDER(Mag2Cb, uavcan::equipment::ahrs::MagneticFieldStrength2);

AP_Compass_UAVCAN::DetectedModules AP_Compass_UAVCAN::_detected_modules[] = {0};
HAL_Semaphore AP_Compass_UAVCAN::_sem_registry;

AP_Compass_UAVCAN::AP_Compass_UAVCAN(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id, uint32_t devid)
    : _ap_uavcan(ap_uavcan)
    , _node_id(node_id)
    , _sensor_id(sensor_id)
    , _devid(devid)
{
}

void AP_Compass_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength, MagCb> *mag_listener;
    mag_listener = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength, MagCb>(*node);
    const int mag_listener_res = mag_listener->start(MagCb(ap_uavcan, &handle_magnetic_field));
    if (mag_listener_res < 0) {
        AP_HAL::panic("UAVCAN Mag subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength2, Mag2Cb> *mag2_listener;
    mag2_listener = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength2, Mag2Cb>(*node);
    const int mag2_listener_res = mag2_listener->start(Mag2Cb(ap_uavcan, &handle_magnetic_field_2));
    if (mag2_listener_res < 0) {
        AP_HAL::panic("UAVCAN Mag subscriber start problem\n\r");
        return;
    }
}

AP_Compass_Backend* AP_Compass_UAVCAN::probe(uint8_t index)
{
    AP_Compass_UAVCAN* driver = nullptr;
    if (!_detected_modules[index].driver && _detected_modules[index].ap_uavcan) {
        WITH_SEMAPHORE(_sem_registry);
        // Register new Compass mode to a backend
        driver = new AP_Compass_UAVCAN(_detected_modules[index].ap_uavcan, _detected_modules[index].node_id, _detected_modules[index].sensor_id, _detected_modules[index].devid);
        if (driver) {
            if (!driver->init()) {
                delete driver;
                return nullptr;
            }
            _detected_modules[index].driver = driver;
            debug_mag_uavcan(2,
                                _detected_modules[index].ap_uavcan->get_driver_index(),
                                "Found Mag Node %d on Bus %d Sensor ID %d\n",
                                _detected_modules[index].node_id,
                                _detected_modules[index].ap_uavcan->get_driver_index(),
                                _detected_modules[index].sensor_id);
        }
    }
    return driver;
}

bool AP_Compass_UAVCAN::init()
{
    // Adding 1 is necessary to allow backward compatibilty, where this field was set as 1 by default
    if (!register_compass(_devid, _instance)) {
        return false;
    }

    set_dev_id(_instance, _devid);
    set_external(_instance, true);

    debug_mag_uavcan(2, _ap_uavcan->get_driver_index(),  "AP_Compass_UAVCAN loaded\n\r");
    return true;
}

AP_Compass_UAVCAN* AP_Compass_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i=0; i<COMPASS_MAX_BACKEND; i++) {
        if (_detected_modules[i].driver &&
            _detected_modules[i].ap_uavcan == ap_uavcan &&
            _detected_modules[i].node_id == node_id &&
            _detected_modules[i].sensor_id == sensor_id) {
            return _detected_modules[i].driver;
        }
    }

    bool already_detected = false;
    // Check if there's an empty spot for possible registeration
    for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
        if (_detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id &&
            _detected_modules[i].sensor_id == sensor_id) {
            // Already Detected
            already_detected = true;
            break;
        }
    }
    if (!already_detected) {
        for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
            if (nullptr == _detected_modules[i].ap_uavcan) {
                _detected_modules[i].ap_uavcan = ap_uavcan;
                _detected_modules[i].node_id = node_id;
                _detected_modules[i].sensor_id = sensor_id;
                _detected_modules[i].devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                 ap_uavcan->get_driver_index(),
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

void AP_Compass_UAVCAN::handle_mag_msg(const Vector3f &mag)
{
    Vector3f raw_field = mag * 1000.0;

    accumulate_sample(raw_field, _instance);
}

void AP_Compass_UAVCAN::handle_magnetic_field(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MagCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    Vector3f mag_vector;
    AP_Compass_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, 0);
    if (driver != nullptr) {
        mag_vector[0] = cb.msg->magnetic_field_ga[0];
        mag_vector[1] = cb.msg->magnetic_field_ga[1];
        mag_vector[2] = cb.msg->magnetic_field_ga[2];
        driver->handle_mag_msg(mag_vector);
    }
}

void AP_Compass_UAVCAN::handle_magnetic_field_2(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Mag2Cb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    Vector3f mag_vector;
    uint8_t sensor_id = cb.msg->sensor_id;
    AP_Compass_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, sensor_id);
    if (driver != nullptr) {
        mag_vector[0] = cb.msg->magnetic_field_ga[0];
        mag_vector[1] = cb.msg->magnetic_field_ga[1];
        mag_vector[2] = cb.msg->magnetic_field_ga[2];
        driver->handle_mag_msg(mag_vector);
    }
}

void AP_Compass_UAVCAN::read(void)
{
    drain_accumulated_samples(_instance);
}

#endif
