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
#include <AP_Common/Semaphore.h>
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

AP_Compass_UAVCAN::AP_Compass_UAVCAN(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id)
    : _ap_uavcan(ap_uavcan)
    , _node_id(node_id)
    , _sensor_id(sensor_id)
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

bool AP_Compass_UAVCAN::take_registry()
{
    return _sem_registry.take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void AP_Compass_UAVCAN::give_registry()
{
    _sem_registry.give();
}

AP_Compass_Backend* AP_Compass_UAVCAN::probe()
{
    if (!take_registry()) {
        return nullptr;
    }
    AP_Compass_UAVCAN* driver = nullptr;
    for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
        if (!_detected_modules[i].driver && _detected_modules[i].ap_uavcan) {
            // Register new Compass mode to a backend
            driver = new AP_Compass_UAVCAN(_detected_modules[i].ap_uavcan, _detected_modules[i].node_id, _detected_modules[i].sensor_id);
            if (driver) {
                _detected_modules[i].driver = driver;
                driver->init();
                debug_mag_uavcan(2,
                                 _detected_modules[i].ap_uavcan->get_driver_index(),
                                 "Found Mag Node %d on Bus %d Sensor ID %d\n",
                                 _detected_modules[i].node_id,
                                 _detected_modules[i].ap_uavcan->get_driver_index(),
                                 _detected_modules[i].sensor_id);
            }
            break;
        }
    }
    give_registry();
    return driver;
}

void AP_Compass_UAVCAN::init()
{
    _instance = register_compass();

    struct DeviceStructure {
        uint8_t bus_type : 3;
        uint8_t bus: 5;
        uint8_t address;
        uint8_t devtype;
    };
    union DeviceId {
        struct DeviceStructure devid_s;
        uint32_t devid;
    };
    union DeviceId d;

    d.devid_s.bus_type = 3;
    d.devid_s.bus = _ap_uavcan->get_driver_index();
    d.devid_s.address = _node_id;
    d.devid_s.devtype = 1;

    set_dev_id(_instance, d.devid);
    set_external(_instance, true);

    debug_mag_uavcan(2, _ap_uavcan->get_driver_index(),  "AP_Compass_UAVCAN loaded\n\r");
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
                break;
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
    if (take_registry()) {
        Vector3f mag_vector;
        AP_Compass_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, 0);
        if (driver != nullptr) {
            mag_vector[0] = cb.msg->magnetic_field_ga[0];
            mag_vector[1] = cb.msg->magnetic_field_ga[1];
            mag_vector[2] = cb.msg->magnetic_field_ga[2];
            driver->handle_mag_msg(mag_vector);
        }
        give_registry();
    }
}

void AP_Compass_UAVCAN::handle_magnetic_field_2(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Mag2Cb &cb)
{
    if (take_registry()) {
        Vector3f mag_vector;
        uint8_t sensor_id = cb.msg->sensor_id;
        AP_Compass_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, sensor_id);
        if (driver != nullptr) {
            mag_vector[0] = cb.msg->magnetic_field_ga[0];
            mag_vector[1] = cb.msg->magnetic_field_ga[1];
            mag_vector[2] = cb.msg->magnetic_field_ga[2];
            driver->handle_mag_msg(mag_vector);
        }
        give_registry();
    }
}

void AP_Compass_UAVCAN::read(void)
{
    drain_accumulated_samples(_instance);
}

#endif
