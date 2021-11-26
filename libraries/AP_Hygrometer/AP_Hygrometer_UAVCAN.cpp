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

#include "AP_Hygrometer.h"

#if AP_HYGROMETER_ENABLED

#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include <AP_HAL/AP_HAL.h>
#include "AP_Hygrometer_UAVCAN.h"
#include <dronecan/sensors/hygrometer/Hygrometer.hpp>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define LOG_TAG "Hygrometer"
#define C_TO_KELVIN 273.15f

UC_REGISTRY_BINDER(HygrometerCb, dronecan::sensors::hygrometer::Hygrometer);

AP_Hygrometer_UAVCAN::DetectedModules AP_Hygrometer_UAVCAN::_detected_modules[];
HAL_Semaphore AP_Hygrometer_UAVCAN::_sem_registry;

AP_Hygrometer_UAVCAN::AP_Hygrometer_UAVCAN(AP_Hygrometer::Param &_params) :
    AP_Hygrometer_Backend(_params)
{}

void AP_Hygrometer_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<dronecan::sensors::hygrometer::Hygrometer, HygrometerCb> *hygrometer_listener;
    hygrometer_listener = new uavcan::Subscriber<dronecan::sensors::hygrometer::Hygrometer, HygrometerCb>(*node);
    if(hygrometer_listener == nullptr) {
        AP_BoardConfig::allocation_error("AP_Hygrometer_UAVCAN: hygrometer_listener");
    }
    const int hygrometer_listener_res = hygrometer_listener->start(HygrometerCb(ap_uavcan, &handle_hygrometer));
    if (hygrometer_listener_res < 0) {
        AP_HAL::panic("UAVCAN hygrometer subscriber start problem");
    }
}

AP_Hygrometer_Backend* AP_Hygrometer_UAVCAN::probe(AP_Hygrometer::Param &_params)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Hygrometer_UAVCAN* backend = nullptr;

    for (uint8_t i = 0; i < AP_HYGROMETER_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_Hygrometer_UAVCAN(_params);
            if (backend == nullptr) {
                AP::can().log_text(AP_CANManager::LOG_INFO,
                                   LOG_TAG,
                                   "Failed register UAVCAN Hygrometer Node %d on Bus %d\n",
                                   _detected_modules[i].node_id,
                                   _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;  
                AP::can().log_text(AP_CANManager::LOG_INFO,
                                   LOG_TAG,
                                   "Registered UAVCAN Hygrometer Node %d on Bus %d\n",
                                   _detected_modules[i].node_id,
                                   _detected_modules[i].ap_uavcan->get_driver_index());
            }
            break;
        }
    }
    return backend;
}

AP_Hygrometer_UAVCAN* AP_Hygrometer_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < AP_HYGROMETER_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    bool detected = false;
    for (uint8_t i = 0; i < AP_HYGROMETER_MAX_SENSORS; i++) {
        if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
            detected = true;
            break;
        }
    }

    if (!detected) {
        for (uint8_t i = 0; i < AP_HYGROMETER_MAX_SENSORS; i++) {
            if (_detected_modules[i].ap_uavcan == nullptr) {
                _detected_modules[i].ap_uavcan = ap_uavcan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }

    return nullptr;
}

void AP_Hygrometer_UAVCAN::handle_hygrometer(AP_UAVCAN* ap_uavcan, uint8_t node_id, const HygrometerCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Hygrometer_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);

    if (driver == nullptr) {
        return;
    }
    WITH_SEMAPHORE(driver->_sem_hygrometer);
    driver->_humidity = cb.msg->humidity;
    if (!isnan(cb.msg->temperature))  {
        driver->_temperature = cb.msg->temperature - C_TO_KELVIN;
    }
    driver->_last_sample_time_ms = AP_HAL::millis();
}

bool AP_Hygrometer_UAVCAN::get_humidity(float &humidity)
{
    WITH_SEMAPHORE(_sem_hygrometer);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 200) {
        return false;
    }
    humidity = _humidity;

    return true;
}

bool AP_Hygrometer_UAVCAN::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(_sem_hygrometer);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 200) {
        return false;
    }
    temperature = _temperature;

    return true;
}

#endif // HAL_ENABLE_LIBUAVCAN_DRIVERS

#endif // AP_HYGROMETER_ENABLED
