#include "AP_Airspeed_UAVCAN.h"

#if AP_AIRSPEED_UAVCAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/air_data/RawAirData.hpp>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "AirSpeed"

// UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(AirspeedCb, uavcan::equipment::air_data::RawAirData);

AP_Airspeed_UAVCAN::DetectedModules AP_Airspeed_UAVCAN::_detected_modules[];
HAL_Semaphore AP_Airspeed_UAVCAN::_sem_registry;

// constructor
AP_Airspeed_UAVCAN::AP_Airspeed_UAVCAN(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{}

void AP_Airspeed_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::air_data::RawAirData, AirspeedCb> *airspeed_listener;
    airspeed_listener = new uavcan::Subscriber<uavcan::equipment::air_data::RawAirData, AirspeedCb>(*node);

    const int airspeed_listener_res = airspeed_listener->start(AirspeedCb(ap_uavcan, &handle_airspeed));
    if (airspeed_listener_res < 0) {
        AP_HAL::panic("UAVCAN Airspeed subscriber start problem\n");
    }
}

AP_Airspeed_Backend* AP_Airspeed_UAVCAN::probe(AP_Airspeed &_frontend, uint8_t _instance)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_UAVCAN* backend = nullptr;

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_Airspeed_UAVCAN(_frontend, _instance);
            if (backend == nullptr) {
                AP::can().log_text(AP_CANManager::LOG_INFO, 
                                      LOG_TAG,
                                      "Failed register UAVCAN Airspeed Node %d on Bus %d\n",
                                      _detected_modules[i].node_id,
                                      _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                AP::can().log_text(AP_CANManager::LOG_INFO, 
                                      LOG_TAG,
                                      "Registered UAVCAN Airspeed Node %d on Bus %d\n",
                                      _detected_modules[i].node_id,
                                      _detected_modules[i].ap_uavcan->get_driver_index());
                backend->set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                                _detected_modules[i].ap_uavcan->get_driver_index(),
                                                                _detected_modules[i].node_id, 0));
            }
            break;
        }
    }

    return backend;
}

AP_Airspeed_UAVCAN* AP_Airspeed_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    bool detected = false;
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
            // detected
            detected = true;
            break;
        }
    }

    if (!detected) {
        for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
            if (_detected_modules[i].ap_uavcan == nullptr) {
                _detected_modules[i].ap_uavcan = ap_uavcan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }

    return nullptr;
}

void AP_Airspeed_UAVCAN::handle_airspeed(AP_UAVCAN* ap_uavcan, uint8_t node_id, const AirspeedCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_pressure = cb.msg->differential_pressure;
        if (!isnan(cb.msg->static_air_temperature) &&
            cb.msg->static_air_temperature > 0) {
            driver->_temperature = KELVIN_TO_C(cb.msg->static_air_temperature);
            driver->_have_temperature = true;
        }
        driver->_last_sample_time_ms = AP_HAL::millis();
    }
}

bool AP_Airspeed_UAVCAN::init()
{
    // always returns true
    return true;
}

bool AP_Airspeed_UAVCAN::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(_sem_airspeed);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    pressure = _pressure;

    return true;
}

bool AP_Airspeed_UAVCAN::get_temperature(float &temperature)
{
    if (!_have_temperature) {
        return false;
    }
    WITH_SEMAPHORE(_sem_airspeed);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    temperature = _temperature;

    return true;
}

#endif // AP_AIRSPEED_UAVCAN_ENABLED
