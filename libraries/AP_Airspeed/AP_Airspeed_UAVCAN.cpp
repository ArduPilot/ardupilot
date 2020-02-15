#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Airspeed_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/air_data/RawAirData.hpp>

extern const AP_HAL::HAL& hal;

#define debug_airspeed_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)

// UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(AirspeedCb, uavcan::equipment::air_data::RawAirData);

AP_Airspeed_UAVCAN::DetectedModules AP_Airspeed_UAVCAN::_detected_modules[] = {0};
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
                debug_airspeed_uavcan(2,
                                      _detected_modules[i].ap_uavcan->get_driver_index(),
                                      "Failed register UAVCAN Airspeed Node %d on Bus %d\n",
                                      _detected_modules[i].node_id,
                                      _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                debug_airspeed_uavcan(2,
                                      _detected_modules[i].ap_uavcan->get_driver_index(),
                                      "Registered UAVCAN Airspeed Node %d on Bus %d\n",
                                      _detected_modules[i].node_id,
                                      _detected_modules[i].ap_uavcan->get_driver_index());
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
        if (!isnan(cb.msg->static_air_temperature)) {
            driver->_temperature = cb.msg->static_air_temperature - C_TO_KELVIN;
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

#endif // HAL_WITH_UAVCAN
