#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_Baro_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>

extern const AP_HAL::HAL& hal;

#define debug_baro_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)

//UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(PressureCb, uavcan::equipment::air_data::StaticPressure);
UC_REGISTRY_BINDER(TemperatureCb, uavcan::equipment::air_data::StaticTemperature);

AP_Baro_UAVCAN::DetectedModules AP_Baro_UAVCAN::_detected_modules[] = {0};
HAL_Semaphore AP_Baro_UAVCAN::_sem_registry;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_UAVCAN::AP_Baro_UAVCAN(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{}

void AP_Baro_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure, PressureCb> *pressure_listener;
    pressure_listener = new uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure, PressureCb>(*node);
    // Msg Handler
    const int pressure_listener_res = pressure_listener->start(PressureCb(ap_uavcan, &handle_pressure));
    if (pressure_listener_res < 0) {
        AP_HAL::panic("UAVCAN Baro subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature, TemperatureCb> *temperature_listener;
    temperature_listener = new uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature, TemperatureCb>(*node);
    // Msg Handler
    const int temperature_listener_res = temperature_listener->start(TemperatureCb(ap_uavcan, &handle_temperature));
    if (temperature_listener_res < 0) {
        AP_HAL::panic("UAVCAN Baro subscriber start problem\n\r");
        return;
    }
}

AP_Baro_Backend* AP_Baro_UAVCAN::probe(AP_Baro &baro)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Baro_UAVCAN* backend = nullptr;
    for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_Baro_UAVCAN(baro);
            if (backend == nullptr) {
                debug_baro_uavcan(2,
                                  _detected_modules[i].ap_uavcan->get_driver_index(),
                                  "Failed register UAVCAN Baro Node %d on Bus %d\n",
                                  _detected_modules[i].node_id,
                                  _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                backend->_ap_uavcan = _detected_modules[i].ap_uavcan;
                backend->_node_id = _detected_modules[i].node_id;
                backend->register_sensor();
                debug_baro_uavcan(2,
                                  _detected_modules[i].ap_uavcan->get_driver_index(),
                                  "Registered UAVCAN Baro Node %d on Bus %d\n",
                                  _detected_modules[i].node_id,
                                  _detected_modules[i].ap_uavcan->get_driver_index());
            }
            break;
        }
    }
    return backend;
}

AP_Baro_UAVCAN* AP_Baro_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }
    
    if (create_new) {
        bool already_detected = false;
        //Check if there's an empty spot for possible registeration
        for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
            if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
                //Already Detected
                already_detected = true;
                break;
            }
        }
        if (!already_detected) {
            for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
                if (_detected_modules[i].ap_uavcan == nullptr) {
                    _detected_modules[i].ap_uavcan = ap_uavcan;
                    _detected_modules[i].node_id = node_id;
                    break;
                }
            }
        }
    }

    return nullptr;
}

void AP_Baro_UAVCAN::handle_pressure(AP_UAVCAN* ap_uavcan, uint8_t node_id, const PressureCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

        AP_Baro_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, true);
        if (driver == nullptr) {
            return;
        }
        {
            WITH_SEMAPHORE(driver->_sem_baro);
            driver->_pressure = cb.msg->static_pressure;
            driver->new_pressure = true;
        }
}

void AP_Baro_UAVCAN::handle_temperature(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TemperatureCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

        AP_Baro_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, false);
        if (driver == nullptr) {
            return;
        }
        {
            WITH_SEMAPHORE(driver->_sem_baro);
            driver->_temperature = cb.msg->static_temperature - C_TO_KELVIN;
        }
}

// Read the sensor
void AP_Baro_UAVCAN::update(void)
{
    WITH_SEMAPHORE(_sem_baro);
    if (new_pressure) {
        _copy_to_frontend(_instance, _pressure, _temperature);

        _frontend.set_external_temperature(_temperature);
        new_pressure = false;
    }
}

#endif // HAL_WITH_UAVCAN

