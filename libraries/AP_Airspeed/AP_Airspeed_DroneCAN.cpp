#include "AP_Airspeed_DroneCAN.h"

#if AP_AIRSPEED_DRONECAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

AP_Airspeed_DroneCAN::DetectedModules AP_Airspeed_DroneCAN::_detected_modules[];
HAL_Semaphore AP_Airspeed_DroneCAN::_sem_registry;

bool AP_Airspeed_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_airspeed, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_indicated_airspeed, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_true_airspeed, driver_index) != nullptr)
#if AP_AIRSPEED_HYGROMETER_ENABLE
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_hygrometer, driver_index) != nullptr)
#endif
    ;
}

AP_Airspeed_Backend* AP_Airspeed_DroneCAN::probe(AP_Airspeed &_frontend, uint8_t _instance, uint32_t previous_devid)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_DroneCAN* backend = nullptr;

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_dronecan != nullptr) {
            const auto bus_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                            _detected_modules[i].ap_dronecan->get_driver_index(),
                                                            _detected_modules[i].node_id, 0);
            if (previous_devid != 0 && previous_devid != bus_id) {
                // match with previous ID only
                continue;
            }
            backend = NEW_NOTHROW AP_Airspeed_DroneCAN(_frontend, _instance);
            if (backend != nullptr) {
                _detected_modules[i].driver = backend;
                backend->set_bus_id(bus_id);
            }
            break;
        }
    }

    return backend;
}

AP_Airspeed_DroneCAN* AP_Airspeed_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_dronecan == ap_dronecan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    bool detected = false;
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].ap_dronecan == ap_dronecan && _detected_modules[i].node_id == node_id) {
            // detected
            detected = true;
            break;
        }
    }

    if (!detected) {
        for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
            if (_detected_modules[i].ap_dronecan == nullptr) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }

    return nullptr;
}

void AP_Airspeed_DroneCAN::handle_airspeed(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_RawAirData &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_pressure = msg.differential_pressure;
        if (!isnan(msg.static_air_temperature) &&
            msg.static_air_temperature > 0) {
            driver->_temperature = KELVIN_TO_C(msg.static_air_temperature);
            driver->_have_temperature = true;
        }
        driver->_last_sample_time_ms = AP_HAL::millis();
    }
}

void AP_Airspeed_DroneCAN::handle_indicated_airspeed(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_IndicatedAirspeed &msg)
{
    if (!isfinite(msg.indicated_airspeed)) {
        return;
    }

    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_direct.ias = MAX(msg.indicated_airspeed, 0);
        driver->_direct.ias_last_sample_ms = AP_HAL::millis();
        driver->_direct.received = true;
    }
}

void AP_Airspeed_DroneCAN::handle_true_airspeed(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_TrueAirspeed &msg)
{
    if (!isfinite(msg.true_airspeed)) {
        return;
    }

    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_direct.tas = MAX(msg.true_airspeed, 0);
        driver->_direct.tas_last_sample_ms = AP_HAL::millis();
        driver->_direct.received = true;
    }
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
void AP_Airspeed_DroneCAN::handle_hygrometer(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_hygrometer_Hygrometer &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_hygrometer.temperature = KELVIN_TO_C(msg.temperature);
        driver->_hygrometer.humidity = msg.humidity;
        driver->_hygrometer.last_sample_ms = AP_HAL::millis();
    }
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE

bool AP_Airspeed_DroneCAN::init()
{
    // always returns true
    return true;
}

bool AP_Airspeed_DroneCAN::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(_sem_airspeed);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 250) {
        return false;
    }

    pressure = _pressure;

    return true;
}

bool AP_Airspeed_DroneCAN::has_airspeed()
{
    bool received;
    {
        WITH_SEMAPHORE(_sem_airspeed);
        received = _direct.received;
    }
    if (received && !_direct.zero_offset_set) {
        // airspeed is not derived from differential pressure, so no
        // zero offset calibration is needed
        set_use_zero_offset();
        _direct.zero_offset_set = true;
    }
    return received;
}

/*
  return airspeed in m/s. ArduPilot airspeed is equivalent airspeed, so
  IndicatedAirspeed is used directly and preferred. TrueAirspeed is
  converted using the AHRS EAS2TAS ratio.
 */
bool AP_Airspeed_DroneCAN::get_airspeed(float &airspeed)
{
    const float eas2tas = AP::ahrs().get_EAS2TAS();

    WITH_SEMAPHORE(_sem_airspeed);

    const uint32_t now_ms = AP_HAL::millis();

    if (_direct.ias_last_sample_ms != 0 && (now_ms - _direct.ias_last_sample_ms) <= AP_AIRSPEED_DRONECAN_TIMEOUT_MS) {
        airspeed = _direct.ias;
        return true;
    }

    if (_direct.tas_last_sample_ms != 0 && (now_ms - _direct.tas_last_sample_ms) <= AP_AIRSPEED_DRONECAN_TIMEOUT_MS &&
        is_positive(eas2tas)) {
        airspeed = _direct.tas / eas2tas;
        return true;
    }

    return false;
}

bool AP_Airspeed_DroneCAN::get_temperature(float &temperature)
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

#if AP_AIRSPEED_HYGROMETER_ENABLE
/*
  return hygrometer data if available
 */
bool AP_Airspeed_DroneCAN::get_hygrometer(uint32_t &last_sample_ms, float &temperature, float &humidity)
{
    if (_hygrometer.last_sample_ms == 0) {
        return false;
    }
    WITH_SEMAPHORE(_sem_airspeed);
    last_sample_ms = _hygrometer.last_sample_ms;
    temperature = _hygrometer.temperature;
    humidity = _hygrometer.humidity;
    return true;
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE
#endif // AP_AIRSPEED_DRONECAN_ENABLED
