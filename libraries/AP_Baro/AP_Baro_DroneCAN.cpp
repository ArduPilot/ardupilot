#include "AP_Baro_DroneCAN.h"

#if AP_BARO_DRONECAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Baro_SITL.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "Baro"

AP_Baro_DroneCAN::DetectedModules AP_Baro_DroneCAN::_detected_modules[];
HAL_Semaphore AP_Baro_DroneCAN::_sem_registry;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_DroneCAN::AP_Baro_DroneCAN(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{}

bool AP_Baro_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_pressure, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_temperature, driver_index) != nullptr)
    ;
}

AP_Baro_Backend* AP_Baro_DroneCAN::probe(AP_Baro &baro)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Baro_DroneCAN* backend = nullptr;
    for (auto &detected_module : _detected_modules) {
        if (detected_module.driver == nullptr && detected_module.ap_dronecan != nullptr) {
            backend = NEW_NOTHROW AP_Baro_DroneCAN(baro);
            if (backend == nullptr) {
                AP::can().log_text(AP_CANManager::LOG_ERROR,
                            LOG_TAG,
                            "Failed register DroneCAN Baro Node %d on Bus %d\n",
                            detected_module.node_id,
                            detected_module.ap_dronecan->get_driver_index());
            } else {
                detected_module.driver = backend;
                backend->_pressure = 0;
                backend->_pressure_count = 0;
                backend->_ap_dronecan = detected_module.ap_dronecan;
                backend->_node_id = detected_module.node_id;

                backend->_instance = backend->_frontend.register_sensor();
                backend->set_bus_id(backend->_instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                                                    detected_module.ap_dronecan->get_driver_index(),
                                                                                    backend->_node_id, 0));

                AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered DroneCAN Baro Node %d on Bus %d\n",
                            detected_module.node_id,
                            detected_module.ap_dronecan->get_driver_index());
            }
            break;
        }
    }
    return backend;
}

AP_Baro_DroneCAN* AP_Baro_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, bool create_new)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    for (auto &detected_module : _detected_modules) {
        if (detected_module.driver != nullptr &&
            detected_module.ap_dronecan == ap_dronecan &&
            detected_module.node_id == node_id) {
            return detected_module.driver;
        }
    }
    
    if (create_new) {
        bool already_detected = false;
        //Check if there's an empty spot for possible registration
        for (auto &detected_module : _detected_modules) {
            if (detected_module.ap_dronecan == ap_dronecan && detected_module.node_id == node_id) {
                //Already Detected
                already_detected = true;
                break;
            }
        }
        if (!already_detected) {
            for (auto &detected_module : _detected_modules) {
                if (detected_module.ap_dronecan == nullptr) {
                    detected_module.ap_dronecan = ap_dronecan;
                    detected_module.node_id = node_id;
                    break;
                }
            }
        }
    }

    return nullptr;
}


void AP_Baro_DroneCAN::_update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count)
{
    *accum += val;
    *count += 1;
    if (*count == max_count) {
        *count = max_count / 2;
        *accum = *accum / 2;
    }
}

void AP_Baro_DroneCAN::handle_pressure(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticPressure &msg)
{
    AP_Baro_DroneCAN* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, true);
        if (driver == nullptr) {
            return;
        }
    }
    {
        WITH_SEMAPHORE(driver->_sem_baro);
        _update_and_wrap_accumulator(&driver->_pressure, msg.static_pressure, &driver->_pressure_count, 32);
        driver->new_pressure = true;
    }
}

void AP_Baro_DroneCAN::handle_temperature(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticTemperature &msg)
{
    AP_Baro_DroneCAN* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, false);
        if (driver == nullptr) {
            return;
        }
    }
    {
        WITH_SEMAPHORE(driver->_sem_baro);
        driver->_temperature = KELVIN_TO_C(msg.static_temperature);
    }
}

// Read the sensor
void AP_Baro_DroneCAN::update(void)
{
    float pressure = 0;

    WITH_SEMAPHORE(_sem_baro);
    if (new_pressure) {
        if (_pressure_count != 0) {
            pressure = _pressure / _pressure_count;
            _pressure_count = 0;
            _pressure = 0;
        }
        _copy_to_frontend(_instance, pressure, _temperature);


        _frontend.set_external_temperature(_temperature);
        new_pressure = false;
    }
}

#endif // AP_BARO_DRONECAN_ENABLED
