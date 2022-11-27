#include "AP_RangeFinder_UAVCAN.h"

#if AP_RANGEFINDER_UAVCAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <GCS_MAVLink/GCS.h>

#include <uavcan/equipment/range_sensor/Measurement.hpp>

extern const AP_HAL::HAL& hal;

#define debug_range_finder_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { hal.console->printf(fmt, ##args); }} while (0)

//UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(MeasurementCb, uavcan::equipment::range_sensor::Measurement);

//links the rangefinder uavcan message to this backend
void AP_RangeFinder_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::range_sensor::Measurement, MeasurementCb> *measurement_listener;
    measurement_listener = new uavcan::Subscriber<uavcan::equipment::range_sensor::Measurement, MeasurementCb>(*node);
    // Register method to handle incoming RangeFinder measurement
    const int measurement_listener_res = measurement_listener->start(MeasurementCb(ap_uavcan, &handle_measurement));
    if (measurement_listener_res < 0) {
        AP_HAL::panic("UAVCAN RangeFinder subscriber start problem\n\r");
        return;
    }
}

//Method to find the backend relating to the node id
AP_RangeFinder_UAVCAN* AP_RangeFinder_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t address, bool create_new)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    AP_RangeFinder_UAVCAN* driver = nullptr;
    RangeFinder &frontend = *AP::rangefinder();
    //Scan through the Rangefinder params to find UAVCAN RFND with matching address.
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if ((RangeFinder::Type)frontend.params[i].type.get() == RangeFinder::Type::UAVCAN &&
            frontend.params[i].address == address) {
            driver = (AP_RangeFinder_UAVCAN*)frontend.drivers[i];
        }
        //Double check if the driver was initialised as UAVCAN Type
        if (driver != nullptr && (driver->_backend_type == RangeFinder::Type::UAVCAN)) {
            if (driver->_ap_uavcan == ap_uavcan && 
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
        for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
            if ((RangeFinder::Type)frontend.params[i].type.get() == RangeFinder::Type::UAVCAN &&
                frontend.params[i].address == address) {
                WITH_SEMAPHORE(frontend.detect_sem);
                if (frontend.drivers[i] != nullptr) {
                    //we probably initialised this driver as something else, reboot is required for setting
                    //it up as UAVCAN type
                    return nullptr;
                }
                frontend.drivers[i] = new AP_RangeFinder_UAVCAN(frontend.state[i], frontend.params[i]);
                driver = (AP_RangeFinder_UAVCAN*)frontend.drivers[i];
                if (driver == nullptr) {
                    break;
                }
                gcs().send_text(MAV_SEVERITY_INFO, "RangeFinder[%u]: added UAVCAN node %u addr %u",
                                unsigned(i), unsigned(node_id), unsigned(address));
                //Assign node id and respective uavcan driver, for identification
                if (driver->_ap_uavcan == nullptr) {
                    driver->_ap_uavcan = ap_uavcan;
                    driver->_node_id = node_id;
                    break;
                }
            }
        }
    }

    return driver;
}

//Called from frontend to update with the readings received by handler
void AP_RangeFinder_UAVCAN::update()
{
    WITH_SEMAPHORE(_sem);
    if ((AP_HAL::millis() - _last_reading_ms) > 500) {
        //if data is older than 500ms, report NoData
        set_status(RangeFinder::Status::NoData);
    } else if (_status == RangeFinder::Status::Good && new_data) {
        //copy over states
        state.distance_m = _distance_cm * 0.01f;
        state.last_reading_ms = _last_reading_ms;
        update_status();
        new_data = false;
    } else if (_status != RangeFinder::Status::Good) {
        //handle additional states received by measurement handler
        set_status(_status);
    }
}

//RangeFinder message handler
void AP_RangeFinder_UAVCAN::handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb)
{
    //fetch the matching uavcan driver, node id and sensor id backend instance
    AP_RangeFinder_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->sensor_id, true);
    if (driver == nullptr) {
        return;
    }
    WITH_SEMAPHORE(driver->_sem);
    switch (cb.msg->reading_type) {
        case uavcan::equipment::range_sensor::Measurement::READING_TYPE_VALID_RANGE:
        {
            //update the states in backend instance
            driver->_distance_cm = cb.msg->range*100.0f;
            driver->_last_reading_ms = AP_HAL::millis();
            driver->_status = RangeFinder::Status::Good;
            driver->new_data = true;
            break;
        }
        //Additional states supported by RFND message
        case uavcan::equipment::range_sensor::Measurement::READING_TYPE_TOO_CLOSE:
        {
            driver->_last_reading_ms = AP_HAL::millis();
            driver->_status = RangeFinder::Status::OutOfRangeLow;
            break;
        }
        case uavcan::equipment::range_sensor::Measurement::READING_TYPE_TOO_FAR:
        {
            driver->_last_reading_ms = AP_HAL::millis();
            driver->_status = RangeFinder::Status::OutOfRangeHigh;
            break;
        }
        default:
        {
            break;
        }
    }
    //copy over the sensor type of Rangefinder 
    switch (cb.msg->sensor_type) {
        case uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_SONAR:
        {
            driver->_sensor_type = MAV_DISTANCE_SENSOR_ULTRASOUND;
            break;
        }
        case uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_LIDAR:
        {
            driver->_sensor_type = MAV_DISTANCE_SENSOR_LASER;
            break;
        }
        case uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_RADAR:
        {
            driver->_sensor_type = MAV_DISTANCE_SENSOR_RADAR;
            break;
        }
        default:
        {
            driver->_sensor_type = MAV_DISTANCE_SENSOR_UNKNOWN;
            break;
        }
    }
}

#endif  // AP_RANGEFINDER_UAVCAN_ENABLED
