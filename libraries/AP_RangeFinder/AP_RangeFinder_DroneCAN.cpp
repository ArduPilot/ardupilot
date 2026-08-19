#include "AP_RangeFinder_DroneCAN.h"

#if AP_RANGEFINDER_DRONECAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#define debug_range_finder_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { hal.console->printf(fmt, ##args); }} while (0)

//links the rangefinder uavcan message to this backend
bool AP_RangeFinder_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_measurement, driver_index) != nullptr);
}

// find the backend bound to node_id and sensor_id, binding an unused
// instance with a matching ADDR on first contact. An instance stays
// bound so that two sensors reporting the same sensor_id cannot both
// feed it.
AP_RangeFinder_DroneCAN* AP_RangeFinder_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t sensor_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    RangeFinder &frontend = *AP::rangefinder();
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if ((RangeFinder::Type)frontend.params[i].type.get() != RangeFinder::Type::UAVCAN ||
            frontend.params[i].address != sensor_id) {
            continue;
        }
        auto *driver = (AP_RangeFinder_DroneCAN*)frontend.drivers[i];
        if (driver == nullptr || driver->_backend_type != RangeFinder::Type::UAVCAN) {
            continue;
        }
        if (driver->_ap_dronecan == ap_dronecan && driver->_node_id == node_id) {
            return driver;
        }
        if (driver->_ap_dronecan != nullptr) {
            // bound to another sensor with the same sensor_id
            continue;
        }
        driver->_ap_dronecan = ap_dronecan;
        driver->_node_id = node_id;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RangeFinder[%u]: added DroneCAN node %u addr %u",
                      unsigned(i), unsigned(node_id), unsigned(sensor_id));
        return driver;
    }
    return nullptr;
}

//Called from frontend to update with the readings received by handler
void AP_RangeFinder_DroneCAN::update()
{
    WITH_SEMAPHORE(_sem);
    if ((AP_HAL::millis() - _last_reading_ms) > 500U) {
        // if last read was more than 500ms, report NoData
        set_status(RangeFinder::Status::NoData);
        return;
    }

    if (!new_data) {
        return;
    }

    state.distance_m = _distance_m;
    state.last_reading_ms = _last_reading_ms;
    new_data = false;

    if (_status == RangeFinder::Status::Good) {
        // copy over states
        update_status();
    } else {
        // handle additional states received by measurement handler
        set_status(_status);
    }

}

//RangeFinder message handler
void AP_RangeFinder_DroneCAN::handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_range_sensor_Measurement &msg)
{
    //fetch the matching uavcan driver, node id and sensor id backend instance
    AP_RangeFinder_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, msg.sensor_id);
    if (driver == nullptr) {
        return;
    }
    WITH_SEMAPHORE(driver->_sem);
    switch (msg.reading_type) {
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE:
        {
            //update the states in backend instance
            driver->_distance_m = msg.range;
            driver->_last_reading_ms = AP_HAL::millis();
            driver->_status = RangeFinder::Status::Good;
            driver->new_data = true;
            break;
        }
        //Additional states supported by RFND message
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE:
        {
            driver->_distance_m = msg.range;
            driver->_last_reading_ms = AP_HAL::millis();
            driver->_status = RangeFinder::Status::OutOfRangeLow;
            driver->new_data = true;
            break;
        }
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR:
        {
            driver->_distance_m = msg.range;
            driver->_last_reading_ms = AP_HAL::millis();
            driver->_status = RangeFinder::Status::OutOfRangeHigh;
            driver->new_data = true;
            break;
        }
        default:
        {
            break;
        }
    }
    //copy over the sensor type of Rangefinder 
    switch (msg.sensor_type) {
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_SONAR:
        {
            driver->_sensor_type = MAV_DISTANCE_SENSOR_ULTRASOUND;
            break;
        }
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR:
        {
            driver->_sensor_type = MAV_DISTANCE_SENSOR_LASER;
            break;
        }
        case UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_RADAR:
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

#endif  // AP_RANGEFINDER_DRONECAN_ENABLED
