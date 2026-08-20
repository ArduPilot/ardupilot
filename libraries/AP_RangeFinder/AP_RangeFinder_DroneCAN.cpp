#include "AP_RangeFinder_DroneCAN.h"

#if AP_RANGEFINDER_DRONECAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#define debug_range_finder_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { hal.console->printf(fmt, ##args); }} while (0)

const AP_Param::GroupInfo AP_RangeFinder_DroneCAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: RangeFinder DroneCAN node ID
    // @Description: DroneCAN node ID of the sensor to accept measurements from. Zero accepts the first node seen reporting the sensor ID set in ADDR.
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("RECV_ID", 10, AP_RangeFinder_DroneCAN, receive_node_id, 0),

    AP_GROUPEND
};

HAL_Semaphore AP_RangeFinder_DroneCAN::_bind_sem;

AP_RangeFinder_DroneCAN::AP_RangeFinder_DroneCAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

//links the rangefinder uavcan message to this backend
bool AP_RangeFinder_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_measurement, driver_index) != nullptr);
}

void AP_RangeFinder_DroneCAN::bind(AP_DroneCAN *ap_dronecan, uint8_t node_id, uint8_t instance)
{
    if (bound_to(ap_dronecan, node_id)) {
        return;
    }
    _ap_dronecan = ap_dronecan;
    _node_id = node_id;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RangeFinder[%u]: added DroneCAN node %u addr %u",
                  unsigned(instance), unsigned(node_id), unsigned(params.address.get()));
}

// find the backend for a measurement from node_id with sensor_id. An
// instance with RECV_ID set to node_id takes precedence; otherwise the
// instance already bound to node_id, or the first unbound instance
// accepting any node, is used. An instance stays bound to its node so
// two sensors reporting the same sensor_id cannot both feed it.
AP_RangeFinder_DroneCAN* AP_RangeFinder_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t sensor_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    RangeFinder &frontend = *AP::rangefinder();
    WITH_SEMAPHORE(_bind_sem);

    int8_t configured = -1;
    int8_t bound = -1;
    int8_t unbound = -1;
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if ((RangeFinder::Type)frontend.params[i].type.get() != RangeFinder::Type::UAVCAN ||
            frontend.params[i].address != sensor_id) {
            continue;
        }
        const auto *driver = (AP_RangeFinder_DroneCAN*)frontend.drivers[i];
        if (driver == nullptr || driver->_backend_type != RangeFinder::Type::UAVCAN) {
            continue;
        }
        if (driver->receive_node_id != 0) {
            if (driver->receive_node_id == node_id && configured < 0) {
                configured = i;
            }
        } else if (driver->_ap_dronecan == nullptr) {
            if (unbound < 0) {
                unbound = i;
            }
        } else if (driver->bound_to(ap_dronecan, node_id)) {
            bound = i;
        }
    }

    if (configured >= 0 && bound >= 0) {
        // RECV_ID was set at runtime to a node another instance had
        // already taken; release that instance for another node
        ((AP_RangeFinder_DroneCAN*)frontend.drivers[bound])->_ap_dronecan = nullptr;
        bound = -1;
    }
    const int8_t selected = configured >= 0 ? configured : (bound >= 0 ? bound : unbound);
    if (selected < 0) {
        return nullptr;
    }
    auto *driver = (AP_RangeFinder_DroneCAN*)frontend.drivers[selected];
    driver->bind(ap_dronecan, node_id, selected);
    return driver;
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
