#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_PROXIMITY

/*
  proximity support
 */

#include <dronecan_msgs.h>

void AP_Periph_FW::can_proximity_update()
{
    if (proximity.get_type(0) == AP_Proximity::Type::None) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    static uint32_t last_update_ms;
    if (g.proximity_max_rate > 0 &&
        now - last_update_ms < 1000/g.proximity_max_rate) {
        // limit to max rate
        return;
    }
    last_update_ms = now;
    proximity.update();
    AP_Proximity::Status status = proximity.get_status();
    if (status <= AP_Proximity::Status::NoData) {
        // don't send any data
        return;
    }

    ardupilot_equipment_proximity_sensor_Proximity pkt {};

    const uint8_t obstacle_count = proximity.get_obstacle_count();

    // if no objects return
    if (obstacle_count == 0) {
        return;
    }

    // calculate maximum roll, pitch values from objects
    for (uint8_t i=0; i<obstacle_count; i++) {
        if (!proximity.get_obstacle_info(i, pkt.yaw, pkt.pitch, pkt.distance)) {
            // not a valid obstacle
            continue;
        }

        pkt.sensor_id = proximity.get_address(0);

        switch (status) {
        case AP_Proximity::Status::NotConnected:
            pkt.reading_type = ARDUPILOT_EQUIPMENT_PROXIMITY_SENSOR_PROXIMITY_READING_TYPE_NOT_CONNECTED;
            break;
        case AP_Proximity::Status::Good:
            pkt.reading_type = ARDUPILOT_EQUIPMENT_PROXIMITY_SENSOR_PROXIMITY_READING_TYPE_GOOD;
            break;
        case AP_Proximity::Status::NoData:
        default:
            pkt.reading_type = ARDUPILOT_EQUIPMENT_PROXIMITY_SENSOR_PROXIMITY_READING_TYPE_NO_DATA;
            break;
        }

        dronecan->proximity_pub.broadcast(pkt);

    }
}

#endif // HAL_PERIPH_ENABLE_PROXIMITY
