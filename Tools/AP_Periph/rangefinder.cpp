#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER

/*
  rangefinder support
 */

#include <dronecan_msgs.h>

#ifndef AP_PERIPH_PROBE_CONTINUOUS
#define AP_PERIPH_PROBE_CONTINUOUS 0
#endif

/*
  update CAN rangefinder
 */
void AP_Periph_FW::can_rangefinder_update(void)
{
    if (rangefinder.get_type(0) == RangeFinder::Type::NONE) {
        return;
    }
#if AP_PERIPH_PROBE_CONTINUOUS
    if (rangefinder.num_sensors() == 0) {
        uint32_t now = AP_HAL::millis();
        static uint32_t last_probe_ms;
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            rangefinder.init(ROTATION_NONE);
        }
    }
#endif
    uint32_t now = AP_HAL::millis();
    static uint32_t last_update_ms;
    if (g.rangefinder_max_rate > 0 &&
        now - last_update_ms < uint32_t(1000/g.rangefinder_max_rate)) {
        // limit to max rate
        return;
    }
    last_update_ms = now;
    rangefinder.update();
    RangeFinder::Status status = rangefinder.status_orient(ROTATION_NONE);
    if (status <= RangeFinder::Status::NoData) {
        // don't send any data
        return;
    }
    const uint32_t sample_ms = rangefinder.last_reading_ms(ROTATION_NONE);
    if (last_sample_ms == sample_ms) {
        return;
    }
    last_sample_ms = sample_ms;

    uint16_t dist_cm = rangefinder.distance_cm_orient(ROTATION_NONE);
    uavcan_equipment_range_sensor_Measurement pkt {};
    pkt.sensor_id = rangefinder.get_address(0);
    switch (status) {
    case RangeFinder::Status::OutOfRangeLow:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;
        break;
    case RangeFinder::Status::OutOfRangeHigh:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
        break;
    case RangeFinder::Status::Good:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
        break;
    default:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
        break;
    }
    switch (rangefinder.get_mav_distance_sensor_type_orient(ROTATION_NONE)) {
    case MAV_DISTANCE_SENSOR_LASER:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
        break;
    case MAV_DISTANCE_SENSOR_ULTRASOUND:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_SONAR;
        break;
    case MAV_DISTANCE_SENSOR_RADAR:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_RADAR;
        break;
    default:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_UNDEFINED;
        break;
    }

    pkt.range = dist_cm * 0.01;

    uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE] {};
    uint16_t total_size = uavcan_equipment_range_sensor_Measurement_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                    UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}

#endif // HAL_PERIPH_ENABLE_RANGEFINDER
