#include "AP_Periph.h"

#if AP_PERIPH_RANGEFINDER_ENABLED

/*
  rangefinder support
 */

#include <dronecan_msgs.h>

#ifndef AP_PERIPH_PROBE_CONTINUOUS
#define AP_PERIPH_PROBE_CONTINUOUS 0
#endif

extern const AP_HAL::HAL &hal;

/*
  update CAN rangefinder
 */
void AP_Periph_FW::can_rangefinder_update(void)
{
    if (rangefinder.get_type(0) == RangeFinder::Type::NONE) {
        return;
    }
#if AP_PERIPH_PROBE_CONTINUOUS
    // We only allow continuous probing for rangefinders while vehicle is disarmed. Probing is currently inefficient and leads to longer loop times.
    if ((rangefinder.num_sensors() == 0) && !hal.util->get_soft_armed() && option_is_set(PeriphOptions::PROBE_CONTINUOUS)) {
        uint32_t now = AP_HAL::millis();
        static uint32_t last_probe_ms;
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            rangefinder.init(ROTATION_NONE);
        }
    }
#endif
    uint32_t now = AP_HAL::millis();
    if (g.rangefinder_max_rate > 0 &&
        now - last_rangefinder_update_ms < uint32_t(1000/g.rangefinder_max_rate)) {
        // limit to max rate
        return;
    }
    last_rangefinder_update_ms = now;

    // update all rangefinder instances
    rangefinder.update();

    // cycle through each rangefinder instance to find one to send
    // equipment.range_sensor only uses 3 CAN frames so we just send all available sensor measurements.
    for (uint8_t i = 0; i <= rangefinder.num_sensors(); i++) {

        if (rangefinder.get_type(i) == RangeFinder::Type::NONE) {
            continue;
        }

        AP_RangeFinder_Backend *backend = rangefinder.get_backend(i);
        if (backend == nullptr) {
            continue;
        }

        RangeFinder::Status status = backend->status();
        if (status <= RangeFinder::Status::NoData) {
            // don't send any data for this instance
            continue;
        }

        const uint32_t sample_ms = backend->last_reading_ms();
        if (last_rangefinder_sample_ms[i] == sample_ms) {
            // don't same the same reading again
            continue;
        }
        last_rangefinder_sample_ms[i] = sample_ms;

        uavcan_equipment_range_sensor_Measurement pkt {};
        pkt.sensor_id = rangefinder.get_address(i);

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

        switch (backend->get_mav_distance_sensor_type()) {
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

        float dist_m = backend->distance();
        pkt.range = dist_m;

        uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_range_sensor_Measurement_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                        UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);

    }
}

#endif // AP_PERIPH_RANGEFINDER_ENABLED
