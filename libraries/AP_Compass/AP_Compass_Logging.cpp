#include <AP_Compass/AP_Compass_config.h>
#include <AP_Logger/AP_Logger_config.h>

#if AP_COMPASS_ENABLED && HAL_LOGGING_ENABLED

#include "AP_Compass.h"

#include <AP_Logger/AP_Logger.h>

void Compass::Write_Compass_instance(const uint64_t time_us, const uint8_t mag_instance)
{
    const Vector3f &mag_field = get_field(mag_instance);
    const Vector3f &mag_offsets = get_offsets(mag_instance);
    const Vector3f &mag_motor_offsets = get_motor_offsets(mag_instance);
    const struct log_MAG pkt{
        LOG_PACKET_HEADER_INIT(LOG_MAG_MSG),
        time_us         : time_us,
        instance        : mag_instance,
        mag_x           : (int16_t)mag_field.x,
        mag_y           : (int16_t)mag_field.y,
        mag_z           : (int16_t)mag_field.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z,
        health          : (uint8_t)healthy(mag_instance),
        SUS             : last_update_usec(mag_instance)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a Compass packet
void Compass::Write_Compass()
{
    const uint64_t time_us = AP_HAL::micros64();
    for (uint8_t i=0; i<get_count(); i++) {
        Write_Compass_instance(time_us, i);
    }
}

#endif  // AP_COMPASS_ENABLED && HAL_LOGGING_ENABLED
