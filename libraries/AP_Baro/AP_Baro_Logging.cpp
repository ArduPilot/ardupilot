#include "AP_Baro.h"

#include <AP_Logger/AP_Logger.h>

void AP_Baro::Write_Baro_instance(uint64_t time_us, uint8_t baro_instance)
{
    const struct log_BARO pkt{
        LOG_PACKET_HEADER_INIT(LOG_BARO_MSG),
        time_us       : time_us,
        instance      : baro_instance,
        altitude      : get_altitude(baro_instance),
        pressure      : get_pressure(baro_instance),
        temperature   : (int16_t)(get_temperature(baro_instance) * 100 + 0.5f),
        climbrate     : get_climb_rate(),
        sample_time_ms: get_last_update(baro_instance),
        drift_offset  : get_baro_drift_offset(),
        ground_temp   : get_ground_temperature(),
        healthy       : (uint8_t)healthy(baro_instance)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a BARO packet
void AP_Baro::Write_Baro(void)
{
    const uint64_t time_us = AP_HAL::micros64();

    for (uint8_t i=0; i< _num_sensors; i++) {
        Write_Baro_instance(time_us, i);
    }
}
