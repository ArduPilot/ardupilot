#include "AP_DAL_GPS.h"

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

AP_DAL_GPS::AP_DAL_GPS()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RGPI); i++) {
        _RGPI[i].instance = i;
        _RGPJ[i].instance = i;
    }
}

void AP_DAL_GPS::start_frame()
{
    const auto &gps = AP::gps();

    const log_RGPH old_RGPH = _RGPH;
    _RGPH.primary_sensor = gps.primary_sensor();
    _RGPH.num_sensors = gps.num_sensors();

    WRITE_REPLAY_BLOCK_IFCHANGD(RGPH, _RGPH, old_RGPH);

    for (uint8_t i=0; i<ARRAY_SIZE(_RGPI); i++) {
        log_RGPI &RGPI = _RGPI[i];

        RGPI.status = (GPS_Status)gps.status(i);

        const uint32_t last_message_time_ms = gps.last_message_time_ms(i);
        if (last_message_time_ms == _last_logged_message_time_ms[i]) {
            // assume that no other state changes if we don't get a message.
            return;
        }
        _last_logged_message_time_ms[i] = last_message_time_ms;

        const Location &loc = gps.location(i);
        RGPI.last_message_time_ms = last_message_time_ms;
        RGPI.lat = loc.lat;
        RGPI.lng = loc.lng;
        RGPI.alt = loc.alt;
        RGPI.have_vertical_velocity = gps.have_vertical_velocity(i);

        RGPI.horizontal_accuracy_returncode = gps.horizontal_accuracy(i, RGPI.hacc);
        RGPI.vertical_accuracy_returncode = gps.vertical_accuracy(i, RGPI.vacc);
        RGPI.hdop = gps.get_hdop(i);
        RGPI.num_sats = gps.num_sats(i);
        RGPI.get_lag_returncode = gps.get_lag(i, RGPI.lag_sec);
        WRITE_REPLAY_BLOCK(RGPI, RGPI);

        log_RGPJ &RGPJ = _RGPJ[i];

        RGPJ.velocity = gps.velocity(i);
        RGPJ.speed_accuracy_returncode = gps.speed_accuracy(i, RGPJ.sacc);
        RGPJ.gps_yaw_deg_returncode = gps.gps_yaw_deg(i, RGPJ.yaw_deg, RGPJ.yaw_accuracy_deg);
        WRITE_REPLAY_BLOCK(RGPJ, RGPJ);

        // also fetch antenna offset for this frame
        antenna_offset[i] = gps.get_antenna_offset(i);
    }
}
