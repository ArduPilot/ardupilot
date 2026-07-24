#include "AP_DAL_GPS.h"

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

AP_DAL_GPS::AP_DAL_GPS()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RGPI); i++) {
        _RGPI[i].instance = i;
        _RGPJ[i].instance = i;
        // _RGPK defaults to a zero offset so replay of older logs with no
        // RGPK messages applies no moving baseline yaw correction
        _RGPK[i].instance = i;
    }
}

void AP_DAL_GPS::start_frame()
{
    const auto &gps = AP::gps();

    const log_RGPH old_RGPH = _RGPH;
    _RGPH.primary_sensor = gps.primary_sensor();
    _RGPH.num_sensors = gps.num_sensors();

    WRITE_REPLAY_BLOCK_IFCHANGED(RGPH, _RGPH, old_RGPH);

    for (uint8_t i=0; i<ARRAY_SIZE(_RGPI); i++) {
        log_RGPI &RGPI = _RGPI[i];
        log_RGPJ &RGPJ = _RGPJ[i];
        const log_RGPI old_RGPI = RGPI;
        const log_RGPJ old_RGPJ = RGPJ;

        RGPI.status = (uint8_t)gps.status(i);
        RGPI.antenna_offset = gps.get_antenna_offset(i);

        const Location &loc = gps.location(i);
        RGPJ.last_message_time_ms = gps.last_message_time_ms(i);
        RGPJ.lat = loc.lat;
        RGPJ.lng = loc.lng;
        RGPJ.alt = loc.alt;
        RGPI.have_vertical_velocity = gps.have_vertical_velocity(i);

        RGPI.horizontal_accuracy_returncode = gps.horizontal_accuracy(i, RGPJ.hacc);
        RGPI.vertical_accuracy_returncode = gps.vertical_accuracy(i, RGPJ.vacc);
        RGPJ.hdop = gps.get_hdop(i);
        RGPI.num_sats = gps.num_sats(i);
        RGPI.get_lag_returncode = gps.get_lag(i, RGPI.lag_sec);

        RGPJ.velocity = gps.velocity(i);
        RGPI.speed_accuracy_returncode = gps.speed_accuracy(i, RGPJ.sacc);
        RGPI.gps_yaw_deg_returncode = gps.gps_yaw_deg(i, RGPJ.yaw_deg, RGPJ.yaw_accuracy_deg, RGPJ.yaw_deg_time_ms);

        // RGPK holds the body-frame antenna offset used to calculate gps_yaw.
        // The offset is derived from parameters but is zeroed while gps_yaw is
        // not calculated from a moving baseline, so it tracks the active yaw
        // source and yaw validity as well as parameter changes. All of these
        // change rarely, so the message is only written when changed.
        log_RGPK &RGPK = _RGPK[i];
        const log_RGPK old_RGPK = RGPK;
        RGPK.mb_yaw_offset = gps.get_mb_yaw_offset(i);

        WRITE_REPLAY_BLOCK_IFCHANGED(RGPI, RGPI, old_RGPI);
        WRITE_REPLAY_BLOCK_IFCHANGED(RGPJ, RGPJ, old_RGPJ);
        WRITE_REPLAY_BLOCK_IFCHANGED(RGPK, RGPK, old_RGPK);

        tmp_location[i] = {
            RGPJ.lat,
            RGPJ.lng,
            RGPJ.alt,
            Location::AltFrame::ABSOLUTE
        };
    }
}
