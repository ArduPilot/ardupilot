#include "AP_DAL_GPS.h"

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

// we use a static here as the "location" accessor wants to be const
static Location tmp_location[GPS_MAX_INSTANCES];

AP_DAL_GPS::AP_DAL_GPS()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RGPI); i++) {
        _RGPI[i].instance = i;
        _RGPJ[i].instance = i;
    }
}

const Location &AP_DAL_GPS::location(uint8_t instance) const
{
    Location &loc = tmp_location[instance];
    loc.lat = _RGPJ[instance].lat;
    loc.lng = _RGPJ[instance].lng;
    loc.alt = _RGPJ[instance].alt;
    return loc;
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

        RGPI.status = (GPS_Status)gps.status(i);
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

        WRITE_REPLAY_BLOCK_IFCHANGED(RGPI, RGPI, old_RGPI);
        WRITE_REPLAY_BLOCK_IFCHANGED(RGPJ, RGPJ, old_RGPJ);
    }
}
