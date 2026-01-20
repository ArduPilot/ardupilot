#include "AP_DAL_GPS.h"

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

#if HAL_MAVLINK_BINDINGS_ENABLED
// ensuring GPS_STATUS enumweration is 1:1 with mavlink when bindings are available
static_assert((uint32_t)AP_DAL_GPS::GPS_Status::NO_GPS == (uint32_t)GPS_FIX_TYPE_NO_GPS, "NO_GPS incorrect");
static_assert((uint32_t)AP_DAL_GPS::GPS_Status::NO_FIX == (uint32_t)GPS_FIX_TYPE_NO_FIX, "NO_FIX incorrect");
static_assert((uint32_t)AP_DAL_GPS::GPS_Status::GPS_OK_FIX_2D == (uint32_t)GPS_FIX_TYPE_2D_FIX, "FIX_2D incorrect");
static_assert((uint32_t)AP_DAL_GPS::GPS_Status::GPS_OK_FIX_3D == (uint32_t)GPS_FIX_TYPE_3D_FIX, "FIX_3D incorrect");
static_assert((uint32_t)AP_DAL_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS == (uint32_t)GPS_FIX_TYPE_DGPS, "FIX_DGPS incorrect");
static_assert((uint32_t)AP_DAL_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT == (uint32_t)GPS_FIX_TYPE_RTK_FLOAT, "FIX_RTK_FLOAT incorrect");
static_assert((uint32_t)AP_DAL_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED == (uint32_t)GPS_FIX_TYPE_RTK_FIXED, "FIX_RTK_FIXED incorrect");
#endif // HAL_MAVLINK_BINDINGS_ENABLED

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

        tmp_location[i] = {
            RGPJ.lat,
            RGPJ.lng,
            RGPJ.alt,
            Location::AltFrame::ABSOLUTE
        };
    }
}
