#include "AP_DDS_GPS_Input.h"

#if AP_DDS_GPS_INPUT_SUB_ENABLED

#include <AP_Common/time.h>
#include <AP_GPS/AP_GPS.h>

void AP_DDS_GPS_Input::handle_gps_input(const obs_msgs_msg_UbloxPvt& msg)
{
    AP_GPS_DDS::NavSatFix pkt {};
    convert(msg, pkt);

    auto &gps = AP::gps();
    // AP_GPS::update() reads backend state under this semaphore on the main
    // thread; this runs on the DDS thread
    WITH_SEMAPHORE(gps.get_semaphore());
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        auto *backend = gps.get_dds_backend(i);
        if (backend != nullptr) {
            backend->handle_navsatfix(pkt);
        }
    }
}

void AP_DDS_GPS_Input::convert(const obs_msgs_msg_UbloxPvt& msg, AP_GPS_DDS::NavSatFix& pkt)
{
    // UBX-NAV-PVT carries iTOW but not the week; derive the week from the UTC
    // date the same way AP_GPS_Backend::make_gps_time() does for NMEA
    pkt.gps_tow_ms = uint32_t(msg.gpstime * 1000.0 + 0.5);
    pkt.gps_week = 0;
    if (msg.year >= 1980) {
        struct tm tm {};
        tm.tm_year = msg.year - 1900;
        tm.tm_mon = msg.month - 1;
        tm.tm_mday = msg.day;
        tm.tm_hour = msg.hour;
        tm.tm_min = msg.min;
        tm.tm_sec = msg.sec;
        const time_t unix_time = ap_mktime(&tm);
        const uint32_t unix_to_GPS_secs = 315964800UL;
        const uint16_t leap_seconds_unix = GPS_LEAPSECONDS_MILLIS/1000U;
        pkt.gps_week = uint16_t((unix_time + leap_seconds_unix - unix_to_GPS_secs) / AP_SEC_PER_WEEK);
    }

    // UBX fixType, mapped as AP_GPS_UBLOX maps it. UbloxPvt drops the PVT
    // flags, so DGPS and RTK solutions cannot be told apart from a plain 3D fix.
    switch (msg.fixtype) {
    case 2:
        pkt.fix_type = AP_GPS_FixType::FIX_2D;
        break;
    case 3: // 3D
    case 4: // GNSS + dead reckoning
        pkt.fix_type = AP_GPS_FixType::FIX_3D;
        break;
    default:
        pkt.fix_type = AP_GPS_FixType::NONE;
        break;
    }

    pkt.num_sats = uint8_t(constrain_int32(msg.numsv, 0, UINT8_MAX));

    pkt.latitude_deg = msg.lat;
    pkt.longitude_deg = msg.lon;
    pkt.altitude_amsl_m = float(msg.hmsl);

    pkt.have_velocity = true;
    pkt.have_vertical_velocity = true;
    pkt.velocity_ned = Vector3f(float(msg.veln), float(msg.vele), float(msg.veld));

    pkt.have_accuracy = true;
    pkt.horizontal_accuracy = float(msg.hacc);
    pkt.vertical_accuracy = float(msg.vacc);
    pkt.speed_accuracy = float(msg.sacc);

    // AP_GPS_UBLOX uses PVT's pDOP for both when NAV-DOP is not received
    pkt.have_dop = true;
    pkt.hdop = float(msg.pdop);
    pkt.vdop = float(msg.pdop);
}

#endif // AP_DDS_GPS_INPUT_SUB_ENABLED
