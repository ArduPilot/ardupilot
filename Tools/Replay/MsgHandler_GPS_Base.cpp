#include "MsgHandler_GPS_Base.h"

void MsgHandler_GPS_Base::update_from_msg_gps(uint8_t gps_offset, uint8_t *msg, bool responsible_for_relalt)
{
    uint32_t timestamp;
    require_field(msg, "T", timestamp);
    wait_timestamp(timestamp);

    Location loc;
    location_from_msg(msg, loc, "Lat", "Lng", "Alt");
    Vector3f vel;
    ground_vel_from_msg(msg, vel, "Spd", "GCrs", "VZ");

    uint8_t status = require_field_uint8_t(msg, "Status");
    gps.setHIL(gps_offset,
               (AP_GPS::GPS_Status)status,
               timestamp,
               loc,
               vel,
               require_field_uint8_t(msg, "NSats"),
               require_field_uint8_t(msg, "HDop"),
               require_field_float(msg, "VZ") != 0);
    if (status == AP_GPS::GPS_OK_FIX_3D && ground_alt_cm == 0) {
        ground_alt_cm = require_field_int32_t(msg, "Alt");
    }

    if (responsible_for_relalt) {
        // this could possibly check for the presence of "RelAlt" label?
        rel_altitude = 0.01f * require_field_int32_t(msg, "RelAlt");
    }

    dataflash.Log_Write_GPS(gps, gps_offset, rel_altitude);
}

