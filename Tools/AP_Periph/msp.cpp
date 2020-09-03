/*
  output MSP protocol from AP_Periph for ArduPilot and INAV
  Thanks to input from Konstantin Sharlaimov
 */

#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_MSP

void AP_Periph_FW::msp_init(AP_HAL::UARTDriver *_uart)
{
    msp.port.uart = _uart;
    msp.port.msp_version = MSP::MSP_V2_NATIVE;
    _uart->begin(115200, 512, 512);
}

void AP_Periph_FW::can_msp_update(void)
{
    struct PACKED {
        uint32_t msTOW;
        uint8_t  fixType;
        uint8_t  satellitesInView;
        uint16_t horizontalPosAccuracy;     // [cm]
        uint16_t verticalPosAccuracy;       // [cm]
        uint16_t horizontalVelAccuracy;     // [cm/s]
        uint16_t hdop;
        int32_t  longitude;
        int32_t  latitude;
        int32_t  mslAltitude;       // cm
        int32_t  nedVelNorth;       // cm/s
        int32_t  nedVelEast;
        int32_t  nedVelDown;
        int16_t  groundCourse;      // deg * 100
        int16_t  trueYaw;           // deg * 100, values of 0..36000 are valid. 65535 = no data available
        uint16_t year;
        uint8_t  month;
        uint8_t  day;
        uint8_t  hour;
        uint8_t  min;
        uint8_t  sec;
    } p;

    if (gps.get_type(0) == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        return;
    }
    gps.update();
    if (msp.last_gps_ms == gps.last_message_time_ms(0)) {
        return;
    }
    msp.last_gps_ms = gps.last_message_time_ms(0);

    const Location &loc = gps.location(0);
    const Vector3f &vel = gps.velocity(0);

    p.msTOW = gps.get_itow(0);
    p.fixType = uint8_t(gps.status(0));
    p.satellitesInView = gps.num_sats(0);

    float hacc=0, vacc=0, sacc=0;
    gps.horizontal_accuracy(0, hacc);
    gps.vertical_accuracy(0, vacc);
    gps.speed_accuracy(0, sacc);

    p.horizontalVelAccuracy = sacc*100;
    p.horizontalPosAccuracy = hacc*100;
    p.verticalPosAccuracy = vacc*100;
    p.hdop = gps.get_hdop(0);
    p.longitude = loc.lng;
    p.latitude = loc.lat;
    p.mslAltitude = loc.alt;
    p.nedVelNorth = vel.x*100;
    p.nedVelEast = vel.y*100;
    p.nedVelDown = vel.z*100;
    p.groundCourse = gps.ground_course(0);
    float yaw_deg=0, acc;
    if (gps.gps_yaw_deg(0, yaw_deg, acc)) {
        p.trueYaw = yaw_deg*100;
    } else {
        p.trueYaw = 65535; // unknown
    }
    uint64_t tepoch_us = gps.time_epoch_usec(0);
    time_t utc_sec = tepoch_us / (1000U * 1000U);
    struct tm* tm = gmtime(&utc_sec);

    p.year = tm->tm_year+1900;
    p.month = tm->tm_mon;
    p.day = tm->tm_mday;
    p.hour = tm->tm_hour;
    p.min = tm->tm_min;
    p.sec = tm->tm_sec;

    uint8_t out_buf[MSP_PORT_OUTBUF_SIZE] {};
    MSP::msp_packet_t pkt = {
        .buf = { .ptr = out_buf, .end = MSP_ARRAYEND(out_buf), },
        .cmd = (int16_t)MSP2_SENSOR_GPS,
        .flags = 0,
        .result = 0,
    };

    sbuf_write_data(&pkt.buf, &p, sizeof(p));
    sbuf_switch_to_reader(&pkt.buf, &out_buf[0]);
    
    MSP::msp_serial_encode(&msp.port, &pkt, MSP::MSP_V2_NATIVE, true);
}

#endif // HAL_PERIPH_ENABLE_MSP
