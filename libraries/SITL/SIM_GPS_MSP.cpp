#include "SIM_config.h"

#if AP_SIM_GPS_MSP_ENABLED

#include "SIM_GPS_MSP.h"

#include <SITL/SITL.h>

#include <time.h>

using namespace SITL;

/*
  send MSP GPS data
 */
void GPS_MSP::publish(const GPS_Data *d)
{
    struct PACKED {
        // header
        struct PACKED {
            uint8_t dollar = '$';
            uint8_t magic = 'X';
            uint8_t code = '<';
            uint8_t  flags;
            uint16_t cmd = 0x1F03; // GPS
            uint16_t size = 52;
        } hdr;
        uint8_t  instance;
        uint16_t gps_week;
        uint32_t ms_tow;
        uint8_t  fix_type;
        uint8_t  satellites_in_view;
        uint16_t horizontal_pos_accuracy;     // [cm]
        uint16_t vertical_pos_accuracy;       // [cm]
        uint16_t horizontal_vel_accuracy;     // [cm/s]
        uint16_t hdop;
        int32_t  longitude;
        int32_t  latitude;
        int32_t  msl_altitude;       // cm
        int32_t  ned_vel_north;       // cm/s
        int32_t  ned_vel_east;
        int32_t  ned_vel_down;
        uint16_t ground_course;      // deg * 100, 0..36000
        uint16_t true_yaw;           // deg * 100, values of 0..36000 are valid. 65535 = no data available
        uint16_t year;
        uint8_t  month;
        uint8_t  day;
        uint8_t  hour;
        uint8_t  min;
        uint8_t  sec;

        // footer CRC
        uint8_t crc;
    } msp_gps {};

    auto t = gps_time();
    struct timeval tv;
    simulation_timeval(&tv);
    struct tm tvd {};
    auto *tm = gmtime_r(&tv.tv_sec, &tvd);

    msp_gps.gps_week = t.week;
    msp_gps.ms_tow = t.ms;
    msp_gps.fix_type = d->have_lock?3:0;
    msp_gps.satellites_in_view = d->have_lock ? _sitl->gps_numsats[instance] : 3;
    msp_gps.horizontal_pos_accuracy = _sitl->gps_accuracy[instance]*100;
    msp_gps.vertical_pos_accuracy = _sitl->gps_accuracy[instance]*100;
    msp_gps.horizontal_vel_accuracy = 30;
    msp_gps.hdop = 100;
    msp_gps.longitude = d->longitude * 1.0e7;
    msp_gps.latitude  = d->latitude * 1.0e7;
    msp_gps.msl_altitude = d->altitude * 100;
    msp_gps.ned_vel_north = 100 * d->speedN;
    msp_gps.ned_vel_east = 100 * d->speedE;
    msp_gps.ned_vel_down = 100 * d->speedD;
    msp_gps.ground_course = ToDeg(atan2f(d->speedE, d->speedN)) * 100;
    msp_gps.true_yaw = wrap_360(d->yaw_deg)*100U; // can send 65535 for no yaw
    msp_gps.year = tm->tm_year;
    msp_gps.month = tm->tm_mon;
    msp_gps.day = tm->tm_mday;
    msp_gps.hour = tm->tm_hour;
    msp_gps.min = tm->tm_min;
    msp_gps.sec = tm->tm_sec;

    // CRC is over packet without first 3 bytes and trailing CRC byte
    msp_gps.crc = crc8_dvb_s2_update(0, (uint8_t *)&msp_gps.hdr.flags, sizeof(msp_gps)-4);

    write_to_autopilot((const char *)&msp_gps, sizeof(msp_gps));
}

#endif  // AP_SIM_GPS_MSP_ENABLED
