#include "SIM_config.h"

#if AP_SIM_GPS_NMEA_ENABLED

#include "SIM_GPS_NMEA.h"

#include <SITL/SITL.h>
#include <AP_Common/NMEA.h>
#include <AP_HAL/AP_HAL.h>

#include <time.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

/*
  formatted print of NMEA message, with checksum appended
 */
void GPS_NMEA::nmea_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    char *s = nmea_vaprintf(fmt, ap);
    va_end(ap);
    if (s != nullptr) {
        write_to_autopilot((const char*)s, strlen(s));
        free(s);
    }
}


/*
  send a new GPS NMEA packet
 */
void GPS_NMEA::publish(const GPS_Data *d)
{
    struct timeval tv;
    struct tm *tm;
    char tstring[20];
    char dstring[20];
    char lat_string[20];
    char lng_string[20];

    simulation_timeval(&tv);

    struct tm tvd {};
    tm = gmtime_r(&tv.tv_sec, &tvd);

    // format time string
    hal.util->snprintf(tstring, sizeof(tstring), "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + tv.tv_usec*1.0e-6);

    // format date string
    hal.util->snprintf(dstring, sizeof(dstring), "%02u%02u%02u", tm->tm_mday, tm->tm_mon+1, tm->tm_year % 100);

    // format latitude
    double deg = fabs(d->latitude);
    hal.util->snprintf(lat_string, sizeof(lat_string), "%02u%08.5f,%c",
             (unsigned)deg,
             (deg - int(deg))*60,
             d->latitude<0?'S':'N');

    // format longitude
    deg = fabs(d->longitude);
    hal.util->snprintf(lng_string, sizeof(lng_string), "%03u%08.5f,%c",
             (unsigned)deg,
             (deg - int(deg))*60,
             d->longitude<0?'W':'E');

    nmea_printf("$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,,",
                     tstring,
                     lat_string,
                     lng_string,
                     d->have_lock?1:0,
                     d->have_lock?_sitl->gps_numsats[instance]:3,
                     1.2,
                     d->altitude);

    const float speed_mps = d->speed_2d();
    const float speed_knots = speed_mps * M_PER_SEC_TO_KNOTS;
    const auto ground_track_deg = degrees(d->ground_track_rad());

    //$GPVTG,133.18,T,120.79,M,0.11,N,0.20,K,A*24
    nmea_printf("$GPVTG,%.2f,T,%.2f,M,%.2f,N,%.2f,K,A",
                     tstring,
                     ground_track_deg,
                     ground_track_deg,
                     speed_knots,
                     speed_knots * KNOTS_TO_METERS_PER_SECOND * 3.6);

    nmea_printf("$GPRMC,%s,%c,%s,%s,%.2f,%.2f,%s,,",
                     tstring,
                     d->have_lock?'A':'V',
                     lat_string,
                     lng_string,
                     speed_knots,
                     ground_track_deg,
                     dstring);

    if (_sitl->gps_hdg_enabled[instance] == SITL::SIM::GPS_HEADING_HDT) {
        nmea_printf("$GPHDT,%.2f,T", d->yaw_deg);
    }
    else if (_sitl->gps_hdg_enabled[instance] == SITL::SIM::GPS_HEADING_THS) {
        nmea_printf("$GPTHS,%.2f,%c,T", d->yaw_deg, d->have_lock ? 'A' : 'V');
    } else if (_sitl->gps_hdg_enabled[instance] == SITL::SIM::GPS_HEADING_KSXT) {
        // Unicore support
        // $KSXT,20211016083433.00,116.31296102,39.95817066,49.4911,223.57,-11.32,330.19,0.024,,1,3,28,27,,,,-0.012,0.021,0.020,,*2D
        nmea_printf("$KSXT,%04u%02u%02u%02u%02u%02u.%02u,%.8f,%.8f,%.4f,%.2f,%.2f,%.2f,%.2f,%.3f,%u,%u,%u,%u,,,,%.3f,%.3f,%.3f,,",
                    tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, unsigned(tv.tv_usec*1.e-4),
                    d->longitude, d->latitude,
                    d->altitude,
                    wrap_360(d->yaw_deg),
                    d->pitch_deg,
                    ground_track_deg,
                    speed_mps,
                    d->roll_deg,
                    d->have_lock?1:0, // 2=rtkfloat 3=rtkfixed,
                    3, // fixed rtk yaw solution,
                    d->have_lock?_sitl->gps_numsats[instance]:3,
                    d->have_lock?_sitl->gps_numsats[instance]:3,
                    d->speedE * 3.6,
                    d->speedN * 3.6,
                    -d->speedD * 3.6);
    }
}

#endif  // AP_SIM_GPS_NMEA_ENABLED
