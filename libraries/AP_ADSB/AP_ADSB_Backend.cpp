/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_ADSB_Backend.h"

#if HAL_ADSB_ENABLED

#include <AP_Math/AP_Math.h>

#include <stdio.h>
#include <time.h>

/*
  base class constructor.
*/
AP_ADSB_Backend::AP_ADSB_Backend(AP_ADSB &frontend, uint8_t instance) :
    _instance(instance),
    _frontend(frontend)
{
}

#if HAL_ADSB_SAGETECH_ENABLED || HAL_ADSB_SAGETECH_MXS_ENABLED

// NOTE: the coordinate maths MUST be done in double or else we get
// roundoff in the maths

void AP_ADSB_Backend::format_longitude(char *buf, size_t buflen, int32_t lng_1e7)
{
    const double deg = lng_1e7 * (double)1.0e-7 * (lng_1e7 < 0 ? -1 : 1);
    const double minutes = (deg - int(deg)) * 60;
    snprintf(buf, buflen, "%03u%02u.%05u",
             MIN(unsigned(deg), 180U),
             MIN(unsigned(minutes), 59U),
             MIN(unsigned((minutes - (int)minutes) * 1.0E5), 99999U));
}

void AP_ADSB_Backend::format_latitude(char *buf, size_t buflen, int32_t lat_1e7)
{
    const double deg = lat_1e7 * (double)1.0e-7 * (lat_1e7 < 0 ? -1 : 1);
    const double minutes = (deg - int(deg)) * 60;
    snprintf(buf, buflen, "%02u%02u.%05u",
             MIN(unsigned(deg), 90U),
             MIN(unsigned(minutes), 59U),
             MIN(unsigned((minutes - (int)minutes) * 1.0E5), 99999U));
}

void AP_ADSB_Backend::format_speed_knots(char *buf, size_t buflen, float knots)
{
    snprintf(buf, buflen, "%03u.%02u",
             MIN(unsigned(knots), 999U),
             MIN(unsigned((knots - (int)knots) * 1.0E2), 99U));
}

void AP_ADSB_Backend::format_track_deg(char *buf, size_t buflen, float track_deg)
{
    snprintf(buf, buflen, "%03u.%04u",
             MIN(unsigned(track_deg), 359U),
             MIN(unsigned((track_deg - (int)track_deg) * 1.0E4), 9999U));
}

void AP_ADSB_Backend::format_time_of_day(char *buf, size_t buflen, uint64_t time_usec)
{
    const time_t time_sec = time_usec / 1000000;
    struct tm tmd {};
    gmtime_r(&time_sec, &tmd);

    // the seconds are formatted as an integer rather than as a float so
    // that the width of the field is known; the milliseconds are rounded
    // and carried into the seconds to match the "%06.3f" this replaced
    const unsigned msec = unsigned((time_usec % 1000000 + 500) / 1000);
    const unsigned sec = unsigned(tmd.tm_sec) + msec / 1000;
    snprintf(buf, buflen, "%02u%02u%02u.%03u",
             MIN(unsigned(tmd.tm_hour), 23U),
             MIN(unsigned(tmd.tm_min), 59U),
             MIN(sec, 61U),
             msec % 1000);
}

#endif // HAL_ADSB_SAGETECH_ENABLED || HAL_ADSB_SAGETECH_MXS_ENABLED

#endif // HAL_ADSB_ENABLED

