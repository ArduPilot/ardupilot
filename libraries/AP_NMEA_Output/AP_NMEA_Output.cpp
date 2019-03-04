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


   Author: Francisco Ferreira (some code is copied from sitl_gps.cpp)

 */

#include "AP_NMEA_Output.h"

#if !HAL_MINIMIZE_FEATURES && AP_AHRS_NAVEKF_AVAILABLE

#include <AP_Math/definitions.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <stdio.h>
#include <time.h>

AP_NMEA_Output* AP_NMEA_Output::_singleton;

AP_NMEA_Output::AP_NMEA_Output()
{
    AP_SerialManager& sm = AP::serialmanager();

    for (uint8_t i = 0; i < ARRAY_SIZE(_uart); i++) {
        _uart[i] = sm.find_serial(AP_SerialManager::SerialProtocol_NMEAOutput, i);

        if (_uart[i] == nullptr) {
            break;
        }
        _num_outputs++;
    }
}

AP_NMEA_Output* AP_NMEA_Output::probe()
{
    if (!_singleton && AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NMEAOutput, 0) != nullptr) {
       _singleton = new AP_NMEA_Output();
    }

    return _singleton;
}

uint8_t AP_NMEA_Output::_nmea_checksum(const char *str)
{
    uint8_t checksum = 0;
    const uint8_t* bytes = (const uint8_t*) str;

    for (uint16_t i = 1; str[i]; i++) {
        checksum ^= bytes[i];
    }

    return checksum;
}

void AP_NMEA_Output::update()
{
    uint16_t now = AP_HAL::millis16();
    uint16_t time_diff = now - _last_sent;

    // only send at 10Hz
    if (time_diff < 100) {
        return;
    }

    // get time and date
    uint64_t time_usec;
    bool time_valid = AP::rtc().get_utc_usec(time_usec);

    if (!time_valid) {
        return;
    }

    // not completely accurate, our time includes leap seconds and time_t should be without
    time_t time_sec = time_usec / 1000000;
    struct tm* tm = gmtime(&time_sec);

    // format time string
    char tstring[11];
    snprintf(tstring, sizeof(tstring), "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (time_usec % 1000000) * 1.0e-6);

    // format date string
    char dstring[7];
    snprintf(dstring, sizeof(dstring), "%02u%02u%02u", tm->tm_mday, tm->tm_mon+1, tm->tm_year % 100);

    AP_AHRS_NavEKF& ahrs = AP::ahrs_navekf();

    // get location (note: get_position from AHRS always returns true after having GPS position once)
    Location loc;
    bool pos_valid = ahrs.get_location(loc);

    // format latitude
    char lat_string[13];
    float deg = fabsf(loc.lat * 1.0e-7f);
    snprintf(lat_string,
             sizeof(lat_string),
             "%02u%08.5f,%c",
             (unsigned) deg,
             double((deg - int(deg)) * 60),
             loc.lat < 0 ? 'S' : 'N');

    // format longitude
    char lng_string[14];
    deg = fabsf(loc.lng * 1.0e-7f);
    snprintf(lng_string,
             sizeof(lng_string),
             "%03u%08.5f,%c",
             (unsigned) deg,
             double((deg - int(deg)) * 60),
             loc.lng < 0 ? 'W' : 'E');

    // format GGA message
    char* gga = nullptr;
    int16_t gga_res = asprintf(&gga,
                               "$GPGGA,%s,%s,%s,%01d,%02d,%04.1f,%07.2f,M,0.0,M,,",
                               tstring,
                               lat_string,
                               lng_string,
                               pos_valid ? 1 : 0,
                               pos_valid ? 6 : 3,
                               2.0,
                               loc.alt * 0.01f);
    char gga_end[6];
    snprintf(gga_end, sizeof(gga_end), "*%02X\r\n", (unsigned) _nmea_checksum(gga));

    // get speed
    Vector2f speed = ahrs.groundspeed_vector();
    float speed_knots = norm(speed.x, speed.y) * M_PER_SEC_TO_KNOTS;
    float heading = wrap_360(degrees(atan2f(speed.x, speed.y)));

    // format RMC message
    char* rmc = nullptr;
    int16_t rmc_res = asprintf(&rmc,
                               "$GPRMC,%s,%c,%s,%s,%.2f,%.2f,%s,,",
                               tstring,
                               pos_valid ? 'A' : 'V',
                               lat_string,
                               lng_string,
                               speed_knots,
                               heading,
                               dstring);
    char rmc_end[6];
    snprintf(rmc_end, sizeof(rmc_end), "*%02X\r\n", (unsigned) _nmea_checksum(rmc));

    // send to all NMEA output ports
    for (uint8_t i = 0; i < _num_outputs; i++) {
        if (gga_res != -1) {
            _uart[i]->write(gga);
            _uart[i]->write(gga_end);
        }

        if (rmc_res != -1) {
            _uart[i]->write(rmc);
            _uart[i]->write(rmc_end);
        }
    }

    if (gga_res != -1) {
        free(gga);
    }

    if (rmc_res != -1) {
        free(rmc);
    }

    _last_sent = now;
}

#endif  // !HAL_MINIMIZE_FEATURES && AP_AHRS_NAVEKF_AVAILABLE
