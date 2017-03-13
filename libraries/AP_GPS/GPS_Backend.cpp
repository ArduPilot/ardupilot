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

#include "AP_GPS.h"
#include "GPS_Backend.h"

extern const AP_HAL::HAL& hal;

AP_GPS_Backend::AP_GPS_Backend(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    port(_port),
    gps(_gps),
    state(_state)
{
    state.have_speed_accuracy = false;
    state.have_horizontal_accuracy = false;
    state.have_vertical_accuracy = false;
}

int32_t AP_GPS_Backend::swap_int32(int32_t v) const
{
    const uint8_t *b = (const uint8_t *)&v;
    union {
        int32_t v;
        uint8_t b[4];
    } u;

    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];

    return u.v;
}

int16_t AP_GPS_Backend::swap_int16(int16_t v) const
{
    const uint8_t *b = (const uint8_t *)&v;
    union {
        int16_t v;
        uint8_t b[2];
    } u;

    u.b[0] = b[1];
    u.b[1] = b[0];

    return u.v;
}


/**
   fill in time_week_ms and time_week from BCD date and time components
   assumes MTK19 millisecond form of bcd_time
 */
void AP_GPS_Backend::make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds)
{
    uint8_t year, mon, day, hour, min, sec;
    uint16_t msec;

    year = bcd_date % 100;
    mon  = (bcd_date / 100) % 100;
    day  = bcd_date / 10000;

    uint32_t v = bcd_milliseconds;
    msec = v % 1000; v /= 1000;
    sec  = v % 100; v /= 100;
    min  = v % 100; v /= 100;
    hour = v % 100; v /= 100;

    int8_t rmon = mon - 2;
    if (0 >= rmon) {    
        rmon += 12;
        year -= 1;
    }

    // get time in seconds since unix epoch
    uint32_t ret = (year/4) - (GPS_LEAPSECONDS_MILLIS / 1000UL) + 367*rmon/12 + day;
    ret += year*365 + 10501;
    ret = ret*24 + hour;
    ret = ret*60 + min;
    ret = ret*60 + sec;

    // convert to time since GPS epoch
    ret -= 272764785UL;

    // get GPS week and time
    state.time_week = ret / SEC_PER_WEEK;
    state.time_week_ms = (ret % SEC_PER_WEEK) * MSEC_PER_SEC;
    state.time_week_ms += msec;
}

/*
  fill in 3D velocity for a GPS that doesn't give vertical velocity numbers
 */
void AP_GPS_Backend::fill_3d_velocity(void)
{
    float gps_heading = radians(state.ground_course);

    state.velocity.x = state.ground_speed * cosf(gps_heading);
    state.velocity.y = state.ground_speed * sinf(gps_heading);
    state.velocity.z = 0;
    state.have_vertical_velocity = false;
}
