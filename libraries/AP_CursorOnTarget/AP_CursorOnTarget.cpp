/*
   Copyright (C) 2023  Kraus Hamdani Aerospace Inc. All rights reserved.

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


   Author: Tom Pittenger

    Implements portions of the Cursor On Target protocol
    https://www.mitre.org/sites/default/files/pdf/09_4937.pdf
 */

#include "AP_CursorOnTarget.h"

#if AP_CURSORONTARGET_ENABLED

#include <stdio.h>
#include <string.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>

#include <AP_SerialManager/AP_SerialManager.h>

#define AP_CURSORONTARGET_SEND_BASIC_POSITION_INTERVAL_MS_MAX 60000 // 1 minute
#define AP_CURSORONTARGET_SEND_BASIC_POSITION_INTERVAL_MS_MIN 100   // 10 Hz

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_CursorOnTarget::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Cursor on Target Enable/Disable
    // @Description: Cursor on Target enable/disable
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_CursorOnTarget, _params.enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SEND_RATE_MS
    // @DisplayName: Rate to send regular position
    // @Description: Rate to send regular position info of ourself
    // @User: Standard
    // @Units: ms
    // @Range: 100 60000
    // @Increment: 500
    AP_GROUPINFO("SEND_RATE_MS", 1, AP_CursorOnTarget, _params.send_basic_position_interval_ms, 1000),

    AP_GROUPEND
};


AP_CursorOnTarget::AP_CursorOnTarget()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many AP_CursorOnTargets");
#endif
        return;
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_CursorOnTarget::init()
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_CursorOnTarget, 0);
}

void AP_CursorOnTarget::update()
{
    if (!enabled()) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    const uint32_t interval_ms = constrain_uint32(
        _params.send_basic_position_interval_ms.get(),
        AP_CURSORONTARGET_SEND_BASIC_POSITION_INTERVAL_MS_MIN,
        AP_CURSORONTARGET_SEND_BASIC_POSITION_INTERVAL_MS_MAX);

    if (now_ms - _last_send_basic_position_ms >= interval_ms) {
        if (send_basic_position()) {
            _last_send_basic_position_ms = now_ms;
        }
    }
}

bool AP_CursorOnTarget::send_basic_position()
{
    if (_uart == nullptr) {
        return false;
    }
    if (_uart->available() < 500) {
        return false;
    }

    uint64_t time_usec;
    if (!AP::rtc().get_utc_usec(time_usec)) {
        return false;
    }

    char time_start_str[30];
    if (!AP::rtc().unix_time_to_string(time_usec, time_start_str, sizeof(time_start_str))) {
        return false;
    }

    char time_stale_str[30];
    if (!AP::rtc().unix_time_to_string(time_usec + (5 * 1E6), time_stale_str, sizeof(time_stale_str))) {
        return false;
    }

#ifdef HAL_BUILD_AP_PERIPH
    if (AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        return false;
    }
    Location loc = AP::gps().location();
    const float course = AP::gps().ground_course();
    const float speed = AP::gps().ground_speed();
#else
    Location loc;
    if (!AP::ahrs().get_location(loc)) {
        return false;
    }
    const float course = AP::ahrs().groundspeed_vector().angle();
    const float speed = AP::ahrs().groundspeed();
#endif

    const char* CoT_MY_UID = "K1000";
    const char* CoT_MY_TYPE = "a-f-A-M-F-Q";
    char s[100]; // buffer we re-use

// <?xml version="1.0" encoding="utf-8" standalone="yes"?>
// <event
//   version="2.0"
//   uid="K1000"
//   type="a-f-A-M-F-Q"
//   time="2023-02-07T16:51:40.3075759Z"
//   start="2023-02-07T16:51:35.3075759Z"
//   stale="2023-02-07T16:51:45.3075759Z"
//   how="m-g">
//   <detail>
//     <track
//       course="0.00"
//       speed="0.00" />
//   </detail>
//   <point
//     lat="-35.3632622"
//     lon="149.1652375"
//     hae="584.14"
//     ce="1.0"
//     le="1.0" />
// </event>

    _uart->write(           "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>\n");
    _uart->write(           "<event version=\"2.0\"\n");
    snprintf(s, sizeof(s),  "  uid=\"%s\"\n",           CoT_MY_UID); _uart->write(s);
    snprintf(s, sizeof(s),  "  type=\"%s\"\n",          CoT_MY_TYPE); _uart->write(s);
    snprintf(s, sizeof(s),  "  time=\"%s\"\n",          time_start_str); _uart->write(s);
    snprintf(s, sizeof(s),  "  start=\"%s\"\n",         time_start_str); _uart->write(s);
    snprintf(s, sizeof(s),  "  stale=\"%s\"\n",         time_stale_str); _uart->write(s);
    _uart->write(           "  how=\"m-g\">\n");
    _uart->write(           "  <detail>\n");
    _uart->write(           "    <track\n");
    snprintf(s, sizeof(s),  "      course=\"%.2f\"\n",  course); _uart->write(s);
    snprintf(s, sizeof(s),  "      speed=\"%.2f\" />\n",speed); _uart->write(s);
    _uart->write(           "  </detail>\n");
    _uart->write(           "  <point\n");
    snprintf(s, sizeof(s),  "    lat=\"%.7f\"\n",       loc.lat * 1.0e-7f); _uart->write(s);
    snprintf(s, sizeof(s),  "    lon=\"%.7f\"\n",       loc.lng * 1.0e-7f); _uart->write(s);
    snprintf(s, sizeof(s),  "    hae=\"%.2f\"\n",       loc.alt * 1.0e-2f); _uart->write(s);
    _uart->write(           "    ce=\"1.0\"\n");
    _uart->write(           "    le=\"1.0\" />\n");
    _uart->write(           "</event>\n\n");

    return true;
}

AP_CursorOnTarget* AP_CursorOnTarget::_singleton;
namespace AP {
AP_CursorOnTarget *CursorOnTarget()
{
    return AP_CursorOnTarget::get_singleton();
}
};

#endif // AP_CURSORONTARGET_ENABLED
