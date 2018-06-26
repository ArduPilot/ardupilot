/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * AP_OSD partially based on betaflight and inav osd.c implemention.
 * clarity.mcm font is taken from inav configurator.
 * Many thanks to their authors.
 */
/*
  parameter settings for one screen
 */
#include "AP_OSD.h"
#include "AP_OSD_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_Notify/AP_Notify.h>
#include <ctype.h>

const AP_Param::GroupInfo AP_OSD_Screen::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable screen
    // @Description: Enable this screen
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_OSD_Screen, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CHAN_MIN
    // @DisplayName: Transmitter switch screen minimum pwm
    // @Description: This sets the PWM lower limit for this screen
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MIN", 2, AP_OSD_Screen, channel_min, 900),

    // @Param: CHAN_MAX
    // @DisplayName: Transmitter switch screen maximum pwm
    // @Description: This sets the PWM upper limit for this screen
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MAX", 3, AP_OSD_Screen, channel_max, 2100),

    // @Group: ALTITUDE
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(altitude, "ALTITUDE", 4, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: BATVOLT
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(bat_volt, "BAT_VOLS", 5, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: RSSI
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(rssi, "RSSI", 6, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: CURRENT
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(current, "CURRENT", 7, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: BATUSED
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(batused, "BATUSED", 8, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: SATS
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(sats, "SATS", 9, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: FLTMODE
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(fltmode, "FLTMODE", 10, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: MESSAGE
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(message, "MESSAGE", 11, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: GSPEED
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(gspeed, "GSPEED", 12, AP_OSD_Screen, AP_OSD_Setting),

    AP_GROUPEND
};

// constructor
AP_OSD_Screen::AP_OSD_Screen()
{
}

//Symbols
#define SYM_BLANK 0x20
#define SYM_COLON 0x2D
#define SYM_ZERO_HALF_TRAILING_DOT 192
#define SYM_ZERO_HALF_LEADING_DOT 208

#define SYM_M       177
#define SYM_ALT_M       177
#define SYM_BATT_FULL   0x90
#define SYM_RSSI 0x01

#define SYM_VOLT  0x06
#define SYM_AMP   0x9A
#define SYM_MAH   0x07
#define SYM_KMH   0xA1

#define SYM_SAT_L 0x1E
#define SYM_SAT_R 0x1F

#define AH_SYMBOL_COUNT 9

void AP_OSD_Screen::draw_altitude(uint8_t x, uint8_t y)
{
    float alt;
    AP_AHRS::get_singleton()->get_relative_position_D_home(alt);
    backend->write(x, y, false, "%4.0f%c", alt, SYM_ALT_M);
}

void AP_OSD_Screen::draw_bat_volt(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    uint8_t p = battery.capacity_remaining_pct();
    p = (100 - p) / 16.6;
    backend->write(x,y, battery.has_failsafed(), "%c%2.1fV", SYM_BATT_FULL + p, battery.voltage());
}

void AP_OSD_Screen::draw_rssi(uint8_t x, uint8_t y)
{
    int rssiv = AP_RSSI::get_instance()->read_receiver_rssi_uint8();
    rssiv = (rssiv * 99) / 255;
    backend->write(x, y, rssiv < 5, "%c%2d", SYM_RSSI, rssiv);
}

void AP_OSD_Screen::draw_current(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    float amps = battery.current_amps();
    backend->write(x, y, false, "%c%2.1f", SYM_AMP, amps);
}

void AP_OSD_Screen::draw_fltmode(uint8_t x, uint8_t y)
{
    backend->write(x, y, AP_Notify::instance()->get_flight_mode_str());
}

void AP_OSD_Screen::draw_sats(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    backend->write(x, y, false, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, gps.num_sats());
}

void AP_OSD_Screen::draw_batused(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    backend->write(x,y, battery.has_failsafed(), "%c%4.0f", SYM_MAH, battery.consumed_mah());
}

void AP_OSD_Screen::draw_message(uint8_t x, uint8_t y)
{
    AP_Notify * notify = AP_Notify::instance();
    if (notify) {
        bool text_is_valid = AP_HAL::millis() - notify->get_text_updated_millis() < 20000;
        if (text_is_valid) {
            char buffer[NOTIFY_TEXT_BUFFER_SIZE];
            //converted to uppercase,
            //because we do not have small letter chars inside used font
            strncpy(buffer, notify->get_text(), sizeof(buffer));
            for (uint8_t i=0; i<sizeof(buffer); i++) {
                buffer[i] = toupper(buffer[i]);
            }
            backend->write(x, y, buffer);
        }
    }
}

void AP_OSD_Screen::draw_gspeed(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    float v = gps.ground_speed() * 3.6;
    backend->write(x, y, false, "%3.0f%c", v, SYM_KMH);
}

#define DRAW_SETTING(n) if (n.enabled) draw_ ## n(n.xpos, n.ypos)

void AP_OSD_Screen::draw(void)
{
    if (!enabled || !backend) {
        return;
    }

    DRAW_SETTING(altitude);
    DRAW_SETTING(bat_volt);
    DRAW_SETTING(rssi);
    DRAW_SETTING(current);
    DRAW_SETTING(batused);
    DRAW_SETTING(sats);
    DRAW_SETTING(fltmode);
    DRAW_SETTING(message);
    DRAW_SETTING(gspeed);
}
