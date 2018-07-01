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
    AP_SUBGROUPINFO(bat_volt, "BAT_VOLT", 5, AP_OSD_Screen, AP_OSD_Setting),

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

    // @Group: HORIZON
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(horizon, "HORIZON", 13, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: HOME
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(home, "HOME", 14, AP_OSD_Screen, AP_OSD_Setting),

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

#define SYM_M           0xB9
#define SYM_KM          0xBA
#define SYM_ALT_M       0xB1
#define SYM_BATT_FULL   0x90
#define SYM_RSSI        0x01

#define SYM_VOLT  0x06
#define SYM_AMP   0x9A
#define SYM_MAH   0x07
#define SYM_KMH   0xA1

#define SYM_SAT_L 0x1E
#define SYM_SAT_R 0x1F

#define SYM_HOME 0xBF

#define SYM_ARROW_START 0x60
#define SYM_ARROW_COUNT 16

#define SYM_AH_START 0x80
#define SYM_AH_COUNT 9

#define SYM_AH_CENTER_LINE_LEFT   0x26
#define SYM_AH_CENTER_LINE_RIGHT  0x27
#define SYM_AH_CENTER             0x7E

#define SYM_HEADING_N             0x18
#define SYM_HEADING_S             0x19
#define SYM_HEADING_E             0x1A
#define SYM_HEADING_W             0x1B
#define SYM_HEADING_DIVIDED_LINE  0x1C
#define SYM_HEADING_LINE          0x1D

void AP_OSD_Screen::draw_altitude(uint8_t x, uint8_t y)
{
    float alt;
    AP::ahrs().get_relative_position_D_home(alt);
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
    AP_RSSI *ap_rssi = AP_RSSI::get_instance();
    if (ap_rssi) {
        int rssiv = ap_rssi->read_receiver_rssi_uint8();
        rssiv = (rssiv * 99) / 255;
        backend->write(x, y, rssiv < 5, "%c%2d", SYM_RSSI, rssiv);
    }
}

void AP_OSD_Screen::draw_current(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    float amps = battery.current_amps();
    backend->write(x, y, false, "%c%2.1f", SYM_AMP, amps);
}

void AP_OSD_Screen::draw_fltmode(uint8_t x, uint8_t y)
{
    AP_Notify * notify = AP_Notify::instance();
    if (notify) {
        backend->write(x, y, notify->get_flight_mode_str());
    }
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
        uint32_t visible_time = AP_HAL::millis() - notify->get_text_updated_millis();
        if (visible_time < message_show_time_ms) {
            char buffer[NOTIFY_TEXT_BUFFER_SIZE];
            strncpy(buffer, notify->get_text(), sizeof(buffer));
            int16_t len = strnlen(buffer, sizeof(buffer));

            //converted to uppercase,
            //because we do not have small letter chars inside used font
            for (int16_t i=0; i<len; i++) {
                buffer[i] = toupper(buffer[i]);
            }

            int16_t start_position = 0;
            //scroll if required
            //scroll pattern: wait, scroll to the left, wait, scroll to the right
            if (len > message_visible_width) {
                int16_t chars_to_scroll = len - message_visible_width;
                int16_t total_cycles = 2*message_scroll_delay + 2*chars_to_scroll;
                int16_t current_cycle = (visible_time / message_scroll_time_ms) % total_cycles;

                //calculate scroll start_position
                if (current_cycle < total_cycles/2) {
                    //move to the left
                    start_position = current_cycle - message_scroll_delay;
                } else {
                    //move to the right
                    start_position = total_cycles - current_cycle;
                }
                start_position = constrain_int16(start_position, 0, chars_to_scroll);
                int16_t end_position = start_position + message_visible_width;

                //ensure array boundaries
                start_position = MIN(start_position, int(sizeof(buffer)-1));
                end_position = MIN(end_position, int(sizeof(buffer)-1));

                //trim invisible part
                buffer[end_position] = 0;
            }

            backend->write(x, y, buffer + start_position);
        }
    }
}

void AP_OSD_Screen::draw_gspeed(uint8_t x, uint8_t y)
{
    float v = AP::ahrs().groundspeed() * 3.6;
    backend->write(x, y, false, "%3.0f%c", v, SYM_KMH);
}

void AP_OSD_Screen::draw_horizon(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    float roll = -ahrs.roll;
    float pitch = ahrs.pitch;

    roll = constrain_float(roll, -ah_max_roll, ah_max_roll);
    pitch = constrain_float(pitch, -ah_max_pitch, ah_max_pitch);

    for (int dx = -4; dx <= 4; dx++) {
        float fy =  dx * roll + pitch * ah_pitch_rad_to_char + 0.5f;
        int dy = floorf(fy);
        char c = (fy - dy) * SYM_AH_COUNT;
        //chars in font in reversed order
        c = SYM_AH_START + ((SYM_AH_COUNT - 1) - c);
        if (dy >= -4 && dy <= 4) {
            backend->write(x + dx, y - dy, false, "%c", c);
        }
    }
    backend->write(x-1,y, false, "%c%c%c", SYM_AH_CENTER_LINE_LEFT, SYM_AH_CENTER, SYM_AH_CENTER_LINE_RIGHT);
}

void AP_OSD_Screen::draw_home(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    Location loc;
    if (ahrs.get_position(loc) && ahrs.home_is_set()) {
        const Location &home_loc = ahrs.get_home();
        float distance = get_distance(home_loc, loc);
        int32_t angle = get_bearing_cd(loc, home_loc);
        int32_t interval = 36000 / SYM_ARROW_COUNT;
        if (distance < 2.0f) {
            //avoid fast rotating arrow at small distances
            angle = 0;
        }
        char arrow = SYM_ARROW_START + ((angle + interval / 2) / interval) % SYM_ARROW_COUNT;
        if (distance < 999.0f) {
            backend->write(x, y, false, "%c%c%3.0f%c", SYM_HOME, arrow, distance, SYM_M);
        } else if (distance < 9999.0f) {
            backend->write(x, y, false, "%c%c%1.1f%c", SYM_HOME, arrow, distance/1000, SYM_KM);
        } else {
            backend->write(x, y, false, "%c%c%3.0f%c", SYM_HOME, arrow, distance/1000, SYM_KM);
        }
    } else {
        backend->write(x, y, true, "%c", SYM_HOME);
    }
}

#define DRAW_SETTING(n) if (n.enabled) draw_ ## n(n.xpos, n.ypos)

void AP_OSD_Screen::draw(void)
{
    if (!enabled || !backend) {
        return;
    }

    //Note: draw order should be optimized.
    //Big and less important items should be drawn first,
    //so they will not overwrite more important ones.
    DRAW_SETTING(message);
    DRAW_SETTING(horizon);
    DRAW_SETTING(altitude);
    DRAW_SETTING(bat_volt);
    DRAW_SETTING(rssi);
    DRAW_SETTING(current);
    DRAW_SETTING(batused);
    DRAW_SETTING(sats);
    DRAW_SETTING(fltmode);
    DRAW_SETTING(gspeed);
    DRAW_SETTING(home);
}
