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
#include <AP_Stats/AP_Stats.h>

#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

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

    //@Group: HEADING
    //@Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(heading, "HEADING", 15, AP_OSD_Screen, AP_OSD_Setting),

    //@Group: THROTTLE
    //@Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(throttle, "THROTTLE", 16, AP_OSD_Screen, AP_OSD_Setting),

    //@Group: COMPASS
    //@Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(compass, "COMPASS", 17, AP_OSD_Screen, AP_OSD_Setting),

    //@Group: WIND
    //@Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(wind, "WIND", 18, AP_OSD_Screen, AP_OSD_Setting),

    //@Group: ASPEED
    //@Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(aspeed, "ASPEED", 19, AP_OSD_Screen, AP_OSD_Setting),

    //@Group: VSPEED
    //@Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(vspeed, "VSPEED", 20, AP_OSD_Screen, AP_OSD_Setting),

#ifdef HAVE_AP_BLHELI_SUPPORT
    // @Group: BLHTEMP
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(blh_temp, "BLHTEMP", 21, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: BLHRPM
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(blh_rpm, "BLHRPM", 22, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: BLHAMPS
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(blh_amps, "BLHAMPS", 23, AP_OSD_Screen, AP_OSD_Setting),
#endif

    // @Group: GPSLAT
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(gps_latitude, "GPSLAT", 24, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: GPSLONG
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(gps_longitude, "GPSLONG", 25, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: ROLL
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(roll_angle, "ROLL", 26, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: PITCH
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(pitch_angle, "PITCH", 27, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: TEMP
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(temp, "TEMP", 28, AP_OSD_Screen, AP_OSD_Setting),
    
    // @Group: HDOP
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(hdop, "HDOP", 29, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: WAYPOINT
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(waypoint, "WAYPOINT", 30, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: XTRACK
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(xtrack_error, "XTRACK", 31, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: DIST
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(dist, "DIST", 32, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: STATS
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(stat, "STATS", 33, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: FLTIME
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(flightime, "FLTIME", 34, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: CLIMBEFF
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(climbeff, "CLIMBEFF", 35, AP_OSD_Screen, AP_OSD_Setting),

    // @Group: EFF
    // @Path: AP_OSD_Setting.cpp
    AP_SUBGROUPINFO(eff, "EFF", 36, AP_OSD_Screen, AP_OSD_Setting),

    AP_GROUPEND
};

// constructor
AP_OSD_Screen::AP_OSD_Screen()
{
}

//Symbols

#define SYM_M           0xB9
#define SYM_KM          0xBA
#define SYM_FT          0x0F
#define SYM_MI          0xBB
#define SYM_ALT_M       0xB1
#define SYM_ALT_FT      0xB3
#define SYM_BATT_FULL   0x90
#define SYM_RSSI        0x01

#define SYM_VOLT  0x06
#define SYM_AMP   0x9A
#define SYM_MAH   0x07
#define SYM_MS    0x9F
#define SYM_FS    0x99
#define SYM_KMH   0xA1
#define SYM_MPH   0xB0
#define SYM_DEGR  0xA8
#define SYM_PCNT  0x25
#define SYM_RPM   0xE0
#define SYM_ASPD  0xE1
#define SYM_GSPD  0xE2
#define SYM_WSPD  0xE3
#define SYM_VSPD  0xE4
#define SYM_WPNO  0xE5
#define SYM_WPDIR  0xE6
#define SYM_WPDST  0xE7
#define SYM_FTMIN  0xE8
#define SYM_FTSEC  0x99



#define SYM_SAT_L 0x1E
#define SYM_SAT_R 0x1F
#define SYM_HDOP_L 0xBD
#define SYM_HDOP_R 0xBE

#define SYM_HOME 0xBF
#define SYM_WIND 0x16

#define SYM_ARROW_START 0x60
#define SYM_ARROW_COUNT 16

#define SYM_AH_H_START 0x80
#define SYM_AH_H_COUNT 9

#define SYM_AH_V_START 0xCA
#define SYM_AH_V_COUNT 6


#define SYM_AH_CENTER_LINE_LEFT   0x26
#define SYM_AH_CENTER_LINE_RIGHT  0x27
#define SYM_AH_CENTER             0x7E

#define SYM_HEADING_N             0x18
#define SYM_HEADING_S             0x19
#define SYM_HEADING_E             0x1A
#define SYM_HEADING_W             0x1B
#define SYM_HEADING_DIVIDED_LINE  0x1C
#define SYM_HEADING_LINE          0x1D

#define SYM_UP_UP        0xA2
#define SYM_UP           0xA3
#define SYM_DOWN         0xA4
#define SYM_DOWN_DOWN    0xA5

#define SYM_DEGREES_C 0x0E
#define SYM_DEGREES_F 0x0D
#define SYM_GPS_LAT   0xA6
#define SYM_GPS_LONG  0xA7
#define SYM_ARMED     0x00
#define SYM_DISARMED  0xE9
#define SYM_ROLL0     0x2D
#define SYM_ROLLR     0xEA
#define SYM_ROLLL     0xEB
#define SYM_PTCH0     0x7C
#define SYM_PTCHUP    0xEC
#define SYM_PTCHDWN   0xED
#define SYM_XERR      0xEE
#define SYM_KN        0xF0
#define SYM_NM        0xF1
#define SYM_DIST      0x22
#define SYM_FLY       0x9C
#define SYM_EFF       0xF2

void AP_OSD_Screen::set_backend(AP_OSD_Backend *_backend)
{
    backend = _backend;
    osd = _backend->get_osd();
};

bool AP_OSD_Screen::check_option(uint32_t option)
{
    return (osd->options & option) != 0;
}

/*
  get the right units icon given a unit
 */
char AP_OSD_Screen::u_icon(enum unit_type unit)
{
    static const char icons_metric[UNIT_TYPE_LAST] {
        (char)SYM_ALT_M,    //ALTITUDE
        (char)SYM_KMH,      //SPEED
        (char)SYM_MS,       //VSPEED
        (char)SYM_M,        //DISTANCE
        (char)SYM_KM,       //DISTANCE_LONG
        (char)SYM_DEGREES_C //TEMPERATURE
    };
    static const char icons_imperial[UNIT_TYPE_LAST] {
        (char)SYM_ALT_FT,   //ALTITUDE
        (char)SYM_MPH,      //SPEED
        (char)SYM_FS,       //VSPEED
        (char)SYM_FT,       //DISTANCE
        (char)SYM_MI,       //DISTANCE_LONG
        (char)SYM_DEGREES_F //TEMPERATURE
    };
    static const char icons_SI[UNIT_TYPE_LAST] {
        (char)SYM_ALT_M,    //ALTITUDE
        (char)SYM_MS,       //SPEED
        (char)SYM_MS,       //VSPEED
        (char)SYM_M,        //DISTANCE
        (char)SYM_KM,       //DISTANCE_LONG
        (char)SYM_DEGREES_C //TEMPERATURE
    };
    static const char icons_aviation[UNIT_TYPE_LAST] {
        (char)SYM_ALT_FT,   //ALTITUDE Ft
        (char)SYM_KN,       //SPEED Knots
        (char)SYM_FS,       //VSPEED
        (char)SYM_FT,       //DISTANCE
        (char)SYM_NM,       //DISTANCE_LONG Nm
        (char)SYM_DEGREES_C //TEMPERATURE
    };
    static const char *icons[AP_OSD::UNITS_LAST] = {
        icons_metric,
        icons_imperial,
        icons_SI,
        icons_aviation,
    };
    return icons[constrain_int16(osd->units, 0, AP_OSD::UNITS_LAST-1)][unit];
}

/*
  scale a value for the user selected units
 */
float AP_OSD_Screen::u_scale(enum unit_type unit, float value)
{
    static const float scale_metric[UNIT_TYPE_LAST] = {
        1.0,       //ALTITUDE m
        3.6,       //SPEED km/hr
        1.0,       //VSPEED m/s
        1.0,       //DISTANCE m
        1.0/1000,  //DISTANCE_LONG km
        1.0,       //TEMPERATURE C
    };
    static const float scale_imperial[UNIT_TYPE_LAST] = {
        3.28084,     //ALTITUDE ft
        2.23694,     //SPEED mph
        3.28084,     //VSPEED ft/s
        3.28084,     //DISTANCE ft
        1.0/1609.34, //DISTANCE_LONG miles
        1.8,         //TEMPERATURE F
    };
    static const float offset_imperial[UNIT_TYPE_LAST] = {
        0.0,          //ALTITUDE
        0.0,          //SPEED
        0.0,          //VSPEED
        0.0,          //DISTANCE
        0.0,          //DISTANCE_LONG
        32.0,         //TEMPERATURE F
    };
    static const float scale_SI[UNIT_TYPE_LAST] = {
        1.0,       //ALTITUDE m
        1.0,       //SPEED m/s
        1.0,       //VSPEED m/s
        1.0,       //DISTANCE m
        1.0/1000,  //DISTANCE_LONG km
        1.0,       //TEMPERATURE C
    };
    static const float scale_aviation[UNIT_TYPE_LAST] = {
        3.28084,   //ALTITUDE Ft
        1.94384,   //SPEED Knots
        196.85,    //VSPEED ft/min
        3.28084,   //DISTANCE ft
        0.000539957,  //DISTANCE_LONG Nm
        1.0,       //TEMPERATURE C
    };
    static const float *scale[AP_OSD::UNITS_LAST] = {
        scale_metric,
        scale_imperial,
        scale_SI,
        scale_aviation
    };
    static const float *offsets[AP_OSD::UNITS_LAST] = {
        nullptr,
        offset_imperial,
        nullptr,
        nullptr
    };
    uint8_t units = constrain_int16(osd->units, 0, AP_OSD::UNITS_LAST-1);
    return value * scale[units][unit] + (offsets[units]?offsets[units][unit]:0);
}

void AP_OSD_Screen::draw_altitude(uint8_t x, uint8_t y)
{
    float alt;
    AP::ahrs().get_relative_position_D_home(alt);
    alt = -alt;
    backend->write(x, y, false, "%4d%c", (int)u_scale(ALTITUDE, alt), u_icon(ALTITUDE));
}

void AP_OSD_Screen::draw_bat_volt(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    uint8_t pct = battery.capacity_remaining_pct();
    uint8_t p = (100 - pct) / 16.6;
    float v = battery.voltage();
    backend->write(x,y, v < osd->warn_batvolt, "%c%2.1f%c", SYM_BATT_FULL + p, (double)v, SYM_VOLT);
}

void AP_OSD_Screen::draw_rssi(uint8_t x, uint8_t y)
{
    AP_RSSI *ap_rssi = AP_RSSI::get_instance();
    if (ap_rssi) {
        int rssiv = ap_rssi->read_receiver_rssi_uint8();
        rssiv = (rssiv * 99) / 255;
        backend->write(x, y, rssiv < osd->warn_rssi, "%c%2d", SYM_RSSI, rssiv);
    }
}

void AP_OSD_Screen::draw_current(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    float amps = battery.current_amps();
    backend->write(x, y, false, "%2.1f%c", (double)amps, SYM_AMP);
}

void AP_OSD_Screen::draw_fltmode(uint8_t x, uint8_t y)
{
    AP_Notify * notify = AP_Notify::instance();
    char arm;
    if (AP_Notify::flags.armed) {
        arm = SYM_ARMED;
    } else {
        arm = SYM_DISARMED;
    }
    if (notify) {
        backend->write(x, y, false, "%s%c", notify->get_flight_mode_str(), arm);
    }
}

void AP_OSD_Screen::draw_sats(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    int nsat = gps.num_sats();
    backend->write(x, y, nsat < osd->warn_nsat, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, nsat);
}

void AP_OSD_Screen::draw_batused(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    backend->write(x,y, false, "%4d%c", (int)battery.consumed_mah(), SYM_MAH);
}

//Autoscroll message is the same as in minimosd-extra.
//Thanks to night-ghost for the approach.
void AP_OSD_Screen::draw_message(uint8_t x, uint8_t y)
{
    AP_Notify * notify = AP_Notify::instance();
    if (notify) {
        int32_t visible_time = AP_HAL::millis() - notify->get_text_updated_millis();
        if (visible_time < osd->msgtime_s *1000) {
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

void AP_OSD_Screen::draw_speed_vector(uint8_t x, uint8_t y,Vector2f v, int32_t yaw)
{
    float v_length = v.length();
    char arrow = SYM_ARROW_START;
    if (v_length > 1.0f) {
        int32_t angle = wrap_360_cd(DEGX100 * atan2f(v.y, v.x) - yaw);
        int32_t interval = 36000 / SYM_ARROW_COUNT;
        arrow = SYM_ARROW_START + ((angle + interval / 2) / interval) % SYM_ARROW_COUNT;
    }

    backend->write(x, y, false, "%c%3d%c", arrow, (int)u_scale(SPEED, v_length), u_icon(SPEED));
}

void AP_OSD_Screen::draw_gspeed(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    Vector2f v = ahrs.groundspeed_vector();
    backend->write(x, y, false, "%c", SYM_GSPD);
    draw_speed_vector(x + 1, y, v, ahrs.yaw_sensor);
}

//Thanks to betaflight/inav for simple and clean artificial horizon visual design
void AP_OSD_Screen::draw_horizon(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    float roll = ahrs.roll;
    float pitch = -ahrs.pitch;

    //inverted roll AH
    if (check_option(AP_OSD::OPTION_INVERTED_AH_ROLL)) {
        roll = -roll;
    }

    pitch = constrain_float(pitch, -ah_max_pitch, ah_max_pitch);
    float ky = sinf(roll);
    float kx = cosf(roll);

    if (fabsf(ky) < fabsf(kx)) {
        for (int dx = -4; dx <= 4; dx++) {
            float fy =  dx * (ky/kx) + pitch * ah_pitch_rad_to_char + 0.5f;
            int dy = floorf(fy);
            char c = (fy - dy) * SYM_AH_H_COUNT;
            //chars in font in reversed order
            c = SYM_AH_H_START + ((SYM_AH_H_COUNT - 1) - c);
            if (dy >= -4 && dy <= 4) {
                backend->write(x + dx, y - dy, false, "%c", c);
            }
        }
    } else {
        for (int dy=-4; dy<=4; dy++) {
            float fx = (dy - pitch * ah_pitch_rad_to_char) * (kx/ky) + 0.5f;
            int dx = floorf(fx);
            char c = (fx - dx) * SYM_AH_V_COUNT;
            c = SYM_AH_V_START + c;
            if (dx >= -4 && dx <=4) {
                backend->write(x + dx, y - dy, false, "%c", c);
            }
        }
    }
    backend->write(x-1,y, false, "%c%c%c", SYM_AH_CENTER_LINE_LEFT, SYM_AH_CENTER, SYM_AH_CENTER_LINE_RIGHT);
}

void AP_OSD_Screen::draw_distance(uint8_t x, uint8_t y, float distance)
{
    char unit_icon = u_icon(DISTANCE);
    float distance_scaled = u_scale(DISTANCE, distance);
    const char *fmt = "%4.0f%c";
    if (distance_scaled > 9999.0f) {
        distance_scaled = u_scale(DISTANCE_LONG, distance);
        unit_icon= u_icon(DISTANCE_LONG);
        //try to pack as many useful info as possible
        if (distance_scaled<9.0f) {
            fmt = "%1.3f%c";
        } else if (distance_scaled < 99.0f) {
            fmt = "%2.2f%c";
        } else if (distance_scaled < 999.0f) {
            fmt = "%3.1f%c";
        } else {
            fmt = "%4.0f%c";
        }
    }
    backend->write(x, y, false, fmt, (double)distance_scaled, unit_icon);
}

void AP_OSD_Screen::draw_home(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    Location loc;
    if (ahrs.get_position(loc) && ahrs.home_is_set()) {
        const Location &home_loc = ahrs.get_home();
        float distance = get_distance(home_loc, loc);
        int32_t angle = wrap_360_cd(get_bearing_cd(loc, home_loc) - ahrs.yaw_sensor);
        int32_t interval = 36000 / SYM_ARROW_COUNT;
        if (distance < 2.0f) {
            //avoid fast rotating arrow at small distances
            angle = 0;
        }
        char arrow = SYM_ARROW_START + ((angle + interval / 2) / interval) % SYM_ARROW_COUNT;
        backend->write(x, y, false, "%c%c", SYM_HOME, arrow);
        draw_distance(x+2, y, distance);
    } else {
        backend->write(x, y, true, "%c", SYM_HOME);
    }
}

void AP_OSD_Screen::draw_heading(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    uint16_t yaw = ahrs.yaw_sensor / 100;
    backend->write(x, y, false, "%3d%c", yaw, SYM_DEGR);
}

void AP_OSD_Screen::draw_throttle(uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%3d%c", gcs().get_hud_throttle(), SYM_PCNT);
}

//Thanks to betaflight/inav for simple and clean compass visual design
void AP_OSD_Screen::draw_compass(uint8_t x, uint8_t y)
{
    const int8_t total_sectors = 16;
    static const char compass_circle[total_sectors] = {
        SYM_HEADING_N,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_E,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_S,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_W,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
    };
    AP_AHRS &ahrs = AP::ahrs();
    int32_t yaw = ahrs.yaw_sensor;
    int32_t interval = 36000 / total_sectors;
    int8_t center_sector = ((yaw + interval / 2) / interval) % total_sectors;
    for (int8_t i = -4; i <= 4; i++) {
        int8_t sector = center_sector + i;
        sector = (sector + total_sectors) % total_sectors;
        backend->write(x + i, y, false,  "%c", compass_circle[sector]);
    }
}

void AP_OSD_Screen::draw_wind(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    Vector3f v = ahrs.wind_estimate();
    if (check_option(AP_OSD::OPTION_INVERTED_WIND)) {
        v = -v;
    }
    backend->write(x, y, false, "%c", SYM_WSPD);
    draw_speed_vector(x + 1, y, Vector2f(v.x, v.y), ahrs.yaw_sensor);
}

void AP_OSD_Screen::draw_aspeed(uint8_t x, uint8_t y)
{
    float aspd = 0.0f;
    bool have_estimate = AP::ahrs().airspeed_estimate(&aspd);
    if (have_estimate) {
        backend->write(x, y, false, "%c%4d%c", SYM_ASPD, (int)u_scale(SPEED, aspd), u_icon(SPEED));
    } else {
        backend->write(x, y, false, "%c ---%c", SYM_ASPD, u_icon(SPEED));
    }
}

void AP_OSD_Screen::draw_vspeed(uint8_t x, uint8_t y)
{
    Vector3f v;
    float vspd;
    if (AP::ahrs().get_velocity_NED(v)) {
        vspd = -v.z;
    } else {
        vspd = AP::baro().get_climb_rate();
    }
    char sym;
    if (vspd > 3.0f) {
        sym = SYM_UP_UP;
    } else if (vspd >=0.0f) {
        sym = SYM_UP;
    } else if (vspd >= -3.0f) {
        sym = SYM_DOWN;
    } else {
        sym = SYM_DOWN_DOWN;
    }
    vspd = fabsf(vspd);
    backend->write(x, y, false, "%c%2d%c", sym, (int)u_scale(VSPEED, vspd), u_icon(VSPEED));
}

#ifdef HAVE_AP_BLHELI_SUPPORT

void AP_OSD_Screen::draw_blh_temp(uint8_t x, uint8_t y)
{
    AP_BLHeli *blheli = AP_BLHeli::get_singleton();
    if (blheli) {
        AP_BLHeli::telem_data td;
        // first parameter is index into array of ESC's.  Hardwire to zero (first) for now.
        if (!blheli->get_telem_data(0, td)) {
            return;
        }

        // AP_BLHeli & blh = AP_BLHeli::AP_BLHeli();
        uint8_t esc_temp = td.temperature;
        backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, esc_temp), u_icon(TEMPERATURE));
    }
}

void AP_OSD_Screen::draw_blh_rpm(uint8_t x, uint8_t y)
{
    AP_BLHeli *blheli = AP_BLHeli::get_singleton();
    if (blheli) {
        AP_BLHeli::telem_data td;
        // first parameter is index into array of ESC's.  Hardwire to zero (first) for now.
        if (!blheli->get_telem_data(0, td)) {
            return;
        }

        int esc_rpm = td.rpm * 14;   // hard-wired assumption for now that motor has 14 poles, so multiply eRPM * 14 to get motor RPM.
        backend->write(x, y, false, "%5d%c", esc_rpm, SYM_RPM);
    }
}

void AP_OSD_Screen::draw_blh_amps(uint8_t x, uint8_t y)
{
    AP_BLHeli *blheli = AP_BLHeli::get_singleton();
    if (blheli) {
        AP_BLHeli::telem_data td;
        // first parameter is index into array of ESC's.  Hardwire to zero (first) for now.
        if (!blheli->get_telem_data(0, td)) {
            return;
        }

        float esc_amps = td.current;
        backend->write(x, y, false, "%4.1f%c", esc_amps, SYM_AMP);
    }
}
#endif  //HAVE_AP_BLHELI_SUPPORT

void AP_OSD_Screen::draw_gps_latitude(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    const Location &loc = gps.location();   // loc.lat and loc.lng
    int32_t dec_portion, frac_portion;
    int32_t abs_lat = labs(loc.lat);

    dec_portion = loc.lat / 10000000L;
    frac_portion = abs_lat - labs(dec_portion)*10000000UL;

    backend->write(x, y, false, "%c%4ld.%07ld", SYM_GPS_LAT, (long)dec_portion,(long)frac_portion);
}

void AP_OSD_Screen::draw_gps_longitude(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    const Location &loc = gps.location();   // loc.lat and loc.lng
    int32_t dec_portion, frac_portion;
    int32_t abs_lon = labs(loc.lng);

    dec_portion = loc.lng / 10000000L;
    frac_portion = abs_lon - labs(dec_portion)*10000000UL;

    backend->write(x, y, false, "%c%4ld.%07ld", SYM_GPS_LONG, (long)dec_portion,(long)frac_portion);
}

void AP_OSD_Screen::draw_roll_angle(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    uint16_t roll = abs(ahrs.roll_sensor) / 100;
    char r;
    if (ahrs.roll_sensor > 50) {
        r = SYM_ROLLR;
    } else if (ahrs.roll_sensor < -50) {
        r = SYM_ROLLL;
    } else {
        r = SYM_ROLL0;
    }
    backend->write(x, y, false, "%c%3d%c", r, roll, SYM_DEGR);
}

void AP_OSD_Screen::draw_pitch_angle(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    uint16_t pitch = abs(ahrs.pitch_sensor) / 100;
    char p;
    if (ahrs.pitch_sensor > 50) {
        p = SYM_PTCHUP;
    } else if (ahrs.pitch_sensor < -50) {
        p = SYM_PTCHDWN;
    } else {
        p = SYM_PTCH0;
    }
    backend->write(x, y, false, "%c%3d%c", p, pitch, SYM_DEGR);
}

void AP_OSD_Screen::draw_temp(uint8_t x, uint8_t y)
{
    AP_Baro &barometer = AP::baro();
    float tmp = barometer.get_temperature();
    backend->write(x, y, false, "%3d%c", (int)u_scale(TEMPERATURE, tmp), u_icon(TEMPERATURE));
}


void AP_OSD_Screen::draw_hdop(uint8_t x, uint8_t y)
{
    AP_GPS & gps = AP::gps();
    float hdp = gps.get_hdop() / 100.0f;
    backend->write(x, y, false, "%c%c%3.2f", SYM_HDOP_L, SYM_HDOP_R, hdp);
}

void AP_OSD_Screen::draw_waypoint(uint8_t x, uint8_t y)
{
    AP_AHRS &ahrs = AP::ahrs();
    int32_t angle = wrap_360_cd(osd->nav_info.wp_bearing - ahrs.yaw_sensor);
    int32_t interval = 36000 / SYM_ARROW_COUNT;
    if (osd->nav_info.wp_distance < 2.0f) {
        //avoid fast rotating arrow at small distances
        angle = 0;
    }
    char arrow = SYM_ARROW_START + ((angle + interval / 2) / interval) % SYM_ARROW_COUNT;
    backend->write(x,y, false, "%c%2u%c",SYM_WPNO, osd->nav_info.wp_number, arrow);
    draw_distance(x+4, y, osd->nav_info.wp_distance);
}

void AP_OSD_Screen::draw_xtrack_error(uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%c%4d", SYM_XERR, (int)osd->nav_info.wp_xtrack_error);
}

void AP_OSD_Screen::draw_stat(uint8_t x, uint8_t y)
{
    backend->write(x+2, y, false, "%c%c%c", 0x4d,0x41,0x58);
    backend->write(x, y+1, false, "%c",SYM_GSPD);
    backend->write(x+1, y+1, false, "%4d%c", (int)u_scale(SPEED, osd->max_speed_mps), u_icon(SPEED));
    backend->write(x, y+2, false, "%5.1f%c", (double)osd->max_current_a, SYM_AMP);
    backend->write(x, y+3, false, "%5d%c", (int)u_scale(ALTITUDE, osd->max_alt_m), u_icon(ALTITUDE));
    backend->write(x, y+4, false, "%c", SYM_HOME);
    draw_distance(x+1, y+4, osd->max_dist_m); 
    backend->write(x, y+5, false, "%c", SYM_DIST);
    draw_distance(x+1, y+5, osd->last_distance_m);  
}

void AP_OSD_Screen::draw_dist(uint8_t x, uint8_t y)
{
    backend->write(x, y, false, "%c", SYM_DIST);
    draw_distance(x+1, y, osd->last_distance_m);   
}

void  AP_OSD_Screen::draw_flightime(uint8_t x, uint8_t y)
{
    AP_Stats *stats = AP::stats();
    if (stats) {
        uint32_t t = stats->get_flight_time_s();
        backend->write(x, y, false, "%c%3u:%02u", SYM_FLY, t/60, t%60);
    } 
}

void AP_OSD_Screen::draw_eff(uint8_t x, uint8_t y)
{
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    AP_AHRS &ahrs = AP::ahrs();
    Vector2f v = ahrs.groundspeed_vector();
    float speed = u_scale(SPEED,v.length());
    if (speed > 2.0){
        backend->write(x, y, false, "%c%3d%c", SYM_EFF,int(1000*battery.current_amps()/speed),SYM_MAH);
    } else {
        backend->write(x, y, false, "%c---%c", SYM_EFF,SYM_MAH);
    }
}

void AP_OSD_Screen::draw_climbeff(uint8_t x, uint8_t y)
{
    char unit_icon = u_icon(DISTANCE);
    Vector3f v;
    float vspd;
    if (AP::ahrs().get_velocity_NED(v)) {
        vspd = -v.z;
    } else {
        vspd = AP::baro().get_climb_rate();
    }
    if (vspd < 0.0) vspd = 0.0;
    AP_BattMonitor &battery = AP_BattMonitor::battery();
    float amps = battery.current_amps();
    if (amps > 0.0) {
        backend->write(x, y, false,"%c%c%3.1f%c",SYM_PTCHUP,SYM_EFF,(3.6 * u_scale(VSPEED,vspd)/amps),unit_icon);
    } else {
        backend->write(x, y, false,"%c%c---%c",SYM_PTCHUP,SYM_EFF,unit_icon);
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
    DRAW_SETTING(compass);
    DRAW_SETTING(altitude);
    DRAW_SETTING(waypoint);
    DRAW_SETTING(xtrack_error);
    DRAW_SETTING(bat_volt);
    DRAW_SETTING(rssi);
    DRAW_SETTING(current);
    DRAW_SETTING(batused);
    DRAW_SETTING(sats);
    DRAW_SETTING(fltmode);
    DRAW_SETTING(gspeed);
    DRAW_SETTING(aspeed);
    DRAW_SETTING(vspeed);
    DRAW_SETTING(throttle);
    DRAW_SETTING(heading);
    DRAW_SETTING(wind);
    DRAW_SETTING(home);
    DRAW_SETTING(roll_angle);
    DRAW_SETTING(pitch_angle);
    DRAW_SETTING(temp);
    DRAW_SETTING(hdop);
    DRAW_SETTING(flightime);

#ifdef HAVE_AP_BLHELI_SUPPORT
    DRAW_SETTING(blh_temp);
    DRAW_SETTING(blh_rpm);
    DRAW_SETTING(blh_amps);
#endif

    DRAW_SETTING(gps_latitude);
    DRAW_SETTING(gps_longitude);
    DRAW_SETTING(dist);
    DRAW_SETTING(stat);
    DRAW_SETTING(climbeff);
    DRAW_SETTING(eff);
}

