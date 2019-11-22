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
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_BLHeli/AP_BLHeli.h>

#ifndef OSD_ENABLED
#define OSD_ENABLED 0
#endif

class AP_OSD_Backend;

#define AP_OSD_NUM_SCREENS 4

/*
  class to hold one setting
 */
class AP_OSD_Setting
{
public:
    AP_Int8 enabled;
    AP_Int8 xpos;
    AP_Int8 ypos;

    AP_OSD_Setting(bool enabled, uint8_t x, uint8_t y);

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];
};

class AP_OSD;

/*
  class to hold one screen of settings
 */
class AP_OSD_Screen
{
public:
    // constructor
    AP_OSD_Screen();

    void draw(void);

    void set_backend(AP_OSD_Backend *_backend);

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 enabled;
    AP_Int16 channel_min;
    AP_Int16 channel_max;

private:
    AP_OSD_Backend *backend;
    AP_OSD *osd;

    static const uint8_t message_visible_width = 26;
    static const uint8_t message_scroll_time_ms = 200;
    static const uint8_t message_scroll_delay = 5;

    static constexpr float ah_max_pitch = DEG_TO_RAD * 20;
    //typical fpv camera has 80deg vertical field of view, 16 row of chars
    static constexpr float ah_pitch_rad_to_char = 16.0f/(DEG_TO_RAD * 80);

    AP_OSD_Setting altitude{true, 23, 8};
    AP_OSD_Setting bat_volt{true, 24, 1};
    AP_OSD_Setting rssi{true, 1, 1};
    AP_OSD_Setting current{true, 25, 2};
    AP_OSD_Setting batused{true, 23, 3};
    AP_OSD_Setting sats{true, 1, 3};
    AP_OSD_Setting fltmode{true, 2, 8};
    AP_OSD_Setting message{true, 2, 6};
    AP_OSD_Setting gspeed{true, 2, 14};
    AP_OSD_Setting horizon{true, 14, 8};
    AP_OSD_Setting home{true, 14, 1};
    AP_OSD_Setting throttle{true, 24, 11};
    AP_OSD_Setting heading{true, 13, 2};
    AP_OSD_Setting compass{true, 15, 3};
    AP_OSD_Setting wind{false, 2, 12};
    AP_OSD_Setting aspeed{false, 2, 13};
    AP_OSD_Setting aspd1{false, 0, 0};
    AP_OSD_Setting aspd2{false, 0, 0};
    AP_OSD_Setting vspeed{true, 24, 9};

#ifdef HAVE_AP_BLHELI_SUPPORT
    AP_OSD_Setting blh_temp {false, 24, 13};
    AP_OSD_Setting blh_rpm{false, 22, 12};
    AP_OSD_Setting blh_amps{false, 24, 14};
#endif

    AP_OSD_Setting gps_latitude{true, 9, 13};
    AP_OSD_Setting gps_longitude{true, 9, 14};
    AP_OSD_Setting roll_angle{false, 0, 0};
    AP_OSD_Setting pitch_angle{false, 0, 0};
    AP_OSD_Setting temp{false, 0, 0};
    AP_OSD_Setting btemp{false, 0, 0};
    AP_OSD_Setting hdop{false, 0, 0};
    AP_OSD_Setting waypoint{false, 0, 0};
    AP_OSD_Setting xtrack_error{false, 0, 0};
    AP_OSD_Setting dist{false,22,11};
    AP_OSD_Setting stat{false,0,0};
    AP_OSD_Setting flightime{false, 23, 10};
    AP_OSD_Setting climbeff{false,0,0};
    AP_OSD_Setting eff{false, 22, 10};
    AP_OSD_Setting atemp{false, 0, 0};
    AP_OSD_Setting bat2_vlt{false, 0, 0};
    AP_OSD_Setting bat2used{false, 0, 0};
    AP_OSD_Setting clk{false, 0, 0};

    bool check_option(uint32_t option);

    enum unit_type {
        ALTITUDE=0,
        SPEED=1,
        VSPEED=2,
        DISTANCE=3,
        DISTANCE_LONG=4,
        TEMPERATURE=5,
        UNIT_TYPE_LAST=6,
    };

    char u_icon(enum unit_type unit);
    float u_scale(enum unit_type unit, float value);

    void draw_altitude(uint8_t x, uint8_t y);
    void draw_bat_volt(uint8_t x, uint8_t y);
    void draw_rssi(uint8_t x, uint8_t y);
    void draw_current(uint8_t x, uint8_t y);
    void draw_batused(uint8_t x, uint8_t y);
    void draw_batused(uint8_t instance, uint8_t x, uint8_t y);
    void draw_sats(uint8_t x, uint8_t y);
    void draw_fltmode(uint8_t x, uint8_t y);
    void draw_message(uint8_t x, uint8_t y);
    void draw_gspeed(uint8_t x, uint8_t y);
    void draw_horizon(uint8_t x, uint8_t y);
    void draw_home(uint8_t x, uint8_t y);
    void draw_throttle(uint8_t x, uint8_t y);
    void draw_heading(uint8_t x, uint8_t y);
    void draw_compass(uint8_t x, uint8_t y);
    void draw_wind(uint8_t x, uint8_t y);
    void draw_aspeed(uint8_t x, uint8_t y);
    void draw_aspd1(uint8_t x, uint8_t y);
    void draw_aspd2(uint8_t x, uint8_t y);
    void draw_vspeed(uint8_t x, uint8_t y);

    //helper functions
    void draw_speed_vector(uint8_t x, uint8_t y, Vector2f v, int32_t yaw);
    void draw_distance(uint8_t x, uint8_t y, float distance);

#ifdef HAVE_AP_BLHELI_SUPPORT
    void draw_blh_temp(uint8_t x, uint8_t y);
    void draw_blh_rpm(uint8_t x, uint8_t y);
    void draw_blh_amps(uint8_t x, uint8_t y);
#endif

    void draw_gps_latitude(uint8_t x, uint8_t y);
    void draw_gps_longitude(uint8_t x, uint8_t y);
    void draw_roll_angle(uint8_t x, uint8_t y);
    void draw_pitch_angle(uint8_t x, uint8_t y);
    void draw_temp(uint8_t x, uint8_t y);
    void draw_btemp(uint8_t x, uint8_t y);
    void draw_hdop(uint8_t x, uint8_t y);
    void draw_waypoint(uint8_t x, uint8_t y);
    void draw_xtrack_error(uint8_t x, uint8_t y);
    void draw_dist(uint8_t x, uint8_t y);
    void draw_stat(uint8_t x, uint8_t y);
    void draw_flightime(uint8_t x, uint8_t y);
    void draw_climbeff(uint8_t x, uint8_t y);
    void draw_eff(uint8_t x, uint8_t y);
    void draw_atemp(uint8_t x, uint8_t y);
    void draw_bat2_vlt(uint8_t x, uint8_t y);
    void draw_bat2used(uint8_t x, uint8_t y);
    void draw_clk(uint8_t x, uint8_t y);
};

class AP_OSD
{
public:
    friend class AP_OSD_Screen;
    //constructor
    AP_OSD();

    /* Do not allow copies */
    AP_OSD(const AP_OSD &other) = delete;
    AP_OSD &operator=(const AP_OSD&) = delete;

    // get singleton instance
    static AP_OSD *get_singleton()
    {
        return _singleton;
    }

    // init - perform required initialisation
    void init();

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    enum osd_types {
        OSD_NONE=0,
        OSD_MAX7456=1,
        OSD_SITL=2,
    };
    enum switch_method {
        TOGGLE=0,
        PWM_RANGE=1,
        AUTO_SWITCH=2,
    };

    AP_Int8 osd_type;
    AP_Int8 rc_channel;
    AP_Int8 sw_method;
    AP_Int8 font_num;

    AP_Int8 v_offset;
    AP_Int8 h_offset;

    AP_Int8 warn_rssi;
    AP_Int8 warn_nsat;
    AP_Float warn_batvolt;
    AP_Float warn_bat2volt;
    AP_Int8 msgtime_s;
    AP_Int8 arm_scr;
    AP_Int8 disarm_scr;
    AP_Int8 failsafe_scr;

    enum {
        OPTION_DECIMAL_PACK = 1U<<0,
        OPTION_INVERTED_WIND = 1U<<1,
        OPTION_INVERTED_AH_ROLL = 1U<<2,
    };

    AP_Int32 options;

    enum {
        UNITS_METRIC=0,
        UNITS_IMPERIAL=1,
        UNITS_SI=2,
        UNITS_AVIATION=3,
        UNITS_LAST=4,
    };

    AP_Int8 units;

    AP_OSD_Screen screen[AP_OSD_NUM_SCREENS];

    struct NavInfo {
        float wp_distance;
        int32_t wp_bearing;
        float wp_xtrack_error;
        uint16_t wp_number;
    };

    void set_nav_info(NavInfo &nav_info);
    // disable the display
    void disable() {
        _disable = true;
    }
    // enable the display
    void enable() {
        _disable = false;
    }

private:
    void osd_thread();
    void update_osd();
    void stats();
    void update_current_screen();
    void next_screen();
    AP_OSD_Backend *backend;

    //variables for screen switching
    uint8_t current_screen;
    uint16_t previous_channel_value;
    bool switch_debouncer;
    uint32_t last_switch_ms;
    struct NavInfo nav_info;
    int8_t previous_pwm_screen;
    int8_t pre_fs_screen;
    bool was_armed;
    bool was_failsafe;
    bool _disable;

    uint32_t last_update_ms;
    float last_distance_m;
    float max_dist_m;
    float max_alt_m;
    float max_speed_mps;
    float max_current_a;
    float avg_current_a;

    static AP_OSD *_singleton;
};

namespace AP
{
AP_OSD *osd();
};
