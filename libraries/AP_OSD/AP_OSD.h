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

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_BLHeli/AP_BLHeli.h>

class AP_OSD_Backend;

#define AP_OSD_NUM_SCREENS 4

/*
  class to hold one setting
 */
class AP_OSD_Setting {
public:
    AP_Int8 enabled;
    AP_Int8 xpos;
    AP_Int8 ypos;

    AP_OSD_Setting(bool enabled, uint8_t x, uint8_t y);

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];
};


/*
  class to hold one screen of settings
 */
class AP_OSD_Screen {
public:
    // constructor
    AP_OSD_Screen();

    void draw(void);

    void set_backend(AP_OSD_Backend *_backend)
    {
        backend = _backend;
    };

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 enabled;
    AP_Int16 channel_min;
    AP_Int16 channel_max;

private:
    AP_OSD_Backend *backend;

    static const uint16_t message_show_time_ms = 20000;
    static const uint8_t message_visible_width = 26;
    static const uint8_t message_scroll_time_ms = 200;
    static const uint8_t message_scroll_delay = 5;

    static constexpr float ah_max_roll = DEG_TO_RAD * 40;
    static constexpr float ah_max_pitch = DEG_TO_RAD * 20;
    //typical fpv camera has 80deg vertical field of view, 16 row of chars
    static constexpr float ah_pitch_rad_to_char = 16.0f/(DEG_TO_RAD * 80);

    AP_OSD_Setting altitude{true, 1, 1};
    AP_OSD_Setting bat_volt{true, 9, 1};
    AP_OSD_Setting rssi{false, 0, 0};
    AP_OSD_Setting current{true, 1, 2};
    AP_OSD_Setting batused{true, 1, 3};
    AP_OSD_Setting sats{true, 1, 4};
    AP_OSD_Setting fltmode{true, 12, 14};
    AP_OSD_Setting message{false, 2, 13};
    AP_OSD_Setting gspeed{false, 0, 0};
    AP_OSD_Setting horizon{true, 15, 8};
    AP_OSD_Setting home{false, 0, 0};
    AP_OSD_Setting throttle{false, 0, 0};
    AP_OSD_Setting heading{false, 0, 0};
    AP_OSD_Setting compass{false, 0, 0};
    AP_OSD_Setting wind{false, 0, 0};
    AP_OSD_Setting aspeed{false, 0, 0};
    AP_OSD_Setting vspeed{false, 0, 0};
    
#ifdef HAVE_AP_BLHELI_SUPPORT
    AP_OSD_Setting blh_temp{false, 0, 0};
    AP_OSD_Setting blh_rpm{false, 0, 0};
    AP_OSD_Setting blh_amps{false, 0, 0};
#endif
    
    AP_OSD_Setting gps_latitude{false, 0, 0};
    AP_OSD_Setting gps_longitude{false, 0, 0};

    void draw_altitude(uint8_t x, uint8_t y);
    void draw_bat_volt(uint8_t x, uint8_t y);
    void draw_rssi(uint8_t x, uint8_t y);
    void draw_current(uint8_t x, uint8_t y);
    void draw_batused(uint8_t x, uint8_t y);
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
    void draw_vspeed(uint8_t x, uint8_t y);

    void draw_speed_vector(uint8_t x, uint8_t y, Vector2f v, int32_t yaw);

#ifdef HAVE_AP_BLHELI_SUPPORT
    void draw_blh_temp(uint8_t x, uint8_t y);
    void draw_blh_rpm(uint8_t x, uint8_t y);
    void draw_blh_amps(uint8_t x, uint8_t y);
#endif
    
    void draw_gps_latitude(uint8_t x, uint8_t y);
    void draw_gps_longitude(uint8_t x, uint8_t y);
};

class AP_OSD {
public:
    //constructor
    AP_OSD();

    /* Do not allow copies */
    AP_OSD(const AP_OSD &other) = delete;
    AP_OSD &operator=(const AP_OSD&) = delete;

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

    enum {
        OPTION_DECIMAL_PACK = 1U<<0,
    };
    
    AP_Int32 options;

    AP_OSD_Screen screen[AP_OSD_NUM_SCREENS];

private:
    void osd_thread();
    void update_osd();
    void update_current_screen();
    void next_screen();
    AP_OSD_Backend *backend;
    uint32_t last_update_ms;

    //variables for screen switching
    uint8_t current_screen;
    uint16_t previous_channel_value;
    bool switch_debouncer;
    uint32_t last_switch_ms;
};

