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

    AP_OSD_Setting altitude{true, 1, 1};
    AP_OSD_Setting bat_volt{true, 9, 1};
    AP_OSD_Setting rssi{false, 0, 0};
    AP_OSD_Setting current{true, 1, 2};
    AP_OSD_Setting batused{true, 1, 3};
    AP_OSD_Setting sats{true, 1, 4};
    AP_OSD_Setting fltmode{true, 12, 14};
    AP_OSD_Setting message{false, 0, 0};

    void draw_altitude(uint8_t x, uint8_t y);
    void draw_bat_volt(uint8_t x, uint8_t y);
    void draw_rssi(uint8_t x, uint8_t y);
    void draw_current(uint8_t x, uint8_t y);
    void draw_batused(uint8_t x, uint8_t y);
    void draw_sats(uint8_t x, uint8_t y);
    void draw_fltmode(uint8_t x, uint8_t y);
    void draw_message(uint8_t x, uint8_t y);
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
    };

    AP_Int8 osd_type;
    AP_Int8 rc_channel;

    AP_OSD_Screen screen[AP_OSD_NUM_SCREENS];

private:
    void timer();
    void update_osd();
    void update_current_screen();
    AP_OSD_Backend *backend;
    uint32_t last_update_ms;

    uint8_t current_screen;
};
