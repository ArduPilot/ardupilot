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

    AP_Int8 osd_enabled;
    AP_Int8 update_font;

    AP_Int16 pos_altitude;
    AP_Int16 pos_batt_voltage;
    AP_Int16 pos_rssi;
    AP_Int16 pos_current_draw;
    AP_Int16 pos_mah_drawn;
    AP_Int16 pos_gps_sats;
    AP_Int16 pos_flymode;
    AP_Int16 pos_messages;

private:
    void timer();
    void update_osd();
    AP_OSD_Backend *backend;
    uint32_t last_update_ms;

    void draw_altitude();
    void draw_batt_voltage();
    void draw_rssi();
    void draw_current_draw();
    void draw_mah_drawn();
    void draw_horizon();
    void draw_ontime_flytime();
    void draw_gps_sats();
    void draw_flymode();
    void draw_messages();
};

