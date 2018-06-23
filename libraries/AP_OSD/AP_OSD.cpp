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

#include <AP_OSD/AP_OSD.h>
#include <AP_OSD/AP_OSD_MAX7456.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_Notify/AP_Notify.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>

#include <utility>

// Character coordinate and attributes
#define VISIBLE_FLAG  0x0800
#define VISIBLE(x)    (x & VISIBLE_FLAG)
#define OSD_POS(x,y)  (x | (y << 5))
#define OSD_X(x)      (x & 0x001F)
#define OSD_Y(x)      ((x >> 5) & 0x001F)

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

#define SYM_SAT_L 0x1E
#define SYM_SAT_R 0x1F

#define AH_SYMBOL_COUNT 9

const AP_Param::GroupInfo AP_OSD::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Enable OSD
    // @Description: Enable OSD
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_OSD, osd_enabled, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: UPDATE_FONT
    // @DisplayName: Update font
    // @Description: Undate font inside osd chip
    // @Values: 0:Do not update,1:Update
    // @User: Standard
    AP_GROUPINFO("UPDATE_FONT", 1, AP_OSD, update_font, 1),

    // @Param: 1_ALTITUDE
    // @DisplayName: Altitude position
    // @Description: Altitude position and visibility
    // @Values: 2048 * visible + x + y * 16
    // @User: Advanced
    AP_GROUPINFO("1_ALTITUDE", 2, AP_OSD, pos_altitude, OSD_POS(1, 0) | VISIBLE_FLAG),

    // @Param:1_BATT_VOLTAGE
    // @DisplayName:
    // @Description:
    // @Values: 2048 * visible + x + y * 16
    // @User: Advanced
    AP_GROUPINFO("1_BAT_VOLT", 3, AP_OSD, pos_batt_voltage, OSD_POS(12, 0) | VISIBLE_FLAG),

    // @Param:1_RSSI
    // @DisplayName:
    // @Description:
    // @Values: 2048 * visible + x + y * 16
    // @User: Advanced
    AP_GROUPINFO("1_RSSI", 4, AP_OSD, pos_rssi, OSD_POS(23, 0) | VISIBLE_FLAG),

    // @Param:1_CURRENT_DRAW
    // @DisplayName:
    // @Description:
    // @Values: 2048 * visible + x + y * 16
    // @User: Advanced
    AP_GROUPINFO("1_CUR_DRAW", 5, AP_OSD, pos_current_draw, OSD_POS(1, 3) | VISIBLE_FLAG),

    // @Param:1_MAH_DRAWN
    // @DisplayName:
    // @Description:
    // @Values: 2048 * visible + x + y * 16
    // @User: Advanced
    AP_GROUPINFO("1_MAH_DRAWN", 6, AP_OSD, pos_mah_drawn, OSD_POS(1, 4) | VISIBLE_FLAG),

    // @Param:1_GPS_SATS
    // @DisplayName:
    // @Description:
    // @Values: 2048 * visible + x + y * 16
    // @User: Advanced
    AP_GROUPINFO("1_GPS_SATS", 7, AP_OSD, pos_gps_sats, OSD_POS(0, 11) | VISIBLE_FLAG),

    // @Param:1_FLYMODE
    // @DisplayName:
    // @Description:
    // @Values: 2048 * visible + x + y * 16
    // @User: Advanced
    AP_GROUPINFO("1_FLYMODE", 8, AP_OSD, pos_flymode, OSD_POS(12, 12) | VISIBLE_FLAG),

    // @Param:1_MESSAGES
    // @DisplayName:
    // @Description:
    // @Values: 2048 * visible + x + y * 16
    // @User: Advanced
    AP_GROUPINFO("1_MESSAGES", 9, AP_OSD, pos_messages, OSD_POS(1, 13) | VISIBLE_FLAG),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

AP_OSD::AP_OSD() : backend(nullptr), last_update_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OSD::init()
{
    if (!osd_enabled) {
        return;
    }
    AP_HAL::OwnPtr<AP_HAL::Device> spi_dev = std::move(hal.spi->get_device("osd"));
    if (!spi_dev) {
        return;
    }
    backend = AP_OSD_MAX7456::probe(*this, std::move(spi_dev));
    if (backend == nullptr) {
        return;
    }
    last_update_ms = 0;
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_OSD::timer, void));
}

void AP_OSD::timer()
{
    uint32_t now = AP_HAL::millis();

    if (now - last_update_ms > 16) {
        last_update_ms = now;
        update_osd();
    }
}

void AP_OSD::update_osd()
{
    backend->clear();

    draw_messages();
    draw_flymode();
    draw_altitude();
    draw_batt_voltage();
    draw_rssi();
    draw_current_draw();
    draw_mah_drawn();
    draw_gps_sats();

    backend->flush();
}

void AP_OSD::draw_altitude()
{
    if (VISIBLE(pos_altitude)) {
        uint8_t x = OSD_X(pos_altitude);
        uint8_t y = OSD_Y(pos_altitude);

        float alt;
        AP_AHRS::get_singleton()->get_relative_position_D_home(alt);
        backend->write(x, y, false, "%4.0f%c", alt, SYM_ALT_M);
    }
}

void AP_OSD::draw_batt_voltage()
{
    if (VISIBLE(pos_batt_voltage)) {
        uint8_t x = OSD_X(pos_batt_voltage);
        uint8_t y = OSD_Y(pos_batt_voltage);

        AP_BattMonitor & battery = AP_BattMonitor::battery();
        uint8_t p = battery.capacity_remaining_pct();
        p = (100 - p) / 16.6;
        backend->write(x,y, battery.has_failsafed(), "%c%2.1fV", SYM_BATT_FULL + p, battery.voltage());
    }
}

void AP_OSD::draw_rssi()
{
    if (VISIBLE(pos_rssi)) {
        uint8_t x = OSD_X(pos_rssi);
        uint8_t y = OSD_Y(pos_rssi);
        int rssi = AP_RSSI::get_instance()->read_receiver_rssi_uint8();
        rssi = (rssi * 99) / 255;
        backend->write(x, y, rssi < 5, "%c%2d", SYM_RSSI, rssi);
    }
}

void AP_OSD::draw_current_draw()
{
    if (VISIBLE(pos_current_draw)) {
        uint8_t x = OSD_X(pos_current_draw);
        uint8_t y = OSD_Y(pos_current_draw);
        AP_BattMonitor & battery = AP_BattMonitor::battery();
        float current = battery.current_amps();
        backend->write(x, y, false, "%c%2.1f",SYM_AMP, current);
    }
}

void AP_OSD::draw_flymode()
{
    if (VISIBLE(pos_flymode)) {
        uint8_t x = OSD_X(pos_flymode);
        uint8_t y = OSD_Y(pos_flymode);
        backend->write(x, y, AP_Notify::instance()->get_flight_mode_str());
    }
}

void AP_OSD::draw_gps_sats()
{
    if (VISIBLE(pos_gps_sats)) {
        uint8_t x = OSD_X(pos_gps_sats);
        uint8_t y = OSD_Y(pos_gps_sats);
        AP_GPS& gps = AP::gps();
        bool has_3dfix = gps.status() <= AP_GPS::GPS_OK_FIX_2D;
        backend->write(x, y, !has_3dfix, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, gps.num_sats());
    }
}

void AP_OSD::draw_mah_drawn()
{
    if (VISIBLE(pos_mah_drawn)) {
        uint8_t x = OSD_X(pos_mah_drawn);
        uint8_t y = OSD_Y(pos_mah_drawn);
        AP_BattMonitor & battery = AP_BattMonitor::battery();
        backend->write(x,y, battery.has_failsafed(), "%c%4.0f", SYM_MAH, battery.consumed_mah());
    }
}

void AP_OSD::draw_messages()
{
    if (VISIBLE(pos_messages)) {
        uint8_t x = OSD_X(pos_messages);
        uint8_t y = OSD_Y(pos_messages);
        AP_Notify * notify = AP_Notify::instance();
        bool text_is_valid = AP_HAL::millis() - notify->get_text_updated_millis() < 20000;
        if (text_is_valid) {
            backend->write(x, y, notify->get_text());
        }
    }
}
