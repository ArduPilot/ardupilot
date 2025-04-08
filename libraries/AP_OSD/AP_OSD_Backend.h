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

#include <AP_HAL/HAL.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_Filesystem/AP_Filesystem.h>


class AP_OSD_Backend
{

public:
    //constructor
    AP_OSD_Backend(AP_OSD &osd): _osd(osd) {}

    //destructor
    virtual ~AP_OSD_Backend(void) {}

    //draw given text to framebuffer
    virtual void write(uint8_t x, uint8_t y, const char* text) = 0;

    //draw formatted text to framebuffer
    virtual void write(uint8_t x, uint8_t y, bool blink, const char *fmt, ...) FMT_PRINTF(5, 6);

    //initilize framebuffer and underlying hardware
    virtual bool init() = 0;

    //update screen
    virtual void flush() = 0;

    // return a correction factor used to display angles correctly
    virtual float get_aspect_ratio_correction() const {return 1;}

    //clear screen
    //should match hw blink
    virtual void clear()
    {
        blink_phase = (blink_phase+1)%4;
    }

    // copy the backend specific symbol set to the OSD lookup table
    virtual void init_symbol_set(uint8_t *symbols, const uint8_t size);
    virtual bool is_compatible_with_backend_type(AP_OSD::osd_types type) const = 0;
    virtual AP_OSD::osd_types get_backend_type() const = 0;

    // called by the OSD thread once
    virtual void osd_thread_run_once() { return; }

    AP_OSD * get_osd()
    {
        return &_osd;
    }

protected:
    AP_OSD& _osd;

    // get font choice
    uint8_t get_font_num(void) const
    {
        return (uint8_t)_osd.font_num.get();
    }

    //check option
    bool check_option(uint32_t option) const
    {
        return (_osd.options & option) != 0;
    }

    uint8_t convert_to_decimal_packed_characters(char* buff, uint8_t size);
    virtual uint8_t format_string_for_osd(char* dst, uint8_t size, bool decimal_packed, const char *fmt, va_list ap);

    // load a font from sdcard or ROMFS
    FileData *load_font_data(uint8_t font_num);

    int8_t blink_phase;

    enum vid_format {
        FORMAT_UNKNOWN = 0,
        FORMAT_NTSC = 1,
        FORMAT_PAL = 2,
    } _format;

    // default OSD symbols
    static const uint8_t SYM_M = 0xB9;
    static const uint8_t SYM_KM = 0xBA;
    static const uint8_t SYM_FT = 0x0F;
    static const uint8_t SYM_MI = 0xBB;
    static const uint8_t SYM_ALT_M = 0xB1;
    static const uint8_t SYM_ALT_FT = 0xB3;
    static const uint8_t SYM_BATT_FULL = 0x90;
    static const uint8_t SYM_RSSI = 0x01;

    static const uint8_t SYM_VOLT = 0x06;
    static const uint8_t SYM_AMP = 0x9A;
    static const uint8_t SYM_MAH = 0x07;
    static const uint8_t SYM_MS = 0x9F;
    static const uint8_t SYM_FS = 0x99;
    static const uint8_t SYM_KMH = 0xA1;
    static const uint8_t SYM_MPH = 0xB0;
    static const uint8_t SYM_DEGR = 0xA8;
    static const uint8_t SYM_PCNT = 0x25;
    static const uint8_t SYM_RPM = 0xE0;
    static const uint8_t SYM_ASPD = 0xE1;
    static const uint8_t SYM_GSPD = 0xE2;
    static const uint8_t SYM_WSPD = 0xE3;
    static const uint8_t SYM_VSPD = 0xE4;
    static const uint8_t SYM_WPNO = 0xE5;
    static const uint8_t SYM_WPDIR = 0xE6;
    static const uint8_t SYM_WPDST = 0xE7;
    static const uint8_t SYM_FTMIN = 0xE8;
    static const uint8_t SYM_FTSEC = 0x99;

    static const uint8_t SYM_SAT_L = 0x1E;
    static const uint8_t SYM_SAT_R = 0x1F;
    static const uint8_t SYM_HDOP_L = 0xBD;
    static const uint8_t SYM_HDOP_R = 0xBE;

    static const uint8_t SYM_HOME = 0xBF;
    static const uint8_t SYM_WIND = 0x16;

    static const uint8_t SYM_ARROW_START = 0x60;
    static const uint8_t SYM_ARROW_COUNT = 16;
    static const uint8_t SYM_AH_H_START = 0x80;
    static const uint8_t SYM_AH_H_COUNT = 9;

    static const uint8_t SYM_AH_V_START = 0xCA;
    static const uint8_t SYM_AH_V_COUNT = 6;

    static const uint8_t SYM_AH_CENTER_LINE_LEFT = 0x26;
    static const uint8_t SYM_AH_CENTER_LINE_RIGHT = 0x27;
    static const uint8_t SYM_AH_CENTER = 0x7E;

    static const uint8_t SYM_HEADING_N = 0x18;
    static const uint8_t SYM_HEADING_S = 0x19;
    static const uint8_t SYM_HEADING_E = 0x1A;
    static const uint8_t SYM_HEADING_W = 0x1B;
    static const uint8_t SYM_HEADING_DIVIDED_LINE = 0x1C;
    static const uint8_t SYM_HEADING_LINE = 0x1D;

    static const uint8_t SYM_UP_UP = 0xA2;
    static const uint8_t SYM_UP = 0xA3;
    static const uint8_t SYM_DOWN = 0xA4;
    static const uint8_t SYM_DOWN_DOWN = 0xA5;

    static const uint8_t SYM_DEGREES_C = 0x0E;
    static const uint8_t SYM_DEGREES_F = 0x0D;
    static const uint8_t SYM_GPS_LAT = 0xA6;
    static const uint8_t SYM_GPS_LONG = 0xA7;
    static const uint8_t SYM_ARMED = 0x00;
    static const uint8_t SYM_DISARMED = 0xE9;
    static const uint8_t SYM_ROLL0 = 0x2D;
    static const uint8_t SYM_ROLLR = 0xEA;
    static const uint8_t SYM_ROLLL = 0xEB;
    static const uint8_t SYM_PTCH0 = 0x7C;
    static const uint8_t SYM_PTCHUP = 0xEC;
    static const uint8_t SYM_PTCHDWN = 0xED;
    static const uint8_t SYM_XERR = 0xEE;
    static const uint8_t SYM_KN = 0xF0;
    static const uint8_t SYM_NM = 0xF1;
    static const uint8_t SYM_DIST = 0x22;
    static const uint8_t SYM_FLY = 0x9C;
    static const uint8_t SYM_EFF = 0xF2;
    static const uint8_t SYM_AH = 0xF3;
    static const uint8_t SYM_MW = 0xF4;
    static const uint8_t SYM_CLK = 0xBC;
    static const uint8_t SYM_KILO = 0x4B;
    static const uint8_t SYM_TERALT = 0xEF;
    static const uint8_t SYM_FENCE_ENABLED = 0xF5;
    static const uint8_t SYM_FENCE_DISABLED = 0xF6;
    static const uint8_t SYM_RNGFD = 0xF7;
    static const uint8_t SYM_LQ = 0xF8;
    static const uint8_t SYM_WATT = 0xAE;
    static const uint8_t SYM_WH = 0xAB;
    static const uint8_t SYM_DB = 0xF9;
    static const uint8_t SYM_DBM = 0xFA;
    static const uint8_t SYM_SNR = 0xFB;
    static const uint8_t SYM_ANT = 0xFC;
    static const uint8_t SYM_ARROW_RIGHT = 0xFD;
    static const uint8_t SYM_ARROW_LEFT = 0xFE;
    static const uint8_t SYM_G = 0xDF;
    static const uint8_t SYM_BATT_UNKNOWN = 0x97;
    static const uint8_t SYM_ROLL = 0xA9;
    static const uint8_t SYM_PITCH = 0xAF;
    static const uint8_t SYM_DPS = 0xAA;
    static const uint8_t SYM_HEADING = 0x89;
    static const uint8_t SYM_RADIUS = 0x7A;
    static const uint8_t SYM_FLAP = 0x23;

    static const uint8_t SYM_SIDEBAR_R_ARROW = 0x09;
    static const uint8_t SYM_SIDEBAR_L_ARROW = 0x0A;
    static const uint8_t SYM_SIDEBAR_A = 0x13;
    static const uint8_t SYM_SIDEBAR_B = 0x14;
    static const uint8_t SYM_SIDEBAR_C = 0x15;
    static const uint8_t SYM_SIDEBAR_D = 0xDD;
    static const uint8_t SYM_SIDEBAR_E = 0xDB;
    static const uint8_t SYM_SIDEBAR_F = 0xDC;
    static const uint8_t SYM_SIDEBAR_G = 0xDA;
    static const uint8_t SYM_SIDEBAR_H = 0xDE;
    static const uint8_t SYM_SIDEBAR_I = 0x11;
    static const uint8_t SYM_SIDEBAR_J = 0x12;

    static constexpr uint8_t symbols[AP_OSD_NUM_SYMBOLS] {
        SYM_M,
        SYM_KM,
        SYM_FT,
        SYM_MI,
        SYM_ALT_M,
        SYM_ALT_FT,
        SYM_BATT_FULL,
        SYM_RSSI,
        SYM_VOLT,
        SYM_AMP,
        SYM_MAH,
        SYM_MS,
        SYM_FS,
        SYM_KMH,
        SYM_MPH,
        SYM_DEGR,
        SYM_PCNT,
        SYM_RPM,
        SYM_ASPD,
        SYM_GSPD,
        SYM_WSPD,
        SYM_VSPD,
        SYM_WPNO,
        SYM_WPDIR,
        SYM_WPDST,
        SYM_FTMIN,
        SYM_FTSEC,
        SYM_SAT_L,
        SYM_SAT_R,
        SYM_HDOP_L,
        SYM_HDOP_R,
        SYM_HOME,
        SYM_WIND,
        SYM_ARROW_START,
        SYM_ARROW_COUNT,
        SYM_AH_H_START,
        SYM_AH_H_COUNT,
        SYM_AH_V_START,
        SYM_AH_V_COUNT,
        SYM_AH_CENTER_LINE_LEFT,
        SYM_AH_CENTER_LINE_RIGHT,
        SYM_AH_CENTER,
        SYM_HEADING_N,
        SYM_HEADING_S,
        SYM_HEADING_E,
        SYM_HEADING_W,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_UP_UP,
        SYM_UP,
        SYM_DOWN,
        SYM_DOWN_DOWN,
        SYM_DEGREES_C,
        SYM_DEGREES_F,
        SYM_GPS_LAT,
        SYM_GPS_LONG,
        SYM_ARMED,
        SYM_DISARMED,
        SYM_ROLL0,
        SYM_ROLLR,
        SYM_ROLLL,
        SYM_PTCH0,
        SYM_PTCHUP,
        SYM_PTCHDWN,
        SYM_XERR,
        SYM_KN,
        SYM_NM,
        SYM_DIST,
        SYM_FLY,
        SYM_EFF,
        SYM_AH,
        SYM_MW,
        SYM_CLK,
        SYM_KILO,
        SYM_TERALT,
        SYM_FENCE_ENABLED,
        SYM_FENCE_DISABLED,
        SYM_RNGFD,
        SYM_LQ,
        SYM_SIDEBAR_R_ARROW,
        SYM_SIDEBAR_L_ARROW,
        SYM_SIDEBAR_A,
        SYM_SIDEBAR_B,
        SYM_SIDEBAR_C,
        SYM_SIDEBAR_D,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_H,
        SYM_SIDEBAR_I,
        SYM_SIDEBAR_J,
        SYM_WATT,
        SYM_WH,
        SYM_DB,
        SYM_DBM,
        SYM_SNR,
        SYM_ANT,
        SYM_ARROW_RIGHT,
        SYM_ARROW_LEFT,
        SYM_G,
        SYM_BATT_UNKNOWN,
        SYM_ROLL,
        SYM_PITCH,
        SYM_DPS,
        SYM_HEADING,
        SYM_RADIUS,
        SYM_FLAP,
    };
};
