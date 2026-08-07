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
/*
  OSD backend for MSP
 */
#include <AP_MSP/AP_MSP.h>
#include <AP_MSP/msp.h>
#include "AP_OSD_MSP_DisplayPort.h"

#if HAL_WITH_MSP_DISPLAYPORT

#include <GCS_MAVLink/GCS.h>

static const struct AP_Param::defaults_table_struct defaults_table[] = {
    /*
    { "PARAM_NAME",       value_float }
    */
};

extern const AP_HAL::HAL &hal;
constexpr uint8_t AP_OSD_MSP_DisplayPort::symbols[AP_OSD_NUM_SYMBOLS];

// initialise backend
bool AP_OSD_MSP_DisplayPort::init(void)
{
    // check if we have a DisplayPort backend to use
    const AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MSP backend not available");
        return false;
    }
    _displayport = msp->find_protocol(AP_SerialManager::SerialProtocol_MSP_DisplayPort);
    if (_displayport == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MSP DisplayPort uart not available");
        return false;
    }
    // re-init port here for use in this thread
    _displayport->init_uart();
    return true;
}

// called by the OSD thread once
void AP_OSD_MSP_DisplayPort::osd_thread_run_once()
{
    if (_displayport != nullptr) {
        _displayport->init_uart();
    }
}

void AP_OSD_MSP_DisplayPort::clear(void)
{
    // check if we need to enable some options
    // but only for actual OSD screens
    if (_osd.get_current_screen() < AP_OSD_NUM_DISPLAY_SCREENS) {
        const uint8_t txt_resolution = _osd.screen[_osd.get_current_screen()].get_txt_resolution();
        const uint8_t font_index = _osd.screen[_osd.get_current_screen()].get_font_index();
        _displayport->msp_displayport_set_options(font_index, txt_resolution);
    }

    // clear remote MSP screen
    _displayport->msp_displayport_clear_screen();

    // toggle flashing @1Hz
    const uint32_t now = AP_HAL::millis();
    if ((uint32_t(now * 0.004) & 0x01) != _blink_on) {
        _blink_on = !_blink_on;
        blink_phase = (blink_phase+1)%4;
    }
}

void AP_OSD_MSP_DisplayPort::write(uint8_t x, uint8_t y, const char* text)
{
#if AP_MSP_INAV_FONTS_ENABLED
    const AP_MSP *msp = AP::msp();
    if (msp && msp->is_option_enabled(AP_MSP::Option::DISPLAYPORT_INAV_SYMBOLS)) {
        write_INAV(x, y, text);
        return;
    }
#endif
    _displayport->msp_displayport_write_string(x, y, false, text, 0);
}

#if AP_MSP_INAV_FONTS_ENABLED
void AP_OSD_MSP_DisplayPort::write_INAV(uint8_t x, uint8_t y, const char* text)
{
    uint8_t idx = 0;
    uint8_t buf_idx = 0;
    uint8_t last_inav_table = 0xFF; // 0xFF == unknown
    uint8_t x_offset = 0;
    const uint8_t max_idx = MIN((uint8_t)(strlen(text)-1), DISPLAYPORT_WRITE_BUFFER_MAX_LEN-2);
    uint8_t inav_symbol;
    uint8_t inav_table;
    while (text[idx] != 0 && idx <= max_idx)
    {
        // transcode
        inav_symbol = ap_to_inav_symbols_map[(uint8_t)text[idx]][0];
        inav_table = ap_to_inav_symbols_map[(uint8_t)text[idx]][1];
    
        if (last_inav_table == 0xFF) {
            last_inav_table = inav_table;
        }

        if (inav_table != last_inav_table) {
            displayport_write_buffer[buf_idx] = 0x00; // add a terminator
            _displayport->msp_displayport_write_string(x + x_offset, y, false, displayport_write_buffer, last_inav_table);
            x_offset += buf_idx;
            buf_idx = 0;
            last_inav_table = inav_table;
        }

        displayport_write_buffer[buf_idx++] = inav_symbol;

        if (idx == max_idx) {
            // flush when we detect end of string
            displayport_write_buffer[buf_idx] = 0x00; // add a terminator
            _displayport->msp_displayport_write_string(x+x_offset, y, false, displayport_write_buffer, inav_table);
        }
        idx++;
    }
}
#endif

uint8_t AP_OSD_MSP_DisplayPort::format_string_for_osd(char* buff, uint8_t size, bool decimal_packed, const char *fmt, va_list ap)
{
    const AP_MSP *msp = AP::msp();
    const bool pack =  decimal_packed && msp && !msp->is_option_enabled(AP_MSP::Option::DISPLAYPORT_BTFL_SYMBOLS);
    return AP_OSD_Backend::format_string_for_osd(buff, size, pack, fmt, ap);
}

void AP_OSD_MSP_DisplayPort::flush(void)
{
    // grab the screen and force a redraw
    _displayport->msp_displayport_grab();
    _displayport->msp_displayport_draw_screen();

    // ok done processing displayport data
    // let's process incoming MSP frames (and reply if needed)
    _displayport->process_incoming_data();
}

void AP_OSD_MSP_DisplayPort::init_symbol_set(uint8_t *lookup_table, const uint8_t size)
{
    const AP_MSP *msp = AP::msp();
    // do we use backend specific symbols table?
    if (msp && msp->is_option_enabled(AP_MSP::Option::DISPLAYPORT_BTFL_SYMBOLS)&& !msp->is_option_enabled(AP_MSP::Option::DISPLAYPORT_INAV_SYMBOLS)) {
        memcpy(lookup_table, symbols, size);
    } else {
        memcpy(lookup_table, AP_OSD_Backend::symbols, size);
    }
}

// override built in positions with defaults for MSP OSD
void AP_OSD_MSP_DisplayPort::setup_defaults(void)
{
    AP_Param::set_defaults_from_table(defaults_table, ARRAY_SIZE(defaults_table));
}

AP_OSD_Backend *AP_OSD_MSP_DisplayPort::probe(AP_OSD &osd)
{
    AP_OSD_MSP_DisplayPort *backend = NEW_NOTHROW AP_OSD_MSP_DisplayPort(osd);
    if (!backend) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}
 
// return a correction factor used to display angles correctly
float AP_OSD_MSP_DisplayPort::get_aspect_ratio_correction() const
{
    return 12.0/18.0;
}


#if AP_MSP_INAV_FONTS_ENABLED
/*
  the first column in the code ID, 2nd column is the code page
 */
const uint8_t AP_OSD_MSP_DisplayPort::ap_to_inav_symbols_map[256][2] {
        {0x00,0},
        {0x01,0}, //SYM_RSSI
        {0x02,0},
        {0x03,0},
        {0x04,0},
        {0x05,0},
        {0x1F,0}, //SYM_VOLT
        {0x99,0}, //SYM_MAH
        {0x08,0},
        {0x3E,0}, //SYM_SIDEBAR_R_ARROW
        {0x3C,0}, //SYM_SIDEBAR_L_ARROW
        {0x0B,0},
        {0x0C,0},
        {0x96,0}, //SYM_DEGREES_F
        {0x97,0}, //SYM_DEGREES_C
        {0x74,0}, //SYM_FT
        {0x10,0},
        {0x2E,1}, //SYM_SIDEBAR_I
        {0x31,1}, //SYM_SIDEBAR_J
        {0x2E,1}, //SYM_SIDEBAR_A
        {0x31,1}, //SYM_SIDEBAR_B
        {0x2E,1}, //SYM_SIDEBAR_C
        {0x16,0}, //SYM_WIND
        {0x17,0}, 
        {0x4E,0}, //SYM_HEADING_N
        {0x53,0}, //SYM_HEADING_S
        {0x45,0}, //SYM_HEADING_E
        {0x57,0}, //SYM_HEADING_W
        {0x7C,0}, //SYM_HEADING_DIVIDED_LINE
        {0x7C,0}, //SYM_HEADING_LINE
        {0x09,0}, //SYM_SAT_L
        {0xE0,0}, //SYM_SAT_R
        // 00-1F above        
        {0x20,0},
        {0x21,0},
        {0x75,0}, //SYM_DIST
        {0xD0,0}, //SYM_FLAP
        {0x24,0},
        {0x25,0}, //SYM_PCNT
        {0x5F,1}, //SYM_AH_CENTER_LINE_LEFT
        {0x5A,1}, //SYM_AH_CENTER_LINE_RIGHT
        {0x28,0}, //SYM_ROLL0
        {0x29,0},
        {0x2A,0},
        {0x2B,0},
        {0x2C,0},
        {0xAE,0},
        {0x2E,0},
        {0x2F,0},
        {0x30,0},
        {0x31,0},
        {0x32,0},
        {0x33,0},
        {0x34,0},
        {0x35,0},
        {0x36,0},
        {0x37,0},
        {0x38,0},
        {0x39,0},
        {0x3A,0},
        {0x3B,0},
        {0x3C,0},
        {0x3D,0},
        {0x3E,0},
        {0x3F,0},
        //20-3F above
        {0x40,0},
        {0x41,0},
        {0x42,0},
        {0x43,0},
        {0x44,0},
        {0x45,0},
        {0x46,0},
        {0x47,0},
        {0x48,0},
        {0x49,0},
        {0x4A,0},
        {0x4B,0}, //SYM_KILO
        {0x4C,0},
        {0x4D,0},
        {0x4E,0},
        {0x4F,0},
        {0x50,0},
        {0x51,0},
        {0x52,0},
        {0x53,0},
        {0x54,0},
        {0x55,0},
        {0x56,0},
        {0x57,0},
        {0x58,0},
        {0x59,0},
        {0x5A,0},
        {0x5B,0},
        {0x5C,0},
        {0x5D,0},
        {0x5E,0},
        {0x5F,0},
        //40-5F above
        {0x3C,1}, //SYM_ARROW_START,next 15 symbols for arrows
        {0x3D,1},
        {0x3E,1},
        {0x3F,1},
        {0x40,1},
        {0x41,1},
        {0x42,1},
        {0x43,1},
        {0x44,1},
        {0x45,1},
        {0x46,1},
        {0x47,1},
        {0x48,1},
        {0x49,1},
        {0x4A,1},
        {0x4B,1},
        {0x70,0},
        {0x71,0},
        {0x72,0},
        {0x73,0},
        {0x74,0},
        {0x75,0},
        {0x76,0},
        {0x77,0},
        {0x78,0},
        {0x79,0},
        {0x20,0}, //SYM_RADIUS
        {0x7B,0},
        {0xAE,0}, //SYM_PTCH0
        {0x7D,0},
        {0x66,1}, //SYM_AH_CENTER
        {0x7F,0},
        // 60-7F above
        {0x4C,1}, //SYM_AH_H_START,  and next 8 symbols
        {0x4D,1},
        {0x4E,1},
        {0x4F,1},
        {0x50,1},
        {0x51,1},
        {0x52,1},
        {0x53,1},
        {0x54,1},
        {0x0C,0}, //SYM_HEADING
        {0x8A,0},
        {0x8B,0},
        {0x8C,0},
        {0x8D,0},
        {0x8E,0},
        {0x8F,0},
        {0x63,0}, //SYM_BATT_FULL, and next 6 symbols
        {0x64,0},
        {0x65,0},
        {0x66,0},
        {0x67,0},
        {0x68,0},
        {0x69,0},
        {0xDD,0}, //SYM_BATT_FULL
        {0x98,0},
        {0x8D,0}, //SYM_FS,SYM_FS
        {0x6A,0}, //SYM_AMP
        {0x9B,0},
        {0x9F,0}, //SYM_FLY
        {0x9D,0},
        {0x9E,0},
        {0x8F,0}, //SYM_MS
        //80-9F above
        {0xA0,0},
        {0x90,0}, //SYM_KMH
        {0x55,1}, //SYM_UP_UP
        {0x56,1}, //SYM_UP
        {0x57,1}, //SYM_DOWN
        {0x58,1}, //SYM_DOWN_DOWN
        {0x03,0}, //SYM_GPS_LAT
        {0x04,0}, //SYM_GPS_LONG
        {0x0B,0}, //SYM_DEGR
        {0x52,0}, //SYM_ROLL
        {0x20,0}, //SYM_DPS
        {0x6D,0}, //SYM_WH
        {0xAC,0},
        {0xAD,0},
        {0x71,0}, //SYM_WATT
        {0x50,0}, //SYM_PITCH
        {0x91,0}, //SYM_MPH
        {0x76,0}, //SYM_ALT_M
        {0xB2,0},
        {0x78,0}, //SYM_ALT_FT
        {0xB4,0},
        {0xB5,0},
        {0xB6,0},
        {0xB7,0},
        {0xB8,0},
        {0x82,0}, //SYM_M
        {0x83,0}, //SYM_KM
        {0x84,0}, //SYM_MI
        {0xA0,0}, //SYM_CLK
        {0x0E,0}, //SYM_HDOP_L
        {0x0F,0}, //SYM_HDOP_R
        {0x10,0}, //SYM_HOME
        //A0-BF above
        {0xA1,0}, //start of decimal pack
        {0xA2,0},
        {0xA3,0},
        {0xA4,0},
        {0xA5,0},
        {0xA6,0},
        {0xA7,0},
        {0xA8,0},
        {0xA9,0},
        {0xAA,0},
        {0x5A,1}, //SYM_HOME, next 5 symbols
        {0x5A,1},
        {0x5A,1},
        {0x5A,1},
        {0x5A,1},
        {0x5A,1},
        {0xB1,0}, //continuation of decimal pack
        {0xB2,0},
        {0xB3,0},
        {0xB4,0},
        {0xB5,0},
        {0xB6,0},
        {0xB7,0},
        {0xB8,0},
        {0xB9,0},
        {0xBA,0},
        {0x2E,1}, //SYM_SIDEBAR_G
        {0x2E,1}, //SYM_SIDEBAR_E
        {0x31,1}, //SYM_SIDEBAR_F
        {0x31,1}, //SYM_SIDEBAR_D
        {0x31,1}, //SYM_SIDEBAR_H
        {0xBC,0}, //SYM_G
        //C0-DF above
        {0x8B,0}, //SYM_RPM
        {0x8C,0}, //SYM_ASPD
        {0x20,0}, //SYM_GSPD
        {0x86,0}, //SYM_WSPD
        {0x5E,0}, //SYM_VSPD
        {0x57,0}, //SYM_WPNO
        {0x20,0}, //SYM_WPDIR
        {0x20,0}, //SYM_WPDST
        {0x8E,0}, //SYM_FTMIN
        {0x65,1}, //SYM_DISARMED
        {0xAF,0}, //SYM_ROLLR
        {0xAD,0}, //SYM_ROLLL
        {0x15,0}, //SYM_PTCHUP
        {0x16,0}, //SYM_PTCHDWN
        {0xDC,0}, //SYM_XERR
        {0x59,1}, //SYM_TERALT
        {0x92,0}, //SYM_KN
        {0x85,0}, //SYM_NM
        {0x20,0}, //SYM_EFF
        {0xD3,0}, //SYM_AH
        {0x72,0}, //SYM_MW
        {0x91,1}, //SYM_FENCE_ENABLED
        {0x20,0}, //SYM_FENCE_DISABLED
        {0xBA,1}, //SYM_RNGFD
        {0x02,0}, //SYM_LQ
        {0x12,0}, //SYM_DB
        {0x13,0}, //SYM_DBM
        {0x14,0}, //SYM_SNR
        {0x59,0}, //SYM_ANT
        {0x40,1}, //SYM_ARROW_RIGHT
        {0x48,1}, //SYM_ARROW_LEFT
        {0xFF,0}
        //E0-FF above
    };
#endif //AP_MSP_INAV_FONTS_ENABLED

#endif
