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
        gcs().send_text(MAV_SEVERITY_WARNING,"MSP backend not available");
        return false;
    }
    _displayport = msp->find_protocol(AP_SerialManager::SerialProtocol_MSP_DisplayPort);
    if (_displayport == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING,"MSP DisplayPort uart not available");
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
    _displayport->msp_displayport_write_string(x, y, 0, text);
}

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
    if (msp && msp->is_option_enabled(AP_MSP::Option::DISPLAYPORT_BTFL_SYMBOLS)) {
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
    AP_OSD_MSP_DisplayPort *backend = new AP_OSD_MSP_DisplayPort(osd);
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


#endif
