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
 */

#include <GCS_MAVLink/GCS.h>

#include "AP_HAL_ESP32.h"
#include "RCInput.h"

#include <AP_RCProtocol/AP_RCProtocol_config.h>

#include <AP_Math/AP_Math.h>

using namespace ESP32;

extern const AP_HAL::HAL& hal;

void RCInput::init()
{
    if (_init) {
        return;
    }
#if AP_RCPROTOCOL_ENABLED
    AP::RC().init();
#endif

#ifdef HAL_ESP32_RCIN
    sig_reader.init();
    pulse_input_enabled = true;
#endif

    _init = true;
}

/*
  enable or disable pulse input for RC input. This is used to reduce
  load when we are decoding R/C via a UART
*/
void RCInput::pulse_input_enable(bool enable)
{
    pulse_input_enabled = enable;
#if HAL_ESP32_RCIN
    if (!enable) {
        sig_reader.disable();
    }
#endif
}

bool RCInput::new_input()
{
    if (!_init) {
        return false;
    }
    bool valid;
    {
        WITH_SEMAPHORE(rcin_mutex);
        valid = _rcin_timestamp_last_signal != _last_read;
        _last_read = _rcin_timestamp_last_signal;
    }

    return valid;
}

uint8_t RCInput::num_channels()
{
    if (!_init) {
        return 0;
    }
    return _num_channels;
}

uint16_t RCInput::read(uint8_t channel)
{
    if (!_init || (channel >= MIN(RC_INPUT_MAX_CHANNELS, _num_channels))) {
        return 0;
    }
    uint16_t v;
    {
        WITH_SEMAPHORE(rcin_mutex);
        v = _rc_values[channel];
    }
    return v;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (!_init) {
        return false;
    }

    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    {
        WITH_SEMAPHORE(rcin_mutex);
        memcpy(periods, _rc_values, len*sizeof(periods[0]));
    }
    return len;
}

void RCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }

#if AP_RCPROTOCOL_ENABLED
    AP_RCProtocol &rcprot = AP::RC();

#ifdef HAL_ESP32_RCIN
    if (pulse_input_enabled) {
        uint32_t width_s0, width_s1;
        while (sig_reader.read(width_s0, width_s1)) {
            rcprot.process_pulse(width_s0, width_s1);
        }
    }
#endif

    if (rcprot.new_input()) {
        WITH_SEMAPHORE(rcin_mutex);
        _rcin_timestamp_last_signal = AP_HAL::micros();
        _num_channels = rcprot.num_channels();
        _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
        rcprot.read(_rc_values, _num_channels);
        _rssi = rcprot.get_RSSI();
        _rx_link_quality = rcprot.get_rx_link_quality();
    }
#endif // AP_RCPROTOCOL_ENABLED
}
