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
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_HAL_ESP32.h"
#include "RCInput.h"

using namespace ESP32;

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
#endif
    _init = true;
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
#if AP_RCPROTOCOL_ENABLED
    if (!_init) {
        return;
    }
    AP_RCProtocol &rcprot = AP::RC();

#ifdef HAL_ESP32_RCIN
    uint32_t width_s0, width_s1;
    while (sig_reader.read(width_s0, width_s1)) {
        rcprot.process_pulse(width_s0, width_s1);

    }

#ifndef HAL_NO_UARTDRIVER
    const char *rc_protocol = nullptr;
#endif

    if (rcprot.new_input()) {
        WITH_SEMAPHORE(rcin_mutex);
        _rcin_timestamp_last_signal = AP_HAL::micros();
        _num_channels = rcprot.num_channels();
        _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
        rcprot.read(_rc_values, _num_channels);
        _rssi = rcprot.get_RSSI();
#ifndef HAL_NO_UARTDRIVER
        rc_protocol = rcprot.protocol_name();
#endif
    }

#ifndef HAL_NO_UARTDRIVER
    if (rc_protocol && rc_protocol != last_protocol) {
        last_protocol = rc_protocol;
        gcs().send_text(MAV_SEVERITY_DEBUG, "RCInput: decoding %s", last_protocol);
    }
#endif

#endif
#endif // AP_RCPROTOCOL_ENABLED
}
