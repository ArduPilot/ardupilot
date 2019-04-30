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
#ifdef HAL_ESP32_RCIN
    sig_reader.init();
    rcin_prot.init();
#endif
    _init = true;
}

bool RCInput::new_input()
{
    if (!_init) {
        return false;
    }
    if (!rcin_mutex.take_nonblocking()) {
        return false;
    }
    bool valid = _rcin_timestamp_last_signal != _last_read;

    _last_read = _rcin_timestamp_last_signal;
    rcin_mutex.give();
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
    rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
    uint16_t v = _rc_values[channel];
    rcin_mutex.give();
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
    for (uint8_t i = 0; i < len; i++) {
        periods[i] = read(i);
    }
    return len;
}

void RCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }
#ifdef HAL_ESP32_RCIN
    uint32_t width_s0, width_s1;
    while (sig_reader.read(width_s0, width_s1)) {
        rcin_prot.process_pulse(width_s0, width_s1);
    }

    const char *rc_protocol = nullptr;
    if (rcin_prot.new_input()) {
        rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
        _rcin_timestamp_last_signal = AP_HAL::micros();
        _num_channels = rcin_prot.num_channels();
        _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
        for (uint8_t i=0; i<_num_channels; i++) {
            _rc_values[i] = rcin_prot.read(i);
        }
        rcin_mutex.give();
        rc_protocol = rcin_prot.protocol_name();
    }

    if (rc_protocol && rc_protocol != last_protocol) {
        last_protocol = rc_protocol;
        gcs().send_text(MAV_SEVERITY_DEBUG, "RCInput: decoding %s", last_protocol);
    }
#endif
}
