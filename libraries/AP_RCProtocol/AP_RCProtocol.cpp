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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "AP_RCProtocol.h"
#include "AP_RCProtocol_PPMSum.h"
#include "AP_RCProtocol_DSM.h"
#include "AP_RCProtocol_SBUS.h"
#include "AP_RCProtocol_SUMD.h"
#include "AP_RCProtocol_SRXL.h"
#include "AP_RCProtocol_ST24.h"

// singleton
AP_RCProtocol *AP_RCProtocol::instance;

void AP_RCProtocol::init()
{
    backend[AP_RCProtocol::PPM] = new AP_RCProtocol_PPMSum(*this);
    backend[AP_RCProtocol::SBUS] = new AP_RCProtocol_SBUS(*this, true);
    backend[AP_RCProtocol::SBUS_NI] = new AP_RCProtocol_SBUS(*this, false);
    backend[AP_RCProtocol::DSM] = new AP_RCProtocol_DSM(*this);
    backend[AP_RCProtocol::SUMD] = new AP_RCProtocol_SUMD(*this);
    backend[AP_RCProtocol::SRXL] = new AP_RCProtocol_SRXL(*this);
    backend[AP_RCProtocol::ST24] = new AP_RCProtocol_ST24(*this);
}

AP_RCProtocol::~AP_RCProtocol()
{
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (backend[i] != nullptr) {
            delete backend[i];
            backend[i] = nullptr;
        }
    }
    instance = nullptr;
}

void AP_RCProtocol::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint32_t now = AP_HAL::millis();
    bool searching = (now - _last_input_ms >= 200);
    if (_detected_protocol != AP_RCProtocol::NONE && _detected_with_bytes && !searching) {
        // we're using byte inputs, discard pulses
        return;
    }
    // first try current protocol
    if (_detected_protocol != AP_RCProtocol::NONE && !searching) {
        backend[_detected_protocol]->process_pulse(width_s0, width_s1);
        if (backend[_detected_protocol]->new_input()) {
            _new_input = true;
            _last_input_ms = now;
        }
        return;
    }

    // otherwise scan all protocols
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (_disabled_for_pulses & (1U << i)) {
            // this protocol is disabled for pulse input
            continue;
        }
        if (backend[i] != nullptr) {
            uint32_t frame_count = backend[i]->get_rc_frame_count();
            uint32_t input_count = backend[i]->get_rc_input_count();
            backend[i]->process_pulse(width_s0, width_s1);
            if (frame_count != backend[i]->get_rc_frame_count()) {
                _good_frames[i]++;
                if (requires_3_frames((rcprotocol_t)i) && _good_frames[i] < 3) {
                    continue;
                }
                _new_input = (input_count != backend[i]->get_rc_input_count());
                _detected_protocol = (enum AP_RCProtocol::rcprotocol_t)i;
                memset(_good_frames, 0, sizeof(_good_frames));
                _last_input_ms = now;
                _detected_with_bytes = false;
                break;
            }
        }
    }
}

/*
  process an array of pulses. n must be even
 */
void AP_RCProtocol::process_pulse_list(const uint32_t *widths, uint16_t n, bool need_swap)
{
    if (n & 1) {
        return;
    }
    while (n) {
        uint32_t widths0 = widths[0];
        uint32_t widths1 = widths[1];
        if (need_swap) {
            uint32_t tmp = widths1;
            widths1 = widths0;
            widths0 = tmp;
        }
        widths1 -= widths0;
        process_pulse(widths0, widths1);
        widths += 2;
        n -= 2;
    }
}

void AP_RCProtocol::process_byte(uint8_t byte, uint32_t baudrate)
{
    uint32_t now = AP_HAL::millis();
    bool searching = (now - _last_input_ms >= 200);
    if (_detected_protocol != AP_RCProtocol::NONE && !_detected_with_bytes && !searching) {
        // we're using pulse inputs, discard bytes
        return;
    }
    // first try current protocol
    if (_detected_protocol != AP_RCProtocol::NONE && !searching) {
        backend[_detected_protocol]->process_byte(byte, baudrate);
        if (backend[_detected_protocol]->new_input()) {
            _new_input = true;
            _last_input_ms = now;
        }
        return;
    }

    // otherwise scan all protocols
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (backend[i] != nullptr) {
            uint32_t frame_count = backend[i]->get_rc_frame_count();
            uint32_t input_count = backend[i]->get_rc_input_count();
            backend[i]->process_byte(byte, baudrate);
            if (frame_count != backend[i]->get_rc_frame_count()) {
                _good_frames[i]++;
                if (requires_3_frames((rcprotocol_t)i) && _good_frames[i] < 3) {
                    continue;
                }
                _new_input = (input_count != backend[i]->get_rc_input_count());
                _detected_protocol = (enum AP_RCProtocol::rcprotocol_t)i;
                memset(_good_frames, 0, sizeof(_good_frames));
                _last_input_ms = now;
                _detected_with_bytes = true;
                break;
            }
        }
    }
}

bool AP_RCProtocol::new_input()
{
    bool ret = _new_input;
    _new_input = false;

    // run update function on backends
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (backend[i] != nullptr) {
            backend[i]->update();
        }
    }
    return ret;
}

uint8_t AP_RCProtocol::num_channels()
{
    if (_detected_protocol != AP_RCProtocol::NONE) {
        return backend[_detected_protocol]->num_channels();
    }
    return 0;
}

uint16_t AP_RCProtocol::read(uint8_t chan)
{
    if (_detected_protocol != AP_RCProtocol::NONE) {
        return backend[_detected_protocol]->read(chan);
    }
    return 0;
}

/*
  ask for bind start on supported receivers (eg spektrum satellite)
 */
void AP_RCProtocol::start_bind(void)
{
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (backend[i] != nullptr) {
            backend[i]->start_bind();
        }
    }
}

/*
  return protocol name
 */
const char *AP_RCProtocol::protocol_name_from_protocol(rcprotocol_t protocol)
{
    switch (protocol) {
    case PPM:
        return "PPM";
    case SBUS:
    case SBUS_NI:
        return "SBUS";
    case DSM:
        return "DSM";
    case SUMD:
        return "SUMD";
    case SRXL:
        return "SRXL";
    case ST24:
        return "ST24";
    case NONE:
        break;
    }
    return nullptr;
}

/*
  return protocol name
 */
const char *AP_RCProtocol::protocol_name(void) const
{
    return protocol_name_from_protocol(_detected_protocol);
}
