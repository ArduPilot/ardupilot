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

void AP_RCProtocol::init()
{
    backend[AP_RCProtocol::PPM] = new AP_RCProtocol_PPMSum(*this);
    backend[AP_RCProtocol::SBUS] = new AP_RCProtocol_SBUS(*this);
    backend[AP_RCProtocol::DSM] = new AP_RCProtocol_DSM(*this);
}

void AP_RCProtocol::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint32_t now = AP_HAL::millis();
    // first try current protocol
    if (_detected_protocol != AP_RCProtocol::NONE && now - _last_input_ms < 200) {
        backend[_detected_protocol]->process_pulse(width_s0, width_s1);
        if (backend[_detected_protocol]->new_input()) {
            _new_input = true;
            _last_input_ms = AP_HAL::millis();
        }
        return;
    }

    // otherwise scan all protocols
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (backend[i] != nullptr) {
            backend[i]->process_pulse(width_s0, width_s1);
            if (backend[i]->new_input()) {
                _new_input = true;
                _detected_protocol = (enum AP_RCProtocol::rcprotocol_t)i;
                _last_input_ms = AP_HAL::millis();
            }
        }
    }
}

bool AP_RCProtocol::new_input()
{
    bool ret = _new_input;
    _new_input = false;
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
