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

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_RCProtocol.h"
#include "AP_RCProtocol_PPMSum.h"
#include "AP_RCProtocol_DSM.h"
#include "AP_RCProtocol_IBUS.h"
#include "AP_RCProtocol_SBUS.h"
#include "AP_RCProtocol_SUMD.h"
#include "AP_RCProtocol_SRXL.h"
#ifndef IOMCU_FW
#include "AP_RCProtocol_SRXL2.h"
#endif
#include "AP_RCProtocol_CRSF.h"
#include "AP_RCProtocol_ST24.h"
#include "AP_RCProtocol_FPort.h"
#include "AP_RCProtocol_FPort2.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

void AP_RCProtocol::init()
{
    backend[AP_RCProtocol::PPM] = new AP_RCProtocol_PPMSum(*this);
    backend[AP_RCProtocol::IBUS] = new AP_RCProtocol_IBUS(*this);
    backend[AP_RCProtocol::SBUS] = new AP_RCProtocol_SBUS(*this, true);
    backend[AP_RCProtocol::DSM] = new AP_RCProtocol_DSM(*this);
    backend[AP_RCProtocol::SUMD] = new AP_RCProtocol_SUMD(*this);
    backend[AP_RCProtocol::SRXL] = new AP_RCProtocol_SRXL(*this);
#ifndef IOMCU_FW
    backend[AP_RCProtocol::SBUS_NI] = new AP_RCProtocol_SBUS(*this, false);
    backend[AP_RCProtocol::SRXL2] = new AP_RCProtocol_SRXL2(*this);
    backend[AP_RCProtocol::CRSF] = new AP_RCProtocol_CRSF(*this);
    backend[AP_RCProtocol::FPORT2] = new AP_RCProtocol_FPort2(*this, true);
#endif
    backend[AP_RCProtocol::ST24] = new AP_RCProtocol_ST24(*this);
    backend[AP_RCProtocol::FPORT] = new AP_RCProtocol_FPort(*this, true);
}

AP_RCProtocol::~AP_RCProtocol()
{
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (backend[i] != nullptr) {
            delete backend[i];
            backend[i] = nullptr;
        }
    }
}

bool AP_RCProtocol::should_search(uint32_t now_ms) const
{
#ifndef IOMCU_FW
    if (_detected_protocol != AP_RCProtocol::NONE && !rc().allow_rc_protocol_switching()) {
        return false;
    }
#endif
    return (now_ms - _last_input_ms >= 200);
}

void AP_RCProtocol::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint32_t now = AP_HAL::millis();
    bool searching = should_search(now);

#ifndef IOMCU_FW
    rc_protocols_mask = rc().enabled_protocols();
#endif

    if (_detected_protocol != AP_RCProtocol::NONE &&
        !protocol_enabled(_detected_protocol)) {
        _detected_protocol = AP_RCProtocol::NONE;
    }
    
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
            if (!protocol_enabled(rcprotocol_t(i))) {
                continue;
            }
            const uint32_t frame_count = backend[i]->get_rc_frame_count();
            const uint32_t input_count = backend[i]->get_rc_input_count();
            backend[i]->process_pulse(width_s0, width_s1);
            const uint32_t frame_count2 = backend[i]->get_rc_frame_count();
            if (frame_count2 > frame_count) {
                if (requires_3_frames((rcprotocol_t)i) && frame_count2 < 3) {
                    continue;
                }
                _new_input = (input_count != backend[i]->get_rc_input_count());
                _detected_protocol = (enum AP_RCProtocol::rcprotocol_t)i;
                for (uint8_t j = 0; j < AP_RCProtocol::NONE; j++) {
                    if (backend[j]) {
                        backend[j]->reset_rc_frame_count();
                    }
                }
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

bool AP_RCProtocol::process_byte(uint8_t byte, uint32_t baudrate)
{
    uint32_t now = AP_HAL::millis();
    bool searching = should_search(now);

#ifndef IOMCU_FW
    rc_protocols_mask = rc().enabled_protocols();
#endif

    if (_detected_protocol != AP_RCProtocol::NONE &&
        !protocol_enabled(_detected_protocol)) {
        _detected_protocol = AP_RCProtocol::NONE;
    }

    if (_detected_protocol != AP_RCProtocol::NONE && !_detected_with_bytes && !searching) {
        // we're using pulse inputs, discard bytes
        return false;
    }

    // first try current protocol
    if (_detected_protocol != AP_RCProtocol::NONE && !searching) {
        backend[_detected_protocol]->process_byte(byte, baudrate);
        if (backend[_detected_protocol]->new_input()) {
            _new_input = true;
            _last_input_ms = now;
        }
        return true;
    }

    // otherwise scan all protocols
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (backend[i] != nullptr) {
            if (!protocol_enabled(rcprotocol_t(i))) {
                continue;
            }
            const uint32_t frame_count = backend[i]->get_rc_frame_count();
            const uint32_t input_count = backend[i]->get_rc_input_count();
            backend[i]->process_byte(byte, baudrate);
            const uint32_t frame_count2 = backend[i]->get_rc_frame_count();
            if (frame_count2 > frame_count) {
                if (requires_3_frames((rcprotocol_t)i) && frame_count2 < 3) {
                    continue;
                }
                _new_input = (input_count != backend[i]->get_rc_input_count());
                _detected_protocol = (enum AP_RCProtocol::rcprotocol_t)i;
                _last_input_ms = now;
                _detected_with_bytes = true;
                for (uint8_t j = 0; j < AP_RCProtocol::NONE; j++) {
                    if (backend[j]) {
                        backend[j]->reset_rc_frame_count();
                    }
                }
                // stop decoding pulses to save CPU
                hal.rcin->pulse_input_enable(false);
                break;
            }
        }
    }
    return false;
}

// handshake if nothing else has succeeded so far
void AP_RCProtocol::process_handshake( uint32_t baudrate)
{
    // if we ever succeeded before then do not handshake
    if (_detected_protocol != AP_RCProtocol::NONE || _last_input_ms > 0) {
        return;
    }

    // otherwise handshake all protocols
    for (uint8_t i = 0; i < AP_RCProtocol::NONE; i++) {
        if (backend[i] != nullptr) {
            backend[i]->process_handshake(baudrate);
        }
    }
}

/*
  check for bytes from an additional uart. This is used to support RC
  protocols from SERIALn_PROTOCOL
 */
void AP_RCProtocol::check_added_uart(void)
{
    if (!added.uart) {
        return;
    }
    uint32_t now = AP_HAL::millis();
    bool searching = should_search(now);
    if (!searching && !_detected_with_bytes) {
        // not using this uart
        return;
    }
    if (!added.opened) {
        added.opened = true;
        switch (added.phase) {
        case CONFIG_115200_8N1:
            added.baudrate = 115200;
            added.uart->configure_parity(0);
            added.uart->set_stop_bits(1);
            added.uart->set_options(added.uart->get_options() & ~AP_HAL::UARTDriver::OPTION_RXINV);
            break;
        case CONFIG_115200_8N1I:
            added.baudrate = 115200;
            added.uart->configure_parity(0);
            added.uart->set_stop_bits(1);
            added.uart->set_options(added.uart->get_options() | AP_HAL::UARTDriver::OPTION_RXINV);
            break;
        case CONFIG_100000_8E2I:
            // assume SBUS settings, even parity, 2 stop bits
            added.baudrate = 100000;
            added.uart->configure_parity(2);
            added.uart->set_stop_bits(2);
            added.uart->set_options(added.uart->get_options() | AP_HAL::UARTDriver::OPTION_RXINV);
            break;
        case CONFIG_420000_8N1:
            added.baudrate = CRSF_BAUDRATE;
            added.uart->configure_parity(0);
            added.uart->set_stop_bits(1);
            added.uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
            added.uart->set_blocking_writes(false);
            added.uart->set_options(added.uart->get_options() & ~AP_HAL::UARTDriver::OPTION_RXINV);
            break;
        }
        added.uart->begin(added.baudrate, 128, 128);
        added.last_baud_change_ms = AP_HAL::millis();
    }
#ifndef IOMCU_FW
    rc_protocols_mask = rc().enabled_protocols();
#endif
    process_handshake(added.baudrate);

    uint32_t n = added.uart->available();
    n = MIN(n, 255U);
    for (uint8_t i=0; i<n; i++) {
        int16_t b = added.uart->read();
        if (b >= 0) {
            process_byte(uint8_t(b), added.baudrate);
        }
    }
    if (!_detected_with_bytes) {
        if (now - added.last_baud_change_ms > 1000) {
            // flip baudrates if not detected once a second
            added.phase = (enum config_phase)(uint8_t(added.phase) + 1);
            if (added.phase > CONFIG_420000_8N1) {
                added.phase = (enum config_phase)0;
            }
            added.baudrate = (added.baudrate==100000)?115200:100000;
            added.opened = false;
        }
    }
}

void AP_RCProtocol::update()
{
    check_added_uart();
}

bool AP_RCProtocol::new_input()
{
    bool ret = _new_input;
    _new_input = false;

    // if we have an extra UART from a SERIALn_PROTOCOL then check it for data
    check_added_uart();

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

void AP_RCProtocol::read(uint16_t *pwm, uint8_t n)
{
    if (_detected_protocol != AP_RCProtocol::NONE) {
        backend[_detected_protocol]->read(pwm, n);
    }
}

int16_t AP_RCProtocol::get_RSSI(void) const
{
    if (_detected_protocol != AP_RCProtocol::NONE) {
        return backend[_detected_protocol]->get_RSSI();
    }
    return -1;
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
    case IBUS:
        return "IBUS";
    case SBUS:
    case SBUS_NI:
        return "SBUS";
    case DSM:
        return "DSM";
    case SUMD:
        return "SUMD";
    case SRXL:
        return "SRXL";
    case SRXL2:
        return "SRXL2";
    case CRSF:
        return "CRSF";
    case ST24:
        return "ST24";
    case FPORT:
        return "FPORT";
    case FPORT2:
        return "FPORT2";
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

/*
  add a uart to decode
 */
void AP_RCProtocol::add_uart(AP_HAL::UARTDriver* uart)
{
    added.uart = uart;
    // start with DSM
    added.baudrate = 115200U;
}

// return true if a specific protocol is enabled
bool AP_RCProtocol::protocol_enabled(rcprotocol_t protocol) const
{
    if ((rc_protocols_mask & 1) != 0) {
        // all protocols enabled
        return true;
    }
    return ((1U<<(uint8_t(protocol)+1)) & rc_protocols_mask) != 0;
}

namespace AP {
    AP_RCProtocol &RC()
    {
        static AP_RCProtocol rcprot;
        return rcprot;
    }
};
