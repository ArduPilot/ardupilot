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

#include "AP_RCProtocol_config.h"

#include "AP_RCProtocol.h"

#if AP_RCPROTOCOL_ENABLED

#include "AP_RCProtocol_PPMSum.h"
#include "AP_RCProtocol_DSM.h"
#include "AP_RCProtocol_IBUS.h"
#include "AP_RCProtocol_SBUS.h"
#include "AP_RCProtocol_SUMD.h"
#include "AP_RCProtocol_SRXL.h"
#include "AP_RCProtocol_SRXL2.h"
#include "AP_RCProtocol_CRSF.h"
#include "AP_RCProtocol_ST24.h"
#include "AP_RCProtocol_FPort.h"
#include "AP_RCProtocol_FPort2.h"
#include "AP_RCProtocol_DroneCAN.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

void AP_RCProtocol::init()
{
#if AP_RCPROTOCOL_PPMSUM_ENABLED
    backend[AP_RCProtocol::PPMSUM] = new AP_RCProtocol_PPMSum(*this);
#endif
#if AP_RCPROTOCOL_IBUS_ENABLED
    backend[AP_RCProtocol::IBUS] = new AP_RCProtocol_IBUS(*this);
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED
    backend[AP_RCProtocol::SBUS] = new AP_RCProtocol_SBUS(*this, true, 100000);
#endif
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
    backend[AP_RCProtocol::FASTSBUS] = new AP_RCProtocol_SBUS(*this, true, 200000);
#endif
    backend[AP_RCProtocol::DSM] = new AP_RCProtocol_DSM(*this);
#if AP_RCPROTOCOL_SUMD_ENABLED
    backend[AP_RCProtocol::SUMD] = new AP_RCProtocol_SUMD(*this);
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED
    backend[AP_RCProtocol::SRXL] = new AP_RCProtocol_SRXL(*this);
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
    backend[AP_RCProtocol::SBUS_NI] = new AP_RCProtocol_SBUS(*this, false, 100000);
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED
    backend[AP_RCProtocol::SRXL2] = new AP_RCProtocol_SRXL2(*this);
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
    backend[AP_RCProtocol::CRSF] = new AP_RCProtocol_CRSF(*this);
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED
    backend[AP_RCProtocol::FPORT2] = new AP_RCProtocol_FPort2(*this, true);
#endif
#if AP_RCPROTOCOL_ST24_ENABLED
    backend[AP_RCProtocol::ST24] = new AP_RCProtocol_ST24(*this);
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
    backend[AP_RCProtocol::FPORT] = new AP_RCProtocol_FPort(*this, true);
#endif
#if AP_RCPROTOCOL_DRONECAN_ENABLED
    backend[AP_RCProtocol::DRONECAN] = new AP_RCProtocol_DroneCAN(*this);
#endif
}

AP_RCProtocol::~AP_RCProtocol()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(backend); i++) {
        if (backend[i] != nullptr) {
            delete backend[i];
            backend[i] = nullptr;
        }
    }
}

bool AP_RCProtocol::should_search(uint32_t now_ms) const
{
#if AP_RC_CHANNEL_ENABLED && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    if (_detected_protocol != AP_RCProtocol::NONE && !rc().option_is_enabled(RC_Channels::Option::MULTI_RECEIVER_SUPPORT)) {
        return false;
    }
#else
    // on IOMCU don't allow protocol to change once detected
    if (_detected_protocol != AP_RCProtocol::NONE) {
        return false;
    }
#endif
    return (now_ms - _last_input_ms >= 200);
}

void AP_RCProtocol::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint32_t now = AP_HAL::millis();
    bool searching = should_search(now);

#if AP_RC_CHANNEL_ENABLED
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
    for (uint8_t i = 0; i < ARRAY_SIZE(backend); i++) {
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
                for (uint8_t j = 0; j < ARRAY_SIZE(backend); j++) {
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

#if AP_RC_CHANNEL_ENABLED
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
    for (uint8_t i = 0; i < ARRAY_SIZE(backend); i++) {
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
                for (uint8_t j = 0; j < ARRAY_SIZE(backend); j++) {
                    if (backend[j]) {
                        backend[j]->reset_rc_frame_count();
                    }
                }
                // stop decoding pulses to save CPU
                hal.rcin->pulse_input_enable(false);
                return true;
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
    for (uint8_t i = 0; i < ARRAY_SIZE(backend); i++) {
        if (backend[i] != nullptr) {
            backend[i]->process_handshake(baudrate);
        }
    }
}

/*
  check for bytes from an additional uart. This is used to support RC
  protocols from SERIALn_PROTOCOL
 */
void AP_RCProtocol::SerialConfig::apply_to_uart(AP_HAL::UARTDriver *uart) const
{
    uart->configure_parity(parity);
    uart->set_stop_bits(stop_bits);
    if (invert_rx) {
        uart->set_options(uart->get_options() | AP_HAL::UARTDriver::OPTION_RXINV);
    } else {
        uart->set_options(uart->get_options() & ~AP_HAL::UARTDriver::OPTION_RXINV);
    }
    uart->begin(baud, 128, 128);
}

static const AP_RCProtocol::SerialConfig serial_configs[] {
    // BAUD PRTY STOP INVERT-RX
    // inverted and uninverted 115200 8N1:
    { 115200,  0,   1, false  },
    { 115200,  0,   1, true },
    // SBUS settings, even parity, 2 stop bits:
    { 100000,  2,   2, true },
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
    // FastSBUS:
    { 200000,  2,   2, true },
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
    // CrossFire:
    { 416666,  0,   1, false },
    // CRSFv3 can negotiate higher rates which are sticky on soft reboot
    { 2000000, 0,   1, false },
#endif
};

static_assert(ARRAY_SIZE(serial_configs) > 1, "must have at least one serial config");

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
        added.last_config_change_ms = AP_HAL::millis();
        serial_configs[added.config_num].apply_to_uart(added.uart);
    }
#if AP_RC_CHANNEL_ENABLED
    rc_protocols_mask = rc().enabled_protocols();
#endif
    const uint32_t current_baud = serial_configs[added.config_num].baud;
    process_handshake(current_baud);

    uint32_t n = added.uart->available();
    n = MIN(n, 255U);
    for (uint8_t i=0; i<n; i++) {
        int16_t b = added.uart->read();
        if (b >= 0) {
            process_byte(uint8_t(b), current_baud);
        }
    }
    if (searching) {
        if (now - added.last_config_change_ms > 1000) {
            // change configs if not detected once a second
            added.config_num++;
            if (added.config_num >= ARRAY_SIZE(serial_configs)) {
                added.config_num = 0;
            }
            added.opened = false;
        }
    // power loss on CRSF requires re-bootstrap because the baudrate is reset to the default. The CRSF side will
    // drop back down to 416k if it has received 200 incorrect characters (or none at all)
    } else if (_detected_protocol != AP_RCProtocol::NONE
        // protocols that want to be able to renegotiate should return false in is_rx_active()
        && !backend[_detected_protocol]->is_rx_active()
        && now - added.last_config_change_ms > 1000) {
        added.opened = false;
    }
}

void AP_RCProtocol::update()
{
    check_added_uart();
}

bool AP_RCProtocol::new_input()
{
    // if we have an extra UART from a SERIALn_PROTOCOL then check it for data
    check_added_uart();

    // run update function on backends
    for (uint8_t i = 0; i < ARRAY_SIZE(backend); i++) {
        if (backend[i] != nullptr) {
            backend[i]->update();
        }
    }

#if AP_RCPROTOCOL_DRONECAN_ENABLED
    uint32_t now = AP_HAL::millis();
    if (should_search(now)) {
        if (backend[AP_RCProtocol::DRONECAN] != nullptr &&
            backend[AP_RCProtocol::DRONECAN]->new_input()) {
            _detected_protocol = AP_RCProtocol::DRONECAN;
            _last_input_ms = now;
        }
    } else if (_detected_protocol == AP_RCProtocol::DRONECAN) {
        _new_input = backend[AP_RCProtocol::DRONECAN]->new_input();
        if (_new_input) {
            _last_input_ms = now;
        }
    }
#endif

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
int16_t AP_RCProtocol::get_rx_link_quality(void) const
{
    if (_detected_protocol != AP_RCProtocol::NONE) {
        return backend[_detected_protocol]->get_rx_link_quality();
    }
    return -1;
}
/*
  ask for bind start on supported receivers (eg spektrum satellite)
 */
void AP_RCProtocol::start_bind(void)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(backend); i++) {
        if (backend[i] != nullptr) {
            backend[i]->start_bind();
        }
    }
}

#endif  // AP_RCPROTOCOL_ENABLED

/*
  return protocol name
 */
const char *AP_RCProtocol::protocol_name_from_protocol(rcprotocol_t protocol)
{
    switch (protocol) {
#if AP_RCPROTOCOL_PPMSUM_ENABLED
    case PPMSUM:
        return "PPM";
#endif
#if AP_RCPROTOCOL_IBUS_ENABLED
    case IBUS:
        return "IBUS";
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED
    case SBUS:
        return "SBUS";
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
    case SBUS_NI:
        return "SBUS";
#endif
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
    case FASTSBUS:
        return "FastSBUS";
#endif
    case DSM:
        return "DSM";
#if AP_RCPROTOCOL_SUMD_ENABLED
    case SUMD:
        return "SUMD";
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED
    case SRXL:
        return "SRXL";
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED
    case SRXL2:
        return "SRXL2";
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
    case CRSF:
        return "CRSF";
#endif
#if AP_RCPROTOCOL_ST24_ENABLED
    case ST24:
        return "ST24";
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
    case FPORT:
        return "FPORT";
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED
    case FPORT2:
        return "FPORT2";
#endif
#if AP_RCPROTOCOL_DRONECAN_ENABLED
    case DRONECAN:
        return "DroneCAN";
#endif
    case NONE:
        break;
    }
    return nullptr;
}

#if AP_RCPROTOCOL_ENABLED
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
    added.uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
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

#endif  // AP_RCPROTOCOL_ENABLED
