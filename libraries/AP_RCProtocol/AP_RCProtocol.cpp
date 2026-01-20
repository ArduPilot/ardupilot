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
#include "AP_RCProtocol_Emlid_RCIO.h"
#include "AP_RCProtocol_IBUS.h"
#include "AP_RCProtocol_IOMCU.h"
#include "AP_RCProtocol_SBUS.h"
#include "AP_RCProtocol_SUMD.h"
#include "AP_RCProtocol_SRXL.h"
#include "AP_RCProtocol_SRXL2.h"
#include "AP_RCProtocol_CRSF.h"
#include "AP_RCProtocol_ST24.h"
#include "AP_RCProtocol_FPort.h"
#include "AP_RCProtocol_FPort2.h"
#include "AP_RCProtocol_DroneCAN.h"
#include "AP_RCProtocol_GHST.h"
#include "AP_RCProtocol_MAVLinkRadio.h"
#include "AP_RCProtocol_Joystick_SFML.h"
#include "AP_RCProtocol_UDP.h"
#include "AP_RCProtocol_FDM.h"
#include "AP_RCProtocol_Radio.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void AP_RCProtocol::init()
{
#if AP_RCPROTOCOL_PPMSUM_ENABLED
    backend[AP_RCProtocol::PPMSUM] = NEW_NOTHROW AP_RCProtocol_PPMSum(*this);
#endif
#if AP_RCPROTOCOL_IBUS_ENABLED
    backend[AP_RCProtocol::IBUS] = NEW_NOTHROW AP_RCProtocol_IBUS(*this);
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED
    backend[AP_RCProtocol::SBUS] = NEW_NOTHROW AP_RCProtocol_SBUS(*this, true, 100000);
#endif
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
    backend[AP_RCProtocol::FASTSBUS] = NEW_NOTHROW AP_RCProtocol_SBUS(*this, true, 200000);
#endif
#if AP_RCPROTOCOL_DSM_ENABLED
    backend[AP_RCProtocol::DSM] = NEW_NOTHROW AP_RCProtocol_DSM(*this);
#endif
#if AP_RCPROTOCOL_SUMD_ENABLED
    backend[AP_RCProtocol::SUMD] = NEW_NOTHROW AP_RCProtocol_SUMD(*this);
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED
    backend[AP_RCProtocol::SRXL] = NEW_NOTHROW AP_RCProtocol_SRXL(*this);
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
    backend[AP_RCProtocol::SBUS_NI] = NEW_NOTHROW AP_RCProtocol_SBUS(*this, false, 100000);
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED
    backend[AP_RCProtocol::SRXL2] = NEW_NOTHROW AP_RCProtocol_SRXL2(*this);
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
    backend[AP_RCProtocol::CRSF] = NEW_NOTHROW AP_RCProtocol_CRSF(*this);
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED
    backend[AP_RCProtocol::FPORT2] = NEW_NOTHROW AP_RCProtocol_FPort2(*this, true);
#endif
#if AP_RCPROTOCOL_ST24_ENABLED
    backend[AP_RCProtocol::ST24] = NEW_NOTHROW AP_RCProtocol_ST24(*this);
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
    backend[AP_RCProtocol::FPORT] = NEW_NOTHROW AP_RCProtocol_FPort(*this, true);
#endif
#if AP_RCPROTOCOL_DRONECAN_ENABLED
    backend[AP_RCProtocol::DRONECAN] = NEW_NOTHROW AP_RCProtocol_DroneCAN(*this);
#endif
#if AP_RCPROTOCOL_GHST_ENABLED
    backend[AP_RCProtocol::GHST] = NEW_NOTHROW AP_RCProtocol_GHST(*this);
#endif
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
    backend[AP_RCProtocol::MAVLINK_RADIO] = NEW_NOTHROW AP_RCProtocol_MAVLinkRadio(*this);
#endif
#if AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
    backend[AP_RCProtocol::JOYSTICK_SFML] = NEW_NOTHROW AP_RCProtocol_Joystick_SFML(*this);
#endif
#if AP_RCPROTOCOL_UDP_ENABLED
    const auto UDP_backend = NEW_NOTHROW AP_RCProtocol_UDP(*this);
    backend[AP_RCProtocol::UDP] = UDP_backend;
#endif
#if AP_RCPROTOCOL_FDM_ENABLED
    const auto FDM_backend = NEW_NOTHROW AP_RCProtocol_FDM(*this);;
    backend[AP_RCProtocol::FDM] = FDM_backend;
#if AP_RCPROTOCOL_UDP_ENABLED
    // the UDP-Packed16Bit backend gives way to the FDM backend:
    UDP_backend->set_fdm_backend(FDM_backend);
#endif  // AP_RCPROTOCOL_UDP_ENABLED
#endif  // AP_RCPROTOCOL_FDM_ENABLED
#if AP_RCPROTOCOL_RADIO_ENABLED
    backend[AP_RCProtocol::RADIO] = NEW_NOTHROW AP_RCProtocol_Radio(*this);
#endif
#if AP_RCPROTOCOL_IOMCU_ENABLED
    backend[AP_RCProtocol::IOMCU] = NEW_NOTHROW AP_RCProtocol_IOMCU(*this);
#endif  // AP_RCPROTOCOL_IOMCU_ENABLED
#if AP_RCPROTOCOL_EMLID_RCIO_ENABLED
    backend[AP_RCProtocol::EMLID_RCIO] = NEW_NOTHROW AP_RCProtocol_Emlid_RCIO(*this);
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
#if AP_RCPROTOCOL_FDM_ENABLED && AP_RCPROTOCOL_UDP_ENABLED
    // force re-detection when FDM is active and active backend is UDP values
    if (_detected_protocol == AP_RCProtocol::UDP &&
        ((AP_RCProtocol_FDM*)backend[AP_RCProtocol::FDM])->active()) {
        return true;
    }
#endif  // AP_RCPROTOCOL_FDM_ENABLED && AP_RCPROTOCOL_UDP_ENABLED
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
#if AP_RCPROTOCOL_CRSF_ENABLED || AP_RCPROTOCOL_GHST_ENABLED
    // CrossFire:
    { 416666,  0,   1, false },
    // CRSFv3 can negotiate higher rates which are sticky on soft reboot
    { 2000000, 0,   1, false },
#endif
};

static_assert(ARRAY_SIZE(serial_configs) > 0, "must have at least one serial config");

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

// explicitly investigate a backend for data, as opposed to feeding
// the backend a byte (or pulse-train) at a time and having them make
// an "add_input" callback):
bool AP_RCProtocol::detect_async_protocol(rcprotocol_t protocol)
{
    auto *p = backend[protocol];
    if (p == nullptr) {
        // backend is not allocated?!
        return false;
    }

    if (_detected_protocol == protocol) {
        // we are using this protocol already, see if there is new
        // data.  Caller will handle the case where we stop presenting
        // data
        return p->new_input();
    }

    // we are not the currently in-use protocol.
    const uint32_t now = AP_HAL::millis();

    // see if another backend is providing data:
    if (!should_search(now)) {
        // apparently, yes
        return false;
    }

#if AP_RC_CHANNEL_ENABLED
    rc_protocols_mask = rc().enabled_protocols();
#endif

    if (!protocol_enabled(protocol)) {
        return false;
    }

    // nobody is providing data; can we provide data?
    if (!p->new_input()) {
        // we can't provide data
        return false;
    }

    // we can provide data, change the detected protocol to be us:
    _detected_protocol = protocol;
    return true;
}

const char *AP_RCProtocol::detected_protocol_name() const
{
    switch (_detected_protocol) {
#if AP_RCPROTOCOL_IOMCU_ENABLED
    case AP_RCProtocol::IOMCU:
        return ((AP_RCProtocol_IOMCU*)(backend[AP_RCProtocol::IOMCU]))->get_rc_protocol();
#endif
    default:
        return protocol_name_from_protocol(_detected_protocol);
    }
}

void AP_RCProtocol::announce_detected()
{
    const char *src;
    const char *name = detected_protocol_name();
    if (name == nullptr) {
        return;
    }
    switch (_detected_protocol) {
#if AP_RCPROTOCOL_IOMCU_ENABLED
    case AP_RCProtocol::IOMCU:
        src = "IOMCU";
        break;
#endif
    default:
        src = using_uart() ? "Bytes" : "Pulses";
    }
    (void)src;  // iofirmware doesn't use this
    (void)name;  // iofirmware doesn't use this
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "RCInput: decoding %s (%s)", name, src);
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

    // iterate through backends which don't do either of pulse or uart
    // input, and thus won't update_new_input
    const rcprotocol_t pollable[] {
#if AP_RCPROTOCOL_DRONECAN_ENABLED
        AP_RCProtocol::DRONECAN,
#endif
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
        AP_RCProtocol::MAVLINK_RADIO,
#endif
#if AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
        AP_RCProtocol::JOYSTICK_SFML,
#endif
#if AP_RCPROTOCOL_UDP_ENABLED
        AP_RCProtocol::UDP,
#endif
#if AP_RCPROTOCOL_FDM_ENABLED
        AP_RCProtocol::FDM,
#endif
#if AP_RCPROTOCOL_RADIO_ENABLED
        AP_RCProtocol::RADIO,
#endif
#if AP_RCPROTOCOL_IOMCU_ENABLED
        AP_RCProtocol::IOMCU,
#endif  // AP_RCPROTOCOL_IOMCU_ENABLED
#if AP_RCPROTOCOL_EMLID_RCIO_ENABLED
        AP_RCProtocol::EMLID_RCIO,
#endif
    };
    for (const auto protocol : pollable) {
        if (!detect_async_protocol(protocol)) {
            continue;
        }
        _new_input = true;
        _last_input_ms = AP_HAL::millis();
        break;
    }

#if AP_RCPROTOCOL_IOMCU_ENABLED
    // IOMCU takes precedence over serial-port-type-23 input etc:
    if (((AP_RCProtocol_IOMCU*)(backend[AP_RCProtocol::IOMCU]))->active()) {
        _detected_protocol = AP_RCProtocol::IOMCU;
    }
#endif

    // announce protocol changes:
    if (_detected_protocol != _last_detected_protocol ||
        using_uart() != _last_detected_using_uart) {
        _last_detected_protocol = _detected_protocol;
        _last_detected_using_uart = using_uart();
        announce_detected();
    }

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
void AP_RCProtocol::start_bind()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(backend); i++) {
        if (backend[i] != nullptr && protocol_enabled((rcprotocol_t)i)) {
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
#if AP_RCPROTOCOL_DSM_ENABLED
    case DSM:
        return "DSM";
#endif
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
#if AP_RCPROTOCOL_GHST_ENABLED
    case GHST:
        return "GHST";
#endif
#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
    case MAVLINK_RADIO:
        return "MAVRadio";
#endif
#if AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
    case JOYSTICK_SFML:
        return "SFML";
#endif
#if AP_RCPROTOCOL_UDP_ENABLED
    case UDP:
        return "UDP";
#endif
#if AP_RCPROTOCOL_FDM_ENABLED
    case FDM:
        return "FDM";
#endif
#if AP_RCPROTOCOL_RADIO_ENABLED
    case RADIO:
        return "Radio";
#endif
#if AP_RCPROTOCOL_IOMCU_ENABLED
    case IOMCU:
        return "IOMCU";
#endif  // AP_RCPROTOCOL_IOMCU_ENABLED
#if AP_RCPROTOCOL_EMLID_RCIO_ENABLED
    case EMLID_RCIO:
        return "Emlid RCIO";
#endif
    case NONE:
        break;
    }
    return nullptr;
}

#if AP_RCPROTOCOL_ENABLED
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
    if ((rc_protocols_mask & 1U) != 0) {
        // all protocols enabled
        return true;
    }
    return ((1U<<(uint8_t(protocol)+1)) & rc_protocols_mask) != 0;
}

#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
void AP_RCProtocol::handle_radio_rc_channels(const mavlink_radio_rc_channels_t* packet)
{
    if (backend[AP_RCProtocol::MAVLINK_RADIO] == nullptr) {
        return;
    }

    backend[AP_RCProtocol::MAVLINK_RADIO]->update_radio_rc_channels(packet);
};
#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

namespace AP {
    AP_RCProtocol &RC()
    {
        static AP_RCProtocol rcprot;
        return rcprot;
    }
};

#endif  // AP_RCPROTOCOL_ENABLED
