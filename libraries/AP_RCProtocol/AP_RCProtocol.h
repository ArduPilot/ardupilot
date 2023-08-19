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
#pragma once

#include "AP_RCProtocol_config.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#define MAX_RCIN_CHANNELS 18
#define MIN_RCIN_CHANNELS  5

class AP_RCProtocol_Backend;

class AP_RCProtocol {
public:

    enum rcprotocol_t {
#if AP_RCPROTOCOL_PPMSUM_ENABLED
        PPMSUM     =  0,
#endif
#if AP_RCPROTOCOL_IBUS_ENABLED
        IBUS       =  1,
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED
        SBUS       =  2,
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
        SBUS_NI    =  3,
#endif
        DSM        =  4,
#if AP_RCPROTOCOL_SUMD_ENABLED
        SUMD       =  5,
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED
        SRXL       =  6,
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED
        SRXL2      =  7,
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
        CRSF       =  8,
#endif
#if AP_RCPROTOCOL_ST24_ENABLED
        ST24       =  9,
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
        FPORT      = 10,
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED
        FPORT2     = 11,
#endif
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
        FASTSBUS   = 12,
#endif
#if AP_RCPROTOCOL_DRONECAN_ENABLED
        DRONECAN   = 13,
#endif
        NONE    //last enum always is None
    };

    // return protocol name as a string
    static const char *protocol_name_from_protocol(rcprotocol_t protocol);

#if AP_RCPROTOCOL_ENABLED

    AP_RCProtocol() {}
    ~AP_RCProtocol();
    friend class AP_RCProtocol_Backend;

    void init();
    bool valid_serial_prot() const
    {
        return _valid_serial_prot;
    }
    bool should_search(uint32_t now_ms) const;
    void process_pulse(uint32_t width_s0, uint32_t width_s1);
    void process_pulse_list(const uint32_t *widths, uint16_t n, bool need_swap);
    bool process_byte(uint8_t byte, uint32_t baudrate);
    void process_handshake(uint32_t baudrate);
    void update(void);

    bool failsafe_active() const {
        return _failsafe_active;
    }
    void set_failsafe_active(bool active) {
        _failsafe_active = active;
    }

    void disable_for_pulses(enum rcprotocol_t protocol) {
        _disabled_for_pulses |= (1U<<(uint8_t)protocol);
    }

    // for protocols without strong CRCs we require 3 good frames to lock on
    bool requires_3_frames(enum rcprotocol_t p) {
        switch (p) {
        case DSM:
#if AP_RCPROTOCOL_FASTSBUS_ENABLED
        case FASTSBUS:
#endif
#if AP_RCPROTOCOL_SBUS_ENABLED
        case SBUS:
#endif
#if AP_RCPROTOCOL_SBUS_NI_ENABLED
        case SBUS_NI:
#endif
#if AP_RCPROTOCOL_PPMSUM_ENABLED
        case PPMSUM:
#endif
#if AP_RCPROTOCOL_FPORT_ENABLED
        case FPORT:
#endif
#if AP_RCPROTOCOL_FPORT2_ENABLED
        case FPORT2:
#endif
#if AP_RCPROTOCOL_CRSF_ENABLED
        case CRSF:
#endif
            return true;
#if AP_RCPROTOCOL_IBUS_ENABLED
        case IBUS:
#endif
#if AP_RCPROTOCOL_SUMD_ENABLED
        case SUMD:
#endif
#if AP_RCPROTOCOL_SRXL_ENABLED
        case SRXL:
#endif
#if AP_RCPROTOCOL_SRXL2_ENABLED
        case SRXL2:
#endif
#if AP_RCPROTOCOL_ST24_ENABLED
        case ST24:
#endif
#if AP_RCPROTOCOL_DRONECAN_ENABLED
        case DRONECAN:
#endif
        case NONE:
            return false;
        }
        return false;
    }

    uint8_t num_channels();
    uint16_t read(uint8_t chan);
    void read(uint16_t *pwm, uint8_t n);
    bool new_input();
    void start_bind(void);
    int16_t get_RSSI(void) const;
    int16_t get_rx_link_quality(void) const;

    // return protocol name as a string
    const char *protocol_name(void) const;

    // return detected protocol
    enum rcprotocol_t protocol_detected(void) const {
        return _detected_protocol;
    }

    // add a UART for RCIN
    void add_uart(AP_HAL::UARTDriver* uart);
    bool has_uart() const { return added.uart != nullptr; }

    // set allowed RC protocols
    void set_rc_protocols(uint32_t mask) {
        rc_protocols_mask = mask;
    }

    class SerialConfig {
    public:
        void apply_to_uart(AP_HAL::UARTDriver *uart) const;

        uint32_t baud;
        uint8_t parity;
        uint8_t stop_bits;
        bool invert_rx;
    };

    // return true if we are decoding a byte stream, instead of pulses
    bool using_uart(void) const {
        return _detected_with_bytes;
    }

private:
    void check_added_uart(void);

    // return true if a specific protocol is enabled
    bool protocol_enabled(enum rcprotocol_t protocol) const;

    enum rcprotocol_t _detected_protocol = NONE;
    uint16_t _disabled_for_pulses;
    bool _detected_with_bytes;
    AP_RCProtocol_Backend *backend[NONE];
    bool _new_input;
    uint32_t _last_input_ms;
    bool _failsafe_active;
    bool _valid_serial_prot;

    // optional additional uart
    struct {
        AP_HAL::UARTDriver *uart;
        bool opened;
        uint32_t last_config_change_ms;
        uint8_t config_num;
    } added;

    // allowed RC protocols mask (first bit means "all")
    uint32_t rc_protocols_mask;

#endif  // AP_RCPROTCOL_ENABLED

};

#if AP_RCPROTOCOL_ENABLED
namespace AP {
    AP_RCProtocol &RC();
};

#include "AP_RCProtocol_Backend.h"
#endif  // AP_RCProtocol_enabled
