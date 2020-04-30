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
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#define MAX_RCIN_CHANNELS 18
#define MIN_RCIN_CHANNELS  5

class AP_RCProtocol_Backend;

class AP_RCProtocol {
public:
    AP_RCProtocol() {}
    ~AP_RCProtocol();
    friend class AP_RCProtocol_Backend;

    enum rcprotocol_t {
        PPM = 0,
        IBUS,
        SBUS,
        SBUS_NI,
        DSM,
        SUMD,
        SRXL,
        SRXL2,
        CRSF,
        ST24,
        FPORT,
        NONE    //last enum always is None
    };
    void init();
    bool valid_serial_prot()
    {
        return _valid_serial_prot;
    }
    void process_pulse(uint32_t width_s0, uint32_t width_s1);
    void process_pulse_list(const uint32_t *widths, uint16_t n, bool need_swap);
    bool process_byte(uint8_t byte, uint32_t baudrate);
    void update(void);

    void disable_for_pulses(enum rcprotocol_t protocol) {
        _disabled_for_pulses |= (1U<<(uint8_t)protocol);
    }

    // for protocols without strong CRCs we require 3 good frames to lock on
    bool requires_3_frames(enum rcprotocol_t p) {
        return (p == DSM || p == SBUS || p == SBUS_NI || p == PPM || p == FPORT);
    }

    uint8_t num_channels();
    uint16_t read(uint8_t chan);
    void read(uint16_t *pwm, uint8_t n);
    bool new_input();
    void start_bind(void);
    int16_t get_RSSI(void) const;

    // return protocol name as a string
    static const char *protocol_name_from_protocol(rcprotocol_t protocol);

    // return protocol name as a string
    const char *protocol_name(void) const;

    // return detected protocol
    enum rcprotocol_t protocol_detected(void) const {
        return _detected_protocol;
    }

    // add a UART for RCIN
    void add_uart(AP_HAL::UARTDriver* uart);

private:
    void check_added_uart(void);

    enum rcprotocol_t _detected_protocol = NONE;
    uint16_t _disabled_for_pulses;
    bool _detected_with_bytes;
    AP_RCProtocol_Backend *backend[NONE];
    bool _new_input;
    uint32_t _last_input_ms;
    bool _valid_serial_prot;
    uint8_t _good_frames[NONE];

    enum config_phase {
        CONFIG_115200_8N1 = 0,
        CONFIG_115200_8N1I = 1,
        CONFIG_100000_8E2I = 2,
        CONFIG_420000_8N1 = 3,
    };

    // optional additional uart
    struct {
        AP_HAL::UARTDriver *uart;
        uint32_t baudrate;
        bool opened;
        uint32_t last_baud_change_ms;
        enum config_phase phase;
    } added;
};

namespace AP {
    AP_RCProtocol &RC();
};

#include "AP_RCProtocol_Backend.h"
