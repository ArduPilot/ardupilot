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

#define MAX_RCIN_CHANNELS 32
#define MIN_RCIN_CHANNELS  5

class AP_RCProtocol_Backend;

class AP_RCProtocol {
public:
    AP_RCProtocol() {
        instance = this;
    }
    ~AP_RCProtocol();

    enum rcprotocol_t {
        PPM = 0,
        SBUS,
        SBUS_NI,
        DSM,
        SUMD,
        SRXL,
        ST24,
        NONE    //last enum always is None
    };
    void init();
    bool valid_serial_prot()
    {
        return _valid_serial_prot;
    }
    void process_pulse(uint32_t width_s0, uint32_t width_s1);
    void process_pulse_list(const uint32_t *widths, uint16_t n, bool need_swap);
    void process_byte(uint8_t byte, uint32_t baudrate);

    void disable_for_pulses(enum rcprotocol_t protocol) {
        _disabled_for_pulses |= (1U<<(uint8_t)protocol);
    }

    // for protocols without strong CRCs we require 3 good frames to lock on
    bool requires_3_frames(enum rcprotocol_t p) {
        return (p == DSM || p == SBUS || p == SBUS_NI || p == PPM);
    }
    
    enum rcprotocol_t protocol_detected()
    {
        return _detected_protocol;
    }
    uint8_t num_channels();
    uint16_t read(uint8_t chan);
    bool new_input();
    void start_bind(void);

    // return protocol name as a string
    static const char *protocol_name_from_protocol(rcprotocol_t protocol);

    // return protocol name as a string
    const char *protocol_name(void) const;

    // return protocol name as a string
    enum rcprotocol_t protocol_detected(void) const {
        return _detected_protocol;
    }
    
    // access to singleton
    static AP_RCProtocol *get_instance(void) {
        return instance;
    }

private:
    enum rcprotocol_t _detected_protocol = NONE;
    uint16_t _disabled_for_pulses;
    bool _detected_with_bytes;
    AP_RCProtocol_Backend *backend[NONE];
    bool _new_input = false;
    uint32_t _last_input_ms;
    bool _valid_serial_prot = false;
    uint8_t _good_frames[NONE];

    static AP_RCProtocol *instance;
};

#include "AP_RCProtocol_Backend.h"
