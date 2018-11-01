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
    void process_byte(uint8_t byte);
    enum rcprotocol_t protocol_detected()
    {
        return _detected_protocol;
    }
    uint8_t num_channels();
    uint16_t read(uint8_t chan);
    bool new_input();
    void start_bind(void);

    // access to singleton
    static AP_RCProtocol *get_instance(void) {
        return instance;
    }

private:
    enum rcprotocol_t _detected_protocol = NONE;
    bool _detected_with_bytes;
    AP_RCProtocol_Backend *backend[NONE];
    bool _new_input = false;
    uint32_t _last_input_ms;
    bool _valid_serial_prot = false;

    static AP_RCProtocol *instance;
};

#include "AP_RCProtocol_Backend.h"
