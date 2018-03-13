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

#include "AP_RCProtocol.h"

class AP_RCProtocol_Backend
{
    friend class AP_RCProtcol;

public:
    AP_RCProtocol_Backend(AP_RCProtocol &_frontend);
    virtual void process_pulse(uint32_t width_s0, uint32_t width_s1) = 0;
    uint16_t read(uint8_t chan);
    bool new_input();
    uint8_t num_channels();

protected:
    void add_input(uint8_t num_channels, uint16_t *values, bool in_failsafe);
    
private:
    AP_RCProtocol &frontend;
    unsigned int rc_input_count;
    unsigned int last_rc_input_count;

    uint16_t _pwm_values[MAX_RCIN_CHANNELS];
    uint8_t  _num_channels;
};
