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

class AP_RCProtocol_SBUS : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_SBUS(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
private:
    bool sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values,
                     bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values);
    struct {
        uint16_t bytes[25]; // including start bit, parity and stop bits
        uint16_t bit_ofs;
    } sbus_state;
};