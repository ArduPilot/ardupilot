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
 */

#pragma once

#define IBUS_FRAME_SIZE		32
#define IBUS_INPUT_CHANNELS	14

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

class AP_RCProtocol_IBUS : public AP_RCProtocol_Backend
{
public:
    AP_RCProtocol_IBUS(AP_RCProtocol &_frontend);
    void process_pulse(const uint32_t &width_s0, const uint32_t &width_s1, const uint8_t &pulse_id) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;
private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    bool ibus_decode(const uint8_t frame[IBUS_FRAME_SIZE], uint16_t *values, bool *ibus_failsafe);


    struct {
        uint8_t buf[IBUS_FRAME_SIZE];
        uint8_t ofs;
        uint32_t last_byte_us;
    } byte_input;
};
