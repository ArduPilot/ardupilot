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

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

#define DEBUG_FRAMELEN_MAX 40

class AP_RCProtocol_Debug : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_Debug(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;
private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    uint8_t buffer[DEBUG_FRAMELEN_MAX];       /* buffer for raw frame data in correct order --> buffer[0]=byte0  buffer[1]=byte1  */
    uint8_t buflen;                          /* length in number of bytes of received dataframe in buffer  */
    uint32_t last_data_us;                   /* timespan since last received data in us */

    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
};
