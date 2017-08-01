/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_ParticleSensor_SDS021.h"

#include <stdio.h>

#include <AP_Logger/AP_Logger.h>

AP_ParticleSensor_SDS021::AP_ParticleSensor_SDS021(AP_HAL::UARTDriver &_port) :
        port(_port)
{
        port.begin(9600);
        port.set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
}

void AP_ParticleSensor_SDS021::yield_message()
{
    ::fprintf(stderr, "SDS021: %u: PM1.0=%0.1f PM2.5=%0.1f (bad=%u cksum fails=%u)\n", AP_HAL::millis(), reading.pm10/10.0, reading.pm25/10.0, bad_chars, checksum_failures);
    const uint64_t now = AP_HAL::micros64();
    AP::logger().Write(
        "S021",
        "TimeUS,pm10,pm25",
        "Qhh",
        now, reading.pm10, reading.pm25);
}

void AP_ParticleSensor_SDS021::handle_byte_read(const uint8_t byte)
{
    // fprintf(stderr, "Got byte (0x%02x) (state=%u)\n", byte, state);
    switch(state) {
    case State::WantHeader:
        switch(byte) {
        case 0xAA:
            state = State::WantCommand;
            break;
        default:
            bad_chars++;
            break;
        }
        break;
    case State::WantCommand:
        switch(byte) {
        case 0xC0:
            state = State::WantData1;
            break;
        default:
            bad_chars++;
            state = State::WantHeader;
            break;
        }
        break;
    case State::WantData1:
        reading.pm25 = byte;
        state = State::WantData2;
        checksum = byte;
        break;
    case State::WantData2:
        reading.pm25 |= byte<<8;
        state = State::WantData3;
        checksum += byte;
        break;
    case State::WantData3:
        reading.pm10 = byte;
        state = State::WantData4;
        checksum += byte;
        break;
    case State::WantData4:
        reading.pm10 |= byte<<8;
        state = State::WantData5;
        checksum += byte;
        break;
    case State::WantData5:
        state = State::WantData6;
        checksum += byte;
        break;
    case State::WantData6:
        state = State::WantChecksum;
        checksum += byte;
        break;
    case State::WantChecksum:
        if (byte != checksum) {
            checksum_failures++;
            fprintf(stderr, "checksum failure (%u) vs (%u)\n", byte, checksum);
            state = State::WantHeader;
            break;
        }
        state = State::WantTail;
        yield_message();
        break;
    case State::WantTail:
        if (byte != 0xAB) {
            bad_chars++;
        }
        state = State::WantHeader;
        break;
    }
}

void AP_ParticleSensor_SDS021::update()
{
    uint8_t count = 10; // maximum characters to process per update
    while (count-- > 0 && port.available()) {
        const uint8_t next = port.read();
        handle_byte_read((uint8_t)next);
    }
}
