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
/*
  Base class for FrSky D telemetery
*/

#include "SIM_Frsky_D.h"

#include <stdio.h>

using namespace SITL;

// sadly, this pulls START_STOP_D etc in from the frsky header.
#include <GCS_MAVLink/GCS.h>

// const uint8_t START_STOP_D = 0x5E;
// const uint8_t BYTESTUFF_D = 0x5D;

void Frsky_D::handle_data(uint8_t id, uint16_t data)
{
    ::fprintf(stderr,
              "Frsky: id=%s (0x%02X) data=%u\n",
              dataid_string((DataID)id),
              (unsigned)_id,
              (unsigned)data);
}

void Frsky_D::update()
{
    const ssize_t n = read_from_autopilot(&_buffer[_buflen], ARRAY_SIZE(_buffer) - _buflen - 1);
    if (n != -1) {
        _buflen += n;
    }

    if (_buflen == 0) {
        return;
    }

    while (_buflen) {
        switch (_state) {
        case State::WANT_START_STOP_D:
            if (_buffer[0] != START_STOP_D) {
                AP_HAL::panic("Corrupt?");
                // _lost_bytes++;
                continue;
            }
            memcpy(&_buffer[0], &_buffer[1], --_buflen); //srsly?!
            _state = State::WANT_ID;
            break;
        case State::WANT_ID:
            _id = _buffer[0];
            memcpy(&_buffer[0], &_buffer[1], --_buflen); //srsly?!
            _state = State::WANT_BYTE1;
            break;
        case State::WANT_BYTE1:
        case State::WANT_BYTE2: {
            uint8_t byte;
            uint8_t consume = 1;
            if (_buffer[0] == 0x5D) {
                // byte-stuffed
                if (_buflen < 2) {
                    return;
                }
                if (_buffer[1] == 0x3E) {
                    byte = START_STOP_D;
                } else if (_buffer[1] == 0x3D) {
                    byte = BYTESTUFF_D;
                } else {
                    AP_HAL::panic("Unknown stuffed byte");
                }
                consume = 2;
            } else {
                byte = _buffer[0];
            }

            memcpy(&_buffer[0], &_buffer[consume], _buflen-consume);
            _buflen -= consume;

            switch (_state) {
            case State::WANT_ID:
            case State::WANT_START_STOP_D:
                AP_HAL::panic("Should not get here");
            case State::WANT_BYTE1:
                _data = byte;
                _state = State::WANT_BYTE2;
                break;
            case State::WANT_BYTE2:
                _data |= byte << 8;
                handle_data(_id, _data);
                _state = State::WANT_START_STOP_D;
                break;
            }
        }
        }
    }
}
