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

#include "AP_LightWareSerial.h"

#if AP_LIGHTWARESERIAL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_HEADER    0xAA

// send message to sensor
void AP_LightWareSerial::send_message(uint8_t msgid, bool write, const uint8_t *payload, uint16_t payload_len)
{
    if ((_uart == nullptr) || (payload_len > LIGHTWARE_PAYLOAD_LEN_MAX)) {
        return;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < payload_len + 6U) {
        return;
    }

    // write header
    _uart->write((uint8_t)LIGHTWARE_HEADER);
    uint16_t crc = crc_xmodem_update(0, LIGHTWARE_HEADER);

    // write flags including payload length
    const uint16_t flags = ((payload_len+1) << 6) | (write ? 0x01 : 0);
    _uart->write(LOWBYTE(flags));
    crc = crc_xmodem_update(crc, LOWBYTE(flags));
    _uart->write(HIGHBYTE(flags));
    crc = crc_xmodem_update(crc, HIGHBYTE(flags));

    // msgid
    _uart->write(msgid);
    crc = crc_xmodem_update(crc, msgid);

    // payload
    if ((payload_len > 0) && (payload != nullptr)) {
        for (uint16_t i = 0; i < payload_len; i++) {
            _uart->write(payload[i]);
            crc = crc_xmodem_update(crc, payload[i]);
        }
    }

    // checksum
    _uart->write(LOWBYTE(crc));
    _uart->write(HIGHBYTE(crc));
}

// process one byte received on serial port
// returns true if a complete message has been received
// state is stored in _msg structure
bool AP_LightWareSerial::parse_byte(uint8_t b)
{
    // check that payload buffer is large enough
    static_assert(ARRAY_SIZE(_msg.payload) == LIGHTWARE_PAYLOAD_LEN_MAX, "AP_LightWareSerial: check _msg.payload array size");

    // process byte depending upon current state
    switch (_parse_state) {

    case ParseState::HEADER:
        if (b == LIGHTWARE_HEADER) {
            _crc_expected = crc_xmodem_update(0, b);
            _parse_state = ParseState::FLAGS_L;
        }
        break;

    case ParseState::FLAGS_L:
        _msg.flags_low = b;
        _crc_expected = crc_xmodem_update(_crc_expected, b);
        _parse_state = ParseState::FLAGS_H;
        break;

    case ParseState::FLAGS_H:
        _msg.flags_high = b;
        _crc_expected = crc_xmodem_update(_crc_expected, b);
        _msg.payload_len = UINT16_VALUE(_msg.flags_high, _msg.flags_low) >> 6;
        if ((_msg.payload_len == 0) || (_msg.payload_len > LIGHTWARE_PAYLOAD_LEN_MAX)) {
            // invalid payload length, abandon message
            _parse_state = ParseState::HEADER;
        } else {
            _parse_state = ParseState::MSG_ID;
        }
        break;

    case ParseState::MSG_ID:
        _msg.msgid = b;
        _crc_expected = crc_xmodem_update(_crc_expected, b);
        if (_msg.payload_len > 1) {
            _parse_state = ParseState::PAYLOAD;
        } else {
            _parse_state = ParseState::CRC_L;
        }
        _payload_recv = 0;
        break;

    case ParseState::PAYLOAD:
        if (_payload_recv < (_msg.payload_len - 1)) {
            _msg.payload[_payload_recv] = b;
            _payload_recv++;
            _crc_expected = crc_xmodem_update(_crc_expected, b);
        }
        if (_payload_recv >= (_msg.payload_len - 1)) {
            _parse_state = ParseState::CRC_L;
        }
        break;

    case ParseState::CRC_L:
        _msg.crc_low = b;
        _parse_state = ParseState::CRC_H;
        break;

    case ParseState::CRC_H:
        _parse_state = ParseState::HEADER;
        _msg.crc_high = b;
        if (_crc_expected == UINT16_VALUE(_msg.crc_high, _msg.crc_low)) {
            return true;
        }
        break;
    }

    return false;
}

#endif //AP_LIGHTWARESERIAL_ENABLED

