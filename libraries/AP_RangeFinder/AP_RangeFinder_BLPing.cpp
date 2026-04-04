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

#include "AP_RangeFinder_BLPing.h"

#if AP_RANGEFINDER_BLPING_ENABLED

#include <AP_HAL/AP_HAL.h>

void AP_RangeFinder_BLPing::update(void)
{
    if (uart == nullptr) {
        return;
    }
    AP_RangeFinder_Backend_Serial::update();

    if (status() == RangeFinder::Status::NoData) {
        const uint32_t now = AP_HAL::millis();
        // initialise sensor if no distances recently
        if (now - last_init_ms > read_timeout_ms()) {
            last_init_ms = now;
            init_sensor();
        }
    }
}

void AP_RangeFinder_BLPing::init_sensor()
{
    // Set message interval between pings in ms
    uint16_t ping_interval = _sensor_rate_ms;
    protocol.send_message(uart, PingProtocol::MessageId::SET_PING_INTERVAL, reinterpret_cast<uint8_t*>(&ping_interval), sizeof(ping_interval));

    // Send a message requesting a continuous
    uint16_t continuous_message = static_cast<uint16_t>(PingProtocol::MessageId::DISTANCE_SIMPLE);
    protocol.send_message(uart, PingProtocol::MessageId::CONTINUOUS_START, reinterpret_cast<uint8_t*>(&continuous_message), sizeof(continuous_message));
}

// distance returned in reading_m, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_BLPing::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    struct {
        float sum_cm = 0;
        uint16_t count = 0;
        float mean() const { return sum_cm / count; };
    } averageStruct;

    // read any available lines from the lidar
    for (auto i=0; i<8192; i++) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        if (protocol.parse_byte(b) == PingProtocol::MessageId::DISTANCE_SIMPLE) {
            averageStruct.count++;
            averageStruct.sum_cm += protocol.get_distance_mm()*0.1f;
        }
    }

    if (averageStruct.count > 0) {
        // return average distance of readings
        reading_m = averageStruct.mean() * 0.01f;
        return true;
    }

    // no readings so return false
    return false;
}

int8_t AP_RangeFinder_BLPing::get_signal_quality_pct() const
{
    if (status() != RangeFinder::Status::Good) {
        return RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    }
    return protocol.get_confidence();
}

uint8_t PingProtocol::get_confidence() const
{
    return msg.payload[4];
}

uint32_t PingProtocol::get_distance_mm() const
{
    return (uint32_t)msg.payload[0] |
            (uint32_t)msg.payload[1] << 8 |
            (uint32_t)msg.payload[2] << 16 |
            (uint32_t)msg.payload[3] << 24;
}

void PingProtocol::send_message(AP_HAL::UARTDriver *uart, PingProtocol::MessageId msg_id, const uint8_t *payload, uint16_t payload_len) const
{
    if (uart == nullptr) {
        return;
    }

    // check for sufficient space in outgoing buffer
    if (uart->txspace() < payload_len + 10U) {
        return;
    }

    // write header
    uart->write(_frame_header1);
    uart->write(_frame_header2);
    uint16_t crc = _frame_header1 + _frame_header2;

    // write payload length
    uart->write(LOWBYTE(payload_len));
    uart->write(HIGHBYTE(payload_len));
    crc += LOWBYTE(payload_len) + HIGHBYTE(payload_len);

    // message id
    uart->write(LOWBYTE(msg_id));
    uart->write(HIGHBYTE(msg_id));
    crc += LOWBYTE(msg_id) + HIGHBYTE(msg_id);

    // src dev id
    uart->write(_src_id);
    crc += _src_id;

    // destination dev id
    uart->write(_dst_id);
    crc += _dst_id;

    // payload
    if (payload != nullptr) {
        for (uint16_t i = 0; i<payload_len; i++) {
            uart->write(payload[i]);
            crc += payload[i];
        }
    }

    // checksum
    uart->write(LOWBYTE(crc));
    uart->write(HIGHBYTE(crc));
}

PingProtocol::MessageId PingProtocol::parse_byte(uint8_t b)
{
    // process byte depending upon current state
    switch (msg.state) {

    case ParserState::HEADER1:
        if (b == _frame_header1) {
            msg.crc_expected = _frame_header1;
            msg.state = ParserState::HEADER2;
            msg.done = false;
        }
        break;

    case ParserState::HEADER2:
        if (b == _frame_header2) {
            msg.crc_expected += _frame_header2;
            msg.state = ParserState::LEN_L;
        } else {
            msg.state = ParserState::HEADER1;
        }
        break;

    case ParserState::LEN_L:
        msg.payload_len = b;
        msg.crc_expected += b;
        msg.state = ParserState::LEN_H;
        break;

    case ParserState::LEN_H:
        msg.payload_len |= ((uint16_t)b << 8);
        msg.payload_recv = 0;
        msg.crc_expected += b;
        msg.state = ParserState::MSG_ID_L;
        break;

    case ParserState::MSG_ID_L:
        msg.id = b;
        msg.crc_expected += b;
        msg.state = ParserState::MSG_ID_H;
        break;

    case ParserState::MSG_ID_H:
        msg.id |= ((uint16_t)b << 8);
        msg.crc_expected += b;
        msg.state = ParserState::SRC_ID;
        break;

    case ParserState::SRC_ID:
        msg.crc_expected += b;
        msg.state = ParserState::DST_ID;
        break;

    case ParserState::DST_ID:
        msg.crc_expected += b;
        msg.state = ParserState::PAYLOAD;
        break;

    case ParserState::PAYLOAD:
        if (msg.payload_recv < msg.payload_len) {
            if (msg.payload_recv < ARRAY_SIZE(msg.payload)) {
                msg.payload[msg.payload_recv] = b;
            }
            msg.payload_recv++;
            msg.crc_expected += b;
        }
        if (msg.payload_recv == msg.payload_len) {
            msg.state = ParserState::CRC_L;
        }
        break;

    case ParserState::CRC_L:
        msg.crc = b;
        msg.state = ParserState::CRC_H;
        break;

    case ParserState::CRC_H:
        msg.crc |= ((uint16_t)b << 8);
        msg.state = ParserState::HEADER1;
        msg.done = msg.crc_expected == msg.crc;
        break;
    }


    return msg.done ? get_message_id() : MessageId::INVALID;
}

#endif  // AP_RANGEFINDER_BLPING_ENABLED
