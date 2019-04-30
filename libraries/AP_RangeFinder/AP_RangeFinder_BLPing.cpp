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

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_RangeFinder_BLPing.h"

#define BLPING_TIMEOUT_MS       500     // sensor timeout after 0.5 sec
#define BLPING_INIT_RATE_MS     1000    // initialise sensor at no more than 1hz
#define BLPING_FRAME_HEADER1    0x42    // header first byte ('B')
#define BLPING_FRAME_HEADER2    0x52    // header second byte ('R')

#define BLPING_SRC_ID           0       // vehicle's source id
#define BLPING_DEST_ID          1       // sensor's id

#define BLPING_MSGID_ACK                    1
#define BLPING_MSGID_NACK                   2
#define BLPING_MSGID_SET_PING_INTERVAL      1004
#define BLPING_MSGID_GET_DEVICE_ID          1201
#define BLDPIN_MSGID_DISTANCE_SIMPLE        1211
#define BLPING_MSGID_CONTINUOUS_START       1400

// Protocol implemented by this sensor can be found here: https://github.com/bluerobotics/ping-protocol
//
// Byte     Type        Name            Description
// --------------------------------------------------------------------------------------------------------------
// 0        uint8_t     start1          'B'
// 1        uint8_t     start2          'R'
// 2-3      uint16_t    payload_length  number of bytes in payload (low byte, high byte)
// 4-5      uint16_t    message id      message id (low byte, high byte)
// 6        uint8_t     src_device_id   id of device sending the message
// 7        uint8_t     dst_device_id   id of device of the intended recipient
// 8-n      uint8_t[]   payload         message payload
// (n+1)-(n+2)  uint16_t    checksum    the sum of all the non-checksum bytes in the message (low byte, high byte)

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_BLPing::AP_RangeFinder_BLPing(RangeFinder::RangeFinder_State &_state,
                                             AP_RangeFinder_Params &_params,
                                             AP_SerialManager &serial_manager,
                                             uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

// detect if a serial port has been setup to accept rangefinder input
bool AP_RangeFinder_BLPing::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_BLPing::update(void)
{
    if (uart == nullptr) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = now;
        update_status();
    } else if (now - state.last_reading_ms > BLPING_TIMEOUT_MS) {
        set_status(RangeFinder::RangeFinder_NoData);

        // initialise sensor if no distances recently
        if (now - last_init_ms > BLPING_INIT_RATE_MS) {
            last_init_ms = now;
            init_sensor();
        }
    }
}

void AP_RangeFinder_BLPing::init_sensor()
{
    // request distance from sensor
    send_message(BLDPIN_MSGID_DISTANCE_SIMPLE, nullptr, 0);
}

// send message to sensor
void AP_RangeFinder_BLPing::send_message(uint16_t msgid, const uint8_t *payload, uint16_t payload_len)
{
    if (uart == nullptr) {
        return;
    }

    // check for sufficient space in outgoing buffer
    if (uart->txspace() < payload_len + 10U) {
        return;
    }

    // write header
    uart->write((uint8_t)BLPING_FRAME_HEADER1);
    uart->write((uint8_t)BLPING_FRAME_HEADER2);
    uint16_t crc = BLPING_FRAME_HEADER1 + BLPING_FRAME_HEADER2;

    // write payload length
    uart->write(LOWBYTE(payload_len));
    uart->write(HIGHBYTE(payload_len));
    crc += LOWBYTE(payload_len) + HIGHBYTE(payload_len);

    // msgid
    uart->write(LOWBYTE(msgid));
    uart->write(HIGHBYTE(msgid));
    crc += LOWBYTE(msgid) + HIGHBYTE(msgid);

    // src dev id
    uart->write((uint8_t)BLPING_SRC_ID);
    crc += BLPING_SRC_ID;

    // destination dev id
    uart->write((uint8_t)BLPING_DEST_ID);
    crc += BLPING_DEST_ID;

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

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_BLPing::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        const int16_t b = uart->read();
        if (b < 0) {
            break;
        }
        if (parse_byte(b)) {
            count++;
            sum_cm += distance_cm;
        }
    }

    if (count > 0) {
        // return average distance of readings
        reading_cm = sum_cm / count;

        // request another distance
        send_message(BLDPIN_MSGID_DISTANCE_SIMPLE, nullptr, 0);

        return true;
    }

    // no readings so return false
    return false;
}

// process one byte received on serial port
// returns true if a distance message has been successfully parsed
// state is stored in msg structure
bool AP_RangeFinder_BLPing::parse_byte(uint8_t b)
{
    bool got_distance = false;

    // process byte depending upon current state
    switch (msg.state) {

    case ParseState::HEADER1:
        if (b == BLPING_FRAME_HEADER1) {
            msg.crc_expected = BLPING_FRAME_HEADER1;
            msg.state = ParseState::HEADER2;
        }
        break;

    case ParseState::HEADER2:
        if (b == BLPING_FRAME_HEADER2) {
            msg.crc_expected += BLPING_FRAME_HEADER2;
            msg.state = ParseState::LEN_L;
        } else {
            msg.state = ParseState::HEADER1;
        }
        break;

    case ParseState::LEN_L:
        msg.payload_len = b;
        msg.crc_expected += b;
        msg.state = ParseState::LEN_H;
        break;

    case ParseState::LEN_H:
        msg.payload_len |= ((uint16_t)b << 8);
        msg.payload_recv = 0;
        msg.crc_expected += b;
        msg.state = ParseState::MSG_ID_L;
        break;

    case ParseState::MSG_ID_L:
        msg.msgid = b;
        msg.crc_expected += b;
        msg.state = ParseState::MSG_ID_H;
        break;

    case ParseState::MSG_ID_H:
        msg.msgid |= ((uint16_t)b << 8);
        msg.crc_expected += b;
        msg.state = ParseState::SRC_ID;
        break;

    case ParseState::SRC_ID:
        msg.crc_expected += b;
        msg.state = ParseState::DST_ID;
        break;

    case ParseState::DST_ID:
        msg.crc_expected += b;
        msg.state = ParseState::PAYLOAD;
        break;

    case ParseState::PAYLOAD:
        if (msg.payload_recv < msg.payload_len) {
            if (msg.payload_recv < ARRAY_SIZE(msg.payload)) {
                msg.payload[msg.payload_recv] = b;
            }
            msg.payload_recv++;
            msg.crc_expected += b;
        }
        if (msg.payload_recv == msg.payload_len) {
            msg.state = ParseState::CRC_L;
        }
        break;

    case ParseState::CRC_L:
        msg.crc = b;
        msg.state = ParseState::CRC_H;
        break;

    case ParseState::CRC_H:
        msg.crc |= ((uint16_t)b << 8);
        msg.state = ParseState::HEADER1;
        if (msg.crc_expected == msg.crc) {

            // process payload
            switch (msg.msgid) {

            case BLPING_MSGID_ACK:
            case BLPING_MSGID_NACK:
                // ignore
                break;

            case BLDPIN_MSGID_DISTANCE_SIMPLE:
                const uint32_t dist_mm = (uint32_t)msg.payload[0] | (uint32_t)msg.payload[1] << 8 | (uint32_t)msg.payload[2] << 16 | (uint32_t)msg.payload[3] << 24;
                distance_cm = dist_mm / 10;
                got_distance = true;
                break;
            }
        }
        break;
    }

    return got_distance;
}
