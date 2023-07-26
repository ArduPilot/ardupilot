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

#include "AP_RangeFinder_LeddarVu8.h"

#if AP_RANGEFINDER_LEDDARVU8_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

// LeddarVu8 uses the modbus RTU protocol
// https://autonomoustuff.com/product/leddartech-vu8/

#define LEDDARVU8_ADDR_DEFAULT              0x01        // modbus default device id
#define LEDDARVU8_DIST_MAX_CM               18500       // maximum possible distance reported by lidar
#define LEDDARVU8_OUT_OF_RANGE_ADD_CM       100         // add this many cm to out-of-range values
#define LEDDARVU8_TIMEOUT_MS                200         // timeout in milliseconds if no distance messages received

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_LeddarVu8::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // check for timeout receiving messages
    uint32_t now_ms = AP_HAL::millis();
    if (((now_ms - last_distance_ms) > LEDDARVU8_TIMEOUT_MS) && ((now_ms - last_distance_request_ms) > LEDDARVU8_TIMEOUT_MS)) {
        request_distances();
    }

    // variables for averaging results from multiple messages
    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;
    uint16_t latest_dist_cm = 0;
    bool latest_dist_valid = false;

    // read any available characters from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        if (parse_byte((uint8_t)r, latest_dist_valid, latest_dist_cm)) {
            if (latest_dist_valid) {
                sum_cm += latest_dist_cm;
                count++;
            } else {
                count_out_of_range++;
            }

        }
    }

    if (count > 0 || count_out_of_range > 0) {
        // record time of successful read and request another reading
        last_distance_ms = now_ms;
        request_distances();

        if (count > 0) {
            // return average distance of readings
            reading_m = (sum_cm * 0.01f) / count;
        } else {
            // if only out of range readings return larger of
            // driver defined maximum range for the model and user defined max range + 1m
            reading_m = MAX(LEDDARVU8_DIST_MAX_CM, max_distance_cm() + LEDDARVU8_OUT_OF_RANGE_ADD_CM)/100.0f;
        }
        return true;
    }

    // no readings so return false
    return false;
}

// get sensor address from RNGFNDx_ADDR parameter
uint8_t AP_RangeFinder_LeddarVu8::get_sensor_address() const
{
    if (params.address == 0) {
        return LEDDARVU8_ADDR_DEFAULT;
    }
    return params.address;
}

// send request to device to provide distances
void AP_RangeFinder_LeddarVu8::request_distances()
{
    uint8_t req_buf[] = {
            get_sensor_address(),                       // address
            (uint8_t)FunctionCode::READ_INPUT_REGISTER, // function code low
            0,                                          // function code high
            (uint8_t)RegisterNumber::FIRST_DISTANCE0,   // register number low
            0,                                          // register number high
            8,                                          // register count
            0,                                          // crc low
            0                                           // crc high
    };
    const uint8_t req_buf_len = sizeof(req_buf);

    // fill in crc bytes
    uint16_t crc = calc_crc_modbus(req_buf, req_buf_len - 2);
    req_buf[req_buf_len - 2] = LOWBYTE(crc);
    req_buf[req_buf_len - 1] = HIGHBYTE(crc);

    // send request to device
    uart->write(req_buf, req_buf_len);

    // record time of request
    last_distance_request_ms = AP_HAL::millis();
}

// process one byte received on serial port
// returns true if successfully parsed a message
// if distances are valid, valid_readings is set to true and distance is stored in reading_cm
bool AP_RangeFinder_LeddarVu8::parse_byte(uint8_t b, bool &valid_reading, uint16_t &reading_cm)
{
    // process byte depending upon current state
    switch (parsed_msg.state) {

    case ParseState::WAITING_FOR_ADDRESS: {
        if (b == get_sensor_address()) {
            parsed_msg.address = b;
            parsed_msg.state = ParseState::WAITING_FOR_FUNCTION_CODE;
        }
        break;
    }

    case ParseState::WAITING_FOR_FUNCTION_CODE:
        if (b == (uint8_t)FunctionCode::READ_INPUT_REGISTER) {
            parsed_msg.function_code = b;
            parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD_LEN;
        } else {
            parsed_msg.state = ParseState::WAITING_FOR_ADDRESS;
        }
        break;

    case ParseState::WAITING_FOR_PAYLOAD_LEN:
        // only parse messages of the expected length
        if (b == LEDDARVU8_PAYLOAD_LENGTH) {
            parsed_msg.payload_len = b;
            parsed_msg.payload_recv = 0;
            parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD;
        } else {
            parsed_msg.state = ParseState::WAITING_FOR_ADDRESS;
        }
        break;

    case ParseState::WAITING_FOR_PAYLOAD:
        if (parsed_msg.payload_recv < parsed_msg.payload_len) {
            if (parsed_msg.payload_recv < ARRAY_SIZE(parsed_msg.payload)) {
                parsed_msg.payload[parsed_msg.payload_recv] = b;
            }
            parsed_msg.payload_recv++;
        }
        if (parsed_msg.payload_recv == parsed_msg.payload_len) {
            parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
        }
        break;

    case ParseState::WAITING_FOR_CRC_LOW:
        parsed_msg.crc = b;
        parsed_msg.state = ParseState::WAITING_FOR_CRC_HIGH;
        break;

    case ParseState::WAITING_FOR_CRC_HIGH: {
        parsed_msg.crc |= ((uint16_t)b << 8);
        parsed_msg.state = ParseState::WAITING_FOR_ADDRESS;

        // check crc
        uint16_t expected_crc = calc_crc_modbus(&parsed_msg.address, 3+parsed_msg.payload_recv);
        if (expected_crc == parsed_msg.crc) {
            // calculate and return shortest distance
            reading_cm = 0;
            valid_reading = false;
            for (uint8_t i=0; i<8; i++) {
                uint8_t ix2 = i*2;
                const uint16_t dist_cm = (uint16_t)parsed_msg.payload[ix2] << 8 | (uint16_t)parsed_msg.payload[ix2+1];
                if ((dist_cm > 0) && (!valid_reading || (dist_cm < reading_cm))) {
                    reading_cm = dist_cm;
                    valid_reading = true;
                }
            }
            return true;
        }
        break;
    }
    }

    valid_reading = false;
    return false;
}

#endif  // AP_RANGEFINDER_LEDDARVU8_ENABLED
