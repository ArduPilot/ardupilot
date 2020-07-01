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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include "AP_Proximity_LightWareSF40C.h"

extern const AP_HAL::HAL& hal;

#define PROXIMITY_SF40C_HEADER                  0xAA
#define PROXIMITY_SF40C_DESIRED_OUTPUT_RATE     3

// update the state of the sensor
void AP_Proximity_LightWareSF40C::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // initialise sensor if necessary
    initialise();

    // process incoming messages
    process_replies();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_SF40C_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// initialise sensor
void AP_Proximity_LightWareSF40C::initialise()
{
    // initialise boundary
    init_boundary();

    // exit immediately if we've sent initialisation requests in the last second
    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_request_ms) < 1000) {
        return;
    }
    _last_request_ms = now_ms;

    // re-fetch motor state
    request_motor_state();

    // get token from sensor (required for reseting)
    if (!got_token()) {
        request_token();
        return;
    }

    // if no replies in last 15 seconds reboot sensor
    if ((now_ms > 30000) && (now_ms - _last_reply_ms > 15000)) {
        restart_sensor();
        return;
    }

    // if motor is starting up give more time to succeed or fail
    if ((_sensor_state.motor_state != MotorState::RUNNING_NORMALLY) &&
        (_sensor_state.motor_state != MotorState::FAILED_TO_COMMUNICATE)) {
        return;
    }

    // if motor fails, reset sensor and re-try everything
    if (_sensor_state.motor_state == MotorState::FAILED_TO_COMMUNICATE) {
        restart_sensor();
        return;
    }

    // motor is running correctly (motor_state is RUNNING_NORMALLY) so request start of streaming
    if (!_sensor_state.streaming || (_sensor_state.output_rate != PROXIMITY_SF40C_DESIRED_OUTPUT_RATE)) {
        request_stream_start();
        return;
    }
}

// restart sensor and re-init our state
void AP_Proximity_LightWareSF40C::restart_sensor()
{
    // return immediately if no token or a restart has been requested within the last 30sec
    uint32_t now_ms = AP_HAL::millis();
    if ((_last_restart_ms != 0) && ((now_ms - _last_restart_ms) < 30000)) {
        return;
    }

    // restart sensor and re-initialise sensor state
    request_reset();
    clear_token();
    _last_restart_ms = now_ms;
    _sensor_state.motor_state = MotorState::UNKNOWN;
    _sensor_state.streaming = false;
    _sensor_state.output_rate = 0;
}

// send message to sensor
void AP_Proximity_LightWareSF40C::send_message(MessageID msgid, bool write, const uint8_t *payload, uint16_t payload_len)
{
    if ((_uart == nullptr) || (payload_len > PROXIMITY_SF40C_PAYLOAD_LEN_MAX)) {
        return;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < payload_len + 6U) {
        return;
    }

    // write header
    _uart->write((uint8_t)PROXIMITY_SF40C_HEADER);
    uint16_t crc = crc_xmodem_update(0, PROXIMITY_SF40C_HEADER);

    // write flags including payload length
    const uint16_t flags = ((payload_len+1) << 6) | (write ? 0x01 : 0);
    _uart->write(LOWBYTE(flags));
    crc = crc_xmodem_update(crc, LOWBYTE(flags));
    _uart->write(HIGHBYTE(flags));
    crc = crc_xmodem_update(crc, HIGHBYTE(flags));

    // msgid
    _uart->write((uint8_t)msgid);
    crc = crc_xmodem_update(crc, (uint8_t)msgid);

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

// request motor state
void AP_Proximity_LightWareSF40C::request_motor_state()
{
    send_message(MessageID::MOTOR_STATE, false, (const uint8_t *)nullptr, 0);
}

// request start of streaming of distances
void AP_Proximity_LightWareSF40C::request_stream_start()
{
    // request output rate
    const uint8_t desired_rate = PROXIMITY_SF40C_DESIRED_OUTPUT_RATE; // 0 = 20010, 1 = 10005, 2 = 6670, 3 = 2001
    send_message(MessageID::OUTPUT_RATE, true, &desired_rate, sizeof(desired_rate));

    // request streaming to start
    const le32_t val = htole32(3);
    send_message(MessageID::STREAM, true, (const uint8_t*)&val, sizeof(val));
}

// request token of sensor
void AP_Proximity_LightWareSF40C::request_token()
{
    // request token
    send_message(MessageID::TOKEN, false, nullptr, 0);
}

// request reset of sensor
void AP_Proximity_LightWareSF40C::request_reset()
{
    // send reset request
    send_message(MessageID::RESET, true, _sensor_state.token, ARRAY_SIZE(_sensor_state.token));
}

// check for replies from sensor
void AP_Proximity_LightWareSF40C::process_replies()
{
    if (_uart == nullptr) {
        return;
    }

    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        const int16_t r = _uart->read();
        if ((r < 0) || (r > 0xFF)) {
            continue;
        }
        parse_byte((uint8_t)r);
    }
}

// process one byte received on serial port
// state is stored in _msg structure
void AP_Proximity_LightWareSF40C::parse_byte(uint8_t b)
{
    // check that payload buffer is large enough
    static_assert(ARRAY_SIZE(_msg.payload) == PROXIMITY_SF40C_PAYLOAD_LEN_MAX, "AP_Proximity_LightwareSF40C: check _msg.payload array size ");

    // process byte depending upon current state
    switch (_msg.state) {

    case ParseState::HEADER:
        if (b == PROXIMITY_SF40C_HEADER) {
            _msg.crc_expected = crc_xmodem_update(0, b);
            _msg.state = ParseState::FLAGS_L;
        }
        break;

    case ParseState::FLAGS_L:
        _msg.flags_low = b;
        _msg.crc_expected = crc_xmodem_update(_msg.crc_expected, b);
        _msg.state = ParseState::FLAGS_H;
        break;

    case ParseState::FLAGS_H:
        _msg.flags_high = b;
        _msg.crc_expected = crc_xmodem_update(_msg.crc_expected, b);
        _msg.payload_len = UINT16_VALUE(_msg.flags_high, _msg.flags_low) >> 6;
        if ((_msg.payload_len == 0) || (_msg.payload_len > PROXIMITY_SF40C_PAYLOAD_LEN_MAX)) {
            // invalid payload length, abandon message
            _msg.state = ParseState::HEADER;
        } else {
            _msg.state = ParseState::MSG_ID;
        }
        break;

    case ParseState::MSG_ID:
        _msg.msgid = (MessageID)b;
        _msg.crc_expected = crc_xmodem_update(_msg.crc_expected, b);
        if (_msg.payload_len > 1) {
            _msg.state = ParseState::PAYLOAD;
        } else {
            _msg.state = ParseState::CRC_L;
        }
        _msg.payload_recv = 0;
        break;

    case ParseState::PAYLOAD:
        if (_msg.payload_recv < (_msg.payload_len - 1)) {
            _msg.payload[_msg.payload_recv] = b;
            _msg.payload_recv++;
            _msg.crc_expected = crc_xmodem_update(_msg.crc_expected, b);
        }
        if (_msg.payload_recv >= (_msg.payload_len - 1)) {
            _msg.state = ParseState::CRC_L;
        }
        break;

    case ParseState::CRC_L:
        _msg.crc_low = b;
        _msg.state = ParseState::CRC_H;
        break;

    case ParseState::CRC_H:
        _msg.crc_high = b;
        if (_msg.crc_expected == UINT16_VALUE(_msg.crc_high, _msg.crc_low)) {
            process_message();
            _last_reply_ms = AP_HAL::millis();
        }
        _msg.state = ParseState::HEADER;
        break;
    }
}

// process the latest message held in the _msg structure
void AP_Proximity_LightWareSF40C::process_message()
{
    // process payload
    switch (_msg.msgid) {
    case MessageID::TOKEN:
        // copy token into _sensor_state.token variable
        if (_msg.payload_recv == ARRAY_SIZE(_sensor_state.token)) {
            memcpy(_sensor_state.token, _msg.payload, ARRAY_SIZE(_sensor_state.token));
        }
        break;
    case MessageID::RESET:
        // no need to do anything
        break;
    case MessageID::STREAM:
        if (_msg.payload_recv == sizeof(uint32_t)) {
            _sensor_state.streaming = (buff_to_uint32(_msg.payload[0], _msg.payload[1], _msg.payload[2], _msg.payload[3]) == 3);
        }
        break;
    case MessageID::DISTANCE_OUTPUT: {
        _last_distance_received_ms = AP_HAL::millis();
        const uint16_t point_total = buff_to_uint16(_msg.payload[8], _msg.payload[9]);
        const uint16_t point_count = buff_to_uint16(_msg.payload[10], _msg.payload[11]);
        const uint16_t point_start_index = buff_to_uint16(_msg.payload[12], _msg.payload[13]);
        // sanity check point_total
        if (point_total == 0) {
            break;
        }

        // prepare to push to object database
        Vector3f current_pos;
        Matrix3f body_to_ned;
        const bool database_ready = database_prepare_for_push(current_pos, body_to_ned);

        // process each point
        const float angle_inc_deg = (1.0f / point_total) * 360.0f;
        const float angle_sign = (frontend.get_orientation(state.instance) == 1) ? -1.0f : 1.0f;
        const float angle_correction = frontend.get_yaw_correction(state.instance);
        const uint16_t dist_min_cm = distance_min() * 100;
        const uint16_t dist_max_cm = distance_max() * 100;

        // mini sectors are used to combine several readings together
        uint8_t combined_count = 0;
        float combined_angle_deg = 0;
        float combined_dist_m = INT16_MAX;
        for (uint16_t i = 0; i < point_count; i++) {
            const uint16_t idx = 14 + (i * 2);
            const int16_t dist_cm = (int16_t)buff_to_uint16(_msg.payload[idx], _msg.payload[idx+1]);
            const float angle_deg = wrap_360((point_start_index + i) * angle_inc_deg * angle_sign + angle_correction);
            const uint8_t sector = convert_angle_to_sector(angle_deg);

            // if we've entered a new sector then finish off previous sector
            if (sector != _last_sector) {
                // update boundary used for avoidance
                if (_last_sector != UINT8_MAX) {
                    update_boundary_for_sector(_last_sector, false);
                }
                // init for new sector
                _last_sector = sector;
                _distance[sector] = INT16_MAX;
                _distance_valid[sector] = false;
            }

            // check reading is not within an ignore zone
            if (!ignore_reading(angle_deg)) {
                // check distance reading is valid
                if ((dist_cm >= dist_min_cm) && (dist_cm <= dist_max_cm)) {
                    const float dist_m = dist_cm * 0.01f;

                    // update shortest distance for this sector
                    if (dist_m < _distance[sector]) {
                        _angle[sector] = angle_deg;
                        _distance[sector] = dist_m;
                        _distance_valid[sector] = true;
                    }

                    // calculate shortest of last few readings
                    if (dist_m < combined_dist_m) {
                        combined_dist_m = dist_m;
                        combined_angle_deg = angle_deg;
                    }
                    combined_count++;
                }
            }

            // send combined distance to object database
            if ((i+1 >= point_count) || (combined_count >= PROXIMITY_SF40C_COMBINE_READINGS)) {
                if ((combined_dist_m < INT16_MAX) && database_ready) {
                    database_push(combined_angle_deg, combined_dist_m, _last_distance_received_ms, current_pos,body_to_ned);
                }
                combined_count = 0;
                combined_dist_m = INT16_MAX;
            }
        }
        break;
    }
    case MessageID::MOTOR_STATE:
        if (_msg.payload_recv == 1) {
            _sensor_state.motor_state = (MotorState)_msg.payload[0];
        }
        break;
    case MessageID::OUTPUT_RATE:
        if (_msg.payload_recv == 1) {
            _sensor_state.output_rate = _msg.payload[0];
        }
        break;

    // unsupported messages
    case MessageID::PRODUCT_NAME:
    case MessageID::HARDWARE_VERSION:
    case MessageID::FIRMWARE_VERSION:
    case MessageID::SERIAL_NUMBER:
    case MessageID::TEXT_MESSAGE:
    case MessageID::USER_DATA:
    case MessageID::SAVE_PARAMETERS:
    case MessageID::STAGE_FIRMWARE:
    case MessageID::COMMIT_FIRMWARE:
    case MessageID::INCOMING_VOLTAGE:
    case MessageID::LASER_FIRING:
    case MessageID::TEMPERATURE:
    case MessageID::BAUD_RATE:
    case MessageID::DISTANCE:
    case MessageID::MOTOR_VOLTAGE:
    case MessageID::FORWARD_OFFSET:
    case MessageID::REVOLUTIONS:
    case MessageID::ALARM_STATE:
    case MessageID::ALARM1:
    case MessageID::ALARM2:
    case MessageID::ALARM3:
    case MessageID::ALARM4:
    case MessageID::ALARM5:
    case MessageID::ALARM6:
    case MessageID::ALARM7:
        break;
    }
}

// convert buffer to uint32, uint16
uint32_t AP_Proximity_LightWareSF40C::buff_to_uint32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) const
{
    uint32_t leval = (uint32_t)b0 | (uint32_t)b1 << 8 | (uint32_t)b2 << 16 | (uint32_t)b3 << 24;
    return leval;
}

uint16_t AP_Proximity_LightWareSF40C::buff_to_uint16(uint8_t b0, uint8_t b1) const
{
    uint16_t leval = (uint16_t)b0 | (uint16_t)b1 << 8;
    return leval;
}
