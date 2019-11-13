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
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include "AP_Proximity_LightWareSF40C.h"

extern const AP_HAL::HAL& hal;

#define PROXIMITY_SF40C_HEADER                  0xAA
#define PROXIMITY_SF40C_DESIRED_OUTPUT_RATE     3
#define PROXIMITY_SF40C_UART_RX_SPACE           1280

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_LightWareSF40C::AP_Proximity_LightWareSF40C(AP_Proximity &_frontend,
                                                         AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    if (_uart != nullptr) {
        // start uart with larger receive buffer
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0), PROXIMITY_SF40C_UART_RX_SPACE, 0);
    }
}

// detect if a Lightware proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_LightWareSF40C::detect()
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0) != nullptr;
}

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
    // initialise sectors
    if (!_sector_initialised) {
        init_sectors();
    }

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

// initialise sector angles using user defined ignore areas
void AP_Proximity_LightWareSF40C::init_sectors()
{
    // use defaults if no ignore areas defined
    uint8_t ignore_area_count = get_ignore_area_count();
    if (ignore_area_count == 0) {
        _sector_initialised = true;
        return;
    }

    uint8_t sector = 0;

    for (uint8_t i=0; i<ignore_area_count; i++) {

        // get ignore area info
        uint16_t ign_area_angle;
        uint8_t ign_area_width;
        if (get_ignore_area(i, ign_area_angle, ign_area_width)) {

            // calculate how many degrees of space we have between this end of this ignore area and the start of the end
            int16_t start_angle, end_angle;
            get_next_ignore_start_or_end(1, ign_area_angle, start_angle);
            get_next_ignore_start_or_end(0, start_angle, end_angle);
            int16_t degrees_to_fill = wrap_360(end_angle - start_angle);

            // divide up the area into sectors
            while ((degrees_to_fill > 0) && (sector < PROXIMITY_SECTORS_MAX)) {
                uint16_t sector_size;
                if (degrees_to_fill >= 90) {
                    // set sector to maximum of 45 degrees
                    sector_size = 45;
                } else if (degrees_to_fill > 45) {
                    // use half the remaining area to optimise size of this sector and the next
                    sector_size = degrees_to_fill / 2.0f;
                } else  {
                    // 45 degrees or less are left so put it all into the next sector
                    sector_size = degrees_to_fill;
                }
                // record the sector middle and width
                _sector_middle_deg[sector] = wrap_360(start_angle + sector_size / 2.0f);
                _sector_width_deg[sector] = sector_size;

                // move onto next sector
                start_angle += sector_size;
                sector++;
                degrees_to_fill -= sector_size;
            }
        }
    }

    // set num sectors
    _num_sectors = sector;

    // re-initialise boundary because sector locations have changed
    init_boundary();

    // record success
    _sector_initialised = true;
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
// returns true if a message has been successfully parsed
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
        Location current_loc;
        float current_vehicle_bearing;
        const bool database_ready = database_prepare_for_push(current_loc, current_vehicle_bearing);

        // process each point
        const float angle_inc_deg = (1.0f / point_total) * 360.0f;
        const float angle_sign = (frontend.get_orientation(state.instance) == 1) ? -1.0f : 1.0f;
        const float angle_correction = frontend.get_yaw_correction(state.instance);
        const uint16_t dist_min_cm = distance_min() * 100;
        const uint16_t dist_max_cm = distance_max() * 100;
        for (uint16_t i = 0; i < point_count; i++) {
            const uint16_t idx = 14 + (i * 2);
            const int16_t dist_cm = (int16_t)buff_to_uint16(_msg.payload[idx], _msg.payload[idx+1]);
            const float angle_deg = wrap_360((point_start_index + i) * angle_inc_deg * angle_sign + angle_correction);
            uint8_t sector;
            if (convert_angle_to_sector(angle_deg, sector)) {
                if (sector != _last_sector) {
                    // update boundary used for avoidance
                    if (_last_sector != UINT8_MAX) {
                        update_boundary_for_sector(_last_sector, false);
                    }
                    _last_sector = sector;
                    // init for new sector
                    _distance[sector] = INT16_MAX;
                    _distance_valid[sector] = false;
                }
                if ((dist_cm >= dist_min_cm) && (dist_cm <= dist_max_cm)) {
                    // use shortest valid distance for this sector's distance
                    const float dist_m = dist_cm * 0.01f;
                    if (dist_m < _distance[sector]) {
                        _angle[sector] = angle_deg;
                        _distance[sector] = dist_m;
                        _distance_valid[sector] = true;
                    }
                    // send point to object avoidance database
                    if (database_ready) {
                        database_push(angle_deg, dist_m, _last_distance_received_ms, current_loc, current_vehicle_bearing);
                    }
                }
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
