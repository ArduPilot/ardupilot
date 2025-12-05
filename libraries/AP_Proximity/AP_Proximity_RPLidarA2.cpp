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
 * ArduPilot device driver for SLAMTEC RPLIDAR A2 (16m range version)
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM RPLIDAR DATASHEET:
 *
 * https://www.slamtec.com/en/Lidar
 * http://bucket.download.slamtec.com/63ac3f0d8c859d3a10e51c6b3285fcce25a47357/LR001_SLAMTEC_rplidar_protocol_v1.0_en.pdf
 *
 * Author: Steven Josefs, IAV GmbH
 * Based on the LightWare SF40C ArduPilot device driver from Randy Mackay
 *
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARA2_ENABLED

#include "AP_Proximity_RPLidarA2.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define RP_DEBUG_LEVEL 0

#include <GCS_MAVLink/GCS.h>
#if RP_DEBUG_LEVEL
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

#define COMM_ACTIVITY_TIMEOUT_MS        200

// Commands
//-----------------------------------------

// Commands without payload and response
#define RPLIDAR_PREAMBLE               0xA5
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40

// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO    0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH  0x52

// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82

extern const AP_HAL::HAL& hal;

void AP_Proximity_RPLidarA2::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    get_readings();

    // check for timeout and set health status
    if (AP_HAL::millis() - _last_distance_received_ms > COMM_ACTIVITY_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::NoData);
        Debug(1, "LIDAR NO DATA");
        if (AP_HAL::millis() - _last_reset_ms > 10000) {
            reset_rplidar();
        }
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_max_m() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
        return 8.0f;
    case Model::A2:
        return 16.0f;
    case Model::A2M12:
    case Model::C1:
        return 12.0f;
    case Model::S1:
        return 40.0f;
    case Model::S2:
        return 50.0f;
    }
    return 0.0f;
}

// get minimum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_min_m() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
    case Model::A2:
    case Model::A2M12:
    case Model::C1:
    case Model::S1:
        return 0.2f;
    case Model::S2:
        return 0.05f;
    }
    return 0.0f;
}

// reset lidar
void AP_Proximity_RPLidarA2::reset_rplidar()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    Debug(1, "LIDAR reset");
    _last_reset_ms =  AP_HAL::millis();
    _state = State::RESET;
    _sync_error = 0;
    _express_stream_len = 0;
    _descriptor_pos = 0;
}

// set Lidar into SCAN mode
void AP_Proximity_RPLidarA2::send_scan_mode_request()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_SCAN};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent scan mode request");
}

// send EXPRESS_SCAN request for Dense mode
void AP_Proximity_RPLidarA2::send_express_scan_request()
{
    const uint8_t cmd = RPLIDAR_CMD_EXPRESS_SCAN;
    const uint8_t payload_size = 5;

    // working_mode = 0
    // For S2 this selects Dense capsulated data.
    uint8_t payload[5] {};
    payload[0] = 0; // working_mode = 0
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 0;
    payload[4] = 0;

    uint8_t checksum = 0;
    checksum ^= RPLIDAR_PREAMBLE;
    checksum ^= cmd;
    checksum ^= payload_size;
    for (uint8_t i = 0; i < sizeof(payload); i++) {
        checksum ^= payload[i];
    }

    uint8_t tx_buffer[2 + 1 + 5 + 1];
    tx_buffer[0] = RPLIDAR_PREAMBLE;
    tx_buffer[1] = cmd;
    tx_buffer[2] = payload_size;
    memcpy(&tx_buffer[3], payload, sizeof(payload));
    tx_buffer[3 + sizeof(payload)] = checksum;

    _uart->write(tx_buffer, sizeof(tx_buffer));
    Debug(1, "Sent EXPRESS (Dense) scan request");
}

// send request for sensor health
void AP_Proximity_RPLidarA2::send_request_for_health()                                    //not called yet
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent health request");
}

// send request for device information
void AP_Proximity_RPLidarA2::send_request_for_device_info()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_INFO};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent device information request");
}

void AP_Proximity_RPLidarA2::get_readings()
{
    uint16_t _bytes_read = 0;
    Debug(2, "CURRENT STATE: %u ", (unsigned)_state);
    while (_uart->available() && _bytes_read < MAX_BYTES_CONSUME) {
        switch(_state){
        case State::RESET: {
            if (AP_HAL::millis() - _last_reset_ms < 1000) {
                return;
            }
            _uart->discard_input();
            _express_stream_len = 0;
            _descriptor_pos = 0;
            send_request_for_device_info();
            _state = State::AWAITING_RESPONSE;
            continue;
        }

        case State::AWAITING_RESPONSE: {
            // stream-based descriptor parser:
            // - search for preamble 0xA5
            // - then accumulate 7-byte descriptor
            // - do not hard-reset on stray bytes; resynchronise instead
            while (_uart->available() && _bytes_read < MAX_BYTES_CONSUME) {
                const uint8_t b = _uart->read();
                _bytes_read++;

                if (_descriptor_pos == 0) {
                    if (b != RPLIDAR_PREAMBLE) {
                        Debug(2, "skipping stray byte 0x%02X in descriptor", unsigned(b));
                        continue;
                    }
                    _payload.descriptor.bytes[0] = b;
                    _descriptor_pos = 1;
                    continue;
                }

                // fill remaining descriptor bytes
                _payload.descriptor.bytes[_descriptor_pos++] = b;

                // still not enough for a full descriptor
                if (_descriptor_pos < sizeof(_descriptor)) {
                    continue;
                }

                // full descriptor received
                _descriptor_pos = 0;

                if (_payload.descriptor.bytes[1] != 0x5A) {
                    Debug(2, "unknown descriptor header2=0x%02X",
                          unsigned(_payload.descriptor.bytes[1]));
                    continue;
                }

                const uint8_t data_type = _payload.descriptor.bytes[6];
                Debug(2, "LIDAR descriptor data_type=0x%02x", (unsigned)data_type);

                switch (data_type) {
                case 0x81: // 5-byte scan data
                    _state = State::AWAITING_SCAN_DATA;
                    break;

                case 0x85: // dense capsulated express data
                    if (_use_dense_express) {
                        _state = State::AWAITING_EXPRESS_DATA;
                    }
                    break;

                case 0x04: // device info
                    _state = State::AWAITING_DEVICE_INFO;
                    break;

                case 0x06: // health
                    _state = State::AWAITING_HEALTH;
                    break;

                default:
                    // unknown descriptor type; ignore and keep consuming
                    Debug(1, "Unknown descriptor data_type=0x%02x", (unsigned)data_type);
                    break;
                }
                break;
            }
            break;
        }

        case State::AWAITING_DEVICE_INFO:
            if (_uart->available() < sizeof(_payload.device_info)) {
                return;
            }
            _uart->read(&_payload[0], sizeof(_payload.device_info));
            _bytes_read += sizeof(_payload.device_info);
            parse_response_device_info();
            break;

        case State::AWAITING_SCAN_DATA:
            if (_uart->available() < sizeof(_payload.sensor_scan)) {
                return;
            }
            _uart->read(&_payload[0], sizeof(_payload.sensor_scan));
            _bytes_read += sizeof(_payload.sensor_scan);
            parse_response_data();
            break;

        case State::AWAITING_HEALTH:
            if (_uart->available() < sizeof(_payload.sensor_health)) {
                return;
            }
            _uart->read(&_payload[0], sizeof(_payload.sensor_health));
            _bytes_read += sizeof(_payload.sensor_health);
            parse_response_health();
            break;

        case State::AWAITING_EXPRESS_DATA: {
            // 1) append new bytes from UART into the Express/Dense stream buffer
            while (_uart->available() && _bytes_read < MAX_BYTES_CONSUME) {
                if (_express_stream_len >= EXPRESS_STREAM_BUFFER_SIZE) {
                    // prevent buffer overflow: keep the newest half of the data
                    const uint16_t keep = EXPRESS_STREAM_BUFFER_SIZE / 2;
                    memmove(_express_stream,
                            _express_stream + (_express_stream_len - keep),
                            keep);
                    _express_stream_len = keep;
                    Debug(1, "EXPRESS stream overflow, dropping old data");
                }

                const uint16_t space = EXPRESS_STREAM_BUFFER_SIZE - _express_stream_len;
                const uint16_t to_read = MIN(space, uint16_t(MAX_BYTES_CONSUME - _bytes_read));

                if (to_read == 0) {
                    break;
                }

                const uint16_t n = _uart->read(_express_stream + _express_stream_len, to_read);
                if (n == 0) {
                    break;
                }

                _express_stream_len += n;
                _bytes_read += n;
            }

            // 2) scan the stream buffer for valid headers and process full blocks
            uint16_t idx = 0;

            while (_express_stream_len >= idx + 2) {
                // search for a header candidate (upper nibbles 0xA / 0x5)
                while (idx + 1 < _express_stream_len) {
                    const uint8_t b0 = _express_stream[idx];
                    const uint8_t b1 = _express_stream[idx + 1];
                    if ((b0 >> 4) == 0x0A && (b1 >> 4) == 0x05) {
                        // header candidate found
                        break;
                    }
                    idx++;
                }

                if (idx + 1 >= _express_stream_len) {
                    // no header candidate found, drop all accumulated data
                    if (_express_stream_len > 0) {
                        Debug(1, "EXPRESS: no header candidate, dropping %u bytes", unsigned(_express_stream_len));
                    }
                    _express_stream_len = 0;
                    return;
                }

                // idx now points to a header candidate
                if (_express_stream_len - idx < EXPRESS_BLOCK_SIZE) {
                    // not enough bytes for a full block
                    // move remaining bytes to the front and wait for more data
                    if (idx > 0) {
                        memmove(_express_stream,
                                _express_stream + idx,
                                _express_stream_len - idx);
                        _express_stream_len -= idx;
                    }
                    return;
                }

                // pointer to the candidate block
                uint8_t *blk = _express_stream + idx;

                if (!verify_cabin_checksum(blk, EXPRESS_BLOCK_SIZE)) {
                    // upper nibbles match but checksum is invalid
                    // advance by one byte and try to resynchronise
                    Debug(1, "EXPRESS/DENSE checksum error");
                    _sync_error++;

                    if (_sync_error > 10) {
                        reset_rplidar();
                        _express_stream_len = 0;
                        return;
                    }

                    idx++;
                    continue;
                }

                // valid block found
                _sync_error = 0;
                parse_response_express(blk);

                // consume this block and look for the next one
                idx += EXPRESS_BLOCK_SIZE;
            }

            // compact any remaining unprocessed bytes to the front of the buffer
            if (idx > 0 && idx <= _express_stream_len) {
                const uint16_t remaining = _express_stream_len - idx;
                if (remaining > 0) {
                    memmove(_express_stream,
                            _express_stream + idx,
                            remaining);
                }
                _express_stream_len = remaining;
            }

            return;
        }
        }
    }
}

void AP_Proximity_RPLidarA2::parse_response_device_info()
{
    Debug(1, "Received DEVICE_INFO");
    const char *device_type = "UNKNOWN";
    switch (_payload.device_info.model) {
    case 0x18:
        model = Model::A1;
        device_type = "A1";
        break;
    case 0x28:
        model = Model::A2;
        device_type = "A2";
        break;
    case 0x2C:
        model = Model::A2M12;
        device_type = "A2M12";
        break;
    case 0x41:
        model=Model::C1;
        device_type="C1";
        break;
    case 0x61:
        model = Model::S1;
        device_type = "S1";
        break;
    case 0x71:
        model = Model::S2;
        device_type = "S2";
        break;
    default:
        model = Model::UNKNOWN;
        Debug(1, "Unknown device (%u)", _payload.device_info.model);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar %s hw=%u fw=%u.%u", device_type, _payload.device_info.hardware, _payload.device_info.firmware_minor, _payload.device_info.firmware_major);

    // enable Dense EXPRESS path
    _use_dense_express = (model == Model::S2);
    if (_use_dense_express) {
        send_express_scan_request();
    } else {
        send_scan_mode_request();
    }
    _state = State::AWAITING_RESPONSE;
}

void AP_Proximity_RPLidarA2::parse_response_data()
{
    if (_sync_error) {
        // out of 5-byte sync mask -> catch new revolution
        Debug(1, "OUT OF SYNC");
        // on first revolution bit 1 = 1, bit 2 = 0 of the first byte
        if ((_payload[0] & 0x03) == 0x01) {
            _sync_error = 0;
            Debug(1, "RESYNC");
        } else {
            Debug(1, "NO RESYNC");
            return;
        }
    }
    Debug(2, "UART %02x %02x%02x %02x%02x", _payload[0], _payload[2], _payload[1], _payload[4], _payload[3]); //show HEX values
    // check if valid SCAN packet: a valid packet starts with startbits which are complementary plus a checkbit in byte+1
    if (!((_payload.sensor_scan.startbit == !_payload.sensor_scan.not_startbit) && _payload.sensor_scan.checkbit)) {
        Debug(1, "Invalid Payload");
        _sync_error++;
        return;
    }

    const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;
    const float angle_deg = wrap_360(_payload.sensor_scan.angle_q6/64.0f * angle_sign + params.yaw_correction);
    const float distance_m = (_payload.sensor_scan.distance_q2/4000.0f);
#if RP_DEBUG_LEVEL >= 2
    const float quality = _payload.sensor_scan.quality;
    Debug(2, "   D%02.2f A%03.1f Q%0.2f", distance_m, angle_deg, quality);
#endif
    _last_distance_received_ms = AP_HAL::millis();
    if (!ignore_reading(angle_deg, distance_m)) {
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);

        if (face != _last_face) {
            // distance is for a new face, the previous one can be updated now
            if (_last_distance_valid) {
                frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
            } else {
                // reset distance from last face
                frontend.boundary.reset_face(face, state.instance);
            }

            // initialize the new face
            _last_face = face;
            _last_distance_valid = false;
        }
        if (distance_m > distance_min_m()) {
            // update shortest distance
            if (!_last_distance_valid || (distance_m < _last_distance_m)) {
                _last_distance_m = distance_m;
                _last_distance_valid = true;
                _last_angle_deg = angle_deg;
            }
            // update OA database
            database_push(_last_angle_deg, _last_distance_m);
        }
    }
}

void AP_Proximity_RPLidarA2::parse_response_health()
{
    // health issue if status is "3" -> HW error
    if (_payload.sensor_health.status == 3) {
        Debug(1, "LIDAR Error");
    }
    Debug(1, "LIDAR Healthy");
}

// verify Dense capsulated (Express) block checksum
bool AP_Proximity_RPLidarA2::verify_cabin_checksum(const uint8_t *buf, size_t len)
{
    if (buf == nullptr || len < 2) {
        return false;
    }

    const uint8_t b0 = buf[0];
    const uint8_t b1 = buf[1];

    // upper nibbles must match sync1=0xA, sync2=0x5
    if ((b0 >> 4) != 0x0A || (b1 >> 4) != 0x05) {
        Debug(1, "EXPRESS header mismatch: len=%u b0=0x%02X b1=0x%02X", unsigned(len), unsigned(b0), unsigned(b1));
        return false;
    }

    // checksum: XOR of bytes [2..len-1]
    uint8_t checksum = 0;
    for (size_t i = 2; i < len; i++) {
        checksum ^= buf[i];
    }

    // sync1/sync2 and checksum nibbles (dense format)
    const uint8_t sync1 = 0x0A;
    const uint8_t sync2 = 0x05;

    const uint8_t encoded_b0 = uint8_t((sync1 << 4) | (checksum & 0x0F));
    const uint8_t encoded_b1 = uint8_t((sync2 << 4) | ((checksum >> 4) & 0x0F));

    const bool ok = (b0 == encoded_b0) && (b1 == encoded_b1);
#if RP_DEBUG_LEVEL
    if (!ok) {
        Debug(1, "EXPRESS checksum error: len=%u b0=0x%02X b1=0x%02X chk=0x%02X enc0=0x%02X enc1=0x%02X",
              unsigned(len), unsigned(b0), unsigned(b1),
              unsigned(checksum), unsigned(encoded_b0), unsigned(encoded_b1));
    } else {
        Debug(2, "EXPRESS checksum ok: chk=0x%02X", unsigned(checksum));
    }
#endif
    return ok;
}

// parse Dense capsulated Express block
void AP_Proximity_RPLidarA2::parse_response_express(const uint8_t *buf)
{
    if (buf == nullptr) {
        return;
    }

    // start_angle_q6: 16-bit, Q6 format at bytes [2..3]
    const uint16_t start_angle_q6 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    const float start_angle_deg_raw = start_angle_q6 / 64.0f; // Q6 -> degrees

    const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;
    const float angle_deg = wrap_360(start_angle_deg_raw * angle_sign + params.yaw_correction);

    // cabins: 40 cabins * 2-byte distance (mm) starting at buf[4]
    const uint8_t *cab = buf + 4;
    static constexpr int NUM_CABINS = 40;

    bool  have_distance = false;
    float min_distance_m = 0.0f;

    for (int i = 0; i < NUM_CABINS; i++) {
        const uint16_t dist_mm =
            (uint16_t)cab[0] | ((uint16_t)cab[1] << 8);
        cab += 2;

        if (dist_mm == 0) {
            // zero means invalid or out-of-range
            continue;
        }

        const float d_m = dist_mm * 0.001f; // mm -> m
        if (d_m <= 0.0f) {
            continue;
        }

        if (!have_distance || d_m < min_distance_m) {
            min_distance_m = d_m;
            have_distance = true;
        }
    }

    if (!have_distance) {
        return;
    }

    _last_distance_received_ms = AP_HAL::millis();

    if (ignore_reading(angle_deg, min_distance_m)) {
        return;
    }

    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);

    if (face != _last_face) {
        // distance is for a new face, the previous one can be updated now
        if (_last_distance_valid) {
            frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
        } else {
            // reset distance from last face
            frontend.boundary.reset_face(face, state.instance);
        }

        // initialize the new face
        _last_face = face;
        _last_distance_valid = false;
    }

    if (min_distance_m > distance_min_m()) {
        // update shortest distance
        if (!_last_distance_valid || (min_distance_m < _last_distance_m)) {
            _last_distance_m = min_distance_m;
            _last_distance_valid = true;
            _last_angle_deg = angle_deg;
        }
        // update OA database
        database_push(_last_angle_deg, _last_distance_m);
    }
}

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
