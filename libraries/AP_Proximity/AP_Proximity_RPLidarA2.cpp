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

    // request device info 3sec after reset
    // required for S1 support that sends only 9 bytes after a reset (A1,A2 send 63)
    uint32_t now_ms = AP_HAL::millis();
    if ((_state == State::RESET) && (now_ms - _last_reset_ms > 3000)) {
        send_request_for_device_info();
        _state = State::AWAITING_RESPONSE;
        _byte_count = 0;
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

void AP_Proximity_RPLidarA2::reset_rplidar()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    Debug(1, "LIDAR reset");
    // To-Do: ensure delay of 8m after sending reset request
    _last_reset_ms =  AP_HAL::millis();
    reset();
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
    uint8_t payload[payload_size] {};

    uint8_t checksum = 0;
    checksum ^= RPLIDAR_PREAMBLE;
    checksum ^= cmd;
    checksum ^= payload_size;
    for (uint8_t i = 0; i < sizeof(payload); i++) {
        checksum ^= payload[i];
    }

    // 2 (preamble + cmd) + 1 (payload_size field) + payload_size + 1 (checksum)
    uint8_t tx_buffer[2 + 1 + payload_size + 1];
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

void AP_Proximity_RPLidarA2::consume_bytes(uint16_t count)
{
    if (count > _byte_count) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        _byte_count = 0;
        return;
    }
    _byte_count -= count;
    if (_byte_count) {
        memmove((void*)&_payload[0], (void*)&_payload[count], _byte_count);
    }
}

void AP_Proximity_RPLidarA2::reset()
{
    _state = State::RESET;
    _byte_count = 0;
    _sync_error = 0;
    _use_dense_express = false;
    _express_stream_len = 0;
}

bool AP_Proximity_RPLidarA2::make_first_byte_in_payload(uint8_t desired_byte)
{
    if (_byte_count == 0) {
        return false;
    }
    if (_payload[0] == desired_byte) {
        return true;
    }
    for (auto i=1; i<_byte_count; i++) {
        if (_payload[i] == desired_byte) {
            consume_bytes(i);
            return true;
        }
    }
    // just not in our buffer.  Throw everything away:
    _byte_count = 0;
    return false;
}

void AP_Proximity_RPLidarA2::get_readings()
{
    Debug(2, "             CURRENT STATE: %u ", (unsigned)_state);
    const uint32_t nbytes = _uart->available();
    if (nbytes == 0) {
        return;
    }
    const uint32_t bytes_to_read = MIN(nbytes, sizeof(_payload)-_byte_count);
    if (bytes_to_read == 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        reset();
        return;
    }
    const uint32_t bytes_read = _uart->read(&_payload[_byte_count], bytes_to_read);
    if (bytes_read == 0) {
        // this is bad; we were told there were bytes available
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        reset();
        return;
    }
    _byte_count += bytes_read;

    uint32_t previous_loop_byte_count = UINT32_MAX;
    while (_byte_count) {
        if (_byte_count >= previous_loop_byte_count) {
            // this is a serious error, we should always consume some
            // bytes.  Avoid looping forever.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            _uart = nullptr;
            return;
        }
        previous_loop_byte_count = _byte_count;

        switch(_state){
        case State::RESET: {
            // looking for 0x52 at start of buffer; the 62 following
            // bytes are "information"
            if (!make_first_byte_in_payload('R')) { // that's 'R' as in RPiLidar
                return;
            }
            if (_byte_count < 63) {
                return;
            }
#if RP_DEBUG_LEVEL
            // optionally spit out via mavlink the 63-bytes of cruft
            // that is spat out on device reset
            Debug(1, "Got RPLidar Information");
            char xbuffer[64]{};
            memcpy((void*)xbuffer, (void*)&_payload.information, 63);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar: (%s)", xbuffer);
#endif
            // 63 is the magic number of bytes in the spewed-out
            // reset data ... so now we'll just drop that stuff on
            // the floor.
            consume_bytes(63);
            send_request_for_device_info();
            _state = State::AWAITING_RESPONSE;
            continue;
        }
        case State::AWAITING_RESPONSE:
            if (_payload[0] != RPLIDAR_PREAMBLE) {
                // this is a protocol error.  Reset.
                reset();
                return;
            }

            // descriptor packet has 7 byte in total
            if (_byte_count < sizeof(_descriptor)) {
                return;
            }
            // identify the payload data after the descriptor
            static const _descriptor SCAN_DATA_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81 }
            };
            static const _descriptor HEALTH_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06 }
            };
            static const _descriptor DEVICE_INFO_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04 }
            };
            static const _descriptor EXPRESS_DATA_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x54, 0x00, 0x00, 0x40, 0x85 }
            };
            Debug(2,"LIDAR descriptor found");
            if (memcmp((void*)&_payload[0], SCAN_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SCAN_DATA;
            } else if (memcmp((void*)&_payload[0], DEVICE_INFO_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_DEVICE_INFO;
            } else if (memcmp((void*)&_payload[0], HEALTH_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_HEALTH;
            } else if (_use_dense_express && memcmp((void*)&_payload[0], EXPRESS_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_EXPRESS_DATA;
                _express_stream_len = 0;
                _sync_error = 0;
            } else {
                // unknown descriptor.  Ignore it.
            }
            consume_bytes(sizeof(_descriptor));
            break;

        case State::AWAITING_DEVICE_INFO:
            if (_byte_count < sizeof(_payload.device_info)) {
                return;
            }
            parse_response_device_info();
            consume_bytes(sizeof(_payload.device_info));
            break;

        case State::AWAITING_SCAN_DATA:
            if (_byte_count < sizeof(_payload.sensor_scan)) {
                return;
            }
            parse_response_data();
            consume_bytes(sizeof(_payload.sensor_scan));
            break;

        case State::AWAITING_HEALTH:
            if (_byte_count < sizeof(_payload.sensor_health)) {
                return;
            }
            parse_response_health();
            consume_bytes(sizeof(_payload.sensor_health));
            break;

        case State::AWAITING_EXPRESS_DATA:
            handle_express_data();
            break;
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
        Debug(1, "Unknown device (%u)", _payload.device_info.model);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar %s hw=%u fw=%u.%u", device_type, _payload.device_info.hardware, _payload.device_info.firmware_minor, _payload.device_info.firmware_major);

    // enable Dense EXPRESS path
    _use_dense_express = (model == Model::S2);
    if (_use_dense_express) {
        _express_stream_len = 0;
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
        Debug(1, "       OUT OF SYNC");
        // on first revolution bit 1 = 1, bit 2 = 0 of the first byte
        if ((_payload[0] & 0x03) == 0x01) {
            _sync_error = 0;
            Debug(1, "                  RESYNC");
        } else {
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
    // health issue if status is "3" ->HW error
    if (_payload.sensor_health.status == 3) {
        Debug(1, "LIDAR Error");
    }
    Debug(1, "LIDAR Healthy");
}

// verify Dense capsulated (Express) block checksum
bool AP_Proximity_RPLidarA2::verify_cabin_checksum(const uint8_t *buf, size_t len)
{
    if (buf == nullptr) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = sizeof(_express_header); i < len; i++) {
        checksum ^= buf[i];
    }

    const uint8_t expected_b0 = uint8_t((EXPRESS_SYNC1 << 4) | (checksum & 0x0F));
    const uint8_t expected_b1 = uint8_t((EXPRESS_SYNC2 << 4) | ((checksum >> 4) & 0x0F));

    return (buf[0] == expected_b0) && (buf[1] == expected_b1);
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
        const uint16_t dist_mm = (uint16_t)cab[0] | ((uint16_t)cab[1] << 8);
        cab += 2;
        if (dist_mm == 0) {
            continue;
        }
        const float d_m = dist_mm * 0.001f;

        if (!have_distance || d_m < min_distance_m) {
            min_distance_m = d_m;
            have_distance = true;
        }
    }

    _last_distance_received_ms = AP_HAL::millis();

    if (!have_distance || ignore_reading(angle_deg, min_distance_m)) {
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

void AP_Proximity_RPLidarA2::handle_express_data()
{
    auto ensure_space = [&](uint16_t need) {
        if (_express_stream_len + need <= EXPRESS_STREAM_BUFFER_SIZE) {
            return;
        }
        const uint16_t keep = EXPRESS_STREAM_BUFFER_SIZE / 2;
        memmove(_express_stream, _express_stream + (_express_stream_len - keep), keep);
        _express_stream_len = keep;
        _sync_error = 0;
        Debug(1, "EXPRESS stream overflow, dropping old data");
    };

    uint16_t bytes_read_total = 0;

    if (_byte_count > 0) {
        ensure_space(_byte_count);
        const uint16_t space = EXPRESS_STREAM_BUFFER_SIZE - _express_stream_len;
        const uint16_t to_copy = MIN(space, _byte_count);
        if (to_copy > 0) {
            memcpy(_express_stream + _express_stream_len, (const uint8_t *)&_payload[0], to_copy);
            _express_stream_len += to_copy;
        }
        _byte_count = 0;
    }

    while (_uart->available() && bytes_read_total < EXPRESS_MAX_BYTES_CONSUME) {
        const uint16_t remaining = uint16_t(EXPRESS_MAX_BYTES_CONSUME - bytes_read_total);
        uint16_t space = EXPRESS_STREAM_BUFFER_SIZE - _express_stream_len;
        if (space == 0) {
            ensure_space(1);
            space = EXPRESS_STREAM_BUFFER_SIZE - _express_stream_len;
            if (space == 0) {
                break;
            }
        }
        const uint16_t to_read = MIN(space, remaining);

        if (to_read == 0) {
            break;
        }

        const uint16_t n = _uart->read(_express_stream + _express_stream_len, to_read);
        if (n == 0) {
            break;
        }

        _express_stream_len += n;
        bytes_read_total += n;
    }

    // scan the stream buffer for valid headers and process full blocks
    uint16_t idx = 0;

    while (_express_stream_len >= idx + 2) {
        // search for a header candidate (upper nibbles 0xA / 0x5)
        while (idx + 1 < _express_stream_len) {
            const uint8_t b0 = _express_stream[idx];
            const uint8_t b1 = _express_stream[idx + 1];
            if ((b0 >> 4) == EXPRESS_SYNC1 && (b1 >> 4) == EXPRESS_SYNC2) {
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
            if (_express_stream_len > 0) {
                _express_stream[0] = _express_stream[_express_stream_len - 1];
                _express_stream_len = 1;
            } else {
                _express_stream_len = 0;
            }
            _sync_error = 0;
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
}

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
