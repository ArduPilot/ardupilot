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
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
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
float AP_Proximity_RPLidarA2::distance_max() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
        return 8.0f;
    case Model::A2:
        return 16.0f;
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
float AP_Proximity_RPLidarA2::distance_min() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
    case Model::A2:
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
}

// set Lidar into SCAN mode
void AP_Proximity_RPLidarA2::send_scan_mode_request()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_SCAN};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent scan mode request");
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
            send_request_for_device_info();
            _state = State::AWAITING_RESPONSE;
            continue;
        }
        case State::AWAITING_RESPONSE:
            // descriptor packet has 7 byte in total
            if (_uart->available() < sizeof(_descriptor)) {
                return;
            }
            _uart->read(&_payload[0], sizeof(_descriptor));
            _bytes_read += sizeof(_descriptor);

            if (_payload[0] != RPLIDAR_PREAMBLE) {
                Debug(1, "protocol error");
                // this is a protocol error do a reset.
                reset_rplidar();
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
            Debug(2,"LIDAR descriptor found");
            if (memcmp((void*)&_payload[0], SCAN_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SCAN_DATA;
            } else if (memcmp((void*)&_payload[0], DEVICE_INFO_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_DEVICE_INFO;
            } else if (memcmp((void*)&_payload[0], HEALTH_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_HEALTH;
            } else {
                // unknown descriptor.  Ignore it.
            }
            break;

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
    send_scan_mode_request();
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
        if (distance_m > distance_min()) {
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

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
