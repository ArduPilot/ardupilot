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

/*

  DEVICE=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
  ./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=uart:$DEVICE:115200 --speedup=1

param set SERIAL5_PROTOCOL 11
param set PRX_TYPE 5
reboot

 */

#include "AP_Proximity_RPLidarA2.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>
#include <ctype.h>
#include <stdio.h>

#define RP_DEBUG_LEVEL 0

#if RP_DEBUG_LEVEL
  #include <GCS_MAVLink/GCS.h>
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

#define COMM_ACTIVITY_TIMEOUT_MS        200
#define RESET_RPA2_WAIT_MS              8
#define RESET_S1_WAIT_MS                2100
#define RESYNC_TIMEOUT                  10000

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

// update the _rp_state of the sensor
void AP_Proximity_RPLidarA2::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    get_readings();

    // check for timeout and set health status
    const uint32_t now = AP_HAL::millis();
    if (now - _last_distance_received_ms > COMM_ACTIVITY_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::NoData);
        Debug(1, "LIDAR NO DATA");
        if (now - _last_reset_ms > 10000) {
            Debug(1, "LIDAR timeout");
            send_command(RPLIDAR_CMD_RESET);
            // To-Do: ensure appropriate delay after sending reset request
            _last_reset_ms =  AP_HAL::millis();
            reset();
        }
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

void AP_Proximity_RPLidarA2::send_command(const uint8_t command)
{
    const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, command};
    _uart->write(tx_buffer, 2);
}

void AP_Proximity_RPLidarA2::reset()
{
    _state = State::RESET;
    _byte_count = 0;
}

void AP_Proximity_RPLidarA2::consume_bytes(uint16_t count)
{
    if (count > _byte_count) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    _byte_count -= count;
    if (_byte_count) {
        memmove((void*)&_payload[0], (void*)&_payload[count], _byte_count);
    }
}

bool AP_Proximity_RPLidarA2::make_first_byte_in_payload(uint8_t desired_byte)
{
    if (_byte_count == 0) {
        return false;
    }
    if (_payload[0] == desired_byte) {
        return true;
    }
    for (uint8_t i=1; i<_byte_count; i++) {
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
            //All you have to do is to wait 2.1s after reset and you can start scan mode with S1, 2ms is wrong in documentation
            if ( (frontend.get_type(state.instance) == AP_Proximity::Type::RPLidarS1) {
                //Any input in resetted state come from scan before reset, must discard
                    _uart->discard_input();
               if ((AP_HAL::millis() - _last_reset_ms) > RESET_S1_WAIT_MS ) { 
                         if (!make_first_byte_in_payload('R')) { // that's 'R' as in RPiLidar
                              return;
                         }
                         if (_byte_count < 63) {
                              return;
                         }
               }
            } else {
                if (!make_first_byte_in_payload('R')) { // that's 'R' as in RPiLidar
                    return;
                }
                if (_byte_count < 63) {
                    return;
                }
            }
#if RP_DEBUG_LEVEL
            // optionally spit out via mavlink the 63-bytes of cruft
            // that is spat out on device reset
            Debug(1, "Got RPLidar Information");
            char xbuffer[64]{};
            memcpy((void*)xbuffer, (void*)&_payload.information, 63);
            gcs().send_text(MAV_SEVERITY_INFO, "RPLidar: (%s)", xbuffer);
#endif
            // 63 is the magic number of bytes in the spewed-out
            // reset data ... so now we'll just drop that stuff on
            // the floor.
            consume_bytes(63);
            send_command(RPLIDAR_CMD_SCAN);
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
                RPLIDAR_PREAMBLE, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81
            };
            static const _descriptor HEALTH_DESCRIPTOR[] {
                RPLIDAR_PREAMBLE, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06
            };
            Debug(2,"LIDAR descriptor found");
            if (memcmp((void*)&_payload[0], SCAN_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SCAN_DATA;
            } else if (memcmp((void*)&_payload[0], HEALTH_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_HEALTH;
            } else {
                // unknown descriptor.  Ignore it.
            }
            consume_bytes(sizeof(_descriptor));
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
        }
    }
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

    const float angle_sign = (frontend.get_orientation(state.instance) == 1) ? -1.0f : 1.0f;
    const float angle_deg = wrap_360(_payload.sensor_scan.angle_q6/64.0f * angle_sign + frontend.get_yaw_correction(state.instance));
    const float distance_m = (_payload.sensor_scan.distance_q2/4000.0f);
#if RP_DEBUG_LEVEL >= 2
    const float quality = _payload.sensor_scan.quality;
    Debug(2, "                                       D%02.2f A%03.1f Q%02d", distance_m, angle_deg, quality);
#endif
    _last_distance_received_ms = AP_HAL::millis();
    if (!ignore_reading(angle_deg, distance_m)) {
        const AP_Proximity_Boundary_3D::Face face = boundary.get_face(angle_deg);

        if (face != _last_face) {
            // distance is for a new face, the previous one can be updated now
            if (_last_distance_valid) {
                boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m);
            } else {
                // reset distance from last face
                boundary.reset_face(face);
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
    // health issue if status is "3" ->HW error
    if (_payload.sensor_health.status == 3) {
        Debug(1, "LIDAR Error");
    }
    Debug(1, "LIDAR Healthy");
}

#endif // HAL_PROXIMITY_ENABLED
