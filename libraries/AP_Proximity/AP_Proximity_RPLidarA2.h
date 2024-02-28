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

# to connect device to SITL:
./Tools/autotest/sim_vehicle.py -v Rover --gdb --debug -A --serial5=uart:/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:115200
param set SERIAL5_PROTOCOL 11
param set SERIAL5_BAUD 115200
param set PRX1_TYPE 5
reboot

# short outer-two wires on JST plug to get it to spin

*/


#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARA2_ENABLED

#include "AP_Proximity_Backend_Serial.h"

class AP_Proximity_RPLidarA2 : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:

    enum class State {
        RESET = 56,
        AWAITING_RESPONSE,
        AWAITING_SCAN_DATA,
        AWAITING_HEALTH,
        AWAITING_DEVICE_INFO,
    } _state = State::RESET;

    // send request for something from sensor
    void send_request_for_health();
    void send_scan_mode_request();
    void send_request_for_device_info();

    void parse_response_data();
    void parse_response_health();
    void parse_response_device_info();

    void get_readings();
    void reset_rplidar();
    void reset();

    // remove bytes from read buffer:
    void consume_bytes(uint16_t count);

    uint8_t _sync_error;
    uint16_t _byte_count;

    // request related variables
    uint32_t  _last_distance_received_ms;     ///< system time of last distance measurement received from sensor
    uint32_t  _last_reset_ms;

    // face related variables
    AP_Proximity_Boundary_3D::Face _last_face;///< last face requested
    float _last_angle_deg;                    ///< yaw angle (in degrees) of _last_distance_m
    float _last_distance_m;                   ///< shortest distance for _last_face
    bool _last_distance_valid;                ///< true if _last_distance_m is valid

    struct PACKED _device_info {
        uint8_t model;
        uint8_t firmware_minor;
        uint8_t firmware_major;
        uint8_t hardware;
        uint8_t serial[16];
   };

    struct PACKED _sensor_scan {
        uint8_t startbit      : 1;            ///< on the first revolution 1 else 0
        uint8_t not_startbit  : 1;            ///< complementary to startbit
        uint8_t quality       : 6;            ///< Related the reflected laser pulse strength
        uint8_t checkbit      : 1;            ///< always set to 1
        uint16_t angle_q6     : 15;           ///< Actual heading = angle_q6/64.0 Degree
        uint16_t distance_q2  : 16;           ///< Actual Distance = distance_q2/4.0 mm
    };

    struct PACKED _sensor_health {
        uint8_t status;                       ///< status definition: 0 good, 1 warning, 2 error
        uint16_t error_code;                  ///< the related error code
    };

    struct PACKED _descriptor {
        uint8_t bytes[7];
    };

    // we don't actually *need* to store this.  If we don't, _payload
    // can be just 7 bytes, but that doesn't make for efficient
    // reading.  It also simplifies the state machine to have the read
    // buffer at least this big.  Note that we force the buffer to a
    // larger size below anyway.
    struct PACKED _rpi_information {
        uint8_t bytes[63];
    };

    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        _sensor_scan sensor_scan;
        _sensor_health sensor_health;
        _descriptor descriptor;
        _rpi_information information;
        _device_info device_info;
        uint8_t forced_buffer_size[256]; // just so we read(...) efficiently
    } _payload;
    static_assert(sizeof(_payload) >= 63, "Needed for parsing out reboot data");

    enum class Model {
        UNKNOWN,
        A1,
        A2,
        S1,
    } model = Model::UNKNOWN;

    bool make_first_byte_in_payload(uint8_t desired_byte);
};

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
