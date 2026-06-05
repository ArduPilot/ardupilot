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
 * ArduPilot device driver for Guoke Optics COIN-D6 / CSPC M1CT_TOF 360 degree LiDAR
 *
 * Sold under multiple names including:
 *   - Guoke Optics COIN-D6
 *   - WitMotion dTOF Laser LiDAR
 *
 * Protocol reference: COIN-D6 LiDAR Data Format Standard Specification V1.0 (Guoke Optics)
 * Reference ROS driver: https://github.com/seanight-huwahuwa/coin_d6_lidar_ros2
 *
 * Wire protocol: UART @ 230400 baud, 8N1
 * Packet header: 0xAA 0x55 (little-endian 0x55AA)
 * Variable-length packets with LSN measurements per packet (typically 8-32)
 *
 * Author: Andrew Dewar
 * Based on AP_Proximity_LD06 by Adithya Patil
 */

#pragma once
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_CSPC_ENABLED

#include "AP_Proximity_Backend_Serial.h"

// COIN-D6 / CSPC M1CT_TOF specs
#define CSPC_HEADER_BYTE_1              0xAA
#define CSPC_HEADER_BYTE_2              0x55

// Fixed header is 10 bytes: PH(2) + M&T(1) + LSN(1) + FSA(2) + LSA(2) + CS(2)
#define CSPC_HEADER_LENGTH              10
// Each sample is 3 bytes: S_L, S_2nd, S_H
#define CSPC_SAMPLE_BYTES               3
// Max LSN (samples per packet) seen in field captures; allocate buffer for safe margin
#define CSPC_MAX_SAMPLES                40
#define CSPC_MAX_PACKET_LENGTH          (CSPC_HEADER_LENGTH + CSPC_MAX_SAMPLES * CSPC_SAMPLE_BYTES)

// Sensor distance range (per Guoke spec sheet)
#define MAX_READ_DISTANCE_CSPC          12.0f
#define MIN_READ_DISTANCE_CSPC          0.05f

// Timeout: if we receive no valid packet for this long, mark NoData
#define CSPC_TIMEOUT_MS                 200

// Motor start command (host -> LiDAR) per protocol spec
// Sent at startup so the LiDAR begins scanning
#define CSPC_CMD_MOTOR_START_BYTES      {0xAA, 0x55, 0xF0, 0x0F}
#define CSPC_CMD_MOTOR_START_LEN        4

class AP_Proximity_CSPC : public AP_Proximity_Backend_Serial
{
public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // Update the state of the sensor
    void update(void) override;

    // Get the max and min distances for the sensor being used
    float distance_max_m() const override { return MAX_READ_DISTANCE_CSPC; }
    float distance_min_m() const override { return MIN_READ_DISTANCE_CSPC; }

private:

    // Send the motor-start command to begin scanning
    void send_motor_start();

    // Read bytes from UART and feed the state machine
    void get_readings();

    // Decode the current accumulated packet and push readings to boundary
    void parse_packet();

    // State machine for incoming bytes
    enum class ParseState : uint8_t {
        WAIT_HEAD_1,    // looking for 0xAA
        WAIT_HEAD_2,    // looking for 0x55
        READ_HEADER,    // reading fixed-length header (bytes 2..9)
        READ_SAMPLES,   // reading LSN * 3 sample bytes
    };

    ParseState _parse_state {ParseState::WAIT_HEAD_1};
    uint8_t _buffer[CSPC_MAX_PACKET_LENGTH];
    uint16_t _byte_count;
    uint8_t _expected_samples;       // LSN parsed from header
    uint16_t _expected_packet_bytes; // CSPC_HEADER_LENGTH + LSN * 3

    // Motor start sent once at boot
    bool _motor_started;
    uint32_t _last_distance_received_ms;

    // Boundary helper to batch face updates per rotation
    AP_Proximity_Temp_Boundary _temp_boundary;
};

#endif // AP_PROXIMITY_CSPC_ENABLED
