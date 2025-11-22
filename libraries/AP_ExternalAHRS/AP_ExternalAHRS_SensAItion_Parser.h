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
  SensAItion Protocol Parser
  Handles binary protocol parsing (state machine, packet validation, checksums)
  Separated from ArduPilot state management for testability
*/

#pragma once

#include <stdint.h>
#include <stddef.h>

#include <AP_Math/AP_Math.h>

class AP_ExternalAHRS_SensAItion_Parser
{
public:
    // Protocol constant
    static constexpr int MAX_PACKET_SIZE = 256;

    // Specifies which type of data is read from the sensor
    enum class ConfigMode {
        CONFIG_MODE_IMU = 0,   // IMU only mode (accel, gyro, mag, baro, temp)
        CONFIG_MODE_AHRS = 1   // AHRS mode (IMU + quaternion)
    };

    enum class MeasurementType {
        UNINITIALIZED = 0, // No measurements are initialized
        IMU = 1, // IMU measurements are initialized
        AHRS = 2, // IMU and AHRS measurements are intialized
    };

    // Represents one sample from the sensor
    struct Measurement {
        MeasurementType type;

        // Measurements that are available from the IMU sensor:
        Vector3f acceleration_mss; // (m/s^2)
        Vector3f angular_velocity_rads; // (rad/s)
        float temperature_degc; // (degrees C)
        Vector3f magnetic_field_mgauss; // mgauss
        float air_pressure_p; // Pascal

        // Only available from the AHRS sensor:
        Quaternion orientation;
    };

    // Constructor
    AP_ExternalAHRS_SensAItion_Parser(ConfigMode mode = ConfigMode::CONFIG_MODE_IMU);

    /*
    Parse incoming byte stream and extract any complete SensAItion packets.

    data: Pointer to first input byte
    data_size: Size of input buffer
    measurement: Contains data on success, otherwise type is set to UNINITIALIZED
    */
    void parse_bytes(const uint8_t* data, size_t data_size, Measurement& measurement);

    // Get number of valid packets received
    uint32_t get_valid_packets() const
    {
        return valid_packets;
    }

    // Get number of invalid packets received
    // (Byte sequences that start with the header byte and have the right
    // number of bytes, but do not form a valid packet.)
    uint32_t get_parse_errors() const
    {
        return parse_errors;
    }

private:
    // Protocol constants
    static constexpr uint8_t HEADER_BYTE = 0xFA;
    static constexpr size_t PACKET_SIZE_IMU = 38;   // 36 data bytes + header + checksum
    static constexpr size_t PACKET_SIZE_AHRS = 54;  // 52 data bytes + header + checksum

    // Packet parsing state machine
    enum class ParseState {
        LOOKING_FOR_HEADER,
        COLLECTING_PACKET
    };

    ConfigMode config_mode;
    ParseState parse_state = ParseState::LOOKING_FOR_HEADER;

    uint8_t packet_buffer[MAX_PACKET_SIZE];
    uint16_t packet_buffer_len = 0;

    // Statistics
    uint32_t valid_packets = 0;
    uint32_t parse_errors = 0;

    // Handles invalid packet, fix packet buffer, package size etc
    void handle_invalid_package(void);

    // Update parser with one byte, return true if this was the last byte of a valid packet
    bool parse_single_byte(uint8_t byte);

    // Returns true if packet_buffer contains a packet with valid checksum
    bool buffer_contains_valid_packet() const;

    // Extracts sensor data from (an assumed valid) packet in the packet_buffer
    void extract_sensor_data(Measurement& measurement) const;

    uint8_t calculate_xor_checksum(const uint8_t* data, size_t start, size_t length) const;

    size_t expected_packet_size() const
    {
        return (config_mode == ConfigMode::CONFIG_MODE_IMU) ? PACKET_SIZE_IMU : PACKET_SIZE_AHRS;
    }
};
