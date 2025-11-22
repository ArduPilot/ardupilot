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

#include <stdio.h>
#include "AP_ExternalAHRS_SensAItion_Parser.h"

AP_ExternalAHRS_SensAItion_Parser::AP_ExternalAHRS_SensAItion_Parser(ConfigMode mode) :
    config_mode(mode)
{
}

void AP_ExternalAHRS_SensAItion_Parser::parse_bytes(const uint8_t* data, size_t data_size, Measurement& measurement)
{
    measurement.type = MeasurementType::UNINITIALIZED;

    for (size_t i = 0; i < data_size; i++) {
        if (parse_single_byte(data[i])) {
            // Will change to a valid measurement type
            extract_sensor_data(measurement);
        }
    }
}

void AP_ExternalAHRS_SensAItion_Parser::handle_invalid_package(void)
{
    // Check if any header is in buffer
    uint8_t *p = (uint8_t *)memchr(&packet_buffer[1], HEADER_BYTE, packet_buffer_len - 1);
    if (p) {
        // Found header, copy buffer that header is first, keep collecting data
        packet_buffer_len = packet_buffer_len - (p - packet_buffer);
        memmove(&packet_buffer[0], p, packet_buffer_len);
    } else {
        // No header found, start looking for headear
        parse_state = ParseState::LOOKING_FOR_HEADER;
    }
    parse_errors++;
}

bool AP_ExternalAHRS_SensAItion_Parser::parse_single_byte(uint8_t byte)
{
    switch (parse_state) {
    case ParseState::LOOKING_FOR_HEADER:
        if (byte == HEADER_BYTE) {
            packet_buffer_len = 0;
            packet_buffer[packet_buffer_len++] = byte;
            parse_state = ParseState::COLLECTING_PACKET;
        }
        break;

    case ParseState::COLLECTING_PACKET:
        packet_buffer[packet_buffer_len++] = byte;

        if (packet_buffer_len == expected_packet_size()) {
            // Validate complete packet
            if (buffer_contains_valid_packet()) {
                valid_packets++;
                parse_state = ParseState::LOOKING_FOR_HEADER;
                return true;  // Complete valid packet
            } else {
                handle_invalid_package();
            }
        } else if (packet_buffer_len >= MAX_PACKET_SIZE) {
            handle_invalid_package();
        }
        break;
    }

    return false;
}

bool AP_ExternalAHRS_SensAItion_Parser::buffer_contains_valid_packet() const
{
    // Validate packet structure
    if (packet_buffer_len != expected_packet_size() || packet_buffer[0] != HEADER_BYTE) {
        return false;
    }

    // Validate checksum
    uint8_t calculated = calculate_xor_checksum(packet_buffer, 1, packet_buffer_len - 2);
    uint8_t received = packet_buffer[packet_buffer_len - 1];

    return (calculated == received);
}

void AP_ExternalAHRS_SensAItion_Parser::extract_sensor_data(Measurement& measurement) const
{
    // Start parsing after the header byte
    const uint8_t* packet = packet_buffer + 1;

    // Extract acceleration (µg -> m/s^2)
    int32_t accel_x_ug = (packet[0]<<24)|(packet[1]<<16)|(packet[2]<<8)|packet[3];
    int32_t accel_y_ug = (packet[4]<<24)|(packet[5]<<16)|(packet[6]<<8)|packet[7];
    int32_t accel_z_ug = (packet[8]<<24)|(packet[9]<<16)|(packet[10]<<8)|packet[11];
    measurement.acceleration_mss = Vector3f(accel_x_ug, accel_y_ug, accel_z_ug) * 1e-6f * GRAVITY_MSS;

    // Extract angular velocity from gyroscope (µdeg/s -> rad/s)
    int32_t gyro_x_udegs = (packet[12]<<24)|(packet[13]<<16)|(packet[14]<<8)|packet[15];
    int32_t gyro_y_udegs = (packet[16]<<24)|(packet[17]<<16)|(packet[18]<<8)|packet[19];
    int32_t gyro_z_udegs = (packet[20]<<24)|(packet[21]<<16)|(packet[22]<<8)|packet[23];
    Vector3f gyro_degs = Vector3f(gyro_x_udegs, gyro_y_udegs, gyro_z_udegs) * 1e-6;
    measurement.angular_velocity_rads = Vector3f(radians(gyro_degs.x), radians(gyro_degs.y), radians(gyro_degs.z));

    // Extract and convert temperature (2 bytes, special formula -> degrees C)
    int16_t temp_raw = (packet[24]<<8)|packet[25];
    measurement.temperature_degc = static_cast<float>(temp_raw) * 0.008f + 20.0f;

    // Extract field from magnetometer (2 bytes each, already in mgauss)
    int16_t mag_x_mgauss = (packet[26]<<8)|packet[27];
    int16_t mag_y_mgauss = (packet[28]<<8)|packet[29];
    int16_t mag_z_mgauss = (packet[30]<<8)|packet[31];
    measurement.magnetic_field_mgauss = Vector3f(mag_x_mgauss, mag_y_mgauss, mag_z_mgauss);

    // Extract and scale barometer (4 bytes, units of 0.1 Pa -> Pa)
    int32_t baro_raw = (packet[32]<<24)|(packet[33]<<16)|(packet[34]<<8)|packet[35];
    measurement.air_pressure_p = baro_raw * 0.1f;

    // AHRS mode: extract quaternion (raw values are 1e6 times the actual ones)
    if (config_mode == ConfigMode::CONFIG_MODE_AHRS && packet_buffer_len >= 52) {
        int32_t quat_w_raw = (packet[36]<<24)|(packet[37]<<16)|(packet[38]<<8)|packet[39];
        int32_t quat_x_raw = (packet[40]<<24)|(packet[41]<<16)|(packet[42]<<8)|packet[43];
        int32_t quat_y_raw = (packet[44]<<24)|(packet[45]<<16)|(packet[46]<<8)|packet[47];
        int32_t quat_z_raw = (packet[48]<<24)|(packet[49]<<16)|(packet[50]<<8)|packet[51];
        measurement.orientation = Quaternion(quat_w_raw * 1e-6f, quat_x_raw * 1e-6f,
                                             quat_y_raw * 1e-6f, quat_z_raw * 1e-6f);

        measurement.type = MeasurementType::AHRS;
    } else {
        measurement.type = MeasurementType::IMU;
    }
}

uint8_t AP_ExternalAHRS_SensAItion_Parser::calculate_xor_checksum(const uint8_t* data, size_t start, size_t length) const
{
    uint8_t checksum = 0;
    for (size_t i = start; i < start + length && i < MAX_PACKET_SIZE; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
