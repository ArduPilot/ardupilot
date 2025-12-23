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

#include <string.h> // Required for memchr, memmove
#include "AP_ExternalAHRS_SensAItion_Parser.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// Constructor
AP_ExternalAHRS_SensAItion_Parser::AP_ExternalAHRS_SensAItion_Parser(ConfigMode mode) :
    config_mode(mode)
{
    reset_parser();
}

void AP_ExternalAHRS_SensAItion_Parser::reset_parser()
{
    parse_state = ParseState::WAITING_HEADER;
    packet_buffer_len = 0;
    target_payload_len = 0;
    current_packet_id = PacketID::UNKNOWN;
}

// Error Handler
void AP_ExternalAHRS_SensAItion_Parser::handle_invalid_packet()
{
    // Look for a new header byte starting from index 1 to resync
    uint8_t *p = (uint8_t *)memchr(&packet_buffer[1], HEADER_BYTE, packet_buffer_len - 1);

    if (p) {
        size_t bytes_to_discard = p - packet_buffer;
        size_t bytes_to_keep = packet_buffer_len - bytes_to_discard;

        memmove(&packet_buffer[0], p, bytes_to_keep);
        packet_buffer_len = bytes_to_keep;

        // Determine State based on Mode and remaining data
        if (config_mode == ConfigMode::INTERLEAVED_INS) {
            if (packet_buffer_len >= 2) {
                // We have Header + Potential ID. Process it immediately.
                uint8_t id = packet_buffer[1];
                switch (static_cast<PacketID>(id)) {
                case PacketID::IMU:
                    target_payload_len = PAYLOAD_SIZE_IMU;
                    parse_state = ParseState::COLLECTING_PAYLOAD;
                    current_packet_id = PacketID::IMU;
                    break;
                case PacketID::AHRS:
                    target_payload_len = PAYLOAD_SIZE_QUAT;
                    parse_state = ParseState::COLLECTING_PAYLOAD;
                    current_packet_id = PacketID::AHRS;
                    break;
                case PacketID::INS:
                    target_payload_len = PAYLOAD_SIZE_INS;
                    parse_state = ParseState::COLLECTING_PAYLOAD;
                    current_packet_id = PacketID::INS;
                    break;
                default:
                    // The "New" ID is also bad. Drop header and retry.
                    reset_parser();
                    return;
                }
            } else {
                parse_state = ParseState::WAITING_ID;
            }
        } else {
            // Legacy Mode
            target_payload_len = PAYLOAD_SIZE_IMU;
            parse_state = ParseState::COLLECTING_PAYLOAD;
        }
    } else {
        // No header found, reset completely
        reset_parser();
    }
}

// Checksum Validator & Deep Debugger
bool AP_ExternalAHRS_SensAItion_Parser::buffer_contains_valid_packet() const
{
    if (packet_buffer_len < 2 || packet_buffer[0] != HEADER_BYTE) {
        return false;
    }

    uint8_t calculated = calculate_xor_checksum(packet_buffer, packet_buffer_len - 1);
    uint8_t received = packet_buffer[packet_buffer_len - 1];

    if (calculated != received) {
        return false;
    }

    return true;
}

// The Core State Machine
bool AP_ExternalAHRS_SensAItion_Parser::parse_single_byte(uint8_t byte)
{
    // Safety: Prevent buffer overflow
    if (packet_buffer_len >= MAX_PACKET_SIZE) {
        handle_invalid_packet();
        if (packet_buffer_len >= MAX_PACKET_SIZE) {
            reset_parser();
        }
    }

    switch (parse_state) {
    case ParseState::WAITING_HEADER:
        if (byte == HEADER_BYTE) {
            packet_buffer_len = 0;
            packet_buffer[packet_buffer_len++] = byte;

            if (config_mode == ConfigMode::INTERLEAVED_INS) {
                parse_state = ParseState::WAITING_ID;
            } else {
                target_payload_len = PAYLOAD_SIZE_IMU;
                parse_state = ParseState::COLLECTING_PAYLOAD;
            }
        }
        break;

    case ParseState::WAITING_ID:
        packet_buffer[packet_buffer_len++] = byte;

        switch (static_cast<PacketID>(byte)) {
        case PacketID::IMU:
            target_payload_len = PAYLOAD_SIZE_IMU;
            current_packet_id = PacketID::IMU;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;

        case PacketID::AHRS:
            target_payload_len = PAYLOAD_SIZE_QUAT;
            current_packet_id = PacketID::AHRS;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;

        case PacketID::INS:
            target_payload_len = PAYLOAD_SIZE_INS;
            current_packet_id = PacketID::INS;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;

        default:
            parse_errors++;
            handle_invalid_packet();
            break;
        }
        break;

    case ParseState::COLLECTING_PAYLOAD:
        packet_buffer[packet_buffer_len++] = byte;

        // Calculate Expected Total Length
        // Legacy: Header(1) + Payload(N) + CRC(1)
        // Interleaved: Header(1) + ID(1) + Payload(N) + CRC(1)
        size_t overhead = (config_mode == ConfigMode::INTERLEAVED_INS) ? 3 : 2;
        size_t expected_total_len = target_payload_len + overhead;

        if (packet_buffer_len >= expected_total_len) {
            if (buffer_contains_valid_packet()) {

                valid_packets++;
                return true;
            } else {
                parse_errors++;
                handle_invalid_packet();
                return false;
            }
        }
        break;
    }

    return false;
}

bool AP_ExternalAHRS_SensAItion_Parser::validate_checksum() const
{
    return buffer_contains_valid_packet();
}

uint8_t AP_ExternalAHRS_SensAItion_Parser::calculate_xor_checksum(const uint8_t* data, size_t len) const
{
    uint8_t checksum = 0;
    for (size_t i = 1; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Router
void AP_ExternalAHRS_SensAItion_Parser::decode_packet(Measurement& measurement)
{
    const uint8_t* payload = nullptr;

    if (config_mode == ConfigMode::INTERLEAVED_INS) {
        payload = &packet_buffer[2]; // Skip Header + ID
        switch (current_packet_id) {
        case PacketID::IMU: decode_imu(payload, measurement); break;
        case PacketID::AHRS: decode_ahrs(payload, measurement); break;
        case PacketID::INS: decode_ins(payload, measurement); break;
        default: break;
        }
    } else {
        payload = &packet_buffer[1]; // Skip Header
        decode_imu(payload, measurement);
    }
}

// IMU Decoder (Packet 0)
void AP_ExternalAHRS_SensAItion_Parser::decode_imu(const uint8_t* payload, Measurement& measurement)
{
    // ... [Code omitted: No changes to decoders, use your existing implementation] ...
    // Accel (Bytes 0-11): 3 x Int32 (ug)
    int32_t accel_x_ug = (int32_t)((payload[0]<<24)|(payload[1]<<16)|(payload[2]<<8)|payload[3]);
    int32_t accel_y_ug = (int32_t)((payload[4]<<24)|(payload[5]<<16)|(payload[6]<<8)|payload[7]);
    int32_t accel_z_ug = (int32_t)((payload[8]<<24)|(payload[9]<<16)|(payload[10]<<8)|payload[11]);

    // Gyro (Bytes 12-23): 3 x Int32 (udeg/s)
    int32_t gyro_x_udegs = (int32_t)((payload[12]<<24)|(payload[13]<<16)|(payload[14]<<8)|payload[15]);
    int32_t gyro_y_udegs = (int32_t)((payload[16]<<24)|(payload[17]<<16)|(payload[18]<<8)|payload[19]);
    int32_t gyro_z_udegs = (int32_t)((payload[20]<<24)|(payload[21]<<16)|(payload[22]<<8)|payload[23]);

    // Temp (Bytes 24-25): 1 x Int16 (Scaled)
    int16_t temp_raw = (int16_t)((payload[24]<<8)|payload[25]);

    // Mag (Bytes 26-31): 3 x Int16 (mGauss)
    int16_t mag_x_mgauss = (int16_t)((payload[26]<<8)|payload[27]);
    int16_t mag_y_mgauss = (int16_t)((payload[28]<<8)|payload[29]);
    int16_t mag_z_mgauss = (int16_t)((payload[30]<<8)|payload[31]);

    // Baro (Bytes 32-35): 1 x Int32 (0.1 Pa)
    int32_t baro_raw = (int32_t)((payload[32]<<24)|(payload[33]<<16)|(payload[34]<<8)|payload[35]);

    // Accel: ug -> m/s^2 (Note: 1,000,000 ug = 9.81 m/s^2 roughly)
    const float ug_to_mss = 1.0e-6f * GRAVITY_MSS;
    measurement.acceleration_mss = Vector3f(accel_x_ug, accel_y_ug, accel_z_ug) * ug_to_mss;

    // Gyro: udeg/s -> rad/s (Note: 1,000,000 udeg/s = 1 deg/s = 0.017 rad/s)
    const float udeg_to_rad = 1.0e-6f * DEG_TO_RAD;
    measurement.angular_velocity_rads = Vector3f(gyro_x_udegs, gyro_y_udegs, gyro_z_udegs) * udeg_to_rad;

    // Temp: (Raw * 0.008) + 20
    measurement.temperature_degc = ((float)temp_raw * 0.008f) + 20.0f;

    // Mag: mGauss (Pass-through, ArduPilot expects mGauss)
    measurement.magnetic_field_mgauss = Vector3f(mag_x_mgauss, mag_y_mgauss, mag_z_mgauss);

    // Baro: 0.1 Pa -> Pascal
    measurement.air_pressure_p = (float)baro_raw * 0.1f;

    // Metadata
    measurement.type = MeasurementType::IMU;
    measurement.timestamp_us = AP_HAL::micros64();
}

void AP_ExternalAHRS_SensAItion_Parser::decode_ahrs(const uint8_t* payload, Measurement& measurement)
{
    // Payload layout: W (0-3), X (4-7), Y (8-11), Z (12-15)
    int32_t quat_w_raw = (int32_t)((payload[0]<<24)|(payload[1]<<16)|(payload[2]<<8)|payload[3]);
    int32_t quat_x_raw = (int32_t)((payload[4]<<24)|(payload[5]<<16)|(payload[6]<<8)|payload[7]);
    int32_t quat_y_raw = (int32_t)((payload[8]<<24)|(payload[9]<<16)|(payload[10]<<8)|payload[11]);
    int32_t quat_z_raw = (int32_t)((payload[12]<<24)|(payload[13]<<16)|(payload[14]<<8)|payload[15]);

    const float scale_factor = 1.0e-6f;

    measurement.orientation = Quaternion(
                                  (float)quat_w_raw * scale_factor,
                                  (float)quat_x_raw * scale_factor,
                                  (float)quat_y_raw * scale_factor,
                                  (float)quat_z_raw * scale_factor
                              );

    measurement.type = MeasurementType::AHRS;
    measurement.timestamp_us = AP_HAL::micros64();
}

void AP_ExternalAHRS_SensAItion_Parser::decode_ins(const uint8_t* payload, Measurement& measurement)
{
    // --- 1. PARSE RAW BYTES (Big-Endian) ---

    // 0-3: Num Sats (G2, G1)
    measurement.num_sats_gnss2 = payload[1];
    measurement.num_sats_gnss1 = payload[3];

    // 4-7: Error Flags
    measurement.error_flags = (uint32_t)((payload[4]<<24)|(payload[5]<<16)|(payload[6]<<8)|payload[7]);

    // 8: Sensor Valid
    measurement.sensor_valid = payload[8];

    // 9-16: Lat/Lon (1e-7 deg)
    int32_t lat_raw = (int32_t)((payload[9]<<24)|(payload[10]<<16)|(payload[11]<<8)|payload[12]);
    int32_t lon_raw = (int32_t)((payload[13]<<24)|(payload[14]<<16)|(payload[15]<<8)|payload[16]);

    // 17-28: Velocity N, E, D (mm/s)
    int32_t vel_n_mm = (int32_t)((payload[17]<<24)|(payload[18]<<16)|(payload[19]<<8)|payload[20]);
    int32_t vel_e_mm = (int32_t)((payload[21]<<24)|(payload[22]<<16)|(payload[23]<<8)|payload[24]);
    int32_t vel_d_mm = (int32_t)((payload[25]<<24)|(payload[26]<<16)|(payload[27]<<8)|payload[28]);

    // 29-32: Altitude relative to WGS 84 ellipsoid (mm)
    int32_t alt_raw_mm = (int32_t)((payload[29]<<24)|(payload[30]<<16)|(payload[31]<<8)|payload[32]);

    // 33: Alignment Status
    measurement.alignment_status = payload[33];

    // 34-37: Time of week (ms)
    measurement.time_itow_ms = (uint32_t)((payload[34]<<24)|(payload[35]<<16)|(payload[36]<<8)|payload[37]);

    // 38-39: GNSS Fix
    measurement.gnss2_fix = payload[38];
    measurement.gnss1_fix = payload[39];

    // 40-44: UTC Date/Time
    uint16_t year = (uint16_t)((payload[40]<<8) | payload[41]);
    uint16_t month = (uint16_t)((payload[42]<<8) | payload[43]);
    uint8_t day = payload[44];

    // 45-68: Accuracy Metrics (mm or mm/s)
    int32_t acc_lat_mm = (int32_t)((payload[45]<<24)|(payload[46]<<16)|(payload[47]<<8)|payload[48]);
    int32_t acc_lon_mm = (int32_t)((payload[49]<<24)|(payload[50]<<16)|(payload[51]<<8)|payload[52]);
    int32_t acc_vn_mm  = (int32_t)((payload[53]<<24)|(payload[54]<<16)|(payload[55]<<8)|payload[56]);
    int32_t acc_ve_mm  = (int32_t)((payload[57]<<24)|(payload[58]<<16)|(payload[59]<<8)|payload[60]);
    int32_t acc_vd_mm  = (int32_t)((payload[61]<<24)|(payload[62]<<16)|(payload[63]<<8)|payload[64]);
    int32_t acc_vd_pos_mm = (int32_t)((payload[65]<<24)|(payload[66]<<16)|(payload[67]<<8)|payload[68]);

    // --- 2. POPULATE & CONVERT ---
    const float mm_to_cm = 0.1f;
    measurement.location = Location(lat_raw, lon_raw, alt_raw_mm * mm_to_cm, Location::AltFrame::ABSOLUTE);

    const float mms_to_ms = 0.001f;
    const float mm_to_m = 0.001f;

    measurement.velocity_ned.x = (float)vel_n_mm * mms_to_ms;
    measurement.velocity_ned.y = (float)vel_e_mm * mms_to_ms;
    measurement.velocity_ned.z = (float)vel_d_mm * mms_to_ms;

    // Accuracy
    measurement.pos_accuracy.x = (float)acc_lat_mm * mm_to_m; // Horizontal Latitude
    measurement.pos_accuracy.y = (float)acc_lon_mm * mm_to_m; // Horizontal Longitude
    measurement.pos_accuracy.z = (float)acc_vd_pos_mm * mm_to_m; // Vertical

    measurement.vel_accuracy.x = (float)acc_vn_mm * mms_to_ms;
    measurement.vel_accuracy.y = (float)acc_ve_mm * mms_to_ms;
    measurement.vel_accuracy.z = (float)acc_vd_mm * mms_to_ms;

    // GPS Week Calculation (Gate check)
    if (measurement.gnss1_fix >= 2) {
        measurement.gps_week = calculate_gps_week(year, month, day);
    } else {
        measurement.gps_week = 0;
    }

    measurement.type = MeasurementType::INS;
    measurement.timestamp_us = AP_HAL::micros64();

}

// ---------------------------------------------------------------------------
// Helper: Calculate GPS Week from UTC Date
// -------------------------------------------------incomplete typ--------------------------
uint16_t AP_ExternalAHRS_SensAItion_Parser::calculate_gps_week(uint16_t year, uint8_t month, uint8_t day)
{
    // Sanity Check
    if (year < 1980 || month < 1 || month > 12 || day < 1 || day > 31) {
        return 0;
    }

    // 1. Calculate Total Days since 1980-01-06 (GPS Epoch)
    // We use a simple loop because we only care about ~50 years, extremely fast on MCU.
    uint32_t total_days = 0;

    // A. Add days for full years
    for (uint16_t y = 1980; y < year; y++) {
        bool is_leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
        total_days += is_leap ? 366 : 365;
    }

    // B. Add days for full months in current year
    const uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    bool curr_leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));

    for (uint8_t m = 0; m < month - 1; m++) {
        if (m == 1 && curr_leap) {
            total_days += 29;
        } else {
            total_days += days_in_month[m];
        }
    }

    // C. Add days in current month
    total_days += (day - 1);

    // D. Offset to Jan 6
    // 1980-01-01 to 1980-01-06 is 5 days.
    // If our calc started at Jan 1, we subtract 5 days to align with GPS Epoch (Jan 6).
    // However, ensure we don't underflow if date is Jan 1-5 1980 (not possible with >2020 check).
    if (total_days < 6) {
        return 0;    // Should not happen for modern dates
    }
    total_days -= 5; // GPS Epoch starts Jan 6

    // 2. Return Weeks
    return (uint16_t)(total_days / 7);
}
