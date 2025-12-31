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

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

class AP_ExternalAHRS_SensAItion_Parser
{
public:
    // Configuration Mode defined in EAHRS_OPTIONS (Bit 1)
    enum class ConfigMode {
        IMU = 0,            // IMU Only Mode (No ID byte, fixed 38 byte packet)
        INTERLEAVED_INS = 1 // Interleaved Mode (Header -> ID -> Payload)
    };

    // Type of data contained in a decoded Measurement
    enum class MeasurementType {
        UNINITIALIZED = 0,
        IMU,
        AHRS, // Maps to Packet 1 (Orientation)
        INS   // Maps to Packet 2 (Navigation)
    };

    // Public Constants (Available to Driver)
    static const uint16_t MAX_PACKET_SIZE = 1024;
    static const uint8_t HEADER_BYTE = 0xFA;

    // Container for decoded data passed to Backend
    struct Measurement {
        MeasurementType type;
        uint64_t timestamp_us;

        // Packet 0 (IMU) Data
        Vector3f acceleration_mss;
        Vector3f angular_velocity_rads;
        Vector3f magnetic_field_mgauss;
        float temperature_degc;
        float air_pressure_p;

        // Packet 1 (Orientation) Data
        Quaternion orientation;

        // Packet 2 (INS) Data
        Location location;          // Lat/Lon/Alt
        Vector3f velocity_ned;      // North/East/Down (m/s)

        // Accuracy Metrics (Vectors as requested)
        // ArduPilot often uses float for horiz/vert, but Vector3f is more flexible
        // if the sensor provides 3-axis accuracy.
        // Based on your config (AccLat, AccLon, AccPosD), we have 3 components.
        Vector3f pos_accuracy;      // North/East/Down (m)
        Vector3f vel_accuracy;      // North/East/Down (m/s)

        // Status & Health Flags
        uint8_t alignment_status;   // 1 = Align OK
        uint8_t gnss1_fix;
        uint8_t gnss2_fix;

        uint8_t num_sats_gnss1;
        uint8_t num_sats_gnss2;

        // Time
        uint32_t time_itow_ms;      // GNSS time of week (ms)
        uint16_t gps_week;          // Calculated Week Number

        uint32_t error_flags;       // Bitmask from sensor
        uint8_t sensor_valid;       // Validity bitmask
    };

    // Constructor
    AP_ExternalAHRS_SensAItion_Parser(ConfigMode mode);

    // Parse 'data_size' bytes from 'data'. Call 'handler' for EVERY valid packet found.
    template <typename Functor>
    void parse_stream(const uint8_t* data, size_t data_size, Functor& handler)
    {
        Measurement m;
        for (size_t i = 0; i < data_size; i++) {
            if (parse_single_byte(data[i])) {
                decode_packet(m);
                handler(m);       // <--- Fire the event!
                reset_parser();   // Reset for next packet in same buffer
            }
        }
    }

    // Number of parsed full length buffers that did not contain a valid packet
    uint32_t get_parse_errors() const
    {
        return parse_errors;
    }

    // Number of valid packets received during object lifetime
    uint32_t get_valid_packets() const
    {
        return valid_packets;
    }

private:
    // Payload Sizes (Excluding Header, ID, CRC)
    static const uint8_t PAYLOAD_SIZE_IMU = 36;  // Packet 0
    static const uint8_t PAYLOAD_SIZE_QUAT = 16; // Packet 1
    static const uint8_t PAYLOAD_SIZE_INS = 69;  // Packet 2

    // Packet IDs (Interleaved Mode)
    enum class PacketID : uint8_t {
        IMU = 0x00,
        AHRS = 0x01,
        INS = 0x02,
        UNKNOWN = 0xFF
    };

    // Internal State Machine
    enum class ParseState {
        WAITING_HEADER,
        WAITING_ID,
        COLLECTING_PAYLOAD
    };

    // Core Logic
    void reset_parser(); // Does not reset packet and parse error counters
    bool parse_single_byte(uint8_t byte);
    void handle_invalid_packet(); // Robust error recovery (memmove)
    bool buffer_contains_valid_packet() const;
    bool validate_checksum() const;
    uint8_t calculate_xor_checksum(const uint8_t* data, size_t len) const;

    // Decoders
    void decode_packet(Measurement& measurement);
    void decode_imu(const uint8_t* payload, Measurement& measurement);
    void decode_ahrs(const uint8_t* payload, Measurement& measurement);
    void decode_ins(const uint8_t* payload, Measurement& measurement);

    //Helpers
    uint16_t calculate_gps_week(uint16_t year, uint8_t month, uint8_t day);

    // Members
    ConfigMode config_mode;
    ParseState parse_state;
    PacketID current_packet_id;

    uint8_t packet_buffer[MAX_PACKET_SIZE];
    size_t packet_buffer_len;
    size_t target_payload_len;

    uint32_t valid_packets = 0;
    uint32_t parse_errors = 0;
};
