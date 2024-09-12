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
  support for MicroStrain MIP parsing
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_MICROSTRAIN_ENABLED

#include <AP_GPS/AP_GPS.h>
#include <AP_Math/vector3.h>
#include <AP_Math/quaternion.h>

class AP_MicroStrain
{
public:


protected:

    enum class ParseState {
        WaitingFor_SyncOne,
        WaitingFor_SyncTwo,
        WaitingFor_Descriptor,
        WaitingFor_PayloadLength,
        WaitingFor_Data,
        WaitingFor_Checksum
    };

    struct {
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        Quaternion quat;
        float pressure;
    } imu_data;

    static constexpr uint8_t NUM_GNSS_INSTANCES = 2;

    struct {
        uint16_t week;
        uint32_t tow_ms;
        GPS_FIX_TYPE fix_type;
        uint8_t satellites;
        float horizontal_position_accuracy;
        float vertical_position_accuracy;
        float hdop;
        float vdop;
        int32_t lon;
        int32_t lat;
        int32_t msl_altitude;
        float ned_velocity_north;
        float ned_velocity_east;
        float ned_velocity_down;
        float speed_accuracy;
    } gnss_data[NUM_GNSS_INSTANCES];

    struct {
        uint16_t state;
        uint16_t mode;
        uint16_t flags;
    } filter_status;

    struct {
        uint16_t week;
        uint32_t tow_ms;
        // 1-sigma position uncertainty in the NED local-level frame [meters].
        Vector3f ned_position_uncertainty;
        int32_t lon;
        int32_t lat;
        int32_t hae_altitude;
        float ned_velocity_north;
        float ned_velocity_east;
        float ned_velocity_down;
        // 1-sigma velocity uncertainties in the NED local-level frame.
        Vector3f ned_velocity_uncertainty;
        // 4x1 vector representation of the quaternion describing the orientation of the device with respect to the NED local-level frame.
        // NED [Qw, Qx, Qy, Qz]
        Quaternion attitude_quat;
    } filter_data;

    enum class DescriptorSet {
        BaseCommand = 0x01,
        DMCommand = 0x0C,
        SystemCommand = 0x7F,
        IMUData = 0x80,
        GNSSData = 0x81,
        FilterData = 0x82,
        GNSSRecv1 = 0x91,
        GNSSRecv2 = 0x92
    };

    const uint8_t SYNC_ONE = 0x75;
    const uint8_t SYNC_TWO = 0x65;

    struct MicroStrain_Packet {
        uint8_t header[4];
        uint8_t payload[255];
        uint8_t checksum[2];

        // Gets the payload length
        uint8_t payload_length() const WARN_IF_UNUSED {
            return header[3];
        }

        // Sets the payload length
        void payload_length(const uint8_t len) {
            header[3] = len;
        }

        // Gets the descriptor set
        DescriptorSet descriptor_set() const WARN_IF_UNUSED {
            return DescriptorSet(header[2]);
        }

        // Sets the descriptor set (without validation)
        void descriptor_set(const uint8_t descriptor_set) {
            header[2] = descriptor_set;
        }
    };

    struct {
        MicroStrain_Packet packet;
        AP_MicroStrain::ParseState state;
        uint8_t index;
    } message_in;

    uint32_t last_imu_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_filter_pkt;

    // Handle a single byte.
    // If the byte matches a descriptor, it returns true and that type should be handled.
    bool handle_byte(const uint8_t b, DescriptorSet& descriptor);
    // Returns true if the fletcher checksum for the packet is valid, else false.
    static bool valid_packet(const MicroStrain_Packet &packet);
    // Calls the correct functions based on the packet descriptor of the packet
    DescriptorSet handle_packet(const MicroStrain_Packet &packet);
    // Collects data from an imu packet into `imu_data`
    void handle_imu(const MicroStrain_Packet &packet);
    // Collects data from a gnss packet into `gnss_data`
    void handle_gnss(const MicroStrain_Packet &packet);
    void handle_filter(const MicroStrain_Packet &packet);
    static Vector3f populate_vector3f(const uint8_t* data, uint8_t offset);
    static Quaternion populate_quaternion(const uint8_t* data, uint8_t offset);
    // Depending on the descriptor, the data corresponds to a different GNSS instance.
    static bool get_gnss_instance(const DescriptorSet& descriptor, uint8_t& instance);
};

#endif // AP_MICROSTRAIN_ENABLED
