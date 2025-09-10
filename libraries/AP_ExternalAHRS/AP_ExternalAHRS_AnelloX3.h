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
    driver for Anello X3 IMU system
*/

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ANELLOX3_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_HAL/AP_HAL.h>

class AP_ExternalAHRS_AnelloX3: public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_AnelloX3(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // message type identifiers
    enum class DescriptorSet {
        IMUData = 253
    };

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // check for new data
    void update() override {
        build_packet();
    };
    
    // get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;
    virtual uint8_t num_gps_sensors(void) const override;

    // listens for uart data and parses it into message types
    void build_packet();

    // publishes the imu data
    void post_imu() const;

    // alternate to update(), added to the scheduler
    void update_thread();


private:
    // UART port config
    uint32_t baudrate;
    int8_t port_num;
    bool port_open = false;
    
    // HAL components
    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;

    // parsing state machine defines
    const uint8_t SYNC_ONE = 0xC5;
    const uint8_t SYNC_TWO = 0x50;
    enum class ParseState {
        WaitingFor_SyncOne, // 0xC5
        WaitingFor_SyncTwo, // 0x50
        WaitingFor_Descriptor, // 253
        WaitingFor_PayloadLength, // up to 255
        WaitingFor_Data,
        WaitingFor_Checksum
    };

    // raw data unpacking structure
    struct AnelloX3_BinaryPayload {
        uint64_t mcu_time = 0; // ns -- time since power on
        uint64_t sync_time = 0; // ns -- time of external sync pulse
        int16_t ax1 = 0; // g = value * (range * 0.0000305) -- scaled sensor accel
        int16_t ay1 = 0; 
        int16_t az1 = 0;
        int16_t wx1 = 0; // dps = value * (range * 0.0000305) -- scaled sensor rate
        int16_t wy1 = 0;
        int16_t wz1 = 0;
        int32_t og_wx = 0; // dps * 1e7 -- scaled sensor rate for FOG 
        int32_t og_wy = 0;
        int32_t og_wz = 0;
        int16_t mag_x = 0; // g * 4096 -- scaled magnetometer data
        int16_t mag_y = 0;
        int16_t mag_z = 0;
        int16_t temp = 0; // degC * 100 -- scaled temperature value
        uint16_t mems_ranges = 0; // first 5 bits accel, next 11 bits gyro
        uint16_t fog_range = 0; // fog range in dps
        // bitfield flag values
        // BIT 0 Gyro discrepancy
        // BIT 1 Temperature uncontrolled
        // BIT 2 Over current error
        // BIT 3 SiPhOG supply voltage bad
        uint8_t fusion_status_x = 0;
        uint8_t fusion_status_y = 0;
        uint8_t fusion_status_z = 0;
    };

    // processed data structure
    struct {
        uint64_t b_time;
        uint64_t s_time;
        Vector3f mems_accel;
        Vector3f mems_gyro;
        Vector3f fog_gyro;
        Vector3f mag;
        float temp;
        float mems_acc_range;
        float mems_gyro_range;
        float fog_gyro_range;
        uint8_t fusion_status_x;
        uint8_t fusion_status_y;
        uint8_t fusion_status_z;
    } imu_data;


    // full incoming packet structure
    struct AnelloX3_Packet {
        uint8_t header[4]; // incl 2-byte preamble, 1-byte message type, and 1-byte message length
        uint8_t payload[255];
        uint8_t checksum[2]; // calculated not incl preamble nor checksum bytes

        // gets the payload length
        uint8_t payload_length() const WARN_IF_UNUSED {
            return header[3];
        }

        // sets the payload length
        void payload_length(const uint8_t len) {
            header[3] = len;
        }

        // gets the descriptor set
        DescriptorSet descriptor_set() const WARN_IF_UNUSED {
            return DescriptorSet(header[2]);
        }

        // sets the descriptor set (without validation)
        void descriptor_set(const uint8_t descriptor_set) {
            header[2] = descriptor_set;
        }
    };

    // message in state variables
    struct {
        AnelloX3_Packet packet;
        ParseState state;
        uint8_t index;
    } message_in;


    // passes byte from serial stream to the parser
    bool handle_byte(const uint8_t b, DescriptorSet& descriptor);

    // returns true if the checksum for the packet is valid, else false.
    static bool valid_packet(const AnelloX3_Packet &packet);

    // pulls out data from successfully constructed message
    DescriptorSet handle_packet(const AnelloX3_Packet& packet);

    // collects data from an imu packet into `imu_data`
    void handle_imu(const AnelloX3_Packet &packet);
    
    // timestamp of last recv packet
    uint32_t last_imu_pkt;

    //  converts raw binary data to actual values
    void convert_imu_data(const AnelloX3_BinaryPayload& bin_payload);
};

#endif  // AP_EXTERNAL_AHRS_ANELLOX3_ENABLED
