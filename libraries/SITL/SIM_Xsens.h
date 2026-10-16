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

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL {

class Xsens : public SerialDevice {
public:

    Xsens();

    // update state
    void update(void);

private:
    // Xbus protocol constants
    static constexpr uint8_t XBUS_PREAMBLE = 0xFA;
    static constexpr uint8_t XBUS_MASTERDEVICE = 0xFF;
    static constexpr uint8_t LENGTH_EXTENDER_BYTE = 0xFF;

    // Message IDs
    enum XsMessageId : uint8_t {
        GotoConfig = 0x30,
        GotoConfigAck = 0x31,
        GotoMeasurement = 0x10,
        GotoMeasurementAck = 0x11,
        MtData2 = 0x36,
        SetOutputConfig = 0xC0,
        OutputConfig = 0xC1,
        SetAlignmentRotation = 0xEC,
        AlignmentRotationAck = 0xED
    };

    // XDI (Xsens Data Identifier) constants
    enum XDI : uint16_t {
        PACKET_COUNTER = 0x1020,
        SAMPLE_TIME_FINE = 0x1060,
        STATUS_WORD = 0xE020,
        LAT_LON = 0x5042,
        ALTITUDE_ELLIPSOID = 0x5022,
        VELOCITY_XYZ = 0xD012,
        QUATERNION = 0x2010,
        ACCELERATION = 0x4020,
        RATE_OF_TURN = 0x8020,
        MAGNETIC_FIELD = 0xC020,
        UTC_TIME = 0x1010,
        BAROMETRIC_PRESSURE = 0x3010,
        TEMPERATURE = 0x0810,
        GNSSPVTDATA = 0x7010
    };

    // Device state
    enum class DeviceState {
        CONFIG_MODE,
        MEASUREMENT_MODE
    };

    uint32_t last_mtdata2_us;
    uint32_t last_gnss_pkt_us;
    uint16_t packet_counter;
    DeviceState current_state;
    bool output_configured;
    bool alignment_configured;
    
    // GNSS simulation state
    uint8_t gnss_fix_type;
    uint8_t gnss_num_satellites;
    uint32_t gnss_horizontal_accuracy;  // mm
    uint32_t gnss_vertical_accuracy;    // mm
    uint32_t gnss_speed_accuracy;       // mm/s
    uint16_t gnss_hdop;                 // * 0.01
    uint16_t gnss_vdop;                 // * 0.01
    
    // Message handling
    void process_incoming_data();
    void handle_message(const uint8_t *data, uint8_t length);
    void send_goto_config_ack();
    void send_output_config_ack(const uint8_t *original_payload, uint8_t original_length);
    void send_alignment_rotation_ack();
    void send_goto_measurement_ack();
    void send_mtdata2_packet();
    void send_gnsspvt_packet();
    
    // Xbus protocol helpers
    void create_xbus_message(uint8_t *buffer, uint8_t bid, uint8_t mid, uint16_t length, uint8_t *payload);
    void insert_checksum(uint8_t *message, uint16_t total_length);
    bool verify_checksum(const uint8_t *message, uint16_t total_length);
    uint8_t get_message_id(const uint8_t *message);
    uint8_t get_payload_length(const uint8_t *message);
    const uint8_t* get_const_pointer_to_payload(const uint8_t* xbus_message) const;
    
    // Data formatting helpers
    void write_uint8(uint8_t *buffer, uint8_t value);
    void write_uint16_be(uint8_t *buffer, uint16_t value);
    void write_uint32_be(uint8_t *buffer, uint32_t value);
    void write_int32_be(uint8_t *buffer, int32_t value);
    void write_float_be(uint8_t *buffer, float value);
    void write_double_fp1632(uint8_t *buffer, double value);
    
    // GNSS simulation helpers
    void update_gnss_state();
    void calculate_gps_time_from_utc(uint16_t year, uint8_t month, uint8_t day,
                                   uint8_t hour, uint8_t minute, uint8_t second,
                                   int32_t nano, uint16_t &gps_week, uint32_t &ms_tow);
    
    // Receive buffer for processing commands
    static constexpr size_t RX_BUFFER_SIZE = 256;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    size_t rx_buffer_pos;
};

}