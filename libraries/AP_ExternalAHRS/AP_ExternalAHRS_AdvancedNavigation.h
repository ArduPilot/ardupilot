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
   For more information on ANPP and Advanced Navigation Devices please
   see the following:
   Website: https://www.advancednavigation.com/
   ANPP: https://docs.advancednavigation.com/certus/ANPP/Advanced%20Navigation%20Packet.htm
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#if AP_EXTERNAL_AHRS_ADNAV_ENABLED

#define AN_PACKET_HEADER_SIZE 5
#define AN_MAXIMUM_PACKET_SIZE 255
#define AN_DECODE_BUFFER_SIZE 2*(AN_MAXIMUM_PACKET_SIZE+AN_PACKET_HEADER_SIZE)

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>


class AP_ExternalAHRS_AdvancedNavigation_Decoder
{
public:
    uint8_t _buffer[AN_DECODE_BUFFER_SIZE];
    uint16_t _buffer_length = 0;
    uint64_t _packets_decoded = 0;
    uint64_t _bytes_decoded = 0;
    uint64_t _bytes_discarded = 0;
    uint64_t _lrc_errors = 0;
    uint64_t _crc_errors = 0;
    size_t _bytes_received = 0;

    int decode_packet(uint8_t* out_buffer, size_t buf_size);

    uint8_t* pointer()
    {
        return &_buffer[_buffer_length];
    }

    size_t size() const
    {
        return sizeof(_buffer) - _buffer_length;
    }

    void receive(size_t received)
    {
        _complete = false;
        _bytes_received = received;
        _buffer_length += received;
    }

    size_t bytes_received() const
    {
        return _bytes_received;
    }

    bool is_complete() const
    {
        return _complete;
    }

    uint8_t calculate_header_lrc(uint8_t* data) const
    {
        return ((data[0] + data[1] + data[2] + data[3]) ^ 0xFF) + 1;
    };
private:
    bool _complete = false;
};

class AP_ExternalAHRS_AdvancedNavigation: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_AdvancedNavigation(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;

    // check for new data
    void update() override;

private:
    AP_ExternalAHRS_AdvancedNavigation_Decoder _decoder;

    typedef enum {
        gnss_fix_none,
        gnss_fix_2d,
        gnss_fix_3d,
        gnss_fix_sbas,
        gnss_fix_differential,
        gnss_fix_omnistar,
        gnss_fix_rtk_float,
        gnss_fix_rtk_fixed
    } gnss_fix_type_e;

    typedef enum {
        device_id_spatial = 1,
        device_id_orientus = 3,
        device_id_spatial_fog,
        device_id_spatial_dual,
        device_id_obdii_odometer = 10,
        device_id_orientus_v3,
        device_id_ilu,
        device_id_air_data_unit,
        device_id_spatial_fog_dual = 16,
        device_id_motus,
        device_id_gnss_compass,
        device_id_certus = 26,
        device_id_aries,
        device_id_boreas_d90,
        device_id_boreas_d90_fpga = 35,
        device_id_boreas_coil,
        device_id_certus_mini_a = 49,
        device_id_certus_mini_n,
        device_id_certus_mini_d,
    } device_id_e;



    struct PACKED AN_ACKNOWLEGE {
        uint8_t id_acknowledged;
        uint16_t crc_acknowledged;
        uint8_t result;
    };

    struct PACKED AN_DEVICE_INFO {
        uint32_t software_version;
        uint32_t device_id;
        uint32_t hardware_revision;
        uint32_t serial_1;
        uint32_t serial_2;
        uint32_t serial_3;
    };
    struct PACKED AN_STATUS{
        union {
            uint16_t r;
            struct {
                uint16_t system_failure :1;
                uint16_t accelerometer_sensor_failure :1;
                uint16_t gyroscope_sensor_failure :1;
                uint16_t magnetometer_sensor_failure :1;
                uint16_t pressure_sensor_failure :1;
                uint16_t gnss_failure :1;
                uint16_t accelerometer_over_range :1;
                uint16_t gyroscope_over_range :1;
                uint16_t magnetometer_over_range :1;
                uint16_t pressure_over_range :1;
                uint16_t minimum_temperature_alarm :1;
                uint16_t maximum_temperature_alarm :1;
                uint16_t internal_data_logging_error :1;
                uint16_t high_voltage_alarm :1;
                uint16_t gnss_antenna_fault :1;
                uint16_t serial_port_overflow_alarm :1;
            } b;
        } system;
        union {
            uint16_t r;
            struct {
                uint16_t orientation_filter_initialised :1;
                uint16_t ins_filter_initialised :1;
                uint16_t heading_initialised :1;
                uint16_t utc_time_initialised :1;
                uint16_t gnss_fix_type :3;
                uint16_t event1_flag :1;
                uint16_t event2_flag :1;
                uint16_t internal_gnss_enabled :1;
                uint16_t dual_antenna_heading_active :1;
                uint16_t velocity_heading_enabled :1;
                uint16_t atmospheric_altitude_enabled :1;
                uint16_t external_position_active :1;
                uint16_t external_velocity_active :1;
                uint16_t external_heading_active :1;
            } b;
        } filter;
    };
    
    struct PACKED AN_SYSTEM_STATE {
        AN_STATUS status;
        uint32_t unix_time_seconds;
        uint32_t microseconds;
        double llh[3]; //rad,rad,m
        float velocity_ned[3]; // m/s
        float body_acceleration[3]; // m/s/s
        float g_force;  // g's
        float rph[3]; // rad
        float angular_velocity[3]; // rad/s
        float llh_standard_deviation[3]; //m
    };

    struct PACKED AN_VELOCITY_STANDARD_DEVIATION {
        float sd[3];
    } *_last_vel_sd;

    struct PACKED AN_RAW_SENSORS {
    public:
        float accelerometers[3]; // m/s/s
        float gyroscopes[3]; // rad/s
        float magnetometers[3]; // mG
        float imu_temperature; // deg C
        float pressure; //Pascals
        float pressure_temperature; // deg C
    };

    struct PACKED AN_RAW_GNSS {
        uint32_t unix_time;
        uint32_t unix_microseconds;
        double llh[3]; //rad,rad,m
        float velocity_ned[3]; // m/s
        float llh_standard_deviation[3]; //m
        float tilt; // rad
        float heading;
        float tilt_sd;
        float heading_sd;
        union {
            uint16_t r;
            struct {
                uint16_t fix_type :3;
                uint16_t velocity_valid :1;
                uint16_t time_valid :1;
                uint16_t external_gnss :1;
                uint16_t tilt_valid :1; /* This field will only be valid if an external dual antenna GNSS system is connected */
                uint16_t heading_valid :1; /* This field will only be valid if an external dual antenna GNSS system is connected */
            } b;
        } flags;
    };

    struct PACKED AN_SATELLITES {
        float hdop;
        float vdop;
        uint8_t gps_satellites;
        uint8_t glonass_satellites;
        uint8_t beidou_satellites;
        uint8_t galileo_satellites;
        uint8_t sbas_satellites;
    } *_last_satellites;

    struct PACKED AN_PERIOD {
        uint8_t id;
        uint32_t packet_period;
    };

    struct PACKED AN_PACKETS_PERIOD {
        uint8_t permanent;
        uint8_t clear_existing_packet_periods;
        AN_PERIOD periods[(AN_MAXIMUM_PACKET_SIZE - 2)/sizeof(AN_PERIOD)];
    };

    class PACKED AN_PACKET
    {
    public:
        uint8_t lrc;
        uint8_t id;
        uint8_t length;
        uint16_t crc;

        union payload {
            uint8_t raw_packet[AN_MAXIMUM_PACKET_SIZE];
            AN_DEVICE_INFO device_info;
            AN_SYSTEM_STATE system_state;
            AN_VELOCITY_STANDARD_DEVIATION velocity_standard_deviation;
            AN_RAW_SENSORS raw_sensors;
            AN_RAW_GNSS raw_gnss;
            AN_SATELLITES satellites;
            AN_PACKETS_PERIOD packets_period;
        } payload;

        void update_checks(uint8_t header_id, uint8_t header_length)
        {
            // Update the packet check and header id
            crc = crc16_ccitt(payload.raw_packet, header_length, 0xFFFF);
            id = header_id;

            // Update the header LRC
            uint8_t* id_ptr = &id;
            lrc = ((id_ptr[0] + id_ptr[1] + id_ptr[2] + id_ptr[3]) ^ 0xFF) + 1;
        }

        uint8_t* raw_pointer()
        {
            return &lrc;
        }

        size_t packet_size() const
        {
            return AN_PACKET_HEADER_SIZE + length;
        }
    };

    union PACKED MsgUnion {
        MsgUnion() { }
        uint8_t buffer[AN_PACKET_HEADER_SIZE + AN_MAXIMUM_PACKET_SIZE];
        AN_PACKET packet;
    } _msg;

    uint16_t _current_rate;
    AN_STATUS _device_status;
    Vector3f _gnss_sd;

    AP_HAL::UARTDriver *_uart;
    HAL_Semaphore _sem;

    uint32_t _baudrate;
    int8_t _port_num;

    uint32_t _last_pkt_ms;
    uint32_t _last_state_pkt_ms;
    uint32_t _last_device_info_pkt_ms;
    uint32_t _last_raw_gnss_pkt_ms;
    uint32_t _device_id;
    uint32_t _hardware_rev;

    void update_thread();
    bool get_packets(void);
    bool request_data(void);
    bool sendPacketRequest(void);
    bool get_gnss_capability(void) const;
    bool get_baro_capability(void) const;
    void handle_packet();
};

#endif // AP_EXTERNAL_AHRS_ENABLED

