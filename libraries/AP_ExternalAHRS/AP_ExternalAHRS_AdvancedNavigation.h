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
#define AN_START_SYSTEM_PACKETS 0
#define AN_START_STATE_PACKETS 20
#define AN_START_CONFIGURATION_PACKETS 180

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
        packet_id_acknowledge,
        packet_id_request,
        packet_id_boot_mode,
        packet_id_device_information,
        packet_id_restore_factory_settings,
        packet_id_reset,
        packet_id_6_reserved,
        packet_id_file_transfer_request,
        packet_id_file_transfer_acknowledge,
        packet_id_file_transfer,
        packet_id_serial_port_passthrough,
        packet_id_ip_configuration,
        packet_id_12_reserved,
        packet_id_extended_device_information,
        packet_id_subcomponent_information,
        end_system_packets,

        packet_id_system_state = AN_START_STATE_PACKETS,
        packet_id_unix_time,
        packet_id_formatted_time,
        packet_id_status,
        packet_id_position_standard_deviation,
        packet_id_velocity_standard_deviation,
        packet_id_euler_orientation_standard_deviation,
        packet_id_quaternion_orientation_standard_deviation,
        packet_id_raw_sensors,
        packet_id_raw_gnss,
        packet_id_satellites,
        packet_id_satellites_detailed,
        packet_id_geodetic_position,
        packet_id_ecef_position,
        packet_id_utm_position,
        packet_id_ned_velocity,
        packet_id_body_velocity,
        packet_id_acceleration,
        packet_id_body_acceleration,
        packet_id_euler_orientation,
        packet_id_quaternion_orientation,
        packet_id_dcm_orientation,
        packet_id_angular_velocity,
        packet_id_angular_acceleration,
        packet_id_external_position_velocity,
        packet_id_external_position,
        packet_id_external_velocity,
        packet_id_external_body_velocity,
        packet_id_external_heading,
        packet_id_running_time,
        packet_id_local_magnetics,
        packet_id_odometer_state,
        packet_id_external_time,
        packet_id_external_depth,
        packet_id_geoid_height,
        packet_id_rtcm_corrections,
        packet_id_56_reserved,
        packet_id_wind,
        packet_id_heave,
        packet_id_59_reserved,
        packet_id_raw_satellite_data,
        packet_id_raw_satellite_ephemeris,
        packet_id_62_reserved,
        packet_id_63_reserved,
        packet_id_64_reserved,
        packet_id_65_reserved,
        packet_id_gnss_summary,
        packet_id_external_odometer,
        packet_id_external_air_data,
        packet_id_gnss_receiver_information,
        packet_id_raw_dvl_data,
        packet_id_north_seeking_status,
        packet_id_gimbal_state,
        packet_id_automotive,
        packet_id_74_reserved,
        packet_id_external_magnetometers,
        packet_id_76_reserved,
        packet_id_77_reserved,
        packet_id_78_reserved,
        packet_id_79_reserved,
        packet_id_basestation,
        packet_id_81_reserved,
        packet_id_82_reserved,
        packet_id_zero_angular_velocity,
        packet_id_extended_satellites,
        packet_id_sensor_temperatures,
        packet_id_system_temperature,
        packet_id_87_reserved,
        end_state_packets,

        packet_id_packet_timer_period = AN_START_CONFIGURATION_PACKETS,
        packet_id_packet_periods,
        packet_id_baud_rates,
        packet_id_183_reserved,
        packet_id_sensor_ranges,
        packet_id_installation_alignment,
        packet_id_filter_options,
        packet_id_187_reserved,
        packet_id_gpio_configuration,
        packet_id_magnetic_calibration_values,
        packet_id_magnetic_calibration_configuration,
        packet_id_magnetic_calibration_status,
        packet_id_odometer_configuration,
        packet_id_zero_alignment,
        packet_id_reference_offsets,
        packet_id_gpio_output_configuration,
        packet_id_dual_antenna_configuration,
        packet_id_gnss_configuration,
        packet_id_user_data,
        packet_id_gpio_input_configuration,
        packet_id_200_reserved,
        packet_id_201_reserved,
        packet_id_ip_dataports_configuration,
        packet_id_can_configuration,
        packet_id_device_name,
        end_configuration_packets
    } packet_id_e;

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

    typedef enum {
        vehicle_type_unlimited,
        vehicle_type_bicycle,
        vehicle_type_car,
        vehicle_type_hovercraft,
        vehicle_type_submarine,
        vehicle_type_3d_underwater,
        vehicle_type_fixed_wing_plane,
        vehicle_type_3d_aircraft,
        vehicle_type_human,
        vehicle_type_small_boat,
        vehicle_type_ship,
        vehicle_type_stationary,
        vehicle_type_stunt_plane,
        vehicle_type_race_car
    } vehicle_type_e;

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

    struct PACKED AN_PACKET_PERIODS {
        uint8_t permanent;
        uint8_t clear_existing_packet_periods;
        AN_PERIOD periods[(AN_MAXIMUM_PACKET_SIZE - 2)/sizeof(AN_PERIOD)];
    };

    struct PACKED AN_FILTER_OPTIONS {
        uint8_t permanent;
        uint8_t vehicle_type;
        uint8_t internal_gnss_enabled;
        uint8_t magnetometers_enabled;
        uint8_t atmospheric_altitude_enabled;
        uint8_t velocity_heading_enabled;
        uint8_t reversing_detection_enabled;
        uint8_t motion_analysis_enabled;
        uint8_t automatic_magnetic_calibration_enabled;
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
            AN_PACKET_PERIODS packet_periods;
            AN_FILTER_OPTIONS filter_options;
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
    bool set_filter_options(bool gnss_en, vehicle_type_e vehicle_type, bool permanent = false);
    bool set_filter_options(AN_FILTER_OPTIONS options_packet);
    void handle_packet();
};

#endif // AP_EXTERNAL_AHRS_ENABLED

