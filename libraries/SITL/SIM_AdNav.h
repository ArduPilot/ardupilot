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
    Simulation for a Advanced Navigation External AHRS System.
    usage:
    PARAMS:
     param set AHRS_EKF_TYPE 11
     param set EAHRS_TYPE 3
     param set SERIAL5_PROTOCOL 36
     param set SERIAL5_BAUD 115
    
    sim_vehicle.py -D --console --map -A --serial5=sim:AdNav
 */


#pragma once

#include "SIM_Aircraft.h"
#include <stdio.h>
#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"
#include <AP_Math/vector3.h>

#define AN_PACKET_ID_PACKET_PERIODS 181
#define AN_PACKET_ID_SATELLITES 30
#define AN_PACKET_ID_RAW_GNSS 29
#define AN_PACKET_ID_RAW_SENSORS 28
#define AN_PACKET_ID_VELOCITY_STANDARD_DEVIATION 25
#define AN_PACKET_ID_SYSTEM_STATE 20
#define AN_PACKET_ID_DEVICE_INFO 3
#define AN_PACKET_ID_REQUEST_PACKET 1
#define AN_PACKET_ID_ACKNOWLEDGE 0
#define AN_PACKET_HEADER_SIZE 5
#define AN_MAXIMUM_PACKET_SIZE 255
#define AN_DECODE_BUFFER_SIZE 2*(AN_MAXIMUM_PACKET_SIZE+AN_PACKET_HEADER_SIZE)
#define AN_GPS_EPOCH_UNIX_OFFSET 315964800 // GPS Week 0 sec 0 is midnight Sunday Jan 6th 1980 UTC
#define AN_TIMEOUT 5000 //ms
#define AN_MAXIMUM_PACKET_PERIODS 50

#define an_packet_pointer(packet) packet->header
#define an_packet_size(packet) (packet->length + AN_PACKET_HEADER_SIZE)*sizeof(uint8_t)
#define an_packet_crc(packet) ((packet->header[4]<<8) | packet->header[3])

#define an_decoder_pointer(an_decoder) &(an_decoder)->buffer[(an_decoder)->buffer_length]
#define an_decoder_size(an_decoder) (sizeof((an_decoder)->buffer) - (an_decoder)->buffer_length)
#define an_decoder_increment(an_decoder, bytes_received) (an_decoder)->buffer_length += bytes_received

typedef struct {
    uint8_t id;
    uint8_t length;
    uint8_t header[AN_PACKET_HEADER_SIZE];
    uint8_t data[1];
} an_packet_t;

typedef enum
{
	acknowledge_success,
	acknowledge_failure_crc,
	acknowledge_failure_length,
	acknowledge_failure_range,
	acknowledge_failure_flash,
	acknowledge_failure_not_ready,
	acknowledge_failure_unknown_packet
} acknowledge_result_e;

typedef struct
{
	uint8_t packet_id;
	uint16_t packet_crc;
	uint8_t acknowledge_result;
} acknowledge_packet_t;

typedef struct {
    uint32_t software_version;
    uint32_t device_id;
    uint32_t hardware_revision;
    uint32_t serial_number[3];
} device_information_packet_t;


typedef struct {
    uint8_t buffer[AN_DECODE_BUFFER_SIZE];
    uint16_t buffer_length;
    uint64_t packets_decoded;
    uint64_t bytes_decoded;
    uint64_t bytes_discarded;
    uint64_t lrc_errors;
    uint64_t crc_errors;
} an_decoder_t;

typedef struct {
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
    } system_status;
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
    } filter_status;
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double latitude;
    double longitude;
    double height;
    float velocity[3];
    float body_acceleration[3];
    float g_force;
    float orientation[3];
    float angular_velocity[3];
    float standard_deviation[3];
} system_state_packet_t;

typedef struct {
    float standard_deviation[3];
} velocity_standard_deviation_packet_t;

typedef struct {
    float accelerometers[3];
    float gyroscopes[3];
    float magnetometers[3];
    float imu_temperature;
    float pressure;
    float pressure_temperature;
} raw_sensors_packet_t;

typedef struct {
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double position[3];
    float velocity[3];
    float position_standard_deviation[3];
    float tilt; /* This field will only be valid if an external dual antenna GNSS system is connected */
    float heading; /* This field will only be valid if an external dual antenna GNSS system is connected */
    float tilt_standard_deviation; /* This field will only be valid if an external dual antenna GNSS system is connected */
    float heading_standard_deviation; /* This field will only be valid if an external dual antenna GNSS system is connected */
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
} raw_gnss_packet_t;

typedef struct {
    float hdop;
    float vdop;
    uint8_t gps_satellites;
    uint8_t glonass_satellites;
    uint8_t beidou_satellites;
    uint8_t galileo_satellites;
    uint8_t sbas_satellites;
} satellites_packet_t;

typedef struct {
    uint8_t packet_id;
    uint32_t period;
} packet_period_t;

typedef struct {
    uint8_t permanent;
    uint8_t clear_existing_packets;
    packet_period_t packet_periods[AN_MAXIMUM_PACKET_PERIODS];
} packet_periods_packet_t;




namespace SITL
{

class AdNav : public SerialDevice
{
public:

    AdNav();

    // update state
    void update(void);

private:
    an_decoder_t _an_decoder;

    uint32_t _packet_period_us = 20000; // Period to send packets.
    uint32_t _gnss_period_us = 200000; // Period to send packets.

    uint32_t _last_pkt_sent_us;
    uint32_t _last_gnss_sent_us;

    void receive_packets();
    void send_packet(an_packet_t* an_packet);
    void send_acknowledge(uint16_t crc, uint8_t id);
    void send_device_info_pkt();
    void send_state_pkt();
    void send_vel_sd_pkt();
    void send_raw_sensors_pkt();
    void send_raw_gnss_pkt();
    void send_sat_pkt();

    uint64_t start_us;

    an_packet_t* encode_acknowledge_packet(acknowledge_packet_t* acknowledge_packet);
    an_packet_t* encode_device_information_packet(device_information_packet_t* device_information_packet);
    an_packet_t* encode_system_state_packet(system_state_packet_t* system_state_packet);
    an_packet_t* encode_velocity_standard_deviation_packet(velocity_standard_deviation_packet_t* velocity_standard_deviation_packet);
    an_packet_t* encode_raw_sensors_packet(raw_sensors_packet_t* raw_gnss_packet);
    an_packet_t* encode_raw_gnss_packet(raw_gnss_packet_t* raw_gnss_packet);
    an_packet_t* encode_satellites_packet(satellites_packet_t* satellites_packet);
    int decode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet, an_packet_t* an_packet);


    uint8_t calculate_header_lrc(uint8_t* data);
    an_packet_t* an_packet_allocate(uint8_t length, uint8_t id);
    void an_packet_free(an_packet_t** an_packet);
    void an_decoder_initialise(an_decoder_t* an_decoder);
    an_packet_t* an_packet_decode(an_decoder_t* an_decoder);
    void an_packet_encode(an_packet_t* an_packet);

    
};

}

