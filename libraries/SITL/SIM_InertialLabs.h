//usage:
//PARAMS:
// param set AHRS_EKF_TYPE 11
// param set EAHRS_TYPE 5
// param set SERIAL4_PROTOCOL 36
// param set SERIAL4_BAUD 460800
// sim_vehicle.py -v ArduPlane -D --console --map -A --serial4=sim:ILabs
#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL
{

class InertialLabs : public SerialDevice
{
public:

    InertialLabs();

    // update state
    void update(void);

private:
    void send_packet(void);

    struct PACKED vec3_16_t {
        int16_t x,y,z;
    };
    struct PACKED vec3_32_t {
        int32_t x,y,z;
    };
    struct PACKED vec3_u8_t {
        uint8_t x,y,z;
    };
    struct PACKED vec3_u16_t {
        uint16_t x,y,z;
    };

    struct gnss_extended_info_t {
        uint8_t fix_type;
        uint8_t spoofing_status;
    };

    struct gnss_info_short_t {
        uint8_t info1;
        uint8_t info2;
    };

    struct PACKED ILabsPacket {
        uint16_t magic = 0x55AA;
        uint8_t msg_type = 0x01;
        uint8_t msg_id = 0x95;
        uint16_t msg_len; // total packet length-2

        // send Table4, 32 messages
        uint8_t num_messages = 32;
        uint8_t messages[32] = {
            0x01, 0x3C, 0x23, 0x21, 0x25, 0x24, 0x07, 0x12, 0x10, 0x58, 0x57, 0x53, 0x4a,
            0x3b, 0x30, 0x32, 0x3e, 0x36, 0x41, 0xc0, 0x28, 0x86, 0x8a, 0x8d, 0x50, 0x52,
            0x5a, 0x33, 0x3a, 0x40, 0x42, 0x54
        };
        uint32_t gnss_ins_time_ms; // ms since start of GPS week for IMU data
        uint16_t gnss_week;
        vec3_32_t accel_data_hr; // g * 1e6
        vec3_32_t gyro_data_hr; // deg/s * 1e5
        struct PACKED {
            uint16_t pressure_pa2; // Pascals/2
            int32_t baro_alt; // meters*100
        } baro_data;
        vec3_16_t mag_data; // nT/10
        struct PACKED {
            uint16_t yaw; // deg*100
            int16_t pitch; // deg*100
            int16_t roll; // deg*100
        } orientation_angles; // 321 euler order
        vec3_32_t velocity; // m/s * 100
        struct PACKED {
            int32_t lat; // deg*1e7
            int32_t lon; // deg*1e7
            int32_t alt; // m*100, AMSL
        } position;
        vec3_u8_t kf_vel_covariance; // mm/s
        vec3_u16_t kf_pos_covariance; // mm
        uint16_t unit_status;
        gnss_extended_info_t gnss_extended_info;
        uint8_t num_sats;
        struct PACKED {
            int32_t lat; // deg*1e7
            int32_t lon; // deg*1e7
            int32_t alt; // m*100, AMSL
        } gnss_position;
        struct PACKED {
            int32_t hor_speed; // m/s*100
            uint16_t track_over_ground; // deg*100
            int32_t ver_speed; // m/s*100
        } gnss_vel_track;
        uint32_t gnss_pos_timestamp; // ms
        gnss_info_short_t gnss_info_short;
        uint8_t gnss_new_data;
        uint8_t gnss_jam_status;
        int32_t differential_pressure; // mbar*1e4
        int16_t true_airspeed; // m/s*100
        vec3_16_t wind_speed; // m/s*100
        uint16_t air_data_status;
        uint16_t supply_voltage; // V*100
        int16_t temperature; // degC*10
        uint16_t unit_status2;
        struct PACKED {
            uint16_t heading; // deg*100
            int16_t pitch; // deg*100
        } gnss_angles;
        uint8_t gnss_angle_pos_type;
        uint32_t gnss_heading_timestamp; // ms
        struct PACKED {
            uint16_t gdop;
            uint16_t pdop;
            uint16_t hdop;
            uint16_t vdop;
            uint16_t tdop;
        } gnss_dop; // 10e3
        uint8_t ins_sol_status;
        uint16_t crc;
    } pkt;

    uint32_t last_pkt_us;
    const uint16_t pkt_rate_hz = 200;
    const uint16_t gnss_rate_hz = 10;
    const uint16_t gnss_frequency = pkt_rate_hz / gnss_rate_hz;
    uint32_t packets_sent;
};

}

