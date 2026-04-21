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
  support for serial connected InertialLabs INS system
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_InertialLabs : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override {
        check_uart();
    }

    // Get model/type name
    const char* get_name() const override {
        return "ILabs";
    }

    enum class MessageType : uint8_t {
        GPS_INS_TIME_MS = 0x01,
        GPS_WEEK = 0x3C,
        ACCEL_DATA_HR = 0x23,
        GYRO_DATA_HR = 0x21,
        BARO_DATA = 0x25,
        MAG_DATA = 0x24,
        ORIENTATION_ANGLES = 0x07,
        VELOCITIES = 0x12,
        POSITION = 0x10,
        KF_VEL_COVARIANCE = 0x58,
        KF_POS_COVARIANCE = 0x57,
        UNIT_STATUS = 0x53,
        GNSS_EXTENDED_INFO = 0x4A,
        NUM_SATS = 0x3B,
        GNSS_POSITION = 0x30,
        GNSS_VEL_TRACK = 0x32,
        GNSS_POS_TIMESTAMP = 0x3E,
        GNSS_INFO_SHORT = 0x36,
        GNSS_NEW_DATA = 0x41,
        GNSS_JAM_STATUS = 0xC0,
        DIFFERENTIAL_PRESSURE = 0x28,
        TRUE_AIRSPEED = 0x86,
        WIND_SPEED = 0x8A,
        AIR_DATA_STATUS = 0x8D,
        SUPPLY_VOLTAGE = 0x50,
        TEMPERATURE = 0x52,
        UNIT_STATUS2 = 0x5A,
        GNSS_ANGLES = 0x33,
        GNSS_ANGLE_POS_TYPE = 0x3A,
        GNSS_HEADING_TIMESTAMP = 0x40,
        GNSS_DOP = 0x42,
        INS_SOLUTION_STATUS = 0x54,
    };

    /*
      packets consist of:
         ILabsHeader
         list of MessageType
         sequence of ILabsData
         checksum
     */
    struct PACKED ILabsHeader {
        uint16_t magic; // 0x55AA
        uint8_t msg_type; // always 1 for INS data
        uint8_t msg_id; // always 0x95
        uint16_t msg_len; // msg_len+2 is total packet length
    };

    struct PACKED vec3_16_t {
        int16_t x,y,z;
        Vector3f tofloat(void) {
            return Vector3f(x,y,z);
        }
    };
    struct PACKED vec3_32_t {
        int32_t x,y,z;
        Vector3f tofloat(void) {
            return Vector3f(x,y,z);
        }
    };
    struct PACKED vec3_u8_t {
        uint8_t x,y,z;
        Vector3f tofloat(void) {
            return Vector3f(x,y,z);
        }
    };
    struct PACKED vec3_u16_t {
        uint16_t x,y,z;
        Vector3f tofloat(void) {
            return Vector3f(x,y,z);
        }
    };

    struct gnss_extended_info_t {
        uint8_t fix_type;
        uint8_t spoofing_status;
    };

    struct gnss_info_short_t {
        uint8_t info1;
        uint8_t info2;
    };
    
    union PACKED ILabsData {
        uint32_t gnss_time_ms; // ms since start of GNSS week
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
        } orientation_angles; // 321 euler order?
        vec3_32_t velocity; // m/s * 100
        struct PACKED {
            int32_t lat; // deg*1e7
            int32_t lon; // deg*1e7
            int32_t alt; // m*100, AMSL
        } position;
        vec3_u8_t kf_vel_covariance; // mm/s
        vec3_u16_t kf_pos_covariance; // mm
        uint16_t unit_status; // set ILABS_UNIT_STATUS_*
        gnss_extended_info_t gnss_extended_info;
        uint8_t num_sats;
        struct PACKED {
            int32_t lat; // deg*1e7
            int32_t lon; // deg*1e7
            int32_t alt; // m*100
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
    };

    AP_ExternalAHRS::gps_data_message_t gps_data;
    AP_ExternalAHRS::mag_data_message_t mag_data;
    AP_ExternalAHRS::baro_data_message_t baro_data;
    AP_ExternalAHRS::ins_data_message_t ins_data;
    AP_ExternalAHRS::airspeed_data_message_t airspeed_data;

    uint16_t buffer_ofs;
    uint8_t buffer[256]; // max for normal message set is 167+8

protected:

    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;
    bool setup_complete;

    void update_thread();
    bool check_uart();
    bool check_header(const ILabsHeader *h) const;

    // re-sync on header bytes
    void re_sync(void);

    static const struct MessageLength {
        MessageType mtype;
        uint8_t length;
    } message_lengths[];

    struct {
        float baro_alt;
        Vector3f kf_vel_covariance;
        Vector3f kf_pos_covariance;
        uint16_t unit_status;
        uint16_t unit_status2;
        float differential_pressure;
        float true_airspeed;
        Vector3f wind_speed;
        uint16_t air_data_status;
        float supply_voltage;
        uint8_t ins_sol_status;
    } state2;

    struct {
        float lat;
        float lng;
        float alt;
        float hor_speed;
        float ver_speed;
        float track_over_ground;
        uint8_t new_data;
        uint32_t pos_timestamp;
        uint32_t heading_timestamp;
        uint8_t spoof_status;
        uint8_t jam_status;
        uint8_t angle_pos_type;
        gnss_info_short_t info_short;
        float heading;
        float pitch;
        float gdop;
        float pdop;
        float tdop;
    } gnss_data;

    uint16_t last_unit_status;
    uint16_t last_unit_status2;
    uint16_t last_air_data_status;
    uint8_t last_spoof_status;
    uint8_t last_jam_status;

    uint32_t last_critical_msg_ms;

    uint32_t last_att_ms;
    uint32_t last_vel_ms;
    uint32_t last_pos_ms;
    uint32_t last_gps_ms;
};

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

