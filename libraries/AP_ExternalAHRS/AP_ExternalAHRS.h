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
  suppport for serial connected AHRS systems
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <AP_NavEKF/AP_Nav_Common.h>

#ifndef HAL_EXTERNAL_AHRS_ENABLED
#define HAL_EXTERNAL_AHRS_ENABLED !HAL_MINIMIZE_FEATURES && !defined(HAL_BUILD_AP_PERIPH) && BOARD_FLASH_SIZE > 1024
#endif

#if HAL_EXTERNAL_AHRS_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS {

public:
    AP_ExternalAHRS();

    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

    enum class DevType : uint8_t {
        None   = 0,
        VecNav = 1,
    };

    static AP_ExternalAHRS *get_singleton(void) {
        return _singleton;
    }

    // fixed IMU rate for now
    float get_IMU_rate(void) const {
        return 50;
    }

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const;

    typedef struct {
        uint8_t instance;
        float pressure_pa;
        float temperature;
    } baro_data_message_t;

    typedef struct {
        Vector3f field;
    } mag_data_message_t;

    typedef struct {
        uint16_t gps_week;                   // GPS week, 0xFFFF if not available
        uint32_t ms_tow;
        uint8_t  fix_type;
        uint8_t  satellites_in_view;
        float horizontal_pos_accuracy;
        float vertical_pos_accuracy;
        float horizontal_vel_accuracy;
        float hdop;
        float vdop;
        int32_t  longitude;
        int32_t  latitude;
        int32_t  msl_altitude;       // cm
        float  ned_vel_north;
        float  ned_vel_east;
        float  ned_vel_down;
    } gps_data_message_t;

    typedef struct {
        Vector3f accel;
        Vector3f gyro;
        float temperature;
    } ins_data_message_t;

    // accessors for AP_AHRS
    bool healthy(void) const;
    bool initialised(void) const;
    bool get_quaternion(Quaternion &quat);
    bool get_origin(Location &loc);
    bool get_location(Location &loc);
    Vector2f get_groundspeed_vector();
    bool get_velocity_NED(Vector3f &vel);
    bool get_speed_down(float &speedD);
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;
    void get_filter_status(nav_filter_status &status) const;
    Vector3f get_gyro(void);
    Vector3f get_accel(void);
    void send_status_report(mavlink_channel_t chan) const;

    // check for new data
    void update() {
        check_uart();
    }

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    bool port_opened;
    uint32_t baudrate;

    void update_thread();
    bool check_uart();

    void process_packet1(const uint8_t *b);
    void process_packet2(const uint8_t *b);

    AP_Enum<DevType> devtype;

    static AP_ExternalAHRS *_singleton;

    uint8_t *pktbuf;
    uint16_t pktoffset;
    uint16_t bufsize;

    struct VN_packet1 *last_pkt1;
    struct VN_packet2 *last_pkt2;

    uint32_t last_pkt1_ms;
    uint32_t last_pkt2_ms;

    bool origin_set;
    Location origin;

    static HAL_Semaphore sem;
};

namespace AP {
    AP_ExternalAHRS &externalAHRS();
};

#endif  // HAL_EXTERNAL_AHRS_ENABLED

