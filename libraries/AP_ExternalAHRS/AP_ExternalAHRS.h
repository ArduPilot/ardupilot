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

class AP_ExternalAHRS_backend;

class AP_ExternalAHRS {

public:
    friend class AP_ExternalAHRS_backend;

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

    // expected IMU rate in Hz
    float get_IMU_rate(void) const {
        return rate.get();
    }

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const;

    struct state_t {
        HAL_Semaphore sem;

        Vector3f accel;
        Vector3f gyro;
        Quaternion quat;
        Location location;
        Vector3f velocity;
        Location origin;

        bool have_quaternion;
        bool have_origin;
        bool have_location;
        bool have_velocity;
    } state;

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

    // update backend
    void update();

    /*
      structures passed to other subsystems
     */
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
    
private:
    AP_ExternalAHRS_backend *backend;

    AP_Enum<DevType> devtype;
    AP_Int16         rate;

    static AP_ExternalAHRS *_singleton;
};

namespace AP {
    AP_ExternalAHRS &externalAHRS();
};

#endif  // HAL_EXTERNAL_AHRS_ENABLED

