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

#include "SIM_config.h"

#if AP_SIM_JSON_ENABLED

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

class JSON : public Aircraft {
public:
    JSON(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW JSON(frame_str);
    }

    /* Create and set in/out socket for JSON generic simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:

    struct servo_packet_16 {
        uint16_t magic = 18458; // constant magic value
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[16];
    };

    struct servo_packet_32 {
        uint16_t magic = 29569; // constant magic value
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[32];
    };

    // default connection_info_.ip_address
    const char *target_ip = "127.0.0.1";

    // default connection_info_.sitl_ip_port
    uint16_t control_port = 9002;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SocketAPM_native sock;
#else
    // sim-on-hardware
    SocketAPM sock;
#endif

    uint32_t frame_counter;
    double last_timestamp_s;

    void output_servos(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);

    uint64_t parse_sensors(const char *json);

    // buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[65000];
    uint32_t sensor_buffer_len;

    enum data_type {
        DATA_UINT64,
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR3F,
        DATA_VECTOR3D,
        QUATERNION,
        BOOLEAN,
    };

    struct {
        double timestamp_s;
        double latitude;
        double longitude;
        double altitude;
        struct {
            Vector3f gyro;
            Vector3f accel_body;
        } imu;
        Vector3d position;
        Vector3f attitude;
        Quaternion quaternion;
        Vector3f velocity;
        Vector3f velocity_wind;
        float rng[6];
        float rc[12];
        float bat_volt;
        float bat_amp;
        struct {
            float direction;
            float speed;
        } wind_vane_apparent;
        float airspeed;
        bool no_time_sync;
        bool no_lockstep;
    } state;

    // table to aid parsing of JSON sensor data
    struct keytable {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;
        bool required;
    } keytable[36] {
        { "", "timestamp", &state.timestamp_s, DATA_DOUBLE, true },
        { "", "latitude", &state.latitude, DATA_DOUBLE, false },
        { "", "longitude", &state.longitude, DATA_DOUBLE, false },
        { "", "altitude", &state.altitude, DATA_DOUBLE, false },
        { "imu", "gyro",    &state.imu.gyro, DATA_VECTOR3F, true },
        { "imu", "accel_body", &state.imu.accel_body, DATA_VECTOR3F, true },
        { "", "position", &state.position, DATA_VECTOR3D, false },
        { "", "attitude", &state.attitude, DATA_VECTOR3F, false },
        { "", "quaternion", &state.quaternion, QUATERNION, false },
        { "", "velocity", &state.velocity, DATA_VECTOR3F, true },
        { "", "rng_1", &state.rng[0], DATA_FLOAT, false },
        { "", "rng_2", &state.rng[1], DATA_FLOAT, false },
        { "", "rng_3", &state.rng[2], DATA_FLOAT, false },
        { "", "rng_4", &state.rng[3], DATA_FLOAT, false },
        { "", "rng_5", &state.rng[4], DATA_FLOAT, false },
        { "", "rng_6", &state.rng[5], DATA_FLOAT, false },
        {"","velocity_wind", &state.velocity_wind, DATA_VECTOR3F, false},
        {"windvane","direction", &state.wind_vane_apparent.direction, DATA_FLOAT, false},
        {"windvane","speed", &state.wind_vane_apparent.speed, DATA_FLOAT, false},
        {"", "airspeed", &state.airspeed, DATA_FLOAT, false},
        {"", "no_time_sync", &state.no_time_sync, BOOLEAN, false},
        {"", "no_lockstep", &state.no_lockstep, BOOLEAN, false},
        { "rc", "rc_1", &state.rc[0], DATA_FLOAT, false },
        { "rc", "rc_2", &state.rc[1], DATA_FLOAT, false },
        { "rc", "rc_3", &state.rc[2], DATA_FLOAT, false },
        { "rc", "rc_4", &state.rc[3], DATA_FLOAT, false },
        { "rc", "rc_5", &state.rc[4], DATA_FLOAT, false },
        { "rc", "rc_6", &state.rc[5], DATA_FLOAT, false },
        { "rc", "rc_7", &state.rc[6], DATA_FLOAT, false },
        { "rc", "rc_8", &state.rc[7], DATA_FLOAT, false },
        { "rc", "rc_9", &state.rc[8], DATA_FLOAT, false },
        { "rc", "rc_10", &state.rc[9], DATA_FLOAT, false },
        { "rc", "rc_11", &state.rc[10], DATA_FLOAT, false },
        { "rc", "rc_12", &state.rc[11], DATA_FLOAT, false },
        { "battery", "voltage", &state.bat_volt, DATA_FLOAT, false },
        { "battery", "current", &state.bat_amp, DATA_FLOAT, false },
    };

    // Enum coresponding to the ordering of keys in the keytable.
    enum DataKey : uint64_t {
        TIMESTAMP   = 0x0000000000000001ULL, // 1ULL << 0
        LATITUDE    = 0x0000000000000002ULL, // 1ULL << 1
        LONGITUDE   = 0x0000000000000004ULL, // 1ULL << 2
        ALTITUDE    = 0x0000000000000008ULL, // 1ULL << 3
        GYRO        = 0x0000000000000010ULL, // 1ULL << 4
        ACCEL_BODY  = 0x0000000000000020ULL, // 1ULL << 5
        POSITION    = 0x0000000000000040ULL, // 1ULL << 6
        EULER_ATT   = 0x0000000000000080ULL, // 1ULL << 7
        QUAT_ATT    = 0x0000000000000100ULL, // 1ULL << 8
        VELOCITY    = 0x0000000000000200ULL, // 1ULL << 9
        RNG_1       = 0x0000000000000400ULL, // 1ULL << 10
        RNG_2       = 0x0000000000000800ULL, // 1ULL << 11
        RNG_3       = 0x0000000000001000ULL, // 1ULL << 12
        RNG_4       = 0x0000000000002000ULL, // 1ULL << 13
        RNG_5       = 0x0000000000004000ULL, // 1ULL << 14
        RNG_6       = 0x0000000000008000ULL, // 1ULL << 15
        WIND_VEL    = 0x0000000000010000ULL, // 1ULL << 16
        WIND_DIR    = 0x0000000000020000ULL, // 1ULL << 17
        WIND_SPD    = 0x0000000000040000ULL, // 1ULL << 18
        AIRSPEED    = 0x0000000000080000ULL, // 1ULL << 19
        TIME_SYNC   = 0x0000000000100000ULL, // 1ULL << 20
        LOCKSTEP    = 0x0000000000200000ULL, // 1ULL << 21
        RC_1        = 0x0000000000400000ULL, // 1ULL << 22
        RC_2        = 0x0000000000800000ULL, // 1ULL << 23
        RC_3        = 0x0000000001000000ULL, // 1ULL << 24
        RC_4        = 0x0000000002000000ULL, // 1ULL << 25
        RC_5        = 0x0000000004000000ULL, // 1ULL << 26
        RC_6        = 0x0000000008000000ULL, // 1ULL << 27
        RC_7        = 0x0000000010000000ULL, // 1ULL << 28
        RC_8        = 0x0000000020000000ULL, // 1ULL << 29
        RC_9        = 0x0000000040000000ULL, // 1ULL << 30
        RC_10       = 0x0000000080000000ULL, // 1ULL << 31
        RC_11       = 0x0000000100000000ULL, // 1ULL << 32
        RC_12       = 0x0000000200000000ULL, // 1ULL << 33
        BAT_VOLT    = 0x0000000400000000ULL, // 1ULL << 34
        BAT_AMP     = 0x0000000800000000ULL, // 1ULL << 35
    };
    uint64_t last_received_bitmask;

    uint32_t last_debug_ms;

    bool last_no_lockstep;
};

}

#endif  // AP_SIM_JSON_ENABLED